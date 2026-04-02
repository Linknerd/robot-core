#!/usr/bin/env python3
"""
serial_bridge.py — ROS2 Humble
Bridges the Arduino (robot_controller) to ROS2 topics.

Serial protocol (Arduino → Pi):
  O,linear_m_s,angular_rad_s  Odometry velocities (computed on Arduino, sent every T ms)
  I,ax,ay,az,gx,gy,gz        IMU (accel in g, gyro in deg/s)
  C,temp,humidity,co2         SCD30 environmental sensor
  S,d0,d1,d2                  Sharp IR distances (cm)
  E,message                   Arduino error string

Serial protocol (Pi → Arduino):
  V,vd,wd\\n                   Desired linear [m/s] and angular [rad/s] velocity
  S\\n                         Stop

Odometry architecture
---------------------
Integration is EVENT-DRIVEN: pose is updated the instant each "O" line is
parsed, using the ROS clock Δt between consecutive O arrivals.  This is the
only correct approach when the serial reader and the ROS executor run on
different threads at different rates.

A separate 20 Hz timer ONLY publishes the already-integrated pose — it does
NOT reintegrate.  This keeps the TF/odom topics at a predictable rate without
coupling the integration accuracy to the publish rate.
"""

import math
import threading

import rclpy
from rclpy.node import Node

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


# ── Calibration knob ─────────────────────────────────────────────────────────
#
# If the robot physically travels 1 m but /odom reports X m, set:
#   VEL_SCALE = 1.0 / X
#
# Example: robot moved 1 m, /odom reported 0.1 m  →  VEL_SCALE = 10.0
# Example: robot moved 1 m, /odom reported 1.5 m  →  VEL_SCALE = 0.667
#
# Set to 1.0 once RHO and TPR on the Arduino are correctly calibrated.
VEL_SCALE = 1.0


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')
        self.get_logger().info('Serial Bridge Node started')

        # ── Serial ───────────────────────────────────────────────────────────
        self.port     = '/dev/ttyACM0'
        self.baudrate = 115200
        try:
            # timeout=0.1 s — short enough that a missing newline doesn't
            # stall the reader thread for a full second (the old bug).
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # ── Odometry state (protected by _odom_lock) ─────────────────────────
        #
        # ALL reads and writes to x, y, theta, last_odom_time, latest_v,
        # latest_w MUST be done while holding _odom_lock.  The serial reader
        # thread integrates here; the publish timer reads here.
        self._odom_lock   = threading.Lock()
        self.x            = 0.0
        self.y            = 0.0
        self.theta        = 0.0
        self.last_odom_time: float | None = None
        self.latest_v     = 0.0   # last received linear  velocity [m/s]
        self.latest_w     = 0.0   # last received angular velocity [rad/s]

        # Diagnostic counters (only read/written under _odom_lock too)
        self._odom_msgs_received = 0
        self._last_diag_time: float | None = None

        # ── TF broadcaster ───────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Publishers ───────────────────────────────────────────────────────
        self.odom_pub  = self.create_publisher(Odometry,          'odom',                    10)
        self.imu_pub   = self.create_publisher(Imu,               'imu',                     10)
        self.scd30_pub = self.create_publisher(Float32MultiArray, 'scd30/data',              10)
        self.sharp_pub = self.create_publisher(Float32MultiArray, 'sharp_sensors/distances', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # ── Publish timer (20 Hz) ─────────────────────────────────────────────
        # This timer ONLY publishes the current pose.  It does NOT integrate.
        # Integration happens event-driven in _integrate_odom() below.
        self.create_timer(0.05, self.publish_odom_callback)

        # ── Serial reader thread ──────────────────────────────────────────────
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    # ── cmd_vel → V,vd,wd ────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        """
        Forward linear and angular velocity directly to the Arduino.
        The Arduino PI controller handles converting these to motor PWM.
        """
        vd = msg.linear.x
        wd = msg.angular.z
        command = f'V,{vd:.4f},{wd:.4f}\n'
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial reader ─────────────────────────────────────────────────────────

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                # readline() now has a 0.1 s timeout — if no '\n' arrives it
                # returns an empty/partial bytes object rather than blocking
                # for a full second as in the old code.
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
                if line:
                    self.parse_line(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
            except Exception as e:
                self.get_logger().debug(f'Read loop error: {e}')

    # ── Line parser ───────────────────────────────────────────────────────────

    def parse_line(self, line: str):
        parts  = line.split(',')
        prefix = parts[0]

        try:
            # ── Odometry ─────────────────────────────────────────────────────
            # This is the critical path.  We integrate immediately on receipt
            # of each O message using the ROS clock Δt between consecutive O
            # arrivals.  This is the ONLY correct approach; a decoupled timer
            # integrating stale latest_v values causes scale errors.
            if prefix == 'O' and len(parts) == 3:
                v_raw = float(parts[1]) * VEL_SCALE
                w_raw = float(parts[2]) * VEL_SCALE
                self._integrate_odom(v_raw, w_raw)
                return

            # ── IMU ──────────────────────────────────────────────────────────
            if prefix == 'I' and len(parts) == 7:
                msg = Imu()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.linear_acceleration.x = float(parts[1]) * 9.81
                msg.linear_acceleration.y = float(parts[2]) * 9.81
                msg.linear_acceleration.z = float(parts[3]) * 9.81
                msg.angular_velocity.x    = float(parts[4]) * 0.017453
                msg.angular_velocity.y    = float(parts[5]) * 0.017453
                msg.angular_velocity.z    = float(parts[6]) * 0.017453
                msg.orientation_covariance[0]         = -1.0
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01
                msg.angular_velocity_covariance[0]    = 0.005
                msg.angular_velocity_covariance[4]    = 0.005
                msg.angular_velocity_covariance[8]    = 0.005
                self.imu_pub.publish(msg)

            # ── SCD30 ─────────────────────────────────────────────────────────
            elif prefix == 'C' and len(parts) == 4:
                msg      = Float32MultiArray()
                msg.data = [float(parts[1]), float(parts[2]), float(parts[3])]
                self.scd30_pub.publish(msg)

            # ── Sharp IR ─────────────────────────────────────────────────────
            elif prefix == 'S' and len(parts) >= 2:
                msg      = Float32MultiArray()
                msg.data = [float(p) for p in parts[1:]]
                self.sharp_pub.publish(msg)

            # ── Arduino error ─────────────────────────────────────────────────
            elif prefix == 'E':
                self.get_logger().warn(f'Arduino: {line[2:]}')

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error on line "{line}": {e}')

    # ── Event-driven odometry integration ─────────────────────────────────────

    def _integrate_odom(self, v: float, w: float):
        """
        Called from the serial reader thread every time an O message arrives.

        Uses the ROS clock Δt between consecutive O arrivals as the integration
        step.  This is accurate because both the Arduino T period and the ROS
        clock are stable; any small jitter is naturally absorbed by the real Δt.

        Midpoint (2nd-order Runge-Kutta) integration is used:
            theta_mid = theta + 0.5 * w * dt
            x        += v * cos(theta_mid) * dt
            y        += v * sin(theta_mid) * dt
            theta    += w * dt

        This halves the heading discretisation error compared to forward Euler
        with no additional computational cost.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        with self._odom_lock:
            self.latest_v = v
            self.latest_w = w
            self._odom_msgs_received += 1

            # ── First message — seed the clock, nothing to integrate yet ──────
            if self.last_odom_time is None:
                self.last_odom_time  = now_sec
                self._last_diag_time = now_sec
                return

            dt = now_sec - self.last_odom_time
            self.last_odom_time = now_sec

            # Sanity-check dt: ignore if clock jumped or first tick is huge
            if dt <= 0.0 or dt > 1.0:
                self.get_logger().warn(
                    f'Odom dt out of range ({dt:.3f} s), skipping integration.'
                )
                return

            # ── Midpoint integration ──────────────────────────────────────────
            theta_mid   = self.theta + 0.5 * w * dt
            self.x     += v * math.cos(theta_mid) * dt
            self.y     += v * math.sin(theta_mid) * dt
            self.theta += w * dt

            # ── Diagnostic: log v, w, and pose at 5 Hz ────────────────────────
            if now_sec - self._last_diag_time >= 5.0:
                self.get_logger().info(
                    f'[odom diag] msgs={self._odom_msgs_received}  '
                    f'v={v:.4f} m/s  w={w:.4f} rad/s  '
                    f'x={self.x:.3f}  y={self.y:.3f}  '
                    f'θ={math.degrees(self.theta):.1f}°  '
                    f'dt={dt*1000:.1f} ms'
                )
                self._last_diag_time = now_sec

    # ── Odometry publisher (20 Hz, read-only) ─────────────────────────────────

    def publish_odom_callback(self):
        """
        Publishes the current integrated pose as /odom and the odom→base_footprint
        TF.  Does NOT integrate — that happens in _integrate_odom().
        """
        with self._odom_lock:
            x     = self.x
            y     = self.y
            theta = self.theta
            v     = self.latest_v
            w     = self.latest_w

        now   = self.get_clock().now()
        stamp = now.to_msg()

        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # ── TF: odom → base_footprint ─────────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_footprint'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = 0.0
        tf.transform.rotation.y    = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)

        # ── /odom topic ───────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = w

        # Pose covariance (position uncertainty grows with dead-reckoning error)
        pc = [0.0] * 36
        pc[0]  = 0.1   # x  [m²]
        pc[7]  = 0.1   # y  [m²]
        pc[35] = 0.2   # yaw [rad²]
        odom.pose.covariance = pc

        # Twist covariance (uncertainty in the reported velocity)
        tc = [0.0] * 36
        tc[0]  = 0.05   # linear  [m/s²]
        tc[35] = 0.1    # angular [rad/s²]
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

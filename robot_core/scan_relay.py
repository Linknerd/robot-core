#!/usr/bin/env python3
"""
scan_relay.py
Subscribes to /scan, replaces the header stamp with the current ROS time,
and republishes to /scan_corrected.

This works around a sllidar_ros2 driver bug where scan timestamps are
~10x larger than the actual ROS time, causing SLAM toolbox's TF message
filter to wait forever for a transform that never arrives.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan

# Stamp scans this far in the past so the TF buffer is guaranteed to
# have transforms covering the scan timestamp. Must be larger than the
# TF publish interval (20ms) plus any system latency.
STAMP_OFFSET = Duration(seconds=0.15)


class ScanRelay(Node):

    def __init__(self):
        super().__init__('scan_relay')
        self.pub = self.create_publisher(LaserScan, 'scan_corrected', 10)
        self.create_subscription(LaserScan, 'scan', self.callback, 10)
        self.get_logger().info('Scan relay started: /scan → /scan_corrected')

    def callback(self, msg: LaserScan):
        msg.header.stamp = (self.get_clock().now() - STAMP_OFFSET).to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

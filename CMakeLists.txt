cmake_minimum_required(VERSION 3.8)
project(robot_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  robot_core/serial_bridge.py
  robot_core/robot_logic.py
  robot_core/scan_relay.py
  robot_core/debugging/serial_bridge_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(ros_drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


install(PROGRAMS
  ros_drone_control/battery_gps_node.py
  ros_drone_control/mission_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


cmake_minimum_required(VERSION 3.5)
project(drone_control)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


install(PROGRAMS
  drone_control/battery_gps_node.py
  drone_control/mission_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


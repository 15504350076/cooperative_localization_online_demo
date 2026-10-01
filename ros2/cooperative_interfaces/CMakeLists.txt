# 功能：生成与上交盒端约定一致的 GNSS、身份和 UWB ROS 2 消息类型。
cmake_minimum_required(VERSION 3.8)
project(cooperative_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssPosition.msg"
  "msg/Identity.msg"
  "msg/UwbRange.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

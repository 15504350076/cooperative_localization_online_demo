cmake_minimum_required(VERSION 3.8)
project(cooperative_localization_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeState.msg"
  "msg/VehiclePose2D.msg"
  "msg/CooperativePose2DArray.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

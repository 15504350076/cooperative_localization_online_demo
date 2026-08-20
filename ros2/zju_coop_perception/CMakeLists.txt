cmake_minimum_required(VERSION 3.16)
project(zju_coop_perception VERSION 0.3.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common registration)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(zju_lidar_odometry_node src/lidar_odometry_node.cpp)
target_compile_features(zju_lidar_odometry_node PRIVATE cxx_std_17)
target_compile_options(zju_lidar_odometry_node PRIVATE
  -Wall -Wextra -Wpedantic)
target_include_directories(zju_lidar_odometry_node SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS})
target_link_libraries(zju_lidar_odometry_node ${PCL_LIBRARIES})
ament_target_dependencies(zju_lidar_odometry_node
  geometry_msgs nav_msgs pcl_conversions rclcpp sensor_msgs tf2_ros)

install(TARGETS zju_lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/colored_landmark_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zju_colored_landmark_localizer)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_test(NAME test_colored_landmark_localizer
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_colored_landmark_localizer.py")
  add_launch_test(test/test_lidar_odometry.launch.py TIMEOUT 35)
endif()

ament_package()

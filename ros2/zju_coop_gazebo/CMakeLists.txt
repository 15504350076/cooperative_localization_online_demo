cmake_minimum_required(VERSION 3.8)
project(zju_coop_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS scripts/gazebo_scenario.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zju_gazebo_scenario
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_gazebo_assets
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gazebo_assets.py")

  # Keep this optional so the SDK and non-Gazebo ROS packages remain
  # buildable on deployment hosts. Installing ros-humble-ros-gz enables it.
  find_package(ros_gz_sim QUIET)
  find_package(ros_gz_bridge QUIET)
  if(ros_gz_sim_FOUND AND ros_gz_bridge_FOUND)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_three_vehicle_gazebo.launch.py TIMEOUT 60)
    add_launch_test(test/test_perception_sensors_gazebo.launch.py TIMEOUT 75)
    # Probe the ament index without loading RTAB-Map's large transitive CMake
    # dependency graph.  The SLAM backend is runtime-only and opt-in.
    ament_index_has_resource(HAS_RTABMAP_SLAM packages rtabmap_slam)
    if(HAS_RTABMAP_SLAM)
      add_launch_test(test/test_lidar_slam_gazebo.launch.py TIMEOUT 105)
    endif()
  endif()
endif()

ament_package()

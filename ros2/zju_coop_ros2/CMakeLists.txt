cmake_minimum_required(VERSION 3.16)
project(zju_coop_ros2 VERSION 0.3.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cooperative_localization_msgs REQUIRED)
find_package(zju_coop_test_msgs REQUIRED)

set(ZJU_COOP_SDK_ROOT "" CACHE PATH
    "Root of a native zju_coop SDK installation containing include/ and lib/")
if(NOT ZJU_COOP_SDK_ROOT)
  message(FATAL_ERROR "ZJU_COOP_SDK_ROOT must name the staged zju_coop SDK")
endif()

find_path(ZJU_COOP_INCLUDE_DIR zju_coop/c_api.h
  HINTS "${ZJU_COOP_SDK_ROOT}/include"
  NO_DEFAULT_PATH)
find_library(ZJU_COOP_LIBRARY NAMES zju_coop
  HINTS "${ZJU_COOP_SDK_ROOT}/lib" "${ZJU_COOP_SDK_ROOT}/lib64"
  NO_DEFAULT_PATH)
if(NOT ZJU_COOP_INCLUDE_DIR OR NOT ZJU_COOP_LIBRARY)
  message(FATAL_ERROR
    "No staged zju_coop SDK found below ZJU_COOP_SDK_ROOT=${ZJU_COOP_SDK_ROOT}")
endif()

function(zju_coop_add_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE "${ZJU_COOP_INCLUDE_DIR}")
  # ament_target_dependencies uses CMake's plain target-link signature.
  target_link_libraries(${target} "${ZJU_COOP_LIBRARY}")
  ament_target_dependencies(${target}
    rclcpp sensor_msgs cooperative_localization_msgs zju_coop_test_msgs)
  set_target_properties(${target} PROPERTIES INSTALL_RPATH "$ORIGIN/..")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

zju_coop_add_node(zju_local_inertial_node src/local_inertial_node.cpp)
zju_coop_add_node(zju_cooperative_fusion_node src/cooperative_fusion_node.cpp)

install(PROGRAMS scripts/three_vehicle_simulator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME zju_three_vehicle_simulator)

# Keep the colcon install tree self-contained. RK3588 builds must point
# ZJU_COOP_SDK_ROOT at an aarch64 SDK produced on/for that target.
install(FILES "${ZJU_COOP_LIBRARY}" DESTINATION lib)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_pipeline.launch.py")
    add_launch_test(test/test_pipeline.launch.py TIMEOUT 40)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_required_initialization.launch.py")
    add_launch_test(test/test_required_initialization.launch.py TIMEOUT 20)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_dynamic_pipeline.launch.py")
    add_launch_test(test/test_dynamic_pipeline.launch.py TIMEOUT 60)
  endif()
endif()

ament_package()

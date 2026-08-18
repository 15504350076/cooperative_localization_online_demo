cmake_minimum_required(VERSION 3.8)
project(zju_coop_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_launch.launch.py")
    add_launch_test(test/test_simulation_launch.launch.py TIMEOUT 60)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_gnss_fallback_modes.launch.py")
    add_launch_test(test/test_gnss_fallback_modes.launch.py TIMEOUT 30)
  endif()
endif()

ament_package()

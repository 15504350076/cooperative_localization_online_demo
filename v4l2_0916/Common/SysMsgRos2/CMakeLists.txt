find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(SysMsg STATIC Src/SysMsg.cpp)
target_include_directories(SysMsg PUBLIC Include)
ament_target_dependencies(SysMsg rclcpp std_msgs)

add_executable(SysMsgTest Test/SysMsgTest.cpp)
target_link_libraries(SysMsgTest SysMsg)

add_executable(SysMsgThreadsTest Test/SysMsgThreadsTest.cpp)
target_link_libraries(SysMsgThreadsTest SysMsg)

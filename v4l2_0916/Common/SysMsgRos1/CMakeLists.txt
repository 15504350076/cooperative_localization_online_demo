file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.cpp Include/*.h)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(SysMsg STATIC ${srcs})
target_include_directories(SysMsg PUBLIC Include)

file(GLOB_RECURSE test_msg_srcs CONFIGURE_DEPENDS Test/SysMsgTest.cpp)
add_executable(SysMsgTest ${test_msg_srcs})
target_link_libraries(SysMsgTest SysMsg ${catkin_LIBRARIES})

file(GLOB_RECURSE test_msg_threads_srcs CONFIGURE_DEPENDS Test/SysMsgThreadsTest.cpp)
add_executable(SysMsgThreadsTest ${test_msg_threads_srcs})
target_link_libraries(SysMsgThreadsTest SysMsg ${catkin_LIBRARIES})

file(GLOB_RECURSE test_mcs_node_srcs CONFIGURE_DEPENDS Test/McsNodeTest.cpp)
add_executable(McsNodeTest ${test_mcs_node_srcs})
target_link_libraries(McsNodeTest SysMsg ${catkin_LIBRARIES} SysApi NlohmannJson)

file(GLOB_RECURSE test_fe_node_srcs CONFIGURE_DEPENDS Test/FeNodeTest.cpp)
add_executable(FeNodeTest ${test_fe_node_srcs})
target_link_libraries(FeNodeTest SysMsg ${catkin_LIBRARIES} SysApi NlohmannJson)

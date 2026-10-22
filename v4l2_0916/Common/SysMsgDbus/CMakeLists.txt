find_package(sdbus-c++ REQUIRED)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.cpp Include/SysMsg/*.h)
add_library(SysMsg STATIC ${srcs})
target_include_directories(SysMsg PUBLIC Include)
# 设置库的链接属性，使其包含对sdbus-c++的依赖
target_link_libraries(SysMsg PUBLIC SDBusCpp::sdbus-c++)

add_executable(SysMsgTest Test/SysMsgTest.cpp)
target_link_libraries(SysMsgTest SysMsg SDBusCpp::sdbus-c++)

add_executable(SysMsgThreadsTest Test/SysMsgThreadsTest.cpp)
target_link_libraries(SysMsgThreadsTest SysMsg SDBusCpp::sdbus-c++)

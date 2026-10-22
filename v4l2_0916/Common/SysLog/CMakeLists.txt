project(logadapter VERSION 0.1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Library
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.cpp Include/*.h)
add_library(SysLog STATIC ${srcs})
target_include_directories(SysLog PUBLIC Include)
target_link_libraries(SysLog PRIVATE log4cplus)
set_target_properties(SysLog PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Tests
file(GLOB_RECURSE test_srcs CONFIGURE_DEPENDS Test/*.cpp Test/*.h)
add_executable(SysLogTest ${test_srcs})
target_link_libraries(SysLogTest PRIVATE SysLog Threads::Threads)

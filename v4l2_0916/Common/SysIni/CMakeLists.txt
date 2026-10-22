project(SysIni VERSION 0.1)

# Library
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.c Include/*.h)
add_library(SysIni STATIC ${srcs})
target_include_directories(SysIni PUBLIC Include)

# Tests
file(GLOB_RECURSE test_srcs CONFIGURE_DEPENDS Test/*.cpp Test/*.h)
add_executable(SysIniTest ${test_srcs})
target_link_libraries(SysIniTest PRIVATE SysIni)

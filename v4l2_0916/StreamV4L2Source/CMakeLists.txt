project(StreamV4L2Source VERSION 0.1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Library
file(GLOB_RECURSE srcs CONFIGURE_DEPENDS Src/*.cpp Include/*.h)
add_library(StreamV4L2Source STATIC ${srcs})

target_include_directories(StreamV4L2Source PUBLIC Include)
target_link_libraries(StreamV4L2Source PRIVATE "-Wl,--whole-archive" avcodec avdevice avfilter avformat avutil postproc swresample swscale "-Wl,--no-whole-archive")
set_target_properties(StreamV4L2Source PROPERTIES POSITION_INDEPENDENT_CODE ON)
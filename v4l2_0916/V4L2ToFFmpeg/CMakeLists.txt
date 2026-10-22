cmake_minimum_required(VERSION 3.10)
project(V4L2ToFFmpeg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Find V4L2
find_path(V4L2_INCLUDE_DIR
    NAMES linux/videodev2.h
    PATHS /usr/include
)
find_library(V4L2_LIBRARIES
    NAMES v4l2
)

if(NOT V4L2_LIBRARIES)
    set(V4L2_LIBRARIES "")
endif()

# Source files for basic conversion
set(BASIC_SOURCES
    src/v4l2_capture.cpp
    src/main.cpp
)

set(HEADERS
    include/v4l2_capture.h
)

add_executable(v4l2_to_ffmpeg ${BASIC_SOURCES} ${HEADERS})

# Include directories
target_include_directories(v4l2_to_ffmpeg PRIVATE
    include
    ${V4L2_INCLUDE_DIR}
)


# Link libraries
target_link_libraries(v4l2_to_ffmpeg
    avformat avfilter avcodec  avdevice avutil postproc swscale swresample
    # 添加额外的库来解决链接错误
    avutil
    # Rockchip特定库
    drm rockchip_mpp rga
    # 系统库
    ${V4L2_LIBRARIES}
    pthread
    m
    z
)

# Compiler options
target_compile_options(v4l2_to_ffmpeg PRIVATE
    -Wall
    -Wextra
    -O3
)

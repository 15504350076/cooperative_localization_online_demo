cmake_minimum_required(VERSION 3.15)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
#set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake;${CMAKE_MODULE_PATH}")

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/build/CPackTmp" CACHE PATH "Installation Directory" FORCE)

# Set common output directories
set(COMMON_OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/Install/Bin)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${COMMON_OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${COMMON_OUTPUT_DIR})
#No need to put static libraries to final output folder
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${COMMON_OUTPUT_DIR})

project(TdmmAi LANGUAGES C CXX)

add_definitions(-DNO_STD_LIB)

# All third party libraries are included in ThirdParty.cmake
include(${CMAKE_CURRENT_LIST_DIR}/ThirdParty/ThirdParty.cmake)

# 添加自定义目标来强制执行生成 AiboxVersion.h
add_custom_target(generate_version_files ALL
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_LIST_DIR}/VersionGeneration.cmake
    COMMENT "Generating AiboxVersion.h with latest version information"
    VERBATIM
)

# for subdirectory to include firmware version header file
include_directories(${CMAKE_SOURCE_DIR}/Common/Include)

# 添加选项来选择使用哪个消息系统
option(USE_SYSMSG_DBUS "Use DBUS Message Bus" OFF)
option(USE_SYSMSG_ROS1 "Use ROS1 Message Bus" OFF)
option(USE_SYSMSG_ROS2 "Use ROS2 Message Bus" OFF)

add_subdirectory(Common/SysApi)
add_subdirectory(Common/SysLog)
add_subdirectory(Common/SysIni)

if(USE_SYSMSG_DBUS)
    add_subdirectory(Common/SysMsgDbus)
elseif(USE_SYSMSG_ROS1)
    add_subdirectory(Common/SysMsgRos1)
elseif(USE_SYSMSG_ROS2)
    add_subdirectory(Common/SysMsgRos2)
endif()

add_subdirectory(V4L2ToFFmpeg)

set(CPACK_PROJECT_CONFIG_FILE "${CMAKE_CURRENT_LIST_DIR}/AiboxCPackConfig.cmake")
set(CPACK_GENERATOR "TGZ")
include(CPack)
# 获取当前 Git commit ID 的短字符做为固件版本号的一部分
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_COMMIT_ID
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 获取当前日期
execute_process(
    COMMAND date +%Y-%m-%d
    OUTPUT_VARIABLE BUILD_DATE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 获取当前时间
execute_process(
    COMMAND date +%H:%M:%S
    OUTPUT_VARIABLE BUILD_TIME
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# 从 version.txt 文件中读取 MAIN_VERSION
set(VERSION_FILE "${CMAKE_CURRENT_LIST_DIR}/Common/Include/version.txt")
if (EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} MAIN_VERSION)
    string(STRIP ${MAIN_VERSION} MAIN_VERSION)  # 去除可能的空白字符
else()
    message(FATAL_ERROR "version.txt file not found at ${VERSION_FILE}")
endif()

# 设置完整的固件版本号
set(FIRMWARE_VERSION "${MAIN_VERSION}-${GIT_COMMIT_ID}-${BUILD_DATE}")
message(STATUS "Build Firmware version: ${FIRMWARE_VERSION} at ${BUILD_TIME}")
# 生成 AiboxVersion.h 文件
file(WRITE "${CMAKE_CURRENT_LIST_DIR}/Common/Include/AiboxVersion.h" "#pragma once\n")
file(APPEND "${CMAKE_CURRENT_LIST_DIR}/Common/Include/AiboxVersion.h" "#define FIRMWARE_VERSION \"${FIRMWARE_VERSION}\"" "\n")

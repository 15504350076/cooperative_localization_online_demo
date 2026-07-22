# 模块职责：定义算法核心、稳定C ABI、在线/回放/自检程序、测试和安装导出的唯一构建入口。
# 所有目标统一使用C++17；Windows生成DLL/EXE，Ubuntu与RK3588生成.so和ELF可执行程序。
cmake_minimum_required(VERSION 3.16)

project(zju_coop VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
include(GNUInstallDirs)

# 算法核心不包含ROS 2和平台网络类型，便于独立静态链接到C ABI动态库。
set(ZJU_COOP_CORE_SOURCES
    src/config/ini_config.cpp
    src/core/cooperative_inertial_ekf.cpp
    src/core/dense_matrix.cpp
    src/core/degradation_monitor.cpp
    src/core/engine.cpp
    src/core/inertial_eskf15.cpp
    src/core/quaternion.cpp
    src/core/range_ekf.cpp
    src/core/rigidity.cpp
    src/protocol/crc32.cpp
    src/protocol/event_log.cpp
    src/protocol/wire_protocol.cpp
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/demo.ini
    ${CMAKE_CURRENT_BINARY_DIR}/config/demo.ini
    COPYONLY
)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config/range_only_demo.ini
    ${CMAKE_CURRENT_BINARY_DIR}/config/range_only_demo.ini
    COPYONLY
)

# 第一层：内部静态核心隐藏全部C++符号，只作为工程内实现依赖。
add_library(zju_coop_core STATIC ${ZJU_COOP_CORE_SOURCES})
target_compile_features(zju_coop_core PUBLIC cxx_std_17)
target_include_directories(zju_coop_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
set_target_properties(zju_coop_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

# 第二层：对外交付动态库只导出include/zju_coop/c_api.h声明的稳定C符号。
add_library(zju_coop SHARED src/api/c_api.cpp)
target_compile_features(zju_coop PRIVATE cxx_std_17)
target_include_directories(zju_coop
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(zju_coop PRIVATE ZJU_COOP_BUILDING_LIBRARY)
target_link_libraries(zju_coop PRIVATE zju_coop_core)
set_target_properties(zju_coop PROPERTIES
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)

function(zju_coop_verify_hidden_visibility target_name)
  get_target_property(target_cxx_visibility
      ${target_name} CXX_VISIBILITY_PRESET)
  get_target_property(target_inline_visibility
      ${target_name} VISIBILITY_INLINES_HIDDEN)
  if(NOT target_cxx_visibility STREQUAL "hidden")
    message(FATAL_ERROR
        "${target_name} must use CXX_VISIBILITY_PRESET=hidden")
  endif()
  if(NOT target_inline_visibility)
    message(FATAL_ERROR
        "${target_name} must use VISIBILITY_INLINES_HIDDEN=YES")
  endif()
endfunction()

zju_coop_verify_hidden_visibility(zju_coop_core)
zju_coop_verify_hidden_visibility(zju_coop)

if(UNIX AND NOT APPLE)
  # Linux/ARM64构建后检查动态符号，防止C++实现细节意外成为上交依赖。
  if(NOT CMAKE_NM)
    message(FATAL_ERROR "CMAKE_NM is required for ELF export validation")
  endif()
  add_custom_command(TARGET zju_coop POST_BUILD
      COMMAND ${CMAKE_COMMAND}
          "-DNM_TOOL=${CMAKE_NM}"
          "-DLIBRARY_FILE=$<TARGET_FILE:zju_coop>"
          -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_elf_exports.cmake"
      COMMENT "Checking zju_coop ELF export boundary"
      VERBATIM
  )
endif()

function(zju_coop_enable_warnings target_name)
  if(MSVC)
    # 源码和接口文档包含中文注释；显式使用UTF-8，避免受系统代码页影响。
    # Windows SDK 10.0.26100 的系统头含非UTF-8字节，C4828与本工程源码无关。
    target_compile_options(${target_name} PRIVATE
        /W4 /permissive- /FS /utf-8 /wd4828)
  else()
    target_compile_options(${target_name} PRIVATE
        -Wall -Wextra -Wpedantic
    )
  endif()
endfunction()

zju_coop_enable_warnings(zju_coop_core)
zju_coop_enable_warnings(zju_coop)

set(ZJU_COOP_DEMO_SUPPORT_SOURCES
    src/apps/app_support.cpp
    src/net/udp_socket.cpp
)

# 第三层：临时UDP在线/回放程序用于无ROS 2阶段验证，不属于算法库ABI。
add_executable(zju_coop_online
    src/apps/online_main.cpp
    ${ZJU_COOP_DEMO_SUPPORT_SOURCES}
)
add_executable(zju_coop_replay
    src/apps/replay_main.cpp
    ${ZJU_COOP_DEMO_SUPPORT_SOURCES}
)
foreach(demo_target IN ITEMS zju_coop_online zju_coop_replay)
  target_compile_features(${demo_target} PRIVATE cxx_std_17)
  target_include_directories(${demo_target} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(${demo_target} PRIVATE zju_coop_core zju_coop)
  if(WIN32)
    target_link_libraries(${demo_target} PRIVATE ws2_32)
  endif()
  if(UNIX AND NOT APPLE)
    set_target_properties(${demo_target} PROPERTIES
        INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
    )
  endif()
  zju_coop_enable_warnings(${demo_target})
endforeach()

add_executable(zju_coop_sdk_consumer examples/sdk_consumer.cpp)
target_compile_features(zju_coop_sdk_consumer PRIVATE cxx_std_17)
target_link_libraries(zju_coop_sdk_consumer PRIVATE zju_coop)
zju_coop_enable_warnings(zju_coop_sdk_consumer)

# 独立自检只调用公开C ABI，不启动在线网络和ROS 2，适合交付前快速验收。
add_executable(zju_coop_self_check
    src/apps/self_check_main.cpp
    src/self_check/self_check.cpp
)
target_compile_features(zju_coop_self_check PRIVATE cxx_std_17)
target_include_directories(zju_coop_self_check PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(zju_coop_self_check PRIVATE zju_coop)
zju_coop_enable_warnings(zju_coop_self_check)
if(UNIX AND NOT APPLE)
  set_target_properties(zju_coop_self_check PROPERTIES
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
  )
endif()

if(BUILD_TESTING)
  # 单元测试、C头文件冒烟、SDK示例、自检和Python端到端测试统一纳入CTest。
  add_executable(zju_coop_tests
      tests/test_main.cpp
      tests/test_c_api.cpp
      tests/test_app_telemetry.cpp
      tests/test_cooperative_inertial_ekf.cpp
      tests/test_degradation.cpp
      tests/test_engine.cpp
      tests/test_event_log.cpp
      tests/test_ini_config.cpp
      tests/test_inertial_eskf15.cpp
      tests/test_protocol.cpp
      tests/test_quaternion.cpp
      tests/test_range_ekf.cpp
      tests/test_rigidity.cpp
      tests/test_self_check.cpp
      tests/test_types.cpp
      tests/test_udp_socket.cpp
      src/self_check/self_check.cpp
      src/net/udp_socket.cpp
      src/apps/app_support.cpp
  )
  target_compile_features(zju_coop_tests PRIVATE cxx_std_17)
  target_include_directories(zju_coop_tests PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/tests
  )
  target_link_libraries(zju_coop_tests PRIVATE zju_coop_core zju_coop)
  if(WIN32)
    target_link_libraries(zju_coop_tests PRIVATE ws2_32)
  endif()
  zju_coop_enable_warnings(zju_coop_tests)

  add_executable(c_header_smoke tests/c_header_smoke.c)
  target_link_libraries(c_header_smoke PRIVATE zju_coop)
  zju_coop_enable_warnings(c_header_smoke)

  add_test(NAME zju_coop_tests COMMAND zju_coop_tests)
  add_test(NAME c_header_smoke COMMAND c_header_smoke)
  add_test(NAME zju_coop_sdk_consumer COMMAND zju_coop_sdk_consumer)
  add_test(NAME zju_coop_self_check COMMAND zju_coop_self_check)

  find_package(Python3 QUIET COMPONENTS Interpreter)
  if(Python3_Interpreter_FOUND)
    # Python仅用于模拟数据、协议交叉验证和进程级冒烟，不是RK3588算法运行依赖。
    add_test(NAME zju_coop_python_tests
        COMMAND ${Python3_EXECUTABLE} -B -m unittest discover
            -s ${CMAKE_CURRENT_SOURCE_DIR}/tools -p test_*.py
    )
  add_test(NAME zju_coop_range_only_online_smoke
        COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/online_smoke_test.py
            --online-exe $<TARGET_FILE:zju_coop_online>
            --replay-exe $<TARGET_FILE:zju_coop_replay>
            --config ${CMAKE_CURRENT_BINARY_DIR}/config/range_only_demo.ini
    )
    set_tests_properties(zju_coop_range_only_online_smoke PROPERTIES TIMEOUT 15)
    add_test(NAME zju_coop_default_online_smoke
        COMMAND ${Python3_EXECUTABLE} -B
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/imu_online_smoke_test.py
            --online-exe $<TARGET_FILE:zju_coop_online>
            --replay-exe $<TARGET_FILE:zju_coop_replay>
            --config ${CMAKE_CURRENT_BINARY_DIR}/config/demo.ini
    )
    set_tests_properties(zju_coop_default_online_smoke PROPERTIES TIMEOUT 15)
  endif()
endif()

install(TARGETS zju_coop
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(TARGETS zju_coop_online zju_coop_replay zju_coop_self_check
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES
    include/zju_coop/c_api.h
    include/zju_coop/export.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/zju_coop
)
install(FILES config/demo.ini config/range_only_demo.ini
    DESTINATION ${CMAKE_INSTALL_DATADIR}/zju_coop/config
)
install(FILES
    README.md
    临时UDP演示协议_v1.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/zju_coop
)
install(DIRECTORY docs/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/zju_coop/docs
    FILES_MATCHING PATTERN "*.md"
)
install(PROGRAMS
    tools/zjcl_protocol.py
    tools/uwb_simulator.py
    tools/imu_uwb_simulator.py
    tools/gcs_dashboard.py
    tools/online_smoke_test.py
    tools/imu_online_smoke_test.py
    DESTINATION ${CMAKE_INSTALL_DATADIR}/zju_coop/tools
)

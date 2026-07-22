# 模块职责：Linux/ARM64构建后用nm审计.so动态符号，只允许稳定zju_coop_* C API导出。
# 该脚本由CMake POST_BUILD调用，不参与Windows DLL构建。
if(NOT DEFINED NM_TOOL OR NM_TOOL STREQUAL "")
  message(FATAL_ERROR "NM_TOOL was not provided")
endif()
if(NOT DEFINED LIBRARY_FILE OR LIBRARY_FILE STREQUAL "" OR
   NOT EXISTS "${LIBRARY_FILE}")
  message(FATAL_ERROR "LIBRARY_FILE is missing: ${LIBRARY_FILE}")
endif()

# -D只读取运行时动态符号表，--defined-only排除本库仅引用的外部符号；
# 因而检查目标是“调用方实际可链接到的导出”，不是ELF中的全部调试/局部符号。
execute_process(
    COMMAND "${NM_TOOL}" -D --defined-only "${LIBRARY_FILE}"
    RESULT_VARIABLE nm_result
    OUTPUT_VARIABLE dynamic_symbols
    ERROR_VARIABLE nm_error
)
if(NOT nm_result EQUAL 0)
  message(FATAL_ERROR
      "Failed to inspect ELF exports (${nm_result}): ${nm_error}")
endif()

# Itanium ABI会在普通方法、RTTI、虚表和局部静态辅助符号中编码“3zju4coop”；
# 只匹配该命名空间片段，可在阻止本工程C++符号泄漏的同时允许系统运行时弱符号。
if(dynamic_symbols MATCHES "3zju4coop")
  message(FATAL_ERROR
      "ELF export leak: a zju::coop C++ symbol is dynamically visible:\n"
      "${dynamic_symbols}")
endif()

# 至少检查一个必需C ABI入口，避免可见性配置过严导致生成了没有公开入口的.so。
# 其他zju_coop_*入口的完整性由C头文件冒烟和C ABI单元测试负责。
if(NOT dynamic_symbols MATCHES "zju_coop_abi_version")
  message(FATAL_ERROR
      "ELF export check found no public zju_coop_ C API symbol:\n"
      "${dynamic_symbols}")
endif()

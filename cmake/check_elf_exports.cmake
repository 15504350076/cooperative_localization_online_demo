if(NOT DEFINED NM_TOOL OR NM_TOOL STREQUAL "")
  message(FATAL_ERROR "NM_TOOL was not provided")
endif()
if(NOT DEFINED LIBRARY_FILE OR LIBRARY_FILE STREQUAL "" OR
   NOT EXISTS "${LIBRARY_FILE}")
  message(FATAL_ERROR "LIBRARY_FILE is missing: ${LIBRARY_FILE}")
endif()

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

# The Itanium ABI encodes the namespace as "3zju4coop" in ordinary methods,
# RTTI, vtables, guard variables, and local-static helpers. Matching only that
# namespace fragment leaves unrelated platform/runtime weak symbols allowed.
if(dynamic_symbols MATCHES "3zju4coop")
  message(FATAL_ERROR
      "ELF export leak: a zju::coop C++ symbol is dynamically visible:\n"
      "${dynamic_symbols}")
endif()

if(NOT dynamic_symbols MATCHES "zju_coop_abi_version")
  message(FATAL_ERROR
      "ELF export check found no public zju_coop_ C API symbol:\n"
      "${dynamic_symbols}")
endif()

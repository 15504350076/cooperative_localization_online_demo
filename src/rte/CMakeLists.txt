# Eigen仅提供扩展库内部的QR/SVD/特征值运算，不出现在公开C ABI中。
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_library(zju_coop_rte SHARED relative_transform.cpp)
target_compile_features(zju_coop_rte PRIVATE cxx_std_17)
target_include_directories(zju_coop_rte PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_definitions(zju_coop_rte PRIVATE ZJU_COOP_BUILDING_RTE_LIBRARY)
target_link_libraries(zju_coop_rte PRIVATE Eigen3::Eigen)
set_target_properties(zju_coop_rte PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    VERSION 1.1.0
    SOVERSION 1
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)
zju_coop_enable_warnings(zju_coop_rte)
zju_coop_verify_hidden_visibility(zju_coop_rte)

install(TARGETS zju_coop_rte
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(FILES ${PROJECT_SOURCE_DIR}/include/zju_coop/relative_transform_api.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/zju_coop
)

if(BUILD_TESTING)
  add_executable(zju_coop_rte_self_check ${PROJECT_SOURCE_DIR}/tests/test_relative_transform.cpp)
  target_compile_features(zju_coop_rte_self_check PRIVATE cxx_std_17)
  target_link_libraries(zju_coop_rte_self_check PRIVATE zju_coop_rte)
  set_target_properties(zju_coop_rte_self_check PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  zju_coop_enable_warnings(zju_coop_rte_self_check)
  add_test(NAME zju_coop_rte_self_check COMMAND zju_coop_rte_self_check)

  add_executable(c_header_smoke_rte ${PROJECT_SOURCE_DIR}/tests/c_header_smoke_rte.c)
  target_link_libraries(c_header_smoke_rte PRIVATE zju_coop_rte)
  set_target_properties(c_header_smoke_rte PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
  zju_coop_enable_warnings(c_header_smoke_rte)
  add_test(NAME c_header_smoke_rte COMMAND c_header_smoke_rte)
endif()

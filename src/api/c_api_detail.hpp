// C ABI 实现内部共用的结构头、数组跨度和基础值校验。
// 该头文件不安装、不导出，只用于各领域适配翻译单元复用同一套信任边界。
#ifndef ZJU_COOP_SRC_API_C_API_DETAIL_HPP
#define ZJU_COOP_SRC_API_C_API_DETAIL_HPP

#include "zju_coop/c_api.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>

namespace zju::coop::c_api_detail {

template <typename Structure>
zju_coop_error_code_t validate_header(const Structure* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (value->struct_size < sizeof(Structure)) {
    return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  }
  if (value->abi_version != ZJU_COOP_ABI_VERSION_V1) {
    return ZJU_COOP_ABI_MISMATCH;
  }
  return ZJU_COOP_OK;
}

template <typename Structure>
zju_coop_error_code_t validate_pose2d_header(const Structure* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (value->struct_size < sizeof(Structure)) {
    return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  }
  if (value->abi_version != ZJU_COOP_POSE2D_ABI_VERSION_V2) {
    return ZJU_COOP_ABI_MISMATCH;
  }
  return ZJU_COOP_OK;
}

inline bool valid_boolean(zju_coop_bool_t value) {
  return value == ZJU_COOP_FALSE || value == ZJU_COOP_TRUE;
}

inline bool is_finite_value(double value) { return std::isfinite(value); }

template <std::size_t Size>
bool finite_array(const double (&values)[Size]) {
  return std::all_of(std::begin(values), std::end(values),
                     [](double value) { return is_finite_value(value); });
}

inline bool nul_terminated_nonempty(const char* value,
                                    std::size_t capacity) {
  return value != nullptr && value[0] != '\0' &&
         std::memchr(value, '\0', capacity) != nullptr;
}

template <std::size_t Size>
bool all_zero(const std::uint8_t (&values)[Size]) {
  return std::all_of(std::begin(values), std::end(values),
                     [](std::uint8_t value) { return value == 0U; });
}

template <typename Structure>
bool valid_array_span(const void* base, std::uint32_t count,
                      std::uint32_t stride) {
  if (count == 0U) {
    return true;
  }
  if (base == nullptr || stride < sizeof(Structure) ||
      stride % alignof(Structure) != 0U) {
    return false;
  }

  const auto base_address = reinterpret_cast<std::uintptr_t>(base);
  if (base_address % alignof(Structure) != 0U) {
    return false;
  }
  const auto maximum = std::numeric_limits<std::uintptr_t>::max();
  const std::uintptr_t last_index = count - 1U;
  if (last_index != 0U && stride > maximum / last_index) {
    return false;
  }
  const std::uintptr_t last_offset = last_index * stride;
  if (base_address > maximum - last_offset) {
    return false;
  }
  const std::uintptr_t last_address = base_address + last_offset;
  constexpr std::uintptr_t last_byte = sizeof(Structure) - 1U;
  return last_address <= maximum - last_byte;
}

template <typename Structure>
const Structure* array_element(const void* base, std::uint32_t index,
                               std::uint32_t stride) {
  const auto* bytes = static_cast<const unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<const Structure*>(bytes + offset);
}

template <typename Structure>
Structure* array_element(void* base, std::uint32_t index,
                         std::uint32_t stride) {
  auto* bytes = static_cast<unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<Structure*>(bytes + offset);
}

}  // namespace zju::coop::c_api_detail

#endif

// 无查表依赖的 IEEE CRC32 实现，保证 C++ 与 Python 协议编码结果一致。
#include "protocol/crc32.hpp"

namespace zju::coop::protocol {

std::uint32_t crc32_ieee(
    const std::vector<std::uint8_t>& bytes) noexcept {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (const std::uint8_t byte : bytes) {
    crc ^= byte;
    for (unsigned int bit = 0U; bit < 8U; ++bit) {
      const std::uint32_t mask =
          0U - static_cast<std::uint32_t>(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

}  // namespace zju::coop::protocol

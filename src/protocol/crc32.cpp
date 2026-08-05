// 模块实现：无查表依赖的IEEE CRC32逐位算法，保证C++与Python演示协议黄金向量一致。
// 初学者只需先掌握外层循环“逐字节读入”、内层循环“逐位更新寄存器”；
// 0xEDB88320U是标准规定的多项式常量，不是本项目调参值，也不能随意修改。
// `>>`右移一位相当于处理下一个CRC位；`&`提取最低位；`^`执行按位异或。
#include "protocol/crc32.hpp"

namespace zju::coop::protocol {

std::uint32_t crc32_ieee(
    const std::vector<std::uint8_t>& bytes) noexcept {
  // 采用反射多项式0xEDB88320，初值和终值均异或全1。
  // crc为逐字节更新的反射CRC寄存器；byte遍历调用方指定覆盖范围中的每个线序字节。
  std::uint32_t crc = 0xFFFFFFFFU;
  for (const std::uint8_t byte : bytes) {
    crc ^= byte;
    // bit遍历当前byte的8个低位优先处理步骤；mask把当前最低位扩展成全0或全1。
    for (unsigned int bit = 0U; bit < 8U; ++bit) {
      const std::uint32_t mask =
          0U - static_cast<std::uint32_t>(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

}  // namespace zju::coop::protocol

// 临时在线帧和事件日志共用的 IEEE CRC32 校验接口。
#pragma once

#include <cstdint>
#include <vector>

namespace zju::coop::protocol {

[[nodiscard]] std::uint32_t crc32_ieee(
    const std::vector<std::uint8_t>& bytes) noexcept;

}  // namespace zju::coop::protocol

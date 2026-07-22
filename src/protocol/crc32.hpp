// 模块职责：提供临时在线帧和事件日志共用的IEEE CRC32校验接口。
// CRC只用于发现传输/文件损坏，不提供身份认证或加密；正式通信安全由上交通信系统负责。
#pragma once

#include <cstdint>
#include <vector>

namespace zju::coop::protocol {

[[nodiscard]] std::uint32_t crc32_ieee(
    const std::vector<std::uint8_t>& bytes) noexcept;

}  // namespace zju::coop::protocol

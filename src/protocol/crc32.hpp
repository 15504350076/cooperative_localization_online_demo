// 模块职责：提供临时在线帧和事件日志共用的IEEE CRC32校验接口。
// CRC只用于发现传输/文件损坏，不提供身份认证或加密；正式通信安全由上海交大通信系统负责。
//
// C++初学者阅读提示：输入是若干字节，输出是32位“指纹”。接收方对相同字节重新计算，
// 如果结果不同，说明报文在传输或存储中发生了变化；结果相同也不能证明发送者身份可信。
#pragma once

#include <cstdint>
#include <vector>

namespace zju::coop::protocol {

/**
 * IEEE CRC-32（反射多项式0xEDB88320、初值/终值异或0xFFFFFFFF）。
 * 调用方决定覆盖字节范围；ZJCL v1会把CRC字段从输入中排除，而不是依赖主机布局。
 */
[[nodiscard]] std::uint32_t crc32_ieee(
    // bytes为调用方选择的连续校验输入；ZJCL调用者必须传入头[0,36)+payload，明确排除CRC字段。
    const std::vector<std::uint8_t>& bytes) noexcept;

}  // namespace zju::coop::protocol

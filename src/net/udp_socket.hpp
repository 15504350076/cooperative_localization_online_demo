// 模块职责：封装Windows Winsock与Linux BSD Socket差异，为临时在线演示提供最小UDP收发。
// 模块边界：不实现无线转发、路由、重传或链路维护；这些仍由上交通信模块负责。
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace zju::coop::net {

enum class ReceiveStatus {
  // kTimeout是正常轮询结果；其他socket错误通过异常报告，二者不能混为丢包。
  kData,
  kTimeout,
};

/** 一次完整UDP数据报及发送端地址；UDP边界必须原样保留。 */
struct Datagram {
  std::vector<std::uint8_t> bytes;
  std::string source_address;
  std::uint16_t source_port{};
};

struct ReceiveResult {
  ReceiveStatus status{ReceiveStatus::kTimeout};
  Datagram datagram;
};

/** RAII UDP套接字，支持移动但禁止复制，析构时回收平台网络资源。 */
class UdpSocket {
 public:
  static constexpr std::size_t kMaximumDatagramSize = 65'507U;

  UdpSocket();
  ~UdpSocket();

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;

  /** 只接受数字IPv4地址；port=0允许系统分配临时端口，便于测试。 */
  void bind(const std::string& ipv4_address, std::uint16_t port);
  void set_receive_timeout(std::chrono::milliseconds timeout);
  /** 一次调用返回一个完整数据报；不会把两个数据报拼接，也不做应用层重组。 */
  [[nodiscard]] ReceiveResult receive();
  /** 超过IPv4 UDP最大载荷65507字节时在调用sendto前直接拒绝。 */
  void send_to(const std::string& ipv4_address, std::uint16_t port,
               const std::vector<std::uint8_t>& bytes);
  [[nodiscard]] std::uint16_t local_port() const;

 private:
  static std::uintptr_t invalid_socket() noexcept;
  void close() noexcept;

  std::uintptr_t socket_{};
  bool runtime_started_{};
};

}  // namespace zju::coop::net

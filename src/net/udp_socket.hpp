// Windows 与 Linux 共用的最小 UDP 封装，只服务于临时在线演示，不承担无线通信协议。
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace zju::coop::net {

enum class ReceiveStatus {
  kData,
  kTimeout,
};

struct Datagram {
  std::vector<std::uint8_t> bytes;
  std::string source_address;
  std::uint16_t source_port{};
};

struct ReceiveResult {
  ReceiveStatus status{ReceiveStatus::kTimeout};
  Datagram datagram;
};

class UdpSocket {
 public:
  static constexpr std::size_t kMaximumDatagramSize = 65'507U;

  UdpSocket();
  ~UdpSocket();

  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;

  void bind(const std::string& ipv4_address, std::uint16_t port);
  void set_receive_timeout(std::chrono::milliseconds timeout);
  [[nodiscard]] ReceiveResult receive();
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

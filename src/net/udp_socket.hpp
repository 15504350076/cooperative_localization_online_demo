// 模块职责：封装Windows Winsock与Linux BSD Socket差异，为临时在线演示提供最小UDP收发。
// 模块边界：不实现无线转发、路由、重传或链路维护；这些仍由上海交大通信模块负责。
//
// C++初学者阅读提示：
// 1. UDP把一段字节发送到“IP地址+端口”，它不保证到达、顺序或只到达一次。
// 2. UdpReceiver负责bind本机端口并接收；UdpSender保存目标端点并发送。
// 3. 类析构时自动关闭socket，这种“对象活着=资源有效”的写法称为RAII。
// 4. 这只是脱离ROS 2时的演示入口；正式盒端通常由ROS 2适配节点直接调用C ABI。
// 5. 拷贝构造/赋值被`= delete`禁用，防止两个对象重复关闭同一个系统句柄；
//    `T&&`是右值引用，移动构造把句柄所有权从旧对象转给新对象。
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace zju::coop::net {

enum class ReceiveStatus {
  kData,     ///< 在超时时间内收到了一个完整UDP数据报。
  kTimeout,  ///< 本轮没有数据，是在线循环的正常结果，不等同于已确认丢包。
};

/** 一次完整UDP数据报及发送端地址；UDP边界必须原样保留。 */
struct Datagram {
  std::vector<std::uint8_t> bytes;  ///< 单个UDP数据报的完整载荷字节。
  std::string source_address;       ///< 发送端的数字IPv4地址文本。
  std::uint16_t source_port{};      ///< 发送端的主机字节序UDP端口。
};

struct ReceiveResult {
  ReceiveStatus status{ReceiveStatus::kTimeout};  ///< 本次调用收到整报还是正常超时。
  Datagram datagram;  ///< status为kData时有效的载荷及发送端地址。
};

/** RAII UDP套接字，支持移动但禁止复制，析构时回收平台网络资源。 */
class UdpSocket {
 public:
  static constexpr std::size_t kMaximumDatagramSize = 65'507U;  ///< IPv4 UDP载荷的协议最大字节数。

  UdpSocket();
  ~UdpSocket();

  /** 未命名源对象参数刻意禁用套接字所有权复制。 */
  UdpSocket(const UdpSocket&) = delete;
  /** 未命名源对象参数刻意禁用套接字所有权复制赋值。 */
  UdpSocket& operator=(const UdpSocket&) = delete;
  /** @param other 将平台套接字及Winsock运行时所有权移入本对象。 */
  UdpSocket(UdpSocket&& other) noexcept;
  /** @param other 将平台套接字及Winsock运行时所有权移入本对象。 */
  UdpSocket& operator=(UdpSocket&& other) noexcept;

  /** 只接受数字IPv4地址；port=0允许系统分配临时端口，便于测试。
   * @param ipv4_address 本地数字IPv4地址；@param port 本地监听端口，0表示由系统分配。 */
  void bind(const std::string& ipv4_address, std::uint16_t port);
  /** @param timeout recvfrom等待数据的正毫秒超时。 */
  void set_receive_timeout(std::chrono::milliseconds timeout);
  /** 一次调用返回一个完整数据报；不会把两个数据报拼接，也不做应用层重组。 */
  [[nodiscard]] ReceiveResult receive();
  /**
   * 超过IPv4 UDP最大载荷65507字节时在调用sendto前直接拒绝。
   * @param ipv4_address 目标数字IPv4地址；@param port 非零目标UDP端口。
   * @param bytes 要作为一个完整数据报发送的载荷。
   */
  void send_to(const std::string& ipv4_address, std::uint16_t port,
               const std::vector<std::uint8_t>& bytes);
  [[nodiscard]] std::uint16_t local_port() const;

 private:
  static std::uintptr_t invalid_socket() noexcept;
  void close() noexcept;

  std::uintptr_t socket_{};  ///< 以无符号整型保存的平台原生套接字所有权。
  bool runtime_started_{};   ///< Windows下本对象是否持有一次成功的WSAStartup。
};

}  // namespace zju::coop::net

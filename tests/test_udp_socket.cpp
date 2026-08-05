// 模块职责：用本机回环验证跨平台UDP绑定、超时、数据报边界、发送和移动所有权。
// C++初学者阅读提示：127.0.0.1/回环地址表示数据只在本机流动，不需要真实车载网络；
// 测试分别检查“无数据时超时”和“发送后原样收到”，不会连接AIBrainBox。
#include "net/udp_socket.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using zju::coop::net::ReceiveStatus;
using zju::coop::net::UdpSocket;

// 回环收发验证UDP保持数据报边界和源端点，而不是只验证能否创建socket。
TEST_CASE(udp_socket_loopback_preserves_payload_and_sender) {
  // receiver：绑定临时回环端口并设置500 ms超时的接收端，避免用例无限阻塞。
  UdpSocket receiver;
  receiver.bind("127.0.0.1", 0U);
  receiver.set_receive_timeout(std::chrono::milliseconds(500));
  EXPECT_TRUE(receiver.local_port() != 0U);

  // sender：发送端原生套接字；payload：包含零字节和高位字节的边界载荷；received：接收状态、原包与源端点快照。
  UdpSocket sender;
  const std::vector<std::uint8_t> payload{0U, 1U, 2U, 0xFEU, 0xFFU};
  sender.send_to("127.0.0.1", receiver.local_port(), payload);
  const auto received = receiver.receive();
  EXPECT_EQ(received.status, ReceiveStatus::kData);
  EXPECT_EQ(received.datagram.bytes, payload);
  EXPECT_EQ(received.datagram.source_address, std::string("127.0.0.1"));
  EXPECT_TRUE(received.datagram.source_port != 0U);
}

// 超时是在线主循环的正常轮询路径；移动测试还防止两个对象重复关闭同一原生句柄。
TEST_CASE(udp_socket_timeout_returns_without_unbounded_blocking) {
  // receiver：无发送方的回环接收端；start/elapsed：量取40 ms超时路径，2 s上限吸收调度抖动；received期望为kTimeout。
  UdpSocket receiver;
  receiver.bind("127.0.0.1", 0U);
  receiver.set_receive_timeout(std::chrono::milliseconds(40));
  const auto start = std::chrono::steady_clock::now();
  const auto received = receiver.receive();
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(received.status, ReceiveStatus::kTimeout);
  EXPECT_TRUE(elapsed < std::chrono::seconds(2));
}

TEST_CASE(udp_socket_rejects_oversize_and_invalid_timeout) {
  // socket：承载两个参数校验场景；oversize_rejected/timeout_rejected：对应异常是否按预期抛出的哨兵。
  UdpSocket socket;
  bool oversize_rejected = false;
  try {
    socket.send_to("127.0.0.1", 39001U,
                   std::vector<std::uint8_t>(
                       UdpSocket::kMaximumDatagramSize + 1U));
  } catch (const std::invalid_argument&) {
    oversize_rejected = true;
  }
  EXPECT_TRUE(oversize_rejected);

  bool timeout_rejected = false;
  try {
    socket.set_receive_timeout(std::chrono::milliseconds(0));
  } catch (const std::invalid_argument&) {
    timeout_rejected = true;
  }
  EXPECT_TRUE(timeout_rejected);
}

TEST_CASE(udp_socket_move_transfers_native_resource) {
  // original：持有临时绑定端口的源对象；port：移动前资源身份；moved：期望独占同一原生句柄的目标对象。
  UdpSocket original;
  original.bind("127.0.0.1", 0U);
  const auto port = original.local_port();
  UdpSocket moved(std::move(original));
  EXPECT_EQ(moved.local_port(), port);
}

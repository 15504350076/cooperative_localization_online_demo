// 模块职责：用本机回环验证跨平台UDP绑定、超时、数据报边界、发送和移动所有权。
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
  UdpSocket receiver;
  receiver.bind("127.0.0.1", 0U);
  receiver.set_receive_timeout(std::chrono::milliseconds(500));
  EXPECT_TRUE(receiver.local_port() != 0U);

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
  UdpSocket original;
  original.bind("127.0.0.1", 0U);
  const auto port = original.local_port();
  UdpSocket moved(std::move(original));
  EXPECT_EQ(moved.local_port(), port);
}

// 模块实现：封装Winsock/BSD Socket创建、绑定、超时接收、整报发送和资源释放差异。
// 关键原则：算法核心只处理字节帧，不依赖平台网络API；UDP超时是正常状态，
// 其他系统错误转为异常交给应用入口记录并退出，禁止静默吞掉网络故障。
#include "net/udp_socket.hpp"

#include <cerrno>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <arpa/inet.h>
#  include <netinet/in.h>
#  include <sys/socket.h>
#  include <sys/time.h>
#  include <unistd.h>
#endif

namespace zju::coop::net {
namespace {

#if defined(_WIN32)
using NativeSocket = SOCKET;

NativeSocket native_socket(std::uintptr_t value) noexcept {
  return static_cast<NativeSocket>(value);
}

int last_error() noexcept { return WSAGetLastError(); }

bool timeout_error(int code) noexcept {
  return code == WSAETIMEDOUT || code == WSAEWOULDBLOCK;
}
#else
using NativeSocket = int;

NativeSocket native_socket(std::uintptr_t value) noexcept {
  return static_cast<NativeSocket>(value);
}

int last_error() noexcept { return errno; }

bool timeout_error(int code) noexcept {
  return code == EAGAIN || code == EWOULDBLOCK;
}
#endif

[[noreturn]] void fail(const char* operation, int code) {
  throw std::runtime_error(std::string(operation) +
                           " failed (socket error " +
                           std::to_string(code) + ")");
}

sockaddr_in endpoint(const std::string& address, std::uint16_t port,
                     bool allow_zero_port) {
  if (address.empty() || (!allow_zero_port && port == 0U)) {
    throw std::invalid_argument("UDP IPv4 address/port is invalid");
  }
  sockaddr_in result{};
  result.sin_family = AF_INET;
  result.sin_port = htons(port);
  const int converted =
      inet_pton(AF_INET, address.c_str(), &result.sin_addr);
  if (converted != 1) {
    throw std::invalid_argument("UDP address must be numeric IPv4");
  }
  return result;
}

}  // namespace

std::uintptr_t UdpSocket::invalid_socket() noexcept {
#if defined(_WIN32)
  return static_cast<std::uintptr_t>(INVALID_SOCKET);
#else
  return static_cast<std::uintptr_t>(-1);
#endif
}

UdpSocket::UdpSocket() : socket_(invalid_socket()) {
  // Windows需要初始化Winsock运行时；Linux构造路径直接创建BSD套接字。
#if defined(_WIN32)
  WSADATA data{};
  const int startup = WSAStartup(MAKEWORD(2, 2), &data);
  if (startup != 0) {
    fail("WSAStartup", startup);
  }
  runtime_started_ = true;
#endif

  const NativeSocket created =
      ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#if defined(_WIN32)
  if (created == INVALID_SOCKET) {
#else
  if (created < 0) {
#endif
    const int code = last_error();
    close();
    fail("socket", code);
  }
  socket_ = static_cast<std::uintptr_t>(created);
}

UdpSocket::~UdpSocket() { close(); }

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : socket_(other.socket_), runtime_started_(other.runtime_started_) {
  other.socket_ = invalid_socket();
  other.runtime_started_ = false;
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
  if (this != &other) {
    close();
    socket_ = other.socket_;
    runtime_started_ = other.runtime_started_;
    other.socket_ = invalid_socket();
    other.runtime_started_ = false;
  }
  return *this;
}

void UdpSocket::close() noexcept {
  // 析构路径不得抛异常；关闭句柄后再按平台释放网络运行时资源。
  if (socket_ != invalid_socket()) {
#if defined(_WIN32)
    closesocket(native_socket(socket_));
#else
    ::close(native_socket(socket_));
#endif
    socket_ = invalid_socket();
  }
#if defined(_WIN32)
  if (runtime_started_) {
    WSACleanup();
    runtime_started_ = false;
  }
#endif
}

void UdpSocket::bind(const std::string& ipv4_address, std::uint16_t port) {
  const sockaddr_in address = endpoint(ipv4_address, port, true);
  if (::bind(native_socket(socket_),
             reinterpret_cast<const sockaddr*>(&address),
             static_cast<int>(sizeof(address))) != 0) {
    fail("bind", last_error());
  }
}

void UdpSocket::set_receive_timeout(std::chrono::milliseconds timeout) {
  if (timeout.count() <= 0) {
    throw std::invalid_argument("UDP receive timeout must be positive");
  }
#if defined(_WIN32)
  if (static_cast<unsigned long long>(timeout.count()) >
      (std::numeric_limits<DWORD>::max)()) {
    throw std::invalid_argument("UDP receive timeout is too large");
  }
  const DWORD milliseconds = static_cast<DWORD>(timeout.count());
  if (setsockopt(native_socket(socket_), SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&milliseconds),
                 static_cast<int>(sizeof(milliseconds))) != 0) {
#else
  timeval value{};
  value.tv_sec = static_cast<decltype(value.tv_sec)>(timeout.count() / 1000);
  value.tv_usec =
      static_cast<decltype(value.tv_usec)>((timeout.count() % 1000) * 1000);
  if (setsockopt(native_socket(socket_), SOL_SOCKET, SO_RCVTIMEO, &value,
                 static_cast<socklen_t>(sizeof(value))) != 0) {
#endif
    fail("setsockopt(SO_RCVTIMEO)", last_error());
  }
}

ReceiveResult UdpSocket::receive() {
  // UDP一次recvfrom对应一个完整数据报；固定65507字节上限避免无界分配。
  std::vector<std::uint8_t> buffer(kMaximumDatagramSize);
  sockaddr_in source{};
#if defined(_WIN32)
  int source_size = static_cast<int>(sizeof(source));
  const int received = recvfrom(
      native_socket(socket_), reinterpret_cast<char*>(buffer.data()),
      static_cast<int>(buffer.size()), 0,
      reinterpret_cast<sockaddr*>(&source), &source_size);
  if (received == SOCKET_ERROR) {
#else
  socklen_t source_size = static_cast<socklen_t>(sizeof(source));
  const ssize_t received = recvfrom(
      native_socket(socket_), buffer.data(), buffer.size(), 0,
      reinterpret_cast<sockaddr*>(&source), &source_size);
  if (received < 0) {
#endif
    const int code = last_error();
    if (timeout_error(code)) {
      return {};
    }
    fail("recvfrom", code);
  }

  buffer.resize(static_cast<std::size_t>(received));
  char source_text[INET_ADDRSTRLEN]{};
  if (inet_ntop(AF_INET, &source.sin_addr, source_text,
                static_cast<socklen_t>(sizeof(source_text))) == nullptr) {
    fail("inet_ntop", last_error());
  }
  ReceiveResult result{};
  result.status = ReceiveStatus::kData;
  result.datagram.bytes = std::move(buffer);
  result.datagram.source_address = source_text;
  result.datagram.source_port = ntohs(source.sin_port);
  return result;
}

void UdpSocket::send_to(const std::string& ipv4_address, std::uint16_t port,
                        const std::vector<std::uint8_t>& bytes) {
  // UDP不存在继续发送剩余字节的可靠语义，部分发送直接视为系统错误。
  if (bytes.size() > kMaximumDatagramSize) {
    throw std::invalid_argument("UDP datagram exceeds 65507 bytes");
  }
  const sockaddr_in destination = endpoint(ipv4_address, port, false);
#if defined(_WIN32)
  const int sent = ::sendto(
      native_socket(socket_), reinterpret_cast<const char*>(bytes.data()),
      static_cast<int>(bytes.size()), 0,
      reinterpret_cast<const sockaddr*>(&destination),
      static_cast<int>(sizeof(destination)));
  if (sent == SOCKET_ERROR) {
#else
  const ssize_t sent = ::sendto(
      native_socket(socket_), bytes.data(), bytes.size(), 0,
      reinterpret_cast<const sockaddr*>(&destination),
      static_cast<socklen_t>(sizeof(destination)));
  if (sent < 0) {
#endif
    fail("sendto", last_error());
  }
  if (static_cast<std::size_t>(sent) != bytes.size()) {
    throw std::runtime_error("sendto sent a partial UDP datagram");
  }
}

std::uint16_t UdpSocket::local_port() const {
  sockaddr_in address{};
#if defined(_WIN32)
  int address_size = static_cast<int>(sizeof(address));
#else
  socklen_t address_size = static_cast<socklen_t>(sizeof(address));
#endif
  if (getsockname(native_socket(socket_),
                  reinterpret_cast<sockaddr*>(&address),
                  &address_size) != 0) {
    fail("getsockname", last_error());
  }
  return ntohs(address.sin_port);
}

}  // namespace zju::coop::net

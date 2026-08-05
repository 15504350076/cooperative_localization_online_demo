// 模块实现：封装Winsock/BSD Socket创建、绑定、超时接收、整报发送和资源释放差异。
// 关键原则：算法核心只处理字节帧，不依赖平台网络API；UDP超时是正常状态，
// 其他系统错误转为异常交给应用入口记录并退出，禁止静默吞掉网络故障。
//
// C++初学者阅读顺序：SocketRuntime初始化平台网络环境 -> 创建socket整数句柄
// -> UdpReceiver绑定端口并收包 / UdpSender解析目标并发包 -> 析构函数关闭句柄。
// #if defined(_WIN32)中的代码只在Windows编译，#else部分用于Ubuntu和RK3588。
// `native_socket_t`是平台相关句柄别名；`reinterpret_cast`用于系统API要求的sockaddr视图；
// move构造/赋值会把旧对象句柄改成无效值，从而保证最终只关闭一次。
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

/** @param value UdpSocket中保存的平台句柄整数表示。 */
NativeSocket native_socket(std::uintptr_t value) noexcept {
  return static_cast<NativeSocket>(value);
}

int last_error() noexcept { return WSAGetLastError(); }

/** @param code 最近一次Winsock调用返回的错误码。 */
bool timeout_error(int code) noexcept {
  return code == WSAETIMEDOUT || code == WSAEWOULDBLOCK;
}
#else
using NativeSocket = int;

/** @param value UdpSocket中保存的平台文件描述符整数表示。 */
NativeSocket native_socket(std::uintptr_t value) noexcept {
  return static_cast<NativeSocket>(value);
}

int last_error() noexcept { return errno; }

/** @param code 最近一次BSD Socket调用对应的errno值。 */
bool timeout_error(int code) noexcept {
  return code == EAGAIN || code == EWOULDBLOCK;
}
#endif

/** @param operation 失败的系统调用名称；@param code 平台套接字错误码。 */
[[noreturn]] void fail(const char* operation, int code) {
  throw std::runtime_error(std::string(operation) +
                           " failed (socket error " +
                           std::to_string(code) + ")");
}

/**
 * @param address 待转换的数字IPv4文本；@param port 主机字节序端口。
 * @param allow_zero_port 是否允许绑定时由系统分配端口。
 */
sockaddr_in endpoint(const std::string& address, std::uint16_t port,
                     bool allow_zero_port) {
  if (address.empty() || (!allow_zero_port && port == 0U)) {
    throw std::invalid_argument("UDP IPv4 address/port is invalid");
  }
  sockaddr_in result{};  // 填充并以网络字节序返回的IPv4端点结构。
  result.sin_family = AF_INET;  // 指定IPv4地址族；否则系统不知道如何解释结构。
  result.sin_port = htons(port);  // htons把主机端口转换为网络规定的大端16位字节序。
  const int converted =  // inet_pton成功转换的地址数量。
      inet_pton(AF_INET, address.c_str(), &result.sin_addr);
  if (converted != 1) {
    throw std::invalid_argument("UDP address must be numeric IPv4");
  }
  return result;  // 按值返回完整端点，结构很小且通常会消除复制。
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
  WSADATA data{};  // WSAStartup返回的Winsock实现信息。
  const int startup = WSAStartup(MAKEWORD(2, 2), &data);  // Winsock 2.2运行时初始化结果。
  if (startup != 0) {
    fail("WSAStartup", startup);
  }
  runtime_started_ = true;  // 记录清理责任，close中据此只调用一次WSACleanup。
#endif

  const NativeSocket created =  // 新建且尚未转移给socket_的UDP原生句柄。
      ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#if defined(_WIN32)
  if (created == INVALID_SOCKET) {
#else
  if (created < 0) {
#endif
    const int code = last_error();  // 创建失败后在清理前保存的平台错误码。
    close();
    fail("socket", code);
  }
  socket_ = static_cast<std::uintptr_t>(created);  // 统一保存为无符号整数，使头文件不必包含平台Socket类型。
}

UdpSocket::~UdpSocket() { close(); }

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : socket_(other.socket_), runtime_started_(other.runtime_started_) {
  other.socket_ = invalid_socket();  // 清空源对象，防止两个析构函数关闭同一原生句柄。
  other.runtime_started_ = false;  // 同时转移Winsock清理责任。
}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
  if (this != &other) {
    close();
    socket_ = other.socket_;  // 接管源对象的原生句柄。
    runtime_started_ = other.runtime_started_;  // 接管源对象的平台运行时清理责任。
    other.socket_ = invalid_socket();  // 把源对象恢复为可安全析构的空状态。
    other.runtime_started_ = false;  // 源对象不再执行WSACleanup。
  }
  return *this;  // 返回当前对象引用，允许a = std::move(b)这类赋值表达式按惯例工作。
}

void UdpSocket::close() noexcept {
  // 析构路径不得抛异常；关闭句柄后再按平台释放网络运行时资源。
  if (socket_ != invalid_socket()) {
#if defined(_WIN32)
    closesocket(native_socket(socket_));
#else
    ::close(native_socket(socket_));
#endif
    socket_ = invalid_socket();  // 关闭后立刻标无效，重复调用close不会再次关闭。
  }
#if defined(_WIN32)
  if (runtime_started_) {
    WSACleanup();
    runtime_started_ = false;  // 清理后清除标志，使close保持幂等。
  }
#endif
}

void UdpSocket::bind(const std::string& ipv4_address, std::uint16_t port) {
  const sockaddr_in address = endpoint(ipv4_address, port, true);  // 本地绑定IPv4端点。
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
  const DWORD milliseconds =  // 传给Windows SO_RCVTIMEO的毫秒值。
      static_cast<DWORD>(timeout.count());
  if (setsockopt(native_socket(socket_), SOL_SOCKET, SO_RCVTIMEO,
                 reinterpret_cast<const char*>(&milliseconds),
                 static_cast<int>(sizeof(milliseconds))) != 0) {
#else
  timeval value{};  // 传给BSD SO_RCVTIMEO的秒/微秒超时结构。
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
  std::vector<std::uint8_t> buffer(kMaximumDatagramSize);  // 单次recvfrom的最大UDP载荷缓冲。
  sockaddr_in source{};  // recvfrom回填的数据报发送端IPv4端点。
#if defined(_WIN32)
  int source_size = static_cast<int>(sizeof(source));  // Windows传入并回填的地址结构字节数。
  const int received = recvfrom(  // 收到的载荷字节数或SOCKET_ERROR。
      native_socket(socket_), reinterpret_cast<char*>(buffer.data()),
      static_cast<int>(buffer.size()), 0,
      reinterpret_cast<sockaddr*>(&source), &source_size);
  if (received == SOCKET_ERROR) {
#else
  socklen_t source_size = static_cast<socklen_t>(sizeof(source));  // BSD传入并回填的地址结构字节数。
  const ssize_t received = recvfrom(  // 收到的载荷字节数或负错误值。
      native_socket(socket_), buffer.data(), buffer.size(), 0,
      reinterpret_cast<sockaddr*>(&source), &source_size);
  if (received < 0) {
#endif
    const int code = last_error();  // recvfrom失败后用于区分正常超时的错误码。
    if (timeout_error(code)) {
      return {};
    }
    fail("recvfrom", code);
  }

  buffer.resize(static_cast<std::size_t>(received));  // 丢掉未使用尾部容量，让bytes.size等于真实报文长度。
  char source_text[INET_ADDRSTRLEN]{};  // 发送端IPv4地址的点分十进制输出缓冲。
  if (inet_ntop(AF_INET, &source.sin_addr, source_text,
                static_cast<socklen_t>(sizeof(source_text))) == nullptr) {
    fail("inet_ntop", last_error());
  }
  ReceiveResult result{};  // 汇总整报载荷和发送端地址的成功接收结果。
  result.status = ReceiveStatus::kData;  // 与超时返回的默认状态明确区分。
  result.datagram.bytes = std::move(buffer);  // 转移65507字节缓冲，避免复制整份数据报。
  result.datagram.source_address = source_text;  // char数组转换为拥有自身内存的std::string。
  result.datagram.source_port = ntohs(source.sin_port);  // ntohs把网络大端端口恢复为主机字节序。
  return result;  // 返回载荷和来源端点的独立副本。
}

void UdpSocket::send_to(const std::string& ipv4_address, std::uint16_t port,
                        const std::vector<std::uint8_t>& bytes) {
  // UDP不存在继续发送剩余字节的可靠语义，部分发送直接视为系统错误。
  if (bytes.size() > kMaximumDatagramSize) {
    throw std::invalid_argument("UDP datagram exceeds 65507 bytes");
  }
  const sockaddr_in destination =  // 已校验且端口非零的目标IPv4端点。
      endpoint(ipv4_address, port, false);
#if defined(_WIN32)
  const int sent = ::sendto(  // Windows报告的发送字节数或SOCKET_ERROR。
      native_socket(socket_), reinterpret_cast<const char*>(bytes.data()),
      static_cast<int>(bytes.size()), 0,
      reinterpret_cast<const sockaddr*>(&destination),
      static_cast<int>(sizeof(destination)));
  if (sent == SOCKET_ERROR) {
#else
  const ssize_t sent = ::sendto(  // BSD Socket报告的发送字节数或负错误值。
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
  sockaddr_in address{};  // getsockname回填的本地IPv4端点。
#if defined(_WIN32)
  int address_size = static_cast<int>(sizeof(address));  // Windows地址结构输入/输出长度。
#else
  socklen_t address_size = static_cast<socklen_t>(sizeof(address));  // BSD地址结构输入/输出长度。
#endif
  if (getsockname(native_socket(socket_),
                  reinterpret_cast<sockaddr*>(&address),
                  &address_size) != 0) {
    fail("getsockname", last_error());
  }
  return ntohs(address.sin_port);
}

}  // namespace zju::coop::net

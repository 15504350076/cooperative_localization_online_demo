// 模块职责：运行临时UDP在线闭环，接收IMU/测距帧、调用算法、发布GCS结果并记录事件日志。
// 模块边界：仅用于无ROS 2阶段和联调冒烟测试；AIBrainBox正式部署由上海交大ROS 2适配节点
// 直接调用C ABI，通信路由和时间同步仍由上海交大负责。
//
// C++初学者可按main中的三个阶段阅读：
// 1. 读取命令行和INI，创建算法会话、UDP收发器及可选日志文件；
// 2. 循环接收一个完整ZJCL帧，解码后按IMU或测距类型调用对应push函数；
// 3. 到达输出周期时调用step，把快照编码后发给GCS并写日志。
// Ctrl+C触发信号处理函数，只把原子布尔量改为false，让主循环安全结束。
// `std::unique_ptr`管理可选日志对象；`std::atomic<bool>`可在信号处理和主循环之间安全传递停止标志；
// `std::chrono`类型把“时刻”和“时长”分开，减少毫秒/纳秒单位混用。
#include "apps/app_support.hpp"
#include "config/ini_config.hpp"
#include "net/udp_socket.hpp"
#include "protocol/event_log.hpp"
#include "protocol/wire_protocol.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

volatile std::sig_atomic_t stop_requested = 0;  // 信号处理器置位、主循环轮询的退出请求。

/** 未命名参数为触发退出的信号编号；处理器只需统一置位。 */
void handle_signal(int) { stop_requested = 1; }

struct Arguments {
  std::filesystem::path config_path{"config/demo.ini"};  ///< 在线程序读取的INI路径。
  std::uint64_t duration_ms{};  ///< 进程最长稳态运行毫秒数，0表示不限时。
};

/** @param text 待解析的十进制非负整数文本；@param name 报错时使用的选项名。 */
std::uint64_t parse_unsigned(const std::string& text, const char* name) {
  if (text.empty()) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  for (const char character : text) {  // character为待验证的单个十进制字符。
    if (character < '0' || character > '9') {
      throw std::invalid_argument(std::string(name) + " must be an integer");
    }
  }
  std::size_t consumed = 0U;       // stoull回填的已消费字符数。
  unsigned long long value = 0U;   // stoull解析得到的宿主无符号整数。
  try {
    value = std::stoull(text, &consumed, 10);
  } catch (const std::exception&) {  // 具体转换异常不外泄，统一报告选项格式错误。
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  if (consumed != text.size()) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  return static_cast<std::uint64_t>(value);
}

/** @param argc 命令行参数个数；@param argv 以空指针结尾约定传入的参数字符串数组。 */
Arguments parse_arguments(int argc, char** argv) {
  Arguments result{};  // 逐项覆盖默认值的解析结果。
  // index定位当前argv选项；argument保存选项名，value在需取值时保存下一参数。
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help") {
      std::cout << "Usage: zju_coop_online [--config PATH] "
                   "[--duration-ms MILLISECONDS]\n";
      std::exit(0);
    }
    if (argument == "--config" || argument == "--duration-ms") {
      if (index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a value");
      }
      const std::string value = argv[++index];
      if (argument == "--config") {
        result.config_path = value;
      } else {
        result.duration_ms = parse_unsigned(value, "--duration-ms");
      }
      continue;
    }
    throw std::invalid_argument("unknown argument: " + argument);
  }
  return result;
}

struct Statistics {
  std::uint64_t received{};                ///< recvfrom得到的UDP数据报数。
  std::uint64_t range_frames{};            ///< 成功解码并送入C ABI的测距帧数。
  std::uint64_t imu_frames{};              ///< 成功解码并送入C ABI的IMU帧数。
  std::uint64_t propagated_imu{};          ///< C ABI报告完成传播的IMU帧数。
  std::uint64_t accepted_ranges{};         ///< 引擎处置为Processed的测距帧数。
  std::uint64_t rejected_ranges{};         ///< 引擎未处置为Processed的测距帧数。
  std::uint64_t protocol_rejected{};       ///< 帧或类型载荷解码失败数。
  std::uint64_t type_rejected{};           ///< 在线入口不接受的消息类型或模式不匹配数。
  std::uint64_t api_rejected{};            ///< C ABI未处置为Processed的测距调用数。
  std::uint64_t published_localization{};  ///< 已发送的定位输出帧数。
  std::uint64_t published_pose2d{};        ///< 已发送的最小二维位姿输出帧数。
  std::uint64_t published_network{};       ///< 已发送的网络输出帧数。
  std::uint64_t published_observation{};   ///< 已发送的观测输出帧数。
  std::uint64_t published_status{};        ///< 已发送的算法状态帧数。
  std::uint64_t published_alert{};         ///< 已发送的告警活动或清除帧数。
};

/** @param path 配置指定、即将创建或截断覆盖写入的事件日志文件路径。 */
void prepare_log_path(const std::filesystem::path& path) {
  if (path.empty()) {
    throw std::invalid_argument("event log path is empty");
  }
  const auto parent = path.parent_path();  // 需要按需创建的日志父目录。
  if (!parent.empty()) {
    std::error_code error;  // create_directories的非异常错误输出。
    std::filesystem::create_directories(parent, error);
    if (error) {
      throw std::runtime_error("cannot create event log directory: " +
                               error.message());
    }
  }
  if (std::filesystem::is_directory(path)) {
    throw std::runtime_error("event log path names a directory");
  }
}

/** @param type 已成功发送的输出帧类型；@param stats 待原位累加的运行计数。 */
void count_output(zju::coop::protocol::MessageType type, Statistics& stats) {
  using zju::coop::protocol::MessageType;
  switch (type) {
    case MessageType::kLocalization:
      ++stats.published_localization;
      return;
    case MessageType::kNetwork:
      ++stats.published_network;
      return;
    case MessageType::kObservation:
      ++stats.published_observation;
      return;
    case MessageType::kAlgorithmStatus:
      ++stats.published_status;
      return;
    case MessageType::kPose2D:
      ++stats.published_pose2d;
      return;
    case MessageType::kAlert:
      ++stats.published_alert;
      return;
    case MessageType::kRange:
    case MessageType::kImu:
      throw std::runtime_error("unexpected online output message type");
  }
  throw std::runtime_error("unknown online output message type");
}

/** @param arguments 已完成合法性解析的在线运行参数。 */
int run(const Arguments& arguments) {
  using namespace zju::coop;
  // 阶段1：先加载并完整验证配置，再创建算法、输入套接字、输出套接字和可选日志。
  const auto demo_config =  // 同时驱动算法、UDP和日志边界的已验证配置。
      config::load_ini_config(arguments.config_path);
  apps::AlgorithmSession algorithm(demo_config);  // 本进程独占的C ABI算法会话。
  net::UdpSocket input;  // 绑定配置地址并承担数据报接收的套接字。
  input.bind(demo_config.online.input_bind_address,
             demo_config.online.input_port);
  input.set_receive_timeout(std::chrono::milliseconds(10));
  net::UdpSocket output;  // 向配置GCS端点发送整帧的未绑定套接字。

  std::unique_ptr<protocol::EventLogWriter> event_log;  // 启用时独占的事件日志写入器。
  std::filesystem::path event_log_path;  // SUMMARY输出使用的实际日志路径。
  if (demo_config.online.event_log_enabled) {
    event_log_path = demo_config.online.event_log_path;
    prepare_log_path(event_log_path);
    event_log = std::make_unique<protocol::EventLogWriter>(
        event_log_path, demo_config.online.max_log_record_size,
        demo_config.online.max_payload_size);
  }

  Statistics stats{};  // 在线生命周期内累计并最终输出的运行计数。
  apps::TelemetryEncoder telemetry;  // 跨周期保存告警生命周期的编码器。
  std::uint64_t next_output_sequence = 1U;  // 所有输出类型共享的下一个发送序号。
  const auto start = std::chrono::steady_clock::now();  // 运行时长和周期调度的单调起点。
  const auto deadline =  // 单调时钟域中的可选进程停止截止点。
      arguments.duration_ms == 0U
          ? std::chrono::steady_clock::time_point::max()
          : start + std::chrono::milliseconds(arguments.duration_ms);
  const std::chrono::duration<double> output_period(  // 配置输出频率对应的浮点秒周期。
      1.0 / demo_config.online.output_rate_hz);
  const auto output_period_ticks = std::chrono::duration_cast<  // 转到steady_clock刻度的调度周期。
      std::chrono::steady_clock::duration>(output_period);
  if (output_period_ticks <= std::chrono::steady_clock::duration::zero()) {
    throw std::invalid_argument("configured output rate is too high");
  }
  auto next_output = start;  // 单调时钟域中下一次允许输出的周期截止点。

  while (stop_requested == 0 && std::chrono::steady_clock::now() < deadline) {
    // 阶段2：按数据报边界解码，只把通过协议和载荷校验的输入送入C ABI。
    const auto received = input.receive();  // 本轮完整数据报或正常超时结果。
    if (received.status == net::ReceiveStatus::kData) {
      ++stats.received;
      const std::uint64_t receive_timestamp_ns =  // 数据报到达后采集的统一系统墙上时间。
          apps::system_time_ns();
      const auto decoded = protocol::decode_frame(  // 公共头、长度和校验码的解码结果。
          received.datagram.bytes, demo_config.online.max_payload_size);
      if (!decoded.ok()) {
        ++stats.protocol_rejected;
      } else if (decoded.value.header.message_type ==
                 protocol::MessageType::kRange) {
        const auto range =  // 已校验帧内测距载荷的解码结果。
            protocol::decode_range_payload(decoded.value.payload);
        if (!range.ok()) {
          ++stats.protocol_rejected;
        } else {
          if (event_log) {
            // 在C ABI处理前记录已经通过协议校验的输入，使API拒绝、NIS拒绝和
            // 质量状态转换都能在任务后按相同到达顺序复现。
            event_log->append({protocol::EventLogDirection::Input,
                               receive_timestamp_ns,
                               received.datagram.bytes});
          }
          const auto processing = algorithm.push_range(  // C ABI返回的测距入口处置。
              decoded.value, range.value, receive_timestamp_ns);
          ++stats.range_frames;
          if (processing.disposition == ZJU_COOP_PROCESSING_PROCESSED) {
            ++stats.accepted_ranges;
          } else {
            ++stats.rejected_ranges;
            ++stats.api_rejected;
          }
        }
      } else if (decoded.value.header.message_type ==
                 protocol::MessageType::kImu) {
        const auto imu =  // 已校验帧内瞬时IMU载荷的解码结果。
            protocol::decode_imu_payload(decoded.value.payload);
        if (!imu.ok()) {
          ++stats.protocol_rejected;
        } else if (!demo_config.inertial) {
          ++stats.type_rejected;
        } else {
          if (event_log) {
            event_log->append({protocol::EventLogDirection::Input,
                               receive_timestamp_ns,
                               received.datagram.bytes});
          }
          const auto processing = algorithm.push_imu(  // C ABI返回的IMU传播处置。
              decoded.value, imu.value, receive_timestamp_ns);
          ++stats.imu_frames;
          if (processing.propagated == ZJU_COOP_TRUE) {
            ++stats.propagated_imu;
          }
        }
      } else {
        ++stats.type_rejected;
      }
    }

    // 阶段3：输入接收与固定频率输出解耦，无输入时仍可发布超时和拓扑告警。
    // steady_clock只负责本进程调度，system_time_ns写入接口/日志；二者不可混用，
    // 避免系统时间校正造成输出循环倒退或忙等。
    const auto now = std::chrono::steady_clock::now();  // 仅用于本地周期和截止判断的单调时刻。
    if (now >= next_output) {
      const std::uint64_t step_time_ns =  // 传入算法并写入快照的统一系统墙上时间。
          apps::system_time_ns();
      const auto snapshot = algorithm.step(step_time_ns);  // 该统一时刻的原子算法输出。
      const auto frames = apps::encode_snapshot(  // 旧定位、Pose2D、网络和观测协议帧集合。
          snapshot, demo_config.engine.filter.reference_node_id,
          next_output_sequence, demo_config.online.max_payload_size);
      apps::TelemetryCounters counters{};  // 本周期状态帧使用的累计计数副本。
      counters.accepted_ranges = stats.accepted_ranges;
      counters.rejected_ranges = stats.rejected_ranges;
      counters.protocol_errors = stats.protocol_rejected;
      const auto uptime_ns = static_cast<std::uint64_t>(  // 由单调时钟计算的进程运行纳秒数。
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - start)
              .count());
      const auto telemetry_frames = telemetry.encode(  // 本周期状态及可选告警帧集合。
          snapshot.network, counters, uptime_ns,
          demo_config.engine.filter.reference_node_id, next_output_sequence,
          demo_config.online.max_payload_size,
          demo_config.inertial
              ? protocol::AlgorithmMode::kImuUwb15State
              : protocol::AlgorithmMode::kUwbOnlyPlanar);
      const std::uint64_t output_time_ns =  // 同批输出写入事件日志的系统墙上时间。
          apps::system_time_ns();
      // 算法快照和遥测使用同一输出时刻，并同时写入UDP与任务日志。
      for (const auto& frame : frames) {  // frame为待发送并可选写日志的算法输出整帧。
        output.send_to(demo_config.online.output_address,
                       demo_config.online.output_port, frame.bytes);
        if (event_log) {
          event_log->append({protocol::EventLogDirection::Output,
                             output_time_ns, frame.bytes});
        }
        count_output(frame.message_type, stats);
      }
      for (const auto& frame : telemetry_frames) {  // frame为待发送并可选写日志的状态/告警整帧。
        output.send_to(demo_config.online.output_address,
                       demo_config.online.output_port, frame.bytes);
        if (event_log) {
          event_log->append({protocol::EventLogDirection::Output,
                             output_time_ns, frame.bytes});
        }
        count_output(frame.message_type, stats);
      }
      // 处理过慢时跳过已经错过的周期，不突发补发多份同一状态快照。
      do {
        next_output += output_period_ticks;
      } while (next_output <= now);
    }
  }

  // 阶段4：正常退出前刷新日志并输出机器可解析SUMMARY，供自动测试核验计数。
  if (event_log) {
    event_log->flush();
  }
  std::cout << "SUMMARY status=OK"
            << " received=" << stats.received
            << " range=" << stats.range_frames
            << " imu=" << stats.imu_frames
            << " propagated_imu=" << stats.propagated_imu
            << " accepted_ranges=" << stats.accepted_ranges
            << " rejected_ranges=" << stats.rejected_ranges
            << " protocol_rejected=" << stats.protocol_rejected
            << " type_rejected=" << stats.type_rejected
            << " api_rejected=" << stats.api_rejected
            << " published_localization=" << stats.published_localization
            << " published_pose2d=" << stats.published_pose2d
            << " published_network=" << stats.published_network
            << " published_observation=" << stats.published_observation
            << " published_status=" << stats.published_status
            << " published_alert=" << stats.published_alert
            << " log="
            << (event_log ? event_log_path.string() : std::string("disabled"))
            << '\n';
  return 0;
}

}  // namespace

/** @param argc 进程（命令行）参数数量；@param argv 进程参数字符串数组。 */
int main(int argc, char** argv) {
  try {
    const auto arguments = parse_arguments(argc, argv);  // 供run使用的已解析在线参数。
    std::signal(SIGINT, handle_signal);
#if defined(SIGTERM)
    std::signal(SIGTERM, handle_signal);
#endif
    return run(arguments);
  } catch (const std::exception& error) {  // error为入口统一转成错误摘要的标准异常。
    std::cerr << "ERROR " << error.what() << '\n';
    std::cerr << "SUMMARY status=ERROR\n";
    return 2;
  }
}

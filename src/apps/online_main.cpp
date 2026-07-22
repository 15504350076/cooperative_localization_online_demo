// 模块职责：运行临时UDP在线闭环，接收IMU/测距帧、调用算法、发布GCS结果并记录事件日志。
// 模块边界：仅用于无ROS 2阶段和联调冒烟测试；AIBrainBox正式部署由上交ROS 2适配节点
// 直接调用C ABI，通信路由和时间同步仍由上交负责。
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

volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int) { stop_requested = 1; }

struct Arguments {
  std::filesystem::path config_path{"config/demo.ini"};
  std::uint64_t duration_ms{};
};

std::uint64_t parse_unsigned(const std::string& text, const char* name) {
  if (text.empty()) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  for (const char character : text) {
    if (character < '0' || character > '9') {
      throw std::invalid_argument(std::string(name) + " must be an integer");
    }
  }
  std::size_t consumed = 0U;
  unsigned long long value = 0U;
  try {
    value = std::stoull(text, &consumed, 10);
  } catch (const std::exception&) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  if (consumed != text.size()) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  return static_cast<std::uint64_t>(value);
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments result{};
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
  std::uint64_t received{};
  std::uint64_t range_frames{};
  std::uint64_t imu_frames{};
  std::uint64_t propagated_imu{};
  std::uint64_t accepted_ranges{};
  std::uint64_t rejected_ranges{};
  std::uint64_t protocol_rejected{};
  std::uint64_t type_rejected{};
  std::uint64_t api_rejected{};
  std::uint64_t published_localization{};
  std::uint64_t published_network{};
  std::uint64_t published_observation{};
  std::uint64_t published_status{};
  std::uint64_t published_alert{};
};

void prepare_log_path(const std::filesystem::path& path) {
  if (path.empty()) {
    throw std::invalid_argument("event log path is empty");
  }
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
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
    case MessageType::kAlert:
      ++stats.published_alert;
      return;
    case MessageType::kRange:
    case MessageType::kImu:
      throw std::runtime_error("unexpected online output message type");
  }
  throw std::runtime_error("unknown online output message type");
}

int run(const Arguments& arguments) {
  using namespace zju::coop;
  // 阶段1：先加载并完整验证配置，再创建算法、输入套接字、输出套接字和可选日志。
  const auto demo_config = config::load_ini_config(arguments.config_path);
  apps::AlgorithmSession algorithm(demo_config);
  net::UdpSocket input;
  input.bind(demo_config.online.input_bind_address,
             demo_config.online.input_port);
  input.set_receive_timeout(std::chrono::milliseconds(10));
  net::UdpSocket output;

  std::unique_ptr<protocol::EventLogWriter> event_log;
  std::filesystem::path event_log_path;
  if (demo_config.online.event_log_enabled) {
    event_log_path = demo_config.online.event_log_path;
    prepare_log_path(event_log_path);
    event_log = std::make_unique<protocol::EventLogWriter>(
        event_log_path, demo_config.online.max_log_record_size,
        demo_config.online.max_payload_size);
  }

  Statistics stats{};
  apps::TelemetryEncoder telemetry;
  std::uint64_t next_output_sequence = 1U;
  const auto start = std::chrono::steady_clock::now();
  const auto deadline =
      arguments.duration_ms == 0U
          ? std::chrono::steady_clock::time_point::max()
          : start + std::chrono::milliseconds(arguments.duration_ms);
  const std::chrono::duration<double> output_period(
      1.0 / demo_config.online.output_rate_hz);
  const auto output_period_ticks = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(output_period);
  if (output_period_ticks <= std::chrono::steady_clock::duration::zero()) {
    throw std::invalid_argument("configured output rate is too high");
  }
  auto next_output = start;

  while (stop_requested == 0 && std::chrono::steady_clock::now() < deadline) {
    // 阶段2：按数据报边界解码，只把通过协议和载荷校验的输入送入C ABI。
    const auto received = input.receive();
    if (received.status == net::ReceiveStatus::kData) {
      ++stats.received;
      const std::uint64_t receive_timestamp_ns = apps::system_time_ns();
      const auto decoded = protocol::decode_frame(
          received.datagram.bytes, demo_config.online.max_payload_size);
      if (!decoded.ok()) {
        ++stats.protocol_rejected;
      } else if (decoded.value.header.message_type ==
                 protocol::MessageType::kRange) {
        const auto range = protocol::decode_range_payload(decoded.value.payload);
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
          const auto processing = algorithm.push_range(
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
        const auto imu = protocol::decode_imu_payload(decoded.value.payload);
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
          const auto processing = algorithm.push_imu(
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
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_output) {
      const std::uint64_t step_time_ns = apps::system_time_ns();
      const auto snapshot = algorithm.step(step_time_ns);
      const auto frames = apps::encode_snapshot(
          snapshot, demo_config.engine.filter.reference_node_id,
          next_output_sequence, demo_config.online.max_payload_size);
      apps::TelemetryCounters counters{};
      counters.accepted_ranges = stats.accepted_ranges;
      counters.rejected_ranges = stats.rejected_ranges;
      counters.protocol_errors = stats.protocol_rejected;
      const auto uptime_ns = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(now - start)
              .count());
      const auto telemetry_frames = telemetry.encode(
          snapshot.network, counters, uptime_ns,
          demo_config.engine.filter.reference_node_id, next_output_sequence,
          demo_config.online.max_payload_size,
          demo_config.inertial
              ? protocol::AlgorithmMode::kImuUwb15State
              : protocol::AlgorithmMode::kUwbOnlyPlanar);
      const std::uint64_t output_time_ns = apps::system_time_ns();
      // 算法快照和遥测使用同一输出时刻，并同时写入UDP与任务日志。
      for (const auto& frame : frames) {
        output.send_to(demo_config.online.output_address,
                       demo_config.online.output_port, frame.bytes);
        if (event_log) {
          event_log->append({protocol::EventLogDirection::Output,
                             output_time_ns, frame.bytes});
        }
        count_output(frame.message_type, stats);
      }
      for (const auto& frame : telemetry_frames) {
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

int main(int argc, char** argv) {
  try {
    const auto arguments = parse_arguments(argc, argv);
    std::signal(SIGINT, handle_signal);
#if defined(SIGTERM)
    std::signal(SIGTERM, handle_signal);
#endif
    return run(arguments);
  } catch (const std::exception& error) {
    std::cerr << "ERROR " << error.what() << '\n';
    std::cerr << "SUMMARY status=ERROR\n";
    return 2;
  }
}

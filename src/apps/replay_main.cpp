// 模块职责：顺序读取ZJLG历史输入，用当前算法重新生成定位、网络、观测状态和告警。
// 原日志中的输出记录只用于审计并跳过，避免旧结果再次作为输入；可按原速、倍速或最快速度回放。
#include "apps/app_support.hpp"
#include "config/ini_config.hpp"
#include "net/udp_socket.hpp"
#include "protocol/event_log.hpp"
#include "protocol/wire_protocol.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

enum class OutputMode {
  kStream,
  kFinal,
  kNone,
};

struct Arguments {
  std::filesystem::path config_path{"config/demo.ini"};
  std::filesystem::path log_path;
  double speed{1.0};
  OutputMode output_mode{OutputMode::kFinal};
};

double parse_speed(const std::string& text) {
  std::size_t consumed = 0U;
  double result = 0.0;
  try {
    result = std::stod(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument("--speed must be numeric");
  }
  if (consumed != text.size() || !std::isfinite(result) || result < 0.0) {
    throw std::invalid_argument("--speed must be finite and non-negative");
  }
  return result;
}

OutputMode parse_output_mode(const std::string& text) {
  if (text == "stream") {
    return OutputMode::kStream;
  }
  if (text == "final") {
    return OutputMode::kFinal;
  }
  if (text == "none") {
    return OutputMode::kNone;
  }
  throw std::invalid_argument("--output-mode must be stream, final, or none");
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments result{};
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help") {
      std::cout << "Usage: zju_coop_replay --log PATH [--config PATH] "
                   "[--speed SCALE] [--output-mode stream|final|none]\n";
      std::exit(0);
    }
    if (argument == "--config" || argument == "--log" ||
        argument == "--speed" || argument == "--output-mode") {
      if (index + 1 >= argc) {
        throw std::invalid_argument(argument + " requires a value");
      }
      const std::string value = argv[++index];
      if (argument == "--config") {
        result.config_path = value;
      } else if (argument == "--log") {
        result.log_path = value;
      } else if (argument == "--speed") {
        result.speed = parse_speed(value);
      } else {
        result.output_mode = parse_output_mode(value);
      }
      continue;
    }
    throw std::invalid_argument("unknown argument: " + argument);
  }
  if (result.log_path.empty()) {
    throw std::invalid_argument("--log is required");
  }
  return result;
}

class ReplayPacer {
 public:
  explicit ReplayPacer(double speed) : speed_(speed) {}

  void wait(std::uint64_t receive_timestamp_ns) {
    // speed=0关闭等待用于快速回归；其他值按“日志接收时间差/speed”缩放。
    // 不使用帧内测量时间，因为回放还需要保留真实到达延迟和乱序特征。
    if (speed_ == 0.0) {
      return;
    }
    if (!started_) {
      started_ = true;
      first_receive_timestamp_ns_ = receive_timestamp_ns;
      start_ = std::chrono::steady_clock::now();
      return;
    }
    if (receive_timestamp_ns <= first_receive_timestamp_ns_) {
      return;
    }
    const auto elapsed_ns =
        receive_timestamp_ns - first_receive_timestamp_ns_;
    const std::chrono::duration<double> replay_elapsed(
        static_cast<double>(elapsed_ns) / 1.0e9 / speed_);
    std::this_thread::sleep_until(start_ + replay_elapsed);
  }

 private:
  double speed_{};
  bool started_{};
  std::uint64_t first_receive_timestamp_ns_{};
  std::chrono::steady_clock::time_point start_{};
};

struct Statistics {
  std::uint64_t records{};
  std::uint64_t input_ranges{};
  std::uint64_t input_imus{};
  std::uint64_t output_records_skipped{};
  std::uint64_t input_types_skipped{};
  std::uint64_t api_rejected{};
  std::uint64_t accepted_ranges{};
  std::uint64_t rejected_ranges{};
  std::uint64_t protocol_errors{};
  std::uint64_t emitted{};
};

void emit_snapshot(zju::coop::apps::AlgorithmSession& algorithm,
                   zju::coop::apps::TelemetryEncoder& telemetry,
                   const zju::coop::config::DemoConfig& config,
                   zju::coop::net::UdpSocket& output,
                   std::uint64_t timestamp_ns,
                   std::uint64_t uptime_ns,
                   std::uint64_t& next_sequence,
                   Statistics& stats) {
  // 算法快照和状态/告警从同一时间生成，保持与在线入口一致的输出口径。
  const auto snapshot = algorithm.step(timestamp_ns);
  const auto frames = zju::coop::apps::encode_snapshot(
      snapshot, config.engine.filter.reference_node_id, next_sequence,
      config.online.max_payload_size);
  for (const auto& frame : frames) {
    output.send_to(config.online.output_address, config.online.output_port,
                   frame.bytes);
    ++stats.emitted;
  }
  zju::coop::apps::TelemetryCounters counters{};
  counters.accepted_ranges = stats.accepted_ranges;
  counters.rejected_ranges = stats.rejected_ranges;
  counters.protocol_errors = stats.protocol_errors;
  const auto telemetry_frames = telemetry.encode(
      snapshot.network, counters, uptime_ns,
      config.engine.filter.reference_node_id, next_sequence,
      config.online.max_payload_size,
      config.inertial
          ? zju::coop::protocol::AlgorithmMode::kImuUwb15State
          : zju::coop::protocol::AlgorithmMode::kUwbOnlyPlanar);
  for (const auto& frame : telemetry_frames) {
    output.send_to(config.online.output_address, config.online.output_port,
                   frame.bytes);
    ++stats.emitted;
  }
}

int run(const Arguments& arguments) {
  using namespace zju::coop;
  // 阶段1：使用同一INI重新创建算法会话，参数变化可通过回放复现实验。
  const auto demo_config = config::load_ini_config(arguments.config_path);
  apps::AlgorithmSession algorithm(demo_config);
  protocol::EventLogReader reader(
      arguments.log_path, demo_config.online.max_log_record_size,
      demo_config.online.max_payload_size);
  net::UdpSocket output;
  apps::TelemetryEncoder telemetry;
  ReplayPacer pacer(arguments.speed);
  Statistics stats{};
  std::uint64_t next_output_sequence = 1U;
  std::uint64_t first_record_timestamp_ns = 0U;
  std::uint64_t last_record_timestamp_ns = 0U;
  bool has_record = false;
  std::uint64_t next_stream_output_ns = 0U;
  const auto period_as_double =
      1.0e9 / demo_config.online.output_rate_hz;
  const std::uint64_t output_period_ns = static_cast<std::uint64_t>(
      period_as_double < 1.0 ? 1.0 : period_as_double);

  while (true) {
    // 阶段2：严格顺序读取日志，回放节奏只由本机接收时间控制。
    const auto read = reader.next();
    if (read.status == protocol::EventLogReadStatus::kEnd) {
      break;
    }
    ++stats.records;
    const auto& record = read.record;
    pacer.wait(record.receive_timestamp_ns);
    if (!has_record) {
      has_record = true;
      first_record_timestamp_ns = record.receive_timestamp_ns;
      next_stream_output_ns = record.receive_timestamp_ns;
    }
    if (record.receive_timestamp_ns > last_record_timestamp_ns) {
      last_record_timestamp_ns = record.receive_timestamp_ns;
    }
    // 只重放Input记录；历史Output不能反馈进入新算法形成闭环污染。
    if (record.direction == protocol::EventLogDirection::Output) {
      ++stats.output_records_skipped;
    } else {
      const auto decoded = protocol::decode_frame(
          record.frame, demo_config.online.max_payload_size);
      if (!decoded.ok()) {
        ++stats.protocol_errors;
        ++stats.input_types_skipped;
      } else if (decoded.value.header.message_type ==
                 protocol::MessageType::kRange) {
        const auto range =
            protocol::decode_range_payload(decoded.value.payload);
        if (!range.ok()) {
          ++stats.protocol_errors;
          ++stats.input_types_skipped;
        } else {
          const auto processing = algorithm.push_range(
              decoded.value, range.value, record.receive_timestamp_ns);
          ++stats.input_ranges;
          if (processing.disposition == ZJU_COOP_PROCESSING_PROCESSED) {
            ++stats.accepted_ranges;
          } else {
            ++stats.rejected_ranges;
            ++stats.api_rejected;
          }
        }
      } else if (decoded.value.header.message_type ==
                     protocol::MessageType::kImu &&
                 demo_config.inertial) {
        const auto imu = protocol::decode_imu_payload(decoded.value.payload);
        if (!imu.ok()) {
          ++stats.protocol_errors;
          ++stats.input_types_skipped;
        } else {
          static_cast<void>(algorithm.push_imu(
              decoded.value, imu.value, record.receive_timestamp_ns));
          ++stats.input_imus;
        }
      } else {
        ++stats.input_types_skipped;
      }
    }

    // 阶段3：stream按配置频率连续输出，final只在全部输入处理完后输出一次。
    if (arguments.output_mode == OutputMode::kStream) {
      if (record.receive_timestamp_ns >= next_stream_output_ns) {
        emit_snapshot(
            algorithm, telemetry, demo_config, output,
            record.receive_timestamp_ns,
            apps::elapsed_ns(record.receive_timestamp_ns,
                             first_record_timestamp_ns),
            next_output_sequence, stats);
        // 日志记录可能稀疏或跳时，越过的输出周期直接跳过而不制造重复快照。
        do {
          next_stream_output_ns += output_period_ns;
        } while (next_stream_output_ns <= record.receive_timestamp_ns);
      }
    }
  }

  if (has_record && arguments.output_mode == OutputMode::kFinal) {
    emit_snapshot(
        algorithm, telemetry, demo_config, output, last_record_timestamp_ns,
        apps::elapsed_ns(last_record_timestamp_ns,
                         first_record_timestamp_ns),
        next_output_sequence, stats);
  }

  std::cout << "SUMMARY status=OK"
            << " records=" << stats.records
            << " input_ranges=" << stats.input_ranges
            << " input_imus=" << stats.input_imus
            << " output_records_skipped=" << stats.output_records_skipped
            << " input_types_skipped=" << stats.input_types_skipped
            << " api_rejected=" << stats.api_rejected
            << " accepted_ranges=" << stats.accepted_ranges
            << " rejected_ranges=" << stats.rejected_ranges
            << " protocol_errors=" << stats.protocol_errors
            << " emitted=" << stats.emitted
            << " speed=" << arguments.speed << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run(parse_arguments(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "ERROR " << error.what() << '\n';
    std::cerr << "SUMMARY status=ERROR\n";
    return 2;
  }
}

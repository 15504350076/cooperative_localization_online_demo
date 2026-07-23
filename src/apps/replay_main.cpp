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
  std::filesystem::path config_path{"config/demo.ini"};  ///< 重建算法会话使用的INI路径。
  std::filesystem::path log_path;  ///< 必填的ZJLG事件日志路径。
  double speed{1.0};  ///< 日志到达间隔的回放倍率，0表示不等待。
  OutputMode output_mode{OutputMode::kFinal};  ///< 逐周期、仅最终或禁止UDP输出。
};

/** @param text --speed选项的数值文本。 */
double parse_speed(const std::string& text) {
  std::size_t consumed = 0U;  // stod回填的已消费字符数。
  double result = 0.0;        // 解析得到的有限非负回放倍率。
  try {
    result = std::stod(text, &consumed);
  } catch (const std::exception&) {  // 具体转换异常不外泄，统一报告speed格式错误。
    throw std::invalid_argument("--speed must be numeric");
  }
  if (consumed != text.size() || !std::isfinite(result) || result < 0.0) {
    throw std::invalid_argument("--speed must be finite and non-negative");
  }
  return result;
}

/** @param text --output-mode选项的枚举文本。 */
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

/** @param argc 命令行参数个数；@param argv 命令行参数字符串数组。 */
Arguments parse_arguments(int argc, char** argv) {
  Arguments result{};  // 逐项覆盖默认值的解析结果。
  // index定位当前argv选项；argument保存选项名，value在需取值时保存下一参数。
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
  /** @param speed 日志接收时间差的缩放倍率，0禁用等待。 */
  explicit ReplayPacer(double speed) : speed_(speed) {}

  /** @param receive_timestamp_ns 当前日志记录保存的统一接收时间。 */
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
    const auto elapsed_ns =  // 当前记录相对首条记录的非负统一时间差。
        receive_timestamp_ns - first_receive_timestamp_ns_;
    const std::chrono::duration<double> replay_elapsed(  // 按倍率缩放后的本地等待时长。
        static_cast<double>(elapsed_ns) / 1.0e9 / speed_);
    std::this_thread::sleep_until(start_ + replay_elapsed);
  }

 private:
  double speed_{};  ///< 日志接收时间差的回放倍率。
  bool started_{};  ///< 是否已用首条记录建立两种时钟的对应起点。
  std::uint64_t first_receive_timestamp_ns_{};  ///< 首条日志记录的统一接收时间。
  std::chrono::steady_clock::time_point start_{};  ///< 首条记录对应的本地单调调度时刻。
};

struct Statistics {
  std::uint64_t records{};                 ///< 从日志顺序读出的全部记录数。
  std::uint64_t input_ranges{};            ///< 成功解码并送入C ABI的输入测距数。
  std::uint64_t input_imus{};              ///< 成功解码并送入C ABI的输入IMU数。
  std::uint64_t output_records_skipped{};  ///< 为防闭环污染而跳过的历史输出记录数。
  std::uint64_t input_types_skipped{};     ///< 解码失败或当前模式不处理的输入记录数。
  std::uint64_t api_rejected{};            ///< C ABI未处置为Processed的测距数。
  std::uint64_t accepted_ranges{};         ///< 引擎处置为Processed的测距数。
  std::uint64_t rejected_ranges{};         ///< 引擎未处置为Processed的测距数。
  std::uint64_t protocol_errors{};         ///< 帧或载荷解码失败数。
  std::uint64_t emitted{};                 ///< 本次回放实际发送的全部输出帧数。
};

/**
 * @param algorithm 待推进并取快照的算法会话；@param telemetry 跨快照告警编码器。
 * @param config 输出端点及算法模式配置；@param output UDP输出套接字。
 * @param timestamp_ns 传入step的日志统一时间；@param uptime_ns 相对首记录的运行时长。
 * @param next_sequence 跨帧递增的输出序号；@param stats 待累加发送数的回放统计。
 */
void emit_snapshot(zju::coop::apps::AlgorithmSession& algorithm,
                   zju::coop::apps::TelemetryEncoder& telemetry,
                   const zju::coop::config::DemoConfig& config,
                   zju::coop::net::UdpSocket& output,
                   std::uint64_t timestamp_ns,
                   std::uint64_t uptime_ns,
                   std::uint64_t& next_sequence,
                   Statistics& stats) {
  // 算法快照和状态/告警从同一时间生成，保持与在线入口一致的输出口径。
  const auto snapshot = algorithm.step(timestamp_ns);  // 指定日志时间的原子算法快照。
  const auto frames = zju::coop::apps::encode_snapshot(  // 定位、网络及观测输出整帧。
      snapshot, config.engine.filter.reference_node_id, next_sequence,
      config.online.max_payload_size);
  for (const auto& frame : frames) {  // frame为待发送的算法快照整帧。
    output.send_to(config.online.output_address, config.online.output_port,
                   frame.bytes);
    ++stats.emitted;
  }
  zju::coop::apps::TelemetryCounters counters{};  // 状态帧使用的累计测距/协议计数。
  counters.accepted_ranges = stats.accepted_ranges;
  counters.rejected_ranges = stats.rejected_ranges;
  counters.protocol_errors = stats.protocol_errors;
  const auto telemetry_frames = telemetry.encode(  // 同时刻状态及可选告警整帧。
      snapshot.network, counters, uptime_ns,
      config.engine.filter.reference_node_id, next_sequence,
      config.online.max_payload_size,
      config.inertial
          ? zju::coop::protocol::AlgorithmMode::kImuUwb15State
          : zju::coop::protocol::AlgorithmMode::kUwbOnlyPlanar);
  for (const auto& frame : telemetry_frames) {  // frame为待发送的状态或告警整帧。
    output.send_to(config.online.output_address, config.online.output_port,
                   frame.bytes);
    ++stats.emitted;
  }
}

/** @param arguments 已完成合法性解析的回放运行参数。 */
int run(const Arguments& arguments) {
  using namespace zju::coop;
  // 阶段1：使用同一INI重新创建算法会话，参数变化可通过回放复现实验。
  const auto demo_config =  // 同时驱动算法、日志边界和输出端点的已验证配置。
      config::load_ini_config(arguments.config_path);
  apps::AlgorithmSession algorithm(demo_config);  // 按当前配置重建的C ABI算法会话。
  protocol::EventLogReader reader(  // 顺序读取并校验ZJLG记录的日志会话。
      arguments.log_path, demo_config.online.max_log_record_size,
      demo_config.online.max_payload_size);
  net::UdpSocket output;  // 向配置GCS端点发送新生成结果的套接字。
  apps::TelemetryEncoder telemetry;  // 跨输出周期维护告警生命周期。
  ReplayPacer pacer(arguments.speed);  // 将日志时间差映射为本机单调等待。
  Statistics stats{};  // 本次日志遍历与输出的累计计数。
  std::uint64_t next_output_sequence = 1U;  // 所有新输出类型共享的下一发送序号。
  std::uint64_t first_record_timestamp_ns = 0U;  // 首条记录的统一接收时间。
  std::uint64_t last_record_timestamp_ns = 0U;   // 已读记录中的最大统一接收时间。
  bool has_record = false;  // 日志是否至少产生过一条记录。
  std::uint64_t next_stream_output_ns = 0U;  // stream模式下一周期的日志时间截止点。
  const auto period_as_double =  // 配置输出频率换算的浮点纳秒周期。
      1.0e9 / demo_config.online.output_rate_hz;
  const std::uint64_t output_period_ns = static_cast<std::uint64_t>(  // 至少1 ns的整数日志时间周期。
      period_as_double < 1.0 ? 1.0 : period_as_double);

  while (true) {
    // 阶段2：严格顺序读取日志，回放节奏只由本机接收时间控制。
    const auto read = reader.next();  // 下一条完整日志记录或文件结束状态。
    if (read.status == protocol::EventLogReadStatus::kEnd) {
      break;
    }
    ++stats.records;
    const auto& record = read.record;  // 当前顺序处理的日志记录。
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
      const auto decoded = protocol::decode_frame(  // 当前输入记录中协议帧的解码结果。
          record.frame, demo_config.online.max_payload_size);
      if (!decoded.ok()) {
        ++stats.protocol_errors;
        ++stats.input_types_skipped;
      } else if (decoded.value.header.message_type ==
                 protocol::MessageType::kRange) {
        const auto range =  // 当前测距输入载荷的解码结果。
            protocol::decode_range_payload(decoded.value.payload);
        if (!range.ok()) {
          ++stats.protocol_errors;
          ++stats.input_types_skipped;
        } else {
          const auto processing = algorithm.push_range(  // C ABI返回的测距入口处置。
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
        const auto imu =  // 当前瞬时IMU输入载荷的解码结果。
            protocol::decode_imu_payload(decoded.value.payload);
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

/** @param argc 进程参数数量；@param argv 进程参数字符串数组。 */
int main(int argc, char** argv) {
  try {
    return run(parse_arguments(argc, argv));
  } catch (const std::exception& error) {  // error为入口统一转成错误摘要的标准异常。
    std::cerr << "ERROR " << error.what() << '\n';
    std::cerr << "SUMMARY status=ERROR\n";
    return 2;
  }
}

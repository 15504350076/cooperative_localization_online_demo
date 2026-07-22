// 模块职责：定义ZJLG顺序事件日志，记录算法输入、输出及本机接收时刻供任务后回放。
// 日志保存完整ZJCL帧而非原始图像/点云；读取时重新执行协议校验，损坏记录不能进入算法。
#pragma once

#include "protocol/wire_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace zju::coop::protocol {

inline constexpr std::size_t kEventLogHeaderSize = 8U;
inline constexpr std::size_t kDefaultMaxEventLogRecordSize =
    kWireHeaderSize + kDefaultMaxPayloadSize;

enum class EventLogError {
  kNone,
  kInvalidConfiguration,
  kIoFailure,
  kTruncatedFileHeader,
  kBadMagic,
  kUnsupportedVersion,
  kInvalidHeaderSize,
  kTruncatedRecordLength,
  kRecordTooLarge,
  kTruncatedRecordMetadata,
  kInvalidDirection,
  kInvalidReserved,
  kInvalidReceiveTimestamp,
  kTruncatedRecord,
  kInvalidFrame,
};

class EventLogException final : public std::runtime_error {
 public:
  EventLogException(EventLogError code, std::string message,
                    ProtocolError protocol_error = ProtocolError::kNone);

  [[nodiscard]] EventLogError code() const noexcept;
  [[nodiscard]] ProtocolError protocol_error() const noexcept;

 private:
  EventLogError code_;
  ProtocolError protocol_error_;
};

enum class EventLogReadStatus {
  kRecord,
  kEnd,
};

enum class EventLogDirection : std::uint8_t {
  Input = 1U,
  Output = 2U,
};

/** 单条日志记录；receive_timestamp_ns用于回放节奏，不替代帧内测量时间戳。 */
struct EventLogRecord {
  EventLogDirection direction{EventLogDirection::Input};
  std::uint64_t receive_timestamp_ns{};
  std::vector<std::uint8_t> frame;
};

struct EventLogReadResult {
  EventLogReadStatus status{EventLogReadStatus::kEnd};
  EventLogRecord record;
};

/** 追加式日志写入器；每条记录写入前验证内部帧。 */
class EventLogWriter {
 public:
  explicit EventLogWriter(
      const std::filesystem::path& path,
      std::size_t max_record_size = kDefaultMaxEventLogRecordSize,
      std::size_t max_payload_size = kDefaultMaxPayloadSize);

  EventLogWriter(const EventLogWriter&) = delete;
  EventLogWriter& operator=(const EventLogWriter&) = delete;

  void append(const EventLogRecord& record);
  void flush();

 private:
  std::ofstream output_;
  std::size_t max_record_size_{};
  std::size_t max_payload_size_{};
};

/** 严格顺序读取器；截断、超长、保留字段异常和CRC错误均显式失败。 */
class EventLogReader {
 public:
  explicit EventLogReader(
      const std::filesystem::path& path,
      std::size_t max_record_size = kDefaultMaxEventLogRecordSize,
      std::size_t max_payload_size = kDefaultMaxPayloadSize);

  EventLogReader(const EventLogReader&) = delete;
  EventLogReader& operator=(const EventLogReader&) = delete;

  [[nodiscard]] EventLogReadResult next();

 private:
  std::ifstream input_;
  std::size_t max_record_size_{};
  std::size_t max_payload_size_{};
  bool ended_{};
};

}  // namespace zju::coop::protocol

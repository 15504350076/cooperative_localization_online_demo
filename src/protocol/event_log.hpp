// ZJLG 事件日志读写接口：同时记录算法输入和输出，供任务后确定性回放。
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

struct EventLogRecord {
  EventLogDirection direction{EventLogDirection::Input};
  std::uint64_t receive_timestamp_ns{};
  std::vector<std::uint8_t> frame;
};

struct EventLogReadResult {
  EventLogReadStatus status{EventLogReadStatus::kEnd};
  EventLogRecord record;
};

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

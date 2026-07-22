// 模块职责：验证ZJLG输入/输出记录往返、边界EOF、截断、超长记录和内部帧损坏检测。
#include "protocol/event_log.hpp"
#include "protocol/wire_protocol.hpp"
#include "test_support.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {

using zju::coop::protocol::EventLogError;
using zju::coop::protocol::EventLogDirection;
using zju::coop::protocol::EventLogException;
using zju::coop::protocol::EventLogReadStatus;
using zju::coop::protocol::EventLogRecord;
using zju::coop::protocol::EventLogReader;
using zju::coop::protocol::EventLogWriter;
using zju::coop::protocol::Frame;
using zju::coop::protocol::MessageType;
using zju::coop::protocol::ProtocolError;
using zju::coop::protocol::AlertPayload;
using zju::coop::protocol::encode_frame;
using zju::coop::protocol::encode_alert_payload;

class TemporaryFile {
 public:
  TemporaryFile() {
    static std::atomic<unsigned int> serial{0U};
    const auto ticks = std::chrono::steady_clock::now()
                           .time_since_epoch()
                           .count();
    path_ = std::filesystem::temp_directory_path() /
            ("zju_coop_event_log_" + std::to_string(ticks) + "_" +
             std::to_string(serial.fetch_add(1U)) + ".zjlg");
  }

  ~TemporaryFile() {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  [[nodiscard]] const std::filesystem::path& path() const noexcept {
    return path_;
  }

 private:
  std::filesystem::path path_;
};

Frame alert_frame(std::uint64_t sequence,
                  std::vector<std::uint8_t> payload) {
  AlertPayload alert{};
  alert.level = zju::coop::protocol::AlertLevel::kWarning;
  alert.lifecycle = zju::coop::protocol::AlertLifecycle::kActive;
  alert.reason_mask =
      payload.empty() || payload.front() == 0U ? 1U : payload.front();
  alert.first_timestamp_ns = sequence + 100U;
  alert.last_timestamp_ns = sequence + 100U;
  Frame frame{};
  frame.header.message_type = MessageType::kAlert;
  frame.header.sequence = sequence;
  frame.header.timestamp_ns = sequence + 100U;
  frame.header.source_node = 1U;
  frame.header.target_node = 2U;
  frame.payload = encode_alert_payload(alert);
  return frame;
}

std::vector<std::uint8_t> read_all(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(input),
                                   std::istreambuf_iterator<char>());
}

void write_all(const std::filesystem::path& path,
               const std::vector<std::uint8_t>& bytes) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  for (const std::uint8_t byte : bytes) {
    output.put(static_cast<char>(byte));
  }
}

std::vector<std::uint8_t> log_header() {
  return {'Z', 'J', 'L', 'G', 1U, 0U, 8U, 0U};
}

void append_u32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
  for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

void append_u64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    bytes.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

EventLogRecord log_record(EventLogDirection direction,
                          std::uint64_t receive_timestamp_ns,
                          std::vector<std::uint8_t> frame) {
  EventLogRecord record{};
  record.direction = direction;
  record.receive_timestamp_ns = receive_timestamp_ns;
  record.frame = std::move(frame);
  return record;
}

void append_record_metadata(std::vector<std::uint8_t>& bytes,
                            std::uint8_t direction,
                            std::uint64_t receive_timestamp_ns,
                            std::uint8_t reserved0 = 0U,
                            std::uint8_t reserved1 = 0U,
                            std::uint8_t reserved2 = 0U) {
  bytes.push_back(direction);
  bytes.push_back(reserved0);
  bytes.push_back(reserved1);
  bytes.push_back(reserved2);
  append_u64(bytes, receive_timestamp_ns);
}

template <typename Operation>
EventLogError capture_log_error(Operation operation,
                                ProtocolError* protocol_error = nullptr) {
  try {
    operation();
  } catch (const EventLogException& error) {
    if (protocol_error != nullptr) {
      *protocol_error = error.protocol_error();
    }
    return error.code();
  }
  return EventLogError::kNone;
}

}  // namespace

TEST_CASE(event_log_round_trips_records_with_frozen_file_header) {
  TemporaryFile file;
  const auto frame = encode_frame(alert_frame(7U, {0xAAU, 0xBBU}));
  constexpr std::uint64_t kInputTimestamp = 0x0102030405060708ULL;
  constexpr std::uint64_t kOutputTimestamp = 0x1112131415161718ULL;
  {
    EventLogWriter writer(file.path(), 128U);
    writer.append(
        log_record(EventLogDirection::Input, kInputTimestamp, frame));
    writer.append(
        log_record(EventLogDirection::Output, kOutputTimestamp, frame));
    writer.flush();
  }

  const auto bytes = read_all(file.path());
  EXPECT_EQ(bytes.size(), 8U + (16U + frame.size()) * 2U);
  EXPECT_EQ(bytes[0U], static_cast<std::uint8_t>('Z'));
  EXPECT_EQ(bytes[1U], static_cast<std::uint8_t>('J'));
  EXPECT_EQ(bytes[2U], static_cast<std::uint8_t>('L'));
  EXPECT_EQ(bytes[3U], static_cast<std::uint8_t>('G'));
  EXPECT_EQ(bytes[4U], 1U);
  EXPECT_EQ(bytes[5U], 0U);
  EXPECT_EQ(bytes[6U], 8U);
  EXPECT_EQ(bytes[7U], 0U);
  EXPECT_EQ(bytes[8U], static_cast<std::uint8_t>(frame.size()));
  EXPECT_EQ(bytes[9U], 0U);
  EXPECT_EQ(bytes[10U], 0U);
  EXPECT_EQ(bytes[11U], 0U);
  EXPECT_EQ(bytes[12U], 1U);
  EXPECT_EQ(bytes[13U], 0U);
  EXPECT_EQ(bytes[14U], 0U);
  EXPECT_EQ(bytes[15U], 0U);
  EXPECT_EQ(bytes[16U], 0x08U);
  EXPECT_EQ(bytes[17U], 0x07U);
  EXPECT_EQ(bytes[18U], 0x06U);
  EXPECT_EQ(bytes[19U], 0x05U);
  EXPECT_EQ(bytes[20U], 0x04U);
  EXPECT_EQ(bytes[21U], 0x03U);
  EXPECT_EQ(bytes[22U], 0x02U);
  EXPECT_EQ(bytes[23U], 0x01U);
  const std::size_t second_offset = 24U + frame.size();
  EXPECT_EQ(bytes[second_offset], static_cast<std::uint8_t>(frame.size()));
  EXPECT_EQ(bytes[second_offset + 4U], 2U);
  EXPECT_EQ(bytes[second_offset + 8U], 0x18U);
  EXPECT_EQ(bytes[second_offset + 15U], 0x11U);

  EventLogReader reader(file.path(), 128U);
  const auto first_read = reader.next();
  EXPECT_EQ(first_read.status, EventLogReadStatus::kRecord);
  EXPECT_EQ(first_read.record.direction, EventLogDirection::Input);
  EXPECT_EQ(first_read.record.receive_timestamp_ns, kInputTimestamp);
  EXPECT_EQ(first_read.record.frame, frame);
  const auto second_read = reader.next();
  EXPECT_EQ(second_read.status, EventLogReadStatus::kRecord);
  EXPECT_EQ(second_read.record.direction, EventLogDirection::Output);
  EXPECT_EQ(second_read.record.receive_timestamp_ns, kOutputTimestamp);
  EXPECT_EQ(second_read.record.frame, frame);
  EXPECT_EQ(reader.next().status, EventLogReadStatus::kEnd);
  EXPECT_EQ(reader.next().status, EventLogReadStatus::kEnd);
}

TEST_CASE(event_log_writer_validates_before_writing_any_record_bytes) {
  TemporaryFile file;
  auto corrupted = encode_frame(alert_frame(1U, {0x11U}));
  corrupted.back() ^= 0x80U;
  ProtocolError protocol_error = ProtocolError::kNone;
  {
    EventLogWriter writer(file.path(), 128U);
    const auto error = capture_log_error(
        [&writer, &corrupted]() {
          writer.append(
              log_record(EventLogDirection::Input, 1U, corrupted));
        },
        &protocol_error);
    EXPECT_EQ(error, EventLogError::kInvalidFrame);
    EXPECT_EQ(protocol_error, ProtocolError::kCrcMismatch);
    writer.flush();
  }
  EXPECT_EQ(read_all(file.path()), log_header());
}

TEST_CASE(event_log_writer_rejects_invalid_metadata_before_writing_record) {
  const auto frame = encode_frame(alert_frame(1U, {0x11U}));

  TemporaryFile invalid_direction_file;
  {
    EventLogWriter writer(invalid_direction_file.path(), 128U);
    EXPECT_EQ(capture_log_error([&]() {
                writer.append(log_record(
                    static_cast<EventLogDirection>(0xFFU), 1U, frame));
              }),
              EventLogError::kInvalidDirection);
    writer.flush();
  }
  EXPECT_EQ(read_all(invalid_direction_file.path()), log_header());

  TemporaryFile zero_timestamp_file;
  {
    EventLogWriter writer(zero_timestamp_file.path(), 128U);
    EXPECT_EQ(capture_log_error([&]() {
                writer.append(log_record(EventLogDirection::Output, 0U,
                                         frame));
              }),
              EventLogError::kInvalidReceiveTimestamp);
    writer.flush();
  }
  EXPECT_EQ(read_all(zero_timestamp_file.path()), log_header());
}

TEST_CASE(event_log_writer_and_reader_enforce_record_limit_at_boundary) {
  TemporaryFile valid_file;
  const auto frame = encode_frame(alert_frame(1U, {0x11U, 0x22U}));
  {
    EventLogWriter writer(valid_file.path(), frame.size());
    writer.append(log_record(EventLogDirection::Input, 1U, frame));
  }
  EventLogReader valid_reader(valid_file.path(), frame.size());
  EXPECT_EQ(valid_reader.next().status, EventLogReadStatus::kRecord);

  TemporaryFile writer_rejection;
  const auto writer_error = capture_log_error([&]() {
    EventLogWriter writer(writer_rejection.path(), frame.size() - 1U);
    writer.append(log_record(EventLogDirection::Input, 1U, frame));
  });
  EXPECT_EQ(writer_error, EventLogError::kRecordTooLarge);

  TemporaryFile reader_rejection;
  auto bytes = log_header();
  append_u32(bytes, 129U);
  write_all(reader_rejection.path(), bytes);
  EventLogReader reader(reader_rejection.path(), 128U);
  EXPECT_EQ(capture_log_error([&reader]() { static_cast<void>(reader.next()); }),
            EventLogError::kRecordTooLarge);
}

TEST_CASE(event_log_reader_rejects_invalid_or_truncated_record_metadata) {
  const auto frame = encode_frame(alert_frame(3U, {0x33U}));

  TemporaryFile truncated_metadata;
  auto bytes = log_header();
  append_u32(bytes, static_cast<std::uint32_t>(frame.size()));
  bytes.insert(bytes.end(), {1U, 0U, 0U, 0U, 1U});
  write_all(truncated_metadata.path(), bytes);
  EventLogReader truncated_reader(truncated_metadata.path());
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(truncated_reader.next());
            }),
            EventLogError::kTruncatedRecordMetadata);

  TemporaryFile invalid_direction;
  bytes = log_header();
  append_u32(bytes, static_cast<std::uint32_t>(frame.size()));
  append_record_metadata(bytes, 0xFFU, 1U);
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  write_all(invalid_direction.path(), bytes);
  EventLogReader direction_reader(invalid_direction.path());
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(direction_reader.next());
            }),
            EventLogError::kInvalidDirection);

  TemporaryFile nonzero_reserved;
  bytes = log_header();
  append_u32(bytes, static_cast<std::uint32_t>(frame.size()));
  append_record_metadata(bytes, 1U, 1U, 0U, 1U, 0U);
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  write_all(nonzero_reserved.path(), bytes);
  EventLogReader reserved_reader(nonzero_reserved.path());
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(reserved_reader.next());
            }),
            EventLogError::kInvalidReserved);

  TemporaryFile zero_timestamp;
  bytes = log_header();
  append_u32(bytes, static_cast<std::uint32_t>(frame.size()));
  append_record_metadata(bytes, 2U, 0U);
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  write_all(zero_timestamp.path(), bytes);
  EventLogReader timestamp_reader(zero_timestamp.path());
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(timestamp_reader.next());
            }),
            EventLogError::kInvalidReceiveTimestamp);
}

TEST_CASE(event_log_rejects_record_limit_larger_than_any_allowed_frame) {
  TemporaryFile file;
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(EventLogWriter(file.path(), 42U, 1U));
            }),
            EventLogError::kInvalidConfiguration);
}

TEST_CASE(event_log_rejects_bad_or_truncated_file_headers) {
  TemporaryFile truncated;
  write_all(truncated.path(), {'Z', 'J', 'L'});
  EXPECT_EQ(capture_log_error(
                [&]() { static_cast<void>(EventLogReader(truncated.path())); }),
            EventLogError::kTruncatedFileHeader);

  TemporaryFile bad_magic;
  auto bytes = log_header();
  bytes[0U] = 0U;
  write_all(bad_magic.path(), bytes);
  EXPECT_EQ(capture_log_error(
                [&]() { static_cast<void>(EventLogReader(bad_magic.path())); }),
            EventLogError::kBadMagic);

  TemporaryFile bad_version;
  bytes = log_header();
  bytes[4U] = 2U;
  write_all(bad_version.path(), bytes);
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(EventLogReader(bad_version.path()));
            }),
            EventLogError::kUnsupportedVersion);
}

TEST_CASE(event_log_rejects_partial_prefix_truncated_record_and_bad_frame) {
  TemporaryFile partial_prefix;
  auto bytes = log_header();
  bytes.push_back(1U);
  bytes.push_back(0U);
  write_all(partial_prefix.path(), bytes);
  EventLogReader prefix_reader(partial_prefix.path());
  EXPECT_EQ(capture_log_error(
                [&prefix_reader]() { static_cast<void>(prefix_reader.next()); }),
            EventLogError::kTruncatedRecordLength);

  TemporaryFile truncated_record;
  bytes = log_header();
  append_u32(bytes, 40U);
  append_record_metadata(bytes, 1U, 1U);
  bytes.insert(bytes.end(), 10U, 0U);
  write_all(truncated_record.path(), bytes);
  EventLogReader truncated_reader(truncated_record.path());
  EXPECT_EQ(capture_log_error([&truncated_reader]() {
              static_cast<void>(truncated_reader.next());
            }),
            EventLogError::kTruncatedRecord);

  TemporaryFile bad_crc;
  auto frame = encode_frame(alert_frame(9U, {0x01U}));
  frame.back() ^= 1U;
  bytes = log_header();
  append_u32(bytes, static_cast<std::uint32_t>(frame.size()));
  append_record_metadata(bytes, 2U, 1U);
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  write_all(bad_crc.path(), bytes);
  EventLogReader crc_reader(bad_crc.path());
  ProtocolError protocol_error = ProtocolError::kNone;
  EXPECT_EQ(capture_log_error(
                [&crc_reader]() { static_cast<void>(crc_reader.next()); },
                &protocol_error),
            EventLogError::kInvalidFrame);
  EXPECT_EQ(protocol_error, ProtocolError::kCrcMismatch);
}

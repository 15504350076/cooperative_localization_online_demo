// 模块职责：验证ZJLG输入/输出记录往返、边界EOF、截断、超长记录和内部帧损坏检测。
// C++初学者阅读提示：“往返”是先写临时文件再读回并比较；其余用例故意破坏长度或内容，
// 期望读取器明确报错。临时文件由测试创建并清理，不会使用现场任务日志。
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
    // serial：跨用例原子递增的临时文件后缀；ticks：当前单调时钟计数，与serial共同降低并发路径碰撞风险。
    static std::atomic<unsigned int> serial{0U};
    const auto ticks = std::chrono::steady_clock::now()
                           .time_since_epoch()
                           .count();
    path_ = std::filesystem::temp_directory_path() /
            ("zju_coop_event_log_" + std::to_string(ticks) + "_" +
             std::to_string(serial.fetch_add(1U)) + ".zjlg");
  }

  ~TemporaryFile() {
    // ignored：接收析构清理错误，避免测试辅助对象在栈展开期间抛异常。
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  [[nodiscard]] const std::filesystem::path& path() const noexcept {
    return path_;
  }

 private:
  // path_：当前对象独占的临时日志路径，从构造持续到析构时尽力删除。
  std::filesystem::path path_;
};

// sequence写入帧头和告警时间，payload按值接收以提供非空原因位输入；两者不逃逸本次构造。
Frame alert_frame(std::uint64_t sequence,
                  std::vector<std::uint8_t> payload) {
  // alert：待编码的Active告警载荷；frame：包裹该载荷的完整ZJCL帧。
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

// path：调用期间借用的日志路径；input：以二进制模式读取全文件的短生命周期流。
std::vector<std::uint8_t> read_all(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(input),
                                   std::istreambuf_iterator<char>());
}

// path是要截断写入的临时文件，bytes是原始故障注入缓冲；output在调用结束时关闭，byte逐项保留8位模式。
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

// bytes由调用方持有并追加，value按小端编码；shift依次选择0、8、16、24位字节。
void append_u32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
  for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
    bytes.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

// bytes由调用方持有并追加，value按小端编码；shift遍历64位值的八个字节。
void append_u64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    bytes.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

// direction/receive_timestamp_ns定义记录元数据，frame按值接收后移入结果以保留精确线协议字节。
EventLogRecord log_record(EventLogDirection direction,
                          std::uint64_t receive_timestamp_ns,
                          std::vector<std::uint8_t> frame) {
  // record：提交给写入器的完整日志记录。
  EventLogRecord record{};
  record.direction = direction;
  record.receive_timestamp_ns = receive_timestamp_ns;
  record.frame = std::move(frame);
  return record;
}

// bytes为故障注入目标；direction/time写入记录元数据，reserved0..2允许逐字节制造保留位错误。
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
// operation按值持有一次读写操作闭包；protocol_error可选接收内层帧错误，生命周期由调用方保证。
EventLogError capture_log_error(Operation operation,
                                ProtocolError* protocol_error = nullptr) {
  try {
    operation();
  } catch (const EventLogException& error) {
    // error：当前catch块借用的日志异常，用于复制日志错误码和可选协议错误码。
    if (protocol_error != nullptr) {
      *protocol_error = error.protocol_error();
    }
    return error.code();
  }
  return EventLogError::kNone;
}

}  // namespace

// 正常路径冻结ZJLG头、Input/Output方向、接收时刻和内部ZJCL帧的往返结果。
TEST_CASE(event_log_round_trips_records_with_frozen_file_header) {
  // file：本用例独占并自动清理的临时日志；frame：两条记录共用的已编码告警帧。
  TemporaryFile file;
  const auto frame = encode_frame(alert_frame(7U, {0xAAU, 0xBBU}));
  // kInputTimestamp/kOutputTimestamp：刻意使用逐字节可辨模式，冻结小端记录时间布局。
  constexpr std::uint64_t kInputTimestamp = 0x0102030405060708ULL;
  constexpr std::uint64_t kOutputTimestamp = 0x1112131415161718ULL;
  {
    // writer：作用域结束前写入并刷新两条记录，随后关闭文件供原始字节和读取器检查。
    EventLogWriter writer(file.path(), 128U);
    writer.append(
        log_record(EventLogDirection::Input, kInputTimestamp, frame));
    writer.append(
        log_record(EventLogDirection::Output, kOutputTimestamp, frame));
    writer.flush();
  }

  // bytes：落盘文件的完整原始缓冲；second_offset：第二条记录长度字段偏移。
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

  // reader：同一文件的边界读取器；first_read/second_read：期望依次恢复Input和Output记录。
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

// 原子写入组确认坏方向/帧不会留下半条记录，大小边界在分配和写盘前生效。
TEST_CASE(event_log_writer_validates_before_writing_any_record_bytes) {
  // file：应最终只保留文件头的临时日志；corrupted：末字节翻转后的CRC错误帧；protocol_error：捕获内层错误分类。
  TemporaryFile file;
  auto corrupted = encode_frame(alert_frame(1U, {0x11U}));
  corrupted.back() ^= 0x80U;
  ProtocolError protocol_error = ProtocolError::kNone;
  {
    // writer：错误写入后仍可刷新头部；lambda借用writer/corrupted且只在capture_log_error调用期间执行；error保存日志层分类。
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
  // frame：两个元数据失败场景共用的合法帧。
  const auto frame = encode_frame(alert_frame(1U, {0x11U}));

  // invalid_direction_file：非法方向写入目标；lambda借用同作用域writer/frame，期望不追加记录。
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

  // zero_timestamp_file：零接收时间写入目标；lambda同样只在writer有效期内同步执行。
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
  // valid_file/frame/valid_reader：记录大小恰等于上限时的成功写读对照。
  TemporaryFile valid_file;
  const auto frame = encode_frame(alert_frame(1U, {0x11U, 0x22U}));
  {
    EventLogWriter writer(valid_file.path(), frame.size());
    writer.append(log_record(EventLogDirection::Input, 1U, frame));
  }
  EventLogReader valid_reader(valid_file.path(), frame.size());
  EXPECT_EQ(valid_reader.next().status, EventLogReadStatus::kRecord);

  // writer_rejection/writer_error：上限比帧小1字节时的写入失败路径，lambda借用路径和frame。
  TemporaryFile writer_rejection;
  const auto writer_error = capture_log_error([&]() {
    EventLogWriter writer(writer_rejection.path(), frame.size() - 1U);
    writer.append(log_record(EventLogDirection::Input, 1U, frame));
  });
  EXPECT_EQ(writer_error, EventLogError::kRecordTooLarge);

  // reader_rejection/bytes/reader：声明长度129而读取上限128的手工文件与读取器。
  TemporaryFile reader_rejection;
  auto bytes = log_header();
  append_u32(bytes, 129U);
  write_all(reader_rejection.path(), bytes);
  EventLogReader reader(reader_rejection.path(), 128U);
  EXPECT_EQ(capture_log_error([&reader]() { static_cast<void>(reader.next()); }),
            EventLogError::kRecordTooLarge);
}

TEST_CASE(event_log_reader_rejects_invalid_or_truncated_record_metadata) {
  // frame：所有元数据变体后附的合法帧体；bytes：逐场景复用并重建的原始文件缓冲。
  const auto frame = encode_frame(alert_frame(3U, {0x33U}));

  // truncated_metadata/truncated_reader：只有5字节元数据的截断文件及其同步读取器lambda。
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

  // invalid_direction/direction_reader：方向字节0xFF的文件及读取器。
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

  // nonzero_reserved/reserved_reader：第二个保留字节非零的文件及读取器。
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

  // zero_timestamp/timestamp_reader：接收时间为零的文件及读取器。
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
  // file：构造非法“记录上限大于帧上限”写入器的临时路径，lambda只借用该路径。
  TemporaryFile file;
  EXPECT_EQ(capture_log_error([&]() {
              static_cast<void>(EventLogWriter(file.path(), 42U, 1U));
            }),
            EventLogError::kInvalidConfiguration);
}

// 恢复组区分记录边界正常EOF与头、长度、元数据、帧体中途截断。
TEST_CASE(event_log_rejects_bad_or_truncated_file_headers) {
  // truncated：仅含三个魔数字节的文件；构造读取器的lambda同步借用其路径。
  TemporaryFile truncated;
  write_all(truncated.path(), {'Z', 'J', 'L'});
  EXPECT_EQ(capture_log_error(
                [&]() { static_cast<void>(EventLogReader(truncated.path())); }),
            EventLogError::kTruncatedFileHeader);

  // bad_magic/bytes：首魔数字节清零的文件缓冲；bad_version：随后把版本改为2的文件。
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
  // partial_prefix/bytes/prefix_reader：文件头后只有两字节长度前缀的输入与读取器。
  TemporaryFile partial_prefix;
  auto bytes = log_header();
  bytes.push_back(1U);
  bytes.push_back(0U);
  write_all(partial_prefix.path(), bytes);
  EventLogReader prefix_reader(partial_prefix.path());
  EXPECT_EQ(capture_log_error(
                [&prefix_reader]() { static_cast<void>(prefix_reader.next()); }),
            EventLogError::kTruncatedRecordLength);

  // truncated_record/truncated_reader：声明40字节却只提供10字节帧体的输入与读取器。
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

  // bad_crc/frame/crc_reader：帧CRC被翻转的完整记录；protocol_error接收期望kCrcMismatch内层分类。
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

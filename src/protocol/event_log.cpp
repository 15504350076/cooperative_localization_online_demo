// 模块实现：ZJLG日志文件头、记录元数据和内部ZJCL帧的严格顺序读写。
// 关键原则：写入前验证、读取后复验，截断或损坏文件显式失败；接收时间只控制回放节奏，
// 算法仍使用内部帧的统一测量时间戳，保证在线与回放处理语义一致。
#include "protocol/event_log.hpp"

#include <array>
#include <limits>
#include <utility>

namespace zju::coop::protocol {
namespace {

// kLogMagic、主次版本和kRecordMetadataSize分别定义ZJLG文件签名、格式版本及方向/保留/接收时间元数据字节数。
constexpr std::array<std::uint8_t, 4U> kLogMagic{'Z', 'J', 'L', 'G'};
constexpr std::uint8_t kLogMajorVersion = 1U;
constexpr std::uint8_t kLogMinorVersion = 0U;
constexpr std::size_t kRecordMetadataSize = 12U;

// direction为待写入或从线序恢复的记录方向，仅Input/Output两个值合法。
bool is_valid_direction(EventLogDirection direction) {
  return direction == EventLogDirection::Input ||
         direction == EventLogDirection::Output;
}

// max_record_size限制内部完整ZJCL帧，max_payload_size限制其payload，单位字节。
void validate_limits(std::size_t max_record_size,
                     std::size_t max_payload_size) {
  if (max_record_size < kWireHeaderSize ||
      max_record_size >
          static_cast<std::size_t>(
              std::numeric_limits<std::uint32_t>::max()) ||
      max_payload_size > kDefaultMaxPayloadSize) {
    throw EventLogException(EventLogError::kInvalidConfiguration,
                            "event log limits are invalid");
  }
  if (max_record_size > kWireHeaderSize + max_payload_size) {
    throw EventLogException(
        EventLogError::kInvalidConfiguration,
        "event log record limit exceeds the largest allowed wire frame");
  }
}

// output为调用方持有的目标流，bytes为按原顺序写入且不被保存的字节数组。
void write_bytes(std::ostream& output,
                 const std::vector<std::uint8_t>& bytes) {
  // byte遍历待写缓冲的每个无符号线序字节。
  for (const std::uint8_t byte : bytes) {
    output.put(static_cast<char>(byte));
  }
}

std::vector<std::uint8_t> file_header() {
  return {kLogMagic[0U], kLogMagic[1U], kLogMagic[2U], kLogMagic[3U],
          kLogMajorVersion, kLogMinorVersion,
          static_cast<std::uint8_t>(kEventLogHeaderSize), 0U};
}

// input为当前位置流，error/detail指定遇到EOF时应抛出的精确截断类别和诊断。
std::uint8_t read_required_byte(std::istream& input,
                                EventLogError error,
                                const char* detail) {
  // byte使用int保留char_traits::eof哨兵，成功后才收窄为线序字节。
  const int byte = input.get();
  if (byte == std::char_traits<char>::eof()) {
    throw EventLogException(error, detail);
  }
  return static_cast<std::uint8_t>(byte);
}

}  // namespace

EventLogException::EventLogException(EventLogError code,
                                     std::string message,
                                     ProtocolError protocol_error)
    : std::runtime_error(std::move(message)),
      code_(code),
      protocol_error_(protocol_error) {}

EventLogError EventLogException::code() const noexcept { return code_; }

ProtocolError EventLogException::protocol_error() const noexcept {
  return protocol_error_;
}

EventLogWriter::EventLogWriter(const std::filesystem::path& path,
                               std::size_t max_record_size,
                               std::size_t max_payload_size)
    : max_record_size_(max_record_size),
      max_payload_size_(max_payload_size) {
  validate_limits(max_record_size_, max_payload_size_);
  output_.open(path, std::ios::binary | std::ios::trunc);
  if (!output_.is_open()) {
    throw EventLogException(EventLogError::kIoFailure,
                            "event log file could not be opened for writing");
  }
  write_bytes(output_, file_header());
  if (!output_) {
    throw EventLogException(EventLogError::kIoFailure,
                            "event log file header could not be written");
  }
}

void EventLogWriter::append(const EventLogRecord& log_record) {
  // 先校验方向、接收时刻、记录长度和内部帧CRC，再写入任何记录字节。
  if (!is_valid_direction(log_record.direction)) {
    throw EventLogException(EventLogError::kInvalidDirection,
                            "event log record direction is invalid");
  }
  if (log_record.receive_timestamp_ns == 0U) {
    throw EventLogException(
        EventLogError::kInvalidReceiveTimestamp,
        "event log record receive timestamp must be nonzero");
  }
  if (log_record.frame.size() > max_record_size_) {
    throw EventLogException(EventLogError::kRecordTooLarge,
                            "event log record exceeds configured limit");
  }
  // decoded只验证ZJCL外层头、类型、固定载荷长度、总长度及头[0,36)+payload的CRC；
  // 具体payload字段语义由后续对应decode_*处理。
  const FrameDecodeResult decoded =
      decode_frame(log_record.frame, max_payload_size_);
  if (!decoded.ok()) {
    throw EventLogException(EventLogError::kInvalidFrame,
                            "event log record is not a valid wire frame",
                            decoded.error);
  }

  // record暂存4字节frame长度、12字节元数据和完整frame，全部构造后一次写流。
  std::vector<std::uint8_t> record;
  record.reserve(4U + kRecordMetadataSize + log_record.frame.size());
  // length只计内部ZJCL帧字节；固定12字节方向/接收时间元数据由ZJLG格式隐含，
  // 因而读取器可以在分配frame前先独立检查记录上限。
  // length仅表示内部ZJCL frame字节数，不包含4字节自身及12字节记录元数据。
  const std::uint32_t length =
      static_cast<std::uint32_t>(log_record.frame.size());
  // shift依次取0/8/16/24，将length按小端写入四个字节。
  for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
    record.push_back(
        static_cast<std::uint8_t>((length >> shift) & 0xFFU));
  }
  record.push_back(static_cast<std::uint8_t>(log_record.direction));
  record.insert(record.end(), 3U, 0U);
  // shift依次遍历接收时间的八个小端字节位移。
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    record.push_back(static_cast<std::uint8_t>(
        (log_record.receive_timestamp_ns >> shift) & 0xFFU));
  }
  record.insert(record.end(), log_record.frame.begin(),
                log_record.frame.end());
  write_bytes(output_, record);
  if (!output_) {
    throw EventLogException(EventLogError::kIoFailure,
                            "event log record could not be written");
  }
}

void EventLogWriter::flush() {
  output_.flush();
  if (!output_) {
    throw EventLogException(EventLogError::kIoFailure,
                            "event log file could not be flushed");
  }
}

EventLogReader::EventLogReader(const std::filesystem::path& path,
                               std::size_t max_record_size,
                               std::size_t max_payload_size)
    : max_record_size_(max_record_size),
      max_payload_size_(max_payload_size) {
  validate_limits(max_record_size_, max_payload_size_);
  input_.open(path, std::ios::binary);
  if (!input_.is_open()) {
    throw EventLogException(EventLogError::kIoFailure,
                            "event log file could not be opened for reading");
  }

  // header暂存固定8字节ZJLG文件头；index遍历每个必须存在的头字节。
  std::array<std::uint8_t, kEventLogHeaderSize> header{};
  for (std::size_t index = 0U; index < header.size(); ++index) {
    header[index] = read_required_byte(
        input_, EventLogError::kTruncatedFileHeader,
        "event log file header is truncated");
  }
  if (header[0U] != kLogMagic[0U] || header[1U] != kLogMagic[1U] ||
      header[2U] != kLogMagic[2U] || header[3U] != kLogMagic[3U]) {
    throw EventLogException(EventLogError::kBadMagic,
                            "event log magic does not match");
  }
  if (header[4U] != kLogMajorVersion ||
      header[5U] != kLogMinorVersion) {
    throw EventLogException(EventLogError::kUnsupportedVersion,
                            "event log version is unsupported");
  }
  // header_size从文件头[6,8)按小端恢复，必须等于v1固定头长。
  const std::uint16_t header_size =
      static_cast<std::uint16_t>(header[6U]) |
      static_cast<std::uint16_t>(
          static_cast<std::uint16_t>(header[7U]) << 8U);
  if (header_size != kEventLogHeaderSize) {
    throw EventLogException(EventLogError::kInvalidHeaderSize,
                            "event log header size is invalid");
  }
}

EventLogReadResult EventLogReader::next() {
  // 阶段1：EOF只能出现在记录边界；记录中途EOF均属于文件截断。
  if (ended_) {
    return {};
  }

  // first_byte单独读取长度前缀首字节，以区分记录边界EOF与中途截断。
  const int first_byte = input_.get();
  if (first_byte == std::char_traits<char>::eof()) {
    if (input_.eof()) {
      ended_ = true;
      return {};
    }
    throw EventLogException(EventLogError::kIoFailure,
                            "event log record length could not be read");
  }
  // length为内部ZJCL帧记录长度；byte遍历余下三个小端长度字节的索引。
  std::uint32_t length = static_cast<std::uint8_t>(first_byte);
  for (unsigned int byte = 1U; byte < 4U; ++byte) {
    // part为当前必需长度字节，读取失败精确报告长度前缀截断。
    const std::uint8_t part = read_required_byte(
        input_, EventLogError::kTruncatedRecordLength,
        "event log record length prefix is truncated");
    length |= static_cast<std::uint32_t>(part) << (byte * 8U);
  }
  if (static_cast<std::size_t>(length) > max_record_size_) {
    throw EventLogException(EventLogError::kRecordTooLarge,
                            "event log record exceeds configured limit");
  }

  // metadata暂存方向、3字节保留位和8字节接收时间；index遍历固定12字节。
  std::array<std::uint8_t, kRecordMetadataSize> metadata{};
  for (std::size_t index = 0U; index < metadata.size(); ++index) {
    metadata[index] = read_required_byte(
        input_, EventLogError::kTruncatedRecordMetadata,
        "event log record metadata is truncated");
  }
  // 元数据先验证方向和保留位，再解析接收时间；未来版本若复用保留位必须升级格式。
  // direction从元数据首字节恢复，决定回放时是否重新送入算法。
  const EventLogDirection direction =
      static_cast<EventLogDirection>(metadata[0U]);
  if (!is_valid_direction(direction)) {
    throw EventLogException(EventLogError::kInvalidDirection,
                            "event log record direction is invalid");
  }
  if (metadata[1U] != 0U || metadata[2U] != 0U || metadata[3U] != 0U) {
    throw EventLogException(EventLogError::kInvalidReserved,
                            "event log record reserved bytes are nonzero");
  }
  // receive_timestamp_ns为记录本机统一接收时间；byte遍历其八个小端元数据字节。
  std::uint64_t receive_timestamp_ns = 0U;
  for (unsigned int byte = 0U; byte < 8U; ++byte) {
    receive_timestamp_ns |=
        static_cast<std::uint64_t>(metadata[4U + byte]) << (byte * 8U);
  }
  if (receive_timestamp_ns == 0U) {
    throw EventLogException(
        EventLogError::kInvalidReceiveTimestamp,
        "event log record receive timestamp must be nonzero");
  }

  // frame按已验证length独占分配；index遍历每个必须存在的内部ZJCL字节。
  std::vector<std::uint8_t> frame(static_cast<std::size_t>(length));
  for (std::size_t index = 0U; index < frame.size(); ++index) {
    frame[index] = read_required_byte(
        input_, EventLogError::kTruncatedRecord,
        "event log record is truncated");
  }
  // 阶段2：完整读取后重新校验内部ZJCL帧，损坏记录不能进入回放算法。
  // decoded只复验ZJCL外层头、类型、固定载荷长度、总长度及头[0,36)+payload的CRC；
  // 回放消费方仍须调用对应decode_*校验具体payload字段语义。
  const FrameDecodeResult decoded = decode_frame(frame, max_payload_size_);
  if (!decoded.ok()) {
    throw EventLogException(EventLogError::kInvalidFrame,
                            "event log record is not a valid wire frame",
                            decoded.error);
  }

  // result拥有通过验证的记录与frame，返回后不再借用读取器临时缓冲。
  EventLogReadResult result{};
  result.status = EventLogReadStatus::kRecord;
  result.record.direction = direction;
  result.record.receive_timestamp_ns = receive_timestamp_ns;
  result.record.frame = std::move(frame);
  return result;
}

}  // namespace zju::coop::protocol

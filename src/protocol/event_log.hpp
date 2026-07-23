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

// kEventLogHeaderSize为ZJLG文件头字节数；kDefaultMaxEventLogRecordSize为单条完整ZJCL帧的默认上限。
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
  // code为日志层错误，message为上下文文本，protocol_error仅在内部ZJCL校验失败时非kNone。
  EventLogException(EventLogError code, std::string message,
                    ProtocolError protocol_error = ProtocolError::kNone);

  [[nodiscard]] EventLogError code() const noexcept;
  [[nodiscard]] ProtocolError protocol_error() const noexcept;

 private:
  EventLogError code_; /* 随异常对象保存的机器可判定日志错误类别。 */
  ProtocolError protocol_error_; /* kInvalidFrame时保存的内层协议错误，否则为kNone。 */
};

enum class EventLogReadStatus {
  kRecord,
  kEnd,
};

enum class EventLogDirection : std::uint8_t {
  // Input会在回放时重新送入算法；Output只用于审计，不再次作为算法输入。
  Input = 1U,
  Output = 2U,
};

/** 单条日志记录；receive_timestamp_ns用于回放节奏，不替代帧内测量时间戳。 */
struct EventLogRecord {
  EventLogDirection direction{EventLogDirection::Input}; /* Input会回灌算法，Output仅供审计。 */
  std::uint64_t receive_timestamp_ns{}; /* 本机收到/产生记录的统一时间，单位ns，用于回放节奏。 */
  std::vector<std::uint8_t> frame; /* 记录独占的完整ZJCL帧字节，包含40字节头及其payload。 */
};

struct EventLogReadResult {
  EventLogReadStatus status{EventLogReadStatus::kEnd}; /* 区分成功读到一条记录与边界处正常EOF。 */
  EventLogRecord record; /* status=kRecord时有效且拥有帧字节，kEnd时保持默认值。 */
};

/**
 * 追加式日志写入器；每条记录写入前验证内部帧。
 * “追加”指顺序写记录而非打开旧文件续写：构造时创建/截断目标文件并写ZJLG头，
 * 上层负责选择不会误覆盖的任务日志路径。
 */
class EventLogWriter {
 public:
  explicit EventLogWriter(
      // path为将创建/截断的日志文件；max_record_size限制元数据后的frame长度，max_payload_size限制内层载荷。
      const std::filesystem::path& path,
      std::size_t max_record_size = kDefaultMaxEventLogRecordSize,
      std::size_t max_payload_size = kDefaultMaxPayloadSize);

  EventLogWriter(const EventLogWriter&) = delete;
  EventLogWriter& operator=(const EventLogWriter&) = delete;

  // record在调用期间只读借用，方向/接收时间/内层帧全部校验成功后才顺序写入。
  void append(const EventLogRecord& record);
  void flush();

 private:
  std::ofstream output_; /* 写入器独占的二进制输出流，随对象析构关闭。 */
  std::size_t max_record_size_{}; /* 单条记录中frame字节数上限。 */
  std::size_t max_payload_size_{}; /* 验证frame时允许的ZJCL payload字节上限。 */
};

/**
 * 严格顺序读取器；只有在记录边界处遇到EOF才返回kEnd，文件头、元数据或
 * 帧体中途EOF均报告截断，不能把损坏尾部当成正常回放完成。
 */
class EventLogReader {
 public:
  explicit EventLogReader(
      // path为只读日志文件；max_record_size/max_payload_size分别限制外层记录和内层载荷字节数。
      const std::filesystem::path& path,
      std::size_t max_record_size = kDefaultMaxEventLogRecordSize,
      std::size_t max_payload_size = kDefaultMaxPayloadSize);

  EventLogReader(const EventLogReader&) = delete;
  EventLogReader& operator=(const EventLogReader&) = delete;

  [[nodiscard]] EventLogReadResult next();

 private:
  std::ifstream input_; /* 读取器独占的二进制输入流，当前位置始终位于下一记录边界。 */
  std::size_t max_record_size_{}; /* 接受的frame记录长度上限，防止恶意分配。 */
  std::size_t max_payload_size_{}; /* 内层decode_frame允许的payload字节上限。 */
  bool ended_{}; /* true表示已在记录边界遇到EOF，后续next稳定返回kEnd。 */
};

}  // namespace zju::coop::protocol

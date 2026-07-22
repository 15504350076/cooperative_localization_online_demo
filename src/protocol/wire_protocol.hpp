// 模块职责：定义无硬件阶段使用的临时ZJCL二进制帧和各类固定载荷。
// 生产边界：正式AIBrainBox部署优先由上交ROS 2节点调用C ABI，本协议只用于本地UDP演示、
// GCS联调和事件日志；所有多字节字段按小端编码，浮点按IEEE-754位模式传输。
#pragma once

#include "zju_coop/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace zju::coop::protocol {

/* ZJCL仅是脱离ROS 2的临时在线演示协议；生产盒端优先直接调用C ABI。 */

inline constexpr std::uint8_t kProtocolMajorVersion = 1U;
inline constexpr std::uint8_t kProtocolMinorVersion = 0U;
inline constexpr std::size_t kWireHeaderSize = 40U;
inline constexpr std::size_t kDefaultMaxPayloadSize = 1024U * 1024U;
inline constexpr std::size_t kRangePayloadSize = 24U;
inline constexpr std::size_t kImuPayloadSize = 332U;
inline constexpr std::size_t kLocalizationPayloadSize = 64U;
inline constexpr std::size_t kNetworkPayloadSize = 20U;
inline constexpr std::size_t kObservationPayloadSize = 80U;
inline constexpr std::size_t kAlertPayloadSize = 40U;
inline constexpr std::size_t kAlgorithmStatusPayloadSize = 48U;
inline constexpr std::uint32_t kAlgorithmStatusAbiVersion = 0x00010000U;
inline constexpr std::uint32_t kSoftwareVersionPacked = 0x00000100U;
inline constexpr std::uint8_t kRangeStatusOk = 0U;
inline constexpr std::uint8_t kRangeStatusDegraded = 1U;
inline constexpr std::uint8_t kRangeStatusInvalid = 2U;

enum class MessageType : std::uint16_t {
  kRange = 1U,
  kImu = 2U,
  kLocalization = 100U,
  kNetwork = 101U,
  kObservation = 102U,
  kAlert = 103U,
  kAlgorithmStatus = 104U,
};

enum class ProtocolError {
  kNone,
  kTruncated,
  kTrailingBytes,
  kBadMagic,
  kUnsupportedVersion,
  kInvalidHeaderSize,
  kUnknownMessageType,
  kPayloadTooLarge,
  kCrcMismatch,
  kInvalidPayloadSize,
  kInvalidBoolean,
  kNonFiniteValue,
  kInvalidValue,
  kInvalidReserved,
};

/** 40字节公共帧头；CRC覆盖CRC字段清零后的帧头和完整载荷。 */
struct FrameHeader {
  MessageType message_type{MessageType::kRange};
  std::uint16_t flags{};
  std::uint32_t payload_size{};
  std::uint64_t sequence{};
  std::uint64_t timestamp_ns{};
  std::uint16_t source_node{};
  std::uint16_t target_node{};
  std::uint32_t crc32{};
};

struct Frame {
  FrameHeader header{};
  std::vector<std::uint8_t> payload;
};

/** 解码不抛异常，错误枚举与detail供在线程序计数和诊断。 */
struct FrameDecodeResult {
  ProtocolError error{ProtocolError::kNone};
  Frame value{};
  std::string detail;

  [[nodiscard]] bool ok() const noexcept {
    return error == ProtocolError::kNone;
  }
};

template <typename Value>
struct PayloadDecodeResult {
  ProtocolError error{ProtocolError::kNone};
  Value value{};
  std::string detail;

  [[nodiscard]] bool ok() const noexcept {
    return error == ProtocolError::kNone;
  }
};

/** 直接平台间测距及NLOS质量字段，节点和时间位于公共帧头。 */
struct RangePayload {
  double range_m{};
  double range_std_m{};
  float nlos_probability{};
  bool nlos_flag{};
  bool has_nlos_probability{};
  bool valid{};
  std::uint8_t status{};
};

/** 临时在线协议中的标准IMU瞬时量；节点号和采样时间位于40字节帧头。 */
struct ImuPayload {
  std::array<double, 4> orientation_xyzw{};
  std::array<double, 9> orientation_covariance{};
  std::array<double, 3> angular_velocity_rad_s{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 3> linear_acceleration_m_s2{};
  std::array<double, 9> linear_acceleration_covariance{};
  std::array<char, 32> frame_id{};
  bool orientation_valid{};
  bool valid{};
  std::uint8_t status{};
  std::uint8_t reserved{};
};

struct LocalizationPayload {
  double x{};
  double y{};
  double vx{};
  double vy{};
  double cov_xx{};
  double cov_xy{};
  double cov_yy{};
  LocalizationState state{LocalizationState::kUninitialized};
  bool valid{};
  bool yaw_valid{};
  bool z_valid{};
  std::uint32_t capability_mask{};
};

struct NetworkPayload {
  std::uint32_t node_count{};
  std::uint32_t reachable_node_count{};
  std::uint32_t active_edge_count{};
  bool connected{};
  bool observable{};
  LocalizationState state{LocalizationState::kUninitialized};
  std::uint8_t reserved{};
  std::uint32_t reason_mask{};
};

struct ObservationPayload {
  std::uint64_t window_start_ns{};
  std::uint64_t window_end_ns{};
  std::uint32_t expected_count{};
  std::uint32_t received_count{};
  std::uint32_t valid_count{};
  std::uint32_t nlos_count{};
  std::uint32_t residual_rejected_count{};
  std::uint32_t dropped_count{};
  double nlos_ratio{};
  double valid_ratio{};
  double actual_rate_hz{};
  double covariance_scale{1.0};
  ObservationState state{ObservationState::kUnknown};
  FusionAction action{FusionAction::kUseNormal};
  bool input_overflow{};
  std::uint8_t reserved{};
  std::uint32_t reason_mask{};
};

enum class AlgorithmMode : std::uint8_t {
  kUwbOnlyPlanar = 1U,
  kImuUwb15State = 2U,
};

enum class AlgorithmRunState : std::uint8_t {
  kInitializing = 0U,
  kRunning = 1U,
  kDegraded = 2U,
  kError = 3U,
  kStopped = 4U,
};

struct AlgorithmStatusPayload {
  std::uint32_t abi_version{kAlgorithmStatusAbiVersion};
  std::uint32_t software_version_packed{kSoftwareVersionPacked};
  AlgorithmMode mode{AlgorithmMode::kUwbOnlyPlanar};
  AlgorithmRunState run_state{AlgorithmRunState::kInitializing};
  std::uint16_t reserved0{};
  std::uint32_t reserved1{};
  std::uint64_t accepted_ranges{};
  std::uint64_t rejected_ranges{};
  std::uint64_t protocol_errors{};
  std::uint64_t uptime_ns{};
};

enum class AlertCode : std::uint32_t {
  kNetworkState = 1U,
};

enum class AlertLevel : std::uint8_t {
  kInfo = 0U,
  kWarning = 1U,
  kError = 2U,
  kCritical = 3U,
};

enum class AlertLifecycle : std::uint8_t {
  kActive = 0U,
  kCleared = 1U,
};

enum class AlertSource : std::uint8_t {
  kAlgorithm = 0U,
};

struct AlertPayload {
  AlertCode alert_code{AlertCode::kNetworkState};
  AlertLevel level{AlertLevel::kInfo};
  AlertLifecycle lifecycle{AlertLifecycle::kActive};
  AlertSource source{AlertSource::kAlgorithm};
  std::uint8_t reserved0{};
  std::uint32_t reason_mask{};
  std::uint16_t node_id{};
  std::uint16_t from_node{};
  std::uint16_t to_node{};
  std::uint16_t reserved1{};
  std::uint64_t first_timestamp_ns{};
  std::uint64_t last_timestamp_ns{};
  std::uint32_t reserved2{};
};

[[nodiscard]] bool is_known_message_type(MessageType type) noexcept;

[[nodiscard]] std::vector<std::uint8_t> encode_frame(
    const Frame& frame,
    std::size_t max_payload_size = kDefaultMaxPayloadSize);

[[nodiscard]] FrameDecodeResult decode_frame(
    const std::vector<std::uint8_t>& bytes,
    std::size_t max_payload_size = kDefaultMaxPayloadSize);

[[nodiscard]] std::vector<std::uint8_t> encode_range_payload(
    const RangePayload& payload);
[[nodiscard]] PayloadDecodeResult<RangePayload> decode_range_payload(
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_imu_payload(
    const ImuPayload& payload);
[[nodiscard]] PayloadDecodeResult<ImuPayload> decode_imu_payload(
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_localization_payload(
    const LocalizationPayload& payload);
[[nodiscard]] PayloadDecodeResult<LocalizationPayload>
decode_localization_payload(const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_network_payload(
    const NetworkPayload& payload);
[[nodiscard]] PayloadDecodeResult<NetworkPayload> decode_network_payload(
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_observation_payload(
    const ObservationPayload& payload);
[[nodiscard]] PayloadDecodeResult<ObservationPayload>
decode_observation_payload(const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_alert_payload(
    const AlertPayload& payload);
[[nodiscard]] PayloadDecodeResult<AlertPayload> decode_alert_payload(
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_algorithm_status_payload(
    const AlgorithmStatusPayload& payload);
[[nodiscard]] PayloadDecodeResult<AlgorithmStatusPayload>
decode_algorithm_status_payload(const std::vector<std::uint8_t>& bytes);

}  // namespace zju::coop::protocol

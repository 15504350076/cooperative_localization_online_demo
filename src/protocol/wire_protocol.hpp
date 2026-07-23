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

// 协议版本、40字节公共头、默认载荷上限及各固定载荷长度均以线序字节计。
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
// kAlgorithmStatusAbiVersion/kSoftwareVersionPacked写入状态载荷供远端兼容检查；三个range状态值沿用设备质量编码。
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

/**
 * 40字节公共帧头；CRC输入为头部[0,36)字节紧接完整载荷，明确排除CRC字段。
 * sequence由消息生产者按数据流递增，timestamp_ns表示内容对应的测量/输出时刻；
 * source/target的具体含义由消息类型定义，广播输出可使用target=0。
 */
struct FrameHeader {
  MessageType message_type{MessageType::kRange}; /* 决定固定payload布局及source/target语义。 */
  std::uint16_t flags{}; /* v1尚未定义位语义；当前在线/回放输出为0，编解码器允许并原样保留非零值。 */
  std::uint32_t payload_size{}; /* 紧随40字节头的载荷字节数，编码时由payload实际长度覆盖。 */
  std::uint64_t sequence{}; /* 消息生产者在对应数据流内递增的序号。 */
  std::uint64_t timestamp_ns{}; /* 内容对应的统一传感器/算法时间，单位ns。 */
  std::uint16_t source_node{}; /* 消息来源平台编号；输出由消息类型解释。 */
  std::uint16_t target_node{}; /* 目标平台编号，广播型输出可为0。 */
  std::uint32_t crc32{}; /* 头[0,36)+完整payload的IEEE CRC32，明确排除本字段。 */
};

struct Frame {
  FrameHeader header{}; /* 主机序公共字段；编码时生成固定小端布局。 */
  std::vector<std::uint8_t> payload; /* 帧对象独占的消息类型专属载荷字节。 */
};

/**
 * 解码不抛异常，错误枚举与detail供在线程序计数和诊断。
 * 解码顺序固定为外层长度→魔数/版本/类型→固定载荷长度→CRC→字段语义，
 * 任何一步失败都不得向算法返回部分Frame。
 */
struct FrameDecodeResult {
  ProtocolError error{ProtocolError::kNone}; /* kNone时value完整有效，否则不得使用部分值。 */
  Frame value{}; /* 成功时拥有解码后的头与载荷。 */
  std::string detail; /* 面向日志的失败阶段说明，不作为机器分支依据。 */

  [[nodiscard]] bool ok() const noexcept {
    return error == ProtocolError::kNone;
  }
};

template <typename Value>
struct PayloadDecodeResult {
  ProtocolError error{ProtocolError::kNone}; /* kNone时value完整有效。 */
  Value value{}; /* 成功时拥有对应固定载荷的强类型字段。 */
  std::string detail; /* 载荷长度、布尔或数值语义失败的诊断文本。 */

  [[nodiscard]] bool ok() const noexcept {
    return error == ProtocolError::kNone;
  }
};

/** 直接平台间测距及NLOS质量字段，节点和时间位于公共帧头。 */
struct RangePayload {
  double range_m{}; /* 两节点直接距离量测，单位m。 */
  double range_std_m{}; /* 距离1σ标准差，单位m且必须为正。 */
  float nlos_probability{}; /* [0,1]概率，仅has_nlos_probability=true时有效。 */
  bool nlos_flag{}; /* 设备侧NLOS硬判决。 */
  bool has_nlos_probability{}; /* 是否携带可信概率字段。 */
  bool valid{}; /* 上游是否认可整条量测可供算法处理。 */
  std::uint8_t status{}; /* 设备侧OK/DEGRADED/INVALID质量码。 */
};

/** 临时在线协议中的标准IMU瞬时量；节点号和采样时间位于40字节帧头。 */
struct ImuPayload {
  std::array<double, 4> orientation_xyzw{}; /* 车体FLU到导航ENU的[x,y,z,w]四元数。 */
  std::array<double, 9> orientation_covariance{}; /* 姿态误差3×3协方差，ROS行主序。 */
  std::array<double, 3> angular_velocity_rad_s{}; /* 车体FLU瞬时角速度，单位rad/s。 */
  std::array<double, 9> angular_velocity_covariance{}; /* 角速度3×3协方差，行主序。 */
  std::array<double, 3> linear_acceleration_m_s2{}; /* 车体FLU瞬时比力，单位m/s²。 */
  std::array<double, 9> linear_acceleration_covariance{}; /* 比力3×3协方差，行主序。 */
  std::array<char, 32> frame_id{}; /* 固定容量且须NUL结尾的IMU坐标系名。 */
  bool orientation_valid{}; /* orientation_xyzw是否可信，独立于valid。 */
  bool valid{}; /* 角速度/比力与时间是否可处理。 */
  std::uint8_t status{}; /* 设备质量码。 */
  std::uint8_t reserved{}; /* v1线序保留字节，必须为零。 */
};

struct LocalizationPayload {
  // 当前只承诺平面相对位置/速度；capability与三个valid位共同约束消费者显示。
  double x{};  /* source-target在ENU平面的东向相对位置，单位m。 */
  double y{};  /* source-target在ENU平面的北向相对位置，单位m。 */
  double vx{}; /* 东向相对速度，单位m/s。 */
  double vy{}; /* 北向相对速度，单位m/s。 */
  double cov_xx{}; /* 平面位置协方差(0,0)，单位m²。 */
  double cov_xy{}; /* 平面位置协方差对称非对角项，单位m²。 */
  double cov_yy{}; /* 平面位置协方差(1,1)，单位m²。 */
  LocalizationState state{LocalizationState::kUninitialized}; /* 面向GCS的综合定位状态。 */
  bool valid{}; /* 平面位置/速度当前是否可用。 */
  bool yaw_valid{}; /* 航向是否另有有效输出，v1当前为false。 */
  bool z_valid{}; /* 高度是否另有有效输出，v1当前为false。 */
  std::uint32_t capability_mask{}; /* 声明算法可提供的Capability位图，仍须结合三个valid位。 */
};

struct NetworkPayload {
  std::uint32_t node_count{}; /* 已配置节点总数。 */
  std::uint32_t reachable_node_count{}; /* 从参考节点经活动边可达的节点数。 */
  std::uint32_t active_edge_count{}; /* 未超时且参与当前拓扑的无向边数。 */
  bool connected{}; /* 全部节点是否从参考节点可达。 */
  bool observable{}; /* 当前几何图是否满足平面相对定位秩条件。 */
  LocalizationState state{LocalizationState::kUninitialized}; /* 综合网络定位状态。 */
  std::uint8_t reserved{}; /* v1线序保留字节，必须为零。 */
  std::uint32_t reason_mask{}; /* 可按位组合的退化/不可观原因。 */
};

struct ObservationPayload {
  std::uint64_t window_start_ns{}; /* 质量滑窗起点，统一时间轴纳秒。 */
  std::uint64_t window_end_ns{}; /* 质量滑窗终点，统一时间轴纳秒。 */
  std::uint32_t expected_count{}; /* 标称频率换算的窗口应到包数。 */
  std::uint32_t received_count{}; /* 窗口实际接收包数。 */
  std::uint32_t valid_count{}; /* 通过设备有效性检查的包数。 */
  std::uint32_t nlos_count{}; /* 硬标志或概率判为NLOS的包数。 */
  std::uint32_t residual_rejected_count{}; /* 被滤波残差/NIS拒绝的包数。 */
  std::uint32_t dropped_count{}; /* 样本历史容量溢出并淘汰最旧样本的生命周期累计次数。 */
  double nlos_ratio{}; /* nlos_count/received_count，无量纲[0,1]。 */
  double valid_ratio{}; /* valid_count/expected_count；线协议编解码要求位于[0,1]。 */
  double actual_rate_hz{}; /* received_count固定按配置window_ns换算的频率，单位Hz。 */
  double covariance_scale{1.0}; /* 当前动作施加到测距方差的倍率。 */
  ObservationState state{ObservationState::kUnknown}; /* 单边长期质量状态。 */
  FusionAction action{FusionAction::kUseNormal}; /* 当前状态对应的实际滤波动作。 */
  bool input_overflow{}; /* 当前滑窗内是否发生过样本历史容量溢出淘汰，并非永久故障标志。 */
  std::uint8_t reserved{}; /* v1线序保留字节，必须为零。 */
  std::uint32_t reason_mask{}; /* 可按位组合的单边退化原因。 */
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
  // 运行计数是进程级累计诊断，不是滤波状态的一部分，重启后允许从零开始。
  std::uint32_t abi_version{kAlgorithmStatusAbiVersion}; /* 远端解释C ABI语义所需版本。 */
  std::uint32_t software_version_packed{kSoftwareVersionPacked}; /* 主次补丁号打包的软件版本。 */
  AlgorithmMode mode{AlgorithmMode::kUwbOnlyPlanar}; /* 本进程固定采用的状态模型。 */
  AlgorithmRunState run_state{AlgorithmRunState::kInitializing}; /* 当前运行健康状态。 */
  std::uint16_t reserved0{}; /* v1对齐保留位，必须为零。 */
  std::uint32_t reserved1{}; /* v1扩展保留位，必须为零。 */
  std::uint64_t accepted_ranges{}; /* 进程启动后累计接受的测距包数。 */
  std::uint64_t rejected_ranges{}; /* 进程启动后累计拒绝的测距包数。 */
  std::uint64_t protocol_errors{}; /* 进程启动后累计的ZJCL解码错误数。 */
  std::uint64_t uptime_ns{}; /* 进程单调时钟运行时长，单位ns。 */
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
  AlertCode alert_code{AlertCode::kNetworkState}; /* 告警业务类别。 */
  AlertLevel level{AlertLevel::kInfo}; /* 当前严重级别。 */
  AlertLifecycle lifecycle{AlertLifecycle::kActive}; /* 激活或清除事件。 */
  AlertSource source{AlertSource::kAlgorithm}; /* 产生告警的子系统。 */
  std::uint8_t reserved0{}; /* v1线序保留字节，必须为零。 */
  std::uint32_t reason_mask{}; /* 可按位组合的触发原因。 */
  std::uint16_t node_id{}; /* 单节点告警主体，0表示不适用。 */
  std::uint16_t from_node{}; /* 单边告警规范化较小节点号，0表示不适用。 */
  std::uint16_t to_node{}; /* 单边告警规范化较大节点号，0表示不适用。 */
  std::uint16_t reserved1{}; /* v1对齐保留位，必须为零。 */
  std::uint64_t first_timestamp_ns{}; /* 当前告警首次出现的统一时间，单位ns。 */
  std::uint64_t last_timestamp_ns{}; /* 当前状态最近更新的统一时间，单位ns。 */
  std::uint32_t reserved2{}; /* v1尾部保留位，必须为零。 */
};

// type为线序解出的候选消息类型，返回其是否具有v1固定语义和载荷布局。
[[nodiscard]] bool is_known_message_type(MessageType type) noexcept;

/** 编码失败抛invalid_argument；成功结果恰好是一帧，不含UDP或日志外层长度。 */
[[nodiscard]] std::vector<std::uint8_t> encode_frame(
    // frame为主机序头和完整payload；max_payload_size为允许编码的载荷字节上限。
    const Frame& frame,
    std::size_t max_payload_size = kDefaultMaxPayloadSize);

/** 严格要求输入恰好一帧，尾随字节也视为协议错误。 */
[[nodiscard]] FrameDecodeResult decode_frame(
    // bytes必须恰好包含一帧且由调用方持有；max_payload_size限制头部声明的载荷字节数。
    const std::vector<std::uint8_t>& bytes,
    std::size_t max_payload_size = kDefaultMaxPayloadSize);

[[nodiscard]] std::vector<std::uint8_t> encode_range_payload(
    // payload为待按24字节固定小端布局编码的测距字段。
    const RangePayload& payload);
[[nodiscard]] PayloadDecodeResult<RangePayload> decode_range_payload(
    // bytes必须恰好为24字节测距载荷，不含公共帧头。
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_imu_payload(
    // payload为待按332字节固定布局编码的瞬时IMU字段。
    const ImuPayload& payload);
[[nodiscard]] PayloadDecodeResult<ImuPayload> decode_imu_payload(
    // bytes必须恰好为332字节IMU载荷，不含公共帧头。
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_localization_payload(
    // payload为待编码的64字节平面相对定位输出。
    const LocalizationPayload& payload);
[[nodiscard]] PayloadDecodeResult<LocalizationPayload>
// bytes必须恰好为64字节定位载荷。
decode_localization_payload(const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_network_payload(
    // payload为待编码的20字节协同图快照。
    const NetworkPayload& payload);
[[nodiscard]] PayloadDecodeResult<NetworkPayload> decode_network_payload(
    // bytes必须恰好为20字节网络载荷。
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_observation_payload(
    // payload为待编码的80字节单边质量滑窗快照。
    const ObservationPayload& payload);
[[nodiscard]] PayloadDecodeResult<ObservationPayload>
// bytes必须恰好为80字节边质量载荷。
decode_observation_payload(const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_alert_payload(
    // payload为待编码的40字节告警生命周期事件。
    const AlertPayload& payload);
[[nodiscard]] PayloadDecodeResult<AlertPayload> decode_alert_payload(
    // bytes必须恰好为40字节告警载荷。
    const std::vector<std::uint8_t>& bytes);

[[nodiscard]] std::vector<std::uint8_t> encode_algorithm_status_payload(
    // payload为待编码的48字节进程级算法状态与累计计数。
    const AlgorithmStatusPayload& payload);
[[nodiscard]] PayloadDecodeResult<AlgorithmStatusPayload>
// bytes必须恰好为48字节算法状态载荷。
decode_algorithm_status_payload(const std::vector<std::uint8_t>& bytes);

}  // namespace zju::coop::protocol

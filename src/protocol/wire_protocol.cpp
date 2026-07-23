// 模块实现：ZJCL公共帧头和各固定载荷的小端编解码、CRC校验与字段语义校验。
// 关键原则：编码端拒绝不可表示数据，解码端按“长度→头部→CRC→载荷语义”顺序失败；
// C++与Python实现共享固定字节布局，任何新增字段必须升级协议版本或使用新消息类型。
#include "protocol/wire_protocol.hpp"

#include "protocol/crc32.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace zju::coop::protocol {
namespace {

// kMagic为ZJCL线序签名；kCrcOffset为40字节头内CRC字段起始偏移。
constexpr std::array<std::uint8_t, 4U> kMagic{'Z', 'J', 'C', 'L'};
constexpr std::size_t kCrcOffset = 36U;

static_assert(sizeof(float) == sizeof(std::uint32_t),
              "wire protocol requires 32-bit float");
static_assert(sizeof(double) == sizeof(std::uint64_t),
              "wire protocol requires 64-bit double");
static_assert(std::numeric_limits<float>::is_iec559,
              "wire protocol requires IEEE-754 float");
static_assert(std::numeric_limits<double>::is_iec559,
              "wire protocol requires IEEE-754 double");

// output为调用方拥有的线序缓冲，value为待追加的小端16位无符号字段。
void append_u16(std::vector<std::uint8_t>& output, std::uint16_t value) {
  // 协议固定为小端，不能直接复制主机端整数布局，否则跨架构结果不可控。
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
  output.push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
}

// output为线序缓冲，value为待追加的32位字段；shift遍历四个小端字节位移。
void append_u32(std::vector<std::uint8_t>& output, std::uint32_t value) {
  for (unsigned int shift = 0U; shift < 32U; shift += 8U) {
    output.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

// output为线序缓冲，value为待追加的64位字段；shift遍历八个小端字节位移。
void append_u64(std::vector<std::uint8_t>& output, std::uint64_t value) {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    output.push_back(
        static_cast<std::uint8_t>((value >> shift) & 0xFFU));
  }
}

// output为线序缓冲，value为待按IEEE-754位模式编码的单精度字段。
void append_float(std::vector<std::uint8_t>& output, float value) {
  // bits接收不做数值转换的32位浮点位模式。
  std::uint32_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u32(output, bits);
}

// output为线序缓冲，value为待按IEEE-754位模式编码的双精度字段。
void append_double(std::vector<std::uint8_t>& output, double value) {
  // bits接收不做数值转换的64位浮点位模式。
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  append_u64(output, bits);
}

// input为已验证长度的线序缓冲，offset为16位字段首字节偏移。
std::uint16_t read_u16(const std::vector<std::uint8_t>& input,
                       std::size_t offset) {
  return static_cast<std::uint16_t>(input[offset]) |
         static_cast<std::uint16_t>(
             static_cast<std::uint16_t>(input[offset + 1U]) << 8U);
}

// input为已验证长度的线序缓冲，offset为字段首字节；value累加结果，byte遍历4字节。
std::uint32_t read_u32(const std::vector<std::uint8_t>& input,
                       std::size_t offset) {
  std::uint32_t value = 0U;
  for (unsigned int byte = 0U; byte < 4U; ++byte) {
    value |= static_cast<std::uint32_t>(input[offset + byte]) <<
             (byte * 8U);
  }
  return value;
}

// input为已验证长度的线序缓冲，offset为字段首字节；value累加结果，byte遍历8字节。
std::uint64_t read_u64(const std::vector<std::uint8_t>& input,
                       std::size_t offset) {
  std::uint64_t value = 0U;
  for (unsigned int byte = 0U; byte < 8U; ++byte) {
    value |= static_cast<std::uint64_t>(input[offset + byte]) <<
             (byte * 8U);
  }
  return value;
}

// input为已验证长度的线序缓冲，offset指向4字节IEEE-754位模式。
float read_float(const std::vector<std::uint8_t>& input,
                 std::size_t offset) {
  // bits为从input[offset,offset+4)按小端恢复的IEEE-754单精度线序位模式。
  const std::uint32_t bits = read_u32(input, offset);
  // value由memcpy按位恢复浮点值，避免数值转换或违反别名规则。
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// input为已验证长度的线序缓冲，offset指向8字节IEEE-754位模式。
double read_double(const std::vector<std::uint8_t>& input,
                   std::size_t offset) {
  // bits为从input[offset,offset+8)按小端恢复的IEEE-754双精度线序位模式。
  const std::uint64_t bits = read_u64(input, offset);
  // value由memcpy按位恢复浮点值，避免数值转换或违反别名规则。
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// output为已分配帧缓冲，offset为覆盖起点，value按4字节小端回填；byte遍历目标字节。
void write_u32(std::vector<std::uint8_t>& output, std::size_t offset,
               std::uint32_t value) {
  for (unsigned int byte = 0U; byte < 4U; ++byte) {
    output[offset + byte] =
        static_cast<std::uint8_t>((value >> (byte * 8U)) & 0xFFU);
  }
}

// frame_bytes为完整40字节头+payload；返回CRC覆盖的头[0,36)+payload，排除[36,40) CRC字段。
std::vector<std::uint8_t> crc_input(
    const std::vector<std::uint8_t>& frame_bytes) {
  // CRC输入跳过帧头中的CRC字段，再拼接载荷；编码和解码必须使用同一规则。
  // input拥有拼接后的校验字节副本，生命周期独立于frame_bytes。
  std::vector<std::uint8_t> input;
  input.reserve(kCrcOffset + frame_bytes.size() - kWireHeaderSize);
  input.insert(input.end(), frame_bytes.begin(),
               frame_bytes.begin() + static_cast<std::ptrdiff_t>(kCrcOffset));
  input.insert(input.end(),
               frame_bytes.begin() +
                   static_cast<std::ptrdiff_t>(kWireHeaderSize),
               frame_bytes.end());
  return input;
}

// error为机器协议错误，detail为静态诊断文本；result不携带部分Frame。
FrameDecodeResult failure(ProtocolError error, const char* detail) {
  FrameDecodeResult result{};
  result.error = error;
  result.detail = detail;
  return result;
}

template <typename Value>
// error/detail描述固定载荷失败；result的Value保持默认值，调用方不得在失败时使用。
PayloadDecodeResult<Value> payload_failure(ProtocolError error,
                                           const char* detail) {
  PayloadDecodeResult<Value> result{};
  result.error = error;
  result.detail = detail;
  return result;
}

// byte为线序布尔编码；value仅在byte为0或1时写入，非法时保持调用方原值。
bool decode_boolean(std::uint8_t byte, bool& value) {
  if (byte > 1U) {
    return false;
  }
  value = byte == 1U;
  return true;
}

// value为协议语义检查中待排除NaN/Inf的浮点字段。
bool finite(double value) { return std::isfinite(value); }

template <std::size_t Size>
// values为Size个固定浮点字段；lambda的[]明确不捕获外部对象，value依次检查当前数组元素有限。
bool finite_array(const std::array<double, Size>& values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return finite(value); });
}

// covariance为ROS行主序3×3数组，接受全零未知或[-1,0...]不可用哨兵，否则检查对称半正定。
bool valid_imu_covariance(const std::array<double, 9>& covariance) {
  // all_zero判定ROS“协方差未知”编码；两个lambda的[]均不捕获外部对象，value遍历相应元素范围。
  const bool all_zero = std::all_of(
      covariance.begin(), covariance.end(),
      [](double value) { return value == 0.0; });
  if (all_zero) {
    return true;
  }
  if (covariance[0] == -1.0) {
    return std::all_of(covariance.begin() + 1, covariance.end(),
                       [](double value) { return value == 0.0; });
  }
  if (!finite_array(covariance)) {
    return false;
  }
  // tolerance为对称性/主子式数值容差；row/col遍历3×3上三角元素。
  constexpr double tolerance = 1.0e-9;
  for (std::size_t row = 0U; row < 3U; ++row) {
    if (covariance[row * 3U + row] < 0.0) {
      return false;
    }
    for (std::size_t col = row + 1U; col < 3U; ++col) {
      if (std::abs(covariance[row * 3U + col] -
                   covariance[col * 3U + row]) > tolerance) {
        return false;
      }
    }
  }
  // determinant为3×3协方差行列式，用于半正定主子式检查。
  const double determinant =
      covariance[0] * (covariance[4] * covariance[8] -
                       covariance[5] * covariance[7]) -
      covariance[1] * (covariance[3] * covariance[8] -
                       covariance[5] * covariance[6]) +
      covariance[2] * (covariance[3] * covariance[7] -
                       covariance[4] * covariance[6]);
  return covariance[0] * covariance[4] - covariance[1] * covariance[3] >=
             -tolerance &&
         determinant >= -tolerance;
}

// payload为待编码或已解码IMU载荷，统一检查有限值、协方差、帧名、状态、保留位及姿态范数。
bool valid_imu_payload(const ImuPayload& payload) {
  // frame_terminated要求frame_id非空且在32字节内出现NUL。
  const bool frame_terminated =
      std::find(payload.frame_id.begin(), payload.frame_id.end(), '\0') !=
          payload.frame_id.end() &&
      payload.frame_id[0] != '\0';
  if (!finite_array(payload.orientation_xyzw) ||
      !finite_array(payload.angular_velocity_rad_s) ||
      !finite_array(payload.linear_acceleration_m_s2) ||
      !valid_imu_covariance(payload.orientation_covariance) ||
      !valid_imu_covariance(payload.angular_velocity_covariance) ||
      !valid_imu_covariance(payload.linear_acceleration_covariance) ||
      !frame_terminated || payload.status > kRangeStatusInvalid ||
      payload.reserved != 0U) {
    return false;
  }
  if (payload.orientation_valid) {
    // norm_squared累计四元数范数平方；value遍历xyzw四个分量。
    double norm_squared = 0.0;
    for (const double value : payload.orientation_xyzw) {
      norm_squared += value * value;
    }
    if (!finite(norm_squared) || norm_squared <= 0.0) {
      return false;
    }
  }
  return true;
}

// value为概率或比率候选，必须有限且处于闭区间[0,1]。
bool unit_interval(double value) {
  return finite(value) && value >= 0.0 && value <= 1.0;
}

// value为线序定位状态编码，只接受v1枚举覆盖范围。
bool valid_localization_state(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(LocalizationState::kStale);
}

// value为线序观测状态编码，只接受v1枚举覆盖范围。
bool valid_observation_state(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(ObservationState::kRecovering);
}

// value为线序融合动作编码，只接受v1枚举覆盖范围。
bool valid_fusion_action(std::uint8_t value) {
  return value <= static_cast<std::uint8_t>(FusionAction::kTrialRecovery);
}

struct ScaledProduct {
  double fraction{}; /* frexp归一化乘积尾数，用于避免直接乘法溢出。 */
  int exponent{};    /* 两因子与尾数归一化指数之和。 */
};

// left/right为非负有限因子，返回其不溢出的frexp尺度表示。
ScaledProduct scaled_product(double left, double right) {
  // left_exponent/right_exponent接收两因子指数；left_fraction/right_fraction为对应归一化尾数。
  int left_exponent = 0;
  int right_exponent = 0;
  const double left_fraction = std::frexp(left, &left_exponent);
  const double right_fraction = std::frexp(right, &right_exponent);
  // product_exponent/product_fraction再次归一化尾数乘积，便于统一比较。
  int product_exponent = 0;
  const double product_fraction = std::frexp(
      left_fraction * right_fraction, &product_exponent);
  return {product_fraction,
          left_exponent + right_exponent + product_exponent};
}

// value为待平方的协方差非对角幅值，left/right为两个对角项；比较value²<=left×right且避免溢出。
bool square_no_greater_than_product(double value, double left,
                                    double right) {
  // square/product分别保存两侧乘积的尺度表示。
  const ScaledProduct square = scaled_product(value, value);
  const ScaledProduct product = scaled_product(left, right);
  if (square.exponent != product.exponent) {
    return square.exponent < product.exponent;
  }
  return square.fraction <= product.fraction;
}

// payload提供2×2平面位置协方差，检查非负对角和半正定约束。
bool valid_localization_covariance(const LocalizationPayload& payload) {
  if (payload.cov_xx < 0.0 || payload.cov_yy < 0.0) {
    return false;
  }
  if (payload.cov_xx == 0.0 || payload.cov_yy == 0.0) {
    return payload.cov_xy == 0.0;
  }
  // magnitude为非对角协方差绝对值。
  const double magnitude = std::abs(payload.cov_xy);
  return magnitude == 0.0 || square_no_greater_than_product(
                                 magnitude, payload.cov_xx,
                                 payload.cov_yy);
}

// payload为网络快照，检查节点/可达数、活动边上下界及connected/observable逻辑一致性。
bool valid_network_topology(const NetworkPayload& payload) {
  if (payload.reachable_node_count > payload.node_count) {
    return false;
  }
  // node_count/reachable_node_count提升为64位；min/max_edge_count为连通下界和完全图上界。
  const std::uint64_t node_count = payload.node_count;
  const std::uint64_t reachable_node_count =
      payload.reachable_node_count;
  const std::uint64_t min_edge_count =
      reachable_node_count == 0U ? 0U : reachable_node_count - 1U;
  const std::uint64_t max_edge_count =
      node_count * (node_count == 0U ? 0U : node_count - 1U) / 2U;
  // active_edge_count为线序报告的当前活动无向边数。
  const std::uint64_t active_edge_count = payload.active_edge_count;
  if (active_edge_count < min_edge_count ||
      active_edge_count > max_edge_count) {
    return false;
  }

  // expected_connected由节点总数与可达数唯一推导，用于核对显式布尔字段。
  const bool expected_connected =
      payload.node_count > 0U &&
      payload.reachable_node_count == payload.node_count;
  return payload.connected == expected_connected &&
         (!payload.observable || payload.connected);
}

// payload为单边滑窗，检查时间顺序以及各子计数不超过received/valid母计数。
bool valid_observation_counts(const ObservationPayload& payload) {
  return payload.window_start_ns <= payload.window_end_ns &&
         payload.valid_count <= payload.received_count &&
         payload.nlos_count <= payload.received_count &&
         payload.residual_rejected_count <= payload.received_count &&
         payload.residual_rejected_count <= payload.valid_count;
}

// payload为进程状态，检查ABI、算法模式和运行状态均属v1范围。
bool valid_algorithm_status(const AlgorithmStatusPayload& payload) {
  return payload.abi_version == kAlgorithmStatusAbiVersion &&
         (payload.mode == AlgorithmMode::kUwbOnlyPlanar ||
          payload.mode == AlgorithmMode::kImuUwb15State) &&
         static_cast<std::uint8_t>(payload.run_state) <=
             static_cast<std::uint8_t>(AlgorithmRunState::kStopped);
}

// payload为告警事件，检查枚举、时间范围及active/cleared与reason_mask的一致性。
bool valid_alert(const AlertPayload& payload) {
  if (payload.alert_code != AlertCode::kNetworkState ||
      static_cast<std::uint8_t>(payload.level) >
          static_cast<std::uint8_t>(AlertLevel::kCritical) ||
      static_cast<std::uint8_t>(payload.lifecycle) >
          static_cast<std::uint8_t>(AlertLifecycle::kCleared) ||
      payload.source != AlertSource::kAlgorithm ||
      payload.first_timestamp_ns > payload.last_timestamp_ns) {
    return false;
  }
  if (payload.lifecycle == AlertLifecycle::kActive) {
    return payload.reason_mask != 0U;
  }
  return payload.reason_mask == 0U;
}

// type为已知消息类型，返回其v1固定载荷字节数；未知值返回0。
std::size_t fixed_payload_size(MessageType type) {
  switch (type) {
    case MessageType::kRange:
      return kRangePayloadSize;
    case MessageType::kImu:
      return kImuPayloadSize;
    case MessageType::kLocalization:
      return kLocalizationPayloadSize;
    case MessageType::kNetwork:
      return kNetworkPayloadSize;
    case MessageType::kObservation:
      return kObservationPayloadSize;
    case MessageType::kAlert:
      return kAlertPayloadSize;
    case MessageType::kAlgorithmStatus:
      return kAlgorithmStatusPayloadSize;
  }
  return 0U;
}

// payload为编码前测距载荷，非法物理量、概率或设备状态通过invalid_argument拒绝。
void require_range_values(const RangePayload& payload) {
  if (!finite(payload.range_m) || !finite(payload.range_std_m) ||
      !finite(static_cast<double>(payload.nlos_probability))) {
    throw std::invalid_argument("range payload contains non-finite data");
  }
  if (payload.range_m <= 0.0 || payload.range_std_m <= 0.0 ||
      !unit_interval(static_cast<double>(payload.nlos_probability)) ||
      payload.status > kRangeStatusInvalid) {
    throw std::invalid_argument("range payload contains invalid data");
  }
}

// payload为编码前定位载荷，检查七个浮点量、状态及2×2协方差半正定。
void require_localization_values(const LocalizationPayload& payload) {
  // values按x,y,vx,vy,cov_xx,cov_xy,cov_yy排列；value逐项检查有限性。
  const double values[] = {payload.x,      payload.y,      payload.vx,
                           payload.vy,     payload.cov_xx, payload.cov_xy,
                           payload.cov_yy};
  for (const double value : values) {
    if (!finite(value)) {
      throw std::invalid_argument(
          "localization payload contains non-finite data");
    }
  }
  if (!valid_localization_state(
          static_cast<std::uint8_t>(payload.state))) {
    throw std::invalid_argument("localization payload state is invalid");
  }
  if (!valid_localization_covariance(payload)) {
    throw std::invalid_argument(
        "localization payload covariance is not positive semidefinite");
  }
}

// payload为编码前网络载荷，检查状态、保留位及拓扑计数关系。
void require_network_values(const NetworkPayload& payload) {
  if (!valid_localization_state(
          static_cast<std::uint8_t>(payload.state))) {
    throw std::invalid_argument("network payload state is invalid");
  }
  if (payload.reserved != 0U) {
    throw std::invalid_argument("network reserved byte must be zero");
  }
  if (!valid_network_topology(payload)) {
    throw std::invalid_argument("network payload topology is inconsistent");
  }
}

// payload为编码前边质量载荷，检查比率、频率、方差倍率、枚举、保留位及滑窗计数。
void require_observation_values(const ObservationPayload& payload) {
  if (!unit_interval(payload.nlos_ratio) ||
      !unit_interval(payload.valid_ratio) ||
      !finite(payload.actual_rate_hz) || payload.actual_rate_hz < 0.0 ||
      !finite(payload.covariance_scale) || payload.covariance_scale <= 0.0) {
    throw std::invalid_argument("observation payload values are invalid");
  }
  if (!valid_observation_state(
          static_cast<std::uint8_t>(payload.state)) ||
      !valid_fusion_action(static_cast<std::uint8_t>(payload.action))) {
    throw std::invalid_argument("observation payload enum is invalid");
  }
  if (payload.reserved != 0U) {
    throw std::invalid_argument("observation reserved byte must be zero");
  }
  if (!valid_observation_counts(payload)) {
    throw std::invalid_argument(
        "observation payload window or counts are inconsistent");
  }
}

// payload为编码前算法状态载荷，检查保留位、版本和运行枚举。
void require_algorithm_status_values(const AlgorithmStatusPayload& payload) {
  if (payload.reserved0 != 0U || payload.reserved1 != 0U) {
    throw std::invalid_argument(
        "algorithm status reserved fields must be zero");
  }
  if (!valid_algorithm_status(payload)) {
    throw std::invalid_argument("algorithm status payload is invalid");
  }
}

// payload为编码前告警载荷，检查全部保留位、枚举、时间和生命周期语义。
void require_alert_values(const AlertPayload& payload) {
  if (payload.reserved0 != 0U || payload.reserved1 != 0U ||
      payload.reserved2 != 0U) {
    throw std::invalid_argument("alert reserved fields must be zero");
  }
  if (!valid_alert(payload)) {
    throw std::invalid_argument("alert payload is invalid");
  }
}

}  // namespace

bool is_known_message_type(MessageType type) noexcept {
  switch (type) {
    case MessageType::kRange:
    case MessageType::kImu:
    case MessageType::kLocalization:
    case MessageType::kNetwork:
    case MessageType::kObservation:
    case MessageType::kAlert:
    case MessageType::kAlgorithmStatus:
      return true;
  }
  return false;
}

std::vector<std::uint8_t> encode_frame(const Frame& frame,
                                       std::size_t max_payload_size) {
  // 阶段1：fixed_size由消息类型确定，用它和max_payload_size同时约束调用方payload后再分配。
  if (max_payload_size > kDefaultMaxPayloadSize) {
    throw std::invalid_argument(
        "wire payload limit exceeds the protocol hard limit");
  }
  if (!is_known_message_type(frame.header.message_type)) {
    throw std::invalid_argument("unknown wire message type");
  }
  const std::size_t fixed_size =
      fixed_payload_size(frame.header.message_type);
  if (fixed_size != 0U && frame.payload.size() != fixed_size) {
    throw std::invalid_argument(
        "wire payload size does not match its message type");
  }
  if (frame.payload.size() > max_payload_size ||
      frame.payload.size() >
          static_cast<std::size_t>(
              std::numeric_limits<std::uint32_t>::max())) {
    throw std::invalid_argument("wire payload exceeds configured limit");
  }

  // output是最终40字节头+payload目标缓冲：先顺序编码主机字段并写CRC零占位，最后回填。
  std::vector<std::uint8_t> output;
  output.reserve(kWireHeaderSize + frame.payload.size());
  output.insert(output.end(), kMagic.begin(), kMagic.end());
  output.push_back(kProtocolMajorVersion);
  output.push_back(kProtocolMinorVersion);
  append_u16(output,
             static_cast<std::uint16_t>(frame.header.message_type));
  append_u16(output, static_cast<std::uint16_t>(kWireHeaderSize));
  append_u16(output, frame.header.flags);
  append_u32(output, static_cast<std::uint32_t>(frame.payload.size()));
  append_u64(output, frame.header.sequence);
  append_u64(output, frame.header.timestamp_ns);
  append_u16(output, frame.header.source_node);
  append_u16(output, frame.header.target_node);
  append_u32(output, 0U);
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());

  // 阶段2：完整帧先保留CRC占位，crc_input明确跳过[36,40)字段，
  // 对头部[0,36)紧接载荷求CRC，再回填固定偏移；解码端使用完全相同的拼接规则。
  write_u32(output, kCrcOffset, crc32_ieee(crc_input(output)));
  return output;
}

FrameDecodeResult decode_frame(const std::vector<std::uint8_t>& bytes,
                               std::size_t max_payload_size) {
  // 阶段1：先验证总长度和魔数/版本；message_type决定fixed_size，payload_size与
  // expected_size再区分资源超限、类型长度不符、截断和尾随字节。
  if (max_payload_size > kDefaultMaxPayloadSize) {
    return failure(ProtocolError::kPayloadTooLarge,
                   "wire payload limit exceeds the protocol hard limit");
  }
  if (bytes.size() < kWireHeaderSize) {
    return failure(ProtocolError::kTruncated,
                   "wire frame is shorter than its fixed header");
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), bytes.begin())) {
    return failure(ProtocolError::kBadMagic, "wire magic does not match");
  }
  if (bytes[4U] != kProtocolMajorVersion ||
      bytes[5U] != kProtocolMinorVersion) {
    return failure(ProtocolError::kUnsupportedVersion,
                   "wire protocol version is unsupported");
  }
  const MessageType message_type =
      static_cast<MessageType>(read_u16(bytes, 6U));
  if (!is_known_message_type(message_type)) {
    return failure(ProtocolError::kUnknownMessageType,
                   "wire message type is unknown");
  }
  if (read_u16(bytes, 8U) != kWireHeaderSize) {
    return failure(ProtocolError::kInvalidHeaderSize,
                   "wire header size is not 40 bytes");
  }

  const std::uint32_t payload_size = read_u32(bytes, 12U);
  if (static_cast<std::size_t>(payload_size) > max_payload_size) {
    return failure(ProtocolError::kPayloadTooLarge,
                   "wire payload exceeds configured limit");
  }
  const std::size_t fixed_size = fixed_payload_size(message_type);
  if (fixed_size != 0U &&
      static_cast<std::size_t>(payload_size) != fixed_size) {
    return failure(ProtocolError::kInvalidPayloadSize,
                   "wire payload size does not match its message type");
  }
  const std::size_t expected_size =
      kWireHeaderSize + static_cast<std::size_t>(payload_size);
  if (bytes.size() < expected_size) {
    return failure(ProtocolError::kTruncated,
                   "wire frame payload is truncated");
  }
  if (bytes.size() > expected_size) {
    return failure(ProtocolError::kTrailingBytes,
                   "wire frame has trailing bytes");
  }

  // 阶段2：encoded_crc取头[36,40)，复算严格覆盖头[0,36)+payload；通过后才构造Frame。
  // 具体载荷的有限值、布尔和枚举由对应decode_*另行验证。
  const std::uint32_t encoded_crc = read_u32(bytes, kCrcOffset);
  if (crc32_ieee(crc_input(bytes)) != encoded_crc) {
    return failure(ProtocolError::kCrcMismatch,
                   "wire frame CRC32 does not match");
  }

  // result在外层校验完成后拥有主机序头和payload副本，不借用bytes。
  FrameDecodeResult result{};
  result.value.header.message_type = message_type;
  result.value.header.flags = read_u16(bytes, 10U);
  result.value.header.payload_size = payload_size;
  result.value.header.sequence = read_u64(bytes, 16U);
  result.value.header.timestamp_ns = read_u64(bytes, 24U);
  result.value.header.source_node = read_u16(bytes, 32U);
  result.value.header.target_node = read_u16(bytes, 34U);
  result.value.header.crc32 = encoded_crc;
  result.value.payload.assign(
      bytes.begin() + static_cast<std::ptrdiff_t>(kWireHeaderSize),
      bytes.end());
  return result;
}

std::vector<std::uint8_t> encode_range_payload(
    const RangePayload& payload) {
  // 测距固定载荷同时携带标准差和NLOS证据，质量字段不能在传输层丢弃。
  require_range_values(payload);
  // output按24字节固定顺序拥有编码结果。
  std::vector<std::uint8_t> output;
  output.reserve(kRangePayloadSize);
  append_double(output, payload.range_m);
  append_double(output, payload.range_std_m);
  append_float(output, payload.nlos_probability);
  output.push_back(payload.nlos_flag ? 1U : 0U);
  output.push_back(payload.has_nlos_probability ? 1U : 0U);
  output.push_back(payload.valid ? 1U : 0U);
  output.push_back(payload.status);
  return output;
}

PayloadDecodeResult<RangePayload> decode_range_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kRangePayloadSize) {
    return payload_failure<RangePayload>(
        ProtocolError::kInvalidPayloadSize,
        "range payload must be exactly 24 bytes");
  }
  // result逐字段填充，全部布尔和物理语义通过后才返回成功。
  PayloadDecodeResult<RangePayload> result{};
  result.value.range_m = read_double(bytes, 0U);
  result.value.range_std_m = read_double(bytes, 8U);
  result.value.nlos_probability = read_float(bytes, 16U);
  if (!decode_boolean(bytes[20U], result.value.nlos_flag) ||
      !decode_boolean(bytes[21U], result.value.has_nlos_probability) ||
      !decode_boolean(bytes[22U], result.value.valid)) {
    return payload_failure<RangePayload>(
        ProtocolError::kInvalidBoolean,
        "range payload boolean must be zero or one");
  }
  result.value.status = bytes[23U];
  if (!finite(result.value.range_m) ||
      !finite(result.value.range_std_m) ||
      !finite(static_cast<double>(result.value.nlos_probability))) {
    return payload_failure<RangePayload>(
        ProtocolError::kNonFiniteValue,
        "range payload contains non-finite data");
  }
  if (result.value.range_m <= 0.0 || result.value.range_std_m <= 0.0 ||
      !unit_interval(
          static_cast<double>(result.value.nlos_probability)) ||
      result.value.status > kRangeStatusInvalid) {
    return payload_failure<RangePayload>(
        ProtocolError::kInvalidValue,
        "range payload contains invalid data");
  }
  return result;
}

std::vector<std::uint8_t> encode_imu_payload(const ImuPayload& payload) {
  // IMU载荷保持ROS 2 Imu瞬时量语义；frame_id固定32字节且必须NUL结束。
  if (!valid_imu_payload(payload)) {
    throw std::invalid_argument("IMU payload contains invalid data");
  }
  // output按六组double数组、32字节帧名和4个尾字段顺序累计332字节。
  std::vector<std::uint8_t> output;
  output.reserve(kImuPayloadSize);
  // append_values捕获可写output；values为当前固定数组，value遍历其double元素。
  const auto append_values = [&output](const auto& values) {
    for (const double value : values) {
      append_double(output, value);
    }
  };
  append_values(payload.orientation_xyzw);
  append_values(payload.orientation_covariance);
  append_values(payload.angular_velocity_rad_s);
  append_values(payload.angular_velocity_covariance);
  append_values(payload.linear_acceleration_m_s2);
  append_values(payload.linear_acceleration_covariance);
  // value遍历frame_id全部32个原始字符字节，不因提前NUL缩短线序布局。
  for (const char value : payload.frame_id) {
    output.push_back(static_cast<std::uint8_t>(value));
  }
  output.push_back(payload.orientation_valid ? 1U : 0U);
  output.push_back(payload.valid ? 1U : 0U);
  output.push_back(payload.status);
  output.push_back(payload.reserved);
  return output;
}

PayloadDecodeResult<ImuPayload> decode_imu_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kImuPayloadSize) {
    return payload_failure<ImuPayload>(
        ProtocolError::kInvalidPayloadSize,
        "IMU payload must be exactly 332 bytes");
  }
  // result为逐字段构造的IMU值；offset为332字节载荷的当前读取游标。
  PayloadDecodeResult<ImuPayload> result{};
  std::size_t offset = 0U;
  // read_values捕获只读bytes和可写offset；values为目标数组，value遍历其可写double元素。
  const auto read_values = [&bytes, &offset](auto& values) {
    for (double& value : values) {
      value = read_double(bytes, offset);
      offset += sizeof(double);
    }
  };
  read_values(result.value.orientation_xyzw);
  read_values(result.value.orientation_covariance);
  read_values(result.value.angular_velocity_rad_s);
  read_values(result.value.angular_velocity_covariance);
  read_values(result.value.linear_acceleration_m_s2);
  read_values(result.value.linear_acceleration_covariance);
  // value遍历目标frame_id的32个字符槽并推进载荷游标。
  for (char& value : result.value.frame_id) {
    value = static_cast<char>(bytes[offset++]);
  }
  if (!decode_boolean(bytes[offset++], result.value.orientation_valid) ||
      !decode_boolean(bytes[offset++], result.value.valid)) {
    return payload_failure<ImuPayload>(
        ProtocolError::kInvalidBoolean,
        "IMU payload boolean must be zero or one");
  }
  result.value.status = bytes[offset++];
  result.value.reserved = bytes[offset];
  if (!finite_array(result.value.orientation_xyzw) ||
      !finite_array(result.value.orientation_covariance) ||
      !finite_array(result.value.angular_velocity_rad_s) ||
      !finite_array(result.value.angular_velocity_covariance) ||
      !finite_array(result.value.linear_acceleration_m_s2) ||
      !finite_array(result.value.linear_acceleration_covariance)) {
    return payload_failure<ImuPayload>(
        ProtocolError::kNonFiniteValue,
        "IMU payload contains non-finite data");
  }
  if (result.value.reserved != 0U) {
    return payload_failure<ImuPayload>(
        ProtocolError::kInvalidReserved,
        "IMU payload reserved byte must be zero");
  }
  if (!valid_imu_payload(result.value)) {
    return payload_failure<ImuPayload>(
        ProtocolError::kInvalidValue, "IMU payload contains invalid data");
  }
  return result;
}

std::vector<std::uint8_t> encode_localization_payload(
    const LocalizationPayload& payload) {
  // 定位输出同时编码有效位与能力位，未实现yaw/z时必须明确标记无效。
  require_localization_values(payload);
  // output按7个double、状态/有效位和能力位顺序累计64字节。
  std::vector<std::uint8_t> output;
  output.reserve(kLocalizationPayloadSize);
  append_double(output, payload.x);
  append_double(output, payload.y);
  append_double(output, payload.vx);
  append_double(output, payload.vy);
  append_double(output, payload.cov_xx);
  append_double(output, payload.cov_xy);
  append_double(output, payload.cov_yy);
  output.push_back(static_cast<std::uint8_t>(payload.state));
  output.push_back(payload.valid ? 1U : 0U);
  output.push_back(payload.yaw_valid ? 1U : 0U);
  output.push_back(payload.z_valid ? 1U : 0U);
  append_u32(output, payload.capability_mask);
  return output;
}

PayloadDecodeResult<LocalizationPayload> decode_localization_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kLocalizationPayloadSize) {
    return payload_failure<LocalizationPayload>(
        ProtocolError::kInvalidPayloadSize,
        "localization payload must be exactly 64 bytes");
  }
  // result为按固定偏移恢复的定位载荷，成功前完成有限性、协方差和枚举检查。
  PayloadDecodeResult<LocalizationPayload> result{};
  result.value.x = read_double(bytes, 0U);
  result.value.y = read_double(bytes, 8U);
  result.value.vx = read_double(bytes, 16U);
  result.value.vy = read_double(bytes, 24U);
  result.value.cov_xx = read_double(bytes, 32U);
  result.value.cov_xy = read_double(bytes, 40U);
  result.value.cov_yy = read_double(bytes, 48U);
  // values按位置、速度和协方差字段排列；value逐项检查有限性。
  const double values[] = {
      result.value.x,      result.value.y,      result.value.vx,
      result.value.vy,     result.value.cov_xx, result.value.cov_xy,
      result.value.cov_yy};
  for (const double value : values) {
    if (!finite(value)) {
      return payload_failure<LocalizationPayload>(
          ProtocolError::kNonFiniteValue,
          "localization payload contains non-finite data");
    }
  }
  if (!valid_localization_covariance(result.value)) {
    return payload_failure<LocalizationPayload>(
        ProtocolError::kInvalidValue,
        "localization payload covariance is not positive semidefinite");
  }
  if (!valid_localization_state(bytes[56U])) {
    return payload_failure<LocalizationPayload>(
        ProtocolError::kInvalidValue,
        "localization payload state is invalid");
  }
  result.value.state = static_cast<LocalizationState>(bytes[56U]);
  if (!decode_boolean(bytes[57U], result.value.valid) ||
      !decode_boolean(bytes[58U], result.value.yaw_valid) ||
      !decode_boolean(bytes[59U], result.value.z_valid)) {
    return payload_failure<LocalizationPayload>(
        ProtocolError::kInvalidBoolean,
        "localization payload boolean must be zero or one");
  }
  result.value.capability_mask = read_u32(bytes, 60U);
  return result;
}

std::vector<std::uint8_t> encode_network_payload(
    const NetworkPayload& payload) {
  require_network_values(payload);
  // output按计数、布尔、状态、保留位和原因位图累计20字节。
  std::vector<std::uint8_t> output;
  output.reserve(kNetworkPayloadSize);
  append_u32(output, payload.node_count);
  append_u32(output, payload.reachable_node_count);
  append_u32(output, payload.active_edge_count);
  output.push_back(payload.connected ? 1U : 0U);
  output.push_back(payload.observable ? 1U : 0U);
  output.push_back(static_cast<std::uint8_t>(payload.state));
  output.push_back(payload.reserved);
  append_u32(output, payload.reason_mask);
  return output;
}

PayloadDecodeResult<NetworkPayload> decode_network_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kNetworkPayloadSize) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidPayloadSize,
        "network payload must be exactly 20 bytes");
  }
  // result为固定偏移恢复的网络载荷，返回前验证布尔、状态、保留位和拓扑。
  PayloadDecodeResult<NetworkPayload> result{};
  result.value.node_count = read_u32(bytes, 0U);
  result.value.reachable_node_count = read_u32(bytes, 4U);
  result.value.active_edge_count = read_u32(bytes, 8U);
  if (!decode_boolean(bytes[12U], result.value.connected) ||
      !decode_boolean(bytes[13U], result.value.observable)) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidBoolean,
        "network payload boolean must be zero or one");
  }
  if (!valid_localization_state(bytes[14U])) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidValue,
        "network payload state is invalid");
  }
  result.value.state = static_cast<LocalizationState>(bytes[14U]);
  if (bytes[15U] != 0U) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidReserved,
        "network reserved byte must be zero");
  }
  result.value.reason_mask = read_u32(bytes, 16U);
  if (!valid_network_topology(result.value)) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidValue,
        "network payload topology is inconsistent");
  }
  return result;
}

std::vector<std::uint8_t> encode_observation_payload(
    const ObservationPayload& payload) {
  // 保留滑窗计数、原因位图和实际融合动作，便于GCS审计退化判定。
  require_observation_values(payload);
  // output按窗口、六个计数、四个double和尾部状态字段累计80字节。
  std::vector<std::uint8_t> output;
  output.reserve(kObservationPayloadSize);
  append_u64(output, payload.window_start_ns);
  append_u64(output, payload.window_end_ns);
  append_u32(output, payload.expected_count);
  append_u32(output, payload.received_count);
  append_u32(output, payload.valid_count);
  append_u32(output, payload.nlos_count);
  append_u32(output, payload.residual_rejected_count);
  append_u32(output, payload.dropped_count);
  append_double(output, payload.nlos_ratio);
  append_double(output, payload.valid_ratio);
  append_double(output, payload.actual_rate_hz);
  append_double(output, payload.covariance_scale);
  output.push_back(static_cast<std::uint8_t>(payload.state));
  output.push_back(static_cast<std::uint8_t>(payload.action));
  output.push_back(payload.input_overflow ? 1U : 0U);
  output.push_back(payload.reserved);
  append_u32(output, payload.reason_mask);
  return output;
}

PayloadDecodeResult<ObservationPayload> decode_observation_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kObservationPayloadSize) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidPayloadSize,
        "observation payload must be exactly 80 bytes");
  }
  // result为固定偏移恢复的边质量载荷，成功前校验所有比率、枚举和计数关系。
  PayloadDecodeResult<ObservationPayload> result{};
  result.value.window_start_ns = read_u64(bytes, 0U);
  result.value.window_end_ns = read_u64(bytes, 8U);
  result.value.expected_count = read_u32(bytes, 16U);
  result.value.received_count = read_u32(bytes, 20U);
  result.value.valid_count = read_u32(bytes, 24U);
  result.value.nlos_count = read_u32(bytes, 28U);
  result.value.residual_rejected_count = read_u32(bytes, 32U);
  result.value.dropped_count = read_u32(bytes, 36U);
  result.value.nlos_ratio = read_double(bytes, 40U);
  result.value.valid_ratio = read_double(bytes, 48U);
  result.value.actual_rate_hz = read_double(bytes, 56U);
  result.value.covariance_scale = read_double(bytes, 64U);
  if (!finite(result.value.nlos_ratio) ||
      !finite(result.value.valid_ratio) ||
      !finite(result.value.actual_rate_hz) ||
      !finite(result.value.covariance_scale)) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kNonFiniteValue,
        "observation payload contains non-finite data");
  }
  if (!unit_interval(result.value.nlos_ratio) ||
      !unit_interval(result.value.valid_ratio) ||
      result.value.actual_rate_hz < 0.0 ||
      result.value.covariance_scale <= 0.0) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidValue,
        "observation payload values are invalid");
  }
  if (!valid_observation_counts(result.value)) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidValue,
        "observation payload window or counts are inconsistent");
  }
  if (!valid_observation_state(bytes[72U]) ||
      !valid_fusion_action(bytes[73U])) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidValue,
        "observation payload enum is invalid");
  }
  result.value.state = static_cast<ObservationState>(bytes[72U]);
  result.value.action = static_cast<FusionAction>(bytes[73U]);
  if (!decode_boolean(bytes[74U], result.value.input_overflow)) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidBoolean,
        "observation payload boolean must be zero or one");
  }
  if (bytes[75U] != 0U) {
    return payload_failure<ObservationPayload>(
        ProtocolError::kInvalidReserved,
        "observation reserved byte must be zero");
  }
  result.value.reason_mask = read_u32(bytes, 76U);
  return result;
}

std::vector<std::uint8_t> encode_alert_payload(
    const AlertPayload& payload) {
  // 告警使用稳定code/lifecycle/reason_mask，文本解释由GCS按版本映射。
  require_alert_values(payload);
  // output按告警标识、节点/边和首次/最近时间累计40字节。
  std::vector<std::uint8_t> output;
  output.reserve(kAlertPayloadSize);
  append_u32(output, static_cast<std::uint32_t>(payload.alert_code));
  output.push_back(static_cast<std::uint8_t>(payload.level));
  output.push_back(static_cast<std::uint8_t>(payload.lifecycle));
  output.push_back(static_cast<std::uint8_t>(payload.source));
  output.push_back(payload.reserved0);
  append_u32(output, payload.reason_mask);
  append_u16(output, payload.node_id);
  append_u16(output, payload.from_node);
  append_u16(output, payload.to_node);
  append_u16(output, payload.reserved1);
  append_u64(output, payload.first_timestamp_ns);
  append_u64(output, payload.last_timestamp_ns);
  append_u32(output, payload.reserved2);
  return output;
}

PayloadDecodeResult<AlertPayload> decode_alert_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kAlertPayloadSize) {
    return payload_failure<AlertPayload>(
        ProtocolError::kInvalidPayloadSize,
        "alert payload must be exactly 40 bytes");
  }
  // result为固定偏移恢复的告警，返回前检查保留位和生命周期语义。
  PayloadDecodeResult<AlertPayload> result{};
  result.value.alert_code = static_cast<AlertCode>(read_u32(bytes, 0U));
  result.value.level = static_cast<AlertLevel>(bytes[4U]);
  result.value.lifecycle = static_cast<AlertLifecycle>(bytes[5U]);
  result.value.source = static_cast<AlertSource>(bytes[6U]);
  result.value.reserved0 = bytes[7U];
  result.value.reason_mask = read_u32(bytes, 8U);
  result.value.node_id = read_u16(bytes, 12U);
  result.value.from_node = read_u16(bytes, 14U);
  result.value.to_node = read_u16(bytes, 16U);
  result.value.reserved1 = read_u16(bytes, 18U);
  result.value.first_timestamp_ns = read_u64(bytes, 20U);
  result.value.last_timestamp_ns = read_u64(bytes, 28U);
  result.value.reserved2 = read_u32(bytes, 36U);
  if (result.value.reserved0 != 0U || result.value.reserved1 != 0U ||
      result.value.reserved2 != 0U) {
    return payload_failure<AlertPayload>(
        ProtocolError::kInvalidReserved,
        "alert reserved fields must be zero");
  }
  if (!valid_alert(result.value)) {
    return payload_failure<AlertPayload>(
        ProtocolError::kInvalidValue, "alert payload is invalid");
  }
  return result;
}

std::vector<std::uint8_t> encode_algorithm_status_payload(
    const AlgorithmStatusPayload& payload) {
  require_algorithm_status_values(payload);
  // output按版本、模式、累计计数和运行时长累计48字节。
  std::vector<std::uint8_t> output;
  output.reserve(kAlgorithmStatusPayloadSize);
  append_u32(output, payload.abi_version);
  append_u32(output, payload.software_version_packed);
  output.push_back(static_cast<std::uint8_t>(payload.mode));
  output.push_back(static_cast<std::uint8_t>(payload.run_state));
  append_u16(output, payload.reserved0);
  append_u32(output, payload.reserved1);
  append_u64(output, payload.accepted_ranges);
  append_u64(output, payload.rejected_ranges);
  append_u64(output, payload.protocol_errors);
  append_u64(output, payload.uptime_ns);
  return output;
}

PayloadDecodeResult<AlgorithmStatusPayload>
decode_algorithm_status_payload(const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kAlgorithmStatusPayloadSize) {
    return payload_failure<AlgorithmStatusPayload>(
        ProtocolError::kInvalidPayloadSize,
        "algorithm status payload must be exactly 48 bytes");
  }
  // result为固定偏移恢复的进程状态，返回前检查保留位、ABI和枚举。
  PayloadDecodeResult<AlgorithmStatusPayload> result{};
  result.value.abi_version = read_u32(bytes, 0U);
  result.value.software_version_packed = read_u32(bytes, 4U);
  result.value.mode = static_cast<AlgorithmMode>(bytes[8U]);
  result.value.run_state = static_cast<AlgorithmRunState>(bytes[9U]);
  result.value.reserved0 = read_u16(bytes, 10U);
  result.value.reserved1 = read_u32(bytes, 12U);
  result.value.accepted_ranges = read_u64(bytes, 16U);
  result.value.rejected_ranges = read_u64(bytes, 24U);
  result.value.protocol_errors = read_u64(bytes, 32U);
  result.value.uptime_ns = read_u64(bytes, 40U);
  if (result.value.reserved0 != 0U || result.value.reserved1 != 0U) {
    return payload_failure<AlgorithmStatusPayload>(
        ProtocolError::kInvalidReserved,
        "algorithm status reserved fields must be zero");
  }
  if (!valid_algorithm_status(result.value)) {
    return payload_failure<AlgorithmStatusPayload>(
        ProtocolError::kInvalidValue,
        "algorithm status payload is invalid");
  }
  return result;
}

}  // namespace zju::coop::protocol

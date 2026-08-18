// 模块实现：ZJCL公共帧头和各固定载荷的小端编解码、CRC校验与字段语义校验。
// 关键原则：编码端拒绝不可表示数据，解码端按“长度→头部→CRC→载荷语义”顺序失败；
// C++与Python实现共享固定字节布局，任何新增字段必须升级协议版本或使用新消息类型。
//
// C++初学者可先区分两组函数：
// - append_*：把一个整数/浮点数拆成字节，依次追加到vector末尾；
// - read_*：从当前offset读取字节并还原数值，同时向后移动offset。
// 公开encode_*和decode_*只是在这些基础积木上按协议字段顺序组装或拆分。
// `memcpy`只复制浮点数位模式而不做数值转换；`static_assert`在编译期确认类型尺寸；
// `std::variant/std::get`保证载荷是受控类型集合，而不是无类型void指针。
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
constexpr double kPi = 3.141592653589793238462643383279502884;

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
    case MessageType::kPose2D:
      return kPose2DPayloadSize;
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

// payload为编码前二维位姿载荷；三个物理量须有限，保留位须维持零值。
void require_pose2d_values(const Pose2DPayload& payload) {
  if (!finite(payload.x) || !finite(payload.y) ||
      !finite(payload.yaw_rad)) {
    throw std::invalid_argument("pose2d payload contains non-finite data");
  }
  if (payload.yaw_rad < -kPi || payload.yaw_rad >= kPi) {
    throw std::invalid_argument("pose2d yaw must be in [-pi, pi)");
  }
  if (payload.reserved != 0U) {
    throw std::invalid_argument("pose2d reserved field must be zero");
  }
  const auto planar_bit =
      static_cast<std::uint32_t>(Capability::kPlanarPosition);
  const auto yaw_bit = static_cast<std::uint32_t>(Capability::kYaw);
  if ((payload.position_valid &&
       (payload.capability_mask & planar_bit) == 0U) ||
      (payload.yaw_valid && (payload.capability_mask & yaw_bit) == 0U)) {
    throw std::invalid_argument(
        "pose2d valid field is missing its capability bit");
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
    case MessageType::kPose2D:
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
  output.reserve(kWireHeaderSize + frame.payload.size());  // reserve只预留容量，不改变size，避免后续追加时多次分配。
  output.insert(output.end(), kMagic.begin(), kMagic.end());  // 先写4字节ZJCL魔数，接收端据此识别本协议。
  output.push_back(kProtocolMajorVersion);  // 第5字节写主版本；不兼容修改必须提升主版本。
  output.push_back(kProtocolMinorVersion);  // 第6字节写次版本；兼容扩展可提升次版本。
  append_u16(output,
             static_cast<std::uint16_t>(frame.header.message_type));
  // 上一行把强类型enum class显式转为16位线序消息类型，避免隐式收窄。
  append_u16(output, static_cast<std::uint16_t>(kWireHeaderSize));  // 头长固定40字节，便于接收端拒绝布局不一致。
  append_u16(output, frame.header.flags);  // 预留标志位可表达以后兼容功能。
  append_u32(output, static_cast<std::uint32_t>(frame.payload.size()));  // 载荷长度已验证可安全收窄为32位。
  append_u64(output, frame.header.sequence);  // 序号用于丢包、乱序和重复检测。
  append_u64(output, frame.header.timestamp_ns);  // 统一时间轴上的测量/结果时刻。
  append_u16(output, frame.header.source_node);  // 消息来源平台编号。
  append_u16(output, frame.header.target_node);  // 目标平台编号；广播语义由接口文档约定。
  append_u32(output, 0U);  // CRC字段先写零占位，因为必须等完整载荷写完才能计算。
  output.insert(output.end(), frame.payload.begin(), frame.payload.end());  // 将已编码载荷原样追加到固定头后。

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
  result.value.header.message_type = message_type;  // 保存已验证属于已知集合的消息类型。
  result.value.header.flags = read_u16(bytes, 10U);  // 从固定偏移恢复标志位。
  result.value.header.payload_size = payload_size;  // 保存已与总帧长核对过的载荷长度。
  result.value.header.sequence = read_u64(bytes, 16U);  // 恢复发送端单调序号。
  result.value.header.timestamp_ns = read_u64(bytes, 24U);  // 恢复统一时间戳。
  result.value.header.source_node = read_u16(bytes, 32U);  // 恢复来源节点号。
  result.value.header.target_node = read_u16(bytes, 34U);  // 恢复目标节点号。
  result.value.header.crc32 = encoded_crc;  // 保存已复算通过的CRC，供日志或诊断查看。
  result.value.payload.assign(
      bytes.begin() + static_cast<std::ptrdiff_t>(kWireHeaderSize),
      bytes.end());
  return result;  // error保持默认kNone，表示外层帧验证成功。
}

std::vector<std::uint8_t> encode_range_payload(
    const RangePayload& payload) {
  // 测距固定载荷同时携带标准差和NLOS证据，质量字段不能在传输层丢弃。
  require_range_values(payload);
  // output按24字节固定顺序拥有编码结果。
  std::vector<std::uint8_t> output;
  output.reserve(kRangePayloadSize);  // 精确预留24字节，避免逐字段追加触发扩容。
  append_double(output, payload.range_m);  // 偏移0：节点间距离，单位米。
  append_double(output, payload.range_std_m);  // 偏移8：测距1σ标准差，单位米。
  append_float(output, payload.nlos_probability);  // 偏移16：NLOS概率，用float节省线序空间。
  output.push_back(payload.nlos_flag ? 1U : 0U);  // 偏移20：明确NLOS标志；三目运算保证只写0或1。
  output.push_back(payload.has_nlos_probability ? 1U : 0U);  // 偏移21：说明概率字段是否由设备真实提供。
  output.push_back(payload.valid ? 1U : 0U);  // 偏移22：设备/适配层对本包的有效性结论。
  output.push_back(payload.status);  // 偏移23：传感器状态码。
  return output;  // vector按值返回；编译器可移动或消除复制。
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
  result.value.range_m = read_double(bytes, 0U);  // 按协议偏移0恢复距离。
  result.value.range_std_m = read_double(bytes, 8U);  // 按协议偏移8恢复标准差。
  result.value.nlos_probability = read_float(bytes, 16U);  // 按协议偏移16恢复单精度概率。
  if (!decode_boolean(bytes[20U], result.value.nlos_flag) ||
      !decode_boolean(bytes[21U], result.value.has_nlos_probability) ||
      !decode_boolean(bytes[22U], result.value.valid)) {
    return payload_failure<RangePayload>(
        ProtocolError::kInvalidBoolean,
        "range payload boolean must be zero or one");
  }
  result.value.status = bytes[23U];  // 最后1字节是设备状态码，随后再做取值范围检查。
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
  return result;  // 所有长度、布尔、有限性和物理范围检查都通过才走到这里。
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
  append_values(payload.orientation_xyzw);  // 先写4个xyzw姿态分量，与ROS 2 Imu顺序一致。
  append_values(payload.orientation_covariance);  // 再写9个姿态协方差元素，保持行主序。
  append_values(payload.angular_velocity_rad_s);  // 写3轴瞬时角速度，单位rad/s。
  append_values(payload.angular_velocity_covariance);  // 写3×3角速度协方差。
  append_values(payload.linear_acceleration_m_s2);  // 写3轴瞬时线加速度，单位m/s²。
  append_values(payload.linear_acceleration_covariance);  // 写3×3线加速度协方差。
  // value遍历frame_id全部32个原始字符字节，不因提前NUL缩短线序布局。
  for (const char value : payload.frame_id) {
    output.push_back(static_cast<std::uint8_t>(value));
  }
  output.push_back(payload.orientation_valid ? 1U : 0U);  // 标明orientation是否能用于首次姿态初始化。
  output.push_back(payload.valid ? 1U : 0U);  // 标明整包IMU瞬时量是否有效。
  output.push_back(payload.status);  // 写入传感器状态码。
  output.push_back(payload.reserved);  // v1保留字节必须为0，为未来兼容扩展留位置。
  return output;  // 完成固定332字节载荷。
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
  read_values(result.value.orientation_xyzw);  // 按编码顺序读取4个姿态分量。
  read_values(result.value.orientation_covariance);  // 接着读取9个姿态协方差元素。
  read_values(result.value.angular_velocity_rad_s);  // 接着读取3轴角速度。
  read_values(result.value.angular_velocity_covariance);  // 接着读取9个角速度协方差元素。
  read_values(result.value.linear_acceleration_m_s2);  // 接着读取3轴线加速度。
  read_values(result.value.linear_acceleration_covariance);  // 最后读取9个线加速度协方差元素。
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
  result.value.status = bytes[offset++];  // 读取状态码并让游标前移到最后一个保留字节。
  result.value.reserved = bytes[offset];  // 最后无需再递增offset，因为后面不再读取载荷。
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
  return result;  // error保持无错误，value包含独立的主机序IMU副本。
}

std::vector<std::uint8_t> encode_localization_payload(
    const LocalizationPayload& payload) {
  // 定位输出同时编码有效位与能力位，未实现yaw/z时必须明确标记无效。
  require_localization_values(payload);
  // output按7个double、状态/有效位和能力位顺序累计64字节。
  std::vector<std::uint8_t> output;
  output.reserve(kLocalizationPayloadSize);  // 精确预留64字节定位载荷。
  append_double(output, payload.x);  // 偏移0：相对参考节点的东向位置。
  append_double(output, payload.y);  // 偏移8：相对参考节点的北向位置。
  append_double(output, payload.vx);  // 偏移16：东向相对速度。
  append_double(output, payload.vy);  // 偏移24：北向相对速度。
  append_double(output, payload.cov_xx);  // 偏移32：x位置方差。
  append_double(output, payload.cov_xy);  // 偏移40：x-y位置互协方差。
  append_double(output, payload.cov_yy);  // 偏移48：y位置方差。
  output.push_back(static_cast<std::uint8_t>(payload.state));  // 偏移56：强类型定位状态显式转为1字节线序值。
  output.push_back(payload.valid ? 1U : 0U);  // 偏移57：整条定位结果是否可用。
  output.push_back(payload.yaw_valid ? 1U : 0U);  // 偏移58：航向字段能力标志。
  output.push_back(payload.z_valid ? 1U : 0U);  // 偏移59：高度字段能力标志。
  append_u32(output, payload.capability_mask);  // 偏移60：算法当前支持能力的位掩码。
  return output;  // 返回固定64字节载荷。
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
  result.value.x = read_double(bytes, 0U);  // 从偏移0恢复x位置。
  result.value.y = read_double(bytes, 8U);  // 从偏移8恢复y位置。
  result.value.vx = read_double(bytes, 16U);  // 从偏移16恢复x速度。
  result.value.vy = read_double(bytes, 24U);  // 从偏移24恢复y速度。
  result.value.cov_xx = read_double(bytes, 32U);  // 从偏移32恢复x方差。
  result.value.cov_xy = read_double(bytes, 40U);  // 从偏移40恢复xy互协方差。
  result.value.cov_yy = read_double(bytes, 48U);  // 从偏移48恢复y方差。
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
  result.value.state = static_cast<LocalizationState>(bytes[56U]);  // 枚举范围已检查，现可安全恢复强类型枚举。
  if (!decode_boolean(bytes[57U], result.value.valid) ||
      !decode_boolean(bytes[58U], result.value.yaw_valid) ||
      !decode_boolean(bytes[59U], result.value.z_valid)) {
    return payload_failure<LocalizationPayload>(
        ProtocolError::kInvalidBoolean,
        "localization payload boolean must be zero or one");
  }
  result.value.capability_mask = read_u32(bytes, 60U);  // 恢复最后4字节能力位图。
  return result;  // 成功结果包含经过有限性和半正定检查的定位数据。
}

std::vector<std::uint8_t> encode_pose2d_payload(
    const Pose2DPayload& payload) {
  // 先检查有限性和保留位，再按冻结的小端偏移构造恰好32字节载荷。
  require_pose2d_values(payload);
  std::vector<std::uint8_t> output;
  output.reserve(kPose2DPayloadSize);
  append_double(output, payload.x);  // 偏移0：ENU东向相对位置，单位m。
  append_double(output, payload.y);  // 偏移8：ENU北向相对位置，单位m。
  append_double(output, payload.yaw_rad);  // 偏移16：ENU平面航向，单位rad。
  output.push_back(payload.position_valid ? 1U : 0U);  // 偏移24：二维位置有效位。
  output.push_back(payload.yaw_valid ? 1U : 0U);  // 偏移25：航向有效位。
  append_u16(output, payload.reserved);  // 偏移26：必须为零的16位扩展保留槽。
  append_u32(output, payload.capability_mask);  // 偏移28：能力位图。
  return output;
}

PayloadDecodeResult<Pose2DPayload> decode_pose2d_payload(
    const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() != kPose2DPayloadSize) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kInvalidPayloadSize,
        "pose2d payload must be exactly 32 bytes");
  }
  // 固定偏移恢复三个double；任何NaN/Inf都在进入展示层前拒绝。
  PayloadDecodeResult<Pose2DPayload> result{};
  result.value.x = read_double(bytes, 0U);
  result.value.y = read_double(bytes, 8U);
  result.value.yaw_rad = read_double(bytes, 16U);
  if (!finite(result.value.x) || !finite(result.value.y) ||
      !finite(result.value.yaw_rad)) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kNonFiniteValue,
        "pose2d payload contains non-finite data");
  }
  if (result.value.yaw_rad < -kPi || result.value.yaw_rad >= kPi) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kInvalidValue,
        "pose2d yaw must be in [-pi, pi)");
  }
  if (!decode_boolean(bytes[24U], result.value.position_valid) ||
      !decode_boolean(bytes[25U], result.value.yaw_valid)) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kInvalidBoolean,
        "pose2d payload boolean must be zero or one");
  }
  result.value.reserved = read_u16(bytes, 26U);
  if (result.value.reserved != 0U) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kInvalidReserved,
        "pose2d payload reserved field must be zero");
  }
  result.value.capability_mask = read_u32(bytes, 28U);
  const auto planar_bit =
      static_cast<std::uint32_t>(Capability::kPlanarPosition);
  const auto yaw_bit = static_cast<std::uint32_t>(Capability::kYaw);
  if ((result.value.position_valid &&
       (result.value.capability_mask & planar_bit) == 0U) ||
      (result.value.yaw_valid &&
       (result.value.capability_mask & yaw_bit) == 0U)) {
    return payload_failure<Pose2DPayload>(
        ProtocolError::kInvalidValue,
        "pose2d valid field is missing its capability bit");
  }
  return result;
}

std::vector<std::uint8_t> encode_network_payload(
    const NetworkPayload& payload) {
  require_network_values(payload);
  // output按计数、布尔、状态、保留位和原因位图累计20字节。
  std::vector<std::uint8_t> output;
  output.reserve(kNetworkPayloadSize);  // 精确预留20字节网络载荷。
  append_u32(output, payload.node_count);  // 偏移0：配置的节点总数。
  append_u32(output, payload.reachable_node_count);  // 偏移4：从参考节点当前可达的节点数。
  append_u32(output, payload.active_edge_count);  // 偏移8：未超时且质量允许的无向边数。
  output.push_back(payload.connected ? 1U : 0U);  // 偏移12：是否所有节点均可达。
  output.push_back(payload.observable ? 1U : 0U);  // 偏移13：当前协同几何是否满足可观判据。
  output.push_back(static_cast<std::uint8_t>(payload.state));  // 偏移14：综合定位状态。
  output.push_back(payload.reserved);  // 偏移15：v1保留位必须为0。
  append_u32(output, payload.reason_mask);  // 偏移16：断连、不可观或过期等可组合原因位。
  return output;  // 返回固定20字节载荷。
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
  result.value.node_count = read_u32(bytes, 0U);  // 恢复节点总数。
  result.value.reachable_node_count = read_u32(bytes, 4U);  // 恢复可达节点数。
  result.value.active_edge_count = read_u32(bytes, 8U);  // 恢复活动边数。
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
  result.value.state = static_cast<LocalizationState>(bytes[14U]);  // 范围检查后恢复强类型定位状态。
  if (bytes[15U] != 0U) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidReserved,
        "network reserved byte must be zero");
  }
  result.value.reason_mask = read_u32(bytes, 16U);  // 恢复可同时置位的网络异常原因。
  if (!valid_network_topology(result.value)) {
    return payload_failure<NetworkPayload>(
        ProtocolError::kInvalidValue,
        "network payload topology is inconsistent");
  }
  return result;  // 拓扑计数和布尔逻辑一致时才返回成功。
}

std::vector<std::uint8_t> encode_observation_payload(
    const ObservationPayload& payload) {
  // 保留滑窗计数、原因位图和实际融合动作，便于GCS审计退化判定。
  require_observation_values(payload);
  // output按窗口、六个计数、四个double和尾部状态字段累计80字节。
  std::vector<std::uint8_t> output;
  output.reserve(kObservationPayloadSize);  // 精确预留80字节边质量载荷。
  append_u64(output, payload.window_start_ns);  // 偏移0：滑动窗口起点。
  append_u64(output, payload.window_end_ns);  // 偏移8：滑动窗口终点。
  append_u32(output, payload.expected_count);  // 偏移16：按名义频率预期包数。
  append_u32(output, payload.received_count);  // 偏移20：实际收到包数。
  append_u32(output, payload.valid_count);  // 偏移24：有效包数。
  append_u32(output, payload.nlos_count);  // 偏移28：NLOS包数。
  append_u32(output, payload.residual_rejected_count);  // 偏移32：被NIS残差门限拒绝的包数。
  append_u32(output, payload.dropped_count);  // 偏移36：去重、乱序、超时等丢弃包数。
  append_double(output, payload.nlos_ratio);  // 偏移40：NLOS比例。
  append_double(output, payload.valid_ratio);  // 偏移48：有效率。
  append_double(output, payload.actual_rate_hz);  // 偏移56：实际接收频率。
  append_double(output, payload.covariance_scale);  // 偏移64：融合时采用的方差倍率。
  output.push_back(static_cast<std::uint8_t>(payload.state));  // 偏移72：长期观测质量状态。
  output.push_back(static_cast<std::uint8_t>(payload.action));  // 偏移73：当前融合动作。
  output.push_back(payload.input_overflow ? 1U : 0U);  // 偏移74：统计缓存是否溢出。
  output.push_back(payload.reserved);  // 偏移75：v1保留字节。
  append_u32(output, payload.reason_mask);  // 偏移76：退化原因位图。
  return output;  // 返回固定80字节载荷。
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
  result.value.window_start_ns = read_u64(bytes, 0U);  // 恢复滑窗起点。
  result.value.window_end_ns = read_u64(bytes, 8U);  // 恢复滑窗终点。
  result.value.expected_count = read_u32(bytes, 16U);  // 恢复预期包数。
  result.value.received_count = read_u32(bytes, 20U);  // 恢复实际收包数。
  result.value.valid_count = read_u32(bytes, 24U);  // 恢复有效包数。
  result.value.nlos_count = read_u32(bytes, 28U);  // 恢复NLOS包数。
  result.value.residual_rejected_count = read_u32(bytes, 32U);  // 恢复残差拒绝数。
  result.value.dropped_count = read_u32(bytes, 36U);  // 恢复其他丢弃数。
  result.value.nlos_ratio = read_double(bytes, 40U);  // 恢复NLOS比例。
  result.value.valid_ratio = read_double(bytes, 48U);  // 恢复有效率。
  result.value.actual_rate_hz = read_double(bytes, 56U);  // 恢复实际频率。
  result.value.covariance_scale = read_double(bytes, 64U);  // 恢复方差倍率。
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
  result.value.state = static_cast<ObservationState>(bytes[72U]);  // 范围验证后恢复质量状态枚举。
  result.value.action = static_cast<FusionAction>(bytes[73U]);  // 范围验证后恢复融合动作枚举。
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
  result.value.reason_mask = read_u32(bytes, 76U);  // 恢复最后的退化原因位图。
  return result;  // 成功结果已通过比例、计数关系、枚举和保留位检查。
}

std::vector<std::uint8_t> encode_alert_payload(
    const AlertPayload& payload) {
  // 告警使用稳定code/lifecycle/reason_mask，文本解释由GCS按版本映射。
  require_alert_values(payload);
  // output按告警标识、节点/边和首次/最近时间累计40字节。
  std::vector<std::uint8_t> output;
  output.reserve(kAlertPayloadSize);  // 精确预留40字节告警载荷。
  append_u32(output, static_cast<std::uint32_t>(payload.alert_code));  // 偏移0：稳定告警编号，GCS据此显示文本。
  output.push_back(static_cast<std::uint8_t>(payload.level));  // 偏移4：提示、警告、错误或严重等级。
  output.push_back(static_cast<std::uint8_t>(payload.lifecycle));  // 偏移5：告警激活、持续或清除阶段。
  output.push_back(static_cast<std::uint8_t>(payload.source));  // 偏移6：告警来自算法、输入还是网络。
  output.push_back(payload.reserved0);  // 偏移7：v1保留字节。
  append_u32(output, payload.reason_mask);  // 偏移8：同一告警可携带多个原因位。
  append_u16(output, payload.node_id);  // 偏移12：单节点告警对象；不适用时按协议填0。
  append_u16(output, payload.from_node);  // 偏移14：边告警起点。
  append_u16(output, payload.to_node);  // 偏移16：边告警终点。
  append_u16(output, payload.reserved1);  // 偏移18：用于对齐并保留扩展空间。
  append_u64(output, payload.first_timestamp_ns);  // 偏移20：本轮告警首次出现时刻。
  append_u64(output, payload.last_timestamp_ns);  // 偏移28：最近一次仍成立或清除的时刻。
  append_u32(output, payload.reserved2);  // 偏移36：v1末尾保留字段。
  return output;  // 返回固定40字节载荷。
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
  result.value.alert_code = static_cast<AlertCode>(read_u32(bytes, 0U));  // 恢复32位告警编号为强类型枚举。
  result.value.level = static_cast<AlertLevel>(bytes[4U]);  // 恢复告警等级，valid_alert稍后验证范围。
  result.value.lifecycle = static_cast<AlertLifecycle>(bytes[5U]);  // 恢复告警生命周期。
  result.value.source = static_cast<AlertSource>(bytes[6U]);  // 恢复告警来源。
  result.value.reserved0 = bytes[7U];  // 读取保留位是为了明确检查其必须为0。
  result.value.reason_mask = read_u32(bytes, 8U);  // 恢复原因位图。
  result.value.node_id = read_u16(bytes, 12U);  // 恢复单节点对象编号。
  result.value.from_node = read_u16(bytes, 14U);  // 恢复边起点编号。
  result.value.to_node = read_u16(bytes, 16U);  // 恢复边终点编号。
  result.value.reserved1 = read_u16(bytes, 18U);  // 恢复并随后验证中间保留字段。
  result.value.first_timestamp_ns = read_u64(bytes, 20U);  // 恢复首次告警时间。
  result.value.last_timestamp_ns = read_u64(bytes, 28U);  // 恢复最近告警时间。
  result.value.reserved2 = read_u32(bytes, 36U);  // 恢复并随后验证末尾保留字段。
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
  return result;  // 只有枚举、时间和生命周期语义均一致才成功。
}

std::vector<std::uint8_t> encode_algorithm_status_payload(
    const AlgorithmStatusPayload& payload) {
  require_algorithm_status_values(payload);
  // output按版本、模式、累计计数和运行时长累计48字节。
  std::vector<std::uint8_t> output;
  output.reserve(kAlgorithmStatusPayloadSize);  // 精确预留48字节进程状态载荷。
  append_u32(output, payload.abi_version);  // 偏移0：算法库C ABI版本。
  append_u32(output, payload.software_version_packed);  // 偏移4：打包后的主/次/补丁软件版本。
  output.push_back(static_cast<std::uint8_t>(payload.mode));  // 偏移8：当前二维UWB或IMU+UWB模式。
  output.push_back(static_cast<std::uint8_t>(payload.run_state));  // 偏移9：启动、运行、退化或停止状态。
  append_u16(output, payload.reserved0);  // 偏移10：v1保留16位字段。
  append_u32(output, payload.reserved1);  // 偏移12：v1保留32位字段。
  append_u64(output, payload.accepted_ranges);  // 偏移16：累计接受的测距包数。
  append_u64(output, payload.rejected_ranges);  // 偏移24：累计拒绝的测距包数。
  append_u64(output, payload.protocol_errors);  // 偏移32：累计协议解析错误数。
  append_u64(output, payload.uptime_ns);  // 偏移40：进程运行时长。
  return output;  // 返回固定48字节载荷。
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
  result.value.abi_version = read_u32(bytes, 0U);  // 恢复算法库ABI版本。
  result.value.software_version_packed = read_u32(bytes, 4U);  // 恢复打包软件版本。
  result.value.mode = static_cast<AlgorithmMode>(bytes[8U]);  // 恢复运行模式，稍后验证枚举范围。
  result.value.run_state = static_cast<AlgorithmRunState>(bytes[9U]);  // 恢复进程运行状态。
  result.value.reserved0 = read_u16(bytes, 10U);  // 读取保留字段以便拒绝非零扩展数据。
  result.value.reserved1 = read_u32(bytes, 12U);  // 读取第二个保留字段。
  result.value.accepted_ranges = read_u64(bytes, 16U);  // 恢复累计接受量。
  result.value.rejected_ranges = read_u64(bytes, 24U);  // 恢复累计拒绝量。
  result.value.protocol_errors = read_u64(bytes, 32U);  // 恢复累计协议错误量。
  result.value.uptime_ns = read_u64(bytes, 40U);  // 恢复运行时长。
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
  return result;  // ABI、模式、运行状态和保留字段全部合法后返回成功。
}

}  // namespace zju::coop::protocol

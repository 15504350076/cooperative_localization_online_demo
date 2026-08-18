// 模块职责：验证ZJCL帧及全部固定载荷的小端字节布局、CRC黄金向量和严格错误分类。
// C++初学者阅读提示：正常用例执行“对象->encode字节->decode对象->逐字段比较”；
// 错误用例故意改动magic、版本、长度或CRC，确认损坏数据不会进入算法。
#include "protocol/crc32.hpp"
#include "protocol/wire_protocol.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using zju::coop::protocol::crc32_ieee;
using zju::coop::protocol::decode_frame;
using zju::coop::protocol::encode_frame;
using zju::coop::protocol::Frame;
using zju::coop::protocol::MessageType;
using zju::coop::protocol::ImuPayload;
using zju::coop::protocol::ProtocolError;
using zju::coop::protocol::LocalizationPayload;
using zju::coop::protocol::Pose2DPayload;
using zju::coop::protocol::NetworkPayload;
using zju::coop::protocol::ObservationPayload;
using zju::coop::protocol::RangePayload;
using zju::coop::protocol::AlgorithmStatusPayload;
using zju::coop::protocol::AlertPayload;
using zju::coop::protocol::decode_localization_payload;
using zju::coop::protocol::decode_pose2d_payload;
using zju::coop::protocol::decode_network_payload;
using zju::coop::protocol::decode_observation_payload;
using zju::coop::protocol::decode_range_payload;
using zju::coop::protocol::encode_localization_payload;
using zju::coop::protocol::encode_pose2d_payload;
using zju::coop::protocol::encode_network_payload;
using zju::coop::protocol::encode_observation_payload;
using zju::coop::protocol::encode_range_payload;
using zju::coop::protocol::decode_imu_payload;
using zju::coop::protocol::encode_imu_payload;
using zju::coop::protocol::decode_algorithm_status_payload;
using zju::coop::protocol::decode_alert_payload;
using zju::coop::protocol::encode_algorithm_status_payload;
using zju::coop::protocol::encode_alert_payload;

Frame sample_frame() {
  // alert：字段采用逐字节可辨模式的Active告警载荷；frame：包裹该载荷的冻结帧头输入。
  AlertPayload alert{};
  alert.level = zju::coop::protocol::AlertLevel::kWarning;
  alert.lifecycle = zju::coop::protocol::AlertLifecycle::kActive;
  alert.reason_mask = 0x10203040U;
  alert.from_node = 1U;
  alert.to_node = 2U;
  alert.first_timestamp_ns = 0x0102030405060708ULL;
  alert.last_timestamp_ns = 0x1112131415161718ULL;
  Frame frame{};
  frame.header.message_type = MessageType::kAlert;
  frame.header.flags = 0x1234U;
  frame.header.sequence = 0x0102030405060708ULL;
  frame.header.timestamp_ns = 0x1112131415161718ULL;
  frame.header.source_node = 0x2233U;
  frame.header.target_node = 0x4455U;
  frame.payload = encode_alert_payload(alert);
  return frame;
}

// bytes是预期失败的完整帧缓冲，expected是精确错误分类，max_payload_size控制载荷边界。
void expect_decode_error(const std::vector<std::uint8_t>& bytes,
                         ProtocolError expected,
                         std::size_t max_payload_size =
                             zju::coop::protocol::kDefaultMaxPayloadSize) {
  // result：解帧失败回执，期望不含值且错误码等于expected。
  const auto result = decode_frame(bytes, max_payload_size);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.error, expected);
}

template <typename Operation>
// operation按值持有同步执行的失败场景闭包，返回是否观察到invalid_argument。
bool throws_invalid_argument(Operation operation) {
  try {
    operation();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

// bytes由调用方持有，offset是首写偏移，value按小端覆盖；byte依次选择四个源字节。
void write_u32_le(std::vector<std::uint8_t>& bytes, std::size_t offset,
                  std::uint32_t value) {
  for (unsigned int byte = 0U; byte < 4U; ++byte) {
    bytes[offset + byte] = static_cast<std::uint8_t>(
        (value >> (byte * 8U)) & 0xFFU);
  }
}

// bytes由调用方持有，offset是首写偏移，value按小端覆盖；byte依次选择八个源字节。
void write_u64_le(std::vector<std::uint8_t>& bytes, std::size_t offset,
                  std::uint64_t value) {
  for (unsigned int byte = 0U; byte < 8U; ++byte) {
    bytes[offset + byte] = static_cast<std::uint8_t>(
        (value >> (byte * 8U)) & 0xFFU);
  }
}

// bytes/offset指定故障注入位置，value是待写double；bits暂存不改变位模式的uint64表示。
void write_double_le(std::vector<std::uint8_t>& bytes,
                     std::size_t offset, double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  write_u64_le(bytes, offset, bits);
}

}  // namespace

// 外层帧组：标准CRC向量和冻结黄金字节共同锁定小端偏移及“排除CRC字段”的口径。
TEST_CASE(crc32_ieee_matches_standard_check_value) {
  // bytes：CRC-32/IEEE标准“123456789”检查向量，期望0xCBF43926。
  const std::vector<std::uint8_t> bytes{
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};

  EXPECT_EQ(crc32_ieee(bytes), 0xCBF43926U);
  EXPECT_EQ(crc32_ieee({}), 0U);
}

TEST_CASE(wire_frame_matches_frozen_little_endian_golden_bytes) {
  // expected：冻结的103字节小端告警帧；encoded/decoded：当前编码输出与黄金帧解析回执。
  const std::vector<std::uint8_t> expected{
      0x5AU, 0x4AU, 0x43U, 0x4CU, 0x01U, 0x00U, 0x67U, 0x00U,
      0x28U, 0x00U, 0x34U, 0x12U, 0x28U, 0x00U, 0x00U, 0x00U,
      0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U,
      0x18U, 0x17U, 0x16U, 0x15U, 0x14U, 0x13U, 0x12U, 0x11U,
      0x33U, 0x22U, 0x55U, 0x44U, 0xD7U, 0x90U, 0x88U, 0xA0U,
      0x01U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U,
      0x40U, 0x30U, 0x20U, 0x10U, 0x00U, 0x00U, 0x01U, 0x00U,
      0x02U, 0x00U, 0x00U, 0x00U, 0x08U, 0x07U, 0x06U, 0x05U,
      0x04U, 0x03U, 0x02U, 0x01U, 0x18U, 0x17U, 0x16U, 0x15U,
      0x14U, 0x13U, 0x12U, 0x11U, 0x00U, 0x00U, 0x00U, 0x00U};

  const auto encoded = encode_frame(sample_frame());
  EXPECT_EQ(encoded, expected);

  const auto decoded = decode_frame(expected);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.header.message_type, MessageType::kAlert);
  EXPECT_EQ(decoded.value.header.flags, 0x1234U);
  EXPECT_EQ(decoded.value.header.payload_size, 40U);
  EXPECT_EQ(decoded.value.header.sequence, 0x0102030405060708ULL);
  EXPECT_EQ(decoded.value.header.timestamp_ns, 0x1112131415161718ULL);
  EXPECT_EQ(decoded.value.header.source_node, 0x2233U);
  EXPECT_EQ(decoded.value.header.target_node, 0x4455U);
  EXPECT_EQ(decoded.value.header.crc32, 0xA08890D7U);
  EXPECT_EQ(decoded.value.payload, sample_frame().payload);
}

TEST_CASE(wire_frame_rejects_wrong_size_for_fixed_payload_type) {
  // short_range_frame：头部声明Range但仅含3字节载荷的冻结错误帧。
  const std::vector<std::uint8_t> short_range_frame{
      0x5AU, 0x4AU, 0x43U, 0x4CU, 0x01U, 0x00U, 0x01U, 0x00U,
      0x28U, 0x00U, 0x34U, 0x12U, 0x03U, 0x00U, 0x00U, 0x00U,
      0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U,
      0x18U, 0x17U, 0x16U, 0x15U, 0x14U, 0x13U, 0x12U, 0x11U,
      0x33U, 0x22U, 0x55U, 0x44U, 0x52U, 0x54U, 0x96U, 0x3DU,
      0xAAU, 0xBBU, 0xCCU};
  expect_decode_error(short_range_frame,
                      ProtocolError::kInvalidPayloadSize);

  // frame：把合法告警帧类型改成Range但保留40字节载荷；rejected：编码器是否拒绝固定大小不匹配。
  Frame frame = sample_frame();
  frame.header.message_type = MessageType::kRange;
  bool rejected = false;
  try {
    static_cast<void>(encode_frame(frame));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(wire_frame_rejects_corruption_truncation_trailing_and_bad_headers) {
  // valid：所有单点故障变体的合法帧基线。
  const auto valid = encode_frame(sample_frame());

  // corrupted/truncated/trailing：分别翻转CRC字段末字节、删除尾字节和追加尾随字节的输入。
  auto corrupted = valid;
  corrupted.back() ^= 0x01U;
  expect_decode_error(corrupted, ProtocolError::kCrcMismatch);

  auto truncated = valid;
  truncated.pop_back();
  expect_decode_error(truncated, ProtocolError::kTruncated);

  auto trailing = valid;
  trailing.push_back(0U);
  expect_decode_error(trailing, ProtocolError::kTrailingBytes);

  // bad_magic/bad_version/bad_header_size/bad_type：逐字段破坏外层头部的独立缓冲副本。
  auto bad_magic = valid;
  bad_magic[0U] = 0U;
  expect_decode_error(bad_magic, ProtocolError::kBadMagic);

  auto bad_version = valid;
  bad_version[4U] = 2U;
  expect_decode_error(bad_version, ProtocolError::kUnsupportedVersion);

  auto bad_header_size = valid;
  bad_header_size[8U] = 39U;
  expect_decode_error(bad_header_size, ProtocolError::kInvalidHeaderSize);

  auto bad_type = valid;
  bad_type[6U] = 0xFFU;
  bad_type[7U] = 0x7FU;
  expect_decode_error(bad_type, ProtocolError::kUnknownMessageType);
}

TEST_CASE(wire_frame_enforces_payload_limit_at_boundary) {
  // frame/encoded：40字节告警载荷及其在恰等于上限40时的成功编码；rejected：上限39时编码是否失败。
  Frame frame = sample_frame();
  const auto encoded = encode_frame(frame, 40U);
  EXPECT_TRUE(decode_frame(encoded, 40U).ok());
  expect_decode_error(encoded, ProtocolError::kPayloadTooLarge, 39U);

  bool rejected = false;
  try {
    static_cast<void>(encode_frame(frame, 39U));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(wire_frame_rejects_alert_payload_above_hard_limit) {
  // frame：载荷人为扩大到协议硬上限加1；rejected：编码器是否拒绝该超限输入。
  Frame frame = sample_frame();
  frame.payload.assign(
      zju::coop::protocol::kDefaultMaxPayloadSize + 1U, 0x5AU);
  bool rejected = false;
  try {
    static_cast<void>(encode_frame(frame));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(wire_frame_encoder_rejects_custom_limit_above_hard_limit) {
  // rejected：调用方自定义上限超过协议硬上限时编码器是否拒绝。
  bool rejected = false;
  try {
    static_cast<void>(encode_frame(
        sample_frame(),
        zju::coop::protocol::kDefaultMaxPayloadSize + 1U));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(wire_frame_decoder_rejects_custom_limit_above_hard_limit) {
  // bytes：合法告警帧；decoded：使用非法过大自定义上限时的解码失败回执。
  const auto bytes = encode_frame(sample_frame());
  const auto decoded = decode_frame(
      bytes, zju::coop::protocol::kDefaultMaxPayloadSize + 1U);
  EXPECT_EQ(decoded.error, ProtocolError::kPayloadTooLarge);
}

// 输入载荷组：测距和332字节IMU须在C++/Python间逐字节一致并严格拒绝坏布尔/数值。
TEST_CASE(range_payload_matches_frozen_bytes_and_round_trips) {
  // payload：包含距离、不确定度、NLOS和归一化状态的输入；expected：冻结24字节；decoded：往返解析回执。
  RangePayload payload{};
  payload.range_m = 3.0;
  payload.range_std_m = 0.25;
  payload.nlos_probability = 0.5F;
  payload.nlos_flag = true;
  payload.has_nlos_probability = true;
  payload.valid = true;
  payload.status = zju::coop::protocol::kRangeStatusInvalid;
  const std::vector<std::uint8_t> expected{
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x08U, 0x40U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xD0U, 0x3FU,
      0x00U, 0x00U, 0x00U, 0x3FU, 0x01U, 0x01U, 0x01U, 0x02U};

  EXPECT_EQ(encode_range_payload(payload), expected);
  const auto decoded = decode_range_payload(expected);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.range_m, 3.0);
  EXPECT_EQ(decoded.value.range_std_m, 0.25);
  EXPECT_EQ(decoded.value.nlos_probability, 0.5F);
  EXPECT_TRUE(decoded.value.nlos_flag);
  EXPECT_TRUE(decoded.value.has_nlos_probability);
  EXPECT_TRUE(decoded.value.valid);
  EXPECT_EQ(decoded.value.status,
            zju::coop::protocol::kRangeStatusInvalid);
}

TEST_CASE(imu_payload_has_frozen_332_byte_layout_and_round_trips) {
  // payload：含单位姿态、角速率、比力和frame_id的IMU输入；bytes/decoded：332字节编码与往返结果。
  ImuPayload payload{};
  payload.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
  payload.angular_velocity_rad_s = {0.1, 0.2, 0.3};
  payload.linear_acceleration_m_s2 = {1.0, 2.0, 9.80665};
  std::copy_n("imu_link", 9U, payload.frame_id.begin());
  payload.valid = true;

  const auto bytes = encode_imu_payload(payload);
  const auto decoded = decode_imu_payload(bytes);

  EXPECT_EQ(bytes.size(), 332U);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.frame_id[0], 'i');
  EXPECT_EQ(decoded.value.valid, true);
  EXPECT_TRUE(std::abs(decoded.value.linear_acceleration_m_s2[2] - 9.80665) <
              1.0e-12);
}

TEST_CASE(range_payload_rejects_bad_size_boolean_and_numeric_values) {
  // payload/bytes：合法3 m测距载荷及其编码基线。
  RangePayload payload{};
  payload.range_m = 3.0;
  payload.range_std_m = 0.1;
  payload.nlos_probability = 0.25F;
  auto bytes = encode_range_payload(payload);

  // short_bytes/trailing/invalid_boolean/infinity：分别制造欠长、超长、布尔值2和距离正无穷的解码输入。
  auto short_bytes = bytes;
  short_bytes.pop_back();
  EXPECT_EQ(decode_range_payload(short_bytes).error,
            ProtocolError::kInvalidPayloadSize);
  auto trailing = bytes;
  trailing.push_back(0U);
  EXPECT_EQ(decode_range_payload(trailing).error,
            ProtocolError::kInvalidPayloadSize);
  auto invalid_boolean = bytes;
  invalid_boolean[20U] = 2U;
  EXPECT_EQ(decode_range_payload(invalid_boolean).error,
            ProtocolError::kInvalidBoolean);
  auto infinity = bytes;
  infinity[0U] = 0U;
  infinity[1U] = 0U;
  infinity[2U] = 0U;
  infinity[3U] = 0U;
  infinity[4U] = 0U;
  infinity[5U] = 0U;
  infinity[6U] = 0xF0U;
  infinity[7U] = 0x7FU;
  EXPECT_EQ(decode_range_payload(infinity).error,
            ProtocolError::kNonFiniteValue);

  payload.range_m = 0.0;
  // rejected：复用于确认零距离和NaN标准差均在编码侧抛出参数异常。
  bool rejected = false;
  try {
    static_cast<void>(encode_range_payload(payload));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
  payload.range_m = 1.0;
  payload.range_std_m = std::numeric_limits<double>::quiet_NaN();
  rejected = false;
  try {
    static_cast<void>(encode_range_payload(payload));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_payload_rejects_status_outside_normalized_v1_enum) {
  // payload：状态值3的非法编码输入；encode_rejected：编码侧拒绝哨兵；bytes：状态字节被改为3的解码输入。
  RangePayload payload{};
  payload.range_m = 3.0;
  payload.range_std_m = 0.1;
  payload.nlos_probability = 0.0F;
  payload.valid = true;
  payload.status = 3U;

  bool encode_rejected = false;
  try {
    static_cast<void>(encode_range_payload(payload));
  } catch (const std::invalid_argument&) {
    encode_rejected = true;
  }
  EXPECT_TRUE(encode_rejected);

  payload.status = 0U;
  auto bytes = encode_range_payload(payload);
  bytes[23U] = 3U;
  EXPECT_EQ(decode_range_payload(bytes).error, ProtocolError::kInvalidValue);
}

// 输出载荷组：除固定大小外还验证能力位、拓扑计数、滑窗计数和告警生命周期约束。
TEST_CASE(localization_network_and_observation_payloads_have_frozen_sizes) {
  // localization：字段可辨的64字节定位载荷；localization_bytes/decoded_localization：编码和往返回执。
  LocalizationPayload localization{};
  localization.x = 1.0;
  localization.y = 2.0;
  localization.vx = 3.0;
  localization.vy = 4.0;
  localization.cov_xx = 5.0;
  localization.cov_xy = -0.5;
  localization.cov_yy = 0.25;
  localization.state = zju::coop::LocalizationState::kDegraded;
  localization.valid = true;
  localization.capability_mask = 0x01020304U;
  const auto localization_bytes = encode_localization_payload(localization);
  EXPECT_EQ(localization_bytes.size(), 64U);
  EXPECT_EQ(localization_bytes[56U], 2U);
  EXPECT_EQ(localization_bytes[60U], 0x04U);
  EXPECT_EQ(localization_bytes[63U], 0x01U);
  const auto decoded_localization =
      decode_localization_payload(localization_bytes);
  EXPECT_TRUE(decoded_localization.ok());
  EXPECT_EQ(decoded_localization.value.cov_xy, -0.5);
  EXPECT_EQ(decoded_localization.value.state,
            zju::coop::LocalizationState::kDegraded);
  EXPECT_EQ(decoded_localization.value.capability_mask, 0x01020304U);

  // network/network_expected/decoded_network：字段可辨的20字节拓扑载荷、黄金字节和解析回执。
  NetworkPayload network{};
  network.node_count = 0x11121314U;
  network.reachable_node_count = 0x11121314U;
  network.active_edge_count = 0x21222324U;
  network.connected = true;
  network.observable = false;
  network.state = zju::coop::LocalizationState::kUnobservable;
  network.reason_mask = 0x31323334U;
  const std::vector<std::uint8_t> network_expected{
      0x14U, 0x13U, 0x12U, 0x11U, 0x14U, 0x13U, 0x12U, 0x11U,
      0x24U, 0x23U, 0x22U, 0x21U, 0x01U, 0x00U, 0x03U, 0x00U,
      0x34U, 0x33U, 0x32U, 0x31U};
  EXPECT_EQ(encode_network_payload(network), network_expected);
  const auto decoded_network = decode_network_payload(network_expected);
  EXPECT_TRUE(decoded_network.ok());
  EXPECT_EQ(decoded_network.value.active_edge_count, 0x21222324U);
  EXPECT_EQ(decoded_network.value.reason_mask, 0x31323334U);

  // observation/observation_bytes/decoded_observation：完整80字节滑窗质量载荷、编码和解析回执。
  ObservationPayload observation{};
  observation.window_start_ns = 0x0102030405060708ULL;
  observation.window_end_ns = 0x1112131415161718ULL;
  observation.expected_count = 40U;
  observation.received_count = 39U;
  observation.valid_count = 38U;
  observation.nlos_count = 3U;
  observation.residual_rejected_count = 2U;
  observation.dropped_count = 1U;
  observation.nlos_ratio = 0.25;
  observation.valid_ratio = 0.95;
  observation.actual_rate_hz = 19.5;
  observation.covariance_scale = 4.0;
  observation.state = zju::coop::ObservationState::kDegraded;
  observation.action = zju::coop::FusionAction::kUseDownweighted;
  observation.input_overflow = true;
  observation.reason_mask = 0x10203040U;
  const auto observation_bytes = encode_observation_payload(observation);
  EXPECT_EQ(observation_bytes.size(), 80U);
  EXPECT_EQ(observation_bytes[0U], 0x08U);
  EXPECT_EQ(observation_bytes[15U], 0x11U);
  EXPECT_EQ(observation_bytes[72U], 2U);
  EXPECT_EQ(observation_bytes[73U], 1U);
  EXPECT_EQ(observation_bytes[74U], 1U);
  EXPECT_EQ(observation_bytes[75U], 0U);
  EXPECT_EQ(observation_bytes[76U], 0x40U);
  EXPECT_EQ(observation_bytes[79U], 0x10U);
  const auto decoded_observation =
      decode_observation_payload(observation_bytes);
  EXPECT_TRUE(decoded_observation.ok());
  EXPECT_EQ(decoded_observation.value.window_end_ns,
            0x1112131415161718ULL);
  EXPECT_EQ(decoded_observation.value.covariance_scale, 4.0);
  EXPECT_EQ(decoded_observation.value.reason_mask, 0x10203040U);
}

TEST_CASE(algorithm_status_payload_matches_frozen_48_byte_golden) {
  // payload/expected/decoded：版本、模式、状态和64位计数均可辨的48字节算法状态黄金往返。
  AlgorithmStatusPayload payload{};
  payload.abi_version = 0x00010000U;
  payload.software_version_packed = 0x00000300U;
  payload.mode = zju::coop::protocol::AlgorithmMode::kUwbOnlyPlanar;
  payload.run_state = zju::coop::protocol::AlgorithmRunState::kDegraded;
  payload.accepted_ranges = 0x0102030405060708ULL;
  payload.rejected_ranges = 0x1112131415161718ULL;
  payload.protocol_errors = 0x2122232425262728ULL;
  payload.uptime_ns = 0x3132333435363738ULL;
  const std::vector<std::uint8_t> expected{
      0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x03U, 0x00U, 0x00U,
      0x01U, 0x02U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
      0x08U, 0x07U, 0x06U, 0x05U, 0x04U, 0x03U, 0x02U, 0x01U,
      0x18U, 0x17U, 0x16U, 0x15U, 0x14U, 0x13U, 0x12U, 0x11U,
      0x28U, 0x27U, 0x26U, 0x25U, 0x24U, 0x23U, 0x22U, 0x21U,
      0x38U, 0x37U, 0x36U, 0x35U, 0x34U, 0x33U, 0x32U, 0x31U};
  EXPECT_EQ(encode_algorithm_status_payload(payload), expected);
  const auto decoded = decode_algorithm_status_payload(expected);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.abi_version, 0x00010000U);
  EXPECT_EQ(decoded.value.software_version_packed, 0x00000300U);
  EXPECT_EQ(decoded.value.mode,
            zju::coop::protocol::AlgorithmMode::kUwbOnlyPlanar);
  EXPECT_EQ(decoded.value.run_state,
            zju::coop::protocol::AlgorithmRunState::kDegraded);
  EXPECT_EQ(decoded.value.accepted_ranges, 0x0102030405060708ULL);
  EXPECT_EQ(decoded.value.uptime_ns, 0x3132333435363738ULL);

  // invalid_mode/invalid_reserved：分别破坏模式枚举和保留字节的独立解码缓冲。
  auto invalid_mode = expected;
  invalid_mode[8U] = 0U;
  EXPECT_EQ(decode_algorithm_status_payload(invalid_mode).error,
            ProtocolError::kInvalidValue);
  auto invalid_reserved = expected;
  invalid_reserved[10U] = 1U;
  EXPECT_EQ(decode_algorithm_status_payload(invalid_reserved).error,
            ProtocolError::kInvalidReserved);
}

TEST_CASE(alert_payload_matches_frozen_40_byte_golden_and_lifecycle_rules) {
  // payload/expected/decoded：Active网络告警的40字节黄金输入、字节向量和解析回执。
  AlertPayload payload{};
  payload.alert_code = zju::coop::protocol::AlertCode::kNetworkState;
  payload.level = zju::coop::protocol::AlertLevel::kWarning;
  payload.lifecycle = zju::coop::protocol::AlertLifecycle::kActive;
  payload.source = zju::coop::protocol::AlertSource::kAlgorithm;
  payload.reason_mask = 0x10203040U;
  payload.node_id = 0U;
  payload.from_node = 1U;
  payload.to_node = 2U;
  payload.first_timestamp_ns = 0x0102030405060708ULL;
  payload.last_timestamp_ns = 0x1112131415161718ULL;
  const std::vector<std::uint8_t> expected{
      0x01U, 0x00U, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U,
      0x40U, 0x30U, 0x20U, 0x10U, 0x00U, 0x00U, 0x01U, 0x00U,
      0x02U, 0x00U, 0x00U, 0x00U, 0x08U, 0x07U, 0x06U, 0x05U,
      0x04U, 0x03U, 0x02U, 0x01U, 0x18U, 0x17U, 0x16U, 0x15U,
      0x14U, 0x13U, 0x12U, 0x11U, 0x00U, 0x00U, 0x00U, 0x00U};
  EXPECT_EQ(encode_alert_payload(payload), expected);
  const auto decoded = decode_alert_payload(expected);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.reason_mask, 0x10203040U);
  EXPECT_EQ(decoded.value.first_timestamp_ns, 0x0102030405060708ULL);

  // active_without_reason/cleared_with_reason/reversed_time/invalid_reserved：逐条违反生命周期、时间序和保留位约束。
  auto active_without_reason = expected;
  write_u32_le(active_without_reason, 8U, 0U);
  EXPECT_EQ(decode_alert_payload(active_without_reason).error,
            ProtocolError::kInvalidValue);
  auto cleared_with_reason = expected;
  cleared_with_reason[5U] = 1U;
  EXPECT_EQ(decode_alert_payload(cleared_with_reason).error,
            ProtocolError::kInvalidValue);
  auto reversed_time = expected;
  write_u64_le(reversed_time, 20U, 100U);
  write_u64_le(reversed_time, 28U, 99U);
  EXPECT_EQ(decode_alert_payload(reversed_time).error,
            ProtocolError::kInvalidValue);
  auto invalid_reserved = expected;
  invalid_reserved[7U] = 1U;
  EXPECT_EQ(decode_alert_payload(invalid_reserved).error,
            ProtocolError::kInvalidReserved);

  payload.lifecycle = zju::coop::protocol::AlertLifecycle::kCleared;
  payload.reason_mask = 0U;
  EXPECT_TRUE(decode_alert_payload(encode_alert_payload(payload)).ok());
}

TEST_CASE(localization_covariance_is_psd_on_encode_and_decode) {
  // valid：单位二维协方差基线；zero_covariance：允许的半正定零矩阵；overflow_safe_boundary：最大有限等值边界。
  LocalizationPayload valid{};
  valid.cov_xx = 1.0;
  valid.cov_xy = 0.0;
  valid.cov_yy = 1.0;

  LocalizationPayload zero_covariance = valid;
  zero_covariance.cov_xx = 0.0;
  zero_covariance.cov_yy = 0.0;
  EXPECT_TRUE(
      decode_localization_payload(
          encode_localization_payload(zero_covariance))
          .ok());

  LocalizationPayload overflow_safe_boundary = valid;
  overflow_safe_boundary.cov_xx =
      std::numeric_limits<double>::max();
  overflow_safe_boundary.cov_xy =
      std::numeric_limits<double>::max();
  overflow_safe_boundary.cov_yy =
      std::numeric_limits<double>::max();
  EXPECT_TRUE(
      decode_localization_payload(
          encode_localization_payload(overflow_safe_boundary))
          .ok());

  // invalid：依次注入负方差或过大互协方差；bytes：在解码侧覆盖相同字段；lambda同步借用当前invalid。
  LocalizationPayload invalid = valid;
  invalid.cov_xx = -1.0;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_localization_payload(invalid)); }));
  auto bytes = encode_localization_payload(valid);
  write_double_le(bytes, 32U, -1.0);
  EXPECT_EQ(decode_localization_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.cov_yy = -1.0;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_localization_payload(invalid)); }));
  bytes = encode_localization_payload(valid);
  write_double_le(bytes, 48U, -1.0);
  EXPECT_EQ(decode_localization_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.cov_xy = 2.0;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_localization_payload(invalid)); }));
  bytes = encode_localization_payload(valid);
  write_double_le(bytes, 40U, 2.0);
  EXPECT_EQ(decode_localization_payload(bytes).error,
            ProtocolError::kInvalidValue);
}

TEST_CASE(pose2d_payload_matches_frozen_32_byte_layout_and_strict_fields) {
  // payload覆盖三个double、两个独立有效位、零保留位和非对称能力位图。
  Pose2DPayload payload{};
  payload.x = 1.0;
  payload.y = -2.0;
  payload.yaw_rad = 0.5;
  payload.position_valid = true;
  payload.yaw_valid = true;
  payload.capability_mask = 0x0A0B0C0FU;

  // expected冻结小端线序：x/y/yaw占前24字节，有效位和保留位占4字节，能力位占末4字节。
  const std::vector<std::uint8_t> expected{
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xF0U, 0x3FU,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xC0U,
      0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0xE0U, 0x3FU,
      0x01U, 0x01U, 0x00U, 0x00U, 0x0FU, 0x0CU, 0x0BU, 0x0AU};
  EXPECT_EQ(encode_pose2d_payload(payload), expected);
  const auto decoded = decode_pose2d_payload(expected);
  EXPECT_TRUE(decoded.ok());
  EXPECT_EQ(decoded.value.x, 1.0);
  EXPECT_EQ(decoded.value.y, -2.0);
  EXPECT_EQ(decoded.value.yaw_rad, 0.5);
  EXPECT_TRUE(decoded.value.position_valid);
  EXPECT_TRUE(decoded.value.yaw_valid);
  EXPECT_EQ(decoded.value.capability_mask, 0x0A0B0C0FU);

  // 完整帧必须接受新增类型105，并继续执行固定载荷长度和CRC校验。
  Frame frame{};
  frame.header.message_type = MessageType::kPose2D;
  frame.header.sequence = 9U;
  frame.header.timestamp_ns = 123U;
  frame.header.source_node = 2U;
  frame.header.target_node = 1U;
  frame.payload = expected;
  const auto decoded_frame = decode_frame(encode_frame(frame));
  EXPECT_TRUE(decoded_frame.ok());
  EXPECT_EQ(decoded_frame.value.header.message_type, MessageType::kPose2D);
  EXPECT_EQ(decoded_frame.value.payload.size(), 32U);

  // 布尔值只允许0/1，两个保留字节必须全零，三个浮点量必须有限。
  auto invalid_boolean = expected;
  invalid_boolean[24U] = 2U;
  EXPECT_EQ(decode_pose2d_payload(invalid_boolean).error,
            ProtocolError::kInvalidBoolean);
  auto invalid_reserved = expected;
  invalid_reserved[26U] = 1U;
  EXPECT_EQ(decode_pose2d_payload(invalid_reserved).error,
            ProtocolError::kInvalidReserved);
  auto invalid_number = expected;
  write_u64_le(invalid_number, 16U, 0x7FF8000000000000ULL);
  EXPECT_EQ(decode_pose2d_payload(invalid_number).error,
            ProtocolError::kNonFiniteValue);
  auto missing_capability = expected;
  write_u32_le(missing_capability, 28U, 0U);
  EXPECT_EQ(decode_pose2d_payload(missing_capability).error,
            ProtocolError::kInvalidValue);
  payload.x = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_pose2d_payload(payload)); }));
  payload.x = 0.0;
  payload.yaw_rad = -3.14159265358979323846;
  EXPECT_TRUE(decode_pose2d_payload(encode_pose2d_payload(payload)).ok());
  payload.yaw_rad = 3.14159265358979323846;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_pose2d_payload(payload)); }));
}

TEST_CASE(network_topology_invariants_are_symmetric_and_overflow_safe) {
  // empty/single/large：零节点、单节点和uint32极值三类应合法且不溢出的拓扑边界。
  NetworkPayload empty{};
  EXPECT_TRUE(decode_network_payload(encode_network_payload(empty)).ok());

  NetworkPayload single{};
  single.node_count = 1U;
  single.reachable_node_count = 1U;
  single.connected = true;
  single.observable = true;
  EXPECT_TRUE(decode_network_payload(encode_network_payload(single)).ok());

  NetworkPayload large{};
  large.node_count = std::numeric_limits<std::uint32_t>::max();
  large.reachable_node_count = large.node_count;
  large.active_edge_count = std::numeric_limits<std::uint32_t>::max();
  large.connected = true;
  large.observable = true;
  EXPECT_TRUE(decode_network_payload(encode_network_payload(large)).ok());

  // connected：三节点三边全可达但不可观的合法基线；invalid/bytes：逐项复用于编码和解码侧拓扑违规注入。
  NetworkPayload connected{};
  connected.node_count = 3U;
  connected.reachable_node_count = 3U;
  connected.active_edge_count = 3U;
  connected.connected = true;
  connected.observable = false;

  NetworkPayload invalid = connected;
  invalid.reachable_node_count = 4U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  auto bytes = encode_network_payload(connected);
  write_u32_le(bytes, 4U, 4U);
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = connected;
  invalid.active_edge_count = 4U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(connected);
  write_u32_le(bytes, 8U, 4U);
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = connected;
  invalid.active_edge_count = 1U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(connected);
  write_u32_le(bytes, 8U, 1U);
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = connected;
  invalid.connected = false;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(connected);
  bytes[12U] = 0U;
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  // disconnected：三节点仅两节点可达的合法基线，用于反向验证connected/observable一致性。
  NetworkPayload disconnected{};
  disconnected.node_count = 3U;
  disconnected.reachable_node_count = 2U;
  disconnected.active_edge_count = 1U;
  invalid = disconnected;
  invalid.connected = true;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(disconnected);
  bytes[12U] = 1U;
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = disconnected;
  invalid.observable = true;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(disconnected);
  bytes[13U] = 1U;
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = empty;
  invalid.connected = true;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_network_payload(invalid)); }));
  bytes = encode_network_payload(empty);
  bytes[12U] = 1U;
  EXPECT_EQ(decode_network_payload(bytes).error,
            ProtocolError::kInvalidValue);
}

TEST_CASE(observation_windows_and_counts_are_symmetric) {
  // valid：起止相等且各子计数等于接收数3的合法边界；invalid/bytes：复用于编码与解码侧窗口和计数违规。
  ObservationPayload valid{};
  valid.window_start_ns = 10U;
  valid.window_end_ns = 10U;
  valid.received_count = 3U;
  valid.valid_count = 3U;
  valid.nlos_count = 3U;
  valid.residual_rejected_count = 3U;
  EXPECT_TRUE(
      decode_observation_payload(encode_observation_payload(valid)).ok());

  ObservationPayload invalid = valid;
  invalid.window_start_ns = 11U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_observation_payload(invalid)); }));
  auto bytes = encode_observation_payload(valid);
  write_u64_le(bytes, 0U, 11U);
  EXPECT_EQ(decode_observation_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.valid_count = 4U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_observation_payload(invalid)); }));
  bytes = encode_observation_payload(valid);
  write_u32_le(bytes, 24U, 4U);
  EXPECT_EQ(decode_observation_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.nlos_count = 4U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_observation_payload(invalid)); }));
  bytes = encode_observation_payload(valid);
  write_u32_le(bytes, 28U, 4U);
  EXPECT_EQ(decode_observation_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.residual_rejected_count = 4U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_observation_payload(invalid)); }));
  bytes = encode_observation_payload(valid);
  write_u32_le(bytes, 32U, 4U);
  EXPECT_EQ(decode_observation_payload(bytes).error,
            ProtocolError::kInvalidValue);

  invalid = valid;
  invalid.valid_count = 1U;
  invalid.residual_rejected_count = 2U;
  EXPECT_TRUE(throws_invalid_argument(
      [&]() { static_cast<void>(encode_observation_payload(invalid)); }));
  bytes = encode_observation_payload(valid);
  write_u32_le(bytes, 24U, 1U);
  write_u32_le(bytes, 32U, 2U);
  EXPECT_EQ(decode_observation_payload(bytes).error,
            ProtocolError::kInvalidValue);
}

TEST_CASE(fixed_payload_decoders_reject_invalid_enums_reserved_and_floats) {
  // localization/localization_bytes：定位枚举和布尔字段故障注入基线及编码缓冲。
  LocalizationPayload localization{};
  localization.x = 1.0;
  localization.y = 2.0;
  localization.vx = 3.0;
  localization.vy = 4.0;
  localization.cov_xx = 1.0;
  localization.cov_xy = 0.0;
  localization.cov_yy = 1.0;
  auto localization_bytes = encode_localization_payload(localization);
  localization_bytes[56U] = 0xFFU;
  EXPECT_EQ(decode_localization_payload(localization_bytes).error,
            ProtocolError::kInvalidValue);
  localization_bytes = encode_localization_payload(localization);
  localization_bytes[57U] = 2U;
  EXPECT_EQ(decode_localization_payload(localization_bytes).error,
            ProtocolError::kInvalidBoolean);

  // network/network_bytes：网络保留字节非零场景的零拓扑基线与缓冲。
  NetworkPayload network{};
  auto network_bytes = encode_network_payload(network);
  network_bytes[15U] = 1U;
  EXPECT_EQ(decode_network_payload(network_bytes).error,
            ProtocolError::kInvalidReserved);

  // observation/observation_bytes：观测布尔、保留位和非有限频率场景基线与缓冲；rejected记录编码异常。
  ObservationPayload observation{};
  observation.nlos_ratio = 0.0;
  observation.valid_ratio = 1.0;
  observation.actual_rate_hz = 20.0;
  observation.covariance_scale = 1.0;
  auto observation_bytes = encode_observation_payload(observation);
  observation_bytes[74U] = 2U;
  EXPECT_EQ(decode_observation_payload(observation_bytes).error,
            ProtocolError::kInvalidBoolean);
  observation_bytes = encode_observation_payload(observation);
  observation_bytes[75U] = 1U;
  EXPECT_EQ(decode_observation_payload(observation_bytes).error,
            ProtocolError::kInvalidReserved);
  observation.actual_rate_hz =
      std::numeric_limits<double>::infinity();
  bool rejected = false;
  try {
    static_cast<void>(encode_observation_payload(observation));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

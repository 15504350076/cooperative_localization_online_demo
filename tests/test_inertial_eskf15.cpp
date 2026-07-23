// 模块职责：验证单节点15维惯导的首帧基准、中值传播、消息协方差和异常输入回滚。
// 关键判据同时覆盖ENU/FLU重力方向、ROS四元数换序和固定δp/δv/δθ/δbg/δba状态顺序。
#include "core/inertial_eskf15.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {

using zju::coop::ImuDisposition;
using zju::coop::ImuPacket;
using zju::coop::InertialConfig;
using zju::coop::InertialEskf15;
using zju::coop::InertialNodeInitialization;
using zju::coop::Vec3;

InertialConfig make_config() {
  // config：固定重力、时间步、噪声密度和frame_id的惯导基准配置。
  InertialConfig config{};
  config.gravity_mps2 = 9.80665;
  config.min_imu_dt_s = 1.0e-6;
  config.max_imu_dt_s = 2.0;
  config.max_propagation_substep_s = 0.01;
  config.gyro_noise_density_rad_s_sqrt_hz = 1.0e-4;
  config.accel_noise_density_m_s2_sqrt_hz = 1.0e-3;
  config.gyro_bias_random_walk_rad_s2_sqrt_hz = 1.0e-6;
  config.accel_bias_random_walk_m_s3_sqrt_hz = 1.0e-5;
  config.expected_frame_id = "imu_link";
  return config;
}

InertialNodeInitialization make_initialization() {
  // initialization：节点7、单位姿态的单节点名义状态初值。
  InertialNodeInitialization initialization{};
  initialization.node_id = 7U;
  initialization.orientation_b_to_n = {1.0, 0.0, 0.0, 0.0};
  return initialization;
}

// sequence/timestamp_ns用于去重与积分时基，angular_velocity/linear_acceleration是本次FLU惯性量测。
ImuPacket make_imu(std::uint64_t sequence, std::uint64_t timestamp_ns,
                   const Vec3& angular_velocity,
                   const Vec3& linear_acceleration) {
  // packet：按节点7和期望坐标系封装的有效IMU输入；frame：复制进定长frame_id缓冲的源字符串。
  ImuPacket packet{};
  packet.node_id = 7U;
  packet.sequence = sequence;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns + 1U;
  packet.angular_velocity_rad_s = {angular_velocity.x, angular_velocity.y,
                                   angular_velocity.z};
  packet.linear_acceleration_m_s2 = {linear_acceleration.x,
                                     linear_acceleration.y,
                                     linear_acceleration.z};
  const char* frame = "imu_link";
  std::copy(frame, frame + std::strlen(frame) + 1U, packet.frame_id.begin());
  packet.valid = true;
  return packet;
}

// value：只在本次计算中借用的三维向量，返回其欧氏范数供绝对容差判定。
double vector_norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y +
                   value.z * value.z);
}

}  // namespace

// 时间/运动组：首帧不积分，静止比力抵消重力，恒加速度和恒角速率符合解析结果。
TEST_CASE(inertial_eskf_first_sample_only_establishes_timebase) {
  // filter：单位姿态的被测15维ESKF；result：首帧静止IMU回执，期望只建立时基不传播。
  InertialEskf15 filter(make_initialization(), make_config());

  const auto result = filter.push_imu(
      make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
               {0.0, 0.0, 9.80665}));

  EXPECT_EQ(result.disposition, ImuDisposition::kBaselineEstablished);
  EXPECT_FALSE(result.propagated);
  EXPECT_TRUE(vector_norm(filter.state().position_n_m) < 1.0e-15);
  EXPECT_TRUE(vector_norm(filter.state().velocity_n_mps) < 1.0e-15);
}

TEST_CASE(inertial_eskf_stationary_flu_sample_cancels_enu_gravity) {
  // filter：先接收静止基准帧再接收10 ms后同量测；result期望传播且速度、位置保持近零。
  InertialEskf15 filter(make_initialization(), make_config());
  (void)filter.push_imu(make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
                                 {0.0, 0.0, 9.80665}));

  const auto result = filter.push_imu(
      make_imu(2U, 1'010'000'000ULL, {0.0, 0.0, 0.0},
               {0.0, 0.0, 9.80665}));

  EXPECT_EQ(result.disposition, ImuDisposition::kPropagated);
  EXPECT_TRUE(result.propagated);
  EXPECT_TRUE(vector_norm(filter.state().velocity_n_mps) < 1.0e-10);
  EXPECT_TRUE(vector_norm(filter.state().position_n_m) < 1.0e-10);
  EXPECT_TRUE(std::abs(filter.state().orientation_b_to_n.norm() - 1.0) <
              1.0e-12);
}

TEST_CASE(inertial_eskf_constant_acceleration_matches_analytic_motion) {
  // filter：承载1 s恒定1 m/s²前向加速度场景；result用于确认传播状态。
  InertialEskf15 filter(make_initialization(), make_config());
  (void)filter.push_imu(make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
                                 {1.0, 0.0, 9.80665}));

  const auto result = filter.push_imu(
      make_imu(2U, 2'000'000'000ULL, {0.0, 0.0, 0.0},
               {1.0, 0.0, 9.80665}));

  EXPECT_EQ(result.disposition, ImuDisposition::kPropagated);
  EXPECT_TRUE(std::abs(filter.state().velocity_n_mps.x - 1.0) < 1.0e-9);
  EXPECT_TRUE(std::abs(filter.state().position_n_m.x - 0.5) < 1.0e-9);
}

TEST_CASE(inertial_eskf_constant_yaw_rate_rotates_body_x_to_navigation_y) {
  // filter：积分1 s恒定偏航率；half_pi：90度角速度/解析转角；result/rotated_x：传播回执和旋转后前向轴。
  InertialEskf15 filter(make_initialization(), make_config());
  const double half_pi = std::acos(-1.0) / 2.0;
  (void)filter.push_imu(make_imu(1U, 1'000'000'000ULL,
                                 {0.0, 0.0, half_pi},
                                 {0.0, 0.0, 9.80665}));

  const auto result = filter.push_imu(
      make_imu(2U, 2'000'000'000ULL, {0.0, 0.0, half_pi},
               {0.0, 0.0, 9.80665}));
  const Vec3 rotated_x =
      filter.state().orientation_b_to_n.rotate({1.0, 0.0, 0.0});

  EXPECT_EQ(result.disposition, ImuDisposition::kPropagated);
  EXPECT_TRUE(std::abs(rotated_x.x) < 1.0e-9);
  EXPECT_TRUE(std::abs(rotated_x.y - 1.0) < 1.0e-9);
}

TEST_CASE(inertial_eskf_compensates_configured_gyro_and_accel_bias) {
  // initialization：注入已知陀螺与加计偏置；filter：验证偏置补偿的ESKF。
  auto initialization = make_initialization();
  initialization.gyro_bias_rad_s = {0.0, 0.0, 0.1};
  initialization.accel_bias_m_s2 = {0.2, 0.0, 0.0};
  InertialEskf15 filter(initialization, make_config());
  // measured_acceleration/measured_angular_velocity：恰等于静止真值加所配偏置的两帧输入；result期望传播后无运动。
  const Vec3 measured_acceleration{0.2, 0.0, 9.80665};
  const Vec3 measured_angular_velocity{0.0, 0.0, 0.1};
  (void)filter.push_imu(make_imu(1U, 1'000'000'000ULL,
                                 measured_angular_velocity,
                                 measured_acceleration));

  const auto result = filter.push_imu(
      make_imu(2U, 2'000'000'000ULL, measured_angular_velocity,
               measured_acceleration));

  EXPECT_EQ(result.disposition, ImuDisposition::kPropagated);
  EXPECT_TRUE(vector_norm(filter.state().velocity_n_mps) < 1.0e-9);
  EXPECT_TRUE(std::abs(filter.state().orientation_b_to_n.w - 1.0) < 1.0e-12);
}

// 边界组：乱序、frame_id、非有限值和姿态有效条件失败时不能污染上一时间基准。
TEST_CASE(inertial_eskf_rejects_invalid_order_frame_and_nonfinite_data) {
  // filter：依次承载重复、乱序、错坐标系和NaN场景；baseline/state_before：合法首帧及拒绝前状态基线。
  InertialEskf15 filter(make_initialization(), make_config());
  const ImuPacket baseline =
      make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
               {0.0, 0.0, 9.80665});
  (void)filter.push_imu(baseline);
  const auto state_before = filter.state();

  EXPECT_EQ(filter.push_imu(baseline).disposition,
            ImuDisposition::kDuplicate);

  // old：时间倒退的乱序包；wrong_frame/other：坐标系不匹配包及其字符串源；nonfinite：角速度含NaN的非法包。
  ImuPacket old = baseline;
  old.sequence = 2U;
  old.timestamp_ns -= 1U;
  EXPECT_EQ(filter.push_imu(old).disposition, ImuDisposition::kOutOfOrder);

  ImuPacket wrong_frame = baseline;
  wrong_frame.sequence = 3U;
  wrong_frame.timestamp_ns += 10U;
  wrong_frame.frame_id = {};
  const char* other = "other_imu";
  std::copy(other, other + std::strlen(other) + 1U,
            wrong_frame.frame_id.begin());
  EXPECT_EQ(filter.push_imu(wrong_frame).disposition,
            ImuDisposition::kFrameMismatch);

  ImuPacket nonfinite = baseline;
  nonfinite.sequence = 4U;
  nonfinite.timestamp_ns += 20U;
  nonfinite.angular_velocity_rad_s[0] =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(filter.push_imu(nonfinite).disposition,
            ImuDisposition::kInvalidPacket);

  EXPECT_TRUE(vector_norm(filter.state().position_n_m -
                          state_before.position_n_m) < 1.0e-15);
  EXPECT_TRUE(vector_norm(filter.state().velocity_n_mps -
                          state_before.velocity_n_mps) < 1.0e-15);
}

TEST_CASE(inertial_eskf_uses_valid_orientation_only_for_first_initialization) {
  // config/filter：启用首帧姿态初始化的配置与滤波器；half_angle：ROS xyzw中90度偏航所需半角。
  auto config = make_config();
  config.use_orientation_for_initialization = true;
  InertialEskf15 filter(make_initialization(), config);
  const double half_angle = std::acos(-1.0) / 4.0;
  // first：带有效姿态与协方差的首帧；rotated：初始化后前向轴；second：带不同姿态的后续帧，期望不再重置姿态。
  ImuPacket first = make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
                             {0.0, 0.0, 9.80665});
  first.orientation_valid = true;
  first.orientation_xyzw = {0.0, 0.0, std::sin(half_angle),
                            std::cos(half_angle)};
  first.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0,
                                  0.0, 0.0, 0.01};

  EXPECT_EQ(filter.push_imu(first).disposition,
            ImuDisposition::kBaselineEstablished);
  Vec3 rotated = filter.state().orientation_b_to_n.rotate({1.0, 0.0, 0.0});
  EXPECT_TRUE(std::abs(rotated.x) < 1.0e-9);
  EXPECT_TRUE(std::abs(rotated.y - 1.0) < 1.0e-9);

  ImuPacket second = make_imu(2U, 1'010'000'000ULL, {0.0, 0.0, 0.0},
                              {0.0, 0.0, 9.80665});
  second.orientation_valid = true;
  second.orientation_xyzw = {0.0, 0.0, 1.0, 0.0};
  second.orientation_covariance = first.orientation_covariance;
  EXPECT_EQ(filter.push_imu(second).disposition,
            ImuDisposition::kPropagated);
  rotated = filter.state().orientation_b_to_n.rotate({1.0, 0.0, 0.0});
  EXPECT_TRUE(std::abs(rotated.x) < 1.0e-9);
  EXPECT_TRUE(std::abs(rotated.y - 1.0) < 1.0e-9);
}

TEST_CASE(inertial_eskf_ignores_unavailable_orientation_covariance) {
  // config/filter：开启姿态初始化；first：协方差首项为-1的“不可用”ROS姿态包，期望保持单位姿态。
  auto config = make_config();
  config.use_orientation_for_initialization = true;
  InertialEskf15 filter(make_initialization(), config);
  ImuPacket first = make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
                             {0.0, 0.0, 9.80665});
  first.orientation_valid = true;
  first.orientation_xyzw = {0.0, 0.0, 1.0, 0.0};
  first.orientation_covariance[0] = -1.0;

  EXPECT_EQ(filter.push_imu(first).disposition,
            ImuDisposition::kBaselineEstablished);
  EXPECT_TRUE(std::abs(filter.state().orientation_b_to_n.w - 1.0) < 1.0e-12);
}

// 噪声组：只有通过ROS协方差可用性/对称性检查时，消息协方差才替代配置噪声。
TEST_CASE(inertial_eskf_message_covariance_changes_discrete_process_noise) {
  // configured_only/message_driven：只用配置噪声与允许消息协方差的对照配置；first_filter/second_filter：对应滤波器。
  auto configured_only = make_config();
  auto message_driven = make_config();
  message_driven.use_message_covariance = true;
  InertialEskf15 first_filter(make_initialization(), configured_only);
  InertialEskf15 second_filter(make_initialization(), message_driven);
  // first/second：携带大协方差的相邻静止IMU包；configured_result/message_result：两种噪声来源下的离散过程噪声回执。
  ImuPacket first = make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
                             {0.0, 0.0, 9.80665});
  first.angular_velocity_covariance = {4.0, 0.0, 0.0, 0.0, 4.0, 0.0,
                                      0.0, 0.0, 4.0};
  first.linear_acceleration_covariance = {9.0, 0.0, 0.0, 0.0, 9.0, 0.0,
                                         0.0, 0.0, 9.0};
  ImuPacket second = first;
  second.sequence = 2U;
  second.timestamp_ns += 10'000'000ULL;
  second.receive_timestamp_ns += 10'000'000ULL;
  (void)first_filter.push_imu(first);
  (void)second_filter.push_imu(first);

  const auto configured_result = first_filter.push_imu(second);
  const auto message_result = second_filter.push_imu(second);

  EXPECT_EQ(configured_result.disposition, ImuDisposition::kPropagated);
  EXPECT_EQ(message_result.disposition, ImuDisposition::kPropagated);
  EXPECT_TRUE(message_result.qd(6U, 6U) > configured_result.qd(6U, 6U));
  EXPECT_TRUE(message_result.qd(3U, 3U) > configured_result.qd(3U, 3U));
}

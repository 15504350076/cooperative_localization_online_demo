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
  InertialNodeInitialization initialization{};
  initialization.node_id = 7U;
  initialization.orientation_b_to_n = {1.0, 0.0, 0.0, 0.0};
  return initialization;
}

ImuPacket make_imu(std::uint64_t sequence, std::uint64_t timestamp_ns,
                   const Vec3& angular_velocity,
                   const Vec3& linear_acceleration) {
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

double vector_norm(const Vec3& value) {
  return std::sqrt(value.x * value.x + value.y * value.y +
                   value.z * value.z);
}

}  // namespace

TEST_CASE(inertial_eskf_first_sample_only_establishes_timebase) {
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
  auto initialization = make_initialization();
  initialization.gyro_bias_rad_s = {0.0, 0.0, 0.1};
  initialization.accel_bias_m_s2 = {0.2, 0.0, 0.0};
  InertialEskf15 filter(initialization, make_config());
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

TEST_CASE(inertial_eskf_rejects_invalid_order_frame_and_nonfinite_data) {
  InertialEskf15 filter(make_initialization(), make_config());
  const ImuPacket baseline =
      make_imu(1U, 1'000'000'000ULL, {0.0, 0.0, 0.0},
               {0.0, 0.0, 9.80665});
  (void)filter.push_imu(baseline);
  const auto state_before = filter.state();

  EXPECT_EQ(filter.push_imu(baseline).disposition,
            ImuDisposition::kDuplicate);

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
  auto config = make_config();
  config.use_orientation_for_initialization = true;
  InertialEskf15 filter(make_initialization(), config);
  const double half_angle = std::acos(-1.0) / 4.0;
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

TEST_CASE(inertial_eskf_message_covariance_changes_discrete_process_noise) {
  auto configured_only = make_config();
  auto message_driven = make_config();
  message_driven.use_message_covariance = true;
  InertialEskf15 first_filter(make_initialization(), configured_only);
  InertialEskf15 second_filter(make_initialization(), message_driven);
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

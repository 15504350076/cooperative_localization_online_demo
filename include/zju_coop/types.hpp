// 算法核心公共 C++ 数据类型；时间单位均为纳秒，距离采用米制 SI 单位。
#pragma once

#include <array>
#include <cstdint>

namespace zju::coop {

enum class Capability : std::uint32_t {
  kNone = 0U,
  kUwbRange = 1U << 0U,
  kPlanarPosition = 1U << 1U,
  kVelocity = 1U << 2U,
  kYaw = 1U << 3U,
  kAltitude = 1U << 4U,
};

constexpr Capability operator|(Capability left, Capability right) noexcept {
  return static_cast<Capability>(static_cast<std::uint32_t>(left) |
                                 static_cast<std::uint32_t>(right));
}

constexpr bool has_capability(Capability mask, Capability value) noexcept {
  return value != Capability::kNone &&
         (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(value)) ==
             static_cast<std::uint32_t>(value);
}

enum class ObservationState : std::uint8_t {
  kUnknown,
  kNormal,
  kDegraded,
  kSuspended,
  kRejected,
  kRecovering,
};

enum class FusionAction : std::uint8_t {
  kUseNormal,
  kUseDownweighted,
  kHold,
  kReject,
  kTrialRecovery,
};

enum class LocalizationState : std::uint8_t {
  kUninitialized,
  kNormal,
  kDegraded,
  kUnobservable,
  kStale,
};

struct RangePacket {
  std::uint16_t from_node{};
  std::uint16_t to_node{};
  std::uint64_t sequence{};
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  double range_m{};
  double range_std_m{};
  float nlos_probability{};
  bool nlos_flag{};
  bool has_nlos_probability{};
  bool valid{};
  std::uint8_t status{};
};

/**
 * 标准ROS 2 IMU消息映射后的算法输入。
 * 角速度单位为rad/s，线加速度按IMU比力解释，单位为m/s²。
 * 温度不属于sensor_msgs/Imu，本结构不携带温度。
 */
struct ImuPacket {
  std::uint32_t node_id{};
  std::uint64_t sequence{};
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
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
};

}  // namespace zju::coop

// 模块职责：定义算法核心使用的普通C++数据结构，不依赖ROS 2消息类型。
// 统一约定：时间戳为上交同步后的纳秒时间轴，距离/速度/加速度采用SI单位；
// 所有跨进程或跨语言数据应先由适配层转换到这些结构，再进入算法核心。
#pragma once

#include <array>
#include <cstdint>

namespace zju::coop {

/** 输出能力位图；能力存在不等于当前帧有效，仍须同时检查valid和状态字段。 */
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

/** 单条相对观测边的质量状态，由退化监测状态机产生。 */
enum class ObservationState : std::uint8_t {
  kUnknown,
  kNormal,
  kDegraded,
  kSuspended,
  kRejected,
  kRecovering,
};

/** 质量状态映射到滤波器后的实际处理动作。 */
enum class FusionAction : std::uint8_t {
  kUseNormal,
  kUseDownweighted,
  kHold,
  kReject,
  kTrialRecovery,
};

/** 面向GCS的综合定位状态，不能仅由单次量测是否成功判断。 */
enum class LocalizationState : std::uint8_t {
  kUninitialized,
  kNormal,
  kDegraded,
  kUnobservable,
  kStale,
};

/**
 * 平台间直接测距观测。
 * from/to表示有向数据来源，但拓扑质量按无向协同边统计；receive_timestamp_ns
 * 仅用于时延检查，滤波更新使用timestamp_ns对应的统一测量时刻。
 */
struct RangePacket {
  // 节点号定义一条有向数据链；EdgeKey会在质量/拓扑层规范成无向边。
  std::uint16_t from_node{};
  std::uint16_t to_node{};
  // sequence只要求在同一有向链路内单调；时间戳使用上交统一时间轴。
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
  // sequence用于重复诊断，真正的传播顺序由timestamp_ns严格决定。
  std::uint64_t sequence{};
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  std::array<double, 4> orientation_xyzw{};
  // 三组协方差均按ROS消息的行主序3×3布局保存，不在适配层重排。
  std::array<double, 9> orientation_covariance{};
  std::array<double, 3> angular_velocity_rad_s{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 3> linear_acceleration_m_s2{};
  std::array<double, 9> linear_acceleration_covariance{};
  std::array<char, 32> frame_id{};
  // orientation_valid独立于整包valid，允许不含可信姿态的角速度/比力继续传播。
  bool orientation_valid{};
  bool valid{};
  std::uint8_t status{};
};

}  // namespace zju::coop

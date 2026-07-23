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

// left、right分别是待合并的能力位图；返回值保留两者声明的全部输出能力。
constexpr Capability operator|(Capability left, Capability right) noexcept {
  return static_cast<Capability>(static_cast<std::uint32_t>(left) |
                                 static_cast<std::uint32_t>(right));
}

// mask为待查询的能力集合，value为要求同时具备的非空能力位组合。
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
  // timestamp_ns为测距发生时刻；receive_timestamp_ns为同一时基的本机收包时刻，仅作延迟门限检查。
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  // range_m为节点间距离，range_std_m为其1σ不确定度，单位均为m且仅正有限值可参与更新。
  double range_m{};
  double range_std_m{};
  // nlos_probability为[0,1]概率；has_nlos_probability=false时忽略该数值，nlos_flag保留设备硬判决。
  float nlos_probability{};
  bool nlos_flag{};
  bool has_nlos_probability{};
  // valid表示生产端是否认可整包；status保留设备侧OK/DEGRADED/INVALID质量码供入口判定。
  bool valid{};
  std::uint8_t status{};
};

/**
 * 标准ROS 2 IMU消息映射后的算法输入。
 * 角速度单位为rad/s，线加速度按IMU比力解释，单位为m/s²。
 * 温度不属于sensor_msgs/Imu，本结构不携带温度。
 */
struct ImuPacket {
  // node_id标识产生该IMU样本、且必须已在惯性配置中注册的平台。
  std::uint32_t node_id{};
  // sequence用于重复诊断，真正的传播顺序由timestamp_ns严格决定。
  std::uint64_t sequence{};
  // timestamp_ns为统一时间轴的传感器采样时刻；receive_timestamp_ns为同一时基的本机收包时刻。
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  // orientation_xyzw为车体FLU到导航ENU的[x,y,z,w]四元数，仅orientation_valid=true时可信。
  std::array<double, 4> orientation_xyzw{};
  // 三组协方差均按ROS消息的行主序3×3布局保存，不在适配层重排。
  std::array<double, 9> orientation_covariance{};
  // angular_velocity_rad_s为采样瞬时车体系角速度；其协方差按行主序3×3排列。
  std::array<double, 3> angular_velocity_rad_s{};
  std::array<double, 9> angular_velocity_covariance{};
  // linear_acceleration_m_s2为采样瞬时车体系比力（含传感器对重力的响应），协方差为行主序3×3。
  std::array<double, 3> linear_acceleration_m_s2{};
  std::array<double, 9> linear_acceleration_covariance{};
  // frame_id为固定32字节、必须NUL结尾的来源坐标系名，用于拒绝与配置不符的IMU流。
  std::array<char, 32> frame_id{};
  // orientation_valid独立于整包valid，允许不含可信姿态的角速度/比力继续传播。
  bool orientation_valid{};
  // valid表示角速度/比力及时间字段可处理；status携带设备侧质量码。
  bool valid{};
  std::uint8_t status{};
};

}  // namespace zju::coop

// 模块职责：定义算法核心使用的普通C++数据结构，不依赖ROS 2消息类型。
// 统一约定：时间戳为上海交大完成同步后的纳秒时间轴，距离/速度/加速度采用SI单位；
// 所有跨进程或跨语言数据应先由适配层转换到这些结构，再进入算法核心。
//
// C++初学者阅读提示：
// 1. struct只是把一组相关变量装进一个“数据包”；这里的数据结构本身不会执行算法。
// 2. enum class给状态取有意义的名字，避免在代码中直接猜测0、1、2分别代表什么。
// 3. std::array<T, N>表示长度固定为N的数组；花括号{}让成员默认清零。
// 4. 本文件是算法核心的C++内部类型；跨动态库调用使用c_api.h，UDP字节格式使用wire_protocol.hpp，
//    三者用途不同，不能直接把某个struct的内存原样当网络报文发送。
#pragma once

#include <array>
#include <cstdint>

namespace zju::coop {

/**
 * 输出能力位图：一个32位整数中的不同二进制位分别代表不同能力。
 * 例如kPlanarPosition | kVelocity表示“既能给平面位置，也能给速度”。
 * 能力存在不等于当前帧有效，使用者仍须同时检查valid和状态字段。
 */
enum class Capability : std::uint32_t {
  kNone = 0U,                 ///< 所有二进制位均为0，表示没有声明任何输出能力。
  kUwbRange = 1U << 0U,      ///< 把无符号整数1左移0位，占用第0位，表示具备测距输入。
  kPlanarPosition = 1U << 1U,///< 左移1位得到二进制0010，表示可输出平面位置。
  kVelocity = 1U << 2U,      ///< 左移2位得到二进制0100，表示可输出速度。
  kYaw = 1U << 3U,           ///< 左移3位，表示可输出航向角。
  kAltitude = 1U << 4U,      ///< 左移4位，表示可输出高度。
};

// 运算符“|”在这里不是普通逻辑或，而是把left和right两个能力集合合并。
constexpr Capability operator|(Capability left, Capability right) noexcept {
  // enum class不能直接做按位或，先用static_cast显式转回其底层uint32_t整数类型。
  return static_cast<Capability>(static_cast<std::uint32_t>(left) |
                                 static_cast<std::uint32_t>(right));
}

// mask为现有能力集合，value为想检查的能力；返回true表示value中的每一位都已具备。
constexpr bool has_capability(Capability mask, Capability value) noexcept {
  // `&&`要求左右条件都成立：查询值不能是空集合，并且mask必须包含value的全部二进制位。
  return value != Capability::kNone &&
         // `&`只保留mask与value中同时为1的位；结果等于value表示没有遗漏任何被查询能力。
         (static_cast<std::uint32_t>(mask) & static_cast<std::uint32_t>(value)) ==
             static_cast<std::uint32_t>(value);
}

/** 单条相对观测边的质量状态，由退化监测状态机产生。 */
enum class ObservationState : std::uint8_t {
  kUnknown,    ///< 尚无足够历史样本，不能评价该协同边。
  kNormal,     ///< 质量满足正常融合要求。
  kDegraded,   ///< 质量变差，但仍可增大协方差后降权使用。
  kSuspended,  ///< 暂停融合，继续观察后续样本是否恢复。
  kRejected,   ///< 连续异常达到剔除条件，不进入滤波器。
  kRecovering, ///< 异常后正在试探恢复，尚未回到稳定正常状态。
};

/** 质量状态映射到滤波器后的实际处理动作。 */
enum class FusionAction : std::uint8_t {
  kUseNormal,       ///< 使用原始测量方差执行正常更新。
  kUseDownweighted, ///< 放大测量方差，使当前观测对状态修正的影响减小。
  kHold,            ///< 暂缓当前观测，不调用滤波测量更新。
  kReject,          ///< 明确拒绝当前观测。
  kTrialRecovery,   ///< 恢复阶段按受控频率试探性执行更新。
};

/** 面向GCS的综合定位状态，不能仅由单次量测是否成功判断。 */
enum class LocalizationState : std::uint8_t {
  kUninitialized, ///< 尚未形成可发布的定位状态。
  kNormal,        ///< 定位、拓扑和数据新鲜度均满足正常条件。
  kDegraded,      ///< 仍可输出，但至少一项质量指标已经退化。
  kUnobservable,  ///< 当前几何约束不足以唯一确定所需相对状态。
  kStale,         ///< 最近有效输入距当前输出时刻过久。
};

/**
 * 平台间直接测距观测。
 * from/to表示有向数据来源，但拓扑质量按无向协同边统计；receive_timestamp_ns
 * 仅用于时延检查，滤波更新使用timestamp_ns对应的统一测量时刻。
 */
struct RangePacket {
  // 节点号定义一条有向数据链；EdgeKey会在质量/拓扑层规范成无向边。
  std::uint16_t from_node{};  ///< 测距发起端节点编号；uint16_t固定占16位。
  std::uint16_t to_node{};    ///< 测距目标端节点编号；不能与from_node相同。
  // sequence只要求在同一有向链路内单调；时间戳使用上海交大提供的统一时间轴。
  std::uint64_t sequence{};  ///< 同一有向链路上的包序号；{}执行值初始化，因此默认为0。
  // timestamp_ns为测距发生时刻；receive_timestamp_ns为同一时基的本机收包时刻，仅作延迟门限检查。
  std::uint64_t timestamp_ns{};          ///< 传感器完成本次测量的统一时刻，单位ns。
  std::uint64_t receive_timestamp_ns{};  ///< 本机收到数据包的统一时刻，单位ns。
  // range_m为节点间距离，range_std_m为其1σ不确定度，单位均为m且仅正有限值可参与更新。
  double range_m{};      ///< 两节点天线/标签参考点之间的测得距离，单位m。
  double range_std_m{};  ///< 距离的1σ标准差，越大表示测量越不可信。
  // nlos_probability为[0,1]概率；has_nlos_probability=false时忽略该数值，nlos_flag保留设备硬判决。
  float nlos_probability{};      ///< 设备估计的非视距概率；float用于匹配常见设备输出精度。
  bool nlos_flag{};              ///< true表示设备已经判定当前测量为NLOS。
  bool has_nlos_probability{};   ///< true才允许读取nlos_probability，避免把默认0误当有效值。
  // valid表示生产端是否认可整包；status保留设备侧OK/DEGRADED/INVALID质量码供入口判定。
  bool valid{};            ///< false表示生产端已判定整包不可用于算法。
  std::uint8_t status{};   ///< 设备质量状态原始编码，由适配层和入口规则共同解释。
};

/**
 * 标准ROS 2 IMU消息映射后的算法输入。
 * 角速度单位为rad/s，线加速度按IMU比力解释，单位为m/s²。
 * 温度不属于sensor_msgs/Imu，本结构不携带温度。
 */
struct ImuPacket {
  // node_id标识产生该IMU样本、且必须已在惯性配置中注册的平台。
  std::uint32_t node_id{};  ///< 产生这条IMU消息的平台唯一编号。
  // sequence用于重复诊断，真正的传播顺序由timestamp_ns严格决定。
  std::uint64_t sequence{};  ///< IMU消息递增序号，用于发现重复包，不代替时间戳排序。
  // timestamp_ns为统一时间轴的传感器采样时刻；receive_timestamp_ns为同一时基的本机收包时刻。
  std::uint64_t timestamp_ns{};          ///< IMU实际采样时刻，惯导积分使用该字段。
  std::uint64_t receive_timestamp_ns{};  ///< 算法侧收到消息时刻，只用于统计传输时延。
  // orientation_xyzw为车体FLU到导航ENU的[x,y,z,w]四元数，仅orientation_valid=true时可信。
  std::array<double, 4> orientation_xyzw{};  ///< 长度固定为4的ROS顺序姿态数组[x,y,z,w]。
  // 三组协方差均按ROS消息的行主序3×3布局保存，不在适配层重排。
  std::array<double, 9> orientation_covariance{};  ///< 姿态误差3×3协方差，共9个double。
  // angular_velocity_rad_s为采样瞬时车体系角速度；其协方差按行主序3×3排列。
  std::array<double, 3> angular_velocity_rad_s{};  ///< FLU三轴瞬时角速度[ωx,ωy,ωz]。
  std::array<double, 9> angular_velocity_covariance{};  ///< 角速度3×3测量协方差。
  // linear_acceleration_m_s2为采样瞬时车体系比力（含传感器对重力的响应），协方差为行主序3×3。
  std::array<double, 3> linear_acceleration_m_s2{};  ///< FLU三轴瞬时比力[fx,fy,fz]。
  std::array<double, 9> linear_acceleration_covariance{};  ///< 比力3×3测量协方差。
  // frame_id为固定32字节、必须NUL结尾的来源坐标系名，用于拒绝与配置不符的IMU流。
  std::array<char, 32> frame_id{};  ///< 固定容量C字符串；最后至少留一个'\0'终止字符。
  // orientation_valid独立于整包valid，允许不含可信姿态的角速度/比力继续传播。
  bool orientation_valid{};  ///< true表示orientation_xyzw及其协方差可以被使用。
  // valid表示角速度/比力及时间字段可处理；status携带设备侧质量码。
  bool valid{};           ///< true表示本条角速度、比力和时间字段通过生产端初检。
  std::uint8_t status{};  ///< IMU设备或驱动给出的原始质量状态码。
};

}  // namespace zju::coop

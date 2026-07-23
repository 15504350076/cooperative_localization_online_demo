// 模块职责：定义单节点15维误差状态惯导的配置、名义状态和IMU传播接口。
// 状态顺序固定为δp/δv/δθ/δbg/δba，每项3维；本模块只传播名义状态并返回
// Phi/Qd，完整15N联合协方差由CooperativeInertialEkf统一维护。
#pragma once

#include "dense_matrix.hpp"
#include "quaternion.hpp"
#include "zju_coop/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace zju::coop {

inline constexpr auto kInertialErrorStateSize = 15U;  ///< 单节点误差状态固定维数。

/**
 * 单节点15维惯性预测所需的运行参数，全部由配置文件或C ABI提供。
 * min/max_imu_dt拒绝不可用采样间隔，max_propagation_substep只负责把一次
 * 合法长间隔细分以改善离散化精度，不能绕过max_imu_dt的输入质量限制。
 */
struct InertialConfig {
  double gravity_mps2{9.80665};  ///< ENU重力向量模长，传播时沿导航系-z轴加入，单位m/s²。
  double min_imu_dt_s{1.0e-6};  ///< 相邻ROS 2传感器时间戳可接受的最小间隔，单位s。
  double max_imu_dt_s{0.1};     ///< 相邻ROS 2传感器时间戳可接受的最大间隔，单位s。
  double max_propagation_substep_s{0.01};  ///< 合法IMU区间的一阶离散化最大子步，单位s。
  double gyro_noise_density_rad_s_sqrt_hz{1.0e-4};  ///< 角速度白噪声密度，单位rad/s/√Hz。
  double accel_noise_density_m_s2_sqrt_hz{1.0e-3};  ///< 比力白噪声密度，单位m/s²/√Hz。
  double gyro_bias_random_walk_rad_s2_sqrt_hz{1.0e-6};  ///< 陀螺零偏随机游走密度，单位rad/s²/√Hz。
  double accel_bias_random_walk_m_s3_sqrt_hz{1.0e-5};  ///< 加速度计零偏随机游走密度，单位m/s³/√Hz。
  double min_covariance_diagonal{1.0e-12};  ///< ROS姿态/角速度/线加速度消息3×3协方差Cholesky对角残差下限，量纲随被检查量的平方变化。
  double quaternion_norm_tolerance{1.0e-3};  ///< 首帧消息姿态范数偏离1的无量纲容差。
  double covariance_symmetry_tolerance{1.0e-9};  ///< ROS 2消息3×3协方差对称/半正定检查的绝对容差。
  bool use_message_covariance{};  ///< `true`时优先采用两帧ROS 2瞬时IMU协方差生成Qd。
  bool use_orientation_for_initialization{};  ///< `true`时仅允许合法首帧xyzw姿态覆盖初始b到n姿态。
  std::string expected_frame_id{"imu_link"};  ///< ROS 2 IMU消息应声明的车体FLU坐标系标识。
};

/**
 * 单节点名义状态初值及15维误差状态的初始标准差。
 * 下标n表示导航ENU系，b表示车体FLU系；标准差必须非负，构造联合
 * 协方差时按节点顺序写入各15×15对角块。
 */
struct InertialNodeInitialization {
  std::uint32_t node_id{};  ///< 该15维状态块所属的平台编号，可不连续但必须唯一。
  Vec3 position_n_m{};      ///< 初始导航ENU位置，单位m，对应名义p_n。
  Vec3 velocity_n_mps{};    ///< 初始导航ENU速度，单位m/s，对应名义v_n。
  Quaternion orientation_b_to_n{};  ///< 初始主动旋转，内部wxyz，将车体FLU向量映射到导航ENU。
  Vec3 gyro_bias_rad_s{};   ///< 初始车体FLU陀螺零偏，单位rad/s。
  Vec3 accel_bias_m_s2{};   ///< 初始车体FLU加速度计零偏，单位m/s²。
  Vec3 position_std_m{0.1, 0.1, 0.1};  ///< δp三轴初始标准差，单位m。
  Vec3 velocity_std_mps{0.1, 0.1, 0.1};  ///< δv三轴初始标准差，单位m/s。
  Vec3 attitude_std_rad{0.1, 0.1, 0.1};  ///< δθ右乘小角度三轴初始标准差，单位rad。
  Vec3 gyro_bias_std_rad_s{0.01, 0.01, 0.01};  ///< δbg三轴初始标准差，单位rad/s。
  Vec3 accel_bias_std_m_s2{0.1, 0.1, 0.1};  ///< δba三轴初始标准差，单位m/s²。
};

/** 名义惯性状态；姿态为车体FLU到局部ENU的旋转。 */
struct InertialNominalState {
  Vec3 position_n_m{};      ///< 导航ENU中的名义位置p_n，单位m。
  Vec3 velocity_n_mps{};    ///< 导航ENU中的名义速度v_n，单位m/s。
  Quaternion orientation_b_to_n{};  ///< 内部wxyz主动旋转q_b_to_n，将FLU向量映射到ENU。
  Vec3 gyro_bias_rad_s{};   ///< 车体FLU陀螺名义零偏bg，单位rad/s。
  Vec3 accel_bias_m_s2{};   ///< 车体FLU加速度计名义零偏ba，单位m/s²。
};

/** IMU结果区分“未传播的合法首帧”和各种拒绝原因，便于上交侧诊断。 */
enum class ImuDisposition {
  kBaselineEstablished,
  kPropagated,
  kInvalidPacket,
  kUnknownNode,
  kDuplicate,
  kOutOfOrder,
  kIntervalRejected,
  kFrameMismatch,
  kNumericalFailure,
};

/** 一帧IMU的处理结果；phi和qd供多节点联合协方差传播使用。 */
struct ImuProcessingResult {
  ImuDisposition disposition{ImuDisposition::kInvalidPacket};  ///< 本帧最终处理分类。
  bool propagated{};  ///< `true`表示名义状态和15维传播量均已成功生成并提交。
  double dt_s{};      ///< 本帧与上一帧统一传感器时间戳之差，单位s；首帧为0。
  DenseMatrix phi{kInertialErrorStateSize, kInertialErrorStateSize};  ///< 15×15离散误差状态转移，行是新δx、列是旧δx。
  DenseMatrix qd{kInertialErrorStateSize, kInertialErrorStateSize};   ///< 15×15单区间离散过程噪声协方差，按δp/δv/δθ/δbg/δba分块。
};

/**
 * 单节点15维ESKF的名义状态传播器。
 * 本类不持有协方差；联合滤波器利用返回的phi/qd传播15N联合协方差。
 */
class InertialEskf15 {
 public:
  /**
   * `initialization`给出本节点名义初值和五个三维误差块标准差；
   * `config`给出固定的IMU时间、重力、噪声和坐标系校验参数。
   */
  InertialEskf15(InertialNodeInitialization initialization,
                 InertialConfig config);

  /**
   * `packet`输入一帧ROS 2语义的瞬时角速度(rad/s)和线加速度(m/s²)；
   * 二者位于车体FLU系，线加速度按含重力影响的比力解释。首帧不传播，
   * 仅建立时间基准；启用`use_orientation_for_initialization`且消息姿态合法时，
   * 首帧还完成首次姿态对准。后续帧采用前后帧中值传播。
   * 返回值明确区分重复、乱序、间隔过大、坐标系不匹配和数值失败。
   */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);
  [[nodiscard]] const InertialNominalState& state() const noexcept;
  [[nodiscard]] const InertialNodeInitialization& initialization() const
      noexcept;
  [[nodiscard]] std::uint32_t node_id() const noexcept;
  [[nodiscard]] bool has_timebase() const noexcept;
  [[nodiscard]] std::uint64_t timestamp_ns() const noexcept;

  /**
   * 将15维误差状态反馈到名义状态，姿态采用小角度右乘修正：
   * `error`按δp、δv、δθ、δbg、δba排列，每块3维；
   * q_b_to_n <- q_b_to_n * Exp(δθ)。任一分量或归一化失败均返回false，
   * 调用方必须保留注入前名义状态。
   */
  [[nodiscard]] bool inject_error(
      const std::array<double, kInertialErrorStateSize>& error) noexcept;

 private:
  /** `packet`为待做有效位、接收时间、状态码与有限值检查的ROS 2 IMU帧。 */
  [[nodiscard]] bool structurally_valid(const ImuPacket& packet) const;
  /** `packet`为待核对NUL结尾frame_id是否等于预期FLU坐标系的IMU帧。 */
  [[nodiscard]] bool frame_matches(const ImuPacket& packet) const;

  InertialNodeInitialization initialization_;  ///< 构造后保留的节点初值与15维初始标准差。
  InertialConfig config_;                      ///< 构造时校验后固定的传播与噪声参数。
  InertialNominalState state_;                 ///< 当前已提交的单节点名义惯性状态。
  ImuPacket previous_{};  ///< 上一帧已接纳的ROS 2车体FLU瞬时角速度(rad/s)与比力(m/s²，含重力影响)，用于中值积分；对象自持副本。
  bool has_previous_{};   ///< `true`表示`previous_`已建立统一传感器时间基准。
};

}  // namespace zju::coop

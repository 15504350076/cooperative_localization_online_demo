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

inline constexpr auto kInertialErrorStateSize = 15U;

/** 单节点15维惯性预测所需的运行参数，全部由配置文件或C ABI提供。 */
struct InertialConfig {
  double gravity_mps2{9.80665};
  double min_imu_dt_s{1.0e-6};
  double max_imu_dt_s{0.1};
  double max_propagation_substep_s{0.01};
  double gyro_noise_density_rad_s_sqrt_hz{1.0e-4};
  double accel_noise_density_m_s2_sqrt_hz{1.0e-3};
  double gyro_bias_random_walk_rad_s2_sqrt_hz{1.0e-6};
  double accel_bias_random_walk_m_s3_sqrt_hz{1.0e-5};
  double min_covariance_diagonal{1.0e-12};
  double quaternion_norm_tolerance{1.0e-3};
  double covariance_symmetry_tolerance{1.0e-9};
  bool use_message_covariance{};
  bool use_orientation_for_initialization{};
  std::string expected_frame_id{"imu_link"};
};

/** 单节点名义状态初值及15维误差状态的初始标准差。 */
struct InertialNodeInitialization {
  std::uint32_t node_id{};
  Vec3 position_n_m{};
  Vec3 velocity_n_mps{};
  Quaternion orientation_b_to_n{};
  Vec3 gyro_bias_rad_s{};
  Vec3 accel_bias_m_s2{};
  Vec3 position_std_m{0.1, 0.1, 0.1};
  Vec3 velocity_std_mps{0.1, 0.1, 0.1};
  Vec3 attitude_std_rad{0.1, 0.1, 0.1};
  Vec3 gyro_bias_std_rad_s{0.01, 0.01, 0.01};
  Vec3 accel_bias_std_m_s2{0.1, 0.1, 0.1};
};

/** 名义惯性状态；姿态为车体FLU到局部ENU的旋转。 */
struct InertialNominalState {
  Vec3 position_n_m{};
  Vec3 velocity_n_mps{};
  Quaternion orientation_b_to_n{};
  Vec3 gyro_bias_rad_s{};
  Vec3 accel_bias_m_s2{};
};

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
  ImuDisposition disposition{ImuDisposition::kInvalidPacket};
  bool propagated{};
  double dt_s{};
  DenseMatrix phi{kInertialErrorStateSize, kInertialErrorStateSize};
  DenseMatrix qd{kInertialErrorStateSize, kInertialErrorStateSize};
};

/**
 * 单节点15维ESKF的名义状态传播器。
 * 本类不持有协方差；联合滤波器利用返回的phi/qd传播15N联合协方差。
 */
class InertialEskf15 {
 public:
  InertialEskf15(InertialNodeInitialization initialization,
                 InertialConfig config);

  /**
   * 输入一帧瞬时角速度和比力；首帧只建立时间基准，后续帧采用前后帧中值传播。
   * 返回值明确区分重复、乱序、间隔过大、坐标系不匹配和数值失败。
   */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);
  [[nodiscard]] const InertialNominalState& state() const noexcept;
  [[nodiscard]] const InertialNodeInitialization& initialization() const
      noexcept;
  [[nodiscard]] std::uint32_t node_id() const noexcept;
  [[nodiscard]] bool has_timebase() const noexcept;
  [[nodiscard]] std::uint64_t timestamp_ns() const noexcept;

  /** 将15维误差状态反馈到名义状态，姿态采用小角度右乘修正。 */
  [[nodiscard]] bool inject_error(
      const std::array<double, kInertialErrorStateSize>& error) noexcept;

 private:
  [[nodiscard]] bool structurally_valid(const ImuPacket& packet) const;
  [[nodiscard]] bool frame_matches(const ImuPacket& packet) const;

  InertialNodeInitialization initialization_;
  InertialConfig config_;
  InertialNominalState state_;
  ImuPacket previous_{};
  bool has_previous_{};
};

}  // namespace zju::coop

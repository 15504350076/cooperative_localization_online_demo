// 模块实现：使用ROS 2 Imu可提供的瞬时角速度和线加速度完成单节点15维ESKF名义传播。
// 关键约定：导航系ENU、车体系FLU、加速度按比力解释；采用相邻两帧中值积分而非调用方预积分，
// 同时生成15维误差转移Phi和离散噪声Qd，供多平台联合协方差传播。
#include "inertial_eskf15.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

constexpr std::size_t kPosition = 0U;   // 15维误差状态中δp三维块的起始下标。
constexpr std::size_t kVelocity = 3U;   // 15维误差状态中δv三维块的起始下标。
constexpr std::size_t kAttitude = 6U;   // 15维误差状态中δθ三维块的起始下标。
constexpr std::size_t kGyroBias = 9U;   // 15维误差状态中δbg三维块的起始下标。
constexpr std::size_t kAccelBias = 12U; // 15维误差状态中δba三维块的起始下标。

// `value`是应严格大于零的配置标量，有限时才可参与传播或阈值检查。
bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

// `value`按x/y/z排列，转换后的Vec3保持原单位和坐标系。
Vec3 to_vec3(const std::array<double, 3>& value) {
  return {value[0], value[1], value[2]};
}

// `value`是待逐元素检查的x/y/z三分量数组。
bool finite_array(const std::array<double, 3>& value) {
  // Lambda参数`item`是当前接受有限值检查的单个三轴分量。
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

// `value`是待逐元素检查的ROS 2 xyzw四元数数组。
bool finite_array(const std::array<double, 4>& value) {
  // Lambda参数`item`是当前接受有限值检查的单个xyzw分量。
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

// `covariance`是ROS 2 Imu携带的行主序3×3瞬时量协方差。
bool covariance_is_available(const std::array<double, 9>& covariance) {
  // sensor_msgs/Imu 约定首元素为 -1 时表示该协方差不可用；全零则表示未知。
  if (covariance[0] == -1.0) {
    return false;
  }
  // Lambda参数`value`是当前协方差元素，用于区分“全零未知”。
  return std::any_of(covariance.begin(), covariance.end(),
                     [](double value) { return value != 0.0; });
}

// `covariance`是待验证的ROS 2行主序3×3协方差；`config`提供
// 对称容差和Cholesky对角下限。
bool valid_covariance(const std::array<double, 9>& covariance,
                      const InertialConfig& config) {
  if (!covariance_is_available(covariance) ||
      // Lambda参数`value`逐一检查当前3×3协方差元素是否有限。
      !std::all_of(covariance.begin(), covariance.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }

  // 先检查对称性，再用容差 Cholesky 检查半正定性，避免坏协方差进入滤波器。
  // `row`遍历3×3协方差行；`col`遍历严格上三角列以检查对称元素对。
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = row + 1U; col < 3U; ++col) {
      if (std::abs(covariance[row * 3U + col] -
                   covariance[col * 3U + row]) >
          config.covariance_symmetry_tolerance) {
        return false;
      }
    }
  }

  // `lower`是行主序3×3 Cholesky下三角工作区。
  std::array<double, 9> lower{};
  // `row`遍历待分解协方差行；`col`遍历当前行的下三角列。
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = 0U; col <= row; ++col) {
      // `value`是扣除已知内积后的当前Cholesky残差。
      double value = covariance[row * 3U + col];
      // `inner`遍历当前(row,col)之前已经求得的下三角列。
      for (std::size_t inner = 0U; inner < col; ++inner) {
        value -= lower[row * 3U + inner] * lower[col * 3U + inner];
      }
      if (row == col) {
        if (value < -config.covariance_symmetry_tolerance) {
          return false;
        }
        lower[row * 3U + col] =
            std::sqrt(std::max(value, config.min_covariance_diagonal));
      } else {
        lower[row * 3U + col] = value / lower[col * 3U + col];
      }
    }
  }
  return true;
}

// `state`是待确认全部名义位置、速度、b到n姿态和零偏有限的候选状态。
bool finite_state(const InertialNominalState& state) {
  return finite(state.position_n_m) && finite(state.velocity_n_mps) &&
         state.orientation_b_to_n.finite() && finite(state.gyro_bias_rad_s) &&
         finite(state.accel_bias_m_s2);
}

// `orientation`是内部wxyz、主动b到n单位旋转。
std::array<double, 9> rotation_matrix(const Quaternion& orientation) {
  // `x`、`y`、`z`分别是车体FLU三个单位基向量主动旋转到导航ENU后的列向量。
  const Vec3 x = orientation.rotate({1.0, 0.0, 0.0});
  const Vec3 y = orientation.rotate({0.0, 1.0, 0.0});
  const Vec3 z = orientation.rotate({0.0, 0.0, 1.0});
  return {x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z};
}

// `left`与`right`均为行主序3×3矩阵，结果行继承left、列继承right。
std::array<double, 9> multiply3(const std::array<double, 9>& left,
                               const std::array<double, 9>& right) {
  // `result`是清零后累计的行主序3×3乘积矩阵。
  std::array<double, 9> result{};
  // `row`遍历结果行，`col`遍历结果列，`inner`遍历共享收缩维。
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = 0U; col < 3U; ++col) {
      for (std::size_t inner = 0U; inner < 3U; ++inner) {
        result[row * 3U + col] +=
            left[row * 3U + inner] * right[inner * 3U + col];
      }
    }
  }
  return result;
}

// `matrix`是待转置的行主序3×3矩阵。
std::array<double, 9> transpose3(const std::array<double, 9>& matrix) {
  return {matrix[0], matrix[3], matrix[6], matrix[1], matrix[4], matrix[7],
          matrix[2], matrix[5], matrix[8]};
}

std::array<double, 9> average_covariance(
    // `previous`与`current`分别来自相邻两帧同类ROS 2瞬时IMU量的3×3协方差。
    const std::array<double, 9>& previous,
    const std::array<double, 9>& current) {
  // `result`是相邻两帧逐元素平均后的行主序3×3区间协方差。
  std::array<double, 9> result{};
  // `index`遍历两个3×3数组一致的行主序元素位置。
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index] = 0.5 * (previous[index] + current[index]);
  }
  return result;
}

// `matrix`是接收累加的目标矩阵；`row`与`col`是目标3×3块左上角；
// `block`是行主序3×3源块；`scale`是写入前的统一倍率。
void add_block(DenseMatrix& matrix, std::size_t row, std::size_t col,
               const std::array<double, 9>& block, double scale = 1.0) {
  // `i`遍历源块/目标块行，`j`遍历源块/目标块列。
  for (std::size_t i = 0U; i < 3U; ++i) {
    for (std::size_t j = 0U; j < 3U; ++j) {
      matrix(row + i, col + j) += block[i * 3U + j] * scale;
    }
  }
}

// `matrix`是接收累加的目标矩阵；`row`与`col`是3×3块左上角；
// `scale`是加入该块主对角线的统一值。
void add_identity_block(DenseMatrix& matrix, std::size_t row,
                        std::size_t col, double scale) {
  // `axis`同步遍历目标3×3块的三个行列主对角分量。
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    matrix(row + axis, col + axis) += scale;
  }
}

// `left`与`right`是维度相同的稠密矩阵，按对应元素相加。
DenseMatrix add(const DenseMatrix& left, const DenseMatrix& right) {
  // `result`维度与两个输入一致，行列角色不变。
  DenseMatrix result(left.rows(), left.cols());
  // `row`与`col`遍历结果及两个输入的对应矩阵元素。
  for (std::size_t row = 0U; row < left.rows(); ++row) {
    for (std::size_t col = 0U; col < left.cols(); ++col) {
      result(row, col) = left(row, col) + right(row, col);
    }
  }
  return result;
}

// `matrix`是待逐元素检查的任意维稠密矩阵。
bool finite_matrix(const DenseMatrix& matrix) {
  // `row`与`col`分别遍历矩阵全部行、列。
  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col < matrix.cols(); ++col) {
      if (!std::isfinite(matrix(row, col))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

InertialEskf15::InertialEskf15(InertialNodeInitialization initialization,
                               InertialConfig config)
    : initialization_(std::move(initialization)),
      config_(std::move(config)) {
  // 构造阶段一次性拒绝不完整参数，运行线程不再回退到隐藏默认值。
  if (!positive_finite(config_.gravity_mps2) ||
      !positive_finite(config_.min_imu_dt_s) ||
      !positive_finite(config_.max_imu_dt_s) ||
      !positive_finite(config_.max_propagation_substep_s) ||
      config_.min_imu_dt_s > config_.max_imu_dt_s ||
      config_.max_propagation_substep_s > config_.max_imu_dt_s ||
      !positive_finite(config_.gyro_noise_density_rad_s_sqrt_hz) ||
      !positive_finite(config_.accel_noise_density_m_s2_sqrt_hz) ||
      !positive_finite(config_.gyro_bias_random_walk_rad_s2_sqrt_hz) ||
      !positive_finite(config_.accel_bias_random_walk_m_s3_sqrt_hz) ||
      !positive_finite(config_.min_covariance_diagonal) ||
      !positive_finite(config_.quaternion_norm_tolerance) ||
      !positive_finite(config_.covariance_symmetry_tolerance) ||
      config_.expected_frame_id.empty() ||
      config_.expected_frame_id.size() >= 32U) {
    throw std::invalid_argument("invalid inertial configuration");
  }

  state_.position_n_m = initialization_.position_n_m;
  state_.velocity_n_mps = initialization_.velocity_n_mps;
  state_.orientation_b_to_n = initialization_.orientation_b_to_n;
  state_.gyro_bias_rad_s = initialization_.gyro_bias_rad_s;
  state_.accel_bias_m_s2 = initialization_.accel_bias_m_s2;
  if (!finite_state(state_) || !state_.orientation_b_to_n.normalize()) {
    throw std::invalid_argument("invalid inertial initialization");
  }
  initialization_.orientation_b_to_n = state_.orientation_b_to_n;
}

bool InertialEskf15::structurally_valid(const ImuPacket& packet) const {
  return packet.valid && packet.receive_timestamp_ns != 0U &&
         packet.status <= 2U && finite_array(packet.angular_velocity_rad_s) &&
         finite_array(packet.linear_acceleration_m_s2);
}

bool InertialEskf15::frame_matches(const ImuPacket& packet) const {
  // `terminator`指向固定长frame_id中的首个NUL，用于界定有效字符串长度。
  const auto terminator =
      std::find(packet.frame_id.begin(), packet.frame_id.end(), '\0');
  if (terminator == packet.frame_id.end()) {
    return false;
  }
  // `frame`复制NUL之前的ROS 2坐标系标识，随后与预期FLU帧名比较。
  const std::string frame(packet.frame_id.begin(), terminator);
  return frame == config_.expected_frame_id;
}

ImuProcessingResult InertialEskf15::push_imu(const ImuPacket& packet) {
  // `result`携带本帧处置、统一时间间隔及供15N协方差传播的15×15 Phi/Qd。
  ImuProcessingResult result{};
  result.phi = DenseMatrix::identity(kInertialErrorStateSize);
  // 阶段1：先做节点、结构、坐标系、序号和时间检查，失败时不污染上一帧基准。
  if (packet.node_id != initialization_.node_id) {
    result.disposition = ImuDisposition::kUnknownNode;
    return result;
  }
  if (!structurally_valid(packet)) {
    result.disposition = ImuDisposition::kInvalidPacket;
    return result;
  }
  if (!frame_matches(packet)) {
    result.disposition = ImuDisposition::kFrameMismatch;
    return result;
  }
  if (has_previous_ && packet.sequence == previous_.sequence &&
      packet.timestamp_ns == previous_.timestamp_ns) {
    result.disposition = ImuDisposition::kDuplicate;
    return result;
  }
  if (has_previous_ && packet.timestamp_ns <= previous_.timestamp_ns) {
    result.disposition = ImuDisposition::kOutOfOrder;
    return result;
  }
  if (!has_previous_) {
    // 阶段2：首帧没有积分区间，只建立时间基准；可选姿态仅用于首次对准。
    if (config_.use_orientation_for_initialization &&
        packet.orientation_valid && finite_array(packet.orientation_xyzw) &&
        valid_covariance(packet.orientation_covariance, config_)) {
      // ROS 2 消息为 x/y/z/w，算法内部统一为 w/x/y/z。
      // `orientation`是由首帧xyzw显式换序得到的候选主动b到n四元数。
      Quaternion orientation{packet.orientation_xyzw[3],
                             packet.orientation_xyzw[0],
                             packet.orientation_xyzw[1],
                             packet.orientation_xyzw[2]};
      if (std::abs(orientation.norm() - 1.0) <=
              config_.quaternion_norm_tolerance &&
          orientation.normalize()) {
        // 姿态只用于首次对准；后续不直接融合消息中的 orientation。
        state_.orientation_b_to_n = orientation;
      }
    }
    previous_ = packet;
    has_previous_ = true;
    result.disposition = ImuDisposition::kBaselineEstablished;
    return result;
  }

  // 阶段3：统一时间轴差值转换为秒，并把过长空洞作为新的基准而不是盲目外推。
  // `dt`是当前帧与上一帧统一传感器时间戳的正间隔，单位s。
  const double dt = static_cast<double>(packet.timestamp_ns -
                                        previous_.timestamp_ns) *
                    1.0e-9;
  result.dt_s = dt;
  if (!std::isfinite(dt) || dt < config_.min_imu_dt_s ||
      dt > config_.max_imu_dt_s) {
    // 时间空洞无法由插值恢复，丢弃该区间并以当前帧重新建立基准。
    previous_ = packet;
    result.disposition = ImuDisposition::kIntervalRejected;
    return result;
  }

  // `state_before`与`previous_before`保存本次传播前的名义状态和IMU基准，
  // 用于任一子步数值失败时完整回滚。
  const InertialNominalState state_before = state_;
  const ImuPacket previous_before = previous_;
  // `phi_total`累计整个dt的15×15误差状态转移；`q_total`累计同区间离散过程噪声。
  DenseMatrix phi_total = DenseMatrix::identity(kInertialErrorStateSize);
  DenseMatrix q_total(kInertialErrorStateSize, kInertialErrorStateSize);

  // 阶段4：相邻两帧瞬时量分别扣除当前零偏，再取中值作为区间输入。
  // `omega_previous`与`omega_current`是相邻ROS 2帧扣除bg后的车体FLU瞬时角速度，单位rad/s。
  const Vec3 omega_previous =
      to_vec3(previous_.angular_velocity_rad_s) - state_.gyro_bias_rad_s;
  const Vec3 omega_current =
      to_vec3(packet.angular_velocity_rad_s) - state_.gyro_bias_rad_s;
  // `force_previous`与`force_current`是相邻ROS 2帧扣除ba后的车体FLU瞬时比力，
  // 单位m/s²；仍包含重力对加速度计读数的影响。
  const Vec3 force_previous =
      to_vec3(previous_.linear_acceleration_m_s2) - state_.accel_bias_m_s2;
  const Vec3 force_current =
      to_vec3(packet.linear_acceleration_m_s2) - state_.accel_bias_m_s2;
  // `omega_mid`是相邻车体FLU瞬时样本的中值角速度(rad/s)；`force_mid`是
  // 相邻车体FLU瞬时样本的中值比力(m/s²)，仍包含重力对加速度计读数的影响。
  const Vec3 omega_mid = 0.5 * (omega_previous + omega_current);
  const Vec3 force_mid = 0.5 * (force_previous + force_current);

  // 阶段5：合法IMU区间按最大传播步长细分，降低一阶离散化误差。
  // `raw_steps`是尚未转换为整数的向上取整子步数，用于先做范围检查。
  const double raw_steps =
      std::ceil(dt / config_.max_propagation_substep_s);
  if (!std::isfinite(raw_steps) || raw_steps < 1.0 ||
      raw_steps > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    result.disposition = ImuDisposition::kNumericalFailure;
    return result;
  }
  // `steps`是本区间实际执行的等长传播子步数；`step_dt`是每子步时长，单位s。
  const auto steps = static_cast<std::uint32_t>(raw_steps);
  const double step_dt = dt / static_cast<double>(steps);
  // `use_gyro_message_covariance`和`use_accel_message_covariance`分别表示两端帧的
  // ROS 2瞬时角速度/线加速度3×3协方差均合法，可替代配置噪声密度。
  const bool use_gyro_message_covariance =
      config_.use_message_covariance &&
      valid_covariance(previous_.angular_velocity_covariance, config_) &&
      valid_covariance(packet.angular_velocity_covariance, config_);
  const bool use_accel_message_covariance =
      config_.use_message_covariance &&
      valid_covariance(previous_.linear_acceleration_covariance, config_) &&
      valid_covariance(packet.linear_acceleration_covariance, config_);
  // `gyro_message_covariance`是相邻帧车体FLU瞬时角速度协方差的3×3均值，
  // 单位(rad/s)²；`accel_message_covariance`是相邻帧车体FLU瞬时比力协方差
  // 的3×3均值，单位(m/s²)²；仅在对应布尔量为true时用于构造q_step并累计Qd。
  const auto gyro_message_covariance =
      average_covariance(previous_.angular_velocity_covariance,
                         packet.angular_velocity_covariance);
  const auto accel_message_covariance =
      average_covariance(previous_.linear_acceleration_covariance,
                         packet.linear_acceleration_covariance);

  // `step`遍历当前合法IMU区间内的等长传播子步。
  for (std::uint32_t step = 0U; step < steps; ++step) {
    // `delta_theta`是本子步由车体FLU中值角速度形成的旋转向量，单位rad。
    const Vec3 delta_theta = omega_mid * step_dt;
    // 用中点姿态把车体系比力旋转到ENU，可减少姿态变化对速度积分的偏差。
    // `half_rotation`是半子步增量姿态；`mid_orientation`是归一化前的子步中点q_b_to_n。
    Quaternion half_rotation = Quaternion::exp(delta_theta * 0.5);
    Quaternion mid_orientation = state_.orientation_b_to_n * half_rotation;
    if (!mid_orientation.normalize()) {
      state_ = state_before;
      previous_ = previous_before;
      result.disposition = ImuDisposition::kNumericalFailure;
      return result;
    }
    // IMU输出的是比力，导航系真实加速度等于R_nb*f_b加重力向量。
    // `acceleration_n`是导航ENU真实加速度，单位m/s²，已显式加入-z方向重力。
    const Vec3 acceleration_n =
        mid_orientation.rotate(force_mid) + Vec3{0.0, 0.0, -config_.gravity_mps2};
    state_.position_n_m = state_.position_n_m +
                          state_.velocity_n_mps * step_dt +
                          acceleration_n * (0.5 * step_dt * step_dt);
    state_.velocity_n_mps = state_.velocity_n_mps + acceleration_n * step_dt;
    state_.orientation_b_to_n =
        state_.orientation_b_to_n * Quaternion::exp(delta_theta);
    if (!state_.orientation_b_to_n.normalize()) {
      state_ = state_before;
      previous_ = previous_before;
      result.disposition = ImuDisposition::kNumericalFailure;
      return result;
    }

    // 误差状态顺序固定为δp、δv、δθ、δbg、δba。
    // `f`是本子步15×15连续误差动力学矩阵，行是δx导数、列是δx。
    DenseMatrix f(kInertialErrorStateSize, kInertialErrorStateSize);
    add_identity_block(f, kPosition, kVelocity, 1.0);
    // `rotation`是中点主动b到n旋转的行主序3×3矩阵R_nb。
    const auto rotation = rotation_matrix(mid_orientation);
    add_block(f, kVelocity, kAttitude,
              multiply3(rotation, skew(force_mid)), -1.0);
    add_block(f, kVelocity, kAccelBias, rotation, -1.0);
    add_block(f, kAttitude, kAttitude, skew(omega_mid), -1.0);
    add_identity_block(f, kAttitude, kGyroBias, -1.0);

    // 阶段6：一阶离散化连续误差动力学，并累计整个IMU区间的Phi/Qd。
    // `phi_step`是I+F*step_dt得到的15×15单子步状态转移。
    DenseMatrix phi_step = DenseMatrix::identity(kInertialErrorStateSize);
    // `row`遍历新误差状态分量，`col`遍历旧误差状态分量。
    for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
      for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
        phi_step(row, col) += f(row, col) * step_dt;
      }
    }

    // `q_step`是当前子步15×15离散过程噪声协方差，按五个三维误差块排列。
    DenseMatrix q_step(kInertialErrorStateSize, kInertialErrorStateSize);
    // `gyro_variance`、`accel_variance`分别是配置白噪声密度离散到本子步后的
    // 角度误差和速度误差轴向方差；`gyro_bias_variance`、`accel_bias_variance`
    // 分别是δbg与δba随机游走在本子步的轴向方差。
    const double gyro_variance =
        config_.gyro_noise_density_rad_s_sqrt_hz *
        config_.gyro_noise_density_rad_s_sqrt_hz * step_dt;
    const double accel_variance =
        config_.accel_noise_density_m_s2_sqrt_hz *
        config_.accel_noise_density_m_s2_sqrt_hz * step_dt;
    const double gyro_bias_variance =
        config_.gyro_bias_random_walk_rad_s2_sqrt_hz *
        config_.gyro_bias_random_walk_rad_s2_sqrt_hz * step_dt;
    const double accel_bias_variance =
        config_.accel_bias_random_walk_m_s3_sqrt_hz *
        config_.accel_bias_random_walk_m_s3_sqrt_hz * step_dt;
    if (use_accel_message_covariance) {
      // 瞬时加速度协方差乘 dt^2 后成为速度增量协方差，并旋转到 ENU。
      // `accel_covariance_n`是从车体FLU旋转到导航ENU的行主序3×3瞬时比力
      // 协方差，单位(m/s²)²，用于生成q_step的δv块并累计到Qd。
      const auto accel_covariance_n =
          multiply3(multiply3(rotation, accel_message_covariance),
                    transpose3(rotation));
      add_block(q_step, kVelocity, kVelocity, accel_covariance_n,
                step_dt * step_dt);
    } else {
      add_identity_block(q_step, kVelocity, kVelocity, accel_variance);
    }
    if (use_gyro_message_covariance) {
      // 瞬时角速度协方差乘 dt^2 后成为小角度增量协方差。
      add_block(q_step, kAttitude, kAttitude, gyro_message_covariance,
                step_dt * step_dt);
    } else {
      add_identity_block(q_step, kAttitude, kAttitude, gyro_variance);
    }
    add_identity_block(q_step, kGyroBias, kGyroBias, gyro_bias_variance);
    add_identity_block(q_step, kAccelBias, kAccelBias, accel_bias_variance);

    // 递推组合子步噪声：Q(0,k+1)=Phi_k*Q(0,k)*Phi_k^T+Q_k；
    // 不能简单相加Q_k，否则早期噪声不会被后续状态转移映射。
    q_total = add(phi_step * q_total * phi_step.transpose(), q_step);
    phi_total = phi_step * phi_total;
  }

  if (!finite_state(state_) || !finite_matrix(phi_total) ||
      !finite_matrix(q_total)) {
    state_ = state_before;
    previous_ = previous_before;
    result.disposition = ImuDisposition::kNumericalFailure;
    return result;
  }

  // 阶段7：名义状态、Phi和Qd全部通过有限值检查后才提交新的时间基准。
  previous_ = packet;
  result.disposition = ImuDisposition::kPropagated;
  result.propagated = true;
  result.phi = std::move(phi_total);
  result.qd = q_total.symmetrized();
  return result;
}

const InertialNominalState& InertialEskf15::state() const noexcept {
  return state_;
}

const InertialNodeInitialization& InertialEskf15::initialization() const
    noexcept {
  return initialization_;
}

std::uint32_t InertialEskf15::node_id() const noexcept {
  return initialization_.node_id;
}

bool InertialEskf15::has_timebase() const noexcept { return has_previous_; }

std::uint64_t InertialEskf15::timestamp_ns() const noexcept {
  return has_previous_ ? previous_.timestamp_ns : 0U;
}

bool InertialEskf15::inject_error(
    const std::array<double, kInertialErrorStateSize>& error) noexcept {
  // 测距更新得到误差状态：平移/速度/零偏直接相加，姿态用小角度右乘注入。
  // Lambda参数`value`依次检查δp/δv/δθ/δbg/δba的15个候选分量。
  if (!std::all_of(error.begin(), error.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }
  // 候选状态保证位置、姿态或零偏任一注入失败时，原名义状态完全不变。
  // `candidate`是完成全部15维反馈并通过有限性检查前不提交的名义状态副本。
  InertialNominalState candidate = state_;
  candidate.position_n_m =
      candidate.position_n_m + Vec3{error[0], error[1], error[2]};
  candidate.velocity_n_mps =
      candidate.velocity_n_mps + Vec3{error[3], error[4], error[5]};
  candidate.orientation_b_to_n =
      candidate.orientation_b_to_n *
      Quaternion::exp({error[6], error[7], error[8]});
  candidate.gyro_bias_rad_s =
      candidate.gyro_bias_rad_s + Vec3{error[9], error[10], error[11]};
  candidate.accel_bias_m_s2 =
      candidate.accel_bias_m_s2 + Vec3{error[12], error[13], error[14]};
  if (!candidate.orientation_b_to_n.normalize() || !finite_state(candidate)) {
    return false;
  }
  state_ = candidate;
  return true;
}

}  // namespace zju::coop

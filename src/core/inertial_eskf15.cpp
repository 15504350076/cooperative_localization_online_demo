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

constexpr std::size_t kPosition = 0U;
constexpr std::size_t kVelocity = 3U;
constexpr std::size_t kAttitude = 6U;
constexpr std::size_t kGyroBias = 9U;
constexpr std::size_t kAccelBias = 12U;

bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

Vec3 to_vec3(const std::array<double, 3>& value) {
  return {value[0], value[1], value[2]};
}

bool finite_array(const std::array<double, 3>& value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

bool finite_array(const std::array<double, 4>& value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

bool covariance_is_available(const std::array<double, 9>& covariance) {
  // sensor_msgs/Imu 约定首元素为 -1 时表示该协方差不可用；全零则表示未知。
  if (covariance[0] == -1.0) {
    return false;
  }
  return std::any_of(covariance.begin(), covariance.end(),
                     [](double value) { return value != 0.0; });
}

bool valid_covariance(const std::array<double, 9>& covariance,
                      const InertialConfig& config) {
  if (!covariance_is_available(covariance) ||
      !std::all_of(covariance.begin(), covariance.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }

  // 先检查对称性，再用容差 Cholesky 检查半正定性，避免坏协方差进入滤波器。
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = row + 1U; col < 3U; ++col) {
      if (std::abs(covariance[row * 3U + col] -
                   covariance[col * 3U + row]) >
          config.covariance_symmetry_tolerance) {
        return false;
      }
    }
  }

  std::array<double, 9> lower{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = 0U; col <= row; ++col) {
      double value = covariance[row * 3U + col];
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

bool finite_state(const InertialNominalState& state) {
  return finite(state.position_n_m) && finite(state.velocity_n_mps) &&
         state.orientation_b_to_n.finite() && finite(state.gyro_bias_rad_s) &&
         finite(state.accel_bias_m_s2);
}

std::array<double, 9> rotation_matrix(const Quaternion& orientation) {
  const Vec3 x = orientation.rotate({1.0, 0.0, 0.0});
  const Vec3 y = orientation.rotate({0.0, 1.0, 0.0});
  const Vec3 z = orientation.rotate({0.0, 0.0, 1.0});
  return {x.x, y.x, z.x, x.y, y.y, z.y, x.z, y.z, z.z};
}

std::array<double, 9> multiply3(const std::array<double, 9>& left,
                               const std::array<double, 9>& right) {
  std::array<double, 9> result{};
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

std::array<double, 9> transpose3(const std::array<double, 9>& matrix) {
  return {matrix[0], matrix[3], matrix[6], matrix[1], matrix[4], matrix[7],
          matrix[2], matrix[5], matrix[8]};
}

std::array<double, 9> average_covariance(
    const std::array<double, 9>& previous,
    const std::array<double, 9>& current) {
  std::array<double, 9> result{};
  for (std::size_t index = 0U; index < result.size(); ++index) {
    result[index] = 0.5 * (previous[index] + current[index]);
  }
  return result;
}

void add_block(DenseMatrix& matrix, std::size_t row, std::size_t col,
               const std::array<double, 9>& block, double scale = 1.0) {
  for (std::size_t i = 0U; i < 3U; ++i) {
    for (std::size_t j = 0U; j < 3U; ++j) {
      matrix(row + i, col + j) += block[i * 3U + j] * scale;
    }
  }
}

void add_identity_block(DenseMatrix& matrix, std::size_t row,
                        std::size_t col, double scale) {
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    matrix(row + axis, col + axis) += scale;
  }
}

DenseMatrix add(const DenseMatrix& left, const DenseMatrix& right) {
  DenseMatrix result(left.rows(), left.cols());
  for (std::size_t row = 0U; row < left.rows(); ++row) {
    for (std::size_t col = 0U; col < left.cols(); ++col) {
      result(row, col) = left(row, col) + right(row, col);
    }
  }
  return result;
}

bool finite_matrix(const DenseMatrix& matrix) {
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
  const auto terminator =
      std::find(packet.frame_id.begin(), packet.frame_id.end(), '\0');
  if (terminator == packet.frame_id.end()) {
    return false;
  }
  const std::string frame(packet.frame_id.begin(), terminator);
  return frame == config_.expected_frame_id;
}

ImuProcessingResult InertialEskf15::push_imu(const ImuPacket& packet) {
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

  const InertialNominalState state_before = state_;
  const ImuPacket previous_before = previous_;
  DenseMatrix phi_total = DenseMatrix::identity(kInertialErrorStateSize);
  DenseMatrix q_total(kInertialErrorStateSize, kInertialErrorStateSize);

  // 阶段4：相邻两帧瞬时量分别扣除当前零偏，再取中值作为区间输入。
  const Vec3 omega_previous =
      to_vec3(previous_.angular_velocity_rad_s) - state_.gyro_bias_rad_s;
  const Vec3 omega_current =
      to_vec3(packet.angular_velocity_rad_s) - state_.gyro_bias_rad_s;
  const Vec3 force_previous =
      to_vec3(previous_.linear_acceleration_m_s2) - state_.accel_bias_m_s2;
  const Vec3 force_current =
      to_vec3(packet.linear_acceleration_m_s2) - state_.accel_bias_m_s2;
  const Vec3 omega_mid = 0.5 * (omega_previous + omega_current);
  const Vec3 force_mid = 0.5 * (force_previous + force_current);

  // 阶段5：合法IMU区间按最大传播步长细分，降低一阶离散化误差。
  const double raw_steps =
      std::ceil(dt / config_.max_propagation_substep_s);
  if (!std::isfinite(raw_steps) || raw_steps < 1.0 ||
      raw_steps > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
    result.disposition = ImuDisposition::kNumericalFailure;
    return result;
  }
  const auto steps = static_cast<std::uint32_t>(raw_steps);
  const double step_dt = dt / static_cast<double>(steps);
  const bool use_gyro_message_covariance =
      config_.use_message_covariance &&
      valid_covariance(previous_.angular_velocity_covariance, config_) &&
      valid_covariance(packet.angular_velocity_covariance, config_);
  const bool use_accel_message_covariance =
      config_.use_message_covariance &&
      valid_covariance(previous_.linear_acceleration_covariance, config_) &&
      valid_covariance(packet.linear_acceleration_covariance, config_);
  const auto gyro_message_covariance =
      average_covariance(previous_.angular_velocity_covariance,
                         packet.angular_velocity_covariance);
  const auto accel_message_covariance =
      average_covariance(previous_.linear_acceleration_covariance,
                         packet.linear_acceleration_covariance);

  for (std::uint32_t step = 0U; step < steps; ++step) {
    const Vec3 delta_theta = omega_mid * step_dt;
    // 用中点姿态把车体系比力旋转到ENU，可减少姿态变化对速度积分的偏差。
    Quaternion half_rotation = Quaternion::exp(delta_theta * 0.5);
    Quaternion mid_orientation = state_.orientation_b_to_n * half_rotation;
    if (!mid_orientation.normalize()) {
      state_ = state_before;
      previous_ = previous_before;
      result.disposition = ImuDisposition::kNumericalFailure;
      return result;
    }
    // IMU输出的是比力，导航系真实加速度等于R_nb*f_b加重力向量。
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
    DenseMatrix f(kInertialErrorStateSize, kInertialErrorStateSize);
    add_identity_block(f, kPosition, kVelocity, 1.0);
    const auto rotation = rotation_matrix(mid_orientation);
    add_block(f, kVelocity, kAttitude,
              multiply3(rotation, skew(force_mid)), -1.0);
    add_block(f, kVelocity, kAccelBias, rotation, -1.0);
    add_block(f, kAttitude, kAttitude, skew(omega_mid), -1.0);
    add_identity_block(f, kAttitude, kGyroBias, -1.0);

    // 阶段6：一阶离散化连续误差动力学，并累计整个IMU区间的Phi/Qd。
    DenseMatrix phi_step = DenseMatrix::identity(kInertialErrorStateSize);
    for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
      for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
        phi_step(row, col) += f(row, col) * step_dt;
      }
    }

    DenseMatrix q_step(kInertialErrorStateSize, kInertialErrorStateSize);
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
  if (!std::all_of(error.begin(), error.end(),
                   [](double value) { return std::isfinite(value); })) {
    return false;
  }
  // 候选状态保证位置、姿态或零偏任一注入失败时，原名义状态完全不变。
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

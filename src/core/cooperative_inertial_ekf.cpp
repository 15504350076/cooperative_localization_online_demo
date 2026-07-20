// 维护各节点名义状态和完整交叉协方差，使一次测距更新可修正所有相关节点。
#include "core/cooperative_inertial_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

void set_initial_variance(DenseMatrix& covariance, std::size_t offset,
                          const Vec3& standard_deviation,
                          double minimum_diagonal) {
  const double values[3]{standard_deviation.x, standard_deviation.y,
                         standard_deviation.z};
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    covariance(offset + axis, offset + axis) =
        std::max(values[axis] * values[axis], minimum_diagonal);
  }
}

}  // namespace

CooperativeInertialEkf::CooperativeInertialEkf(
    CooperativeInertialConfig cooperative_config,
    InertialConfig inertial_config,
    std::vector<InertialNodeInitialization> initializations)
    : config_(std::move(cooperative_config)), covariance_(0U, 0U) {
  if (initializations.empty() || !positive_finite(config_.nis_gate) ||
      !positive_finite(config_.min_covariance_diagonal) ||
      config_.max_inertial_state_dimension < kInertialErrorStateSize ||
      initializations.size() >
          std::numeric_limits<std::size_t>::max() /
              kInertialErrorStateSize) {
    throw std::invalid_argument("invalid cooperative inertial configuration");
  }
  const std::size_t dimension =
      initializations.size() * kInertialErrorStateSize;
  if (dimension > config_.max_inertial_state_dimension) {
    throw std::invalid_argument("inertial state dimension exceeds limit");
  }

  filters_.reserve(initializations.size());
  node_ids_.reserve(initializations.size());
  bool reference_found = false;
  for (std::size_t index = 0U; index < initializations.size(); ++index) {
    const auto& node = initializations[index];
    if (!finite_nonnegative(node.position_std_m.x) ||
        !finite_nonnegative(node.position_std_m.y) ||
        !finite_nonnegative(node.position_std_m.z) ||
        !finite_nonnegative(node.velocity_std_mps.x) ||
        !finite_nonnegative(node.velocity_std_mps.y) ||
        !finite_nonnegative(node.velocity_std_mps.z) ||
        !finite_nonnegative(node.attitude_std_rad.x) ||
        !finite_nonnegative(node.attitude_std_rad.y) ||
        !finite_nonnegative(node.attitude_std_rad.z) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.x) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.y) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.z) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.x) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.y) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.z)) {
      throw std::invalid_argument("invalid inertial initial standard deviation");
    }
    if (!node_lookup_.emplace(node.node_id, index).second) {
      throw std::invalid_argument("duplicate inertial node id");
    }
    reference_found = reference_found ||
                      node.node_id == config_.reference_node_id;
    node_ids_.push_back(node.node_id);
    filters_.emplace_back(node, inertial_config);
  }
  if (!reference_found) {
    throw std::invalid_argument("reference inertial node is missing");
  }

  covariance_ = DenseMatrix(dimension, dimension);
  for (std::size_t index = 0U; index < initializations.size(); ++index) {
    const std::size_t offset = index * kInertialErrorStateSize;
    const auto& node = initializations[index];
    set_initial_variance(covariance_, offset, node.position_std_m,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 3U, node.velocity_std_mps,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 6U, node.attitude_std_rad,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 9U, node.gyro_bias_std_rad_s,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 12U, node.accel_bias_std_m_s2,
                         config_.min_covariance_diagonal);
  }
}

ImuProcessingResult CooperativeInertialEkf::push_imu(
    const ImuPacket& packet) {
  const auto found = node_lookup_.find(packet.node_id);
  if (found == node_lookup_.end()) {
    ImuProcessingResult result{};
    result.disposition = ImuDisposition::kUnknownNode;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }

  // 在副本上完成名义状态和协方差传播；任何数值异常都不会污染在线状态。
  std::vector<InertialEskf15> candidate_filters = filters_;
  ImuProcessingResult result = candidate_filters[found->second].push_imu(packet);
  if (!result.propagated) {
    if (result.disposition == ImuDisposition::kBaselineEstablished ||
        result.disposition == ImuDisposition::kIntervalRejected) {
      filters_ = std::move(candidate_filters);
    }
    return result;
  }

  DenseMatrix candidate_covariance = covariance_;
  const std::size_t node_offset =
      found->second * kInertialErrorStateSize;
  const std::size_t dimension = covariance_.rows();

  // P_ii=Phi*P_ii*Phi^T+Q；P_ij=Phi*P_ij；P_ji=P_ji*Phi^T。
  for (std::size_t other_offset = 0U; other_offset < dimension;
       other_offset += kInertialErrorStateSize) {
    if (other_offset == node_offset) {
      continue;
    }
    for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
      for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
        double left_value = 0.0;
        double right_value = 0.0;
        for (std::size_t inner = 0U; inner < kInertialErrorStateSize;
             ++inner) {
          left_value += result.phi(row, inner) *
                        covariance_(node_offset + inner,
                                    other_offset + col);
          right_value += covariance_(other_offset + row,
                                     node_offset + inner) *
                         result.phi(col, inner);
        }
        candidate_covariance(node_offset + row, other_offset + col) =
            left_value;
        candidate_covariance(other_offset + row, node_offset + col) =
            right_value;
      }
    }
  }

  for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
    for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
      double value = result.qd(row, col);
      for (std::size_t left = 0U; left < kInertialErrorStateSize; ++left) {
        for (std::size_t right = 0U; right < kInertialErrorStateSize;
             ++right) {
          value += result.phi(row, left) *
                   covariance_(node_offset + left, node_offset + right) *
                   result.phi(col, right);
        }
      }
      candidate_covariance(node_offset + row, node_offset + col) = value;
    }
  }
  candidate_covariance = candidate_covariance.symmetrized();
  if (!valid_covariance(candidate_covariance)) {
    result.disposition = ImuDisposition::kNumericalFailure;
    result.propagated = false;
    return result;
  }

  filters_ = std::move(candidate_filters);
  covariance_ = std::move(candidate_covariance);
  return result;
}

UpdateResult CooperativeInertialEkf::update_range(const RangePacket& packet,
                                                   double covariance_scale) {
  UpdateResult result{};
  result.covariance_scale = covariance_scale;
  if (!packet.valid || packet.receive_timestamp_ns == 0U ||
      packet.status > 2U || !std::isfinite(packet.range_m) ||
      !std::isfinite(packet.range_std_m) || packet.range_m <= 0.0 ||
      packet.range_std_m <= 0.0 || !std::isfinite(covariance_scale) ||
      covariance_scale < 1.0) {
    result.disposition = packet.range_m <= 0.0
                             ? UpdateDisposition::NonPositiveRange
                             : UpdateDisposition::InvalidPacket;
    return result;
  }
  if (packet.from_node == packet.to_node) {
    result.disposition = UpdateDisposition::SelfRange;
    return result;
  }
  const auto from = node_lookup_.find(packet.from_node);
  const auto to = node_lookup_.find(packet.to_node);
  if (from == node_lookup_.end() || to == node_lookup_.end()) {
    result.disposition = UpdateDisposition::UnknownNode;
    return result;
  }
  if (has_range_timebase_ && packet.timestamp_ns < last_range_timestamp_ns_) {
    result.disposition = UpdateDisposition::OutOfOrder;
    return result;
  }

  const Vec3 difference = filters_[to->second].state().position_n_m -
                          filters_[from->second].state().position_n_m;
  const double predicted_range = norm(difference);
  if (!positive_finite(predicted_range)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  const Vec3 direction = (1.0 / predicted_range) * difference;
  const std::size_t dimension = covariance_.rows();
  std::vector<double> jacobian(dimension, 0.0);
  const std::size_t from_offset = from->second * kInertialErrorStateSize;
  const std::size_t to_offset = to->second * kInertialErrorStateSize;
  jacobian[from_offset] = -direction.x;
  jacobian[from_offset + 1U] = -direction.y;
  jacobian[from_offset + 2U] = -direction.z;
  jacobian[to_offset] = direction.x;
  jacobian[to_offset + 1U] = direction.y;
  jacobian[to_offset + 2U] = direction.z;

  const std::vector<double> covariance_times_jacobian =
      covariance_ * jacobian;
  double projected_variance = 0.0;
  for (std::size_t index = 0U; index < dimension; ++index) {
    projected_variance +=
        jacobian[index] * covariance_times_jacobian[index];
  }
  const double measurement_variance =
      packet.range_std_m * packet.range_std_m * covariance_scale;
  const double innovation_variance =
      projected_variance + measurement_variance;
  const double innovation = packet.range_m - predicted_range;
  if (!positive_finite(measurement_variance) ||
      !positive_finite(innovation_variance) || !std::isfinite(innovation)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  result.innovation_m = innovation;
  result.innovation_variance = innovation_variance;
  result.nis = innovation * innovation / innovation_variance;
  if (!std::isfinite(result.nis)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  if (result.nis > config_.nis_gate) {
    has_range_timebase_ = true;
    last_range_timestamp_ns_ =
        std::max(last_range_timestamp_ns_, packet.timestamp_ns);
    result.disposition = UpdateDisposition::NisRejected;
    return result;
  }

  std::vector<double> gain(dimension, 0.0);
  std::vector<double> error(dimension, 0.0);
  for (std::size_t index = 0U; index < dimension; ++index) {
    gain[index] = covariance_times_jacobian[index] / innovation_variance;
    error[index] = gain[index] * innovation;
  }

  // Joseph形式避免有限精度下协方差失去对称性或正定性。
  DenseMatrix identity_minus_gain_h = DenseMatrix::identity(dimension);
  for (std::size_t row = 0U; row < dimension; ++row) {
    for (std::size_t col = 0U; col < dimension; ++col) {
      identity_minus_gain_h(row, col) -= gain[row] * jacobian[col];
    }
  }
  DenseMatrix candidate_covariance =
      identity_minus_gain_h * covariance_ * identity_minus_gain_h.transpose();
  for (std::size_t row = 0U; row < dimension; ++row) {
    for (std::size_t col = 0U; col < dimension; ++col) {
      candidate_covariance(row, col) +=
          gain[row] * measurement_variance * gain[col];
    }
  }
  candidate_covariance = candidate_covariance.symmetrized();
  for (std::size_t index = 0U; index < dimension; ++index) {
    candidate_covariance(index, index) =
        std::max(candidate_covariance(index, index),
                 config_.min_covariance_diagonal);
  }

  std::vector<InertialEskf15> candidate_filters = filters_;
  for (std::size_t node = 0U; node < candidate_filters.size(); ++node) {
    std::array<double, kInertialErrorStateSize> node_error{};
    std::copy_n(error.begin() +
                    static_cast<std::ptrdiff_t>(node *
                                                kInertialErrorStateSize),
                kInertialErrorStateSize, node_error.begin());
    if (!candidate_filters[node].inject_error(node_error)) {
      result.disposition = UpdateDisposition::NumericalFailure;
      return result;
    }
  }
  if (!valid_covariance(candidate_covariance)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  filters_ = std::move(candidate_filters);
  covariance_ = std::move(candidate_covariance);
  has_range_timebase_ = true;
  last_range_timestamp_ns_ =
      std::max(last_range_timestamp_ns_, packet.timestamp_ns);
  result.disposition = UpdateDisposition::Accepted;
  return result;
}

NodeEstimate CooperativeInertialEkf::estimate(std::uint32_t node_id) const {
  const auto found = node_lookup_.find(node_id);
  const auto reference = node_lookup_.find(config_.reference_node_id);
  if (found == node_lookup_.end() || reference == node_lookup_.end()) {
    throw std::out_of_range("unknown inertial node id");
  }
  const auto& node_state = filters_[found->second].state();
  const auto& reference_state = filters_[reference->second].state();
  const Vec3 relative_position =
      node_state.position_n_m - reference_state.position_n_m;
  const Vec3 relative_velocity =
      node_state.velocity_n_mps - reference_state.velocity_n_mps;
  const std::size_t node_offset =
      found->second * kInertialErrorStateSize;
  const std::size_t reference_offset =
      reference->second * kInertialErrorStateSize;

  NodeEstimate output{};
  output.node_id = node_id;
  output.timestamp_ns = latest_timestamp_ns();
  output.x = relative_position.x;
  output.y = relative_position.y;
  output.vx = relative_velocity.x;
  output.vy = relative_velocity.y;
  // Cov(p_i-p_r)=P_ii+P_rr-P_ir-P_ri。
  output.cov_xx = covariance_(node_offset, node_offset) +
                  covariance_(reference_offset, reference_offset) -
                  covariance_(node_offset, reference_offset) -
                  covariance_(reference_offset, node_offset);
  output.cov_xy = covariance_(node_offset, node_offset + 1U) +
                  covariance_(reference_offset, reference_offset + 1U) -
                  covariance_(node_offset, reference_offset + 1U) -
                  covariance_(reference_offset, node_offset + 1U);
  output.cov_yy = covariance_(node_offset + 1U, node_offset + 1U) +
                  covariance_(reference_offset + 1U,
                              reference_offset + 1U) -
                  covariance_(node_offset + 1U, reference_offset + 1U) -
                  covariance_(reference_offset + 1U, node_offset + 1U);
  output.valid = std::isfinite(output.x) && std::isfinite(output.y) &&
                 std::isfinite(output.vx) && std::isfinite(output.vy) &&
                 std::isfinite(output.cov_xx) &&
                 std::isfinite(output.cov_xy) &&
                 std::isfinite(output.cov_yy);
  return output;
}

std::vector<NodeEstimate> CooperativeInertialEkf::estimates() const {
  std::vector<NodeEstimate> output;
  output.reserve(node_ids_.size());
  for (const std::uint32_t node_id : node_ids_) {
    output.push_back(estimate(node_id));
  }
  return output;
}

const InertialNominalState& CooperativeInertialEkf::state(
    std::uint32_t node_id) const {
  const auto found = node_lookup_.find(node_id);
  if (found == node_lookup_.end()) {
    throw std::out_of_range("unknown inertial node id");
  }
  return filters_[found->second].state();
}

const DenseMatrix& CooperativeInertialEkf::covariance() const noexcept {
  return covariance_;
}

std::size_t CooperativeInertialEkf::state_dimension() const noexcept {
  return covariance_.rows();
}

const std::vector<std::uint32_t>& CooperativeInertialEkf::node_ids() const
    noexcept {
  return node_ids_;
}

bool CooperativeInertialEkf::valid_covariance(
    const DenseMatrix& covariance) const {
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    if (!std::isfinite(covariance(row, row)) ||
        covariance(row, row) < config_.min_covariance_diagonal) {
      return false;
    }
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      if (!std::isfinite(covariance(row, col))) {
        return false;
      }
    }
  }
  return true;
}

std::uint64_t CooperativeInertialEkf::latest_timestamp_ns() const noexcept {
  std::uint64_t timestamp =
      has_range_timebase_ ? last_range_timestamp_ns_ : 0U;
  for (const auto& filter : filters_) {
    timestamp = std::max(timestamp, filter.timestamp_ns());
  }
  return timestamp;
}

}  // namespace zju::coop

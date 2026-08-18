#include "core/distributed_fusion2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

constexpr std::size_t kMaximumHistorySize = 64U;
constexpr double kMinimumRangeForDerivative = 1.0e-9;

std::uint64_t range_edge_key(std::uint32_t first, std::uint32_t second) {
  const std::uint64_t lower = std::min(first, second);
  const std::uint64_t upper = std::max(first, second);
  return (lower << 32U) | upper;
}

bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
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

Quaternion interpolate_orientation(const Quaternion& left,
                                   const Quaternion& right, double alpha) {
  Quaternion first = left;
  Quaternion second = right;
  if (!first.normalize() || !second.normalize()) {
    return {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
  }
  const double product = first.w * second.w + first.x * second.x +
                         first.y * second.y + first.z * second.z;
  if (product < 0.0) {
    second = {-second.w, -second.x, -second.y, -second.z};
  }
  Quaternion result{(1.0 - alpha) * first.w + alpha * second.w,
                    (1.0 - alpha) * first.x + alpha * second.x,
                    (1.0 - alpha) * first.y + alpha * second.y,
                    (1.0 - alpha) * first.z + alpha * second.z};
  if (!result.normalize()) {
    return {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0};
  }
  return result;
}

UpdateResult rejected(UpdateDisposition disposition) {
  UpdateResult result{};
  result.disposition = disposition;
  return result;
}

}  // namespace

DistributedFusion2D::DistributedFusion2D(DistributedFusionConfig config)
    : config_(std::move(config)),
      covariance_(2U * (config_.node_ids.empty()
                            ? 0U
                            : config_.node_ids.size() - 1U),
                  2U * (config_.node_ids.empty()
                            ? 0U
                            : config_.node_ids.size() - 1U)) {
  if (config_.node_ids.size() < 2U ||
      !positive_finite(config_.initial_correction_std_m) ||
      !positive_finite(config_.process_accel_std_mps2) ||
      !positive_finite(config_.nis_gate) ||
      !positive_finite(config_.min_covariance_diagonal) ||
      config_.max_extrapolation_ns == 0U || config_.node_timeout_ns == 0U) {
    throw std::invalid_argument("invalid distributed fusion configuration");
  }

  bool reference_found = false;
  std::size_t nonreference_index = 0U;
  nodes_.reserve(config_.node_ids.size());
  for (const std::uint32_t node_id : config_.node_ids) {
    if (node_lookup_.find(node_id) != node_lookup_.end()) {
      throw std::invalid_argument("distributed node identifiers must be unique");
    }
    const bool reference = node_id == config_.reference_node_id;
    reference_found = reference_found || reference;
    const std::size_t offset = reference ? 0U : 2U * nonreference_index++;
    node_lookup_.emplace(node_id, nodes_.size());
    nodes_.push_back({node_id, offset, reference, {}});
  }
  if (!reference_found) {
    throw std::invalid_argument("distributed reference node is missing");
  }
  for (std::size_t first = 0U; first < nodes_.size(); ++first) {
    for (std::size_t second = first + 1U; second < nodes_.size(); ++second) {
      last_range_timestamp_by_edge_.emplace(
          range_edge_key(nodes_[first].node_id, nodes_[second].node_id), 0U);
    }
  }

  correction_.assign(2U * nonreference_index, 0.0);
  const double variance = config_.initial_correction_std_m *
                          config_.initial_correction_std_m;
  for (std::size_t index = 0U; index < correction_.size(); ++index) {
    covariance_(index, index) =
        std::max(variance, config_.min_covariance_diagonal);
  }
}

DistributedFusion2D::NodeRecord* DistributedFusion2D::find_node(
    std::uint32_t node_id) {
  const auto found = node_lookup_.find(node_id);
  return found == node_lookup_.end() ? nullptr : &nodes_[found->second];
}

const DistributedFusion2D::NodeRecord* DistributedFusion2D::find_node(
    std::uint32_t node_id) const {
  const auto found = node_lookup_.find(node_id);
  return found == node_lookup_.end() ? nullptr : &nodes_[found->second];
}

bool DistributedFusion2D::push_node_state(NodeState state) {
  NodeRecord* node = find_node(state.node_id);
  if (node == nullptr || !state.valid || state.timestamp_ns == 0U ||
      state.receive_timestamp_ns == 0U ||
      !finite(state.position_enu_m) || !finite(state.velocity_enu_mps) ||
      !state.orientation_flu_to_enu.normalize()) {
    return false;
  }
  if (state.timestamp_ns > state.receive_timestamp_ns &&
      state.timestamp_ns - state.receive_timestamp_ns >
          config_.max_future_skew_ns) {
    return false;
  }
  if (state.receive_timestamp_ns > state.timestamp_ns &&
      state.receive_timestamp_ns - state.timestamp_ns >
          config_.max_receive_delay_ns) {
    return false;
  }
  if (!node->history.empty() &&
      state.timestamp_ns <= node->history.back().timestamp_ns) {
    return false;
  }
  node->history.push_back(state);
  if (node->history.size() > kMaximumHistorySize) {
    node->history.pop_front();
  }
  return true;
}

bool DistributedFusion2D::all_nodes_ready() const {
  return std::all_of(nodes_.begin(), nodes_.end(), [](const NodeRecord& node) {
    return !node.history.empty();
  });
}

bool DistributedFusion2D::aligned_state(const NodeRecord& node,
                                        std::uint64_t timestamp_ns,
                                        NodeState& output) const {
  if (node.history.empty()) {
    return false;
  }
  const auto upper = std::lower_bound(
      node.history.begin(), node.history.end(), timestamp_ns,
      [](const NodeState& state, std::uint64_t time) {
        return state.timestamp_ns < time;
      });
  if (upper != node.history.end() && upper->timestamp_ns == timestamp_ns) {
    output = *upper;
    return true;
  }
  if (upper == node.history.begin()) {
    return false;
  }
  if (upper == node.history.end()) {
    const NodeState& latest = node.history.back();
    const std::uint64_t delta_ns = timestamp_ns - latest.timestamp_ns;
    if (delta_ns > config_.max_extrapolation_ns) {
      return false;
    }
    output = latest;
    const double delta_s = static_cast<double>(delta_ns) * 1.0e-9;
    output.position_enu_m =
        latest.position_enu_m + latest.velocity_enu_mps * delta_s;
    output.timestamp_ns = timestamp_ns;
    return finite(output.position_enu_m);
  }

  const NodeState& right = *upper;
  const NodeState& left = *(upper - 1);
  const std::uint64_t interval_ns = right.timestamp_ns - left.timestamp_ns;
  if (interval_ns == 0U) {
    return false;
  }
  const double alpha = static_cast<double>(timestamp_ns - left.timestamp_ns) /
                       static_cast<double>(interval_ns);
  output = left;
  output.timestamp_ns = timestamp_ns;
  output.receive_timestamp_ns =
      std::max(left.receive_timestamp_ns, right.receive_timestamp_ns);
  output.position_enu_m =
      left.position_enu_m * (1.0 - alpha) + right.position_enu_m * alpha;
  output.velocity_enu_mps =
      left.velocity_enu_mps * (1.0 - alpha) + right.velocity_enu_mps * alpha;
  output.orientation_flu_to_enu = interpolate_orientation(
      left.orientation_flu_to_enu, right.orientation_flu_to_enu, alpha);
  return finite(output.position_enu_m) && finite(output.velocity_enu_mps) &&
         output.orientation_flu_to_enu.finite();
}

bool DistributedFusion2D::state_is_fresh(const NodeRecord& node,
                                         std::uint64_t now_ns) const {
  if (node.history.empty()) {
    return false;
  }
  const std::uint64_t received = node.history.back().receive_timestamp_ns;
  return now_ns >= received && now_ns - received <= config_.node_timeout_ns;
}

Vec3 DistributedFusion2D::corrected_position(
    const NodeRecord& node, const NodeState& state,
    const std::vector<double>& correction) const {
  Vec3 result = state.position_enu_m;
  if (!node.reference) {
    result.x += correction[node.correction_offset];
    result.y += correction[node.correction_offset + 1U];
  }
  return result;
}

UpdateResult DistributedFusion2D::push_range(const RangePacket& packet) {
  if (!packet.valid || packet.status >= 2U ||
      !positive_finite(packet.range_std_m) || packet.timestamp_ns == 0U ||
      packet.receive_timestamp_ns == 0U) {
    return rejected(UpdateDisposition::InvalidPacket);
  }
  if ((packet.timestamp_ns > packet.receive_timestamp_ns &&
       packet.timestamp_ns - packet.receive_timestamp_ns >
           config_.max_future_skew_ns) ||
      (packet.receive_timestamp_ns > packet.timestamp_ns &&
       packet.receive_timestamp_ns - packet.timestamp_ns >
           config_.max_receive_delay_ns)) {
    return rejected(UpdateDisposition::InvalidPacket);
  }
  if (!positive_finite(packet.range_m)) {
    return rejected(UpdateDisposition::NonPositiveRange);
  }
  if (packet.from_node == packet.to_node) {
    return rejected(UpdateDisposition::SelfRange);
  }
  const NodeRecord* from = find_node(packet.from_node);
  const NodeRecord* to = find_node(packet.to_node);
  if (from == nullptr || to == nullptr) {
    return rejected(UpdateDisposition::UnknownNode);
  }
  const std::uint64_t edge_key =
      range_edge_key(packet.from_node, packet.to_node);
  const auto previous_edge_time = last_range_timestamp_by_edge_.find(edge_key);
  if (previous_edge_time != last_range_timestamp_by_edge_.end() &&
      packet.timestamp_ns <= previous_edge_time->second) {
    return rejected(UpdateDisposition::OutOfOrder);
  }
  if (!all_nodes_ready()) {
    return rejected(UpdateDisposition::InvalidPacket);
  }
  if (has_range_timebase_ && packet.timestamp_ns < last_range_timestamp_ns_) {
    return rejected(UpdateDisposition::OutOfOrder);
  }

  NodeState from_state{};
  NodeState to_state{};
  if (!aligned_state(*from, packet.timestamp_ns, from_state) ||
      !aligned_state(*to, packet.timestamp_ns, to_state)) {
    return rejected(UpdateDisposition::OutOfOrder);
  }

  DenseMatrix candidate_covariance = covariance_;
  if (has_range_timebase_ && packet.timestamp_ns > last_range_timestamp_ns_) {
    const double dt = static_cast<double>(packet.timestamp_ns -
                                          last_range_timestamp_ns_) *
                      1.0e-9;
    const double accel_variance = config_.process_accel_std_mps2 *
                                  config_.process_accel_std_mps2;
    const double position_variance = 0.25 * accel_variance * dt * dt * dt * dt;
    if (!std::isfinite(position_variance)) {
      return rejected(UpdateDisposition::NumericalFailure);
    }
    for (std::size_t index = 0U; index < correction_.size(); ++index) {
      candidate_covariance(index, index) += position_variance;
    }
  }

  const Vec3 from_position = corrected_position(*from, from_state, correction_);
  const Vec3 to_position = corrected_position(*to, to_state, correction_);
  const Vec3 difference = to_position - from_position;
  const double expected_range = norm(difference);
  if (!positive_finite(expected_range) ||
      expected_range <= kMinimumRangeForDerivative) {
    return rejected(UpdateDisposition::NumericalFailure);
  }

  std::vector<double> jacobian(correction_.size(), 0.0);
  const double unit_x = difference.x / expected_range;
  const double unit_y = difference.y / expected_range;
  if (!from->reference) {
    jacobian[from->correction_offset] = -unit_x;
    jacobian[from->correction_offset + 1U] = -unit_y;
  }
  if (!to->reference) {
    jacobian[to->correction_offset] = unit_x;
    jacobian[to->correction_offset + 1U] = unit_y;
  }

  const std::vector<double> covariance_times_jacobian =
      candidate_covariance * jacobian;
  double projected_variance = 0.0;
  for (std::size_t index = 0U; index < jacobian.size(); ++index) {
    projected_variance += jacobian[index] * covariance_times_jacobian[index];
  }
  const double measurement_variance =
      packet.range_std_m * packet.range_std_m;
  const double innovation_variance =
      projected_variance + measurement_variance;
  const double innovation = packet.range_m - expected_range;

  UpdateResult result{};
  result.innovation_m = innovation;
  result.innovation_variance = innovation_variance;
  result.covariance_scale = 1.0;
  if (!positive_finite(innovation_variance) || !std::isfinite(innovation)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  result.nis = innovation * innovation / innovation_variance;
  if (!std::isfinite(result.nis)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  if (result.nis > config_.nis_gate) {
    covariance_ = candidate_covariance;
    last_range_timestamp_by_edge_[edge_key] = packet.timestamp_ns;
    last_range_timestamp_ns_ = packet.timestamp_ns;
    has_range_timebase_ = true;
    result.disposition = UpdateDisposition::NisRejected;
    return result;
  }

  std::vector<double> gain(correction_.size(), 0.0);
  std::vector<double> candidate_correction = correction_;
  for (std::size_t row = 0U; row < correction_.size(); ++row) {
    gain[row] = covariance_times_jacobian[row] / innovation_variance;
    candidate_correction[row] += gain[row] * innovation;
  }
  DenseMatrix identity_minus_gain_jacobian =
      DenseMatrix::identity(correction_.size());
  for (std::size_t row = 0U; row < correction_.size(); ++row) {
    for (std::size_t col = 0U; col < correction_.size(); ++col) {
      identity_minus_gain_jacobian(row, col) -= gain[row] * jacobian[col];
    }
  }
  DenseMatrix posterior = identity_minus_gain_jacobian *
                          candidate_covariance *
                          identity_minus_gain_jacobian.transpose();
  for (std::size_t row = 0U; row < correction_.size(); ++row) {
    for (std::size_t col = 0U; col < correction_.size(); ++col) {
      posterior(row, col) +=
          gain[row] * measurement_variance * gain[col];
    }
  }
  posterior = posterior.symmetrized();
  for (std::size_t index = 0U; index < correction_.size(); ++index) {
    posterior(index, index) =
        std::max(posterior(index, index), config_.min_covariance_diagonal);
  }
  if (!std::all_of(candidate_correction.begin(), candidate_correction.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !finite_matrix(posterior)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }

  correction_ = std::move(candidate_correction);
  covariance_ = std::move(posterior);
  last_range_timestamp_by_edge_[edge_key] = packet.timestamp_ns;
  last_range_timestamp_ns_ = packet.timestamp_ns;
  has_range_timebase_ = true;
  result.disposition = UpdateDisposition::Accepted;
  return result;
}

DistributedPose2DSnapshot DistributedFusion2D::pose2d_snapshot(
    std::uint64_t now_ns) const {
  DistributedPose2DSnapshot snapshot{};
  snapshot.reference_node_id = config_.reference_node_id;
  snapshot.vehicles.reserve(nodes_.size());
  if (!all_nodes_ready()) {
    for (const auto& node : nodes_) {
      snapshot.vehicles.push_back({node.node_id});
    }
    return snapshot;
  }

  std::uint64_t common_time = nodes_.front().history.back().timestamp_ns;
  for (const auto& node : nodes_) {
    common_time = std::min(common_time, node.history.back().timestamp_ns);
  }
  const NodeRecord* reference = find_node(config_.reference_node_id);
  NodeState reference_state{};
  if (reference == nullptr ||
      !aligned_state(*reference, common_time, reference_state)) {
    for (const auto& node : nodes_) {
      snapshot.vehicles.push_back({node.node_id});
    }
    return snapshot;
  }

  snapshot.timestamp_ns = common_time;
  const bool reference_fresh = state_is_fresh(*reference, now_ns);
  for (const auto& node : nodes_) {
    DistributedVehiclePose2D pose{};
    pose.node_id = node.node_id;
    NodeState aligned{};
    if (aligned_state(node, common_time, aligned)) {
      const Vec3 position = corrected_position(node, aligned, correction_);
      const Vec3 reference_position =
          corrected_position(*reference, reference_state, correction_);
      pose.x_m = position.x - reference_position.x;
      pose.y_m = position.y - reference_position.y;
      pose.position_valid = reference_fresh && state_is_fresh(node, now_ns) &&
                            std::isfinite(pose.x_m) &&
                            std::isfinite(pose.y_m);
      pose.yaw_valid = state_is_fresh(node, now_ns) &&
                       yaw_enu_rad(aligned.orientation_flu_to_enu,
                                   pose.yaw_rad);
    }
    snapshot.vehicles.push_back(pose);
  }
  return snapshot;
}

}  // namespace zju::coop

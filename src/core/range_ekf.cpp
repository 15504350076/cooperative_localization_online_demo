// 二维恒速预测和平台间距离更新实现，用于回退运行与历史回归验证。
#include "core/range_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kMinimumRangeForDerivative = 1.0e-12;
constexpr double kNanosecondsPerSecond = 1.0e9;

bool finite(double value) { return std::isfinite(value); }

long double scaled_positive_product(
    std::initializer_list<long double> factors) {
  long double mantissa = 1.0L;
  long long exponent = 0LL;
  for (const long double factor : factors) {
    if (!std::isfinite(factor) || factor <= 0.0L) {
      throw std::overflow_error("scaled product factor is invalid");
    }
    int factor_exponent = 0;
    const long double factor_mantissa =
        std::frexp(factor, &factor_exponent);
    mantissa *= factor_mantissa;
    exponent += factor_exponent;

    int normalization_exponent = 0;
    mantissa = std::frexp(mantissa, &normalization_exponent);
    exponent += normalization_exponent;
  }
  if (exponent > std::numeric_limits<int>::max() ||
      exponent < std::numeric_limits<int>::min()) {
    throw std::overflow_error("scaled product exponent is out of range");
  }
  const long double product =
      std::scalbn(mantissa, static_cast<int>(exponent));
  if (!std::isfinite(product) || product <= 0.0L) {
    throw std::overflow_error("scaled product is not representable");
  }
  return product;
}

UpdateResult result(UpdateDisposition disposition, double covariance_scale) {
  UpdateResult value{};
  value.disposition = disposition;
  value.covariance_scale = covariance_scale;
  return value;
}

bool has_strict_cholesky_factor(const DenseMatrix& matrix,
                                double minimum_pivot = 0.0,
                                bool require_scaled_margin = false) {
  if (matrix.rows() != matrix.cols()) {
    return false;
  }
  DenseMatrix lower(matrix.rows(), matrix.cols());
  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col <= row; ++col) {
      double residual = matrix(row, col);
      for (std::size_t inner = 0U; inner < col; ++inner) {
        residual -= lower(row, inner) * lower(col, inner);
      }
      if (!finite(residual)) {
        return false;
      }
      if (row == col) {
        double pivot_threshold = minimum_pivot;
        if (require_scaled_margin) {
          pivot_threshold = std::max(
              pivot_threshold,
              std::numeric_limits<double>::epsilon() *
                  std::abs(matrix(row, row)) * 8.0);
        }
        if (residual <= pivot_threshold) {
          return false;
        }
        lower(row, col) = std::sqrt(residual);
      } else {
        lower(row, col) = residual / lower(col, col);
        if (!finite(lower(row, col))) {
          return false;
        }
      }
    }
  }
  return true;
}

bool stabilize_positive_definite(DenseMatrix& covariance,
                                 double minimum_diagonal) {
  double scale = minimum_diagonal;
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      if (!finite(covariance(row, col))) {
        return false;
      }
    }
    covariance(row, row) =
        std::max(covariance(row, row), minimum_diagonal);
    scale = std::max(scale, std::abs(covariance(row, row)));
  }
  covariance = covariance.symmetrized();
  double jitter = std::max(
      minimum_diagonal,
      std::numeric_limits<double>::epsilon() * scale * 8.0);
  if (has_strict_cholesky_factor(covariance, minimum_diagonal, true)) {
    return true;
  }
  for (unsigned int attempt = 0U; attempt < 6U; ++attempt) {
    if (!finite(jitter) || jitter <= 0.0) {
      return false;
    }
    DenseMatrix jittered = covariance;
    for (std::size_t index = 0U; index < jittered.rows(); ++index) {
      jittered(index, index) += jitter;
      if (!finite(jittered(index, index))) {
        return false;
      }
    }
    if (has_strict_cholesky_factor(jittered)) {
      covariance = std::move(jittered);
      return true;
    }
    jitter *= 10.0;
  }
  return false;
}

}  // namespace

RangeEkf::RangeEkf(FilterConfig config,
                   std::vector<NodeInitialization> initializations)
    : config_(config), covariance_(0U, 0U) {
  if (!finite(config_.process_accel_std_mps2) ||
      config_.process_accel_std_mps2 <= 0.0 || !finite(config_.nis_gate) ||
      config_.nis_gate < 0.0 || !finite(config_.max_prediction_step_s) ||
      config_.max_prediction_step_s <= 0.0 ||
      !finite(config_.min_covariance_diagonal) ||
      config_.min_covariance_diagonal <= 0.0) {
    throw std::invalid_argument("RangeEkf configuration is invalid");
  }
  const double process_variance = config_.process_accel_std_mps2 *
                                  config_.process_accel_std_mps2;
  if (!finite(process_variance) || process_variance <= 0.0) {
    throw std::invalid_argument(
        "process acceleration variance is not representable");
  }

  const auto reference_iterator = std::find_if(
      initializations.begin(), initializations.end(),
      [this](const NodeInitialization& node) {
        return node.node_id == config_.reference_node_id;
      });
  if (reference_iterator == initializations.end()) {
    throw std::invalid_argument("reference node is not initialized");
  }
  const double origin_x = reference_iterator->x;
  const double origin_y = reference_iterator->y;
  const double origin_vx = reference_iterator->vx;
  const double origin_vy = reference_iterator->vy;
  if (!finite(origin_x) || !finite(origin_y) || !finite(origin_vx) ||
      !finite(origin_vy)) {
    throw std::invalid_argument("reference state must be finite");
  }

  std::size_t nonreference_count = 0U;
  for (const auto& initialization : initializations) {
    if (!finite(initialization.x) || !finite(initialization.y) ||
        !finite(initialization.vx) || !finite(initialization.vy) ||
        !finite(initialization.position_std_m) ||
        initialization.position_std_m <= 0.0 ||
        !finite(initialization.velocity_std_mps) ||
        initialization.velocity_std_mps <= 0.0) {
      throw std::invalid_argument("node initialization is invalid");
    }
    if (node_lookup_.find(initialization.node_id) != node_lookup_.end()) {
      throw std::invalid_argument("node identifiers must be unique");
    }

    const bool is_reference =
        initialization.node_id == config_.reference_node_id;
    const std::size_t offset = nonreference_count * 4U;
    node_lookup_.emplace(initialization.node_id, nodes_.size());
    nodes_.push_back({initialization.node_id, offset, is_reference});
    if (!is_reference) {
      ++nonreference_count;
    }
  }

  state_.assign(nonreference_count * 4U, 0.0);
  covariance_ = DenseMatrix(state_.size(), state_.size());
  for (std::size_t node_index = 0U; node_index < nodes_.size(); ++node_index) {
    const NodeRecord& node = nodes_[node_index];
    if (node.reference) {
      continue;
    }
    const NodeInitialization& initialization = initializations[node_index];
    const std::size_t offset = node.offset;
    state_[offset] = initialization.x - origin_x;
    state_[offset + 1U] = initialization.y - origin_y;
    state_[offset + 2U] = initialization.vx - origin_vx;
    state_[offset + 3U] = initialization.vy - origin_vy;
    const double position_variance =
        initialization.position_std_m * initialization.position_std_m;
    const double velocity_variance =
        initialization.velocity_std_mps * initialization.velocity_std_mps;
    if (!finite(position_variance) || !finite(velocity_variance)) {
      throw std::invalid_argument("node initialization variance overflows");
    }
    covariance_(offset, offset) =
        std::max(position_variance, config_.min_covariance_diagonal);
    covariance_(offset + 1U, offset + 1U) =
        std::max(position_variance, config_.min_covariance_diagonal);
    covariance_(offset + 2U, offset + 2U) =
        std::max(velocity_variance, config_.min_covariance_diagonal);
    covariance_(offset + 3U, offset + 3U) =
        std::max(velocity_variance, config_.min_covariance_diagonal);
  }
  if (!finite_state_and_covariance()) {
    throw std::invalid_argument(
        "relative initialization produced a non-finite value");
  }
}

void RangeEkf::predict_to(std::uint64_t timestamp_ns) {
  if (has_timebase_ && timestamp_ns < last_timestamp_ns_) {
    return;
  }
  if (!has_timebase_) {
    last_timestamp_ns_ = timestamp_ns;
    has_timebase_ = true;
    return;
  }
  if (timestamp_ns == last_timestamp_ns_) {
    return;
  }

  const double total_seconds =
      static_cast<double>(timestamp_ns - last_timestamp_ns_) /
      kNanosecondsPerSecond;
  const long double required_steps = std::ceil(
      static_cast<long double>(total_seconds) /
      static_cast<long double>(config_.max_prediction_step_s));
  const std::vector<double> state_before = state_;
  const DenseMatrix covariance_before = covariance_;
  const std::uint64_t timestamp_before = last_timestamp_ns_;
  const bool had_timebase = has_timebase_;
  try {
    predict_interval(total_seconds, required_steps);
    if (!finite_state_and_covariance()) {
      throw std::overflow_error("prediction produced a non-finite value");
    }
  } catch (...) {
    state_ = state_before;
    covariance_ = covariance_before;
    last_timestamp_ns_ = timestamp_before;
    has_timebase_ = had_timebase;
    throw;
  }
  last_timestamp_ns_ = timestamp_ns;
}

void RangeEkf::predict_interval(double total_seconds,
                                long double step_count) {
  if (!finite(total_seconds) || total_seconds <= 0.0 ||
      !std::isfinite(step_count) || step_count < 1.0L) {
    throw std::overflow_error("prediction interval is not representable");
  }

  const long double duration = static_cast<long double>(total_seconds);
  const long double q =
      static_cast<long double>(config_.process_accel_std_mps2) *
      static_cast<long double>(config_.process_accel_std_mps2);
  const long double inverse_count = 1.0L / step_count;
  const long double inverse_count2 = inverse_count * inverse_count;
  const long double process_pp = scaled_positive_product(
      {q, duration, duration, duration, duration, inverse_count,
       (4.0L - inverse_count2) / 12.0L});
  const long double process_pv = scaled_positive_product(
      {q, duration, duration, duration, inverse_count, 0.5L});
  const long double process_vv = scaled_positive_product(
      {q, duration, duration, inverse_count});
  const long double intermediates[] = {
      duration,       q,          inverse_count, inverse_count2,
      process_pp,     process_pv, process_vv,
  };
  for (const long double value : intermediates) {
    if (!std::isfinite(value)) {
      throw std::overflow_error(
          "segmented process-noise calculation overflowed");
    }
  }
  const double process_pp_double = static_cast<double>(process_pp);
  const double process_pv_double = static_cast<double>(process_pv);
  const double process_vv_double = static_cast<double>(process_vv);
  if (!finite(process_pp_double) || !finite(process_pv_double) ||
      !finite(process_vv_double)) {
    throw std::overflow_error("process-noise result exceeds double range");
  }

  DenseMatrix transition = DenseMatrix::identity(state_.size());
  for (const auto& node : nodes_) {
    if (node.reference) {
      continue;
    }
    transition(node.offset, node.offset + 2U) = total_seconds;
    transition(node.offset + 1U, node.offset + 3U) = total_seconds;
    state_[node.offset] += state_[node.offset + 2U] * total_seconds;
    state_[node.offset + 1U] +=
        state_[node.offset + 3U] * total_seconds;
  }

  covariance_ = transition * covariance_ * transition.transpose();
  for (const auto& node : nodes_) {
    if (node.reference) {
      continue;
    }
    const std::size_t x = node.offset;
    const std::size_t y = node.offset + 1U;
    const std::size_t vx = node.offset + 2U;
    const std::size_t vy = node.offset + 3U;
    covariance_(x, x) += process_pp_double;
    covariance_(y, y) += process_pp_double;
    covariance_(x, vx) += process_pv_double;
    covariance_(vx, x) += process_pv_double;
    covariance_(y, vy) += process_pv_double;
    covariance_(vy, y) += process_pv_double;
    covariance_(vx, vx) += process_vv_double;
    covariance_(vy, vy) += process_vv_double;
  }
  if (!stabilize_positive_definite(covariance_,
                                   config_.min_covariance_diagonal)) {
    throw std::overflow_error(
        "predicted covariance is not positive definite");
  }
}

UpdateResult RangeEkf::update(const RangePacket& packet,
                              double covariance_scale) {
  if (has_timebase_ && packet.timestamp_ns < last_timestamp_ns_) {
    return result(UpdateDisposition::OutOfOrder, covariance_scale);
  }
  try {
    predict_to(packet.timestamp_ns);
  } catch (const std::exception&) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  if (!packet.valid || !finite(packet.range_m) ||
      !finite(packet.range_std_m) || packet.range_std_m <= 0.0 ||
      !finite(covariance_scale) || covariance_scale < 1.0) {
    return result(UpdateDisposition::InvalidPacket, covariance_scale);
  }
  if (packet.range_m <= 0.0) {
    return result(UpdateDisposition::NonPositiveRange, covariance_scale);
  }

  const NodeRecord* from = find_node(packet.from_node);
  const NodeRecord* to = find_node(packet.to_node);
  if (from == nullptr || to == nullptr) {
    return result(UpdateDisposition::UnknownNode, covariance_scale);
  }
  if (from == to) {
    return result(UpdateDisposition::SelfRange, covariance_scale);
  }
  if (!finite_state_and_covariance()) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  const double from_x = from->reference ? 0.0 : state_[from->offset];
  const double from_y = from->reference ? 0.0 : state_[from->offset + 1U];
  const double to_x = to->reference ? 0.0 : state_[to->offset];
  const double to_y = to->reference ? 0.0 : state_[to->offset + 1U];
  const double delta_x = to_x - from_x;
  const double delta_y = to_y - from_y;
  const double expected_range = std::hypot(delta_x, delta_y);
  if (!finite(expected_range) ||
      expected_range <= kMinimumRangeForDerivative) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  std::vector<double> jacobian(state_.size(), 0.0);
  const double unit_x = delta_x / expected_range;
  const double unit_y = delta_y / expected_range;
  if (!from->reference) {
    jacobian[from->offset] = -unit_x;
    jacobian[from->offset + 1U] = -unit_y;
  }
  if (!to->reference) {
    jacobian[to->offset] = unit_x;
    jacobian[to->offset + 1U] = unit_y;
  }

  std::vector<double> covariance_times_jacobian(state_.size(), 0.0);
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      covariance_times_jacobian[row] +=
          covariance_(row, col) * jacobian[col];
    }
  }
  double projected_variance = 0.0;
  for (std::size_t index = 0U; index < state_.size(); ++index) {
    projected_variance +=
        jacobian[index] * covariance_times_jacobian[index];
  }
  double measurement_variance = 0.0;
  try {
    const long double scaled_measurement_variance = scaled_positive_product(
        {static_cast<long double>(packet.range_std_m),
         static_cast<long double>(packet.range_std_m),
         static_cast<long double>(covariance_scale)});
    measurement_variance =
        static_cast<double>(scaled_measurement_variance);
  } catch (const std::exception&) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }
  const double innovation_variance =
      projected_variance + measurement_variance;
  const double innovation = packet.range_m - expected_range;

  UpdateResult update_result{};
  update_result.innovation_m = innovation;
  update_result.innovation_variance = innovation_variance;
  update_result.covariance_scale = covariance_scale;
  if (!finite(measurement_variance) || measurement_variance <= 0.0 ||
      !finite(innovation_variance) || innovation_variance <= 0.0 ||
      !finite(innovation)) {
    update_result.disposition = UpdateDisposition::NumericalFailure;
    return update_result;
  }

  const double standardized_residual =
      std::abs(innovation) / std::sqrt(innovation_variance);
  const double maximum_root =
      std::sqrt(std::numeric_limits<double>::max());
  update_result.nis =
      !finite(standardized_residual) || standardized_residual > maximum_root
          ? std::numeric_limits<double>::max()
          : standardized_residual * standardized_residual;
  if (standardized_residual > std::sqrt(config_.nis_gate)) {
    update_result.disposition = UpdateDisposition::NisRejected;
    return update_result;
  }

  std::vector<double> candidate_state = state_;
  std::vector<double> gain(state_.size(), 0.0);
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    gain[row] = covariance_times_jacobian[row] / innovation_variance;
    candidate_state[row] += gain[row] * innovation;
  }

  DenseMatrix identity_minus_gain_jacobian =
      DenseMatrix::identity(state_.size());
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      identity_minus_gain_jacobian(row, col) -=
          gain[row] * jacobian[col];
    }
  }
  DenseMatrix candidate_covariance =
      identity_minus_gain_jacobian * covariance_ *
      identity_minus_gain_jacobian.transpose();
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      candidate_covariance(row, col) +=
          gain[row] * measurement_variance * gain[col];
    }
  }

  for (double value : candidate_state) {
    if (!finite(value)) {
      update_result.disposition = UpdateDisposition::NumericalFailure;
      return update_result;
    }
  }
  if (!stabilize_positive_definite(candidate_covariance,
                                   config_.min_covariance_diagonal)) {
    update_result.disposition = UpdateDisposition::NumericalFailure;
    return update_result;
  }

  state_ = std::move(candidate_state);
  covariance_ = std::move(candidate_covariance);
  update_result.disposition = UpdateDisposition::Accepted;
  return update_result;
}

const RangeEkf::NodeRecord* RangeEkf::find_node(
    std::uint32_t node_id) const {
  const auto iterator = node_lookup_.find(node_id);
  if (iterator == node_lookup_.end()) {
    return nullptr;
  }
  return &nodes_[iterator->second];
}

NodeEstimate RangeEkf::estimate(std::uint32_t node_id) const {
  const NodeRecord* node = find_node(node_id);
  if (node == nullptr) {
    NodeEstimate invalid{};
    invalid.node_id = node_id;
    invalid.timestamp_ns = last_timestamp_ns_;
    invalid.x = kNan;
    invalid.y = kNan;
    invalid.vx = kNan;
    invalid.vy = kNan;
    invalid.cov_xx = kNan;
    invalid.cov_xy = kNan;
    invalid.cov_yy = kNan;
    return invalid;
  }
  return make_estimate(*node);
}

std::vector<NodeEstimate> RangeEkf::estimates() const {
  std::vector<NodeEstimate> values;
  values.reserve(nodes_.size());
  for (const auto& node : nodes_) {
    values.push_back(make_estimate(node));
  }
  return values;
}

const DenseMatrix& RangeEkf::covariance() const noexcept {
  return covariance_;
}

NodeEstimate RangeEkf::make_estimate(const NodeRecord& node) const {
  NodeEstimate value{};
  value.node_id = node.node_id;
  value.timestamp_ns = last_timestamp_ns_;
  value.valid = true;
  if (node.reference) {
    return value;
  }

  value.x = state_[node.offset];
  value.y = state_[node.offset + 1U];
  value.vx = state_[node.offset + 2U];
  value.vy = state_[node.offset + 3U];
  value.cov_xx = covariance_(node.offset, node.offset);
  value.cov_xy = covariance_(node.offset, node.offset + 1U);
  value.cov_yy = covariance_(node.offset + 1U, node.offset + 1U);
  return value;
}

bool RangeEkf::finite_state_and_covariance() const {
  for (double value : state_) {
    if (!finite(value)) {
      return false;
    }
  }
  for (std::size_t row = 0U; row < covariance_.rows(); ++row) {
    for (std::size_t col = 0U; col < covariance_.cols(); ++col) {
      if (!finite(covariance_(row, col))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace zju::coop

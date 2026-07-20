#include "core/range_ekf.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using zju::coop::FilterConfig;
using zju::coop::DenseMatrix;
using zju::coop::NodeEstimate;
using zju::coop::NodeInitialization;
using zju::coop::RangeEkf;
using zju::coop::RangePacket;
using zju::coop::UpdateDisposition;

constexpr double kTolerance = 1.0e-9;

bool near(double left, double right, double tolerance = kTolerance) {
  return std::abs(left - right) <= tolerance;
}

FilterConfig config(std::uint32_t reference_node_id = 10U) {
  return {reference_node_id, 0.2, 100.0, 0.25, 1.0e-12};
}

std::vector<NodeInitialization> triangle_nodes() {
  return {
      {10U, 100.0, -50.0, 0.0, 0.0, 0.1, 0.1},
      {42U, 103.0, -46.0, 1.0, -0.5, 2.0, 1.0},
      {7U, 95.0, -38.0, -0.25, 0.75, 2.0, 1.0},
  };
}

RangePacket packet(std::uint16_t from, std::uint16_t to, double range,
                   double range_std, std::uint64_t timestamp_ns = 1U) {
  RangePacket value{};
  value.from_node = from;
  value.to_node = to;
  value.timestamp_ns = timestamp_ns;
  value.range_m = range;
  value.range_std_m = range_std;
  value.valid = true;
  return value;
}

double distance(const NodeEstimate& left, const NodeEstimate& right) {
  return std::hypot(left.x - right.x, left.y - right.y);
}

bool same_state_and_covariance(const RangeEkf& left, const RangeEkf& right,
                               double tolerance) {
  for (const std::uint32_t node_id : {10U, 42U, 7U}) {
    const NodeEstimate left_estimate = left.estimate(node_id);
    const NodeEstimate right_estimate = right.estimate(node_id);
    if (!near(left_estimate.x, right_estimate.x, tolerance) ||
        !near(left_estimate.y, right_estimate.y, tolerance) ||
        !near(left_estimate.vx, right_estimate.vx, tolerance) ||
        !near(left_estimate.vy, right_estimate.vy, tolerance)) {
      return false;
    }
  }

  const auto& left_covariance = left.covariance();
  const auto& right_covariance = right.covariance();
  if (left_covariance.rows() != right_covariance.rows() ||
      left_covariance.cols() != right_covariance.cols()) {
    return false;
  }
  for (std::size_t row = 0U; row < left_covariance.rows(); ++row) {
    for (std::size_t col = 0U; col < left_covariance.cols(); ++col) {
      if (!near(left_covariance(row, col), right_covariance(row, col),
                tolerance)) {
        return false;
      }
    }
  }
  return true;
}

bool has_strict_cholesky_factor(const DenseMatrix& matrix) {
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
      if (!std::isfinite(residual)) {
        return false;
      }
      if (row == col) {
        if (residual <= 0.0) {
          return false;
        }
        lower(row, col) = std::sqrt(residual);
      } else {
        lower(row, col) = residual / lower(col, col);
      }
    }
  }
  return true;
}

double matrix_determinant(DenseMatrix matrix) {
  if (matrix.rows() != matrix.cols()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double determinant = 1.0;
  for (std::size_t col = 0U; col < matrix.cols(); ++col) {
    std::size_t pivot_row = col;
    for (std::size_t row = col + 1U; row < matrix.rows(); ++row) {
      if (std::abs(matrix(row, col)) >
          std::abs(matrix(pivot_row, col))) {
        pivot_row = row;
      }
    }
    if (matrix(pivot_row, col) == 0.0) {
      return 0.0;
    }
    if (pivot_row != col) {
      for (std::size_t swap_col = col; swap_col < matrix.cols(); ++swap_col) {
        std::swap(matrix(col, swap_col), matrix(pivot_row, swap_col));
      }
      determinant = -determinant;
    }
    const double pivot = matrix(col, col);
    determinant *= pivot;
    for (std::size_t row = col + 1U; row < matrix.rows(); ++row) {
      const double factor = matrix(row, col) / pivot;
      for (std::size_t update_col = col + 1U;
           update_col < matrix.cols(); ++update_col) {
        matrix(row, update_col) -= factor * matrix(col, update_col);
      }
    }
  }
  return determinant;
}

double minimum_symmetric_eigenvalue(DenseMatrix matrix) {
  matrix = matrix.symmetrized();
  for (unsigned int iteration = 0U; iteration < 128U; ++iteration) {
    std::size_t pivot_row = 0U;
    std::size_t pivot_col = 0U;
    double largest = 0.0;
    for (std::size_t row = 0U; row < matrix.rows(); ++row) {
      for (std::size_t col = row + 1U; col < matrix.cols(); ++col) {
        const double magnitude = std::abs(matrix(row, col));
        if (magnitude > largest) {
          largest = magnitude;
          pivot_row = row;
          pivot_col = col;
        }
      }
    }
    if (largest <= 1.0e-18) {
      break;
    }

    const double app = matrix(pivot_row, pivot_row);
    const double aqq = matrix(pivot_col, pivot_col);
    const double apq = matrix(pivot_row, pivot_col);
    const double tau = (aqq - app) / (2.0 * apq);
    const double tangent = std::copysign(
        1.0 / (std::abs(tau) + std::sqrt(1.0 + tau * tau)), tau);
    const double cosine = 1.0 / std::sqrt(1.0 + tangent * tangent);
    const double sine = tangent * cosine;
    for (std::size_t index = 0U; index < matrix.rows(); ++index) {
      if (index == pivot_row || index == pivot_col) {
        continue;
      }
      const double aip = matrix(index, pivot_row);
      const double aiq = matrix(index, pivot_col);
      matrix(index, pivot_row) = cosine * aip - sine * aiq;
      matrix(pivot_row, index) = matrix(index, pivot_row);
      matrix(index, pivot_col) = sine * aip + cosine * aiq;
      matrix(pivot_col, index) = matrix(index, pivot_col);
    }
    matrix(pivot_row, pivot_row) =
        cosine * cosine * app - 2.0 * sine * cosine * apq +
        sine * sine * aqq;
    matrix(pivot_col, pivot_col) =
        sine * sine * app + 2.0 * sine * cosine * apq +
        cosine * cosine * aqq;
    matrix(pivot_row, pivot_col) = 0.0;
    matrix(pivot_col, pivot_row) = 0.0;
  }

  double minimum = matrix(0U, 0U);
  for (std::size_t index = 1U; index < matrix.rows(); ++index) {
    minimum = std::min(minimum, matrix(index, index));
  }
  return minimum;
}

}  // namespace

TEST_CASE(range_ekf_translates_all_initial_positions_to_reference_origin) {
  RangeEkf filter(config(), triangle_nodes());

  const NodeEstimate reference = filter.estimate(10U);
  const NodeEstimate b = filter.estimate(42U);
  const NodeEstimate c = filter.estimate(7U);

  EXPECT_TRUE(reference.valid);
  EXPECT_TRUE(near(reference.x, 0.0));
  EXPECT_TRUE(near(reference.y, 0.0));
  EXPECT_TRUE(near(b.x, 3.0));
  EXPECT_TRUE(near(b.y, 4.0));
  EXPECT_TRUE(near(c.x, -5.0));
  EXPECT_TRUE(near(c.y, 12.0));
  EXPECT_FALSE(filter.estimate(999U).valid);
}

TEST_CASE(range_ekf_rejects_nonpositive_process_standard_deviation) {
  FilterConfig invalid_config = config();
  invalid_config.process_accel_std_mps2 = 0.0;
  bool rejected = false;
  try {
    static_cast<void>(RangeEkf(invalid_config, triangle_nodes()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_ekf_applies_covariance_floor_at_construction) {
  FilterConfig floored = config();
  floored.min_covariance_diagonal = 0.25;
  auto nodes = triangle_nodes();
  nodes[1].position_std_m = 0.01;
  nodes[1].velocity_std_mps = 0.01;
  RangeEkf filter(floored, nodes);

  const auto& covariance = filter.covariance();
  for (std::size_t index = 0U; index < covariance.rows(); ++index) {
    EXPECT_TRUE(covariance(index, index) >= 0.25);
  }
}

TEST_CASE(range_ekf_rejects_initialization_variance_overflow) {
  auto nodes = triangle_nodes();
  nodes[1].position_std_m = std::numeric_limits<double>::max();
  bool rejected = false;
  try {
    static_cast<void>(RangeEkf(config(), nodes));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_ekf_rejects_relative_position_overflow_at_construction) {
  auto nodes = triangle_nodes();
  nodes[0].x = -std::numeric_limits<double>::max();
  nodes[1].x = std::numeric_limits<double>::max();
  bool rejected = false;
  try {
    static_cast<void>(RangeEkf(config(), nodes));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_ekf_rejects_process_variance_overflow) {
  FilterConfig invalid_config = config();
  invalid_config.process_accel_std_mps2 =
      std::numeric_limits<double>::max();
  bool rejected = false;
  try {
    static_cast<void>(RangeEkf(invalid_config, triangle_nodes()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_ekf_rejects_process_variance_underflow) {
  FilterConfig invalid_config = config();
  invalid_config.process_accel_std_mps2 = 1.0e-200;
  bool rejected = false;
  try {
    static_cast<void>(RangeEkf(invalid_config, triangle_nodes()));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(range_ekf_tiny_prediction_step_still_advances_full_duration) {
  FilterConfig tiny_step = config();
  tiny_step.max_prediction_step_s = 1.0e-20;
  RangeEkf filter(tiny_step, triangle_nodes());
  filter.predict_to(1'000'000'000ULL);
  filter.predict_to(2'000'000'000ULL);

  EXPECT_TRUE(near(filter.estimate(42U).x, 4.0, 1.0e-8));
  EXPECT_EQ(filter.estimate(42U).timestamp_ns, 2'000'000'000ULL);
}

TEST_CASE(one_prediction_matches_four_explicit_quarter_second_predictions) {
  FilterConfig quarter_step = config();
  quarter_step.max_prediction_step_s = 0.25;
  RangeEkf one_call(quarter_step, triangle_nodes());
  RangeEkf four_calls(quarter_step, triangle_nodes());
  constexpr std::uint64_t base = 1U;
  one_call.predict_to(base);
  four_calls.predict_to(base);

  one_call.predict_to(base + 1'000'000'000ULL);
  for (std::uint64_t step = 1U; step <= 4U; ++step) {
    four_calls.predict_to(base + step * 250'000'000ULL);
  }

  EXPECT_TRUE(same_state_and_covariance(one_call, four_calls, 1.0e-10));
}

TEST_CASE(closed_form_prediction_matches_more_than_ten_thousand_substeps) {
  FilterConfig tiny_step = config();
  tiny_step.max_prediction_step_s = 0.0001;
  RangeEkf one_call(tiny_step, triangle_nodes());
  RangeEkf explicit_calls(tiny_step, triangle_nodes());
  constexpr std::uint64_t base = 1U;
  constexpr std::uint64_t step_ns = 100'000ULL;
  constexpr std::uint64_t step_count = 10'001ULL;
  one_call.predict_to(base);
  explicit_calls.predict_to(base);

  one_call.predict_to(base + step_count * step_ns);
  for (std::uint64_t step = 1U; step <= step_count; ++step) {
    explicit_calls.predict_to(base + step * step_ns);
  }

  EXPECT_TRUE(same_state_and_covariance(one_call, explicit_calls, 1.0e-8));
}

TEST_CASE(extreme_segment_count_uses_finite_inverse_count_process_noise) {
  FilterConfig extreme_steps = config();
  extreme_steps.max_prediction_step_s = 1.0e-200;
  RangeEkf filter(extreme_steps, triangle_nodes());
  filter.predict_to(1U);
  bool predicted = true;
  try {
    filter.predict_to(1'000'000'001ULL);
  } catch (const std::exception&) {
    predicted = false;
  }

  EXPECT_TRUE(predicted);
  EXPECT_EQ(filter.estimate(42U).timestamp_ns, 1'000'000'001ULL);
  const auto& covariance = filter.covariance();
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      EXPECT_TRUE(std::isfinite(covariance(row, col)));
    }
  }
}

TEST_CASE(extreme_finite_process_noise_avoids_intermediate_product_overflow) {
  FilterConfig extreme_noise = config();
  extreme_noise.process_accel_std_mps2 = 1.0e150;
  extreme_noise.max_prediction_step_s = 1.0e-91;
  RangeEkf filter(extreme_noise, triangle_nodes());
  filter.predict_to(0U);
  bool predicted = true;
  try {
    filter.predict_to(1'000'000'000'000'000'000ULL);
  } catch (const std::exception&) {
    predicted = false;
  }

  EXPECT_TRUE(predicted);
  EXPECT_EQ(filter.estimate(42U).timestamp_ns,
            1'000'000'000'000'000'000ULL);
  const auto& covariance = filter.covariance();
  EXPECT_TRUE(std::isfinite(covariance(0U, 0U)));
  EXPECT_TRUE(covariance(0U, 0U) > 1.0e234);
  EXPECT_TRUE(covariance(0U, 0U) < 1.0e237);
  EXPECT_TRUE(covariance(0U, 2U) > 1.0e225);
  EXPECT_TRUE(covariance(0U, 2U) < 1.0e228);
  EXPECT_TRUE(covariance(2U, 2U) > 1.0e217);
  EXPECT_TRUE(covariance(2U, 2U) < 1.0e219);
}

TEST_CASE(numerical_prediction_failure_rolls_back_state_and_time) {
  auto nodes = triangle_nodes();
  nodes[1].x = 1.0e308;
  nodes[1].vx = 1.0e308;
  RangeEkf filter(config(), nodes);
  filter.predict_to(1'000'000'000ULL);
  const NodeEstimate before = filter.estimate(42U);

  const auto update_result =
      filter.update(packet(10U, 42U, 5.0, 0.1, 2'000'000'000ULL));
  const NodeEstimate after = filter.estimate(42U);
  EXPECT_EQ(update_result.disposition, UpdateDisposition::NumericalFailure);
  EXPECT_TRUE(after.valid);
  EXPECT_TRUE(std::isfinite(after.x));
  EXPECT_TRUE(near(after.x, before.x));
  EXPECT_EQ(after.timestamp_ns, before.timestamp_ns);
}

TEST_CASE(range_ekf_accepts_arbitrary_initialization_order_and_sparse_ids) {
  auto nodes = triangle_nodes();
  std::reverse(nodes.begin(), nodes.end());
  RangeEkf filter(config(), nodes);

  const auto estimates = filter.estimates();
  EXPECT_EQ(estimates.size(), 3U);
  EXPECT_TRUE(filter.estimate(7U).valid);
  EXPECT_TRUE(filter.estimate(10U).valid);
  EXPECT_TRUE(filter.estimate(42U).valid);
  EXPECT_EQ(filter.covariance().rows(), 8U);
  EXPECT_EQ(filter.covariance().cols(), 8U);
}

TEST_CASE(range_ekf_first_timestamp_only_establishes_timebase_then_predicts_cv) {
  RangeEkf filter(config(), triangle_nodes());

  filter.predict_to(2'000'000'000ULL);
  EXPECT_TRUE(near(filter.estimate(42U).x, 3.0));
  EXPECT_TRUE(near(filter.estimate(42U).y, 4.0));

  filter.predict_to(4'000'000'000ULL);
  const NodeEstimate b = filter.estimate(42U);
  const NodeEstimate c = filter.estimate(7U);
  EXPECT_TRUE(near(b.x, 5.0, 1.0e-8));
  EXPECT_TRUE(near(b.y, 3.0, 1.0e-8));
  EXPECT_TRUE(near(c.x, -5.5, 1.0e-8));
  EXPECT_TRUE(near(c.y, 13.5, 1.0e-8));
  EXPECT_EQ(b.timestamp_ns, 4'000'000'000ULL);
}

TEST_CASE(timestamp_zero_establishes_timebase_before_one_second_prediction) {
  RangeEkf filter(config(), triangle_nodes());
  filter.predict_to(0U);
  filter.predict_to(1'000'000'000ULL);

  const NodeEstimate b = filter.estimate(42U);
  EXPECT_TRUE(near(b.x, 4.0, 1.0e-8));
  EXPECT_TRUE(near(b.y, 3.5, 1.0e-8));
  EXPECT_EQ(b.timestamp_ns, 1'000'000'000ULL);
}

TEST_CASE(equal_absolute_velocities_produce_zero_relative_velocity) {
  auto nodes = triangle_nodes();
  nodes[0].vx = 10.0;
  nodes[0].vy = -2.0;
  nodes[1].vx = 10.0;
  nodes[1].vy = -2.0;
  RangeEkf filter(config(), nodes);
  const double initial_range =
      distance(filter.estimate(10U), filter.estimate(42U));
  filter.predict_to(0U);
  filter.predict_to(1'000'000'000ULL);

  const NodeEstimate reference = filter.estimate(10U);
  const NodeEstimate b = filter.estimate(42U);
  EXPECT_TRUE(near(reference.vx, 0.0));
  EXPECT_TRUE(near(reference.vy, 0.0));
  EXPECT_TRUE(near(b.vx, 0.0));
  EXPECT_TRUE(near(b.vy, 0.0));
  EXPECT_TRUE(near(distance(reference, b), initial_range, 1.0e-8));
}

TEST_CASE(reference_ranges_correct_only_the_observed_nonreference_node) {
  RangeEkf filter(config(), triangle_nodes());
  const NodeEstimate c_before = filter.estimate(7U);

  const auto ab = filter.update(packet(10U, 42U, 4.5, 0.1));
  const NodeEstimate b_after = filter.estimate(42U);
  const NodeEstimate c_after_ab = filter.estimate(7U);
  EXPECT_EQ(ab.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(distance(filter.estimate(10U), b_after) < 5.0);
  EXPECT_TRUE(near(c_after_ab.x, c_before.x));
  EXPECT_TRUE(near(c_after_ab.y, c_before.y));

  const auto ac = filter.update(packet(10U, 7U, 12.5, 0.1));
  const NodeEstimate b_after_ac = filter.estimate(42U);
  EXPECT_EQ(ac.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(distance(filter.estimate(10U), filter.estimate(7U)) < 13.0);
  EXPECT_TRUE(near(b_after_ac.x, b_after.x));
  EXPECT_TRUE(near(b_after_ac.y, b_after.y));
}

TEST_CASE(peer_range_moves_both_nodes_and_builds_cross_covariance) {
  RangeEkf filter(config(), triangle_nodes());
  const NodeEstimate b_before = filter.estimate(42U);
  const NodeEstimate c_before = filter.estimate(7U);

  const auto result = filter.update(packet(42U, 7U, 13.0, 0.2));
  const NodeEstimate b_after = filter.estimate(42U);
  const NodeEstimate c_after = filter.estimate(7U);
  EXPECT_EQ(result.disposition, UpdateDisposition::Accepted);
  EXPECT_FALSE(near(b_after.x, b_before.x));
  EXPECT_FALSE(near(c_after.x, c_before.x));

  bool has_cross_covariance = false;
  const auto& covariance = filter.covariance();
  for (std::size_t row = 0U; row < 4U; ++row) {
    for (std::size_t col = 4U; col < 8U; ++col) {
      has_cross_covariance =
          has_cross_covariance || std::abs(covariance(row, col)) > 1.0e-12;
    }
  }
  EXPECT_TRUE(has_cross_covariance);
}

TEST_CASE(cross_covariance_propagates_a_later_correction_to_correlated_node) {
  RangeEkf filter(config(), triangle_nodes());
  EXPECT_EQ(filter.update(packet(42U, 7U, 13.0, 0.2)).disposition,
            UpdateDisposition::Accepted);
  const NodeEstimate c_before = filter.estimate(7U);

  EXPECT_EQ(filter.update(packet(10U, 42U, 4.7, 0.2)).disposition,
            UpdateDisposition::Accepted);
  const NodeEstimate c_after = filter.estimate(7U);
  EXPECT_TRUE(!near(c_after.x, c_before.x) || !near(c_after.y, c_before.y));
}

TEST_CASE(accepted_small_residual_reduces_range_innovation) {
  RangeEkf filter(config(), triangle_nodes());
  const double measured_range = 4.8;
  const double before = std::abs(measured_range -
                                 distance(filter.estimate(10U),
                                          filter.estimate(42U)));

  const auto result =
      filter.update(packet(10U, 42U, measured_range, 0.2), 2.0);
  const double after = std::abs(measured_range -
                                distance(filter.estimate(10U),
                                         filter.estimate(42U)));

  EXPECT_EQ(result.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(near(result.innovation_m, -0.2, 1.0e-9));
  EXPECT_TRUE(result.innovation_variance > 0.0);
  EXPECT_TRUE(result.nis >= 0.0);
  EXPECT_TRUE(near(result.covariance_scale, 2.0));
  EXPECT_TRUE(after < before);
}

TEST_CASE(subnormal_measurement_variance_avoids_intermediate_underflow) {
  FilterConfig subnormal_config{10U, 0.2, 100.0, 0.25, 1.0e-300};
  const std::vector<NodeInitialization> nodes{
      {10U, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
      {42U, 3.0, 4.0, 0.0, 0.0, 1.0e-160, 1.0},
  };
  RangeEkf filter(subnormal_config, nodes);

  const auto result =
      filter.update(packet(10U, 42U, 5.0, 1.0e-200, 0U), 1.0e100);
  EXPECT_EQ(result.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(std::isfinite(result.innovation_variance));
  EXPECT_TRUE(result.innovation_variance > 1.5e-300);
  EXPECT_TRUE(result.innovation_variance < 2.5e-300);
}

TEST_CASE(range_ekf_reports_invalid_packet_categories_without_correction) {
  RangeEkf filter(config(), triangle_nodes());

  auto invalid = packet(10U, 42U, 5.0, 0.1);
  invalid.valid = false;
  EXPECT_EQ(filter.update(invalid).disposition,
            UpdateDisposition::InvalidPacket);
  EXPECT_EQ(filter.update(packet(10U, 99U, 5.0, 0.1)).disposition,
            UpdateDisposition::UnknownNode);
  EXPECT_EQ(filter.update(packet(42U, 42U, 5.0, 0.1)).disposition,
            UpdateDisposition::SelfRange);
  EXPECT_EQ(filter.update(packet(10U, 42U, 0.0, 0.1)).disposition,
            UpdateDisposition::NonPositiveRange);
  EXPECT_EQ(filter.update(packet(10U, 42U, -1.0, 0.1)).disposition,
            UpdateDisposition::NonPositiveRange);
  EXPECT_EQ(filter.update(packet(10U, 42U, 5.0, 0.0)).disposition,
            UpdateDisposition::InvalidPacket);

  auto nonfinite = packet(10U, 42U, 5.0, 0.1);
  nonfinite.range_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(filter.update(nonfinite).disposition,
            UpdateDisposition::InvalidPacket);
  EXPECT_EQ(filter.update(packet(10U, 42U, 5.0, 0.1), 0.99).disposition,
            UpdateDisposition::InvalidPacket);
}

TEST_CASE(nis_rejection_keeps_prediction_but_not_measurement_correction) {
  FilterConfig strict = config();
  strict.nis_gate = 0.01;
  RangeEkf rejected(strict, triangle_nodes());
  RangeEkf predicted_only(strict, triangle_nodes());
  rejected.predict_to(1'000'000'000ULL);
  predicted_only.predict_to(1'000'000'000ULL);
  predicted_only.predict_to(2'000'000'000ULL);

  const auto result =
      rejected.update(packet(10U, 42U, 50.0, 0.1, 2'000'000'000ULL));
  const NodeEstimate actual = rejected.estimate(42U);
  const NodeEstimate expected = predicted_only.estimate(42U);
  EXPECT_EQ(result.disposition, UpdateDisposition::NisRejected);
  EXPECT_TRUE(result.nis > strict.nis_gate);
  EXPECT_TRUE(near(actual.x, expected.x));
  EXPECT_TRUE(near(actual.y, expected.y));
  EXPECT_TRUE(near(actual.vx, expected.vx));
  EXPECT_TRUE(near(actual.vy, expected.vy));
  EXPECT_EQ(actual.timestamp_ns, 2'000'000'000ULL);
  const auto& actual_covariance = rejected.covariance();
  const auto& expected_covariance = predicted_only.covariance();
  for (std::size_t row = 0U; row < actual_covariance.rows(); ++row) {
    for (std::size_t col = 0U; col < actual_covariance.cols(); ++col) {
      EXPECT_TRUE(near(actual_covariance(row, col),
                       expected_covariance(row, col)));
    }
  }
}

TEST_CASE(nis_equal_to_gate_is_accepted) {
  FilterConfig preview_config = config();
  preview_config.nis_gate = 1.0e9;
  RangeEkf preview(preview_config, triangle_nodes());
  const auto preview_result =
      preview.update(packet(10U, 42U, 4.5, 0.2));

  FilterConfig exact_config = config();
  exact_config.nis_gate = preview_result.nis;
  RangeEkf exact(exact_config, triangle_nodes());
  EXPECT_EQ(exact.update(packet(10U, 42U, 4.5, 0.2)).disposition,
            UpdateDisposition::Accepted);
}

TEST_CASE(overflowing_innovation_square_is_reported_as_nis_rejection) {
  RangeEkf filter(config(), triangle_nodes());
  const auto result = filter.update(packet(
      10U, 42U, std::numeric_limits<double>::max(), 0.1));

  EXPECT_EQ(result.disposition, UpdateDisposition::NisRejected);
  EXPECT_EQ(result.nis, std::numeric_limits<double>::max());
}

TEST_CASE(coincident_nodes_report_numerical_failure_for_range_derivative) {
  auto nodes = triangle_nodes();
  nodes[1].x = nodes[0].x;
  nodes[1].y = nodes[0].y;
  RangeEkf filter(config(), nodes);

  EXPECT_EQ(filter.update(packet(10U, 42U, 1.0, 0.1)).disposition,
            UpdateDisposition::NumericalFailure);
}

TEST_CASE(out_of_order_range_is_rejected_without_rewinding_filter) {
  RangeEkf filter(config(), triangle_nodes());
  filter.predict_to(10U);
  filter.predict_to(20U);
  const NodeEstimate before = filter.estimate(42U);

  const auto result = filter.update(packet(10U, 42U, 4.9, 0.1, 19U));
  const NodeEstimate after = filter.estimate(42U);
  EXPECT_EQ(result.disposition, UpdateDisposition::OutOfOrder);
  EXPECT_EQ(after.timestamp_ns, 20U);
  EXPECT_TRUE(near(after.x, before.x));
  EXPECT_TRUE(near(after.y, before.y));
}

TEST_CASE(accepted_updates_leave_covariance_symmetric_with_positive_diagonal) {
  RangeEkf filter(config(), triangle_nodes());
  static_cast<void>(filter.update(packet(10U, 42U, 4.9, 0.2)));
  static_cast<void>(filter.update(packet(42U, 7U, 13.1, 0.2)));

  const auto& covariance = filter.covariance();
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    EXPECT_TRUE(std::isfinite(covariance(row, row)));
    EXPECT_TRUE(covariance(row, row) >= config().min_covariance_diagonal);
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      EXPECT_TRUE(near(covariance(row, col), covariance(col, row), 1.0e-10));
    }
  }
}

TEST_CASE(near_singular_range_update_keeps_full_covariance_positive_definite) {
  FilterConfig near_singular_config{10U, 0.2, 100.0, 0.25, 1.0e-300};
  const std::vector<NodeInitialization> nodes{
      {10U, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
      {42U, std::sqrt(0.9999), 0.01, 0.0, 0.0, 1.0, 1.0},
  };
  RangeEkf filter(near_singular_config, nodes);
  const auto result =
      filter.update(packet(10U, 42U, 1.0, 1.0e-150, 0U));

  EXPECT_EQ(result.disposition, UpdateDisposition::Accepted);
  const auto& covariance = filter.covariance();
  const double position_determinant =
      covariance(0U, 0U) * covariance(1U, 1U) -
      covariance(0U, 1U) * covariance(1U, 0U);
  EXPECT_TRUE(position_determinant >= 0.0);
  EXPECT_TRUE(has_strict_cholesky_factor(covariance));
}

TEST_CASE(prediction_stabilizes_near_singular_full_covariance) {
  FilterConfig near_singular_config{10U, 1.0e-161, 100.0, 1.0,
                                    1.0e-300};
  const std::vector<NodeInitialization> nodes{
      {10U, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
      {42U, std::sqrt(1.0 - 1.0e-6), 1.0e-3, 0.0, 0.0, 1.0, 1.0},
  };
  RangeEkf filter(near_singular_config, nodes);
  EXPECT_EQ(filter.update(packet(10U, 42U, 1.0, 1.0e-11, 0U)).disposition,
            UpdateDisposition::Accepted);
  filter.predict_to(800'004'000ULL);
  const auto& covariance = filter.covariance();
  EXPECT_TRUE(matrix_determinant(covariance) >= 0.0);
  const double velocity_determinant =
      covariance(2U, 2U) * covariance(3U, 3U) -
      covariance(2U, 3U) * covariance(3U, 2U);
  const double inverse_v00 = covariance(3U, 3U) / velocity_determinant;
  const double inverse_v01 = -covariance(2U, 3U) / velocity_determinant;
  const double inverse_v10 = -covariance(3U, 2U) / velocity_determinant;
  const double inverse_v11 = covariance(2U, 2U) / velocity_determinant;
  const double schur_xx =
      covariance(0U, 0U) -
      covariance(0U, 2U) *
          (inverse_v00 * covariance(2U, 0U) +
           inverse_v01 * covariance(3U, 0U)) -
      covariance(0U, 3U) *
          (inverse_v10 * covariance(2U, 0U) +
           inverse_v11 * covariance(3U, 0U));
  const double schur_xy =
      covariance(0U, 1U) -
      covariance(0U, 2U) *
          (inverse_v00 * covariance(2U, 1U) +
           inverse_v01 * covariance(3U, 1U)) -
      covariance(0U, 3U) *
          (inverse_v10 * covariance(2U, 1U) +
           inverse_v11 * covariance(3U, 1U));
  const double schur_yy =
      covariance(1U, 1U) -
      covariance(1U, 2U) *
          (inverse_v00 * covariance(2U, 1U) +
           inverse_v01 * covariance(3U, 1U)) -
      covariance(1U, 3U) *
          (inverse_v10 * covariance(2U, 1U) +
           inverse_v11 * covariance(3U, 1U));
  EXPECT_TRUE(schur_xx * schur_yy - schur_xy * schur_xy >= 0.0);
  EXPECT_TRUE(minimum_symmetric_eigenvalue(covariance) >= 0.0);
  EXPECT_TRUE(has_strict_cholesky_factor(covariance));

  RangeEkf rounding_edge(near_singular_config, nodes);
  EXPECT_EQ(
      rounding_edge.update(packet(10U, 42U, 1.0, 1.0e-11, 0U)).disposition,
      UpdateDisposition::Accepted);
  rounding_edge.predict_to(799'900'003ULL);
  EXPECT_TRUE(minimum_symmetric_eigenvalue(rounding_edge.covariance()) >= 0.0);
  EXPECT_TRUE(has_strict_cholesky_factor(rounding_edge.covariance()));
}

TEST_CASE(three_consistent_ranges_converge_to_stable_triangle_geometry) {
  FilterConfig loose = config();
  loose.nis_gate = 1.0e9;
  const std::vector<NodeInitialization> nodes{
      {10U, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01},
      {42U, 3.3, 0.25, 0.0, 0.0, 1.0, 0.2},
      {7U, -0.3, 4.2, 0.0, 0.0, 1.0, 0.2},
  };
  RangeEkf filter(loose, nodes);

  for (std::uint64_t timestamp = 1U; timestamp <= 10U; ++timestamp) {
    EXPECT_EQ(filter.update(packet(10U, 42U, 3.0, 0.05, timestamp)).disposition,
              UpdateDisposition::Accepted);
    EXPECT_EQ(filter.update(packet(10U, 7U, 4.0, 0.05, timestamp)).disposition,
              UpdateDisposition::Accepted);
    EXPECT_EQ(filter.update(packet(42U, 7U, 5.0, 0.05, timestamp)).disposition,
              UpdateDisposition::Accepted);
  }

  const NodeEstimate a = filter.estimate(10U);
  const NodeEstimate b = filter.estimate(42U);
  const NodeEstimate c = filter.estimate(7U);
  EXPECT_TRUE(near(distance(a, b), 3.0, 0.05));
  EXPECT_TRUE(near(distance(a, c), 4.0, 0.05));
  EXPECT_TRUE(near(distance(b, c), 5.0, 0.05));
}

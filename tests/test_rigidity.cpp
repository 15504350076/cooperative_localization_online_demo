// 模块职责：验证动态图去重、主参考可达性、二维刚度秩和共线/断边退化判定。
#include "test_support.hpp"

#include "core/dense_matrix.hpp"
#include "core/rigidity.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {

using zju::coop::DenseMatrix;
using zju::coop::Edge;
using zju::coop::Point2;

bool throws_invalid_argument(void (*action)()) {
  try {
    action();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

bool throws_length_error(void (*action)()) {
  try {
    action();
  } catch (const std::length_error&) {
    return true;
  }
  return false;
}

}  // namespace

TEST_CASE(dense_matrix_supports_required_linear_algebra_operations) {
  DenseMatrix left(2U, 2U);
  left(0U, 0U) = 1.0;
  left(0U, 1U) = 2.0;
  left(1U, 0U) = 3.0;
  left(1U, 1U) = 4.0;

  EXPECT_EQ(left.rows(), 2U);
  EXPECT_EQ(left.cols(), 2U);
  EXPECT_EQ(left.transpose()(0U, 1U), 3.0);

  const DenseMatrix product = left * DenseMatrix::identity(2U);
  EXPECT_EQ(product(1U, 0U), 3.0);
  EXPECT_EQ(product(1U, 1U), 4.0);

  const std::vector<double> vector_product =
      left * std::vector<double>{2.0, 1.0};
  EXPECT_EQ(vector_product.size(), 2U);
  EXPECT_EQ(vector_product[0U], 4.0);
  EXPECT_EQ(vector_product[1U], 10.0);

  const DenseMatrix symmetric = left.symmetrized();
  EXPECT_EQ(symmetric(0U, 1U), 2.5);
  EXPECT_EQ(symmetric(1U, 0U), 2.5);
}

TEST_CASE(dense_matrix_symmetrization_keeps_equal_maximum_entries_finite) {
  const double maximum = std::numeric_limits<double>::max();
  DenseMatrix matrix(2U, 2U);
  matrix(0U, 1U) = maximum;
  matrix(1U, 0U) = maximum;

  const DenseMatrix symmetric = matrix.symmetrized();
  EXPECT_TRUE(std::isfinite(symmetric(0U, 1U)));
  EXPECT_EQ(symmetric(0U, 1U), maximum);
  EXPECT_EQ(symmetric(1U, 0U), maximum);
}

TEST_CASE(numeric_rank_uses_partial_pivoting_and_tolerance) {
  DenseMatrix pivoting_required(2U, 2U);
  pivoting_required(0U, 1U) = 1.0;
  pivoting_required(1U, 0U) = 1.0;
  EXPECT_EQ(zju::coop::numeric_rank(pivoting_required, 1.0e-12), 2U);

  DenseMatrix nearly_dependent(2U, 2U);
  nearly_dependent(0U, 0U) = 1.0;
  nearly_dependent(0U, 1U) = 1.0;
  nearly_dependent(1U, 0U) = 1.0;
  nearly_dependent(1U, 1U) = 1.0 + 1.0e-10;
  EXPECT_EQ(zju::coop::numeric_rank(nearly_dependent, 1.0e-8), 1U);
}

TEST_CASE(numeric_rank_rejects_nan_entries) {
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    DenseMatrix matrix(1U, 1U);
    matrix(0U, 0U) = std::numeric_limits<double>::quiet_NaN();
    static_cast<void>(zju::coop::numeric_rank(matrix, 1.0e-9));
  }));
}

TEST_CASE(numeric_rank_rejects_infinite_entries) {
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    DenseMatrix matrix(1U, 1U);
    matrix(0U, 0U) = std::numeric_limits<double>::infinity();
    static_cast<void>(zju::coop::numeric_rank(matrix, 1.0e-9));
  }));
}

TEST_CASE(numeric_rank_rejects_non_finite_elimination_results) {
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    const double maximum = std::numeric_limits<double>::max();
    DenseMatrix overflowing(2U, 2U);
    overflowing(0U, 0U) = maximum;
    overflowing(0U, 1U) = maximum;
    overflowing(1U, 0U) = maximum;
    overflowing(1U, 1U) = -maximum;
    static_cast<void>(zju::coop::numeric_rank(overflowing, 1.0e-9));
  }));
}

TEST_CASE(dense_matrix_rejects_element_count_overflow) {
  EXPECT_TRUE(throws_length_error(+[]() {
    static_cast<void>(DenseMatrix(
        std::numeric_limits<std::size_t>::max() / 2U + 1U, 2U));
  }));
}

TEST_CASE(non_collinear_triangle_is_observable) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{10U, 30U, 20U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      std::vector<Edge>{{10U, 30U}, {30U, 20U}, {20U, 10U}}, 10U,
      1.0e-9);

  EXPECT_EQ(result.rank, 3U);
  EXPECT_EQ(result.target_rank, 3U);
  EXPECT_EQ(result.reachable_count, 3U);
  EXPECT_TRUE(result.connected);
  EXPECT_TRUE(result.observable);
}

TEST_CASE(collinear_triangle_is_not_observable) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{1U, 2U, 3U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
      std::vector<Edge>{{1U, 2U}, {2U, 3U}, {3U, 1U}}, 1U, 1.0e-9);

  EXPECT_EQ(result.rank, 2U);
  EXPECT_EQ(result.target_rank, 3U);
  EXPECT_TRUE(result.connected);
  EXPECT_FALSE(result.observable);
}

TEST_CASE(triangle_missing_one_edge_is_not_observable) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{1U, 2U, 3U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      std::vector<Edge>{{1U, 2U}, {1U, 3U}}, 1U, 1.0e-9);

  EXPECT_EQ(result.rank, 2U);
  EXPECT_EQ(result.reachable_count, 3U);
  EXPECT_TRUE(result.connected);
  EXPECT_FALSE(result.observable);
}

TEST_CASE(disconnected_nodes_are_counted_from_reference) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{10U, 20U, 30U, 40U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {5.0, 5.0}},
      std::vector<Edge>{{10U, 20U}, {20U, 30U}, {30U, 10U}}, 10U,
      1.0e-9);

  EXPECT_EQ(result.reachable_count, 3U);
  EXPECT_FALSE(result.connected);
  EXPECT_FALSE(result.observable);
}

TEST_CASE(isolated_reference_reaches_only_itself) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{10U, 20U, 30U, 40U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {5.0, 5.0}},
      std::vector<Edge>{{10U, 20U}, {20U, 30U}, {30U, 10U}}, 40U,
      1.0e-9);

  EXPECT_EQ(result.reachable_count, 1U);
  EXPECT_FALSE(result.connected);
  EXPECT_FALSE(result.observable);
}

TEST_CASE(rigidity_analysis_rejects_non_finite_coordinate_differences) {
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    const double maximum = std::numeric_limits<double>::max();
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U},
        std::vector<Point2>{{maximum, 0.0}, {-maximum, 0.0}},
        std::vector<Edge>{{1U, 2U}}, 1U, 1.0e-9));
  }));
}

TEST_CASE(unknown_self_loop_and_duplicate_edges_do_not_change_analysis) {
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{7U, 8U, 9U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      std::vector<Edge>{{7U, 8U}, {8U, 7U}, {8U, 9U}, {9U, 7U},
                        {7U, 7U}, {7U, 999U}},
      7U, 1.0e-9);

  EXPECT_EQ(result.rank, 3U);
  EXPECT_EQ(result.reachable_count, 3U);
  EXPECT_EQ(result.effective_edge_count, 3U);
  EXPECT_TRUE(result.observable);
}

TEST_CASE(invalid_rigidity_inputs_throw_invalid_argument) {
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U}, std::vector<Point2>{{0.0, 0.0}},
        std::vector<Edge>{}, 1U, 1.0e-9));
  }));
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 1U},
        std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}},
        std::vector<Edge>{}, 1U, 1.0e-9));
  }));
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U},
        std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}},
        std::vector<Edge>{}, 3U, 1.0e-9));
  }));
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U},
        std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}},
        std::vector<Edge>{}, 1U, 0.0));
  }));
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U},
        std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}},
        std::vector<Edge>{}, 1U, -1.0e-9));
  }));
}

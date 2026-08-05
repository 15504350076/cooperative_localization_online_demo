// 模块职责：验证动态图去重、主参考可达性、二维刚度秩和共线/断边退化判定。
// C++初学者阅读提示：节点是图中的点，测距是无向边；测试通过三角形、共线点和断开的图，
// 检查算法能否区分“从主车可达”与“几何约束足以唯一确定形状”这两个概念。
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

// action：无捕获且仅在本次调用执行的失败场景入口；返回值表示是否抛出invalid_argument。
bool throws_invalid_argument(void (*action)()) {
  try {
    action();
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

// action：无捕获且仅在本次调用执行的容量失败场景入口；返回值表示是否抛出length_error。
bool throws_length_error(void (*action)()) {
  try {
    action();
  } catch (const std::length_error&) {
    return true;
  }
  return false;
}

}  // namespace

// 第一组保护滤波和刚度共用的最小线性代数：维度、有限值、对称化和数值秩。
TEST_CASE(dense_matrix_supports_required_linear_algebra_operations) {
  // left：填入1至4的2×2输入矩阵；product/vector_product/symmetric：分别验证单位阵乘、矩阵向量乘和对称化输出。
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
  // maximum：double最大有限值；matrix：两侧非对角元均取maximum的输入；symmetric：期望不因直接相加而溢出。
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
  // pivoting_required：首主元为零但满秩的矩阵；nearly_dependent：按1e-8容差应判为秩1的近相关矩阵。
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
  // lambda内matrix：唯一元素为NaN的秩计算输入，期望触发参数异常。
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    DenseMatrix matrix(1U, 1U);
    matrix(0U, 0U) = std::numeric_limits<double>::quiet_NaN();
    static_cast<void>(zju::coop::numeric_rank(matrix, 1.0e-9));
  }));
}

TEST_CASE(numeric_rank_rejects_infinite_entries) {
  // lambda内matrix：唯一元素为正无穷的秩计算输入，期望触发参数异常。
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    DenseMatrix matrix(1U, 1U);
    matrix(0U, 0U) = std::numeric_limits<double>::infinity();
    static_cast<void>(zju::coop::numeric_rank(matrix, 1.0e-9));
  }));
}

TEST_CASE(numeric_rank_rejects_non_finite_elimination_results) {
  // lambda内maximum/overflowing：消元会由最大有限值产生溢出的2×2输入。
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

// 第二组区分“图连通”和“几何可观”：非共线三角形达到2N-3秩，共线或缺边不达到。
TEST_CASE(non_collinear_triangle_is_observable) {
  // result：非共线三节点三边图的刚度分析，期望达到目标秩3且从节点10全可达。
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
  // result：共线但连通的三角图分析，期望秩2、目标秩3并判为不可观。
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
  // result：缺少一条约束边的非共线三节点分析，期望虽连通但秩不足。
  const auto result = zju::coop::analyze_rigidity(
      std::vector<std::uint32_t>{1U, 2U, 3U},
      std::vector<Point2>{{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}},
      std::vector<Edge>{{1U, 2U}, {1U, 3U}}, 1U, 1.0e-9);

  EXPECT_EQ(result.rank, 2U);
  EXPECT_EQ(result.reachable_count, 3U);
  EXPECT_TRUE(result.connected);
  EXPECT_FALSE(result.observable);
}

// 第三组验证可达性严格从主参考出发，未知边、自环和重复方向不会虚增约束。
TEST_CASE(disconnected_nodes_are_counted_from_reference) {
  // result：含孤立节点40且主参考为10的图分析，期望可达计数仅为3。
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
  // result：把孤立节点40设为参考后的图分析，期望可达计数仅包含参考自身。
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
  // lambda内maximum：构造正负最大坐标，其差值溢出，期望分析拒绝非有限几何量。
  EXPECT_TRUE(throws_invalid_argument(+[]() {
    const double maximum = std::numeric_limits<double>::max();
    static_cast<void>(zju::coop::analyze_rigidity(
        std::vector<std::uint32_t>{1U, 2U},
        std::vector<Point2>{{maximum, 0.0}, {-maximum, 0.0}},
        std::vector<Edge>{{1U, 2U}}, 1U, 1.0e-9));
  }));
}

TEST_CASE(unknown_self_loop_and_duplicate_edges_do_not_change_analysis) {
  // result：混入反向重复边、自环和未知端点后的分析，期望仅计三条有效边且仍可观。
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

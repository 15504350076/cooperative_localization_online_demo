// 模块职责：提供小规模滤波协方差和刚度矩阵所需的最小稠密矩阵运算。
// 模块边界：不追求通用线性代数功能，所有维度、索引和有限值错误均显式失败，
// 从而使算法库在Windows与RK3588上保持一致且不引入第三方矩阵库ABI。
#pragma once

#include <cstddef>
#include <vector>

namespace zju::coop {

/**
 * 行主序动态矩阵；构造后维度固定，元素通过(row,col)访问。
 * 越界、维度不匹配或元素数量溢出都抛出异常，让上层事务式更新回滚，
 * 而不是在滤波状态中留下静默截断或未定义值。
 */
class DenseMatrix {
 public:
  DenseMatrix(std::size_t rows, std::size_t cols, double value = 0.0);

  [[nodiscard]] std::size_t rows() const noexcept;
  [[nodiscard]] std::size_t cols() const noexcept;

  double& operator()(std::size_t row, std::size_t col);
  const double& operator()(std::size_t row, std::size_t col) const;

  [[nodiscard]] static DenseMatrix identity(std::size_t size);
  [[nodiscard]] DenseMatrix transpose() const;
  /** 消除浮点乘加造成的微小非对称；不能修复本身非正定的协方差。 */
  [[nodiscard]] DenseMatrix symmetrized() const;

  [[nodiscard]] DenseMatrix operator*(const DenseMatrix& right) const;
  [[nodiscard]] std::vector<double> operator*(
      const std::vector<double>& right) const;

 private:
  std::size_t rows_;
  std::size_t cols_;
  std::vector<double> values_;
};

/**
 * 使用带阈值的主元消元计算数值秩，供动态拓扑几何可观性判断。
 * tolerance是绝对主元阈值，调用方应先按问题尺度选择，不能把结果理解为
 * 对任意量纲都成立的符号秩。
 */
[[nodiscard]] std::size_t numeric_rank(const DenseMatrix& matrix,
                                       double tolerance);

}  // namespace zju::coop

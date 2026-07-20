// 小规模稠密矩阵工具，仅服务于滤波协方差和几何秩计算，不依赖第三方线性代数库。
#pragma once

#include <cstddef>
#include <vector>

namespace zju::coop {

class DenseMatrix {
 public:
  DenseMatrix(std::size_t rows, std::size_t cols, double value = 0.0);

  [[nodiscard]] std::size_t rows() const noexcept;
  [[nodiscard]] std::size_t cols() const noexcept;

  double& operator()(std::size_t row, std::size_t col);
  const double& operator()(std::size_t row, std::size_t col) const;

  [[nodiscard]] static DenseMatrix identity(std::size_t size);
  [[nodiscard]] DenseMatrix transpose() const;
  [[nodiscard]] DenseMatrix symmetrized() const;

  [[nodiscard]] DenseMatrix operator*(const DenseMatrix& right) const;
  [[nodiscard]] std::vector<double> operator*(
      const std::vector<double>& right) const;

 private:
  std::size_t rows_;
  std::size_t cols_;
  std::vector<double> values_;
};

[[nodiscard]] std::size_t numeric_rank(const DenseMatrix& matrix,
                                       double tolerance);

}  // namespace zju::coop

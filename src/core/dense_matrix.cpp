// 模块实现：小规模稠密矩阵乘法、转置、对称化和带阈值数值秩计算。
// 关键原则：所有维度、容量和有限值错误均显式失败，禁止异常矩阵静默进入滤波；
// 这里采用清晰可审查的实现，便于Windows与ARM64保持一致的错误处理行为。
#include "core/dense_matrix.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

// `rows`与`cols`是待分配矩阵的行列数，用于在分配前验证元素总数可表示。
std::size_t checked_element_count(std::size_t rows, std::size_t cols) {
  // 先用除法判断乘法溢出，不能在rows*cols已经溢出后再比较。
  if (rows != 0U && cols > std::numeric_limits<std::size_t>::max() / rows) {
    throw std::length_error("DenseMatrix element count overflows size_t");
  }

  const std::size_t count = rows * cols;  // 已确认无乘法溢出的行主序元素总数。
  if (count > std::vector<double>{}.max_size()) {
    throw std::length_error("DenseMatrix exceeds the container maximum size");
  }
  return count;
}

}  // namespace

DenseMatrix::DenseMatrix(std::size_t rows, std::size_t cols, double value)
    : rows_(rows), cols_(cols), values_(checked_element_count(rows, cols), value) {}

std::size_t DenseMatrix::rows() const noexcept { return rows_; }

std::size_t DenseMatrix::cols() const noexcept { return cols_; }

double& DenseMatrix::operator()(std::size_t row, std::size_t col) {
  if (row >= rows_ || col >= cols_) {
    throw std::out_of_range("DenseMatrix index is out of range");
  }
  return values_[row * cols_ + col];
}

const double& DenseMatrix::operator()(std::size_t row,
                                      std::size_t col) const {
  if (row >= rows_ || col >= cols_) {
    throw std::out_of_range("DenseMatrix index is out of range");
  }
  return values_[row * cols_ + col];
}

DenseMatrix DenseMatrix::identity(std::size_t size) {
  // `result`是待填充主对角线的size×size单位矩阵。
  DenseMatrix result(size, size);
  // `index`同步遍历单位矩阵的行列主对角下标。
  for (std::size_t index = 0; index < size; ++index) {
    result(index, index) = 1.0;
  }
  return result;
}

DenseMatrix DenseMatrix::transpose() const {
  // `result`的行对应原矩阵列、列对应原矩阵行，维度为cols_×rows_。
  DenseMatrix result(cols_, rows_);
  // `row`遍历原矩阵行；`col`遍历原矩阵列并成为转置结果行。
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t col = 0; col < cols_; ++col) {
      result(col, row) = (*this)(row, col);
    }
  }
  return result;
}

DenseMatrix DenseMatrix::symmetrized() const {
  if (rows_ != cols_) {
    throw std::invalid_argument("only a square matrix can be symmetrized");
  }

  // `result`与方阵同维，元素(i,j)取输入(i,j)与(j,i)的均值。
  DenseMatrix result(rows_, cols_);
  // `row`和`col`分别遍历待对称化方阵的输出行、输出列。
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t col = 0; col < cols_; ++col) {
      result(row, col) =
          0.5 * (*this)(row, col) + 0.5 * (*this)(col, row);
    }
  }
  return result;
}

DenseMatrix DenseMatrix::operator*(const DenseMatrix& right) const {
  if (cols_ != right.rows_) {
    throw std::invalid_argument("matrix dimensions are incompatible");
  }

  // row-inner-col顺序复用同一个左矩阵元素，同时仍保持C_ij=ΣA_ikB_kj。
  // `result`为rows_×right.cols_乘积矩阵，行继承左矩阵、列继承右矩阵。
  DenseMatrix result(rows_, right.cols_);
  // `row`遍历乘积矩阵行；`inner`遍历左右矩阵共享收缩维；
  // `col`遍历右矩阵及乘积矩阵列。
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t inner = 0; inner < cols_; ++inner) {
      // `left_value`缓存当前A(row,inner)，供该结果行的所有列复用。
      const double left_value = (*this)(row, inner);
      for (std::size_t col = 0; col < right.cols_; ++col) {
        result(row, col) += left_value * right(inner, col);
      }
    }
  }
  return result;
}

std::vector<double> DenseMatrix::operator*(
    const std::vector<double>& right) const {
  if (cols_ != right.size()) {
    throw std::invalid_argument("matrix and vector dimensions are incompatible");
  }

  // `result`为矩阵每一行与输入列向量的内积，共rows_项。
  std::vector<double> result(rows_, 0.0);
  // `row`遍历输出向量及矩阵行；`col`遍历该行与输入向量的收缩分量。
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t col = 0; col < cols_; ++col) {
      result[row] += (*this)(row, col) * right[col];
    }
  }
  return result;
}

std::size_t numeric_rank(const DenseMatrix& matrix, double tolerance) {
  if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
    throw std::invalid_argument("rank tolerance must be positive and finite");
  }

  // `row`与`col`遍历输入矩阵全部元素，先拒绝非有限值。
  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col < matrix.cols(); ++col) {
      if (!std::isfinite(matrix(row, col))) {
        throw std::invalid_argument("rank input entries must be finite");
      }
    }
  }

  // 在副本上执行逐列部分选主元消元，调用方提供的刚度矩阵保持不变。
  // pivot_row最终就是找到的独立约束行数，无需继续化成完整单位阶梯形。
  // `work`是消元工作矩阵副本；`pivot_row`是下一主元行，也累计已找到的秩。
  DenseMatrix work = matrix;
  std::size_t pivot_row = 0U;
  // `col`遍历候选主元列，最多为每个独立列增加一次数值秩。
  for (std::size_t col = 0U;
       col < work.cols() && pivot_row < work.rows(); ++col) {
    // `best_row`为当前列绝对值最大主元候选行；`best_magnitude`为其绝对值。
    std::size_t best_row = pivot_row;
    double best_magnitude = std::abs(work(pivot_row, col));
    // `row`遍历当前主元行下方的候选行。
    for (std::size_t row = pivot_row + 1U; row < work.rows(); ++row) {
      // `magnitude`为当前候选元素的绝对值，用于部分选主元。
      const double magnitude = std::abs(work(row, col));
      if (magnitude > best_magnitude) {
        best_magnitude = magnitude;
        best_row = row;
      }
    }

    // 当前列没有高于容差的主元时，该列不增加数值秩。
    if (best_magnitude <= tolerance) {
      continue;
    }

    if (best_row != pivot_row) {
      // `swap_col`从当前主元列遍历到末列，交换两行仍参与消元的后缀。
      for (std::size_t swap_col = col; swap_col < work.cols(); ++swap_col) {
        std::swap(work(pivot_row, swap_col), work(best_row, swap_col));
      }
    }

    // `pivot`为已通过绝对容差检查的当前主元值。
    const double pivot = work(pivot_row, col);
    // `row`遍历主元下方待消元的各行。
    for (std::size_t row = pivot_row + 1U; row < work.rows(); ++row) {
      // `factor`为当前行消去主元列所用的倍数。
      const double factor = work(row, col) / pivot;
      if (!std::isfinite(factor)) {
        throw std::invalid_argument("rank elimination produced a non-finite factor");
      }
      work(row, col) = 0.0;
      // `update_col`遍历当前消元行中主元右侧的剩余列。
      for (std::size_t update_col = col + 1U; update_col < work.cols();
           ++update_col) {
        // `updated`为应用行消元后尚未提交的单个矩阵元素。
        const double updated =
            work(row, update_col) - factor * work(pivot_row, update_col);
        if (!std::isfinite(updated)) {
          throw std::invalid_argument(
              "rank elimination produced a non-finite entry");
        }
        work(row, update_col) = updated;
      }
    }
    ++pivot_row;
  }
  return pivot_row;
}

}  // namespace zju::coop

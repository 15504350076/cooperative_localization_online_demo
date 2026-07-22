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

std::size_t checked_element_count(std::size_t rows, std::size_t cols) {
  // 先用除法判断乘法溢出，不能在rows*cols已经溢出后再比较。
  if (rows != 0U && cols > std::numeric_limits<std::size_t>::max() / rows) {
    throw std::length_error("DenseMatrix element count overflows size_t");
  }

  const std::size_t count = rows * cols;
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
  DenseMatrix result(size, size);
  for (std::size_t index = 0; index < size; ++index) {
    result(index, index) = 1.0;
  }
  return result;
}

DenseMatrix DenseMatrix::transpose() const {
  DenseMatrix result(cols_, rows_);
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

  DenseMatrix result(rows_, cols_);
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

  DenseMatrix result(rows_, right.cols_);
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t inner = 0; inner < cols_; ++inner) {
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

  std::vector<double> result(rows_, 0.0);
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

  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col < matrix.cols(); ++col) {
      if (!std::isfinite(matrix(row, col))) {
        throw std::invalid_argument("rank input entries must be finite");
      }
    }
  }

  // 在副本上执行带列主元的消元，调用方提供的刚度矩阵保持不变。
  DenseMatrix work = matrix;
  std::size_t pivot_row = 0U;
  for (std::size_t col = 0U;
       col < work.cols() && pivot_row < work.rows(); ++col) {
    std::size_t best_row = pivot_row;
    double best_magnitude = std::abs(work(pivot_row, col));
    for (std::size_t row = pivot_row + 1U; row < work.rows(); ++row) {
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
      for (std::size_t swap_col = col; swap_col < work.cols(); ++swap_col) {
        std::swap(work(pivot_row, swap_col), work(best_row, swap_col));
      }
    }

    const double pivot = work(pivot_row, col);
    for (std::size_t row = pivot_row + 1U; row < work.rows(); ++row) {
      const double factor = work(row, col) / pivot;
      if (!std::isfinite(factor)) {
        throw std::invalid_argument("rank elimination produced a non-finite factor");
      }
      work(row, col) = 0.0;
      for (std::size_t update_col = col + 1U; update_col < work.cols();
           ++update_col) {
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

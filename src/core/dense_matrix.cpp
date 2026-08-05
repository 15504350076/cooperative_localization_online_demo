// 模块实现：小规模稠密矩阵乘法、转置、对称化和带阈值数值秩计算。
// 关键原则：所有维度、容量和有限值错误均显式失败，禁止异常矩阵静默进入滤波；
// 这里采用清晰可审查的实现，便于Windows与ARM64保持一致的错误处理行为。
//
// 初学者阅读提示：矩阵代码中的row/col/k就是课本矩阵运算的行、列和公共求和下标。
// 遇到throw表示当前输入不能安全计算，上层会捕获异常或回滚状态；这比继续使用错误矩阵更安全。
// `namespace {}`中的辅助函数只在本.cpp内部可见，不会成为对外接口。
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
  // size_t是无符号整数，rows*cols太大时会回绕成一个小数。
  // 因此先改用“cols是否大于最大值/rows”判断，不能等乘法已经溢出后再检查。
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
    // 冒号开始“成员初始化列表”：成员在进入构造函数体前按类中声明顺序完成构造。
    : rows_(rows), cols_(cols), values_(checked_element_count(rows, cols), value) {}

// 单行函数直接返回私有成员rows_；末尾const保证查询不会修改矩阵。
std::size_t DenseMatrix::rows() const noexcept { return rows_; }

// 与rows()相同，只返回列数cols_。
std::size_t DenseMatrix::cols() const noexcept { return cols_; }

double& DenseMatrix::operator()(std::size_t row, std::size_t col) {
  // `||`表示“或”：行或列任意一个越界都不能访问vector。
  if (row >= rows_ || col >= cols_) {
    throw std::out_of_range("DenseMatrix index is out of range");
  }
  // 返回double&引用而不是副本，因此调用方可写matrix(row,col)=value修改原元素。
  return values_[row * cols_ + col];
}

const double& DenseMatrix::operator()(std::size_t row,
                                      std::size_t col) const {
  // const重载供只读矩阵使用，执行与可写重载相同的边界检查。
  if (row >= rows_ || col >= cols_) {
    throw std::out_of_range("DenseMatrix index is out of range");
  }
  // 返回const double&允许读取但禁止调用方通过引用修改该元素。
  return values_[row * cols_ + col];
}

DenseMatrix DenseMatrix::identity(std::size_t size) {
  // `result`是待填充主对角线的size×size单位矩阵。
  DenseMatrix result(size, size);
  // `index`同步遍历单位矩阵的行列主对角下标。
  for (std::size_t index = 0; index < size; ++index) {
    // 单位矩阵只有主对角线为1，其余元素在构造时已用默认值0初始化。
    result(index, index) = 1.0;
  }
  // 按值返回填好的独立矩阵对象。
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
  // `*this`表示当前DenseMatrix对象；上面的循环没有修改它，只写入result。
  return result;
}

DenseMatrix DenseMatrix::symmetrized() const {
  // 对称化只对方阵有定义，因此行数和列数必须相同。
  if (rows_ != cols_) {
    throw std::invalid_argument("only a square matrix can be symmetrized");
  }

  // `result`与方阵同维，元素(i,j)取输入(i,j)与(j,i)的均值。
  DenseMatrix result(rows_, cols_);
  // `row`和`col`分别遍历待对称化方阵的输出行、输出列。
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t col = 0; col < cols_; ++col) {
      result(row, col) =
          // 取A(i,j)与镜像元素A(j,i)的算术平均，确保输出两侧完全相等。
          0.5 * (*this)(row, col) + 0.5 * (*this)(col, row);
    }
  }
  // 返回新矩阵而不是原地修改，便于滤波更新失败时保留旧协方差。
  return result;
}

DenseMatrix DenseMatrix::operator*(const DenseMatrix& right) const {
  // A(m×n)乘B(n×p)要求A列数n等于B行数n。
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
        // `+=`把当前k项累加到C(row,col)，初始值由构造函数设为0。
        result(row, col) += left_value * right(inner, col);
      }
    }
  }
  // 所有公共维度项累加完后返回乘积矩阵。
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
  // result[row]现在等于矩阵第row行与输入列向量的内积。
  return result;
}

std::size_t numeric_rank(const DenseMatrix& matrix, double tolerance) {
  // `!(tolerance > 0)`还能同时拒绝NaN，因为NaN参与大小比较结果为false。
  // tolerance用于判断一个矩阵元素（或者消元后的主元）是否足够大，从而认为它代表一个有效的独立方向。
  if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
    throw std::invalid_argument("rank tolerance must be positive and finite");
  }

  // `row`与`col`遍历输入矩阵全部元素，先拒绝非有限值。
  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col < matrix.cols(); ++col) {
      if (!std::isfinite(matrix(row, col))) {
        // 数值秩依赖大小比较，NaN/Inf会破坏主元选择，因此提前抛出参数错误。
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
        // 发现更大的候选值后，同时更新大小及其所在行。
        best_magnitude = magnitude;
        best_row = row;
      }
    }

    // 当前列没有高于容差的主元时，该列不增加数值秩。
    if (best_magnitude <= tolerance) {
      // continue跳过本轮for循环余下代码，直接考察下一列。
      continue;
    }

    if (best_row != pivot_row) {
      // 交换行把绝对值最大的候选主元移到pivot_row，提高数值稳定性。
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
      // 主元列理论上被消为0，直接赋精确0可避免保留浮点舍入残差。
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
        // 只有验证updated为有限值后才写回工作矩阵，防止部分无效结果继续传播。
        work(row, update_col) = updated;
      }
    }
    ++pivot_row;  // 前缀++把下一主元行推进一行；成功找到一个主元即使秩增加1。
  }
  // 消元结束时pivot_row恰好等于成功找到的主元数量，也就是数值秩。
  return pivot_row;
}

}  // namespace zju::coop

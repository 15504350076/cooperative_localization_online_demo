// 模块职责：提供小规模滤波协方差和刚度矩阵所需的最小稠密矩阵运算。
// 模块边界：不追求通用线性代数功能，所有维度、索引和有限值错误均显式失败，
// 从而使算法库在Windows与RK3588上保持一致且不引入第三方矩阵库ABI。
//
// 初学者阅读提示：
// 1. 矩阵元素实际连续保存在std::vector<double>中，operator(row,col)负责把二维下标换成一维下标。
// 2. “行主序”表示先存完第0行，再存第1行；位置(row,col)对应row*列数+col。
// 3. class把数据和操作放在一起；public是调用方能用的接口，private是类自己维护的实现细节。
// 4. 本类故意功能很少，只实现当前滤波器确实需要的乘法、转置、对称化和数值秩。
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
  /**
   * `rows`和`cols`分别给出矩阵行数与列数；`value`用于初始化全部
   * `rows*cols`个行主序元素。
   */
  DenseMatrix(std::size_t rows, std::size_t cols, double value = 0.0);

  /** 返回固定行数；const不修改对象，noexcept承诺不抛异常。 */
  [[nodiscard]] std::size_t rows() const noexcept;
  /** 返回固定列数；std::size_t是用于数组大小和下标的无符号整数类型。 */
  [[nodiscard]] std::size_t cols() const noexcept;

  /** `row`为待访问的矩阵行号，`col`为列号，二者均从0开始。 */
  double& operator()(std::size_t row, std::size_t col);
  /** `row`为只读访问的矩阵行号，`col`为列号，二者均从0开始。 */
  const double& operator()(std::size_t row, std::size_t col) const;

  /** `size`同时指定单位矩阵的行数和列数。 */
  [[nodiscard]] static DenseMatrix identity(std::size_t size);
  /** 返回新矩阵Aᵀ，不修改当前矩阵；返回值优化通常会省掉临时复制。 */
  [[nodiscard]] DenseMatrix transpose() const;
  /** 消除浮点乘加造成的微小非对称；不能修复本身非正定的协方差。 */
  [[nodiscard]] DenseMatrix symmetrized() const;

  /** `right`为右乘矩阵，其行数必须等于当前矩阵列数。 */
  [[nodiscard]] DenseMatrix operator*(const DenseMatrix& right) const;
  /** `right`为列向量，其元素数必须等于当前矩阵列数。 */
  [[nodiscard]] std::vector<double> operator*(
      const std::vector<double>& right) const;

 private:
  std::size_t rows_;             ///< 固定的矩阵行数。
  std::size_t cols_;             ///< 固定的矩阵列数。
  std::vector<double> values_;   ///< 本对象独占的行主序元素，共`rows_*cols_`项。
};

/**
 * 使用带阈值的主元消元计算数值秩，供动态拓扑几何可观性判断。
 * `matrix`为待判秩的有限值矩阵；`tolerance`是绝对主元阈值，
 * 调用方应先按问题尺度选择，不能把结果理解为
 * 对任意量纲都成立的符号秩。
 */
[[nodiscard]] std::size_t numeric_rank(const DenseMatrix& matrix,
                                       double tolerance);

}  // namespace zju::coop

// 模块职责：实现惯导传播所需的三维向量、反对称矩阵和单位四元数运算。
// 关键约定：内部四元数顺序为w/x/y/z，姿态方向为车体FLU到导航ENU；
// ROS 2 Imu的x/y/z/w排列只允许在接口适配边界转换一次。
#pragma once

#include <array>

namespace zju::coop {

/** 三维向量。所有成员的单位由调用场景决定，坐标轴遵循右手系。 */
struct Vec3 {
  double x{};  ///< 所属右手坐标系的x轴分量，单位继承调用场景。
  double y{};  ///< 所属右手坐标系的y轴分量，单位继承调用场景。
  double z{};  ///< 所属右手坐标系的z轴分量，单位继承调用场景。
};

/** `left`和`right`是在同一坐标系、同一量纲下逐轴相加的向量。 */
[[nodiscard]] Vec3 operator+(const Vec3& left, const Vec3& right) noexcept;
/** `left`为被减向量，`right`为同坐标系、同量纲的减向量。 */
[[nodiscard]] Vec3 operator-(const Vec3& left, const Vec3& right) noexcept;
/** `value`为逐轴取反的向量，坐标系与量纲保持不变。 */
[[nodiscard]] Vec3 operator-(const Vec3& value) noexcept;
/** `value`为待缩放向量，`scale`为作用于三轴的无量纲倍率。 */
[[nodiscard]] Vec3 operator*(const Vec3& value, double scale) noexcept;
/** `scale`为无量纲倍率，`value`为待缩放向量。 */
[[nodiscard]] Vec3 operator*(double scale, const Vec3& value) noexcept;
/** `value`为待缩放向量，`scale`为三轴共用且由调用方保证非零的除数。 */
[[nodiscard]] Vec3 operator/(const Vec3& value, double scale) noexcept;
/** `left`和`right`为同一坐标系中的两个向量，返回其内积。 */
[[nodiscard]] double dot(const Vec3& left, const Vec3& right) noexcept;
/** `left`和`right`为同一右手坐标系中的两个向量，返回`left×right`。 */
[[nodiscard]] Vec3 cross(const Vec3& left, const Vec3& right) noexcept;
/** `value`为待计算欧氏模长的三维向量。 */
[[nodiscard]] double norm(const Vec3& value) noexcept;
/** `value`为待逐轴检查有限性的三维向量。 */
[[nodiscard]] bool finite(const Vec3& value) noexcept;

/** `value`为右手系向量；返回满足`skew(value)*v=value×v`的行主序3×3矩阵。 */
[[nodiscard]] std::array<double, 9> skew(const Vec3& value) noexcept;

/**
 * 单位四元数，内部顺序为 w,x,y,z，表示从车体坐标系到导航坐标系的旋转。
 * ROS消息使用 x,y,z,w，跨接口时必须显式换序。
 */
struct Quaternion {
  double w{1.0};  ///< w/x/y/z内部布局中的标量部，单位姿态默认取1。
  double x{};     ///< 四元数向量部x分量，对应车体FLU的x轴旋转成分。
  double y{};     ///< 四元数向量部y分量，对应车体FLU的y轴旋转成分。
  double z{};     ///< 四元数向量部z分量，对应车体FLU的z轴旋转成分。

  [[nodiscard]] bool finite() const noexcept;
  [[nodiscard]] double norm() const noexcept;
  /** 零范数或非有限输入返回false，调用方据此拒绝整次状态提交。 */
  [[nodiscard]] bool normalize() noexcept;
  [[nodiscard]] Quaternion conjugate() const noexcept;
  /**
   * `right`为Hamilton积右操作数；R(*this * right)=R(*this)R(right)，
   * 因而主动旋转向量时先应用`right`，再应用`*this`。
   */
  [[nodiscard]] Quaternion operator*(const Quaternion& right) const noexcept;
  /** `value`为车体FLU向量；主动旋转后得到导航ENU分量，不改变其物理量。 */
  [[nodiscard]] Vec3 rotate(const Vec3& value) const noexcept;

  /**
   * 将`rotation_vector`（方向为转轴、模长为弧度）映射为单位四元数。
   * 小角度分支使用一阶近似，避免sin(theta/2)/theta在零附近失去数值精度。
   */
  [[nodiscard]] static Quaternion exp(const Vec3& rotation_vector) noexcept;
};

}  // namespace zju::coop

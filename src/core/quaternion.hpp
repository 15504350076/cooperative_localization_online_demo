// 模块职责：实现惯导传播所需的三维向量、反对称矩阵和单位四元数运算。
// 关键约定：内部四元数顺序为w/x/y/z，姿态方向为车体FLU到导航ENU；
// ROS 2 Imu的x/y/z/w排列只允许在接口适配边界转换一次。
#pragma once

#include <array>

namespace zju::coop {

/** 三维向量。所有成员的单位由调用场景决定，坐标轴遵循右手系。 */
struct Vec3 {
  double x{};
  double y{};
  double z{};
};

[[nodiscard]] Vec3 operator+(const Vec3& left, const Vec3& right) noexcept;
[[nodiscard]] Vec3 operator-(const Vec3& left, const Vec3& right) noexcept;
[[nodiscard]] Vec3 operator-(const Vec3& value) noexcept;
[[nodiscard]] Vec3 operator*(const Vec3& value, double scale) noexcept;
[[nodiscard]] Vec3 operator*(double scale, const Vec3& value) noexcept;
[[nodiscard]] Vec3 operator/(const Vec3& value, double scale) noexcept;
[[nodiscard]] double dot(const Vec3& left, const Vec3& right) noexcept;
[[nodiscard]] Vec3 cross(const Vec3& left, const Vec3& right) noexcept;
[[nodiscard]] double norm(const Vec3& value) noexcept;
[[nodiscard]] bool finite(const Vec3& value) noexcept;

/** 返回向量的3×3反对称矩阵，采用行主序。 */
[[nodiscard]] std::array<double, 9> skew(const Vec3& value) noexcept;

/**
 * 单位四元数，内部顺序为 w,x,y,z，表示从车体坐标系到导航坐标系的旋转。
 * ROS消息使用 x,y,z,w，跨接口时必须显式换序。
 */
struct Quaternion {
  double w{1.0};
  double x{};
  double y{};
  double z{};

  [[nodiscard]] bool finite() const noexcept;
  [[nodiscard]] double norm() const noexcept;
  [[nodiscard]] bool normalize() noexcept;
  [[nodiscard]] Quaternion conjugate() const noexcept;
  [[nodiscard]] Quaternion operator*(const Quaternion& right) const noexcept;
  [[nodiscard]] Vec3 rotate(const Vec3& value) const noexcept;

  /** 将旋转向量（方向为转轴、模长为弧度）映射为单位四元数。 */
  [[nodiscard]] static Quaternion exp(const Vec3& rotation_vector) noexcept;
};

}  // namespace zju::coop

// 模块实现：惯导使用的向量代数、单位四元数归一化、复合、旋转和指数映射。
// 关键原则：遇到零范数或非有限值立即返回失败，避免错误姿态继续污染速度、位置和协方差。
#include "core/quaternion.hpp"

#include <cmath>
#include <limits>

namespace zju::coop {

Vec3 operator+(const Vec3& left, const Vec3& right) noexcept {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 operator-(const Vec3& left, const Vec3& right) noexcept {
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 operator-(const Vec3& value) noexcept {
  return {-value.x, -value.y, -value.z};
}

Vec3 operator*(const Vec3& value, double scale) noexcept {
  return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 operator*(double scale, const Vec3& value) noexcept {
  return value * scale;
}

Vec3 operator/(const Vec3& value, double scale) noexcept {
  return {value.x / scale, value.y / scale, value.z / scale};
}

double dot(const Vec3& left, const Vec3& right) noexcept {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec3 cross(const Vec3& left, const Vec3& right) noexcept {
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

double norm(const Vec3& value) noexcept {
  return std::sqrt(dot(value, value));
}

bool finite(const Vec3& value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

std::array<double, 9> skew(const Vec3& value) noexcept {
  return {0.0, -value.z, value.y, value.z, 0.0,
          -value.x, -value.y, value.x, 0.0};
}

bool Quaternion::finite() const noexcept {
  return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) &&
         std::isfinite(z);
}

double Quaternion::norm() const noexcept {
  return std::sqrt(w * w + x * x + y * y + z * z);
}

bool Quaternion::normalize() noexcept {
  // `magnitude`为当前wxyz四元数的欧氏范数，用于单位化与失效判定。
  const double magnitude = norm();
  // 接近零或非有限范数没有可定义的旋转方向，不能用任意单位姿态替代。
  if (!finite() || !std::isfinite(magnitude) ||
      magnitude <= std::numeric_limits<double>::min()) {
    return false;
  }
  w /= magnitude;
  x /= magnitude;
  y /= magnitude;
  z /= magnitude;
  return finite();
}

Quaternion Quaternion::conjugate() const noexcept {
  return {w, -x, -y, -z};
}

Quaternion Quaternion::operator*(const Quaternion& right) const noexcept {
  // Hamilton积的左右顺序决定旋转复合方向，惯导传播使用当前姿态右乘增量姿态。
  return {w * right.w - x * right.x - y * right.y - z * right.z,
          w * right.x + x * right.w + y * right.z - z * right.y,
          w * right.y - x * right.z + y * right.w + z * right.x,
          w * right.z + x * right.y - y * right.x + z * right.w};
}

Vec3 Quaternion::rotate(const Vec3& value) const noexcept {
  // 单位四元数旋转的向量形式，避免构造两个临时四元数。
  // `vector`是当前wxyz四元数的(x,y,z)向量部。
  const Vec3 vector{x, y, z};
  // `twice_cross`为2*(q_vec×value)，供主动旋转闭式表达式复用。
  const Vec3 twice_cross = 2.0 * cross(vector, value);
  return value + w * twice_cross + cross(vector, twice_cross);
}

Quaternion Quaternion::exp(const Vec3& rotation_vector) noexcept {
  // 旋转向量模长是总转角；四元数标量部和向量部均使用半角关系。
  // `angle`是旋转向量的总转角，单位rad。
  const double angle = zju::coop::norm(rotation_vector);
  if (!std::isfinite(angle) || !zju::coop::finite(rotation_vector)) {
    // `invalid`是为四个分量统一返回的静默NaN失败标记。
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid, invalid};
  }

  // `half_angle`为四元数三角函数所需半角，单位rad；
  // `vector_scale`为向量部相对旋转向量的无量纲系数。
  const double half_angle = 0.5 * angle;
  double vector_scale = 0.5;
  if (angle > 1.0e-8) {
    vector_scale = std::sin(half_angle) / angle;
  } else {
    // sin(theta/2)/theta = 1/2 - theta^2/48 + O(theta^4)。
    vector_scale = 0.5 - angle * angle / 48.0;
  }

  // `result`是按指数映射构造并将在返回前归一化的wxyz增量四元数。
  Quaternion result{std::cos(half_angle),
                    rotation_vector.x * vector_scale,
                    rotation_vector.y * vector_scale,
                    rotation_vector.z * vector_scale};
  if (!result.normalize()) {
    // `invalid`是归一化失败时为四个分量统一返回的NaN标记。
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid, invalid};
  }
  return result;
}

}  // namespace zju::coop

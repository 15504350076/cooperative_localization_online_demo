// 模块实现：惯导使用的向量代数、单位四元数归一化、复合、旋转和指数映射。
// 关键原则：遇到零范数或非有限值立即返回失败，避免错误姿态继续污染速度、位置和协方差。
//
// 初学者阅读顺序：先看normalize()如何保持单位四元数，再看operator*()如何合成旋转，
// 然后看rotate()如何旋转一个三维向量，最后看exp()如何把“转轴×转角”变成增量四元数。
// 本文件没有滤波器，只提供惯导会反复调用的基础数学积木。
#include "core/quaternion.hpp"

#include <cmath>
#include <limits>

namespace zju::coop {

Vec3 operator+(const Vec3& left, const Vec3& right) noexcept {
  // 花括号返回值按Vec3成员声明顺序填入x、y、z，分别执行逐轴相加。
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 operator-(const Vec3& left, const Vec3& right) noexcept {
  // 二元减号逐轴计算left-right，结果仍在与输入相同的坐标系中。
  return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 operator-(const Vec3& value) noexcept {
  // 一元减号把三个分量全部取反，相当于向量方向反转180度而模长不变。
  return {-value.x, -value.y, -value.z};
}

Vec3 operator*(const Vec3& value, double scale) noexcept {
  // 向量乘标量：三个分量使用同一个倍率scale，不改变坐标系定义。
  return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 operator*(double scale, const Vec3& value) noexcept {
  // 复用上一个value*scale重载，保证scale*value和value*scale得到完全一致的实现。
  return value * scale;
}

Vec3 operator/(const Vec3& value, double scale) noexcept {
  // 逐分量除以同一scale；底层函数不检查零，调用者必须保证scale可除。
  return {value.x / scale, value.y / scale, value.z / scale};
}

double dot(const Vec3& left, const Vec3& right) noexcept {
  // 内积公式x1*x2+y1*y2+z1*z2，常用于计算长度、夹角和投影。
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vec3 cross(const Vec3& left, const Vec3& right) noexcept {
  // 下面三项是右手系叉积的行列式展开；结果同时垂直于left和right。
  return {left.y * right.z - left.z * right.y,
          left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

double norm(const Vec3& value) noexcept {
  // value与自身内积等于长度平方，再用sqrt开平方得到欧氏长度。
  return std::sqrt(dot(value, value));
}

bool finite(const Vec3& value) noexcept {
  // `&&`具有短路特性：任意一个分量不是有限数，整个表达式立即为false。
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

std::array<double, 9> skew(const Vec3& value) noexcept {
  // 按行主序依次返回三行；该反对称矩阵左乘向量b等价于value×b。
  return {0.0, -value.z, value.y, value.z, 0.0,
          -value.x, -value.y, value.x, 0.0};
}

bool Quaternion::finite() const noexcept {
  // 成员函数中的w/x/y/z隐含来自当前对象this；四项必须全部有限。
  return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) &&
         std::isfinite(z);
}

double Quaternion::norm() const noexcept {
  // 四元数范数是四个分量平方和的平方根；有效旋转要求归一化为1。
  return std::sqrt(w * w + x * x + y * y + z * z);
}

bool Quaternion::normalize() noexcept {
  // `magnitude`为当前wxyz四元数的欧氏范数，用于单位化与失效判定。
  const double magnitude = norm();
  // 接近零或非有限范数没有可定义的旋转方向，不能用任意单位姿态替代。
  if (!finite() || !std::isfinite(magnitude) ||
      magnitude <= std::numeric_limits<double>::min()) {
    // 返回false而不是抛异常，使高频IMU路径能让上层按数值失败统一回滚。
    return false;
  }
  // `/=`是“先除再赋值”的复合运算符；四个分量除以同一范数后长度变成1。
  w /= magnitude;
  x /= magnitude;
  y /= magnitude;
  z /= magnitude;
  // 再次检查可防止极端数值在除法后产生NaN/Inf。
  return finite();
}

Quaternion Quaternion::conjugate() const noexcept {
  // 标量部不变、向量部取反；单位四元数的该结果表示相反方向的旋转。
  return {w, -x, -y, -z};
}

Quaternion Quaternion::operator*(const Quaternion& right) const noexcept {
  // Hamilton积把“当前姿态”和“本采样间隔内新增的旋转”合成新姿态。
  // 这里不能写成w*right.w、x*right.x的逐项乘法；下面四行是四元数复合旋转的固定公式。
  // 左右顺序决定先后次序，惯导传播采用“当前姿态 * 车体系增量姿态”。
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
  // 这是q*[0,value]*q共轭的等价展开，少创建临时对象但数学结果相同。
  return value + w * twice_cross + cross(vector, twice_cross);
}

Quaternion Quaternion::exp(const Vec3& rotation_vector) noexcept {
  // rotation_vector是旋转向量：方向是旋转轴，长度是旋转角度（rad）。
  // 例如(0,0,0.1)表示绕z轴旋转0.1 rad。函数名exp来自李群指数映射，
  // 不是把三个分量分别计算普通的e^x。它把三维小旋转转换成可与姿态相乘的四元数。
  // 旋转向量模长是总转角；四元数标量部和向量部使用半角关系。
  // `angle`是旋转向量的总转角，单位rad。
  const double angle = zju::coop::norm(rotation_vector);
  // `!`表示逻辑取反；只要角度或任一旋转向量分量非法，就返回统一NaN标记。
  if (!std::isfinite(angle) || !zju::coop::finite(rotation_vector)) {
    // `invalid`是为四个分量统一返回的静默NaN失败标记。
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return {invalid, invalid, invalid, invalid};
  }

  // 单位旋转四元数公式是[cos(theta/2), axis*sin(theta/2)]。
  // 因为rotation_vector=axis*theta，所以向量部也可写为
  // rotation_vector * sin(theta/2)/theta。
  // `half_angle`为公式中的theta/2；`vector_scale`就是sin(theta/2)/theta。
  const double half_angle = 0.5 * angle;
  double vector_scale = 0.5;  // 非const，因为下面会按普通角或小角分支重新赋值。
  // 1e-8 rad以上直接使用精确三角函数；该阈值只用于数值实现分支。
  if (angle > 1.0e-8) {
    // 四元数向量部=rotation_vector*sin(theta/2)/theta。
    vector_scale = std::sin(half_angle) / angle;
  } else {
    // theta接近0时直接相除会损失精度，因此使用泰勒展开：
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
  // 返回已经归一化的增量四元数；返回值按值传递，编译器可执行返回值优化而不产生额外复制。
  return result;
}

}  // namespace zju::coop

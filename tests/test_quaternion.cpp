// 模块职责：验证四元数归一化、旋转、指数映射和小角度数值稳定性。
// C++初学者阅读提示：这些用例用“已知旋转应得到已知方向”验证数学实现；
// 例如绕z轴正转90度应把x轴转到y轴，比直接比较四元数四个分量更直观。
#include "core/quaternion.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

using zju::coop::Quaternion;
using zju::coop::Vec3;

// 大角度方向、小角度展开、无效归一化和右手叉积共同锁定ENU/FLU姿态约定。
TEST_CASE(quaternion_exponential_rotates_positive_x_to_positive_y) {
  // half_pi：绕z轴的90度解析角；rotation：其指数映射单位四元数；rotated：x轴旋转后的期望y轴向量。
  const double half_pi = std::acos(-1.0) / 2.0;
  const Quaternion rotation = Quaternion::exp({0.0, 0.0, half_pi});
  const Vec3 rotated = rotation.rotate({1.0, 0.0, 0.0});

  EXPECT_TRUE(std::abs(rotated.x) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotated.y - 1.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotated.z) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotation.norm() - 1.0) < 1.0e-12);
}

TEST_CASE(quaternion_small_angle_exponential_remains_finite_and_unit) {
  // rotation：由皮弧度级旋转向量构造，用于检查小角展开不会产生非有限值或范数漂移。
  const Quaternion rotation = Quaternion::exp({1.0e-12, -2.0e-12, 3.0e-12});

  EXPECT_TRUE(rotation.finite());
  EXPECT_TRUE(std::abs(rotation.norm() - 1.0) < 1.0e-15);
}

TEST_CASE(quaternion_rejects_non_finite_normalization) {
  // rotation：w分量含NaN的非法输入，期望归一化显式失败而非传播污染值。
  Quaternion rotation{1.0, 0.0, 0.0,
                      std::numeric_limits<double>::quiet_NaN()};

  EXPECT_FALSE(rotation.normalize());
}

TEST_CASE(vec3_cross_product_follows_right_handed_flu_axes) {
  // result：FLU坐标中前向x与左向y叉乘的输出，期望严格指向上方z。
  const Vec3 result = zju::coop::cross({1.0, 0.0, 0.0},
                                       {0.0, 1.0, 0.0});

  EXPECT_TRUE(std::abs(result.x) < 1.0e-15);
  EXPECT_TRUE(std::abs(result.y) < 1.0e-15);
  EXPECT_TRUE(std::abs(result.z - 1.0) < 1.0e-15);
}

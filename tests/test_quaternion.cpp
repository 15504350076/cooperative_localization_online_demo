#include "core/quaternion.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>

using zju::coop::Quaternion;
using zju::coop::Vec3;

TEST_CASE(quaternion_exponential_rotates_positive_x_to_positive_y) {
  const double half_pi = std::acos(-1.0) / 2.0;
  const Quaternion rotation = Quaternion::exp({0.0, 0.0, half_pi});
  const Vec3 rotated = rotation.rotate({1.0, 0.0, 0.0});

  EXPECT_TRUE(std::abs(rotated.x) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotated.y - 1.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotated.z) < 1.0e-12);
  EXPECT_TRUE(std::abs(rotation.norm() - 1.0) < 1.0e-12);
}

TEST_CASE(quaternion_small_angle_exponential_remains_finite_and_unit) {
  const Quaternion rotation = Quaternion::exp({1.0e-12, -2.0e-12, 3.0e-12});

  EXPECT_TRUE(rotation.finite());
  EXPECT_TRUE(std::abs(rotation.norm() - 1.0) < 1.0e-15);
}

TEST_CASE(quaternion_rejects_non_finite_normalization) {
  Quaternion rotation{1.0, 0.0, 0.0,
                      std::numeric_limits<double>::quiet_NaN()};

  EXPECT_FALSE(rotation.normalize());
}

TEST_CASE(vec3_cross_product_follows_right_handed_flu_axes) {
  const Vec3 result = zju::coop::cross({1.0, 0.0, 0.0},
                                       {0.0, 1.0, 0.0});

  EXPECT_TRUE(std::abs(result.x) < 1.0e-15);
  EXPECT_TRUE(std::abs(result.y) < 1.0e-15);
  EXPECT_TRUE(std::abs(result.z - 1.0) < 1.0e-15);
}

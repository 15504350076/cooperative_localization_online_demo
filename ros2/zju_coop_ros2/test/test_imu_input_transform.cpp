#include "../src/imu_input_transform.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace {

bool close(double left, double right) {
  return std::abs(left - right) < 1.0e-12;
}

void require(bool condition) {
  if (!condition) {
    throw std::runtime_error("IMU input transform self-check failed");
  }
}

}  // namespace

int main() {
  const zju_coop_ros2::ImuInputTransform identity;
  const auto unchanged = identity.linear_acceleration({1.0, 2.0, 3.0});
  require(close(unchanged[0], 1.0));
  require(close(unchanged[1], 2.0));
  require(close(unchanged[2], 3.0));

  // Use four distinct components so an accidental xyzw/wxyz permutation
  // cannot pass this check merely because most components are zero.
  const std::array<double, 4> asymmetric_xyzw{0.1, 0.2, 0.3,
                                               std::sqrt(0.86)};
  const auto unchanged_orientation =
      identity.orientation_flu_to_navigation(asymmetric_xyzw);
  for (std::size_t index = 0U; index < asymmetric_xyzw.size(); ++index) {
    require(close(unchanged_orientation[index], asymmetric_xyzw[index]));
  }

  constexpr double degrees_to_radians =
      3.14159265358979323846 / 180.0;
  const zju_coop_ros2::ImuInputTransform frd_degrees_to_flu(
      {1.0, 0.0, 0.0, 0.0}, degrees_to_radians, 1.0);
  const auto acceleration =
      frd_degrees_to_flu.linear_acceleration({1.0, 2.0, -9.8});
  require(close(acceleration[0], 1.0));
  require(close(acceleration[1], -2.0));
  require(close(acceleration[2], 9.8));
  const auto angular_velocity =
      frd_degrees_to_flu.angular_velocity({180.0, 0.0, 0.0});
  require(close(angular_velocity[0], 3.14159265358979323846));

  const std::array<double, 9> covariance{
      1.0, 0.2, 0.3,
      0.2, 2.0, 0.4,
      0.3, 0.4, 3.0};
  const auto transformed =
      frd_degrees_to_flu.orientation_covariance(covariance);
  require(close(transformed[1], -0.2));
  require(close(transformed[2], -0.3));
  require(close(transformed[5], 0.4));
  std::array<double, 9> unavailable{};
  unavailable[0] = -1.0;
  const auto preserved =
      frd_degrees_to_flu.angular_velocity_covariance(unavailable);
  require(close(preserved[0], -1.0));
  require(close(preserved[1], 0.0));

  bool rejected = false;
  try {
    const zju_coop_ros2::ImuInputTransform invalid(
        {0.0, 0.0, 0.0, 0.0}, 1.0, 1.0);
    static_cast<void>(invalid);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected);
  return 0;
}

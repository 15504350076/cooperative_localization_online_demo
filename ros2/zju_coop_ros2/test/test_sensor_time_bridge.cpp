#include "../src/sensor_time_bridge.hpp"

#include <cstdint>
#include <limits>

int main() {
  zju_coop_ros2::SensorTimeBridge clock;
  constexpr std::uint64_t second = 1'000'000'000ULL;

  if (clock.current_time_ns(100U * second) != 0U ||
      !clock.measurement_is_plausible(10U * second, 100U * second,
                                     100'000'000ULL, 500'000'000ULL) ||
      clock.observe(10U * second, 100U * second) != 10U * second ||
      clock.current_time_ns(100U * second + 25'000'000ULL) !=
          10U * second + 25'000'000ULL) {
    return 1;
  }
  if (!clock.measurement_is_plausible(
          10U * second + 125'000'000ULL,
          100U * second + 25'000'000ULL, 100'000'000ULL, 500'000'000ULL) ||
      clock.measurement_is_plausible(
          10U * second + 125'000'001ULL,
          100U * second + 25'000'000ULL, 100'000'000ULL, 500'000'000ULL) ||
      !clock.measurement_is_plausible(
          9U * second + 525'000'000ULL,
          100U * second + 25'000'000ULL, 100'000'000ULL, 500'000'000ULL) ||
      clock.measurement_is_plausible(
          9U * second + 524'999'999ULL,
          100U * second + 25'000'000ULL, 100'000'000ULL, 500'000'000ULL)) {
    return 2;
  }

  // An IMU sample may be delayed; receive time must remain in the UWB domain
  // and must not move the local UWB clock backwards.
  if (clock.observe(10U * second + 30'000'000ULL,
                    100U * second + 125'000'000ULL) !=
          10U * second + 125'000'000ULL ||
      clock.current_time_ns(100U * second + 130'000'000ULL) !=
          10U * second + 125'000'000ULL) {
    return 3;
  }

  // A subsequent fresh sample releases the monotonic floor and disciplines
  // steady-clock frequency error without moving exposed time backwards.
  if (clock.observe(10U * second + 130'000'000ULL,
                    100U * second + 140'000'000ULL) !=
          10U * second + 130'000'000ULL ||
      clock.current_time_ns(100U * second + 145'000'000ULL) !=
          10U * second + 135'000'000ULL ||
      clock.current_time_ns(100U * second + 155'000'000ULL) !=
          10U * second + 145'000'000ULL) {
    return 4;
  }

  // A forward correction follows the sensor clock; overflow saturates.
  if (clock.observe(20U * second, 100U * second + 160'000'000ULL) !=
          20U * second ||
      clock.observe(std::numeric_limits<std::uint64_t>::max() - 5U,
                    100U * second + 170'000'000ULL) !=
          std::numeric_limits<std::uint64_t>::max() - 5U ||
      clock.current_time_ns(100U * second + 170'000'010ULL) !=
          std::numeric_limits<std::uint64_t>::max()) {
    return 5;
  }
}

#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

namespace zju_coop_ros2 {

// Estimates local arrival time in a sensor header's UWB time domain. This does
// not synchronize clocks; the sensor driver must already stamp the sample time.
class SensorTimeBridge {
 public:
  [[nodiscard]] std::uint64_t current_time_ns(
      std::uint64_t steady_time_ns) const noexcept {
    if (!initialized_) {
      return 0U;
    }
    std::uint64_t estimate = anchor_uwb_time_ns_;
    if (steady_time_ns > anchor_steady_time_ns_) {
      const auto elapsed = steady_time_ns - anchor_steady_time_ns_;
      estimate = elapsed > std::numeric_limits<std::uint64_t>::max() -
                               anchor_uwb_time_ns_
                     ? std::numeric_limits<std::uint64_t>::max()
                     : anchor_uwb_time_ns_ + elapsed;
    }
    return std::max(estimate, monotonic_floor_uwb_time_ns_);
  }

  [[nodiscard]] std::uint64_t receive_time_ns(
      std::uint64_t measurement_time_ns,
      std::uint64_t steady_receive_time_ns) const noexcept {
    return std::max(measurement_time_ns,
                    current_time_ns(steady_receive_time_ns));
  }

  [[nodiscard]] bool measurement_is_plausible(
      std::uint64_t measurement_time_ns,
      std::uint64_t steady_receive_time_ns,
      std::uint64_t max_future_skew_ns,
      std::uint64_t max_receive_delay_ns) const noexcept {
    if (!initialized_) {
      return true;
    }
    const auto current = current_time_ns(steady_receive_time_ns);
    return measurement_time_ns >= current
               ? measurement_time_ns - current <= max_future_skew_ns
               : current - measurement_time_ns <= max_receive_delay_ns;
  }

  std::uint64_t observe(std::uint64_t measurement_time_ns,
                        std::uint64_t steady_receive_time_ns) noexcept {
    const auto previous_current =
        current_time_ns(steady_receive_time_ns);
    // Re-anchor to every accepted sensor sample so steady-clock frequency
    // error cannot accumulate indefinitely. The floor prevents current time
    // from moving backwards when a delayed sample corrects the estimate.
    monotonic_floor_uwb_time_ns_ = previous_current;
    anchor_uwb_time_ns_ = measurement_time_ns;
    anchor_steady_time_ns_ = steady_receive_time_ns;
    initialized_ = true;
    return current_time_ns(steady_receive_time_ns);
  }

 private:
  bool initialized_{};
  std::uint64_t anchor_uwb_time_ns_{};
  std::uint64_t anchor_steady_time_ns_{};
  std::uint64_t monotonic_floor_uwb_time_ns_{};
};

}  // namespace zju_coop_ros2

#pragma once

#include <algorithm>
#include <cstdint>
#include <limits>

namespace zju_coop_ros2 {

// 将本机 steady_clock 的经过时间映射到传感器头使用的 UWB_SYSTEM_TIME。
// 它只估计“当前 UWB 时刻/接收时刻”，不执行设备授时；上游驱动仍必须把
// header.stamp 填成真实测量事件的 UWB 时间，不能填 ROS 发布时间。
class SensorTimeBridge {
 public:
  // 以最近一次 (UWB时刻, steady时刻) 为锚点外推当前 UWB 时刻。
  // monotonic floor 保证迟到样本重新锚定后，估计时间也不会倒退。
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

  // SDK 要求 receive_timestamp_ns 与 measurement_time_ns 在同一时间域，
  // 且接收时刻不能早于测量时刻，因此取测量时刻和当前估计的较大值。
  [[nodiscard]] std::uint64_t receive_time_ns(
      std::uint64_t measurement_time_ns,
      std::uint64_t steady_receive_time_ns) const noexcept {
    return std::max(measurement_time_ns,
                    current_time_ns(steady_receive_time_ns));
  }

  // 检查测量相对当前估计是否明显超前或滞后；未初始化时允许首个样本建锚。
  // 两个门限是数据包时效保护，不是对 UWB 同步误差的估计。
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

  // 用已接受的传感器样本更新锚点。每次重新锚定可限制 steady_clock 与
  // UWB 时钟频率差长期积累；返回更新后的当前 UWB 时刻。
  std::uint64_t observe(std::uint64_t measurement_time_ns,
                        std::uint64_t steady_receive_time_ns) noexcept {
    const auto previous_current =
        current_time_ns(steady_receive_time_ns);
    // 先保存旧锚点外推结果作为下界，再切换到新锚点。
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

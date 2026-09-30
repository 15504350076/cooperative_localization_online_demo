// 参考车侧二维协同融合 ROS 2 适配节点。
// 数据流：各车 NodeState + 车间 UWB 测距 -> SDK 分布式二维修正器 -> 三车相对位姿。
// 滤波状态只含各非参考车的 [delta_e, delta_n]；各车完整惯导状态仍留在本机。
// UWB 修正相对平面位置，不在本节点估计姿态或 IMU 零偏。
#include "cooperative_interfaces/msg/uwb_range.hpp"
#include "cooperative_localization_msgs/msg/cooperative_pose2_d_array.hpp"
#include "cooperative_localization_msgs/msg/node_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_time_bridge.hpp"
#include "zju_coop/c_api.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
constexpr std::uint64_t kNanosecondsPerMillisecond = 1'000'000ULL;
constexpr std::uint64_t kMaximumMilliseconds =
    std::numeric_limits<std::uint64_t>::max() / kNanosecondsPerMillisecond;

// SDK 句柄创建失败属于不可恢复的启动错误，直接转成异常交给 main 报告。
void require_ok(zju_coop_error_code_t code, const char* operation) {
  if (code != ZJU_COOP_OK) {
    throw std::runtime_error(std::string(operation) + ": " +
                             zju_coop_error_string(code));
  }
}

// 将 ROS 消息时间转成 SDK 使用的纳秒整数；0 表示无效。
std::uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         stamp.nanosec;
}

// 本机 steady_clock 只提供可靠的经过时间，由 SensorTimeBridge 映射到 UWB 时间域。
std::uint64_t steady_time_ns() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count > 0 ? static_cast<std::uint64_t>(count) : 0U;
}

class CooperativeFusionNode final : public rclcpp::Node {
 public:
  CooperativeFusionNode()
      : Node("zju_cooperative_fusion_node") {
    // node_ids 定义本次融合允许接收的车辆集合；reference_node_id 定义输出原点。
    // 从车反馈默认关闭，最小实现只发布参考车本地的 poses_2d。
    const auto configured_ids =
        declare_parameter<std::vector<std::int64_t>>("node_ids", {1, 2, 3});
    const auto configured_reference =
        declare_parameter<std::int64_t>("reference_node_id", 1);
    input_mode_ = declare_parameter<std::string>("input_mode", "uwb_only");
    uwb_only_ = input_mode_ == "uwb_only";
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);
    const bool enable_follower_feedback =
        declare_parameter<bool>("enable_follower_feedback", false);
    range_std_m_ = declare_parameter<double>("range_std_m", 0.10);
    minimum_triangle_height_m_ =
        declare_parameter<double>("minimum_triangle_height_m", 1.0e-3);
    const auto uwb_epoch_wait_ms =
        declare_parameter<std::int64_t>("uwb_epoch_wait_ms", 10);
    const auto uwb_epoch_timeout_ms =
        declare_parameter<std::int64_t>("uwb_epoch_timeout_ms", 100);
    uwb_reciprocal_max_difference_m_ = declare_parameter<double>(
        "uwb_reciprocal_max_difference_m", 0.5);
    uwb_maximum_range_rate_mps_ =
        declare_parameter<double>("uwb_maximum_range_rate_mps", 10.0);
    uwb_range_jump_tolerance_m_ =
        declare_parameter<double>("uwb_range_jump_tolerance_m", 0.5);
    const auto timeout_ms =
        declare_parameter<std::int64_t>("node_state_timeout_ms", 300);
    // 数据包时效门限只用于拒绝异常时间戳，不是 UWB 授时精度指标。
    const auto max_future_skew_ms =
        declare_parameter<std::int64_t>("max_future_skew_ms", 100);
    const auto max_receive_delay_ms =
        declare_parameter<std::int64_t>("max_receive_delay_ms", 500);
    common_enu_frame_id_ =
        declare_parameter<std::string>("common_enu_frame_id", "common_enu");
    use_sim_time_ = get_parameter("use_sim_time").as_bool();

    // 启动时严格校验，避免编号截断、零噪声或毫秒转纳秒溢出进入 SDK。
    if ((input_mode_ != "uwb_only" && input_mode_ != "imu_uwb") ||
        configured_ids.size() < 2U ||
        (uwb_only_ && configured_ids.size() != 3U) ||
        configured_reference <= 0 ||
        configured_reference > std::numeric_limits<std::uint16_t>::max() ||
        !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
        !std::isfinite(range_std_m_) || range_std_m_ <= 0.0 ||
        !std::isfinite(minimum_triangle_height_m_) ||
        minimum_triangle_height_m_ <= 0.0 ||
        uwb_epoch_wait_ms < 0 || uwb_epoch_timeout_ms <= uwb_epoch_wait_ms ||
        static_cast<std::uint64_t>(uwb_epoch_timeout_ms) >
            kMaximumMilliseconds ||
        !std::isfinite(uwb_reciprocal_max_difference_m_) ||
        uwb_reciprocal_max_difference_m_ < 0.0 ||
        !std::isfinite(uwb_maximum_range_rate_mps_) ||
        uwb_maximum_range_rate_mps_ <= 0.0 ||
        !std::isfinite(uwb_range_jump_tolerance_m_) ||
        uwb_range_jump_tolerance_m_ < 0.0 ||
        timeout_ms <= 0 ||
        static_cast<std::uint64_t>(timeout_ms) > kMaximumMilliseconds ||
        max_future_skew_ms <= 0 ||
        static_cast<std::uint64_t>(max_future_skew_ms) >
            kMaximumMilliseconds ||
        max_receive_delay_ms <= 0 ||
        static_cast<std::uint64_t>(max_receive_delay_ms) >
            kMaximumMilliseconds ||
        common_enu_frame_id_.empty()) {
      throw std::invalid_argument("invalid cooperative fusion parameters");
    }
    reference_node_id_ = static_cast<std::uint32_t>(configured_reference);

    // 同时建立有序车辆列表、去重集合和每车最后接收时刻。
    std::vector<zju_coop_node_initialization_t> nodes(configured_ids.size());
    std::unordered_set<std::uint32_t> unique_ids;
    node_ids_.reserve(configured_ids.size());
    for (std::size_t index = 0U; index < configured_ids.size(); ++index) {
      const auto value = configured_ids[index];
      if (value <= 0 || value > std::numeric_limits<std::uint16_t>::max()) {
        throw std::invalid_argument("node_ids must fit the current UWB C ABI");
      }
      const auto node_id = static_cast<std::uint32_t>(value);
      if (!unique_ids.emplace(node_id).second) {
        throw std::invalid_argument("node_ids must be unique");
      }
      node_ids_.push_back(node_id);
      last_node_state_timestamp_ns_.emplace(node_id, 0U);
      require_ok(zju_coop_node_initialization_init(&nodes[index]),
                 "zju_coop_node_initialization_init");
      nodes[index].node_id = node_id;
    }
    if (unique_ids.count(reference_node_id_) == 0U) {
      throw std::invalid_argument("reference_node_id is absent from node_ids");
    }

    if (uwb_only_) {
      canonical_node_ids_[0] = reference_node_id_;
      std::vector<std::uint32_t> other_ids;
      for (const auto node_id : node_ids_) {
        if (node_id != reference_node_id_) {
          other_ids.push_back(node_id);
        }
      }
      std::sort(other_ids.begin(), other_ids.end());
      canonical_node_ids_[1] = other_ids[0];
      canonical_node_ids_[2] = other_ids[1];
    }

    // 创建参考车上的分布式修正器。max_prediction_step_s 限制一次外推跨度，
    // edge_timeout_ns 决定一条测距边多久后不再支撑 position_valid。
    max_future_skew_ns_ = static_cast<std::uint64_t>(max_future_skew_ms) *
                          kNanosecondsPerMillisecond;
    max_receive_delay_ns_ =
        static_cast<std::uint64_t>(max_receive_delay_ms) *
        kNanosecondsPerMillisecond;
    uwb_epoch_wait_ns_ = static_cast<std::uint64_t>(uwb_epoch_wait_ms) *
                         kNanosecondsPerMillisecond;
    uwb_epoch_timeout_ns_ =
        static_cast<std::uint64_t>(uwb_epoch_timeout_ms) *
        kNanosecondsPerMillisecond;
    if (!uwb_only_) {
      zju_coop_config_t config{};
      require_ok(zju_coop_config_init(&config), "zju_coop_config_init");
      config.reference_node_id = reference_node_id_;
      config.nodes = nodes.data();
      config.node_count = static_cast<std::uint32_t>(nodes.size());
      config.max_prediction_step_s = 0.10;
      config.edge_timeout_ns = static_cast<std::uint64_t>(timeout_ms) *
                               kNanosecondsPerMillisecond;
      config.max_future_skew_ns = max_future_skew_ns_;
      config.max_receive_delay_ns = max_receive_delay_ns_;
      require_ok(zju_coop_distributed_create(&config, &handle_),
                 "zju_coop_distributed_create");
    }

    // poses_2d 面向参考车本地规划/控制；feedback_poses_2d 是参数控制的同内容反馈口。
    // NodeState/UWB 使用 best-effort 以优先实时性，融合结果使用 reliable。
    pose_publisher_ = create_publisher<
        cooperative_localization_msgs::msg::CooperativePose2DArray>(
        "poses_2d", rclcpp::QoS(1).reliable().durability_volatile());
    if (enable_follower_feedback) {
      feedback_publisher_ = create_publisher<
          cooperative_localization_msgs::msg::CooperativePose2DArray>(
          "feedback_poses_2d",
          rclcpp::QoS(1).reliable().durability_volatile());
    }
    if (!uwb_only_) {
      node_state_subscription_ = create_subscription<
          cooperative_localization_msgs::msg::NodeState>(
          "node_state", rclcpp::QoS(5).best_effort().durability_volatile(),
          [this](cooperative_localization_msgs::msg::NodeState::ConstSharedPtr message) {
            on_node_state(*message);
          });
    }
    uwb_subscription_ = create_subscription<cooperative_interfaces::msg::UwbRange>(
        "uwb_range", rclcpp::QoS(20).best_effort().durability_volatile(),
        [this](cooperative_interfaces::msg::UwbRange::ConstSharedPtr message) {
          on_uwb_range(*message);
        });
    if (uwb_only_) {
      const auto timer_ms = std::max<std::int64_t>(
          1, std::min<std::int64_t>(10, (uwb_epoch_wait_ms + 1) / 2));
      uwb_epoch_timer_ = create_wall_timer(
          std::chrono::milliseconds(timer_ms),
          [this]() { process_uwb_epochs(steady_time_ns()); });
    } else {
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_hz));
      publish_timer_ = create_wall_timer(period, [this]() { publish_pose(); });
    }

    RCLCPP_INFO(get_logger(),
                "cooperative output ready: mode=%s, %zu nodes, reference=%u, "
                "time_source=%s; timestamp guards future=%lld ms, "
                "receive-delay=%lld ms (data rejection, not sync accuracy)",
                input_mode_.c_str(), node_ids_.size(), reference_node_id_,
                uwb_only_ ? "UWB message header"
                          : (use_sim_time_ ? "ROS simulation clock"
                                           : "reference NodeState UWB clock"),
                static_cast<long long>(max_future_skew_ms),
                static_cast<long long>(max_receive_delay_ms));
  }

  ~CooperativeFusionNode() override {
    if (handle_ != nullptr) {
      static_cast<void>(zju_coop_distributed_destroy(handle_));
    }
  }

 private:
  struct UwbEpoch {
    std::array<std::array<double, 2>, 3> distance_m{};
    std::array<std::array<bool, 2>, 3> present{};
    std::uint64_t first_receive_steady_ns{};
  };

  // 返回当前统一算法时刻。实盒由参考车 NodeState 的 UWB 时间锚点外推，
  // Gazebo 则直接使用 /clock；锚点尚未建立时返回 0，暂停 UWB处理和输出。
  std::uint64_t current_time_ns(std::uint64_t steady_now_ns) const {
    if (!use_sim_time_) {
      return reference_time_bridge_.current_time_ns(steady_now_ns);
    }
    const auto ros_time = now().nanoseconds();
    return ros_time > 0 ? static_cast<std::uint64_t>(ros_time) : 0U;
  }

  // 接收各车本地惯导航位推算结果，校验 frame/id/时序后写入 SDK 状态历史。
  // SDK 会在测距时刻对两端历史状态插值或有限外推，不直接用消息到达时刻融合。
  void on_node_state(
      const cooperative_localization_msgs::msg::NodeState& message) {
    if (message.header.frame_id != common_enu_frame_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting NodeState outside configured common ENU frame");
      return;
    }
    const std::uint64_t measurement_time = stamp_ns(message.header.stamp);
    const auto previous = last_node_state_timestamp_ns_.find(message.node_id);
    if (previous == last_node_state_timestamp_ns_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting NodeState from an unconfigured node");
      return;
    }
    if (measurement_time == 0U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting NodeState with zero/invalid timestamp");
      return;
    }
    if (measurement_time <= previous->second) {
      // 所有车辆都要求时间严格递增；参考车回拨还会破坏全局 UWB 时间锚点。
      if (message.node_id == reference_node_id_ &&
          measurement_time < previous->second) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "reference NodeState UWB time rolled back; rejecting the new "
            "epoch until this fusion node is restarted");
        return;
      }
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "rejecting duplicate/out-of-order NodeState from node %u",
          message.node_id);
      return;
    }
    const auto steady_receive_time = steady_time_ns();
    auto receive_time = current_time_ns(steady_receive_time);
    const bool is_reference = message.node_id == reference_node_id_;
    const bool initializes_reference_clock =
        !use_sim_time_ && is_reference && receive_time == 0U;
    if (initializes_reference_clock) {
      // 首个参考车本地状态是启动阶段唯一可信的 UWB 时间锚点。
      receive_time = measurement_time;
    } else if (!use_sim_time_ && is_reference &&
               !reference_time_bridge_.measurement_is_plausible(
                   measurement_time, steady_receive_time,
                   max_future_skew_ns_, max_receive_delay_ns_)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "reference NodeState UWB time discontinuity exceeds the configured "
          "window; reject this epoch and restart all localization nodes if "
          "the new time base persists");
      return;
    }
    zju_coop_node_state_t state{};
    if (receive_time == 0U ||
        zju_coop_node_state_init(&state) != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "waiting for the reference NodeState UWB time anchor");
      return;
    }
    state.node_id = message.node_id;
    state.timestamp_ns = measurement_time;
    state.receive_timestamp_ns = receive_time;
    std::copy(message.position_enu_m.begin(), message.position_enu_m.end(),
              state.position_enu_m);
    std::copy(message.velocity_enu_mps.begin(), message.velocity_enu_mps.end(),
              state.velocity_enu_mps);
    std::copy(message.orientation_flu_to_enu_xyzw.begin(),
              message.orientation_flu_to_enu_xyzw.end(),
              state.orientation_flu_to_enu_xyzw);
    state.valid = message.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    const auto code = zju_coop_distributed_push_node_state(handle_, &state);
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "NodeState rejected: %s",
                           zju_coop_error_string(code));
      return;
    }
    previous->second = measurement_time;
    if (!use_sim_time_ && is_reference) {
      // 只让已被 SDK 接受的参考车状态校准时间桥；从车网络延迟不能改变统一时钟。
      static_cast<void>(reference_time_bridge_.observe(
          measurement_time, steady_receive_time));
    }
  }

  // 返回规范节点顺序中的无向边编号：01、02、12；3表示不是本次三车边。
  std::size_t canonical_edge_index(std::uint32_t first,
                                   std::uint32_t second) const {
    if (first > second) {
      std::swap(first, second);
    }
    for (std::size_t index = 0U; index < 3U; ++index) {
      auto left = canonical_node_ids_[index == 2U ? 1U : 0U];
      auto right = canonical_node_ids_[index == 0U ? 1U : 2U];
      if (left > right) {
        std::swap(left, right);
      }
      if (first == left && second == right) {
        return index;
      }
    }
    return 3U;
  }

  void publish_uwb_formation(std::uint64_t measurement_time,
                             bool solved) {
    cooperative_localization_msgs::msg::CooperativePose2DArray output;
    output.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(measurement_time), RCL_ROS_TIME);
    output.header.frame_id =
        "coop_ref_" + std::to_string(reference_node_id_) + "_relative";
    output.reference_node_id = reference_node_id_;
    output.vehicles.reserve(3U);
    for (std::size_t index = 0U; index < 3U; ++index) {
      cooperative_localization_msgs::msg::VehiclePose2D pose;
      pose.node_id = canonical_node_ids_[index];
      pose.x_m = latest_uwb_x_m_[index];
      pose.y_m = latest_uwb_y_m_[index];
      pose.yaw_rad = 0.0;
      pose.position_valid = solved;
      pose.yaw_valid = false;
      output.vehicles.push_back(pose);
    }
    pose_publisher_->publish(output);
    if (feedback_publisher_ != nullptr) {
      feedback_publisher_->publish(output);
    }
    last_uwb_output_timestamp_ns_ = measurement_time;
  }

  bool uwb_epoch_has_all_edges(const UwbEpoch& epoch) const {
    return std::all_of(
        epoch.present.begin(), epoch.present.end(),
        [](const auto& directions) { return directions[0] || directions[1]; });
  }

  bool uwb_epoch_has_all_reciprocals(const UwbEpoch& epoch) const {
    return std::all_of(
        epoch.present.begin(), epoch.present.end(),
        [](const auto& directions) { return directions[0] && directions[1]; });
  }

  // 收齐同一header.stamp的三条无向边后，优先等待双向值并取均值；若只有
  // 单向值则在短等待窗结束后继续。缺边超时、双向差过大或距离突变都发布
  // position_valid=false，而不是把上一帧伪装成新的有效定位。
  void solve_uwb_epoch(std::uint64_t measurement_time,
                       const UwbEpoch& epoch) {
    if (!uwb_epoch_has_all_edges(epoch)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "dropping UWB-only epoch with a missing edge");
      return;
    }

    std::array<double, 3> distance_m{};
    bool measurements_valid = true;
    for (std::size_t edge = 0U; edge < distance_m.size(); ++edge) {
      const bool forward = epoch.present[edge][0];
      const bool reverse = epoch.present[edge][1];
      if (forward && reverse) {
        const double difference = std::abs(
            epoch.distance_m[edge][0] - epoch.distance_m[edge][1]);
        if (difference > uwb_reciprocal_max_difference_m_) {
          measurements_valid = false;
        }
        distance_m[edge] =
            0.5 * (epoch.distance_m[edge][0] + epoch.distance_m[edge][1]);
      } else {
        distance_m[edge] = epoch.distance_m[edge][forward ? 0U : 1U];
      }

      if (last_accepted_uwb_timestamp_ns_[edge] != 0U &&
          measurement_time > last_accepted_uwb_timestamp_ns_[edge]) {
        const double elapsed_s = static_cast<double>(
            measurement_time - last_accepted_uwb_timestamp_ns_[edge]) /
            static_cast<double>(kNanosecondsPerSecond);
        const double maximum_jump = uwb_range_jump_tolerance_m_ +
                                    uwb_maximum_range_rate_mps_ * elapsed_s;
        if (std::abs(distance_m[edge] - last_accepted_uwb_distance_m_[edge]) >
            maximum_jump) {
          measurements_valid = false;
        }
      }
    }

    zju_coop_three_node_ranges_t ranges{};
    zju_coop_three_node_formation_t formation{};
    if (zju_coop_three_node_ranges_init(&ranges) != ZJU_COOP_OK ||
        zju_coop_three_node_formation_init(&formation) != ZJU_COOP_OK) {
      return;
    }
    std::copy(canonical_node_ids_.begin(), canonical_node_ids_.end(),
              ranges.node_ids);
    ranges.range_01_m = distance_m[0];
    ranges.range_02_m = distance_m[1];
    ranges.range_12_m = distance_m[2];
    ranges.minimum_height_m = minimum_triangle_height_m_;
    const auto code = measurements_valid
                          ? zju_coop_solve_three_node_formation(&ranges,
                                                                &formation)
                          : ZJU_COOP_OK;
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "UWB-only formation call failed: %s",
                           zju_coop_error_string(code));
      return;
    }

    const bool solved = measurements_valid &&
                        formation.disposition == ZJU_COOP_FORMATION_SOLVED;
    if (solved) {
      for (std::size_t index = 0U; index < 3U; ++index) {
        latest_uwb_x_m_[index] = formation.x_m[index];
        latest_uwb_y_m_[index] = formation.y_m[index];
      }
      last_accepted_uwb_distance_m_ = distance_m;
      last_accepted_uwb_timestamp_ns_.fill(measurement_time);
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "UWB-only epoch rejected (measurement_consistency=%s, "
          "formation=%d)",
          measurements_valid ? "ok" : "failed",
          static_cast<int>(formation.disposition));
    }
    publish_uwb_formation(measurement_time, solved);
  }

  void process_uwb_epochs(std::uint64_t steady_now_ns) {
    while (!uwb_epochs_.empty()) {
      auto epoch = uwb_epochs_.begin();
      if (epoch->first <= last_uwb_output_timestamp_ns_) {
        uwb_epochs_.erase(epoch);
        continue;
      }
      const auto age_ns = steady_now_ns >= epoch->second.first_receive_steady_ns
                              ? steady_now_ns -
                                    epoch->second.first_receive_steady_ns
                              : 0U;
      const bool complete = uwb_epoch_has_all_edges(epoch->second);
      const bool has_newer_epoch = std::next(epoch) != uwb_epochs_.end();
      const bool ready = complete &&
                         (uwb_epoch_has_all_reciprocals(epoch->second) ||
                          age_ns >= uwb_epoch_wait_ns_);
      const bool expired = age_ns >= uwb_epoch_timeout_ns_;
      const bool superseded_incomplete =
          !complete && has_newer_epoch && age_ns >= uwb_epoch_wait_ns_;
      if (!ready && !expired && !superseded_incomplete) {
        break;
      }
      const auto measurement_time = epoch->first;
      const auto value = epoch->second;
      uwb_epochs_.erase(epoch);
      last_uwb_retired_timestamp_ns_ = measurement_time;
      solve_uwb_epoch(measurement_time, value);
    }
  }

  // UWB-only模式按完全相同的header.stamp归并三条无向边和两个测量方向。
  void on_uwb_only_range(
      const cooperative_interfaces::msg::UwbRange& message,
      std::uint64_t measurement_time) {
    if (measurement_time <= last_uwb_retired_timestamp_ns_) {
      return;
    }
    const auto edge_index =
        canonical_edge_index(message.src_id, message.target_id);
    if (edge_index >= 3U) {
      return;
    }
    const std::size_t direction =
        message.src_id < message.target_id ? 0U : 1U;
    const auto steady_receive_time = steady_time_ns();
    auto [epoch, inserted] = uwb_epochs_.try_emplace(measurement_time);
    if (inserted) {
      epoch->second.first_receive_steady_ns = steady_receive_time;
    }
    epoch->second.distance_m[edge_index][direction] =
        static_cast<double>(message.distance);
    epoch->second.present[edge_index][direction] = true;
    process_uwb_epochs(steady_receive_time);
    while (uwb_epochs_.size() > 64U) {
      const auto oldest_time = uwb_epochs_.begin()->first;
      uwb_epochs_.erase(uwb_epochs_.begin());
      last_uwb_retired_timestamp_ns_ = oldest_time;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "dropping an old incomplete UWB-only epoch");
    }
  }

  // 接收上交盒端正式 UwbRange。rolling只供上交调试时间戳，定位算法不使用；
  // 消息没有测距标准差，因此标准差取配置值。
  void on_uwb_range(const cooperative_interfaces::msg::UwbRange& message) {
    const auto measurement_time = stamp_ns(message.header.stamp);
    if (message.src_id == message.target_id ||
        last_node_state_timestamp_ns_.count(message.src_id) == 0U ||
        last_node_state_timestamp_ns_.count(message.target_id) == 0U ||
        message.src_id > std::numeric_limits<std::uint16_t>::max() ||
        message.target_id > std::numeric_limits<std::uint16_t>::max() ||
        measurement_time == 0U || !std::isfinite(message.distance) ||
        message.distance <= 0.0F) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting invalid/unconfigured UWB range");
      return;
    }
    if (uwb_only_) {
      on_uwb_only_range(message, measurement_time);
      return;
    }
    const auto receive_time = current_time_ns(steady_time_ns());
    zju_coop_range_packet_t packet{};
    zju_coop_range_processing_result_t result{};
    if (receive_time == 0U ||
        zju_coop_range_packet_init(&packet) != ZJU_COOP_OK ||
        zju_coop_range_processing_result_init(&result) != ZJU_COOP_OK) {
      return;
    }
    packet.from_node = static_cast<std::uint16_t>(message.src_id);
    packet.to_node = static_cast<std::uint16_t>(message.target_id);
    packet.sequence = ++uwb_sequence_;
    packet.timestamp_ns = measurement_time;
    packet.receive_timestamp_ns = receive_time;
    packet.range_m = message.distance;
    packet.range_std_m = range_std_m_;
    packet.valid = ZJU_COOP_TRUE;
    packet.status = ZJU_COOP_RANGE_STATUS_OK;
    // SDK 在测距时刻对齐两车 NodeState，以预测距离形成创新，经 NIS 门限后
    // 更新非参考车平面修正量及协方差；被拒绝的测距不会改变滤波状态。
    const auto code =
        zju_coop_distributed_push_range(handle_, &packet, &result);
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "UWB packet rejected: %s",
                           zju_coop_error_string(code));
      return;
    }
    if (result.update_disposition != ZJU_COOP_UPDATE_ACCEPTED) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "UWB did not update the filter (processing=%d, update=%d, "
          "innovation=%.3f m, NIS=%.3f)",
          static_cast<int>(result.disposition),
          static_cast<int>(result.update_disposition), result.innovation_m,
          result.nis);
    }
  }

  // 按统一算法时刻读取只读快照，输出以参考车为原点、轴向平行公共 ENU 的结果；
  // 这里只平移原点，不随参考车航向旋转坐标轴。
  // position_valid/yaw_valid 由 SDK 根据状态新鲜度和近期有效测距连通性给出，
  // 无效时数值只用于诊断，规划控制端不能继续使用。
  void publish_pose() {
    const auto now_value = current_time_ns(steady_time_ns());
    if (now_value == 0U) {
      return;
    }
    zju_coop_pose2d_snapshot_v2_t snapshot{};
    if (zju_coop_pose2d_snapshot_v2_init(&snapshot) != ZJU_COOP_OK) {
      return;
    }
    std::vector<zju_coop_vehicle_pose2d_v2_t> vehicles(node_ids_.size());
    for (auto& vehicle : vehicles) {
      if (zju_coop_vehicle_pose2d_v2_init(&vehicle) != ZJU_COOP_OK) {
        return;
      }
    }
    std::uint32_t count{};
    const auto code = zju_coop_distributed_get_pose2d_v2(
        handle_, now_value, &snapshot,
        vehicles.data(), static_cast<std::uint32_t>(vehicles.size()),
        static_cast<std::uint32_t>(sizeof(vehicles.front())), &count);
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Pose2D unavailable: %s", zju_coop_error_string(code));
      return;
    }

    cooperative_localization_msgs::msg::CooperativePose2DArray output;
    output.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(snapshot.timestamp_ns), RCL_ROS_TIME);
    output.header.frame_id = snapshot.frame_id;
    output.reference_node_id = snapshot.reference_node_id;
    output.vehicles.reserve(count);
    for (std::uint32_t index = 0U; index < count; ++index) {
      cooperative_localization_msgs::msg::VehiclePose2D pose;
      pose.node_id = vehicles[index].node_id;
      pose.x_m = vehicles[index].x_m;
      pose.y_m = vehicles[index].y_m;
      pose.yaw_rad = vehicles[index].yaw_rad;
      pose.position_valid = vehicles[index].position_valid == ZJU_COOP_TRUE;
      pose.yaw_valid = vehicles[index].yaw_valid == ZJU_COOP_TRUE;
      output.vehicles.push_back(pose);
    }
    pose_publisher_->publish(output);
    if (feedback_publisher_ != nullptr) {
      feedback_publisher_->publish(output);
    }
  }

  // SDK 状态、允许车辆和每车输入顺序保护。
  zju_coop_distributed_handle_t* handle_{};
  std::vector<std::uint32_t> node_ids_;
  std::unordered_map<std::uint32_t, std::uint64_t>
      last_node_state_timestamp_ns_;
  std::uint32_t reference_node_id_{};
  std::uint64_t uwb_sequence_{};
  double range_std_m_{};
  bool use_sim_time_{};
  std::uint64_t max_future_skew_ns_{};
  std::uint64_t max_receive_delay_ns_{};
  std::string common_enu_frame_id_;
  std::string input_mode_;
  bool uwb_only_{};
  double minimum_triangle_height_m_{};
  std::uint64_t uwb_epoch_wait_ns_{};
  std::uint64_t uwb_epoch_timeout_ns_{};
  double uwb_reciprocal_max_difference_m_{};
  double uwb_maximum_range_rate_mps_{};
  double uwb_range_jump_tolerance_m_{};
  std::array<std::uint32_t, 3> canonical_node_ids_{};
  std::array<double, 3> latest_uwb_x_m_{};
  std::array<double, 3> latest_uwb_y_m_{};
  std::array<double, 3> last_accepted_uwb_distance_m_{};
  std::array<std::uint64_t, 3> last_accepted_uwb_timestamp_ns_{};
  std::uint64_t last_uwb_output_timestamp_ns_{};
  std::uint64_t last_uwb_retired_timestamp_ns_{};
  std::map<std::uint64_t, UwbEpoch> uwb_epochs_;
  // 仅由参考车本地 NodeState 驱动的 UWB 时间桥。
  zju_coop_ros2::SensorTimeBridge reference_time_bridge_;
  rclcpp::Subscription<cooperative_localization_msgs::msg::NodeState>::SharedPtr
      node_state_subscription_;
  rclcpp::Subscription<cooperative_interfaces::msg::UwbRange>::SharedPtr
      uwb_subscription_;
  rclcpp::Publisher<
      cooperative_localization_msgs::msg::CooperativePose2DArray>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<
      cooperative_localization_msgs::msg::CooperativePose2DArray>::SharedPtr
      feedback_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr uwb_epoch_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // 单线程执行保证 NodeState、UWB 和发布回调串行访问 SDK 句柄。
    rclcpp::spin(std::make_shared<CooperativeFusionNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("zju_cooperative_fusion_node"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

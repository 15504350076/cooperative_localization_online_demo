#include "cooperative_localization_msgs/msg/cooperative_pose2_d_array.hpp"
#include "cooperative_localization_msgs/msg/node_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_time_bridge.hpp"
#include "zju_coop/c_api.h"
#include "zju_coop_test_msgs/msg/uwb_range.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
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

void require_ok(zju_coop_error_code_t code, const char* operation) {
  if (code != ZJU_COOP_OK) {
    throw std::runtime_error(std::string(operation) + ": " +
                             zju_coop_error_string(code));
  }
}

std::uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         stamp.nanosec;
}

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
    const auto configured_ids =
        declare_parameter<std::vector<std::int64_t>>("node_ids", {1, 2, 3});
    const auto configured_reference =
        declare_parameter<std::int64_t>("reference_node_id", 1);
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);
    const bool enable_follower_feedback =
        declare_parameter<bool>("enable_follower_feedback", false);
    range_std_m_ = declare_parameter<double>("range_std_m", 0.10);
    const auto timeout_ms =
        declare_parameter<std::int64_t>("node_state_timeout_ms", 300);
    // Packet-age guards; neither value is a UWB synchronization-accuracy target.
    const auto max_future_skew_ms =
        declare_parameter<std::int64_t>("max_future_skew_ms", 100);
    const auto max_receive_delay_ms =
        declare_parameter<std::int64_t>("max_receive_delay_ms", 500);
    common_enu_frame_id_ =
        declare_parameter<std::string>("common_enu_frame_id", "common_enu");
    use_sim_time_ = get_parameter("use_sim_time").as_bool();

    if (configured_ids.size() < 2U || configured_reference <= 0 ||
        configured_reference > std::numeric_limits<std::uint16_t>::max() ||
        !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
        !std::isfinite(range_std_m_) || range_std_m_ <= 0.0 ||
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

    zju_coop_config_t config{};
    require_ok(zju_coop_config_init(&config), "zju_coop_config_init");
    config.reference_node_id = reference_node_id_;
    config.nodes = nodes.data();
    config.node_count = static_cast<std::uint32_t>(nodes.size());
    config.max_prediction_step_s = 0.10;
    config.edge_timeout_ns = static_cast<std::uint64_t>(timeout_ms) *
                             kNanosecondsPerMillisecond;
    max_future_skew_ns_ = static_cast<std::uint64_t>(max_future_skew_ms) *
                          kNanosecondsPerMillisecond;
    max_receive_delay_ns_ =
        static_cast<std::uint64_t>(max_receive_delay_ms) *
        kNanosecondsPerMillisecond;
    config.max_future_skew_ns = max_future_skew_ns_;
    config.max_receive_delay_ns = max_receive_delay_ns_;
    require_ok(zju_coop_distributed_create(&config, &handle_),
               "zju_coop_distributed_create");

    pose_publisher_ = create_publisher<
        cooperative_localization_msgs::msg::CooperativePose2DArray>(
        "poses_2d", rclcpp::QoS(1).reliable().durability_volatile());
    if (enable_follower_feedback) {
      feedback_publisher_ = create_publisher<
          cooperative_localization_msgs::msg::CooperativePose2DArray>(
          "feedback_poses_2d",
          rclcpp::QoS(1).reliable().durability_volatile());
    }
    node_state_subscription_ = create_subscription<
        cooperative_localization_msgs::msg::NodeState>(
        "node_state", rclcpp::QoS(5).best_effort().durability_volatile(),
        [this](cooperative_localization_msgs::msg::NodeState::ConstSharedPtr message) {
          on_node_state(*message);
        });
    uwb_subscription_ = create_subscription<zju_coop_test_msgs::msg::UwbRange>(
        "uwb_range", rclcpp::QoS(20).best_effort().durability_volatile(),
        [this](zju_coop_test_msgs::msg::UwbRange::ConstSharedPtr message) {
          on_uwb_range(*message);
        });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(period, [this]() { publish_pose(); });

    RCLCPP_INFO(get_logger(),
                "distributed fusion ready: %zu nodes, reference=%u, "
                "time_source=%s; timestamp guards future=%lld ms, "
                "receive-delay=%lld ms (data rejection, not sync accuracy)",
                node_ids_.size(), reference_node_id_,
                use_sim_time_ ? "ROS simulation clock"
                              : "reference NodeState UWB clock",
                static_cast<long long>(max_future_skew_ms),
                static_cast<long long>(max_receive_delay_ms));
  }

  ~CooperativeFusionNode() override {
    if (handle_ != nullptr) {
      static_cast<void>(zju_coop_distributed_destroy(handle_));
    }
  }

 private:
  std::uint64_t current_time_ns(std::uint64_t steady_now_ns) const {
    if (!use_sim_time_) {
      return reference_time_bridge_.current_time_ns(steady_now_ns);
    }
    const auto ros_time = now().nanoseconds();
    return ros_time > 0 ? static_cast<std::uint64_t>(ros_time) : 0U;
  }

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
      // The first local reference state is the only available UWB-time anchor.
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
      // Only an accepted local reference state may discipline the bridge.
      static_cast<void>(reference_time_bridge_.observe(
          measurement_time, steady_receive_time));
    }
  }

  void on_uwb_range(const zju_coop_test_msgs::msg::UwbRange& message) {
    if (message.src_id > std::numeric_limits<std::uint16_t>::max() ||
        message.target_id > std::numeric_limits<std::uint16_t>::max()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting UWB node id outside uint16 C ABI");
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
    packet.timestamp_ns = stamp_ns(message.header.stamp);
    packet.receive_timestamp_ns = receive_time;
    packet.range_m = message.distance;
    packet.range_std_m = range_std_m_;
    packet.valid = ZJU_COOP_TRUE;
    packet.status = ZJU_COOP_RANGE_STATUS_OK;
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
  zju_coop_ros2::SensorTimeBridge reference_time_bridge_;
  rclcpp::Subscription<cooperative_localization_msgs::msg::NodeState>::SharedPtr
      node_state_subscription_;
  rclcpp::Subscription<zju_coop_test_msgs::msg::UwbRange>::SharedPtr
      uwb_subscription_;
  rclcpp::Publisher<
      cooperative_localization_msgs::msg::CooperativePose2DArray>::SharedPtr
      pose_publisher_;
  rclcpp::Publisher<
      cooperative_localization_msgs::msg::CooperativePose2DArray>::SharedPtr
      feedback_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
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

// 参考车侧二维协同融合 ROS 2 适配节点。
// 数据流：各车 NodeState + 车间 UWB 测距 -> SDK 分布式二维修正器 -> 三车相对位姿。
// 滤波状态只含各非参考车的 [delta_e, delta_n]；各车完整惯导状态仍留在本机。
// UWB 修正相对平面位置，不在本节点估计姿态或 IMU 零偏。
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
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 10.0);
    const bool enable_follower_feedback =
        declare_parameter<bool>("enable_follower_feedback", false);
    range_std_m_ = declare_parameter<double>("range_std_m", 0.10);
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

    // 创建参考车上的分布式修正器。max_prediction_step_s 限制一次外推跨度，
    // edge_timeout_ns 决定一条测距边多久后不再支撑 position_valid。
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

  // 接收临时 UwbRange 测试消息，补充配置给出的统一测距标准差后送入滤波器。
  // 正式硬件消息若提供质量/方差/状态，需要在此处按真实字段填写，不应固定为 OK。
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

  // 按统一算法时刻读取一次只读快照，并转换成参考车坐标系下的 ROS 2 输出。
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
  // 仅由参考车本地 NodeState 驱动的 UWB 时间桥。
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

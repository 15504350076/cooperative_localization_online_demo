// 本车侧惯导 ROS 2 适配节点。
// 数据流：sensor_msgs/Imu -> SDK 单车 15 维 ESKF 传播 -> NodeState。
// NodeState 只发送公共 ENU 下的位置、速度和姿态；零偏及 15x15 协方差留在本机。
// 点云和图像是默认关闭的预留输入，当前仅完成格式/时间校验，不参与状态更新。
#include "cooperative_localization_msgs/msg/node_state.hpp"
#include "imu_input_transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_time_bridge.hpp"
#include "zju_coop/c_api.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000ULL;
// 只用于拒绝明显超前或滞后的数据包，不代表 UWB 授时精度指标。
constexpr std::uint64_t kMaxImuFutureSkewNs = 100'000'000ULL;
constexpr std::uint64_t kMaxImuReceiveDelayNs = 500'000'000ULL;

// SDK C 接口在初始化阶段失败时直接终止节点，避免带着半初始化状态运行。
void require_ok(zju_coop_error_code_t code, const char* operation) {
  if (code != ZJU_COOP_OK) {
    throw std::runtime_error(std::string(operation) + ": " +
                             zju_coop_error_string(code));
  }
}

// ROS 时间戳统一转换成无符号纳秒；0 作为无效时间哨兵。
std::uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    return 0U;
  }
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         stamp.nanosec;
}

// 单调时钟只计算本机经过时间，不与 UWB_SYSTEM_TIME 直接比较。
std::uint64_t steady_time_ns() {
  const auto count = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch())
                         .count();
  return count > 0 ? static_cast<std::uint64_t>(count) : 0U;
}

// 声明定长向量参数，并在启动期一次性检查维数和有限性。
std::vector<double> vector_parameter(rclcpp::Node& node, const char* name,
                                     std::vector<double> defaults,
                                     std::size_t expected_size) {
  const auto value = node.declare_parameter<std::vector<double>>(name, defaults);
  if (value.size() != expected_size ||
      !std::all_of(value.begin(), value.end(),
                   [](double item) { return std::isfinite(item); })) {
    throw std::invalid_argument(std::string(name) +
                                " must contain finite values with the required length");
  }
  return value;
}

template <std::size_t Size>
void copy_values(double (&destination)[Size], const std::vector<double>& source) {
  std::copy_n(source.begin(), Size, destination);
}

// C ABI 使用定长字符数组；拒绝空字符串和会被截断的内容。
template <std::size_t Capacity>
bool copy_c_string(char (&destination)[Capacity], const std::string& source) {
  if (source.empty() || source.size() >= Capacity) {
    return false;
  }
  std::memcpy(destination, source.c_str(), source.size() + 1U);
  return true;
}

class LocalInertialNode final : public rclcpp::Node {
 public:
  LocalInertialNode()
      : Node("zju_local_inertial_node") {
    // 必选项给出车辆身份和惯导初值；点云、相机开关默认关闭。
    // 话题使用相对名称，实际三车话题由 namespace/remap 区分。
    const auto configured_node_id = declare_parameter<std::int64_t>("node_id", 0);
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 20.0);
    expected_imu_frame_id_ =
        declare_parameter<std::string>("expected_imu_frame_id", "imu_link");
    common_enu_frame_id_ =
        declare_parameter<std::string>("common_enu_frame_id", "common_enu");
    const auto sensor_to_flu = vector_parameter(
        *this, "imu_sensor_to_flu_xyzw", {0.0, 0.0, 0.0, 1.0}, 4U);
    const double gyro_scale_to_rad_s =
        declare_parameter<double>("gyro_scale_to_rad_s", 1.0);
    const double accel_scale_to_m_s2 =
        declare_parameter<double>("accel_scale_to_m_s2", 1.0);
    const bool enable_point_cloud_input =
        declare_parameter<bool>("enable_point_cloud_input", false);
    const bool enable_camera_input =
        declare_parameter<bool>("enable_camera_input", false);
    const auto configured_point_cloud_sensor_id =
        declare_parameter<std::int64_t>("point_cloud_sensor_id", 1);
    const auto configured_camera_id =
        declare_parameter<std::int64_t>("camera_id", 1);
    point_cloud_frame_alias_ =
        declare_parameter<std::string>("point_cloud_frame_alias", "");
    camera_frame_alias_ =
        declare_parameter<std::string>("camera_frame_alias", "");
    use_sim_time_ = get_parameter("use_sim_time").as_bool();

    // node_id 受当前 UWB 协议 uint16 编号限制；传感器 id 使用 uint32。
    // frame_id 还必须放入 SDK C ABI 的 32 字节数组。
    if (configured_node_id <= 0 ||
        configured_node_id > std::numeric_limits<std::uint16_t>::max() ||
        configured_point_cloud_sensor_id <= 0 ||
        configured_point_cloud_sensor_id >
            std::numeric_limits<std::uint32_t>::max() ||
        configured_camera_id <= 0 ||
        configured_camera_id > std::numeric_limits<std::uint32_t>::max() ||
        !std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0 ||
        expected_imu_frame_id_.empty() ||
        expected_imu_frame_id_.size() >= 32U || common_enu_frame_id_.empty()) {
      throw std::invalid_argument("invalid local inertial node parameters");
    }
    if (point_cloud_frame_alias_.size() >= 32U ||
        camera_frame_alias_.size() >= 32U) {
      throw std::invalid_argument("raw input frame aliases must fit the C ABI");
    }
    node_id_ = static_cast<std::uint32_t>(configured_node_id);
    point_cloud_sensor_id_ =
        static_cast<std::uint32_t>(configured_point_cloud_sensor_id);
    camera_id_ = static_cast<std::uint32_t>(configured_camera_id);
    std::array<double, 4> sensor_to_flu_xyzw{};
    std::copy_n(sensor_to_flu.begin(), sensor_to_flu_xyzw.size(),
                sensor_to_flu_xyzw.begin());
    imu_input_transform_ = zju_coop_ros2::ImuInputTransform(
        sensor_to_flu_xyzw, gyro_scale_to_rad_s, accel_scale_to_m_s2);

    const auto position = vector_parameter(
        *this, "initial_position_enu_m", {}, 3U);
    const auto velocity = vector_parameter(
        *this, "initial_velocity_enu_mps", {}, 3U);
    const auto orientation = vector_parameter(
        *this, "initial_orientation_flu_to_enu_xyzw",
        {}, 4U);
    const auto gyro_bias = vector_parameter(
        *this, "initial_gyro_bias_flu_rad_s", {0.0, 0.0, 0.0}, 3U);
    const auto accel_bias = vector_parameter(
        *this, "initial_accel_bias_flu_m_s2", {0.0, 0.0, 0.0}, 3U);

    // SDK 要求 FLU->ENU 四元数已归一化，不在节点内静默修正错误初值。
    const double orientation_norm =
        std::sqrt(orientation[0] * orientation[0] +
                  orientation[1] * orientation[1] +
                  orientation[2] * orientation[2] +
                  orientation[3] * orientation[3]);
    if (!std::isfinite(orientation_norm) ||
        std::abs(orientation_norm - 1.0) > 1.0e-3) {
      throw std::invalid_argument(
          "initial_orientation_flu_to_enu_xyzw must be a unit quaternion");
    }

    // 先建立只含本车的 SDK 基础实例。本节点不是三车集中式滤波器：
    // reference_node_id 设成本车，仅用于满足单车 SDK 实例的基础配置。
    zju_coop_node_initialization_t base_node{};
    require_ok(zju_coop_node_initialization_init(&base_node),
               "zju_coop_node_initialization_init");
    base_node.node_id = node_id_;
    base_node.x = position[0];
    base_node.y = position[1];
    base_node.vx = velocity[0];
    base_node.vy = velocity[1];

    zju_coop_config_t base_config{};
    require_ok(zju_coop_config_init(&base_config), "zju_coop_config_init");
    base_config.reference_node_id = node_id_;
    base_config.nodes = &base_node;
    base_config.node_count = 1U;

    zju_coop_handle_t* candidate{};
    require_ok(zju_coop_create(&base_config, &candidate), "zju_coop_create");
    handle_ = candidate;

    try {
      // 给单车惯导装载名义状态 p/v/q/bg/ba。误差状态和协方差由 SDK 持有，
      // 后续每个有效 IMU 样本通过 zju_coop_push_imu 推进。
      zju_coop_inertial_node_initialization_t initial{};
      require_ok(zju_coop_inertial_node_initialization_init(&initial),
                 "zju_coop_inertial_node_initialization_init");
      initial.node_id = node_id_;
      copy_values(initial.position_n_m, position);
      copy_values(initial.velocity_n_mps, velocity);
      copy_values(initial.orientation_xyzw, orientation);
      copy_values(initial.gyro_bias_rad_s, gyro_bias);
      copy_values(initial.accel_bias_m_s2, accel_bias);

      zju_coop_inertial_config_t inertial_config{};
      require_ok(zju_coop_inertial_config_init(&inertial_config),
                 "zju_coop_inertial_config_init");
      inertial_config.nodes = &initial;
      inertial_config.node_count = 1U;
      std::memcpy(inertial_config.expected_frame_id,
                  expected_imu_frame_id_.c_str(),
                  expected_imu_frame_id_.size() + 1U);
      require_ok(zju_coop_configure_inertial(handle_, &inertial_config),
                 "zju_coop_configure_inertial");
    } catch (...) {
      static_cast<void>(zju_coop_destroy(handle_));
      handle_ = nullptr;
      throw;
    }

    // IMU/NodeState 属于高频实时数据，使用小队列和 best-effort，避免旧样本积压。
    // 点云/图像只保留最新一帧；未打开参数时不会创建对应订阅者。
    node_state_publisher_ =
        create_publisher<cooperative_localization_msgs::msg::NodeState>(
            "node_state", rclcpp::QoS(5).best_effort().durability_volatile());
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          on_imu(*message);
        });
    if (enable_point_cloud_input) {
      point_cloud_subscription_ =
          create_subscription<sensor_msgs::msg::PointCloud2>(
              "point_cloud", rclcpp::SensorDataQoS().keep_last(1),
              [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                on_point_cloud(*message);
              });
    }
    if (enable_camera_input) {
      camera_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
          "camera_image", rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
            on_camera_image(*message);
          });
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(period, [this]() { publish_state(); });

    RCLCPP_INFO(
        get_logger(),
        "local INS ready: node_id=%u, imu frame=%s, gyro_scale=%.9g, "
        "accel_scale=%.9g, point_cloud=%s, camera=%s",
        node_id_, expected_imu_frame_id_.c_str(),
        gyro_scale_to_rad_s, accel_scale_to_m_s2,
        enable_point_cloud_input ? "enabled" : "disabled",
        enable_camera_input ? "enabled" : "disabled");
  }

  ~LocalInertialNode() override {
    if (handle_ != nullptr) {
      static_cast<void>(zju_coop_destroy(handle_));
    }
  }

 private:
  // 实盒：把本机 steady 到达时刻映射到传感器头中的 UWB 时间域。
  // Gazebo：所有消息和 /clock 已在同一仿真时间域，直接使用 ROS 时间。
  std::uint64_t receive_time_ns(
      std::uint64_t measurement_time_ns,
      std::uint64_t steady_receive_time_ns,
      const zju_coop_ros2::SensorTimeBridge& bridge) {
    if (!use_sim_time_) {
      return bridge.receive_time_ns(measurement_time_ns,
                                    steady_receive_time_ns);
    }
    const auto ros_time = now().nanoseconds();
    return ros_time > 0 ? static_cast<std::uint64_t>(ros_time) : 0U;
  }

  // 运行中时间纪元变化或采样间断后，纯惯导无法补回丢失的运动。
  // 故障一旦锁存，本进程停止解算和发布，必须重启以重新建立完整时间基线。
  void latch_imu_time_fault(const char* reason) {
    if (imu_time_faulted_) {
      return;
    }
    imu_time_faulted_ = true;
    RCLCPP_ERROR(get_logger(),
                 "IMU time fault latched (%s); restart the local inertial "
                 "node after fixing the upstream clock",
                 reason);
  }

  // 接收一帧本车 IMU，校验时间顺序后转换成稳定 C ABI 数据包并推进惯导。
  void on_imu(const sensor_msgs::msg::Imu& message) {
    if (imu_time_faulted_) {
      return;
    }
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const std::uint64_t steady_receive_time_ns = steady_time_ns();
    const std::uint64_t receive_time = receive_time_ns(
        measurement_time_ns, steady_receive_time_ns, imu_time_bridge_);
    if (measurement_time_ns == 0U || receive_time == 0U) {
      if (last_imu_measurement_time_ns_ != 0U) {
        latch_imu_time_fault("timestamp reset to zero/invalid");
        return;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting IMU with zero/invalid timestamp");
      return;
    }
    if (last_imu_measurement_time_ns_ != 0U &&
        measurement_time_ns <= last_imu_measurement_time_ns_) {
      if (measurement_time_ns < last_imu_measurement_time_ns_) {
        latch_imu_time_fault("timestamp rollback");
      }
      // 相等时间戳只是重复帧，丢弃即可，不破坏已有时间基线。
      return;
    }
    if (!use_sim_time_ && !imu_time_bridge_.measurement_is_plausible(
            measurement_time_ns, steady_receive_time_ns,
            kMaxImuFutureSkewNs, kMaxImuReceiveDelayNs)) {
      latch_imu_time_fault("timestamp outside established UWB time window");
      return;
    }

    zju_coop_imu_packet_t packet{};
    if (zju_coop_imu_packet_init(&packet) != ZJU_COOP_OK) {
      return;
    }
    packet.node_id = node_id_;
    packet.sequence = ++imu_sequence_;
    packet.timestamp_ns = measurement_time_ns;
    packet.receive_timestamp_ns = receive_time;
    const auto angular_velocity = imu_input_transform_.angular_velocity(
        {message.angular_velocity.x, message.angular_velocity.y,
         message.angular_velocity.z});
    const auto linear_acceleration = imu_input_transform_.linear_acceleration(
        {message.linear_acceleration.x, message.linear_acceleration.y,
         message.linear_acceleration.z});
    const auto orientation_covariance =
        imu_input_transform_.orientation_covariance(
            message.orientation_covariance);
    const auto angular_velocity_covariance =
        imu_input_transform_.angular_velocity_covariance(
            message.angular_velocity_covariance);
    const auto linear_acceleration_covariance =
        imu_input_transform_.linear_acceleration_covariance(
            message.linear_acceleration_covariance);
    std::copy(angular_velocity.begin(), angular_velocity.end(),
              packet.angular_velocity_rad_s);
    std::copy(linear_acceleration.begin(), linear_acceleration.end(),
              packet.linear_acceleration_m_s2);
    std::copy(orientation_covariance.begin(), orientation_covariance.end(),
              packet.orientation_covariance);
    std::copy(angular_velocity_covariance.begin(),
              angular_velocity_covariance.end(),
              packet.angular_velocity_covariance);
    std::copy(linear_acceleration_covariance.begin(),
              linear_acceleration_covariance.end(),
              packet.linear_acceleration_covariance);

    // ROS Imu 约定 covariance[0] < 0 表示未提供姿态。
    // 只有声明可用且四元数归一化时，才把姿态观测交给 SDK。
    const auto orientation =
        imu_input_transform_.orientation_flu_to_navigation(
            {message.orientation.x, message.orientation.y,
             message.orientation.z, message.orientation.w});
    const double orientation_norm =
        std::sqrt(orientation[0] * orientation[0] +
                  orientation[1] * orientation[1] +
                  orientation[2] * orientation[2] +
                  orientation[3] * orientation[3]);
    if (message.orientation_covariance[0] >= 0.0 &&
        std::isfinite(orientation_norm) &&
        std::abs(orientation_norm - 1.0) <= 1.0e-3) {
      std::copy(orientation.begin(), orientation.end(),
                packet.orientation_xyzw);
      packet.orientation_valid = ZJU_COOP_TRUE;
    }
    if (message.header.frame_id.size() >= sizeof(packet.frame_id)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting IMU frame_id longer than C ABI capacity");
      return;
    }
    std::memcpy(packet.frame_id, message.header.frame_id.c_str(),
                message.header.frame_id.size() + 1U);
    packet.valid = ZJU_COOP_TRUE;

    // 第一个 IMU 样本只建立积分基线；从第二个合格样本开始进行传播。
    zju_coop_imu_processing_result_t result{};
    if (zju_coop_imu_processing_result_init(&result) != ZJU_COOP_OK) {
      return;
    }
    const auto code = zju_coop_push_imu(handle_, &packet, &result);
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "IMU rejected at C boundary: %s",
                           zju_coop_error_string(code));
      return;
    }
    if (result.disposition == ZJU_COOP_IMU_INTERVAL_REJECTED) {
      latch_imu_time_fault("integration interval rejected");
      return;
    }
    if (result.disposition != ZJU_COOP_IMU_BASELINE_ESTABLISHED &&
        result.disposition != ZJU_COOP_IMU_PROPAGATED) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU sample was not consumed (disposition=%u)",
          static_cast<unsigned int>(result.disposition));
    }
    if (result.disposition == ZJU_COOP_IMU_BASELINE_ESTABLISHED ||
        result.disposition == ZJU_COOP_IMU_PROPAGATED) {
      last_imu_measurement_time_ns_ = measurement_time_ns;
      if (!use_sim_time_) {
        static_cast<void>(imu_time_bridge_.observe(
            measurement_time_ns, steady_receive_time_ns));
      }
    }
  }

  // 可选点云入口。完整保留 PointCloud2 字段布局和原始字节，不做点类型假设。
  // 当前 SDK 预期返回 VALIDATED_NOT_USED，因此这里不会改变惯导状态。
  void on_point_cloud(const sensor_msgs::msg::PointCloud2& message) {
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const std::uint64_t steady_receive_time_ns = steady_time_ns();
    const std::uint64_t receive_time = receive_time_ns(
        measurement_time_ns, steady_receive_time_ns,
        point_cloud_time_bridge_);
    if (measurement_time_ns == 0U || receive_time == 0U ||
        message.fields.size() > std::numeric_limits<std::uint32_t>::max()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting point cloud with invalid timestamp or fields");
      return;
    }

    // 将 ROS 字段描述转换为 C ABI；字段数组和 data 指针只在同步 push 调用期间使用。
    std::vector<zju_coop_point_field_t> fields(message.fields.size());
    for (std::size_t index = 0U; index < message.fields.size(); ++index) {
      if (zju_coop_point_field_init(&fields[index]) != ZJU_COOP_OK ||
          !copy_c_string(fields[index].name, message.fields[index].name)) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "rejecting point cloud with an invalid field name");
        return;
      }
      fields[index].offset = message.fields[index].offset;
      fields[index].datatype =
          static_cast<zju_coop_point_field_datatype_t>(
              message.fields[index].datatype);
      fields[index].count = message.fields[index].count;
    }

    zju_coop_point_cloud_packet_t packet{};
    const auto& packet_frame = point_cloud_frame_alias_.empty()
                                   ? message.header.frame_id
                                   : point_cloud_frame_alias_;
    // frame alias 用于适配上游过长或不符合约定的 frame_id，不修改原消息。
    if (zju_coop_point_cloud_packet_init(&packet) != ZJU_COOP_OK ||
        !copy_c_string(packet.frame_id, packet_frame)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "rejecting point cloud frame_id longer than C ABI capacity");
      return;
    }
    packet.node_id = node_id_;
    packet.sensor_id = point_cloud_sensor_id_;
    packet.sequence = ++point_cloud_sequence_;
    packet.timestamp_ns = measurement_time_ns;
    packet.receive_timestamp_ns = receive_time;
    packet.height = message.height;
    packet.width = message.width;
    packet.fields = fields.data();
    packet.field_count = static_cast<std::uint32_t>(fields.size());
    packet.field_stride = sizeof(zju_coop_point_field_t);
    packet.point_step = message.point_step;
    packet.row_step = message.row_step;
    packet.data = message.data.data();
    packet.data_size = message.data.size();
    packet.is_bigendian =
        message.is_bigendian ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    packet.is_dense = message.is_dense ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    packet.valid = ZJU_COOP_TRUE;
    packet.status = ZJU_COOP_RANGE_STATUS_OK;

    zju_coop_raw_input_result_t result{};
    if (zju_coop_raw_input_result_init(&result) != ZJU_COOP_OK) {
      return;
    }
    const auto code = zju_coop_push_point_cloud(handle_, &packet, &result);
    if (code != ZJU_COOP_OK ||
        result.input_type != ZJU_COOP_RAW_INPUT_POINT_CLOUD ||
        result.disposition != ZJU_COOP_RAW_INPUT_VALIDATED_NOT_USED ||
        result.node_id != node_id_ ||
        result.sensor_id != point_cloud_sensor_id_ ||
        result.sequence != packet.sequence ||
        result.timestamp_ns != packet.timestamp_ns) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "point cloud input rejected or returned an inconsistent result "
          "(code=%s, disposition=%u)",
          zju_coop_error_string(code),
          static_cast<unsigned int>(result.disposition));
      return;
    }
    if (!use_sim_time_) {
      static_cast<void>(point_cloud_time_bridge_.observe(
          measurement_time_ns, steady_receive_time_ns));
    }
    if (!point_cloud_input_seen_) {
      RCLCPP_INFO(get_logger(), "point cloud input: VALIDATED_NOT_USED");
      point_cloud_input_seen_ = true;
    }
  }

  // 可选图像入口。保留 encoding/step/原始字节；当前同样只校验、不参与定位。
  void on_camera_image(const sensor_msgs::msg::Image& message) {
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const std::uint64_t steady_receive_time_ns = steady_time_ns();
    const std::uint64_t receive_time = receive_time_ns(
        measurement_time_ns, steady_receive_time_ns, camera_time_bridge_);
    if (measurement_time_ns == 0U || receive_time == 0U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting camera image with invalid timestamp");
      return;
    }

    zju_coop_camera_image_packet_t packet{};
    const auto& packet_frame = camera_frame_alias_.empty()
                                   ? message.header.frame_id
                                   : camera_frame_alias_;
    if (zju_coop_camera_image_packet_init(&packet) != ZJU_COOP_OK ||
        !copy_c_string(packet.frame_id, packet_frame) ||
        !copy_c_string(packet.encoding, message.encoding)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "rejecting camera frame_id or encoding longer than C ABI capacity");
      return;
    }
    packet.node_id = node_id_;
    packet.camera_id = camera_id_;
    packet.sequence = ++camera_sequence_;
    packet.timestamp_ns = measurement_time_ns;
    packet.receive_timestamp_ns = receive_time;
    packet.height = message.height;
    packet.width = message.width;
    packet.step = message.step;
    packet.data = message.data.data();
    packet.data_size = message.data.size();
    packet.is_bigendian =
        message.is_bigendian != 0U ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    packet.valid = ZJU_COOP_TRUE;
    packet.status = ZJU_COOP_RANGE_STATUS_OK;

    zju_coop_raw_input_result_t result{};
    if (zju_coop_raw_input_result_init(&result) != ZJU_COOP_OK) {
      return;
    }
    const auto code = zju_coop_push_camera_image(handle_, &packet, &result);
    if (code != ZJU_COOP_OK ||
        result.input_type != ZJU_COOP_RAW_INPUT_CAMERA_IMAGE ||
        result.disposition != ZJU_COOP_RAW_INPUT_VALIDATED_NOT_USED ||
        result.node_id != node_id_ || result.sensor_id != camera_id_ ||
        result.sequence != packet.sequence ||
        result.timestamp_ns != packet.timestamp_ns) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "camera input rejected or returned an inconsistent result "
          "(code=%s, disposition=%u)",
          zju_coop_error_string(code),
          static_cast<unsigned int>(result.disposition));
      return;
    }
    if (!use_sim_time_) {
      static_cast<void>(camera_time_bridge_.observe(
          measurement_time_ns, steady_receive_time_ns));
    }
    if (!camera_input_seen_) {
      RCLCPP_INFO(get_logger(), "camera input: VALIDATED_NOT_USED");
      camera_input_seen_ = true;
    }
  }

  // 以固定频率读取 SDK 最新状态；仅在出现新的 IMU 状态时发布一次 NodeState。
  // header.stamp 是该状态最后接受的 IMU 样本时刻，不是定时器触发时刻。
  void publish_state() {
    if (imu_time_faulted_) {
      return;
    }
    zju_coop_node_state_t state{};
    if (zju_coop_node_state_init(&state) != ZJU_COOP_OK) {
      return;
    }
    const auto code = zju_coop_get_node_state(handle_, node_id_, &state);
    if (code == ZJU_COOP_NOT_READY ||
        (code == ZJU_COOP_OK && state.valid != ZJU_COOP_TRUE)) {
      return;
    }
    if (code != ZJU_COOP_OK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "local state unavailable: %s",
                           zju_coop_error_string(code));
      return;
    }
    if (state.timestamp_ns == last_published_timestamp_ns_) {
      return;
    }

    cooperative_localization_msgs::msg::NodeState output;
    output.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(state.timestamp_ns), RCL_ROS_TIME);
    output.header.frame_id = common_enu_frame_id_;
    output.node_id = state.node_id;
    std::copy(std::begin(state.position_enu_m),
              std::end(state.position_enu_m), output.position_enu_m.begin());
    std::copy(std::begin(state.velocity_enu_mps),
              std::end(state.velocity_enu_mps), output.velocity_enu_mps.begin());
    std::copy(std::begin(state.orientation_flu_to_enu_xyzw),
              std::end(state.orientation_flu_to_enu_xyzw),
              output.orientation_flu_to_enu_xyzw.begin());
    output.valid = true;
    node_state_publisher_->publish(output);
    last_published_timestamp_ns_ = state.timestamp_ns;
  }

  // SDK 生命周期与车辆/传感器标识。
  zju_coop_handle_t* handle_{};
  std::uint32_t node_id_{};
  std::uint32_t point_cloud_sensor_id_{1U};
  std::uint32_t camera_id_{1U};
  std::uint64_t imu_sequence_{};
  std::uint64_t point_cloud_sequence_{};
  std::uint64_t camera_sequence_{};
  // 输入顺序和重复发布保护。
  std::uint64_t last_imu_measurement_time_ns_{};
  std::uint64_t last_published_timestamp_ns_{};
  bool point_cloud_input_seen_{};
  bool camera_input_seen_{};
  bool use_sim_time_{};
  bool imu_time_faulted_{};
  std::string expected_imu_frame_id_;
  std::string common_enu_frame_id_;
  std::string point_cloud_frame_alias_;
  std::string camera_frame_alias_;
  // 不同传感器可能有独立驱动延迟，分别维护时间桥，避免相互污染。
  zju_coop_ros2::SensorTimeBridge imu_time_bridge_;
  zju_coop_ros2::ImuInputTransform imu_input_transform_;
  zju_coop_ros2::SensorTimeBridge point_cloud_time_bridge_;
  zju_coop_ros2::SensorTimeBridge camera_time_bridge_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      camera_image_subscription_;
  rclcpp::Publisher<cooperative_localization_msgs::msg::NodeState>::SharedPtr
      node_state_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // 单线程 spin 使回调顺序确定；当前成员状态无需额外加锁。
    rclcpp::spin(std::make_shared<LocalInertialNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("zju_local_inertial_node"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

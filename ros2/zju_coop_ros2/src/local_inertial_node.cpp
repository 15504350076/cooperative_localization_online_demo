#include "cooperative_localization_msgs/msg/node_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
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
    const auto configured_node_id = declare_parameter<std::int64_t>("node_id", 0);
    const double publish_rate_hz =
        declare_parameter<double>("publish_rate_hz", 20.0);
    expected_imu_frame_id_ =
        declare_parameter<std::string>("expected_imu_frame_id", "imu_link");
    common_enu_frame_id_ =
        declare_parameter<std::string>("common_enu_frame_id", "common_enu");
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
        "local INS ready: node_id=%u, imu frame=%s, point_cloud=%s, camera=%s",
        node_id_, expected_imu_frame_id_.c_str(),
        enable_point_cloud_input ? "enabled" : "disabled",
        enable_camera_input ? "enabled" : "disabled");
  }

  ~LocalInertialNode() override {
    if (handle_ != nullptr) {
      static_cast<void>(zju_coop_destroy(handle_));
    }
  }

 private:
  void on_imu(const sensor_msgs::msg::Imu& message) {
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const auto now_value = now().nanoseconds();
    if (measurement_time_ns == 0U || now_value <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting IMU with zero/invalid timestamp");
      return;
    }

    zju_coop_imu_packet_t packet{};
    if (zju_coop_imu_packet_init(&packet) != ZJU_COOP_OK) {
      return;
    }
    packet.node_id = node_id_;
    packet.sequence = ++imu_sequence_;
    packet.timestamp_ns = measurement_time_ns;
    packet.receive_timestamp_ns = static_cast<std::uint64_t>(now_value);
    packet.angular_velocity_rad_s[0] = message.angular_velocity.x;
    packet.angular_velocity_rad_s[1] = message.angular_velocity.y;
    packet.angular_velocity_rad_s[2] = message.angular_velocity.z;
    packet.linear_acceleration_m_s2[0] = message.linear_acceleration.x;
    packet.linear_acceleration_m_s2[1] = message.linear_acceleration.y;
    packet.linear_acceleration_m_s2[2] = message.linear_acceleration.z;
    std::copy(message.orientation_covariance.begin(),
              message.orientation_covariance.end(),
              packet.orientation_covariance);
    std::copy(message.angular_velocity_covariance.begin(),
              message.angular_velocity_covariance.end(),
              packet.angular_velocity_covariance);
    std::copy(message.linear_acceleration_covariance.begin(),
              message.linear_acceleration_covariance.end(),
              packet.linear_acceleration_covariance);

    const double orientation_norm =
        std::sqrt(message.orientation.x * message.orientation.x +
                  message.orientation.y * message.orientation.y +
                  message.orientation.z * message.orientation.z +
                  message.orientation.w * message.orientation.w);
    if (message.orientation_covariance[0] >= 0.0 &&
        std::isfinite(orientation_norm) &&
        std::abs(orientation_norm - 1.0) <= 1.0e-3) {
      packet.orientation_xyzw[0] = message.orientation.x;
      packet.orientation_xyzw[1] = message.orientation.y;
      packet.orientation_xyzw[2] = message.orientation.z;
      packet.orientation_xyzw[3] = message.orientation.w;
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
    if (result.disposition != ZJU_COOP_IMU_BASELINE_ESTABLISHED &&
        result.disposition != ZJU_COOP_IMU_PROPAGATED) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "IMU sample was not consumed (disposition=%u)",
          static_cast<unsigned int>(result.disposition));
    }
  }

  void on_point_cloud(const sensor_msgs::msg::PointCloud2& message) {
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const auto now_value = now().nanoseconds();
    if (measurement_time_ns == 0U || now_value <= 0 ||
        message.fields.size() > std::numeric_limits<std::uint32_t>::max()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "rejecting point cloud with invalid timestamp or fields");
      return;
    }

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
    packet.receive_timestamp_ns = static_cast<std::uint64_t>(now_value);
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
    if (!point_cloud_input_seen_) {
      RCLCPP_INFO(get_logger(), "point cloud input: VALIDATED_NOT_USED");
      point_cloud_input_seen_ = true;
    }
  }

  void on_camera_image(const sensor_msgs::msg::Image& message) {
    const std::uint64_t measurement_time_ns = stamp_ns(message.header.stamp);
    const auto now_value = now().nanoseconds();
    if (measurement_time_ns == 0U || now_value <= 0) {
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
    packet.receive_timestamp_ns = static_cast<std::uint64_t>(now_value);
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
    if (!camera_input_seen_) {
      RCLCPP_INFO(get_logger(), "camera input: VALIDATED_NOT_USED");
      camera_input_seen_ = true;
    }
  }

  void publish_state() {
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

  zju_coop_handle_t* handle_{};
  std::uint32_t node_id_{};
  std::uint32_t point_cloud_sensor_id_{1U};
  std::uint32_t camera_id_{1U};
  std::uint64_t imu_sequence_{};
  std::uint64_t point_cloud_sequence_{};
  std::uint64_t camera_sequence_{};
  std::uint64_t last_published_timestamp_ns_{};
  bool point_cloud_input_seen_{};
  bool camera_input_seen_{};
  std::string expected_imu_frame_id_;
  std::string common_enu_frame_id_;
  std::string point_cloud_frame_alias_;
  std::string camera_frame_alias_;
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

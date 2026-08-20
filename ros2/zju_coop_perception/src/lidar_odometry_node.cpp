#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kUnknownVariance = 1.0e6;

double normalize_yaw(double yaw) {
  while (yaw >= kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

bool has_xyz_fields(const sensor_msgs::msg::PointCloud2& message) {
  std::array<bool, 3> found{};
  for (const auto& field : message.fields) {
    if (field.name == "x") {
      found[0] = true;
    } else if (field.name == "y") {
      found[1] = true;
    } else if (field.name == "z") {
      found[2] = true;
    }
  }
  return std::all_of(found.begin(), found.end(), [](bool value) {
    return value;
  });
}

std::int64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

class LidarOdometryNode final : public rclcpp::Node {
 public:
  LidarOdometryNode() : Node("zju_lidar_odometry_node") {
    odom_frame_id_ = declare_parameter<std::string>(
        "odom_frame_id", "lidar_odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    min_range_m_ = declare_parameter<double>("min_range_m", 0.25);
    max_range_m_ = declare_parameter<double>("max_range_m", 30.0);
    minimum_points_ = declare_parameter<std::int64_t>("minimum_points", 30);
    minimum_geometry_variance_m2_ = declare_parameter<double>(
        "minimum_geometry_variance_m2", 0.01);
    minimum_geometry_ratio_ = declare_parameter<double>(
        "minimum_geometry_ratio", 0.02);
    max_correspondence_distance_m_ = declare_parameter<double>(
        "max_correspondence_distance_m", 0.5);
    maximum_iterations_ = declare_parameter<std::int64_t>(
        "maximum_iterations", 40);
    max_fitness_score_m2_ = declare_parameter<double>(
        "max_fitness_score_m2", 0.04);
    max_translation_per_scan_m_ = declare_parameter<double>(
        "max_translation_per_scan_m", 0.5);
    max_yaw_per_scan_rad_ = declare_parameter<double>(
        "max_yaw_per_scan_rad", 0.35);

    validate_parameters();

    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_m_);
    icp_.setMaximumIterations(static_cast<int>(maximum_iterations_));
    icp_.setTransformationEpsilon(1.0e-8);
    icp_.setEuclideanFitnessEpsilon(1.0e-8);
    auto planar_estimator = std::make_shared<
        pcl::registration::TransformationEstimation2D<Point, Point>>();
    icp_.setTransformationEstimation(planar_estimator);

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
        "lidar_odometry", rclcpp::QoS(5).reliable());
    if (publish_tf_) {
      transform_broadcaster_ =
          std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    point_cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", rclcpp::SensorDataQoS().keep_last(1),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
              on_point_cloud(*message);
            });

    RCLCPP_INFO(get_logger(),
                "planar lidar odometry ready: input=point_cloud, "
                "output=lidar_odometry");
  }

 private:
  void validate_parameters() const {
    const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
    if (odom_frame_id_.empty()) {
      throw std::invalid_argument("odom_frame_id must not be empty");
    }
    if (!finite_positive(min_range_m_) ||
        !finite_positive(max_range_m_) || min_range_m_ >= max_range_m_) {
      throw std::invalid_argument("invalid lidar range limits");
    }
    if (minimum_points_ < 3 ||
        minimum_points_ > std::numeric_limits<int>::max()) {
      throw std::invalid_argument("minimum_points must be in [3, INT_MAX]");
    }
    if (!finite_positive(minimum_geometry_variance_m2_) ||
        !finite_positive(minimum_geometry_ratio_) ||
        minimum_geometry_ratio_ > 1.0) {
      throw std::invalid_argument("invalid planar geometry thresholds");
    }
    if (!finite_positive(max_correspondence_distance_m_) ||
        maximum_iterations_ <= 0 ||
        maximum_iterations_ > std::numeric_limits<int>::max() ||
        !finite_positive(max_fitness_score_m2_) ||
        !finite_positive(max_translation_per_scan_m_) ||
        !finite_positive(max_yaw_per_scan_rad_) ||
        max_yaw_per_scan_rad_ >= kPi) {
      throw std::invalid_argument("invalid ICP acceptance thresholds");
    }
  }

  Cloud::Ptr filtered_planar_cloud(
      const sensor_msgs::msg::PointCloud2& message) const {
    Cloud raw;
    pcl::fromROSMsg(message, raw);

    auto filtered = std::make_shared<Cloud>();
    filtered->reserve(raw.size());
    for (const auto& point : raw) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const double range = std::hypot(static_cast<double>(point.x),
                                      static_cast<double>(point.y));
      if (range <= min_range_m_ || range > max_range_m_) {
        continue;
      }
      filtered->emplace_back(point.x, point.y, 0.0F);
    }
    filtered->width = static_cast<std::uint32_t>(filtered->size());
    filtered->height = 1U;
    filtered->is_dense = true;
    return filtered;
  }

  bool has_observable_planar_geometry(const Cloud& cloud,
                                      double& largest_variance) const {
    if (cloud.size() < static_cast<std::size_t>(minimum_points_)) {
      return false;
    }

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& point : cloud) {
      centroid.x() += point.x;
      centroid.y() += point.y;
    }
    centroid /= static_cast<double>(cloud.size());

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto& point : cloud) {
      const Eigen::Vector2d centered(point.x - centroid.x(),
                                     point.y - centroid.y());
      covariance.noalias() += centered * centered.transpose();
    }
    covariance /= static_cast<double>(cloud.size());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    if (solver.info() != Eigen::Success ||
        !solver.eigenvalues().allFinite()) {
      return false;
    }
    const double smallest_variance = solver.eigenvalues()[0];
    largest_variance = solver.eigenvalues()[1];
    return largest_variance >= minimum_geometry_variance_m2_ &&
           smallest_variance / largest_variance >=
               minimum_geometry_ratio_;
  }

  void on_point_cloud(const sensor_msgs::msg::PointCloud2& message) {
    const std::int64_t current_stamp_ns = stamp_ns(message.header.stamp);
    if (current_stamp_ns <= 0 || message.header.frame_id.empty() ||
        !has_xyz_fields(message)) {
      RCLCPP_WARN(get_logger(),
                  "rejected point cloud with invalid stamp, frame or XYZ fields");
      return;
    }
    if (initialized_ && message.header.frame_id != child_frame_id_) {
      RCLCPP_WARN(get_logger(),
                  "rejected point cloud frame change from '%s' to '%s'",
                  child_frame_id_.c_str(), message.header.frame_id.c_str());
      return;
    }
    if (initialized_ && current_stamp_ns <= previous_stamp_ns_) {
      RCLCPP_WARN(get_logger(), "rejected non-monotonic point cloud stamp");
      return;
    }

    Cloud::Ptr cloud;
    try {
      cloud = filtered_planar_cloud(message);
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "rejected malformed point cloud: %s",
                  error.what());
      return;
    }

    double geometry_variance = 0.0;
    if (!has_observable_planar_geometry(*cloud, geometry_variance)) {
      RCLCPP_WARN(get_logger(),
                  "rejected point cloud with too few or degenerate planar points");
      return;
    }

    if (!initialized_) {
      previous_cloud_ = cloud;
      previous_stamp_ns_ = current_stamp_ns;
      child_frame_id_ = message.header.frame_id;
      initialized_ = true;
      publish_odometry(message, 0.0, geometry_variance);
      return;
    }

    icp_.setInputSource(cloud);
    icp_.setInputTarget(previous_cloud_);
    Cloud aligned;
    icp_.align(aligned, Eigen::Matrix4f::Identity());
    if (!icp_.hasConverged()) {
      RCLCPP_WARN(get_logger(), "rejected point cloud: planar ICP did not converge");
      return;
    }

    const double fitness_score =
        icp_.getFitnessScore(max_correspondence_distance_m_);
    const Eigen::Matrix4f delta = icp_.getFinalTransformation();
    if (!std::isfinite(fitness_score) ||
        fitness_score > max_fitness_score_m2_ || !delta.allFinite()) {
      RCLCPP_WARN(get_logger(),
                  "rejected point cloud: ICP fitness or transform is invalid");
      return;
    }

    const double delta_x = delta(0, 3);
    const double delta_y = delta(1, 3);
    const double delta_yaw = std::atan2(delta(1, 0), delta(0, 0));
    if (std::hypot(delta_x, delta_y) > max_translation_per_scan_m_ ||
        std::abs(delta_yaw) > max_yaw_per_scan_rad_) {
      RCLCPP_WARN(get_logger(), "rejected implausible lidar odometry jump");
      return;
    }

    const double cos_yaw = std::cos(yaw_rad_);
    const double sin_yaw = std::sin(yaw_rad_);
    x_m_ += cos_yaw * delta_x - sin_yaw * delta_y;
    y_m_ += sin_yaw * delta_x + cos_yaw * delta_y;
    yaw_rad_ = normalize_yaw(yaw_rad_ + delta_yaw);

    previous_cloud_ = cloud;
    previous_stamp_ns_ = current_stamp_ns;
    publish_odometry(message, fitness_score, geometry_variance);
  }

  void publish_odometry(const sensor_msgs::msg::PointCloud2& source,
                        double fitness_score,
                        double geometry_variance) {
    nav_msgs::msg::Odometry output;
    output.header.stamp = source.header.stamp;
    output.header.frame_id = odom_frame_id_;
    output.child_frame_id = child_frame_id_;
    output.pose.pose.position.x = x_m_;
    output.pose.pose.position.y = y_m_;
    output.pose.pose.orientation.z = std::sin(0.5 * yaw_rad_);
    output.pose.pose.orientation.w = std::cos(0.5 * yaw_rad_);

    const double position_variance = std::max(0.0025, fitness_score);
    const double yaw_variance = std::max(
        0.0012184696791468343,
        fitness_score / std::max(geometry_variance, 0.01));
    output.pose.covariance[0] = position_variance;
    output.pose.covariance[7] = position_variance;
    output.pose.covariance[14] = kUnknownVariance;
    output.pose.covariance[21] = kUnknownVariance;
    output.pose.covariance[28] = kUnknownVariance;
    output.pose.covariance[35] = yaw_variance;
    for (const std::size_t index : {0U, 7U, 14U, 21U, 28U, 35U}) {
      output.twist.covariance[index] = kUnknownVariance;
    }
    odometry_publisher_->publish(output);
    if (transform_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      transform_broadcaster_->sendTransform(transform);
    }
  }

  std::string odom_frame_id_;
  std::string child_frame_id_;
  bool publish_tf_{};
  double min_range_m_{};
  double max_range_m_{};
  std::int64_t minimum_points_{};
  double minimum_geometry_variance_m2_{};
  double minimum_geometry_ratio_{};
  double max_correspondence_distance_m_{};
  std::int64_t maximum_iterations_{};
  double max_fitness_score_m2_{};
  double max_translation_per_scan_m_{};
  double max_yaw_per_scan_rad_{};

  bool initialized_{};
  std::int64_t previous_stamp_ns_{};
  double x_m_{};
  double y_m_{};
  double yaw_rad_{};
  Cloud::ConstPtr previous_cloud_;
  pcl::IterativeClosestPoint<Point, Point> icp_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LidarOdometryNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("zju_lidar_odometry_node"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

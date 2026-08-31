#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace zju_coop_ros2 {

// 把驱动坐标系和单位统一成算法要求的 FLU、rad/s、m/s^2。
class ImuInputTransform {
 public:
  ImuInputTransform() = default;

  ImuInputTransform(const std::array<double, 4>& sensor_to_flu_xyzw,
                    double gyro_scale_to_rad_s,
                    double accel_scale_to_m_s2)
      : gyro_scale_(gyro_scale_to_rad_s),
        accel_scale_(accel_scale_to_m_s2) {
    const double norm = std::sqrt(
        sensor_to_flu_xyzw[0] * sensor_to_flu_xyzw[0] +
        sensor_to_flu_xyzw[1] * sensor_to_flu_xyzw[1] +
        sensor_to_flu_xyzw[2] * sensor_to_flu_xyzw[2] +
        sensor_to_flu_xyzw[3] * sensor_to_flu_xyzw[3]);
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3 ||
        !std::isfinite(gyro_scale_) || gyro_scale_ <= 0.0 ||
        !std::isfinite(accel_scale_) || accel_scale_ <= 0.0) {
      throw std::invalid_argument("invalid IMU input transform");
    }

    sensor_to_flu_xyzw_ = sensor_to_flu_xyzw;
    for (double& value : sensor_to_flu_xyzw_) {
      value /= norm;
    }
    const double x = sensor_to_flu_xyzw_[0];
    const double y = sensor_to_flu_xyzw_[1];
    const double z = sensor_to_flu_xyzw_[2];
    const double w = sensor_to_flu_xyzw_[3];
    sensor_to_flu_rotation_ = {
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
        2.0 * (x * z + y * w), 2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
        2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y)};
  }

  std::array<double, 3> angular_velocity(
      const std::array<double, 3>& sensor_value) const {
    return transform_vector(sensor_value, gyro_scale_);
  }

  std::array<double, 3> linear_acceleration(
      const std::array<double, 3>& sensor_value) const {
    return transform_vector(sensor_value, accel_scale_);
  }

  std::array<double, 9> angular_velocity_covariance(
      const std::array<double, 9>& sensor_covariance) const {
    return transform_covariance(sensor_covariance, gyro_scale_);
  }

  std::array<double, 9> linear_acceleration_covariance(
      const std::array<double, 9>& sensor_covariance) const {
    return transform_covariance(sensor_covariance, accel_scale_);
  }

  std::array<double, 9> orientation_covariance(
      const std::array<double, 9>& sensor_covariance) const {
    return transform_covariance(sensor_covariance, 1.0);
  }

  // 输入是 sensor->navigation；输出是 FLU->navigation。
  std::array<double, 4> orientation_flu_to_navigation(
      const std::array<double, 4>& sensor_to_navigation_xyzw) const {
    const std::array<double, 4> flu_to_sensor{
        -sensor_to_flu_xyzw_[0], -sensor_to_flu_xyzw_[1],
        -sensor_to_flu_xyzw_[2], sensor_to_flu_xyzw_[3]};
    return quaternion_product(sensor_to_navigation_xyzw, flu_to_sensor);
  }

 private:
  std::array<double, 3> transform_vector(
      const std::array<double, 3>& input, double scale) const {
    std::array<double, 3> output{};
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = 0U; column < 3U; ++column) {
        output[row] += sensor_to_flu_rotation_[row * 3U + column] *
                       input[column] * scale;
      }
    }
    return output;
  }

  std::array<double, 9> transform_covariance(
      const std::array<double, 9>& input, double scale) const {
    std::array<double, 9> output{};
    // sensor_msgs/Imu 用首元素 -1 表示该协方差未提供；它不是可旋转矩阵。
    if (input[0] == -1.0) {
      output[0] = -1.0;
      return output;
    }
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = 0U; column < 3U; ++column) {
        for (std::size_t left = 0U; left < 3U; ++left) {
          for (std::size_t right = 0U; right < 3U; ++right) {
            output[row * 3U + column] +=
                sensor_to_flu_rotation_[row * 3U + left] *
                input[left * 3U + right] *
                sensor_to_flu_rotation_[column * 3U + right] * scale * scale;
          }
        }
      }
    }
    return output;
  }

  static std::array<double, 4> quaternion_product(
      const std::array<double, 4>& left,
      const std::array<double, 4>& right) {
    return {
        left[3] * right[0] + left[0] * right[3] +
            left[1] * right[2] - left[2] * right[1],
        left[3] * right[1] - left[0] * right[2] +
            left[1] * right[3] + left[2] * right[0],
        left[3] * right[2] + left[0] * right[1] -
            left[1] * right[0] + left[2] * right[3],
        left[3] * right[3] - left[0] * right[0] -
            left[1] * right[1] - left[2] * right[2]};
  }

  std::array<double, 4> sensor_to_flu_xyzw_{0.0, 0.0, 0.0, 1.0};
  std::array<double, 9> sensor_to_flu_rotation_{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  double gyro_scale_{1.0};
  double accel_scale_{1.0};
};

}  // namespace zju_coop_ros2

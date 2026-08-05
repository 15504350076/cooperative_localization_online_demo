// 模块实现：WGS84经纬高→ECEF→固定ENU、两历元速度估计、杆臂修正和相对真值生成。
#include "gnss_reference.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace zju::coop {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kWgs84SemiMajorM = 6'378'137.0;
constexpr double kWgs84EccentricitySquared = 6.6943799901413165e-3;

bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

bool finite_matrix(const std::array<double, 9>& value) {
  return std::all_of(value.begin(), value.end(),
                     [](double item) { return std::isfinite(item); });
}

bool symmetric_covariance(const std::array<double, 9>& covariance) {
  constexpr double tolerance = 1.0e-9;
  return covariance[0] >= 0.0 && covariance[4] >= 0.0 &&
         covariance[8] >= 0.0 &&
         std::abs(covariance[1] - covariance[3]) <= tolerance &&
         std::abs(covariance[2] - covariance[6]) <= tolerance &&
         std::abs(covariance[5] - covariance[7]) <= tolerance;
}

Vec3 geodetic_to_ecef(double latitude_deg, double longitude_deg,
                      double altitude_m) {
  const double latitude = latitude_deg * kDegreesToRadians;
  const double longitude = longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);
  const double prime_vertical_radius =
      kWgs84SemiMajorM /
      std::sqrt(1.0 - kWgs84EccentricitySquared *
                          sin_latitude * sin_latitude);
  return {(prime_vertical_radius + altitude_m) * cos_latitude *
              cos_longitude,
          (prime_vertical_radius + altitude_m) * cos_latitude *
              sin_longitude,
          (prime_vertical_radius *
               (1.0 - kWgs84EccentricitySquared) +
           altitude_m) *
              sin_latitude};
}

std::array<double, 9> ecef_from_enu(double latitude_rad,
                                    double longitude_rad) {
  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);
  return {-sin_longitude,
          -sin_latitude * cos_longitude,
          cos_latitude * cos_longitude,
          cos_longitude,
          -sin_latitude * sin_longitude,
          cos_latitude * sin_longitude,
          0.0,
          cos_latitude,
          sin_latitude};
}

Vec3 transpose_multiply(const std::array<double, 9>& matrix,
                        const Vec3& value) {
  return {matrix[0] * value.x + matrix[3] * value.y +
              matrix[6] * value.z,
          matrix[1] * value.x + matrix[4] * value.y +
              matrix[7] * value.z,
          matrix[2] * value.x + matrix[5] * value.y +
              matrix[8] * value.z};
}

std::array<double, 9> transpose(const std::array<double, 9>& matrix) {
  return {matrix[0], matrix[3], matrix[6], matrix[1], matrix[4],
          matrix[7], matrix[2], matrix[5], matrix[8]};
}

std::array<double, 9> multiply(const std::array<double, 9>& left,
                               const std::array<double, 9>& right) {
  std::array<double, 9> output{};
  for (std::size_t row = 0U; row < 3U; ++row) {
    for (std::size_t col = 0U; col < 3U; ++col) {
      for (std::size_t inner = 0U; inner < 3U; ++inner) {
        output[row * 3U + col] +=
            left[row * 3U + inner] * right[inner * 3U + col];
      }
    }
  }
  return output;
}

std::array<double, 9> add_covariance(
    const std::array<double, 9>& left,
    const std::array<double, 9>& right) {
  std::array<double, 9> output{};
  for (std::size_t index = 0U; index < output.size(); ++index) {
    output[index] = left[index] + right[index];
  }
  return output;
}

std::uint64_t absolute_difference(std::uint64_t left,
                                  std::uint64_t right) {
  return left >= right ? left - right : right - left;
}

Quaternion quaternion_from_xyzw(const std::array<double, 4>& value) {
  Quaternion output{value[3], value[0], value[1], value[2]};
  if (!output.normalize()) {
    throw std::invalid_argument("GNSS node orientation must be finite and unit");
  }
  return output;
}

}  // namespace

GnssReference::GnssReference(GnssReferenceConfig config)
    : config_(std::move(config)) {
  if (config_.nodes.empty() || config_.max_epoch_skew_ns == 0U ||
      config_.max_truth_age_ns == 0U ||
      config_.max_future_skew_ns == 0U ||
      config_.max_receive_delay_ns == 0U ||
      !positive_finite(config_.min_velocity_dt_s) ||
      !positive_finite(config_.max_velocity_dt_s) ||
      config_.min_velocity_dt_s > config_.max_velocity_dt_s ||
      !positive_finite(config_.max_horizontal_std_m) ||
      !positive_finite(config_.max_vertical_std_m)) {
    throw std::invalid_argument("invalid GNSS reference configuration");
  }
  std::unordered_set<std::uint32_t> unique_nodes;
  for (const auto& node : config_.nodes) {
    if (node.node_id == 0U || !finite(node.antenna_lever_arm_body_m) ||
        !unique_nodes.insert(node.node_id).second) {
      throw std::invalid_argument("invalid or duplicate GNSS node");
    }
    (void)quaternion_from_xyzw(node.orientation_body_to_enu_xyzw);
    node_order_.push_back(node.node_id);
    nodes_.emplace(node.node_id, NodeState{node, {}});
  }
  if (unique_nodes.count(config_.reference_node_id) == 0U) {
    throw std::invalid_argument("GNSS reference node is not configured");
  }
}

bool GnssReference::structurally_valid(const GnssFix& fix) const {
  if (!fix.valid || fix.status < 0 || fix.timestamp_ns == 0U ||
      fix.receive_timestamp_ns == 0U || fix.frame_id.empty() ||
      !std::isfinite(fix.latitude_deg) ||
      !std::isfinite(fix.longitude_deg) || !std::isfinite(fix.altitude_m) ||
      fix.latitude_deg < -90.0 || fix.latitude_deg > 90.0 ||
      fix.longitude_deg < -180.0 || fix.longitude_deg > 180.0 ||
      !finite_matrix(fix.position_covariance_m2) ||
      !symmetric_covariance(fix.position_covariance_m2)) {
    return false;
  }
  if (config_.require_known_covariance &&
      static_cast<std::uint8_t>(fix.covariance_type) <
          static_cast<std::uint8_t>(
              GnssCovarianceType::kDiagonalKnown)) {
    return false;
  }
  return std::sqrt(fix.position_covariance_m2[0]) <=
             config_.max_horizontal_std_m &&
         std::sqrt(fix.position_covariance_m2[4]) <=
             config_.max_horizontal_std_m &&
         std::sqrt(fix.position_covariance_m2[8]) <=
             config_.max_vertical_std_m;
}

GnssPushDisposition GnssReference::push(const GnssFix& fix) {
  const auto found = nodes_.find(fix.node_id);
  if (found == nodes_.end()) {
    return GnssPushDisposition::kUnknownNode;
  }
  if (!structurally_valid(fix)) {
    return GnssPushDisposition::kInvalidPacket;
  }
  const bool too_future =
      fix.timestamp_ns > fix.receive_timestamp_ns &&
      fix.timestamp_ns - fix.receive_timestamp_ns >
          config_.max_future_skew_ns;
  const bool too_delayed =
      fix.receive_timestamp_ns > fix.timestamp_ns &&
      fix.receive_timestamp_ns - fix.timestamp_ns >
          config_.max_receive_delay_ns;
  if (too_future || too_delayed) {
    return GnssPushDisposition::kTimeRejected;
  }
  auto& history = found->second.history;
  if (!history.empty()) {
    const auto& latest = history.back();
    if (fix.timestamp_ns == latest.timestamp_ns &&
        fix.sequence == latest.sequence) {
      return GnssPushDisposition::kDuplicate;
    }
    if (fix.timestamp_ns <= latest.timestamp_ns) {
      return GnssPushDisposition::kOutOfOrder;
    }
  }
  history.push_back(fix);
  if (history.size() > 2U) {
    history.erase(history.begin());
  }
  // 初始化结果只允许生成一次。初始化前，缓存本来就是空的；初始化成功后，
  // 新到达的 RTK 数据只更新“相对真值”，不能让算法重新选原点或重置滤波状态。
  // 因此这里故意不清空 cached_initializations_。
  return GnssPushDisposition::kStored;
}

Vec3 GnssReference::antenna_position_enu(const GnssFix& fix,
                                          const NodeState& node) const {
  if (!reference_frame_) {
    throw std::logic_error("GNSS reference frame is not initialized");
  }
  const Vec3 delta_ecef =
      geodetic_to_ecef(fix.latitude_deg, fix.longitude_deg,
                       fix.altitude_m) -
      reference_frame_->origin_ecef_m;
  const Vec3 antenna_enu =
      transpose_multiply(reference_frame_->ecef_from_enu, delta_ecef);
  const Quaternion body_to_enu =
      quaternion_from_xyzw(node.config.orientation_body_to_enu_xyzw);
  return antenna_enu -
         body_to_enu.rotate(node.config.antenna_lever_arm_body_m);
}

std::array<double, 9> GnssReference::covariance_in_reference_enu(
    const GnssFix& fix) const {
  if (!reference_frame_) {
    throw std::logic_error("GNSS reference frame is not initialized");
  }
  const auto fix_ecef_from_enu =
      ecef_from_enu(fix.latitude_deg * kDegreesToRadians,
                    fix.longitude_deg * kDegreesToRadians);
  const auto common_enu_from_ecef =
      transpose(reference_frame_->ecef_from_enu);
  const auto common_from_fix =
      multiply(common_enu_from_ecef, fix_ecef_from_enu);
  return multiply(multiply(common_from_fix, fix.position_covariance_m2),
                  transpose(common_from_fix));
}

std::optional<std::vector<GnssInitialization>>
GnssReference::build_initializations() {
  if (cached_initializations_) {
    return cached_initializations_;
  }
  std::uint64_t minimum_latest = std::numeric_limits<std::uint64_t>::max();
  std::uint64_t maximum_latest = 0U;
  for (const std::uint32_t node_id : node_order_) {
    const auto& history = nodes_.at(node_id).history;
    if (history.size() < 2U) {
      return std::nullopt;
    }
    minimum_latest = std::min(minimum_latest, history.back().timestamp_ns);
    maximum_latest = std::max(maximum_latest, history.back().timestamp_ns);
    const double dt_s =
        static_cast<double>(history.back().timestamp_ns -
                            history.front().timestamp_ns) *
        1.0e-9;
    if (dt_s < config_.min_velocity_dt_s ||
        dt_s > config_.max_velocity_dt_s) {
      return std::nullopt;
    }
  }
  if (maximum_latest - minimum_latest > config_.max_epoch_skew_ns) {
    return std::nullopt;
  }

  const auto& reference_fix =
      nodes_.at(config_.reference_node_id).history.back();
  ReferenceFrame frame{};
  frame.latitude_rad = reference_fix.latitude_deg * kDegreesToRadians;
  frame.longitude_rad = reference_fix.longitude_deg * kDegreesToRadians;
  frame.origin_ecef_m =
      geodetic_to_ecef(reference_fix.latitude_deg,
                       reference_fix.longitude_deg,
                       reference_fix.altitude_m);
  frame.ecef_from_enu =
      ecef_from_enu(frame.latitude_rad, frame.longitude_rad);
  reference_frame_ = frame;

  std::vector<GnssInitialization> output;
  output.reserve(node_order_.size());
  for (const std::uint32_t node_id : node_order_) {
    const auto& node = nodes_.at(node_id);
    const auto& previous = node.history.front();
    const auto& latest = node.history.back();
    const double dt_s =
        static_cast<double>(latest.timestamp_ns - previous.timestamp_ns) *
        1.0e-9;
    const Vec3 previous_position = antenna_position_enu(previous, node);
    const Vec3 latest_position = antenna_position_enu(latest, node);
    const auto previous_covariance =
        covariance_in_reference_enu(previous);
    const auto latest_covariance =
        covariance_in_reference_enu(latest);
    GnssInitialization initialization{};
    initialization.node_id = node_id;
    initialization.timestamp_ns = latest.timestamp_ns;
    initialization.position_enu_m = latest_position;
    initialization.velocity_enu_mps =
        (latest_position - previous_position) / dt_s;
    initialization.orientation_body_to_enu_xyzw =
        node.config.orientation_body_to_enu_xyzw;
    initialization.position_covariance_m2 = latest_covariance;
    initialization.velocity_covariance_m2ps2 =
        add_covariance(previous_covariance, latest_covariance);
    for (double& value : initialization.velocity_covariance_m2ps2) {
      value /= dt_s * dt_s;
    }
    output.push_back(initialization);
  }
  cached_initializations_ = output;
  return cached_initializations_;
}

std::vector<GnssRelativeTruth> GnssReference::relative_truth(
    std::uint64_t now_ns) const {
  std::vector<GnssRelativeTruth> output;
  output.reserve(node_order_.size());
  if (!reference_frame_ ||
      nodes_.at(config_.reference_node_id).history.empty()) {
    for (const std::uint32_t node_id : node_order_) {
      output.push_back(
          {node_id, config_.reference_node_id, 0U, 0U, {}, {}, false, true});
    }
    return output;
  }

  const auto& reference_node = nodes_.at(config_.reference_node_id);
  const auto& reference_fix = reference_node.history.back();
  const Vec3 reference_position =
      antenna_position_enu(reference_fix, reference_node);
  const auto reference_covariance =
      covariance_in_reference_enu(reference_fix);
  for (const std::uint32_t node_id : node_order_) {
    GnssRelativeTruth truth{};
    truth.node_id = node_id;
    truth.reference_node_id = config_.reference_node_id;
    truth.reference_timestamp_ns = reference_fix.timestamp_ns;
    const auto& node = nodes_.at(node_id);
    if (node.history.empty()) {
      truth.stale = true;
      output.push_back(truth);
      continue;
    }
    const auto& fix = node.history.back();
    truth.node_timestamp_ns = fix.timestamp_ns;
    const bool future = fix.timestamp_ns > now_ns;
    const std::uint64_t age = future ? 0U : now_ns - fix.timestamp_ns;
    truth.stale = future || age > config_.max_truth_age_ns;
    const bool synchronized =
        absolute_difference(fix.timestamp_ns, reference_fix.timestamp_ns) <=
        config_.max_epoch_skew_ns;
    truth.valid = !truth.stale && synchronized;
    if (node_id == config_.reference_node_id) {
      truth.position_enu_m = {};
      truth.position_covariance_m2 = {};
    } else {
      truth.position_enu_m =
          antenna_position_enu(fix, node) - reference_position;
      truth.position_covariance_m2 =
          add_covariance(covariance_in_reference_enu(fix),
                         reference_covariance);
    }
    output.push_back(truth);
  }
  return output;
}

}  // namespace zju::coop

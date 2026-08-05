// 模块职责：把标准NavSatFix语义的WGS84定位转换为固定共同ENU初值和独立RTK相对真值。
// 本模块不持有Engine，也没有任何GNSS量测更新接口，避免评估真值反馈进入协同滤波器。
#pragma once

#include "quaternion.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace zju::coop {

enum class GnssCovarianceType : std::uint8_t {
  kUnknown = 0U,
  kApproximated = 1U,
  kDiagonalKnown = 2U,
  kKnown = 3U,
};

struct GnssFix {
  std::uint32_t node_id{};
  std::uint64_t sequence{};
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  std::string frame_id;
  std::int8_t status{-1};
  std::uint16_t service{};
  double latitude_deg{};
  double longitude_deg{};
  double altitude_m{};
  std::array<double, 9> position_covariance_m2{};
  GnssCovarianceType covariance_type{GnssCovarianceType::kUnknown};
  bool valid{};
};

struct GnssNodeConfig {
  std::uint32_t node_id{};
  Vec3 antenna_lever_arm_body_m{};
  std::array<double, 4> orientation_body_to_enu_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct GnssReferenceConfig {
  std::uint32_t reference_node_id{1U};
  std::vector<GnssNodeConfig> nodes;
  std::uint64_t max_epoch_skew_ns{100'000'000ULL};
  std::uint64_t max_truth_age_ns{500'000'000ULL};
  std::uint64_t max_future_skew_ns{100'000'000ULL};
  std::uint64_t max_receive_delay_ns{500'000'000ULL};
  double min_velocity_dt_s{0.2};
  double max_velocity_dt_s{5.0};
  double max_horizontal_std_m{0.2};
  double max_vertical_std_m{0.5};
  bool require_known_covariance{true};
};

enum class GnssPushDisposition {
  kStored,
  kInvalidPacket,
  kUnknownNode,
  kDuplicate,
  kOutOfOrder,
  kTimeRejected,
};

struct GnssInitialization {
  std::uint32_t node_id{};
  std::uint64_t timestamp_ns{};
  Vec3 position_enu_m{};
  Vec3 velocity_enu_mps{};
  std::array<double, 4> orientation_body_to_enu_xyzw{};
  std::array<double, 9> position_covariance_m2{};
  std::array<double, 9> velocity_covariance_m2ps2{};
};

struct GnssRelativeTruth {
  std::uint32_t node_id{};
  std::uint32_t reference_node_id{};
  std::uint64_t node_timestamp_ns{};
  std::uint64_t reference_timestamp_ns{};
  Vec3 position_enu_m{};
  std::array<double, 9> position_covariance_m2{};
  bool valid{};
  bool stale{};
};

class GnssReference {
 public:
  explicit GnssReference(GnssReferenceConfig config);

  [[nodiscard]] GnssPushDisposition push(const GnssFix& fix);
  [[nodiscard]] std::optional<std::vector<GnssInitialization>>
  build_initializations();
  [[nodiscard]] std::vector<GnssRelativeTruth> relative_truth(
      std::uint64_t now_ns) const;

 private:
  struct NodeState {
    GnssNodeConfig config;
    std::vector<GnssFix> history;
  };

  struct ReferenceFrame {
    double latitude_rad{};
    double longitude_rad{};
    Vec3 origin_ecef_m{};
    std::array<double, 9> ecef_from_enu{};
  };

  [[nodiscard]] bool structurally_valid(const GnssFix& fix) const;
  [[nodiscard]] Vec3 antenna_position_enu(const GnssFix& fix,
                                          const NodeState& node) const;
  [[nodiscard]] std::array<double, 9> covariance_in_reference_enu(
      const GnssFix& fix) const;

  GnssReferenceConfig config_;
  std::vector<std::uint32_t> node_order_;
  std::unordered_map<std::uint32_t, NodeState> nodes_;
  std::optional<ReferenceFrame> reference_frame_;
  std::optional<std::vector<GnssInitialization>> cached_initializations_;
};

}  // namespace zju::coop

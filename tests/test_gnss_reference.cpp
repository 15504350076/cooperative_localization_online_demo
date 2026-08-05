// 模块职责：验证标准NavSatFix映射后的GNSS样本可生成共同ENU初值和独立RTK相对真值。
#include "core/gnss_reference.hpp"

#include "test_support.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOriginLatitudeDeg = 30.0;
constexpr double kOriginLongitudeDeg = 120.0;
constexpr double kOriginAltitudeM = 10.0;
constexpr std::uint64_t kFirstTimestampNs = 1'000'000'000ULL;
constexpr std::uint64_t kSecondTimestampNs = 2'000'000'000ULL;

// near用绝对误差判断米级地理变换结果，避免直接比较浮点二进制表示。
bool near(double left, double right, double tolerance) {
  return std::abs(left - right) <= tolerance;
}

// offset_fix用小范围曲率半径把期望东、北偏移换算成WGS84经纬度，构造可解释测试数据。
zju::coop::GnssFix offset_fix(std::uint32_t node_id,
                              std::uint64_t sequence,
                              std::uint64_t timestamp_ns, double east_m,
                              double north_m, double up_m = 0.0) {
  constexpr double semi_major = 6'378'137.0;
  constexpr double eccentricity_squared = 6.6943799901413165e-3;
  const double latitude_rad = kOriginLatitudeDeg * kPi / 180.0;
  const double sin_latitude = std::sin(latitude_rad);
  const double denominator = std::sqrt(
      1.0 - eccentricity_squared * sin_latitude * sin_latitude);
  const double prime_vertical_radius = semi_major / denominator;
  const double meridian_radius =
      semi_major * (1.0 - eccentricity_squared) /
      (denominator * denominator * denominator);

  zju::coop::GnssFix fix{};
  fix.node_id = node_id;
  fix.sequence = sequence;
  fix.timestamp_ns = timestamp_ns;
  fix.receive_timestamp_ns = timestamp_ns;
  fix.latitude_deg =
      kOriginLatitudeDeg +
      north_m / (meridian_radius + kOriginAltitudeM) * 180.0 / kPi;
  fix.longitude_deg =
      kOriginLongitudeDeg +
      east_m /
          ((prime_vertical_radius + kOriginAltitudeM) *
           std::cos(latitude_rad)) *
          180.0 / kPi;
  fix.altitude_m = kOriginAltitudeM + up_m;
  fix.status = 0;
  fix.service = 1U;
  fix.covariance_type = zju::coop::GnssCovarianceType::kKnown;
  fix.valid = true;
  fix.frame_id = "gnss_link";
  fix.position_covariance_m2 = {0.01, 0.0, 0.0, 0.0, 0.01,
                                0.0, 0.0, 0.0, 0.04};
  return fix;
}

zju::coop::GnssReferenceConfig three_node_config() {
  zju::coop::GnssReferenceConfig config{};
  config.reference_node_id = 1U;
  config.max_epoch_skew_ns = 20'000'000ULL;
  config.max_truth_age_ns = 500'000'000ULL;
  config.max_future_skew_ns = 100'000'000ULL;
  config.max_receive_delay_ns = 500'000'000ULL;
  config.min_velocity_dt_s = 0.2;
  config.max_velocity_dt_s = 2.0;
  config.max_horizontal_std_m = 0.2;
  config.max_vertical_std_m = 0.5;
  config.require_known_covariance = true;
  for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
    zju::coop::GnssNodeConfig node{};
    node.node_id = node_id;
    node.orientation_body_to_enu_xyzw[3] = 1.0;
    config.nodes.push_back(node);
  }
  return config;
}

}  // namespace

TEST_CASE(gnss_reference_builds_three_node_enu_initialization_and_velocity) {
  zju::coop::GnssReference reference(three_node_config());

  EXPECT_EQ(reference.push(offset_fix(1U, 1U, kFirstTimestampNs, 0.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(2U, 1U, kFirstTimestampNs, 2.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(3U, 1U, kFirstTimestampNs, 0.0, 4.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(1U, 2U, kSecondTimestampNs, 0.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(2U, 2U, kSecondTimestampNs, 3.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(3U, 2U, kSecondTimestampNs, 0.0, 4.0)),
            zju::coop::GnssPushDisposition::kStored);

  const auto initializations = reference.build_initializations();
  EXPECT_TRUE(initializations.has_value());
  EXPECT_EQ(initializations->size(), 3U);
  EXPECT_TRUE(near((*initializations)[0U].position_enu_m.x, 0.0, 0.02));
  EXPECT_TRUE(near((*initializations)[1U].position_enu_m.x, 3.0, 0.02));
  EXPECT_TRUE(near((*initializations)[2U].position_enu_m.y, 4.0, 0.02));
  EXPECT_TRUE(
      near((*initializations)[1U].velocity_enu_mps.x, 1.0, 0.03));
}

TEST_CASE(gnss_reference_truth_is_relative_and_becomes_stale) {
  zju::coop::GnssReference reference(three_node_config());
  for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
    EXPECT_EQ(reference.push(offset_fix(node_id, 1U, kFirstTimestampNs,
                                        node_id == 2U ? 2.0 : 0.0,
                                        node_id == 3U ? 4.0 : 0.0)),
              zju::coop::GnssPushDisposition::kStored);
    EXPECT_EQ(reference.push(offset_fix(node_id, 2U, kSecondTimestampNs,
                                        node_id == 2U ? 3.0 : 0.0,
                                        node_id == 3U ? 4.0 : 0.0)),
              zju::coop::GnssPushDisposition::kStored);
  }
  EXPECT_TRUE(reference.build_initializations().has_value());

  const auto fresh = reference.relative_truth(kSecondTimestampNs);
  EXPECT_EQ(fresh.size(), 3U);
  EXPECT_TRUE(fresh[0U].valid);
  EXPECT_TRUE(near(fresh[0U].position_enu_m.x, 0.0, 1.0e-9));
  EXPECT_TRUE(near(fresh[1U].position_enu_m.x, 3.0, 0.02));
  EXPECT_TRUE(near(fresh[2U].position_enu_m.y, 4.0, 0.02));
  EXPECT_TRUE(near(fresh[1U].position_covariance_m2[0], 0.02, 0.002));

  const auto stale = reference.relative_truth(
      kSecondTimestampNs + 600'000'000ULL);
  EXPECT_FALSE(stale[1U].valid);
  EXPECT_TRUE(stale[1U].stale);
}

TEST_CASE(gnss_reference_rejects_bad_quality_order_and_epoch_skew) {
  auto config = three_node_config();
  zju::coop::GnssReference reference(config);

  auto bad_covariance = offset_fix(1U, 1U, kFirstTimestampNs, 0.0, 0.0);
  bad_covariance.position_covariance_m2[0] = 1.0;
  EXPECT_EQ(reference.push(bad_covariance),
            zju::coop::GnssPushDisposition::kInvalidPacket);
  EXPECT_EQ(reference.push(offset_fix(99U, 1U, kFirstTimestampNs, 0.0, 0.0)),
            zju::coop::GnssPushDisposition::kUnknownNode);

  for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
    EXPECT_EQ(reference.push(offset_fix(node_id, 1U, kFirstTimestampNs,
                                        static_cast<double>(node_id), 0.0)),
              zju::coop::GnssPushDisposition::kStored);
    const std::uint64_t second_time =
        kSecondTimestampNs + (node_id == 3U ? 50'000'000ULL : 0ULL);
    EXPECT_EQ(reference.push(offset_fix(node_id, 2U, second_time,
                                        static_cast<double>(node_id), 0.0)),
              zju::coop::GnssPushDisposition::kStored);
  }
  EXPECT_FALSE(reference.build_initializations().has_value());
  EXPECT_EQ(reference.push(offset_fix(2U, 1U, kFirstTimestampNs, 0.0, 0.0)),
            zju::coop::GnssPushDisposition::kOutOfOrder);
}

TEST_CASE(gnss_reference_freezes_initialization_origin_after_first_success) {
  zju::coop::GnssReference reference(three_node_config());
  for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
    EXPECT_EQ(reference.push(offset_fix(node_id, 1U, kFirstTimestampNs,
                                        node_id == 2U ? 2.0 : 0.0, 0.0)),
              zju::coop::GnssPushDisposition::kStored);
    EXPECT_EQ(reference.push(offset_fix(node_id, 2U, kSecondTimestampNs,
                                        node_id == 2U ? 3.0 : 0.0, 0.0)),
              zju::coop::GnssPushDisposition::kStored);
  }
  const auto first = reference.build_initializations();
  EXPECT_TRUE(first.has_value());
  EXPECT_TRUE(near((*first)[1U].position_enu_m.x, 3.0, 0.02));

  constexpr std::uint64_t third_time = 3'000'000'000ULL;
  EXPECT_EQ(reference.push(offset_fix(1U, 3U, third_time, 10.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(2U, 3U, third_time, 13.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  EXPECT_EQ(reference.push(offset_fix(3U, 3U, third_time, 10.0, 0.0)),
            zju::coop::GnssPushDisposition::kStored);
  const auto second = reference.build_initializations();
  EXPECT_TRUE(second.has_value());
  // 首次初始化成功后必须保留当时的初始化结果；后续 RTK 样本仅更新真值，
  // 不能悄悄改变初始化时间、公共 ENU 原点或滤波器的初始状态。
  EXPECT_EQ((*second)[0U].timestamp_ns, kSecondTimestampNs);
  EXPECT_EQ((*second)[1U].timestamp_ns, kSecondTimestampNs);
  EXPECT_TRUE(near((*second)[1U].position_enu_m.x, 3.0, 0.02));
}

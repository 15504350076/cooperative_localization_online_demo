#include "core/distributed_fusion2d.hpp"
#include "test_support.hpp"

#include <cmath>

namespace {

using zju::coop::DistributedFusion2D;
using zju::coop::DistributedFusionConfig;
using zju::coop::NodeState;
using zju::coop::RangePacket;

DistributedFusionConfig config() {
  DistributedFusionConfig value{};
  value.reference_node_id = 1U;
  value.node_ids = {1U, 2U, 3U};
  value.initial_correction_std_m = 1.0;
  value.process_accel_std_mps2 = 0.1;
  value.nis_gate = 100.0;
  value.max_extrapolation_ns = 100'000'000ULL;
  value.node_timeout_ns = 500'000'000ULL;
  return value;
}

NodeState state(std::uint32_t node_id, std::uint64_t timestamp_ns, double x,
                double y, double yaw_rad = 0.0) {
  NodeState value{};
  value.node_id = node_id;
  value.timestamp_ns = timestamp_ns;
  value.receive_timestamp_ns = timestamp_ns + 1U;
  value.position_enu_m = {x, y, 0.0};
  value.orientation_flu_to_enu =
      {std::cos(0.5 * yaw_rad), 0.0, 0.0, std::sin(0.5 * yaw_rad)};
  value.valid = true;
  return value;
}

RangePacket range(std::uint16_t from, std::uint16_t to,
                  std::uint64_t timestamp_ns, double distance_m) {
  RangePacket value{};
  value.from_node = from;
  value.to_node = to;
  value.timestamp_ns = timestamp_ns;
  value.receive_timestamp_ns = timestamp_ns + 1U;
  value.range_m = distance_m;
  value.range_std_m = 0.1;
  value.valid = true;
  return value;
}

const zju::coop::DistributedVehiclePose2D& pose(
    const zju::coop::DistributedPose2DSnapshot& snapshot,
    std::uint32_t node_id) {
  for (const auto& value : snapshot.vehicles) {
    if (value.node_id == node_id) {
      return value;
    }
  }
  throw std::runtime_error("pose not found");
}

}  // namespace

TEST_CASE(distributed_fusion_outputs_reference_relative_enu_pose_and_yaw) {
  DistributedFusion2D fusion(config());
  const std::uint64_t time = 1'000'000'000ULL;
  EXPECT_TRUE(fusion.push_node_state(state(1U, time, 10.0, 20.0)));
  EXPECT_TRUE(fusion.push_node_state(state(2U, time, 13.0, 24.0, 0.5)));
  EXPECT_TRUE(fusion.push_node_state(state(3U, time, 8.0, 21.0, -0.25)));

  const auto snapshot = fusion.pose2d_snapshot(time + 2U);
  EXPECT_EQ(snapshot.reference_node_id, 1U);
  EXPECT_EQ(snapshot.timestamp_ns, time);
  EXPECT_TRUE(pose(snapshot, 1U).position_valid);
  EXPECT_TRUE(std::abs(pose(snapshot, 1U).x_m) < 1.0e-12);
  EXPECT_TRUE(std::abs(pose(snapshot, 2U).x_m - 3.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(pose(snapshot, 2U).y_m - 4.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(pose(snapshot, 2U).yaw_rad - 0.5) < 1.0e-12);
}

TEST_CASE(distributed_fusion_uwb_correction_survives_new_node_state) {
  DistributedFusion2D fusion(config());
  const std::uint64_t first = 1'000'000'000ULL;
  (void)fusion.push_node_state(state(1U, first, 0.0, 0.0));
  (void)fusion.push_node_state(state(2U, first, 4.0, 0.0));
  (void)fusion.push_node_state(state(3U, first, 0.0, 4.0));

  const double before = pose(fusion.pose2d_snapshot(first + 2U), 2U).x_m;
  const auto update = fusion.push_range(range(1U, 2U, first, 3.0));
  EXPECT_EQ(update.disposition, zju::coop::UpdateDisposition::Accepted);
  const double corrected =
      pose(fusion.pose2d_snapshot(first + 2U), 2U).x_m;
  EXPECT_TRUE(std::abs(corrected - 3.0) < std::abs(before - 3.0));

  const std::uint64_t second = first + 50'000'000ULL;
  (void)fusion.push_node_state(state(1U, second, 0.0, 0.0));
  (void)fusion.push_node_state(state(2U, second, 5.0, 0.0));
  (void)fusion.push_node_state(state(3U, second, 0.0, 4.0));
  const double moved =
      pose(fusion.pose2d_snapshot(second + 2U), 2U).x_m;
  EXPECT_TRUE(std::abs((moved - 5.0) - (corrected - 4.0)) < 1.0e-9);
}

TEST_CASE(distributed_fusion_new_handle_clears_history_and_uwb_correction) {
  const std::uint64_t time = 1'000'000'000ULL;
  {
    DistributedFusion2D fusion(config());
    (void)fusion.push_node_state(state(1U, time, 0.0, 0.0));
    (void)fusion.push_node_state(state(2U, time, 4.0, 0.0));
    (void)fusion.push_node_state(state(3U, time, 0.0, 4.0));
    EXPECT_EQ(fusion.push_range(range(1U, 2U, time, 3.0)).disposition,
              zju::coop::UpdateDisposition::Accepted);
    EXPECT_TRUE(std::abs(pose(fusion.pose2d_snapshot(time + 1U), 2U).x_m -
                         4.0) > 1.0e-3);
  }

  DistributedFusion2D restarted(config());
  (void)restarted.push_node_state(state(1U, time + 1U, 0.0, 0.0));
  (void)restarted.push_node_state(state(2U, time + 1U, 10.0, 0.0));
  (void)restarted.push_node_state(state(3U, time + 1U, 0.0, 10.0));
  EXPECT_TRUE(std::abs(
      pose(restarted.pose2d_snapshot(time + 2U), 2U).x_m - 10.0) < 1.0e-12);
}

TEST_CASE(distributed_fusion_aligns_range_time_and_rejects_stale_inputs) {
  DistributedFusion2D fusion(config());
  const std::uint64_t first = 1'000'000'000ULL;
  const std::uint64_t second = first + 100'000'000ULL;
  for (const auto& sample : {
           state(1U, first, 0.0, 0.0), state(2U, first, 4.0, 0.0),
           state(3U, first, 0.0, 4.0), state(1U, second, 0.0, 0.0),
           state(2U, second, 6.0, 0.0), state(3U, second, 0.0, 6.0)}) {
    EXPECT_TRUE(fusion.push_node_state(sample));
  }

  const std::uint64_t middle = first + 50'000'000ULL;
  const auto aligned = fusion.push_range(range(1U, 2U, middle, 5.0));
  EXPECT_EQ(aligned.disposition, zju::coop::UpdateDisposition::Accepted);
  EXPECT_TRUE(std::abs(aligned.innovation_m) < 1.0e-12);

  EXPECT_FALSE(fusion.push_node_state(state(2U, second, 6.0, 0.0)));

  const auto too_late =
      fusion.push_range(range(1U, 3U, second + 100'000'001ULL, 6.0));
  EXPECT_EQ(too_late.disposition, zju::coop::UpdateDisposition::OutOfOrder);

  const auto stale =
      fusion.pose2d_snapshot(second + config().node_timeout_ns + 2U);
  EXPECT_FALSE(pose(stale, 1U).position_valid);
  EXPECT_FALSE(pose(stale, 2U).yaw_valid);
}

TEST_CASE(distributed_fusion_reports_specific_invalid_range_reasons) {
  DistributedFusion2D fusion(config());
  const std::uint64_t time = 1'000'000'000ULL;
  (void)fusion.push_node_state(state(1U, time, 0.0, 0.0));
  (void)fusion.push_node_state(state(2U, time, 3.0, 0.0));
  (void)fusion.push_node_state(state(3U, time, 0.0, 4.0));

  EXPECT_EQ(fusion.push_range(range(1U, 2U, time, 0.0)).disposition,
            zju::coop::UpdateDisposition::NonPositiveRange);
  EXPECT_EQ(fusion.push_range(range(1U, 1U, time, 1.0)).disposition,
            zju::coop::UpdateDisposition::SelfRange);
  EXPECT_EQ(fusion.push_range(range(1U, 9U, time, 1.0)).disposition,
            zju::coop::UpdateDisposition::UnknownNode);
  auto delayed = range(1U, 2U, time, 3.0);
  delayed.receive_timestamp_ns = time + config().max_receive_delay_ns + 1U;
  EXPECT_EQ(fusion.push_range(delayed).disposition,
            zju::coop::UpdateDisposition::InvalidPacket);

  auto device_invalid = range(1U, 2U, time, 3.0);
  device_invalid.status = 2U;
  EXPECT_EQ(fusion.push_range(device_invalid).disposition,
            zju::coop::UpdateDisposition::InvalidPacket);
}

TEST_CASE(distributed_fusion_deduplicates_each_uwb_edge_without_blocking_peers) {
  DistributedFusion2D fusion(config());
  const std::uint64_t time = 1'000'000'000ULL;
  (void)fusion.push_node_state(state(1U, time, 0.0, 0.0));
  (void)fusion.push_node_state(state(2U, time, 3.0, 0.0));
  (void)fusion.push_node_state(state(3U, time, 0.0, 4.0));

  EXPECT_EQ(fusion.push_range(range(1U, 2U, time, 3.0)).disposition,
            zju::coop::UpdateDisposition::Accepted);
  EXPECT_EQ(fusion.push_range(range(2U, 1U, time, 3.0)).disposition,
            zju::coop::UpdateDisposition::OutOfOrder);
  EXPECT_EQ(fusion.push_range(range(1U, 3U, time, 4.0)).disposition,
            zju::coop::UpdateDisposition::Accepted);
}

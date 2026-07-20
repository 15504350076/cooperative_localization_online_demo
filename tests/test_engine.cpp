#include "core/engine.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace {

using zju::coop::EdgeKey;
using zju::coop::Engine;
using zju::coop::EngineConfig;
using zju::coop::FusionAction;
using zju::coop::ImuDisposition;
using zju::coop::ImuPacket;
using zju::coop::InertialConfig;
using zju::coop::InertialNodeInitialization;
using zju::coop::LocalizationState;
using zju::coop::NodeInitialization;
using zju::coop::ObservationState;
using zju::coop::ProcessingDisposition;
using zju::coop::RangePacket;
using zju::coop::ReasonMask;
using zju::coop::UpdateDisposition;

constexpr std::uint64_t kStep = 50'000'000ULL;
constexpr std::uint64_t kWindow = 2'000'000'000ULL;

std::vector<NodeInitialization> triangle(bool collinear = false) {
  return {
      {10U, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1},
      {42U, 3.0, 0.0, 0.0, 0.0, 0.1, 0.1},
      {7U, collinear ? 6.0 : 0.0, collinear ? 0.0 : 4.0,
       0.0, 0.0, 0.1, 0.1},
  };
}

EngineConfig config(std::vector<NodeInitialization> nodes = triangle()) {
  EngineConfig value{};
  value.filter = {10U, 0.2, 1.0e9, 0.25, 1.0e-12};
  value.nodes = std::move(nodes);
  value.edge_timeout_ns = 200'000'000ULL;
  value.rigidity_tolerance = 1.0e-9;
  value.degradation.suspend_duration_ns = 500'000'000ULL;
  value.degradation.reject_duration_ns = 1'000'000'000ULL;
  value.degradation.recovery_duration_ns = 500'000'000ULL;
  return value;
}

RangePacket packet(std::uint16_t from, std::uint16_t to, double range,
                   std::uint64_t timestamp, bool valid = true,
                   bool nlos = false) {
  RangePacket value{};
  value.from_node = from;
  value.to_node = to;
  value.timestamp_ns = timestamp;
  value.receive_timestamp_ns = timestamp;
  value.range_m = range;
  value.range_std_m = 0.1;
  value.valid = valid;
  value.nlos_flag = nlos;
  return value;
}

void fill_edge(Engine& engine, std::uint16_t from, std::uint16_t to,
               double range, unsigned int nlos_count = 0U) {
  static_cast<void>(engine.step(0U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    static_cast<void>(engine.push_range(
        packet(from, to, range, (index + 1U) * kStep, true,
               index < nlos_count)));
  }
}

void fill_triangle(Engine& engine, bool collinear = false) {
  static_cast<void>(engine.step(0U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    const auto timestamp = (index + 1U) * kStep;
    static_cast<void>(
        engine.push_range(packet(10U, 42U, 3.0, timestamp)));
    static_cast<void>(engine.push_range(
        packet(10U, 7U, collinear ? 6.0 : 4.0, timestamp)));
    static_cast<void>(engine.push_range(
        packet(42U, 7U, collinear ? 3.0 : 5.0, timestamp)));
  }
}

std::vector<InertialNodeInitialization> inertial_triangle() {
  std::vector<InertialNodeInitialization> result;
  for (const auto& node : triangle()) {
    InertialNodeInitialization initialization{};
    initialization.node_id = node.node_id;
    initialization.position_n_m = {node.x, node.y, 0.0};
    initialization.velocity_n_mps = {node.vx, node.vy, 0.0};
    result.push_back(initialization);
  }
  return result;
}

ImuPacket imu_packet(std::uint32_t node_id, std::uint64_t sequence,
                     std::uint64_t timestamp_ns) {
  ImuPacket packet{};
  packet.node_id = node_id;
  packet.sequence = sequence;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns;
  packet.linear_acceleration_m_s2 = {0.0, 0.0, 9.80665};
  const char* frame = "imu_link";
  std::copy(frame, frame + std::strlen(frame) + 1U, packet.frame_id.begin());
  packet.valid = true;
  return packet;
}

}  // namespace

TEST_CASE(engine_complete_triangle_is_connected_and_observable) {
  Engine engine(config());
  fill_triangle(engine);
  const auto snapshot = engine.step(kWindow);
  EXPECT_EQ(snapshot.network.node_count, 3U);
  EXPECT_EQ(snapshot.network.reachable_node_count, 3U);
  EXPECT_EQ(snapshot.network.active_edge_count, 3U);
  EXPECT_TRUE(snapshot.network.connected);
  EXPECT_TRUE(snapshot.network.observable);
  EXPECT_EQ(snapshot.network.state, LocalizationState::kNormal);
  EXPECT_EQ(snapshot.observations.size(), 3U);
}

TEST_CASE(engine_missing_peer_edge_and_collinear_graph_are_unobservable) {
  Engine missing(config());
  fill_edge(missing, 10U, 42U, 3.0);
  fill_edge(missing, 10U, 7U, 4.0);
  const auto missing_snapshot = missing.step(kWindow);
  EXPECT_TRUE(missing_snapshot.network.connected);
  EXPECT_FALSE(missing_snapshot.network.observable);
  EXPECT_EQ(missing_snapshot.network.active_edge_count, 2U);
  EXPECT_TRUE(zju::coop::has_reason(
      missing_snapshot.network.reason_mask,
      ReasonMask::GRAPH_GEOMETRY_DEGENERATE));

  Engine collinear(config(triangle(true)));
  fill_triangle(collinear, true);
  const auto collinear_snapshot = collinear.step(kWindow);
  EXPECT_TRUE(collinear_snapshot.network.connected);
  EXPECT_FALSE(collinear_snapshot.network.observable);
  EXPECT_TRUE(zju::coop::has_reason(
      collinear_snapshot.network.reason_mask,
      ReasonMask::GRAPH_GEOMETRY_DEGENERATE));
}

TEST_CASE(engine_times_out_previously_valid_edges) {
  Engine engine(config());
  fill_triangle(engine);
  const auto snapshot = engine.step(kWindow + 200'000'001ULL);
  EXPECT_EQ(snapshot.network.active_edge_count, 0U);
  EXPECT_FALSE(snapshot.network.connected);
  EXPECT_FALSE(snapshot.network.observable);
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::LINK_TIMEOUT));
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::NODE_UNREACHABLE));
}

TEST_CASE(engine_packet_nlos_and_degraded_action_scale_ekf_covariance) {
  auto configured = config();
  configured.degradation.nlos_covariance_scale = 7.0;
  Engine engine(configured);
  fill_edge(engine, 10U, 42U, 3.0, 12U);
  static_cast<void>(engine.step(kWindow));

  auto nlos_packet = packet(10U, 42U, 3.0, kWindow, true, true);
  nlos_packet.sequence = 1U;
  const auto processed = engine.push_range(nlos_packet);
  EXPECT_EQ(processed.action, FusionAction::kUseDownweighted);
  EXPECT_TRUE(processed.filter_updated);
  EXPECT_EQ(processed.update.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(std::abs(processed.update.covariance_scale - 7.0) < 1.0e-12);
}

TEST_CASE(engine_nis_rejection_sets_residual_reason_without_double_packet) {
  auto configured = config();
  configured.filter.nis_gate = 0.01;
  Engine engine(configured);
  fill_edge(engine, 10U, 42U, 3.0);
  static_cast<void>(engine.step(kWindow));
  auto outlier = packet(10U, 42U, 30.0, kWindow);
  outlier.sequence = 1U;
  const auto processed = engine.push_range(outlier);
  EXPECT_EQ(processed.update.disposition, UpdateDisposition::NisRejected);
  const auto snapshot = engine.step(kWindow);
  const auto quality = snapshot.observations[0U];
  EXPECT_EQ(quality.received_count, 41U);
  EXPECT_EQ(quality.residual_rejected_count, 1U);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::RANGE_RESIDUAL_HIGH));
}

TEST_CASE(engine_localization_output_marks_yaw_and_altitude_invalid) {
  Engine engine(config());
  const auto snapshot = engine.step(123U);
  EXPECT_EQ(snapshot.localizations.size(), 3U);
  for (const auto& localization : snapshot.localizations) {
    EXPECT_EQ(localization.reference_node_id, 10U);
    EXPECT_EQ(localization.timestamp_ns, 123U);
    EXPECT_TRUE(localization.valid);
    EXPECT_FALSE(localization.yaw_valid);
    EXPECT_FALSE(localization.z_valid);
  }
}

TEST_CASE(engine_tracks_dynamic_complete_graph_not_three_hardcoded_edges) {
  auto nodes = triangle();
  nodes.push_back({99U, 2.0, 2.0, 0.0, 0.0, 0.1, 0.1});
  Engine engine(config(nodes));
  fill_edge(engine, 10U, 42U, 3.0);
  fill_edge(engine, 10U, 7U, 4.0);
  fill_edge(engine, 42U, 7U, 5.0);
  fill_edge(engine, 10U, 99U, std::sqrt(8.0));
  fill_edge(engine, 42U, 99U, std::sqrt(5.0));
  fill_edge(engine, 7U, 99U, std::sqrt(8.0));
  const auto snapshot = engine.step(kWindow);
  EXPECT_EQ(snapshot.network.node_count, 4U);
  EXPECT_EQ(snapshot.network.active_edge_count, 6U);
  EXPECT_EQ(snapshot.observations.size(), 6U);
  EXPECT_TRUE(snapshot.network.connected);
  EXPECT_TRUE(snapshot.network.observable);
}

TEST_CASE(engine_invalid_and_unknown_packets_do_not_pollute_edges) {
  Engine engine(config());
  static_cast<void>(engine.push_range(packet(10U, 42U, 3.0, 1U, false)));
  static_cast<void>(engine.push_range(packet(10U, 99U, 3.0, 2U)));
  const auto snapshot = engine.step(3U);
  EXPECT_EQ(snapshot.network.active_edge_count, 0U);
  std::size_t received_total = 0U;
  std::size_t valid_total = 0U;
  for (const auto& quality : snapshot.observations) {
    received_total += quality.received_count;
    valid_total += quality.valid_count;
    EXPECT_EQ(quality.state, ObservationState::kUnknown);
  }
  EXPECT_EQ(received_total, 1U);
  EXPECT_EQ(valid_total, 0U);
}

TEST_CASE(engine_out_of_order_valid_packet_does_not_rewind_link_freshness) {
  auto configured = config();
  configured.edge_timeout_ns = 75U;
  Engine engine(configured);
  static_cast<void>(engine.push_range(packet(10U, 42U, 3.0, 100U)));
  const auto out_of_order =
      engine.push_range(packet(42U, 10U, 3.0, 50U));

  const auto snapshot = engine.step(150U);
  EXPECT_EQ(snapshot.network.active_edge_count, 1U);
  EXPECT_FALSE(zju::coop::has_reason(snapshot.network.reason_mask,
                                     ReasonMask::LINK_TIMEOUT));
  EXPECT_EQ(out_of_order.update.disposition, UpdateDisposition::OutOfOrder);
  EXPECT_FALSE(out_of_order.filter_updated);
  const auto quality = engine.monitor().quality(EdgeKey(10U, 42U));
  EXPECT_EQ(quality.received_count, 1U);
}

TEST_CASE(engine_rejects_node_ids_not_representable_by_range_protocol) {
  auto nodes = triangle();
  nodes[1U].node_id = 70'000U;
  bool rejected_node = false;
  try {
    static_cast<void>(Engine(config(nodes)));
  } catch (const std::invalid_argument&) {
    rejected_node = true;
  }
  EXPECT_TRUE(rejected_node);
}

TEST_CASE(engine_first_real_timestamp_starts_all_pretracked_edges) {
  constexpr std::uint64_t start = 100'000'000'000ULL;
  Engine engine(config());
  static_cast<void>(engine.push_range(packet(10U, 42U, 3.0, start)));

  auto snapshot = engine.step(start + kWindow - 1U);
  for (const auto& quality : snapshot.observations) {
    EXPECT_EQ(quality.state, ObservationState::kUnknown);
  }

  snapshot = engine.step(start + kWindow);
  EXPECT_EQ(snapshot.observations.size(), 3U);
  for (const auto& quality : snapshot.observations) {
    EXPECT_EQ(quality.state, ObservationState::kDegraded);
  }
}

TEST_CASE(engine_counts_known_invalid_packets_for_quality_but_not_ekf) {
  auto configured = config();
  configured.degradation.valid_ratio_threshold = 0.70;
  configured.degradation.rate_ratio_threshold = 0.90;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    const bool valid = index < 32U;
    const auto result = engine.push_range(
        packet(10U, 42U, 3.0, (index + 1U) * kStep, valid));
    if (!valid) {
      EXPECT_FALSE(result.filter_updated);
      EXPECT_EQ(result.update.disposition, UpdateDisposition::InvalidPacket);
    }
  }

  const auto quality = engine.monitor().quality(EdgeKey(10U, 42U));
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.valid_count, 32U);
  EXPECT_EQ(quality.actual_rate_hz, 20.0);
  EXPECT_EQ(quality.state, ObservationState::kNormal);
}

TEST_CASE(engine_rejects_time_skew_without_overflow_and_clears_active_fault) {
  auto configured = config();
  configured.max_future_skew_ns = 10U;
  configured.max_receive_delay_ns = 10U;
  Engine engine(configured);

  auto future = packet(10U, 42U, 3.0,
                       std::numeric_limits<std::uint64_t>::max());
  future.receive_timestamp_ns = 1U;
  const auto future_result = engine.push_range(future);
  EXPECT_EQ(future_result.disposition,
            ProcessingDisposition::TimeRejected);
  auto snapshot = engine.step(1U);
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::TIME_SYNC_TIMEOUT));
  EXPECT_EQ(engine.monitor().quality(EdgeKey(10U, 42U)).received_count, 0U);
  EXPECT_EQ(snapshot.network.active_edge_count, 0U);

  auto stale_packet = packet(10U, 42U, 3.0, 0U);
  stale_packet.receive_timestamp_ns = 1U;
  const auto stale = engine.push_range(stale_packet);
  EXPECT_EQ(stale.disposition, ProcessingDisposition::OutOfOrder);
  snapshot = engine.step(1U);
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::TIME_SYNC_TIMEOUT));

  auto delayed = packet(10U, 42U, 3.0, 2U);
  delayed.receive_timestamp_ns =
      std::numeric_limits<std::uint64_t>::max();
  const auto delayed_result = engine.push_range(delayed);
  EXPECT_EQ(delayed_result.disposition,
            ProcessingDisposition::TimeRejected);

  const auto normal_result =
      engine.push_range(packet(10U, 42U, 3.0, 2U));
  EXPECT_EQ(normal_result.disposition, ProcessingDisposition::Processed);
  snapshot = engine.step(2U);
  EXPECT_FALSE(zju::coop::has_reason(snapshot.network.reason_mask,
                                     ReasonMask::TIME_SYNC_TIMEOUT));
  EXPECT_EQ(snapshot.network.active_edge_count, 1U);
}

TEST_CASE(network_aggregates_quality_reason_and_stays_degraded_for_suspension) {
  auto nodes = triangle();
  nodes.push_back({99U, 2.0, 2.0, 0.0, 0.0, 0.1, 0.1});
  auto configured = config(nodes);
  configured.edge_timeout_ns = 1'000'000'000ULL;
  configured.degradation.valid_ratio_threshold = 0.0;
  configured.degradation.rate_ratio_threshold = 0.0;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    const auto timestamp = (index + 1U) * kStep;
    const bool nlos = index >= 28U;
    static_cast<void>(
        engine.push_range(packet(10U, 42U, 3.0, timestamp, true, nlos)));
    static_cast<void>(engine.push_range(packet(10U, 7U, 4.0, timestamp)));
    static_cast<void>(engine.push_range(
        packet(42U, 7U, 5.0, timestamp)));
    static_cast<void>(engine.push_range(
        packet(10U, 99U, std::sqrt(8.0), timestamp)));
    static_cast<void>(engine.push_range(
        packet(42U, 99U, std::sqrt(5.0), timestamp)));
    static_cast<void>(engine.push_range(
        packet(7U, 99U, std::sqrt(8.0), timestamp)));
  }

  const auto snapshot = engine.step(kWindow + 500'000'000ULL);
  EXPECT_TRUE(snapshot.network.observable);
  EXPECT_EQ(snapshot.network.active_edge_count, 5U);
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::NLOS_RATIO_HIGH));
  EXPECT_EQ(snapshot.network.state, LocalizationState::kDegraded);
}

TEST_CASE(engine_rejects_exact_directed_duplicates_before_state_changes) {
  auto configured = config();
  configured.duplicate_cache_per_link = 2U;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));

  auto first = packet(10U, 42U, 3.0, 1U);
  first.sequence = 7U;
  EXPECT_EQ(engine.push_range(first).disposition,
            ProcessingDisposition::Processed);
  const auto duplicate = engine.push_range(first);
  EXPECT_EQ(duplicate.disposition, ProcessingDisposition::Duplicate);
  EXPECT_FALSE(duplicate.filter_updated);
  EXPECT_EQ(engine.monitor().quality(EdgeKey(10U, 42U)).received_count, 1U);

  auto restarted = first;
  restarted.timestamp_ns = 2U;
  restarted.receive_timestamp_ns = 2U;
  EXPECT_EQ(engine.push_range(restarted).disposition,
            ProcessingDisposition::Processed);

  auto reverse = restarted;
  reverse.from_node = 42U;
  reverse.to_node = 10U;
  EXPECT_EQ(engine.push_range(reverse).disposition,
            ProcessingDisposition::Processed);
  EXPECT_EQ(engine.monitor().quality(EdgeKey(10U, 42U)).received_count, 3U);
}

TEST_CASE(engine_rejects_excessive_resources_before_filter_construction) {
  auto configured = config();
  configured.max_nodes = 64U;
  configured.max_edges = 2016U;
  configured.max_state_dimension = 252U;
  configured.nodes.clear();
  configured.nodes.reserve(5000U);
  configured.filter.reference_node_id = 0U;
  for (std::uint32_t node_id = 0U; node_id < 5000U; ++node_id) {
    configured.nodes.push_back(
        {node_id, static_cast<double>(node_id), 0.0, 0.0, 0.0, 0.1, 0.1});
  }

  bool rejected = false;
  try {
    static_cast<void>(Engine(std::move(configured)));
  } catch (const std::invalid_argument&) {
    rejected = true;
  } catch (...) {
    rejected = false;
  }
  EXPECT_TRUE(rejected);
}

TEST_CASE(engine_rejects_missing_receive_time_before_any_state_change) {
  auto configured = config();
  configured.max_future_skew_ns = 10U;
  configured.max_receive_delay_ns = 10U;
  Engine engine(configured);

  auto missing_receive = packet(
      10U, 42U, 3.0, std::numeric_limits<std::uint64_t>::max());
  missing_receive.receive_timestamp_ns = 0U;
  const auto rejected = engine.push_range(missing_receive);
  EXPECT_EQ(rejected.disposition, ProcessingDisposition::TimeRejected);
  EXPECT_EQ(engine.monitor().quality(EdgeKey(10U, 42U)).received_count, 0U);

  const auto normal = engine.push_range(packet(10U, 42U, 3.0, 1U));
  EXPECT_EQ(normal.disposition, ProcessingDisposition::Processed);
  EXPECT_EQ(engine.monitor().quality(EdgeKey(10U, 42U)).received_count, 1U);

  Engine zero_timestamp(configured);
  auto at_zero = packet(10U, 42U, 3.0, 0U);
  at_zero.receive_timestamp_ns = 1U;
  EXPECT_EQ(zero_timestamp.push_range(at_zero).disposition,
            ProcessingDisposition::Processed);
}

TEST_CASE(engine_step_failure_does_not_commit_global_timebase) {
  auto configured = config();
  configured.filter.process_accel_std_mps2 = 1.0e150;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));

  bool failed_as_expected = false;
  try {
    static_cast<void>(
        engine.step(std::numeric_limits<std::uint64_t>::max()));
  } catch (const std::exception&) {
    failed_as_expected = true;
  }
  EXPECT_TRUE(failed_as_expected);

  bool recovered = true;
  std::uint64_t recovered_timestamp = 0U;
  try {
    recovered_timestamp = engine.step(1U).timestamp_ns;
  } catch (const std::exception&) {
    recovered = false;
  }
  EXPECT_TRUE(recovered);
  EXPECT_EQ(recovered_timestamp, 1U);
}

TEST_CASE(engine_keeps_uwb_only_default_and_supports_optional_inertial_path) {
  Engine engine(config());
  EXPECT_FALSE(engine.inertial_enabled());

  InertialConfig inertial{};
  inertial.expected_frame_id = "imu_link";
  engine.configure_inertial(inertial, inertial_triangle(), 300U);
  EXPECT_TRUE(engine.inertial_enabled());

  const auto first =
      engine.push_imu(imu_packet(42U, 1U, 1'000'000'000ULL));
  const auto second =
      engine.push_imu(imu_packet(42U, 2U, 1'010'000'000ULL));
  EXPECT_EQ(first.disposition, ImuDisposition::kBaselineEstablished);
  EXPECT_EQ(second.disposition, ImuDisposition::kPropagated);

  auto uwb = packet(10U, 42U, 2.9, 1'010'000'000ULL);
  uwb.sequence = 1U;
  const auto update = engine.push_range(uwb);
  EXPECT_EQ(update.disposition, ProcessingDisposition::Processed);
  EXPECT_TRUE(update.filter_updated);
  EXPECT_TRUE(engine.inertial_filter() != nullptr);
}

TEST_CASE(engine_inertial_step_does_not_apply_constant_velocity_prediction) {
  Engine engine(config());
  InertialConfig inertial{};
  inertial.expected_frame_id = "imu_link";
  auto initializations = inertial_triangle();
  initializations[1].velocity_n_mps = {5.0, 0.0, 0.0};
  engine.configure_inertial(inertial, initializations, 300U);

  const double before = engine.inertial_filter()->state(42U).position_n_m.x;
  static_cast<void>(engine.step(10'000'000'000ULL));
  const double after = engine.inertial_filter()->state(42U).position_n_m.x;

  EXPECT_EQ(after, before);
}

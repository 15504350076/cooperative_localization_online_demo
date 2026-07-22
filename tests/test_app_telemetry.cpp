// 模块职责：验证应用适配层把算法快照编码为GCS帧，并正确维护告警激活/恢复生命周期。
#include "apps/app_support.hpp"
#include "protocol/wire_protocol.hpp"
#include "test_support.hpp"

#include <cstdint>

namespace {

zju_coop_network_t network(std::uint64_t timestamp_ns,
                           zju_coop_localization_state_t state,
                           std::uint32_t reason_mask) {
  zju_coop_network_t result{};
  EXPECT_EQ(zju_coop_network_init(&result), ZJU_COOP_OK);
  result.timestamp_ns = timestamp_ns;
  result.node_count = 3U;
  result.reachable_node_count =
      state == ZJU_COOP_LOCALIZATION_NORMAL ? 3U : 1U;
  result.active_edge_count =
      state == ZJU_COOP_LOCALIZATION_NORMAL ? 3U : 0U;
  result.connected =
      state == ZJU_COOP_LOCALIZATION_NORMAL ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  result.observable = result.connected;
  result.reason_mask = reason_mask;
  result.state = state;
  return result;
}

}  // namespace

TEST_CASE(app_telemetry_emits_status_each_tick_and_tracks_alert_lifecycle) {
  zju::coop::apps::TelemetryEncoder telemetry;
  zju::coop::apps::TelemetryCounters counters{};
  counters.accepted_ranges = 11U;
  counters.rejected_ranges = 2U;
  counters.protocol_errors = 3U;
  std::uint64_t sequence = 1U;

  const auto normal = telemetry.encode(
      network(100U, ZJU_COOP_LOCALIZATION_NORMAL, 0U), counters, 50U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(normal.size(), 1U);
  auto frame = zju::coop::protocol::decode_frame(normal[0U].bytes);
  EXPECT_TRUE(frame.ok());
  EXPECT_EQ(frame.value.header.message_type,
            zju::coop::protocol::MessageType::kAlgorithmStatus);
  auto status = zju::coop::protocol::decode_algorithm_status_payload(
      frame.value.payload);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(status.value.run_state,
            zju::coop::protocol::AlgorithmRunState::kRunning);
  EXPECT_EQ(status.value.accepted_ranges, 11U);
  EXPECT_EQ(status.value.rejected_ranges, 2U);
  EXPECT_EQ(status.value.protocol_errors, 3U);
  EXPECT_EQ(status.value.uptime_ns, 50U);

  const auto first_fault = telemetry.encode(
      network(200U, ZJU_COOP_LOCALIZATION_UNOBSERVABLE, 0x20U), counters,
      150U, 1U, sequence,
      zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(first_fault.size(), 2U);
  frame = zju::coop::protocol::decode_frame(first_fault[1U].bytes);
  auto alert = zju::coop::protocol::decode_alert_payload(frame.value.payload);
  EXPECT_TRUE(alert.ok());
  EXPECT_EQ(alert.value.lifecycle,
            zju::coop::protocol::AlertLifecycle::kActive);
  EXPECT_EQ(alert.value.reason_mask, 0x20U);
  EXPECT_EQ(alert.value.first_timestamp_ns, 200U);
  EXPECT_EQ(alert.value.last_timestamp_ns, 200U);

  const auto updated_fault = telemetry.encode(
      network(300U, ZJU_COOP_LOCALIZATION_DEGRADED, 0x04U), counters, 250U,
      1U, sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(updated_fault.size(), 2U);
  frame = zju::coop::protocol::decode_frame(updated_fault[1U].bytes);
  alert = zju::coop::protocol::decode_alert_payload(frame.value.payload);
  EXPECT_EQ(alert.value.first_timestamp_ns, 200U);
  EXPECT_EQ(alert.value.last_timestamp_ns, 300U);
  EXPECT_EQ(alert.value.reason_mask, 0x04U);

  const auto recovery = telemetry.encode(
      network(400U, ZJU_COOP_LOCALIZATION_NORMAL, 0U), counters, 350U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(recovery.size(), 2U);
  frame = zju::coop::protocol::decode_frame(recovery[1U].bytes);
  alert = zju::coop::protocol::decode_alert_payload(frame.value.payload);
  EXPECT_EQ(alert.value.lifecycle,
            zju::coop::protocol::AlertLifecycle::kCleared);
  EXPECT_EQ(alert.value.reason_mask, 0U);
  EXPECT_EQ(alert.value.first_timestamp_ns, 200U);
  EXPECT_EQ(alert.value.last_timestamp_ns, 400U);

  const auto steady_normal = telemetry.encode(
      network(500U, ZJU_COOP_LOCALIZATION_NORMAL, 0U), counters, 450U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(steady_normal.size(), 1U);
}

TEST_CASE(app_telemetry_maps_initializing_and_degraded_network_states) {
  zju::coop::apps::TelemetryEncoder telemetry;
  std::uint64_t sequence = 1U;
  const zju::coop::apps::TelemetryCounters counters{};
  const auto initializing = telemetry.encode(
      network(1U, ZJU_COOP_LOCALIZATION_UNINITIALIZED, 0U), counters, 0U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  auto frame = zju::coop::protocol::decode_frame(initializing[0U].bytes);
  auto status = zju::coop::protocol::decode_algorithm_status_payload(
      frame.value.payload);
  EXPECT_EQ(status.value.run_state,
            zju::coop::protocol::AlgorithmRunState::kInitializing);

  const auto degraded = telemetry.encode(
      network(2U, ZJU_COOP_LOCALIZATION_STALE, 0x20U), counters, 1U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  frame = zju::coop::protocol::decode_frame(degraded[0U].bytes);
  status = zju::coop::protocol::decode_algorithm_status_payload(
      frame.value.payload);
  EXPECT_EQ(status.value.run_state,
            zju::coop::protocol::AlgorithmRunState::kDegraded);
}

TEST_CASE(app_elapsed_ns_saturates_when_a_log_timestamp_moves_backwards) {
  EXPECT_EQ(zju::coop::apps::elapsed_ns(150U, 100U), 50U);
  EXPECT_EQ(zju::coop::apps::elapsed_ns(100U, 100U), 0U);
  EXPECT_EQ(zju::coop::apps::elapsed_ns(90U, 100U), 0U);
}

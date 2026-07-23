// 模块职责：验证应用适配层把算法快照编码为GCS帧，并正确维护告警激活/恢复生命周期。
#include "apps/app_support.hpp"
#include "protocol/wire_protocol.hpp"
#include "test_support.hpp"

#include <cstdint>

namespace {

// timestamp_ns/state/reason_mask分别指定网络快照时刻、定位状态和退化原因位，均按值写入本次构造结果。
zju_coop_network_t network(std::uint64_t timestamp_ns,
                           zju_coop_localization_state_t state,
                           std::uint32_t reason_mask) {
  // result：经C ABI初始化的三节点网络输入快照，连通性与边数随state切换。
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

// 状态每周期发布，告警first时间跨活动期保持，恢复时生成Cleared生命周期。
TEST_CASE(app_telemetry_emits_status_each_tick_and_tracks_alert_lifecycle) {
  // telemetry：跨多个周期保存告警首发状态；counters：注入接受/拒绝/协议错误计数；sequence：由编码器递增的帧序号。
  zju::coop::apps::TelemetryEncoder telemetry;
  zju::coop::apps::TelemetryCounters counters{};
  counters.accepted_ranges = 11U;
  counters.rejected_ranges = 2U;
  counters.protocol_errors = 3U;
  std::uint64_t sequence = 1U;

  // normal：正常周期只应产生状态帧；frame/status：依次保存解帧和状态载荷解析结果供字段核对。
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

  // first_fault：首次不可观周期应产生状态帧与Active告警；alert：复用的告警载荷解析结果。
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

  // updated_fault：持续故障的第二个周期，期望保留首次时间并刷新末次时间与原因位。
  const auto updated_fault = telemetry.encode(
      network(300U, ZJU_COOP_LOCALIZATION_DEGRADED, 0x04U), counters, 250U,
      1U, sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(updated_fault.size(), 2U);
  frame = zju::coop::protocol::decode_frame(updated_fault[1U].bytes);
  alert = zju::coop::protocol::decode_alert_payload(frame.value.payload);
  EXPECT_EQ(alert.value.first_timestamp_ns, 200U);
  EXPECT_EQ(alert.value.last_timestamp_ns, 300U);
  EXPECT_EQ(alert.value.reason_mask, 0x04U);

  // recovery：恢复正常周期，期望额外产生Cleared告警并闭合原故障时间区间。
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

  // steady_normal：清除后的稳定正常周期，期望重新只发布单个状态帧。
  const auto steady_normal = telemetry.encode(
      network(500U, ZJU_COOP_LOCALIZATION_NORMAL, 0U), counters, 450U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  EXPECT_EQ(steady_normal.size(), 1U);
}

TEST_CASE(app_telemetry_maps_initializing_and_degraded_network_states) {
  // telemetry：独立状态映射编码器；sequence：输出帧序号；counters：保持全零以隔离网络状态映射。
  zju::coop::apps::TelemetryEncoder telemetry;
  std::uint64_t sequence = 1U;
  const zju::coop::apps::TelemetryCounters counters{};
  // initializing：未初始化网络的输出帧集合；frame/status：复用以读取初始化与退化两类运行状态。
  const auto initializing = telemetry.encode(
      network(1U, ZJU_COOP_LOCALIZATION_UNINITIALIZED, 0U), counters, 0U, 1U,
      sequence, zju::coop::protocol::kDefaultMaxPayloadSize);
  auto frame = zju::coop::protocol::decode_frame(initializing[0U].bytes);
  auto status = zju::coop::protocol::decode_algorithm_status_payload(
      frame.value.payload);
  EXPECT_EQ(status.value.run_state,
            zju::coop::protocol::AlgorithmRunState::kInitializing);

  // degraded：陈旧且带原因位的网络输出，期望状态载荷映射为kDegraded。
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

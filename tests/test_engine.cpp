// 模块职责：验证Engine从输入检查、重复/时间拒绝、质量动作、滤波到动态图快照的编排闭环。
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

// kStep：20 Hz测距输入的50 ms间隔；kWindow：质量监视器达到成熟判定的2 s窗口。
constexpr std::uint64_t kStep = 50'000'000ULL;
constexpr std::uint64_t kWindow = 2'000'000'000ULL;

// collinear控制第三节点是否落在x轴上，用同一工厂生成可观与几何退化布局。
std::vector<NodeInitialization> triangle(bool collinear = false) {
  return {
      {10U, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1},
      {42U, 3.0, 0.0, 0.0, 0.0, 0.1, 0.1},
      {7U, collinear ? 6.0 : 0.0, collinear ? 0.0 : 4.0,
       0.0, 0.0, 0.1, 0.1},
  };
}

// nodes按值接收并移入配置，允许调用方提供三角形或扩展节点集合。
EngineConfig config(std::vector<NodeInitialization> nodes = triangle()) {
  // value：主参考10、宽松NIS和缩短退化时长的引擎测试配置。
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

// from/to指定测距边，range/timestamp给出量测与事件时间，valid/nlos控制输入质量分支。
RangePacket packet(std::uint16_t from, std::uint16_t to, double range,
                   std::uint64_t timestamp, bool valid = true,
                   bool nlos = false) {
  // value：发送与接收时间一致、标准差0.1 m的单次测距输入包。
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

// engine保存调用后的滤波与质量状态；from/to/range确定边，nlos_count指定40包中的NLOS前缀长度。
void fill_edge(Engine& engine, std::uint16_t from, std::uint16_t to,
               double range, unsigned int nlos_count = 0U) {
  static_cast<void>(engine.step(0U));
  // index：生成完整2 s窗口的等间隔包序号，并判定是否落入NLOS前缀。
  for (unsigned int index = 0U; index < 40U; ++index) {
    static_cast<void>(engine.push_range(
        packet(from, to, range, (index + 1U) * kStep, true,
               index < nlos_count)));
  }
}

// engine保存三条边的窗口状态，collinear选择3-3-6共线距离或3-4-5非共线距离。
void fill_triangle(Engine& engine, bool collinear = false) {
  static_cast<void>(engine.step(0U));
  // index/timestamp：完整窗口内40个采样序号及对应右端时间，每个时刻写入三条边。
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
  // result：从二维三角布局转换出的三节点惯导初值；node是源节点引用，initialization是当前转换项。
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

// node_id/sequence/timestamp_ns分别指定惯导节点、去重序号和采样时刻。
ImuPacket imu_packet(std::uint32_t node_id, std::uint64_t sequence,
                     std::uint64_t timestamp_ns) {
  // packet：静止比力、期望frame_id的有效IMU包；frame：复制进定长字符缓冲的源字符串。
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

// 正常拓扑组验证活动三边、主参考可达、几何秩和二维有效位来自同一快照。
TEST_CASE(engine_complete_triangle_is_connected_and_observable) {
  // engine：填满三条边窗口的被测编排器；snapshot：2 s时刻的期望正常、连通、可观快照。
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
  // missing/missing_snapshot：仅两条边但仍连通的秩不足场景；collinear/collinear_snapshot：三边共线退化对照。
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

// 质量/超时组确认NLOS降权、NIS残差、边超时和质量原因位会影响动作与网络状态。
TEST_CASE(engine_times_out_previously_valid_edges) {
  // engine：先形成正常三角网络；snapshot：超过200 ms边超时后的断连快照。
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
  // configured：把NLOS协方差倍率设为7；engine：已成熟为30% NLOS的单边状态。
  auto configured = config();
  configured.degradation.nlos_covariance_scale = 7.0;
  Engine engine(configured);
  fill_edge(engine, 10U, 42U, 3.0, 12U);
  static_cast<void>(engine.step(kWindow));

  // nlos_packet：成熟边上的新NLOS包；processed：期望降权并以7倍协方差更新滤波的回执。
  auto nlos_packet = packet(10U, 42U, 3.0, kWindow, true, true);
  nlos_packet.sequence = 1U;
  const auto processed = engine.push_range(nlos_packet);
  EXPECT_EQ(processed.action, FusionAction::kUseDownweighted);
  EXPECT_TRUE(processed.filter_updated);
  EXPECT_EQ(processed.update.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(std::abs(processed.update.covariance_scale - 7.0) < 1.0e-12);
}

TEST_CASE(engine_nis_rejection_sets_residual_reason_without_double_packet) {
  // configured：极严NIS门限；engine：已有40个正常包；outlier/processed：30 m离群包及拒绝回执。
  auto configured = config();
  configured.filter.nis_gate = 0.01;
  Engine engine(configured);
  fill_edge(engine, 10U, 42U, 3.0);
  static_cast<void>(engine.step(kWindow));
  auto outlier = packet(10U, 42U, 30.0, kWindow);
  outlier.sequence = 1U;
  const auto processed = engine.push_range(outlier);
  EXPECT_EQ(processed.update.disposition, UpdateDisposition::NisRejected);
  // snapshot/quality：拒绝后的网络与边质量，期望总接收数只增一次且残差拒绝计数为1。
  const auto snapshot = engine.step(kWindow);
  const auto quality = snapshot.observations[0U];
  EXPECT_EQ(quality.received_count, 41U);
  EXPECT_EQ(quality.residual_rejected_count, 1U);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::RANGE_RESIDUAL_HIGH));
}

TEST_CASE(engine_localization_output_marks_yaw_and_altitude_invalid) {
  // engine/snapshot：未融合任何包的UWB二维输出；localization逐项借用三节点快照核对有效位。
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
  // nodes：在三角布局上追加节点99；engine：填满四节点完全图六条边；snapshot：动态拓扑快照。
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

// 输入防御组覆盖未知节点、乱序、未来/延迟、重复包和缺失接收时间的无副作用拒绝。
TEST_CASE(engine_invalid_and_unknown_packets_do_not_pollute_edges) {
  // engine：依次接收已知无效包和未知节点包；snapshot：应无活动边；received_total/valid_total累计各边计数。
  Engine engine(config());
  static_cast<void>(engine.push_range(packet(10U, 42U, 3.0, 1U, false)));
  static_cast<void>(engine.push_range(packet(10U, 99U, 3.0, 2U)));
  const auto snapshot = engine.step(3U);
  EXPECT_EQ(snapshot.network.active_edge_count, 0U);
  std::size_t received_total = 0U;
  std::size_t valid_total = 0U;
  // quality：逐项借用预跟踪边质量，确认输入拒绝不改变Unknown状态。
  for (const auto& quality : snapshot.observations) {
    received_total += quality.received_count;
    valid_total += quality.valid_count;
    EXPECT_EQ(quality.state, ObservationState::kUnknown);
  }
  EXPECT_EQ(received_total, 1U);
  EXPECT_EQ(valid_total, 0U);
}

TEST_CASE(engine_out_of_order_valid_packet_does_not_rewind_link_freshness) {
  // configured：75 ns边超时配置；engine：先收时刻100再收反向时刻50；out_of_order：乱序回执。
  auto configured = config();
  configured.edge_timeout_ns = 75U;
  Engine engine(configured);
  static_cast<void>(engine.push_range(packet(10U, 42U, 3.0, 100U)));
  const auto out_of_order =
      engine.push_range(packet(42U, 10U, 3.0, 50U));

  // snapshot：时刻150仍应以新鲜时刻100保活；quality：期望乱序包不进入质量计数。
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
  // nodes：把一个节点ID改为超过uint16协议范围；rejected_node：构造器是否按预期拒绝。
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
  // start：远离零的首个真实包时间；engine：已预跟踪完全图；snapshot/quality：暖机前后各边状态。
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
  // configured：放宽质量阈值以隔离计数语义；engine：接收32个有效与8个无效包的实例。
  auto configured = config();
  configured.degradation.valid_ratio_threshold = 0.70;
  configured.degradation.rate_ratio_threshold = 0.90;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));
  // index：40包序号；valid：前32包有效标志；result：逐包处理回执，无效包不得更新EKF。
  for (unsigned int index = 0U; index < 40U; ++index) {
    const bool valid = index < 32U;
    const auto result = engine.push_range(
        packet(10U, 42U, 3.0, (index + 1U) * kStep, valid));
    if (!valid) {
      EXPECT_FALSE(result.filter_updated);
      EXPECT_EQ(result.update.disposition, UpdateDisposition::InvalidPacket);
    }
  }

  // quality：完整窗口边质量，期望接收40、有效32且按放宽阈值保持正常。
  const auto quality = engine.monitor().quality(EdgeKey(10U, 42U));
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.valid_count, 32U);
  EXPECT_EQ(quality.actual_rate_hz, 20.0);
  EXPECT_EQ(quality.state, ObservationState::kNormal);
}

TEST_CASE(engine_rejects_time_skew_without_overflow_and_clears_active_fault) {
  // configured：未来偏差和接收延迟均限10 ns；engine：验证无符号极值时间差的防溢出拒绝。
  auto configured = config();
  configured.max_future_skew_ns = 10U;
  configured.max_receive_delay_ns = 10U;
  Engine engine(configured);

  // future/future_result：采样时间为uint64最大值而接收时间为1的未来包及拒绝回执；snapshot复用观察故障位。
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

  // stale_packet/stale：时间0的乱序包及回执；delayed/delayed_result：接收时间为极大值的过度延迟包及回执。
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

  // normal_result：随后合法包的处理回执，期望清除活动时间同步故障。
  const auto normal_result =
      engine.push_range(packet(10U, 42U, 3.0, 2U));
  EXPECT_EQ(normal_result.disposition, ProcessingDisposition::Processed);
  snapshot = engine.step(2U);
  EXPECT_FALSE(zju::coop::has_reason(snapshot.network.reason_mask,
                                     ReasonMask::TIME_SYNC_TIMEOUT));
  EXPECT_EQ(snapshot.network.active_edge_count, 1U);
}

TEST_CASE(network_aggregates_quality_reason_and_stays_degraded_for_suspension) {
  // nodes/configured：四节点完全图及关闭有效率/频率干扰的长超时配置；engine：仅一边进入NLOS暂缓。
  auto nodes = triangle();
  nodes.push_back({99U, 2.0, 2.0, 0.0, 0.0, 0.1, 0.1});
  auto configured = config(nodes);
  configured.edge_timeout_ns = 1'000'000'000ULL;
  configured.degradation.valid_ratio_threshold = 0.0;
  configured.degradation.rate_ratio_threshold = 0.0;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));
  // index/timestamp/nlos：生成六边完整窗口，只有10-42边最后12包标记NLOS。
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

  // snapshot：暂缓时仍由其余五边保持可观，但网络状态应聚合为Degraded。
  const auto snapshot = engine.step(kWindow + 500'000'000ULL);
  EXPECT_TRUE(snapshot.network.observable);
  EXPECT_EQ(snapshot.network.active_edge_count, 5U);
  EXPECT_TRUE(zju::coop::has_reason(snapshot.network.reason_mask,
                                    ReasonMask::NLOS_RATIO_HIGH));
  EXPECT_EQ(snapshot.network.state, LocalizationState::kDegraded);
}

TEST_CASE(engine_rejects_exact_directed_duplicates_before_state_changes) {
  // configured：每有向链路只缓存两个重复键；engine：被测去重状态；first：序号7的初包。
  auto configured = config();
  configured.duplicate_cache_per_link = 2U;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));

  auto first = packet(10U, 42U, 3.0, 1U);
  first.sequence = 7U;
  EXPECT_EQ(engine.push_range(first).disposition,
            ProcessingDisposition::Processed);
  // duplicate：完全相同有向包的拒绝回执；restarted：同序号新时间包；reverse：反向链路同键包。
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
  // configured：声明上限64节点却装入5000节点的配置；node_id：生成稀疏资源压力项；rejected：是否在滤波器构造前拒绝。
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
  // configured/engine：启用时间防御的实例；missing_receive/rejected：缺失接收时间的极值采样包及回执。
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

  // normal：拒绝后首个合法包回执；zero_timestamp/at_zero：证明采样时间0本身可合法、只要接收时间存在。
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
  // configured：极大过程噪声使超长预测失败；engine：已在时间0建立基线。
  auto configured = config();
  configured.filter.process_accel_std_mps2 = 1.0e150;
  Engine engine(configured);
  static_cast<void>(engine.step(0U));

  // failed_as_expected：极值时间步是否抛异常；recovered/recovered_timestamp：随后1 ns步能否成功及其时间戳。
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

// 模式组锁定“未配置惯性=显式回退、配置惯性=只由真实IMU预测”，防止两模型叠加。
TEST_CASE(engine_keeps_uwb_only_default_and_supports_optional_inertial_path) {
  // engine：先验证默认UWB模式再原位切换惯导；inertial：期望frame_id的单节点传播配置。
  Engine engine(config());
  EXPECT_FALSE(engine.inertial_enabled());

  InertialConfig inertial{};
  inertial.expected_frame_id = "imu_link";
  engine.configure_inertial(inertial, inertial_triangle(), 300U);
  EXPECT_TRUE(engine.inertial_enabled());

  // first/second：节点42的首帧和10 ms后IMU回执；uwb/update：惯导模式下测距包及联合更新回执。
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
  // engine/inertial：启用惯导路径的引擎和坐标系配置；initializations：给节点42注入5 m/s初速。
  Engine engine(config());
  InertialConfig inertial{};
  inertial.expected_frame_id = "imu_link";
  auto initializations = inertial_triangle();
  initializations[1].velocity_n_mps = {5.0, 0.0, 0.0};
  engine.configure_inertial(inertial, initializations, 300U);

  // before/after：无新IMU时调用长时间step前后的节点42 x位置，期望完全相同。
  const double before = engine.inertial_filter()->state(42U).position_n_m.x;
  static_cast<void>(engine.step(10'000'000'000ULL));
  const double after = engine.inertial_filter()->state(42U).position_n_m.x;

  EXPECT_EQ(after, before);
}

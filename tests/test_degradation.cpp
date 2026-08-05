// 模块职责：验证观测质量滑窗、NLOS/频率/有效率证据和降权—暂缓—剔除—恢复状态机。
// C++初学者阅读方法：每个用例连续输入多帧观测，模拟“正常->变差->恢复”的时间过程；
// EXPECT_*检查的不是单帧距离值，而是滑动窗口统计和状态机是否在正确帧数后转换。
#include "core/degradation_monitor.hpp"
#include "test_support.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace {

using zju::coop::DegradationConfig;
using zju::coop::DegradationMonitor;
using zju::coop::EdgeKey;
using zju::coop::FusionAction;
using zju::coop::ObservationState;
using zju::coop::RangePacket;
using zju::coop::ReasonMask;

// kStep：20 Hz输入的50 ms采样间隔；kWindow：成熟质量判定所需的2 s滑窗宽度。
constexpr std::uint64_t kStep = 50'000'000ULL;
constexpr std::uint64_t kWindow = 2'000'000'000ULL;

DegradationConfig config() {
  // value：缩短暂缓、剔除和恢复持续时间的状态机测试配置。
  DegradationConfig value{};
  value.suspend_duration_ns = 500'000'000ULL;
  value.reject_duration_ns = 1'000'000'000ULL;
  value.recovery_duration_ns = 500'000'000ULL;
  return value;
}

// from/to指定无向边端点，timestamp是事件时刻，valid/nlos控制有效率与NLOS证据。
RangePacket packet(std::uint16_t from, std::uint16_t to,
                   std::uint64_t timestamp, bool valid = true,
                   bool nlos = false) {
  // value：固定1 m量测和0.1 m标准差的监视器输入包。
  RangePacket value{};
  value.from_node = from;
  value.to_node = to;
  value.timestamp_ns = timestamp;
  value.range_m = 1.0;
  value.range_std_m = 0.1;
  value.valid = valid;
  value.nlos_flag = nlos;
  return value;
}

// monitor在调用方生命周期内保存窗口；nlos_count/invalid_count指定40包中前缀NLOS数和无效数。
void fill_window(DegradationMonitor& monitor, unsigned int nlos_count = 0U,
                 unsigned int invalid_count = 0U) {
  monitor.advance(0U);
  monitor.track(EdgeKey(70U, 3U));
  // index：生成2 s窗口内40个等间隔样本，并决定其有效/NLOS前缀归属。
  for (unsigned int index = 0U; index < 40U; ++index) {
    monitor.record(packet(70U, 3U, (index + 1U) * kStep,
                          index >= invalid_count, index < nlos_count));
  }
  monitor.advance(kWindow);
}

// value：预期被构造器拒绝的配置副本；返回值记录是否抛出invalid_argument。
bool throws_invalid_config(const DegradationConfig& value) {
  try {
    static_cast<void>(DegradationMonitor(value));
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

}  // namespace

// 边键/配置组保证反向量测共享同一质量窗口，同时保留稀疏uint32节点编号。
TEST_CASE(edge_key_is_undirected_and_supports_sparse_uint32_ids) {
  // forward/reverse：同一稀疏节点边的两个方向输入，期望都归一化为(17, 4000000000)。
  const EdgeKey forward(4'000'000'000U, 17U);
  const EdgeKey reverse(17U, 4'000'000'000U);
  EXPECT_EQ(forward, reverse);
  EXPECT_EQ(forward.first, 17U);
  EXPECT_EQ(forward.second, 4'000'000'000U);
}

TEST_CASE(edge_key_comparison_and_hash_normalize_default_constructed_values) {
  // manually_reversed：手动写成反序的边键；canonical：其规范顺序对照。
  EdgeKey manually_reversed{};
  manually_reversed.first = 70U;
  manually_reversed.second = 3U;
  const EdgeKey canonical(3U, 70U);

  EXPECT_EQ(manually_reversed, canonical);
  EXPECT_EQ(zju::coop::EdgeKeyHash{}(manually_reversed),
            zju::coop::EdgeKeyHash{}(canonical));
}

TEST_CASE(degradation_config_defaults_and_invalid_ranges_are_checked) {
  // defaults：核对公开默认值的基线；invalid：逐项复用并注入NaN、越界比例、非法时长和过高频率。
  const DegradationConfig defaults{};
  EXPECT_EQ(defaults.window_ns, kWindow);
  EXPECT_TRUE(std::abs(defaults.nominal_rate_hz - 20.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(defaults.nlos_ratio_threshold - 0.30) < 1.0e-12);
  EXPECT_TRUE(std::abs(defaults.valid_ratio_threshold - 0.80) < 1.0e-12);
  EXPECT_TRUE(std::abs(defaults.rate_ratio_threshold - 0.80) < 1.0e-12);
  EXPECT_TRUE(defaults.nlos_covariance_scale >= 1.0);

  auto invalid = defaults;
  invalid.nominal_rate_hz = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(throws_invalid_config(invalid));
  invalid = defaults;
  invalid.valid_ratio_threshold = 1.01;
  EXPECT_TRUE(throws_invalid_config(invalid));
  invalid = defaults;
  invalid.nlos_covariance_scale = 0.99;
  EXPECT_TRUE(throws_invalid_config(invalid));
  invalid = defaults;
  invalid.window_ns = 0U;
  EXPECT_TRUE(throws_invalid_config(invalid));
  invalid = defaults;
  invalid.reject_duration_ns = invalid.suspend_duration_ns - 1U;
  EXPECT_TRUE(throws_invalid_config(invalid));
  invalid = defaults;
  invalid.nominal_rate_hz = 1.0e6;
  EXPECT_TRUE(throws_invalid_config(invalid));
}

// 窗口组冻结左右边界、成熟前Unknown、NLOS/有效率/频率和残差计数口径。
TEST_CASE(window_warmup_is_unknown_and_uses_normal_covariance) {
  // monitor：仅填半窗NLOS样本的监视器；index：20个样本序号；quality：成熟前期望Unknown且正常权重的快照。
  DegradationMonitor monitor(config());
  for (unsigned int index = 0U; index < 20U; ++index) {
    monitor.record(packet(3U, 70U, index * kStep, true, true));
  }
  monitor.advance(1'000'000'000ULL);
  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.state, ObservationState::kUnknown);
  EXPECT_EQ(quality.action, FusionAction::kUseNormal);
  EXPECT_EQ(quality.received_count, 20U);
  EXPECT_EQ(quality.covariance_scale, 1.0);
}

TEST_CASE(exactly_thirty_percent_nlos_degrades_mature_window) {
  // monitor：完整40包且12包NLOS的边窗口；quality：期望恰在30%边界进入降权状态。
  DegradationMonitor monitor(config());
  fill_window(monitor, 12U);
  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.valid_count, 40U);
  EXPECT_EQ(quality.nlos_count, 12U);
  EXPECT_EQ(quality.expected_count, 40U);
  EXPECT_TRUE(std::abs(quality.nlos_ratio - 0.30) < 1.0e-12);
  EXPECT_TRUE(std::abs(quality.actual_rate_hz - 20.0) < 1.0e-12);
  EXPECT_EQ(quality.state, ObservationState::kDegraded);
  EXPECT_EQ(quality.action, FusionAction::kUseDownweighted);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::NLOS_RATIO_HIGH));
}

TEST_CASE(thirty_one_valid_of_expected_forty_triggers_low_valid_ratio) {
  // monitor：完整窗口中9包无效的边；quality：31/40有效率低于0.8的状态快照。
  DegradationMonitor monitor(config());
  fill_window(monitor, 0U, 9U);
  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.valid_count, 31U);
  EXPECT_TRUE(quality.valid_ratio < 0.80);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::VALID_RATIO_LOW));
}

TEST_CASE(nlos_probability_at_configured_boundary_counts_as_nlos) {
  // monitor：单包边质量监视器；value：显式NLOS概率等于0.5阈值的输入包。
  DegradationMonitor monitor(config());
  auto value = packet(3U, 70U, 0U);
  value.has_nlos_probability = true;
  value.nlos_probability = 0.5F;
  monitor.record(value);
  monitor.advance(1U);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).nlos_count, 1U);
}

// 状态机组用绝对时间验证降权→暂缓→剔除及试探恢复，调用次数不能替代持续时间。
TEST_CASE(bad_and_good_durations_use_absolute_time_not_call_count) {
  // monitor：先经历完整坏窗再注入恢复好窗的状态机；首个index生成恢复前40包，good_start是好证据起点，第二个index补10包。
  DegradationMonitor monitor(config());
  fill_window(monitor, 12U);
  monitor.advance(kWindow + 499'999'999ULL);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kDegraded);
  monitor.advance(kWindow + 500'000'000ULL);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kSuspended);
  monitor.advance(kWindow + 1'000'000'000ULL);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kRejected);

  for (unsigned int index = 0U; index < 40U; ++index) {
    monitor.record(packet(70U, 3U, kWindow + 1'050'000'000ULL + index * kStep));
  }
  const auto good_start = kWindow + 3'050'000'000ULL;
  monitor.advance(good_start);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kRecovering);
  for (unsigned int index = 0U; index < 10U; ++index) {
    monitor.record(packet(3U, 70U, good_start + (index + 1U) * kStep));
  }
  monitor.advance(good_start + 500'000'000ULL);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kNormal);
}

TEST_CASE(residual_rejection_marks_existing_sample_without_double_counting) {
  // monitor：40个合法样本加一次同时间残差拒绝的窗口；index：样本序号；quality：期望接收数不重复增加。
  DegradationMonitor monitor(config());
  monitor.advance(0U);
  monitor.track(EdgeKey(3U, 70U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    monitor.record(packet(3U, 70U, (index + 1U) * kStep));
  }
  monitor.record_residual_rejection(EdgeKey(70U, 3U), 40U * kStep);
  monitor.advance(kWindow);
  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.residual_rejected_count, 1U);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::RANGE_RESIDUAL_HIGH));
}

TEST_CASE(explicitly_tracked_edge_with_no_packets_degrades_after_full_window) {
  // monitor：显式跟踪但从未收包的边；quality：满窗时应报告零计数、低有效率和低频率。
  DegradationMonitor monitor(config());
  monitor.track(EdgeKey(3U, 70U));
  monitor.advance(0U);
  monitor.advance(kWindow - 1U);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kUnknown);

  monitor.advance(kWindow);
  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.received_count, 0U);
  EXPECT_EQ(quality.valid_count, 0U);
  EXPECT_EQ(quality.state, ObservationState::kDegraded);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::VALID_RATIO_LOW));
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::RATE_LOW));
}

TEST_CASE(mature_window_excludes_left_boundary_and_includes_right_boundary) {
  // monitor：含左边界0和右边界2 s样本的窗口；index：1至40的右闭样本序号；quality：期望计40包。
  DegradationMonitor monitor(config());
  monitor.record(packet(3U, 70U, 0U));
  for (unsigned int index = 1U; index <= 40U; ++index) {
    monitor.record(packet(3U, 70U, index * kStep));
  }
  monitor.advance(kWindow);

  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.window_start_ns, 0U);
  EXPECT_EQ(quality.window_end_ns, kWindow);
  EXPECT_EQ(quality.received_count, 40U);
  EXPECT_EQ(quality.actual_rate_hz, 20.0);
}

// 资源组确认样本缓存有界、溢出原因可见且离开窗口后能够恢复。
TEST_CASE(sample_capacity_overflow_is_bounded_and_reported) {
  // monitor：容量为336样本的单边窗口；index：尝试写入337包；quality：期望保留336包并累计1次丢弃。
  DegradationMonitor monitor(config());
  monitor.track(EdgeKey(3U, 70U));
  monitor.advance(0U);
  for (unsigned int index = 0U; index < 337U; ++index) {
    monitor.record(packet(3U, 70U, 1U));
  }
  monitor.advance(kWindow);

  const auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.received_count, 336U);
  EXPECT_EQ(quality.dropped_count, 1U);
  EXPECT_TRUE(quality.input_overflow);
  EXPECT_TRUE(zju::coop::has_reason(quality.reason_mask,
                                    ReasonMask::INPUT_OVERFLOW));
}

TEST_CASE(pretracked_edges_start_at_first_real_monitor_timestamp) {
  // start：远离零的首个真实监视时刻；monitor：预跟踪有包边和无包边，二者暖机都从start开始。
  constexpr std::uint64_t start = 100'000'000'000ULL;
  DegradationMonitor monitor(config());
  monitor.track(EdgeKey(3U, 70U));
  monitor.track(EdgeKey(3U, 90U));
  monitor.record(packet(3U, 70U, start));

  monitor.advance(start + kWindow - 1U);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kUnknown);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 90U)).state,
            ObservationState::kUnknown);

  monitor.advance(start + kWindow);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kDegraded);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 90U)).state,
            ObservationState::kDegraded);
}

TEST_CASE(direct_monitor_ignores_old_packet_without_advancing_warmup) {
  // start：首个新鲜包时刻；monitor：随后注入时间0旧包；quality：复用以核对旧包不推进暖机也不计NLOS。
  constexpr std::uint64_t start = 100'000'000'000ULL;
  DegradationMonitor monitor(config());
  monitor.record(packet(3U, 70U, start));
  monitor.record(packet(70U, 3U, 0U, false, true));

  monitor.advance(kWindow);
  auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.state, ObservationState::kUnknown);
  EXPECT_EQ(quality.received_count, 0U);

  monitor.advance(start + kWindow - 1U);
  quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.state, ObservationState::kUnknown);
  EXPECT_EQ(quality.received_count, 1U);
  EXPECT_EQ(quality.nlos_count, 0U);
}

TEST_CASE(capacity_overflow_reason_expires_and_link_recovers) {
  // monitor：先制造容量溢出再经历好窗恢复；各index分别生成溢出包、40个好包和10个恢复包。
  DegradationMonitor monitor(config());
  monitor.advance(0U);
  monitor.track(EdgeKey(3U, 70U));
  for (unsigned int index = 0U; index < 337U; ++index) {
    monitor.record(packet(3U, 70U, 1U));
  }
  monitor.advance(kWindow);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).state,
            ObservationState::kDegraded);

  monitor.advance(3'000'000'001ULL);
  for (unsigned int index = 1U; index <= 40U; ++index) {
    monitor.record(packet(3U, 70U, 3'000'000'000ULL + index * kStep));
  }
  monitor.advance(5'000'000'000ULL);
  // quality：恢复阶段质量快照；cumulative_drops：跨窗口保留的历史丢包累计基线。
  auto quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_TRUE(quality.dropped_count >= 1U);
  const std::size_t cumulative_drops = quality.dropped_count;
  EXPECT_FALSE(quality.input_overflow);
  EXPECT_FALSE(zju::coop::has_reason(quality.reason_mask,
                                     ReasonMask::INPUT_OVERFLOW));
  EXPECT_EQ(quality.state, ObservationState::kRecovering);

  for (unsigned int index = 1U; index <= 10U; ++index) {
    monitor.record(
        packet(3U, 70U, 5'000'000'000ULL + index * kStep));
  }
  monitor.advance(5'500'000'000ULL);
  quality = monitor.quality(EdgeKey(3U, 70U));
  EXPECT_EQ(quality.dropped_count, cumulative_drops);
  EXPECT_EQ(quality.state, ObservationState::kNormal);
}

TEST_CASE(degradation_monitor_rejects_more_than_configured_tracked_edges) {
  // configured：把最大跟踪边数限制为1；monitor：已占满容量；rejected：第二条边是否触发参数异常。
  auto configured = config();
  configured.max_tracked_edges = 1U;
  DegradationMonitor monitor(configured);
  monitor.track(EdgeKey(1U, 2U));
  bool rejected = false;
  try {
    monitor.track(EdgeKey(1U, 3U));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT_TRUE(rejected);
}

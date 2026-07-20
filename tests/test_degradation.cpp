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

constexpr std::uint64_t kStep = 50'000'000ULL;
constexpr std::uint64_t kWindow = 2'000'000'000ULL;

DegradationConfig config() {
  DegradationConfig value{};
  value.suspend_duration_ns = 500'000'000ULL;
  value.reject_duration_ns = 1'000'000'000ULL;
  value.recovery_duration_ns = 500'000'000ULL;
  return value;
}

RangePacket packet(std::uint16_t from, std::uint16_t to,
                   std::uint64_t timestamp, bool valid = true,
                   bool nlos = false) {
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

void fill_window(DegradationMonitor& monitor, unsigned int nlos_count = 0U,
                 unsigned int invalid_count = 0U) {
  monitor.advance(0U);
  monitor.track(EdgeKey(70U, 3U));
  for (unsigned int index = 0U; index < 40U; ++index) {
    monitor.record(packet(70U, 3U, (index + 1U) * kStep,
                          index >= invalid_count, index < nlos_count));
  }
  monitor.advance(kWindow);
}

bool throws_invalid_config(const DegradationConfig& value) {
  try {
    static_cast<void>(DegradationMonitor(value));
  } catch (const std::invalid_argument&) {
    return true;
  }
  return false;
}

}  // namespace

TEST_CASE(edge_key_is_undirected_and_supports_sparse_uint32_ids) {
  const EdgeKey forward(4'000'000'000U, 17U);
  const EdgeKey reverse(17U, 4'000'000'000U);
  EXPECT_EQ(forward, reverse);
  EXPECT_EQ(forward.first, 17U);
  EXPECT_EQ(forward.second, 4'000'000'000U);
}

TEST_CASE(edge_key_comparison_and_hash_normalize_default_constructed_values) {
  EdgeKey manually_reversed{};
  manually_reversed.first = 70U;
  manually_reversed.second = 3U;
  const EdgeKey canonical(3U, 70U);

  EXPECT_EQ(manually_reversed, canonical);
  EXPECT_EQ(zju::coop::EdgeKeyHash{}(manually_reversed),
            zju::coop::EdgeKeyHash{}(canonical));
}

TEST_CASE(degradation_config_defaults_and_invalid_ranges_are_checked) {
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

TEST_CASE(window_warmup_is_unknown_and_uses_normal_covariance) {
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
  DegradationMonitor monitor(config());
  auto value = packet(3U, 70U, 0U);
  value.has_nlos_probability = true;
  value.nlos_probability = 0.5F;
  monitor.record(value);
  monitor.advance(1U);
  EXPECT_EQ(monitor.quality(EdgeKey(3U, 70U)).nlos_count, 1U);
}

TEST_CASE(bad_and_good_durations_use_absolute_time_not_call_count) {
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

TEST_CASE(sample_capacity_overflow_is_bounded_and_reported) {
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

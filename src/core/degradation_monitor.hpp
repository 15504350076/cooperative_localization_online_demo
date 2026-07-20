// 观测质量滑窗与状态机接口：正常、降权、暂缓、剔除及试探恢复。
#pragma once

#include "zju_coop/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>

namespace zju::coop {

struct EdgeKey {
  std::uint32_t first{};
  std::uint32_t second{};

  EdgeKey() = default;
  EdgeKey(std::uint32_t node_a, std::uint32_t node_b) noexcept;

  [[nodiscard]] bool operator==(const EdgeKey& other) const noexcept;
  [[nodiscard]] bool operator!=(const EdgeKey& other) const noexcept;
};

struct EdgeKeyHash {
  [[nodiscard]] std::size_t operator()(const EdgeKey& edge) const noexcept;
};

enum class ReasonMask : std::uint32_t {
  NONE = 0U,
  NLOS_RATIO_HIGH = 1U << 0U,
  VALID_RATIO_LOW = 1U << 1U,
  RATE_LOW = 1U << 2U,
  RANGE_RESIDUAL_HIGH = 1U << 3U,
  TIME_SYNC_TIMEOUT = 1U << 4U,
  LINK_TIMEOUT = 1U << 5U,
  GRAPH_GEOMETRY_DEGENERATE = 1U << 6U,
  NODE_UNREACHABLE = 1U << 7U,
  INITIALIZATION_MISSING = 1U << 8U,
  INPUT_OVERFLOW = 1U << 9U,
};

[[nodiscard]] constexpr ReasonMask operator|(ReasonMask left,
                                             ReasonMask right) noexcept {
  return static_cast<ReasonMask>(static_cast<std::uint32_t>(left) |
                                 static_cast<std::uint32_t>(right));
}

constexpr ReasonMask& operator|=(ReasonMask& left, ReasonMask right) noexcept {
  left = left | right;
  return left;
}

[[nodiscard]] constexpr bool has_reason(ReasonMask mask,
                                        ReasonMask reason) noexcept {
  return reason != ReasonMask::NONE &&
         (static_cast<std::uint32_t>(mask) &
          static_cast<std::uint32_t>(reason)) ==
             static_cast<std::uint32_t>(reason);
}

struct DegradationConfig {
  std::uint64_t window_ns{2'000'000'000ULL};
  double nominal_rate_hz{20.0};
  double nlos_ratio_threshold{0.30};
  double valid_ratio_threshold{0.80};
  double rate_ratio_threshold{0.80};
  double nlos_probability_threshold{0.50};
  double nlos_covariance_scale{4.0};
  std::uint64_t suspend_duration_ns{1'000'000'000ULL};
  std::uint64_t reject_duration_ns{3'000'000'000ULL};
  std::uint64_t recovery_duration_ns{1'000'000'000ULL};
  std::size_t max_tracked_edges{2016U};
};

struct ObservationQuality {
  EdgeKey edge{};
  std::uint64_t window_start_ns{};
  std::uint64_t window_end_ns{};
  std::size_t expected_count{};
  std::size_t received_count{};
  std::size_t valid_count{};
  std::size_t nlos_count{};
  std::size_t residual_rejected_count{};
  std::size_t dropped_count{};
  double nlos_ratio{};
  double valid_ratio{};
  double actual_rate_hz{};
  bool input_overflow{};
  ObservationState state{ObservationState::kUnknown};
  FusionAction action{FusionAction::kUseNormal};
  ReasonMask reason_mask{ReasonMask::NONE};
  double covariance_scale{1.0};
};

class DegradationMonitor {
 public:
  explicit DegradationMonitor(DegradationConfig config = {});

  void track(EdgeKey edge);
  void record(const RangePacket& packet);
  void record(EdgeKey edge, std::uint64_t timestamp_ns, bool valid,
              bool nlos_flag = false, bool has_nlos_probability = false,
              double nlos_probability = 0.0);
  void record_residual_rejection(EdgeKey edge,
                                 std::uint64_t timestamp_ns);
  void advance(std::uint64_t now_ns);

  [[nodiscard]] ObservationQuality quality(EdgeKey edge) const;
  [[nodiscard]] const DegradationConfig& config() const noexcept;

 private:
  struct Sample {
    std::uint64_t timestamp_ns{};
    bool valid{};
    bool nlos{};
    bool residual_rejected{};
  };

  struct EdgeRecord {
    std::deque<Sample> samples;
    ObservationQuality quality{};
    std::uint64_t start_timestamp_ns{};
    std::uint64_t latest_processed_timestamp_ns{};
    std::uint64_t bad_since_ns{};
    std::uint64_t good_since_ns{};
    bool started{};
    bool has_latest_processed_timestamp{};
    bool has_bad_since{};
    bool has_good_since{};
    std::size_t dropped_count{};
    std::uint64_t last_overflow_ns{};
    bool has_overflow_event{};
  };

  void evaluate(EdgeRecord& record, std::uint64_t now_ns);
  void start_all(std::uint64_t timestamp_ns);
  [[nodiscard]] FusionAction action_for(ObservationState state) const noexcept;

  DegradationConfig config_;
  std::size_t expected_count_{};
  std::size_t max_samples_{};
  std::uint64_t now_ns_{};
  std::uint64_t start_timestamp_ns_{};
  bool started_{};
  std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> edges_;
};

}  // namespace zju::coop

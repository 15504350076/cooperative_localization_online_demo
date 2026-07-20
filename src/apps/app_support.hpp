// 在线与回放程序共用的 C ABI 会话、输出发布和告警生命周期适配。
#pragma once

#include "config/ini_config.hpp"
#include "protocol/wire_protocol.hpp"
#include "zju_coop/c_api.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace zju::coop::apps {

struct StepSnapshot {
  std::vector<zju_coop_localization_t> localizations;
  std::vector<zju_coop_observation_t> observations;
  zju_coop_network_t network{};
};

class AlgorithmSession {
 public:
  explicit AlgorithmSession(const config::DemoConfig& config);
  ~AlgorithmSession();

  AlgorithmSession(const AlgorithmSession&) = delete;
  AlgorithmSession& operator=(const AlgorithmSession&) = delete;

  [[nodiscard]] zju_coop_range_processing_result_t push_range(
      const protocol::Frame& frame,
      const protocol::RangePayload& payload,
      std::uint64_t receive_timestamp_ns);
  [[nodiscard]] zju_coop_imu_processing_result_t push_imu(
      const protocol::Frame& frame, const protocol::ImuPayload& payload,
      std::uint64_t receive_timestamp_ns);
  [[nodiscard]] StepSnapshot step(std::uint64_t now_ns);

 private:
  zju_coop_handle_t* handle_{};
};

struct EncodedOutput {
  protocol::MessageType message_type{protocol::MessageType::kLocalization};
  std::vector<std::uint8_t> bytes;
};

struct TelemetryCounters {
  std::uint64_t accepted_ranges{};
  std::uint64_t rejected_ranges{};
  std::uint64_t protocol_errors{};
};

class TelemetryEncoder {
 public:
  [[nodiscard]] std::vector<EncodedOutput> encode(
      const zju_coop_network_t& network,
      const TelemetryCounters& counters, std::uint64_t uptime_ns,
      std::uint32_t reference_node_id, std::uint64_t& next_sequence,
      std::size_t max_payload_size,
      protocol::AlgorithmMode mode =
          protocol::AlgorithmMode::kUwbOnlyPlanar);

 private:
  bool alert_active_{};
  std::uint64_t alert_first_timestamp_ns_{};
};

[[nodiscard]] std::vector<EncodedOutput> encode_snapshot(
    const StepSnapshot& snapshot, std::uint32_t reference_node_id,
    std::uint64_t& next_sequence, std::size_t max_payload_size);

[[nodiscard]] std::uint64_t system_time_ns();

[[nodiscard]] std::uint64_t elapsed_ns(
    std::uint64_t timestamp_ns, std::uint64_t start_timestamp_ns) noexcept;

}  // namespace zju::coop::apps

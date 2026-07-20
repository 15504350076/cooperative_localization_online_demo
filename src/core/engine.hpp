// 协同定位总引擎：统一输入校验、质量监测、动态拓扑、滤波和输出快照。
#pragma once

#include "core/cooperative_inertial_ekf.hpp"
#include "core/degradation_monitor.hpp"
#include "core/range_ekf.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace zju::coop {

struct EngineConfig {
  FilterConfig filter{};
  std::vector<NodeInitialization> nodes;
  DegradationConfig degradation{};
  std::uint64_t edge_timeout_ns{500'000'000ULL};
  std::uint64_t max_future_skew_ns{100'000'000ULL};
  std::uint64_t max_receive_delay_ns{500'000'000ULL};
  std::size_t duplicate_cache_per_link{128U};
  std::size_t max_nodes{64U};
  std::size_t max_edges{2016U};
  std::size_t max_state_dimension{252U};
  double rigidity_tolerance{1.0e-9};
};

/** 面向GCS/ROS 2输出的二维主参考相对定位快照。 */

struct LocalizationSnapshot {
  std::uint32_t node_id{};
  std::uint32_t reference_node_id{};
  std::uint64_t timestamp_ns{};
  double x{};
  double y{};
  double vx{};
  double vy{};
  double cov_xx{};
  double cov_xy{};
  double cov_yy{};
  bool valid{};
  bool yaw_valid{};
  bool z_valid{};
  LocalizationState state{LocalizationState::kUninitialized};
};

struct NetworkSnapshot {
  std::uint64_t timestamp_ns{};
  std::size_t node_count{};
  std::size_t reachable_node_count{};
  std::size_t active_edge_count{};
  bool connected{};
  bool observable{};
  ReasonMask reason_mask{ReasonMask::NONE};
  LocalizationState state{LocalizationState::kUninitialized};
};

struct EngineSnapshot {
  std::uint64_t timestamp_ns{};
  std::vector<LocalizationSnapshot> localizations;
  NetworkSnapshot network{};
  std::vector<ObservationQuality> observations;
};

enum class ProcessingDisposition {
  Processed,
  InvalidPacket,
  OutOfOrder,
  TimeRejected,
  Duplicate,
  Held,
  Rejected,
};

struct RangeProcessingResult {
  EdgeKey edge{};
  ProcessingDisposition disposition{ProcessingDisposition::InvalidPacket};
  FusionAction action{FusionAction::kUseNormal};
  bool filter_updated{};
  UpdateResult update{};
};

class Engine {
 public:
  /**
   * C++内核构造时先建立公共状态；交付默认配置随后会在首个输入前启用惯性路径。
   * 直接嵌入本类且不调用configure_inertial时，才进入仅测距兼容模式。
   */
  explicit Engine(EngineConfig config);

  /**
   * 在处理任何输入前启用15维IMU+UWB路径；不调用时完整保留UWB-only行为。
   * 节点集合必须与Engine构造配置一致，节点顺序可以不同。
   */
  void configure_inertial(
      InertialConfig inertial_config,
      std::vector<InertialNodeInitialization> initializations,
      std::size_t max_inertial_state_dimension);
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);
  [[nodiscard]] RangeProcessingResult push_range(const RangePacket& packet);
  [[nodiscard]] EngineSnapshot step(std::uint64_t now_ns);

  [[nodiscard]] const RangeEkf& filter() const noexcept;
  [[nodiscard]] bool inertial_enabled() const noexcept;
  [[nodiscard]] const CooperativeInertialEkf* inertial_filter() const
      noexcept;
  [[nodiscard]] const DegradationMonitor& monitor() const noexcept;

 private:
  struct DirectedLinkKey {
    std::uint32_t from{};
    std::uint32_t to{};

    [[nodiscard]] bool operator==(const DirectedLinkKey& other) const noexcept {
      return from == other.from && to == other.to;
    }
  };

  struct DirectedLinkHash {
    [[nodiscard]] std::size_t operator()(const DirectedLinkKey& link) const
        noexcept;
  };

  struct DuplicateKey {
    std::uint64_t sequence{};
    std::uint64_t timestamp_ns{};
  };

  [[nodiscard]] bool structurally_valid(const RangePacket& packet) const;
  [[nodiscard]] bool packet_is_nlos(const RangePacket& packet) const;
  [[nodiscard]] bool duplicate_and_remember(const RangePacket& packet);
  [[nodiscard]] static EngineConfig validate_config(EngineConfig config);

  EngineConfig config_;
  RangeEkf filter_;
  std::optional<CooperativeInertialEkf> inertial_filter_;
  DegradationMonitor monitor_;
  std::vector<std::uint32_t> node_ids_;
  std::unordered_set<std::uint32_t> known_nodes_;
  std::vector<EdgeKey> all_edges_;
  std::unordered_map<EdgeKey, std::uint64_t, EdgeKeyHash> last_valid_ns_;
  std::unordered_set<EdgeKey, EdgeKeyHash> time_sync_faults_;
  std::unordered_map<DirectedLinkKey, std::deque<DuplicateKey>,
                     DirectedLinkHash>
      duplicate_cache_;
  std::uint64_t latest_timestamp_ns_{};
  bool has_latest_timestamp_{};
  bool processing_started_{};
};

}  // namespace zju::coop

// 模块职责：编排标准化输入、时间/重复检查、质量监测、滤波更新、动态拓扑与输出快照。
// Engine是算法核心的系统边界，但不解析ROS 2消息、不收发无线链路、不输出控制指令；
// 上交适配层通过C ABI串行调用本引擎，GCS只消费它生成的定位/网络/观测/告警结果。
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

/**
 * 组合滤波、退化监测、时间检查和资源上限的运行配置。
 * future_skew比较测量时刻晚于接收时刻的异常，receive_delay比较接收晚于测量
 * 的链路延迟；两者都要求测量与接收时间来自同一统一时间基准。
 */
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

/**
 * 面向GCS/ROS 2输出的二维主参考相对定位快照。
 * x/y/vx/vy均为node-reference，位置协方差是二者差值的2×2协方差；
 * 当前yaw_valid和z_valid固定为false，消费者不得补零后标成有效。
 */
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

/** 当前主参考下的协同网络可达性、可观性和综合原因位图。 */
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

/** 单一时刻的原子输出快照，保证定位、网络和观测状态彼此一致。 */
struct EngineSnapshot {
  std::uint64_t timestamp_ns{};
  std::vector<LocalizationSnapshot> localizations;
  NetworkSnapshot network{};
  std::vector<ObservationQuality> observations;
};

/**
 * 一包测距在引擎级的终止位置。Processed不保证滤波接受量测，还要查看
 * RangeProcessingResult::update；Held/Rejected表示质量状态机主动阻断。
 */
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
  // 指示滤波路径是否消费该量测/时间诊断；详细数值结论以update.disposition为准。
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
  /** 校验统一时间语义后，把瞬时IMU交给所属节点15维传播器。 */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);
  /** 完成时间/重复/质量检查，再选择默认惯性或显式回退测距更新路径。 */
  [[nodiscard]] RangeProcessingResult push_range(const RangePacket& packet);
  /** 推进质量/超时逻辑并生成同一effective_now下的原子输出，不读取新传感器。 */
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
    // sequence与timestamp成对比较，兼容设备重启后序号复位但时间仍前进的情况。
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

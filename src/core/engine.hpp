// 模块职责：编排标准化输入、时间/重复检查、质量监测、滤波更新、动态拓扑与输出快照。
// Engine是算法核心的系统边界，但不解析ROS 2消息、不收发无线链路、不输出控制指令；
// 上海交大适配层通过C ABI串行调用本引擎，GCS只消费它生成的定位/网络/观测/告警结果。
//
// 初学者可把Engine理解为“算法总调度员”，它自己不推导全部滤波公式，而是决定调用顺序：
// push_imu()把IMU交给15维惯导；push_range()先做时间/重复/质量检查，再决定是否融合；
// step()按当前时间淘汰过期边、分析拓扑并组装一次一致的输出快照。
// `std::optional`表示惯性滤波器可以不存在：存在时走IMU+测距，缺省时走仅测距回退路径。
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
  FilterConfig filter{};  ///< 兼容UWB-only滤波参数及主参考编号。
  std::vector<NodeInitialization> nodes;  ///< 固定节点集合及其二维初始状态。
  DegradationConfig degradation{};       ///< 单边质量滑窗和状态机配置。
  std::uint64_t edge_timeout_ns{500'000'000ULL};      ///< 有效边无新量测后的失效时长。
  std::uint64_t max_future_skew_ns{100'000'000ULL};   ///< 测量时间领先接收时间的容许上限。
  std::uint64_t max_receive_delay_ns{500'000'000ULL}; ///< 接收时间落后测量时间的容许上限。
  std::size_t duplicate_cache_per_link{128U};  ///< 每条有向链路保留的近期包键数量。
  std::size_t max_nodes{64U};                  ///< 配置节点数量的资源上限。
  std::size_t max_edges{2016U};                ///< 完全图无向边数量的资源上限。
  std::size_t max_state_dimension{252U};       ///< UWB-only联合状态维数上限。
  double rigidity_tolerance{1.0e-9};           ///< 刚度矩阵数值秩判定容差。
};

/**
 * 面向GCS/ROS 2输出的二维主参考相对定位快照。
 * x/y/vx/vy均为node-reference，位置协方差是二者差值的2×2协方差；
 * 内部惯性模式可携带yaw；兼容C ABI v1仍强制yaw_valid=false，真实航向由Pose2D v2查询。
 */
struct LocalizationSnapshot {
  std::uint32_t node_id{};           ///< 本条估计所属节点编号。
  std::uint32_t reference_node_id{}; ///< 相对坐标所采用的主参考节点编号。
  std::uint64_t timestamp_ns{};      ///< 该节点估计实际推进到的统一时间。
  double x{};                        ///< 节点相对主参考的x位置，单位m。
  double y{};                        ///< 节点相对主参考的y位置，单位m。
  double yaw_rad{};                  ///< 本车前向轴在公共ENU中的航向，单位rad。
  double vx{};                       ///< 节点相对主参考的x速度，单位m/s。
  double vy{};                       ///< 节点相对主参考的y速度，单位m/s。
  double cov_xx{};                   ///< 相对位置协方差的xx分量。
  double cov_xy{};                   ///< 相对位置协方差的xy分量。
  double cov_yy{};                   ///< 相对位置协方差的yy分量。
  bool valid{};                      ///< 滤波器是否给出有效的相对估计。
  bool yaw_valid{};                  ///< yaw字段是否来自同历元的惯性姿态；仅测距模式固定为false。
  bool z_valid{};                    ///< 高度字段是否可用；当前输出固定为false。
  LocalizationState state{LocalizationState::kUninitialized};  ///< 结合节点有效性与网络状态的定位状态。
};

/** Pose2D v2只读查询使用的单车内部元素；位置轴与公共ENU平行。 */
struct VehiclePose2dSnapshot {
  std::uint32_t node_id{};
  double x_m{};
  double y_m{};
  double yaw_rad{};
  bool position_valid{};
  bool yaw_valid{};
};

/** 不推进滤波状态的当前多车二维位姿快照。 */
struct Pose2dSnapshot {
  std::uint64_t timestamp_ns{};  ///< 惯性模式为全部节点共同IMU历元；仅测距模式沿用当前滤波时间。
  std::uint32_t reference_node_id{};
  std::vector<VehiclePose2dSnapshot> vehicles;
};

/** 当前主参考下的协同网络可达性、可观性和综合原因位图。 */
struct NetworkSnapshot {
  std::uint64_t timestamp_ns{};       ///< 本网络快照的统一评估时刻。
  std::size_t node_count{};           ///< 固定配置节点总数。
  std::size_t reachable_node_count{}; ///< 从主参考沿活动边可达的节点数。
  std::size_t active_edge_count{};    ///< 去重后参与本轮拓扑分析的活动边数。
  bool connected{};                   ///< 主参考是否能经活动边到达全部节点。
  bool observable{};                  ///< 当前二维距离约束是否达到目标刚度秩。
  ReasonMask reason_mask{ReasonMask::NONE};  ///< 质量、同步、超时和几何原因的综合位图。
  LocalizationState state{LocalizationState::kUninitialized};  ///< 按可观性优先级归纳的网络定位状态。
};

/** 单一时刻的原子输出快照，保证定位、网络和观测状态彼此一致。 */
struct EngineSnapshot {
  std::uint64_t timestamp_ns{};  ///< 本次原子step采用的统一有效时刻。
  std::vector<LocalizationSnapshot> localizations;  ///< 各配置节点的相对定位输出。
  NetworkSnapshot network{};  ///< 与定位和观测同批生成的网络状态。
  std::vector<ObservationQuality> observations;  ///< 全部预注册无向边的质量快照。
};

/**
 * 一包测距在引擎级的终止位置。Processed不保证滤波接受量测，还要查看
 * RangeProcessingResult::update；Held/Rejected表示质量状态机主动阻断。
 */
enum class ProcessingDisposition {
  Processed,     ///< 已到达底层滤波入口；是否被NIS接纳还需看update.disposition。
  InvalidPacket, ///< 端点、距离、标准差、状态码或有效位不合法。
  OutOfOrder,    ///< 同一引擎时间轴上出现过早的测量。
  TimeRejected, ///< 测量时间与接收时间的偏差超过配置门限。
  Duplicate,    ///< 同一有向链路近期已见相同sequence+timestamp组合。
  Held,         ///< 质量状态为Suspended，本包暂缓而未进入滤波器。
  Rejected,     ///< 质量状态为Rejected，本包被剔除。
};

struct RangeProcessingResult {
  EdgeKey edge{};  ///< 输入端点规范化得到的无向边。
  ProcessingDisposition disposition{ProcessingDisposition::InvalidPacket};  ///< 引擎入口的终止分类。
  FusionAction action{FusionAction::kUseNormal};  ///< 质量状态机为该包选择的融合动作。
  // 指示滤波路径是否消费该量测/时间诊断；详细数值结论以update.disposition为准。
  bool filter_updated{};  ///< 是否调用滤波更新并得到接受或NIS拒绝结果。
  UpdateResult update{};  ///< 底层滤波更新的数值结果与细分处置。
};

class Engine {
 public:
  /**
   * C++内核构造时先建立公共状态；交付默认配置随后会在首个输入前启用惯性路径。
   * 直接嵌入本类且不调用configure_inertial时，才进入仅测距兼容模式。
   * @param config 固定节点、时间校验、质量监视及资源限制配置。
   */
  explicit Engine(EngineConfig config);

  /**
   * 在处理任何输入前启用15维IMU+UWB路径；不调用时完整保留UWB-only行为。
   * 节点集合必须与Engine构造配置一致，节点顺序可以不同。
   * @param inertial_config 15维单节点传播与噪声配置。
   * @param initializations 全部配置节点的惯性初始状态。
   * @param max_inertial_state_dimension 惯性联合状态允许的维数上限。
   */
  void configure_inertial(
      InertialConfig inertial_config,
      std::vector<InertialNodeInitialization> initializations,
      std::size_t max_inertial_state_dimension);
  /** 校验统一时间语义后，把瞬时IMU交给所属节点15维传播器。
   * @param packet 带统一测量/接收时间的瞬时IMU输入。 */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);
  /** 完成时间/重复/质量检查，再选择默认惯性或显式回退测距更新路径。
   * @param packet 带端点、序号、统一时间和质量字段的测距输入。 */
  [[nodiscard]] RangeProcessingResult push_range(const RangePacket& packet);
  /** 推进质量/超时逻辑并生成同一effective_now下的原子输出，不读取新传感器。
   * @param now_ns 调用方提供的统一输出时刻。 */
  [[nodiscard]] EngineSnapshot step(std::uint64_t now_ns);
  /** 读取当前二维位姿，不推进预测、质量窗口或全局时间。 */
  [[nodiscard]] Pose2dSnapshot pose2d_snapshot() const;

  /** 返回仅测距兼容滤波器只读引用；惯性已启用时它不会继续推进。 */
  [[nodiscard]] const RangeEkf& filter() const noexcept;
  /** optional中存在CooperativeInertialEkf时返回true。 */
  [[nodiscard]] bool inertial_enabled() const noexcept;
  /** 返回惯性滤波器地址；未启用时返回nullptr，调用者必须先判空。 */
  [[nodiscard]] const CooperativeInertialEkf* inertial_filter() const
      noexcept;
  /** 返回质量监视器只读引用，供诊断查询。 */
  [[nodiscard]] const DegradationMonitor& monitor() const noexcept;

 private:
  struct DirectedLinkKey {
    std::uint32_t from{};  ///< 有向链路的发送节点编号。
    std::uint32_t to{};    ///< 有向链路的接收节点编号。

    /** @param other 待比较的有向链路键。 */
    [[nodiscard]] bool operator==(const DirectedLinkKey& other) const noexcept {
      // 有向链路必须起点和终点同时相等；1->2与2->1不是同一个重复缓存。
      return from == other.from && to == other.to;
    }
  };

  struct DirectedLinkHash {
    /** @param link 待散列的有向链路键。 */
    [[nodiscard]] std::size_t operator()(const DirectedLinkKey& link) const
        noexcept;
  };

  struct DuplicateKey {
    // sequence与timestamp成对比较，兼容设备重启后序号复位但时间仍前进的情况。
    std::uint64_t sequence{};     ///< 发送方为该包赋予的序号。
    std::uint64_t timestamp_ns{}; ///< 该包携带的统一测量时间。
  };

  /** @param packet 待检查节点、距离及标准差字段的测距包。 */
  [[nodiscard]] bool structurally_valid(const RangePacket& packet) const;
  /** @param packet 待按标志和概率阈值判定NLOS的测距包。 */
  [[nodiscard]] bool packet_is_nlos(const RangePacket& packet) const;
  /** @param packet 待在所属有向链路近期缓存中查询并登记的测距包。 */
  [[nodiscard]] bool duplicate_and_remember(const RangePacket& packet);
  /** @param config 待验证并按值返回的引擎配置。 */
  [[nodiscard]] static EngineConfig validate_config(EngineConfig config);

  EngineConfig config_;  ///< 经资源与一致性校验后固定使用的运行配置。
  RangeEkf filter_;      ///< 未启用惯性模式时唯一更新的UWB-only滤波器。
  std::optional<CooperativeInertialEkf> inertial_filter_;  ///< 首个输入前可一次启用的15维联合滤波器。
  DegradationMonitor monitor_;  ///< 全部可能无向边共享的质量滑窗监视器。
  std::vector<std::uint32_t> node_ids_;  ///< 保留配置顺序的节点编号，用于拓扑矩阵对齐。
  std::unordered_set<std::uint32_t> known_nodes_;  ///< 输入节点合法性检查用的固定集合。
  std::vector<EdgeKey> all_edges_;  ///< 配置节点完全图中的全部无向边。
  std::unordered_map<EdgeKey, std::uint64_t, EdgeKeyHash> last_valid_ns_;  ///< 各边最近结构有效量测时间。
  std::unordered_set<EdgeKey, EdgeKeyHash> time_sync_faults_;  ///< 最近一次时间检查仍未由合法包清除的边。
  std::unordered_map<DirectedLinkKey, std::deque<DuplicateKey>,
                     DirectedLinkHash>
      duplicate_cache_;  ///< 每条有向链路独立维护的有界近期包键FIFO。
  std::uint64_t latest_timestamp_ns_{};  ///< 通过端点/时间/重复/乱序检查并进入质量窗的测距时间（含结构无效包）或step推进时间的最大值，不含IMU。
  bool has_latest_timestamp_{};          ///< 最大统一时间字段是否已初始化。
  bool processing_started_{};            ///< 是否已接收输入或执行step，用于锁定惯性配置阶段。
};

}  // namespace zju::coop

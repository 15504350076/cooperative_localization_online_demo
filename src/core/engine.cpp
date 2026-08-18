// 模块实现：把IMU/测距输入校验、质量监测、15维联合滤波、动态拓扑和输出状态串成闭环。
// 关键原则：惯性模式只由真实IMU推进，测距仅做观测更新；所有时间、重复包和质量结论
// 在进入滤波器前确定，step只生成一致快照，不承担无线协议或车辆控制。
//
// 初学者阅读顺序：
// 1. validate_config()：启动前拒绝不合理参数；
// 2. configure_inertial()：在首个输入前选择默认15维模式；
// 3. push_imu()/push_range()：处理两类异步输入；
// 4. step()：定时生成定位、观测质量和网络状态。
// 看到candidate、旧状态副本或最后swap时，表示代码先在临时对象上计算，全部成功后才提交。
#include "core/engine.hpp"

#include "core/rigidity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

constexpr std::size_t kMaximumDuplicateCachePerLink = 4096U;  // 单有向链路近期包键的实现硬上限。
constexpr std::size_t kMaximumNodes = 64U;  // 引擎节点配置的实现硬上限。
constexpr std::size_t kMaximumEdges = 2016U;  // 64节点完全图的无向边硬上限。
constexpr std::size_t kMaximumStateDimension = 252U;  // 63个非参考节点四维状态的硬上限。
constexpr std::size_t kMaximumTrackedEdges = 1'000'000U;  // 传递给退化监视器的边资源绝对上限。

}  // namespace

EngineConfig Engine::validate_config(EngineConfig config) {
  // 阶段1：在任何矩阵和边表分配前验证时间参数及节点、边、状态资源上限。
  if (config.edge_timeout_ns == 0U || config.max_future_skew_ns == 0U ||
      config.max_receive_delay_ns == 0U ||
      config.duplicate_cache_per_link == 0U ||
      config.duplicate_cache_per_link > kMaximumDuplicateCachePerLink ||
      !(config.rigidity_tolerance > 0.0) ||
      !std::isfinite(config.rigidity_tolerance) || config.max_nodes == 0U ||
      config.max_nodes > kMaximumNodes || config.max_edges == 0U ||
      config.max_edges > kMaximumEdges || config.max_state_dimension == 0U ||
      config.max_state_dimension > kMaximumStateDimension ||
      config.degradation.max_tracked_edges == 0U ||
      config.degradation.max_tracked_edges > kMaximumTrackedEdges ||
      config.filter.reference_node_id >
          std::numeric_limits<std::uint16_t>::max()) {
    throw std::invalid_argument("invalid engine configuration");
  }

  const std::size_t node_count = config.nodes.size();  // 本配置实际声明的固定节点数。
  if (node_count == 0U || node_count > config.max_nodes) {
    throw std::invalid_argument("engine node count exceeds resource limit");
  }
  const std::size_t nonreference_count = node_count - 1U;  // 进入相对状态向量的非参考节点数。
  if (nonreference_count >
      std::numeric_limits<std::size_t>::max() / 4U) {
    throw std::invalid_argument("engine state dimension overflows");
  }
  const std::size_t state_dimension = nonreference_count * 4U;  // UWB-only每节点x/y/vx/vy联合维数。
  if (state_dimension > config.max_state_dimension) {
    throw std::invalid_argument("engine state dimension exceeds limit");
  }
  if (node_count > 1U &&
      node_count >
          std::numeric_limits<std::size_t>::max() / (node_count - 1U)) {
    throw std::invalid_argument("engine edge count overflows");
  }
  const std::size_t edge_count =  // 固定节点完全图中需要预注册的无向边数。
      node_count * (node_count - 1U) / 2U;
  if (edge_count > config.max_edges ||
      edge_count > config.degradation.max_tracked_edges) {
    throw std::invalid_argument("engine edge count exceeds resource limit");
  }

  std::unordered_set<std::uint32_t> identifiers;  // 已见节点编号集合，用于唯一性校验。
  identifiers.reserve(node_count);
  bool has_reference = false;  // 配置节点中是否包含滤波主参考。
  for (const auto& node : config.nodes) {  // node为待校验编号并查找主参考的初始化项。
    if (node.node_id > std::numeric_limits<std::uint16_t>::max() ||
        !identifiers.insert(node.node_id).second) {
      throw std::invalid_argument("invalid engine node identifier");
    }
    has_reference = has_reference ||
                    node.node_id == config.filter.reference_node_id;
  }
  if (!has_reference) {
    throw std::invalid_argument("engine reference node is not initialized");
  }
  return config;
}

Engine::Engine(EngineConfig config)
    // 构造顺序按成员声明而非书写顺序：先校验并移动config，再用已保存配置构造两个子模块。
    : config_(validate_config(std::move(config))),
      filter_(config_.filter, config_.nodes),
      monitor_(config_.degradation) {
  // 阶段2：固定节点集合并预注册全部可能无向边，运行中只改变边的有效状态。
  node_ids_.reserve(config_.nodes.size());
  known_nodes_.reserve(config_.nodes.size());
  for (const auto& node : config_.nodes) {  // node按配置顺序登记到向量和合法集合。
    node_ids_.push_back(node.node_id);
    known_nodes_.insert(node.node_id);
  }
  for (std::size_t first = 0U; first < node_ids_.size(); ++first) {  // first为完全图首端点索引。
    for (std::size_t second = first + 1U; second < node_ids_.size(); ++second) {  // second为不重复的次端点索引。
      const EdgeKey edge(node_ids_[first], node_ids_[second]);  // 当前端点对的规范化无向边。
      all_edges_.push_back(edge);
      monitor_.track(edge);
    }
  }
}

void Engine::configure_inertial(
    InertialConfig inertial_config,
    std::vector<InertialNodeInitialization> initializations,
    std::size_t max_inertial_state_dimension) {
  // 惯性状态只能在首个输入前配置，防止运行中改变状态维度破坏历史协方差。
  if (processing_started_ || inertial_filter_.has_value()) {
    throw std::logic_error(
        "inertial mode must be configured once before processing starts");
  }
  if (initializations.size() != known_nodes_.size()) {
    throw std::invalid_argument("inertial node set does not match engine");
  }
  std::unordered_set<std::uint32_t> inertial_nodes;  // 惯性初始化中实际出现的节点编号集合。
  for (const auto& node : initializations) {  // node为待登记集合的惯性初始化项。
    inertial_nodes.insert(node.node_id);
  }
  if (inertial_nodes != known_nodes_) {
    throw std::invalid_argument("inertial node set does not match engine");
  }
  CooperativeInertialConfig cooperative{};  // 从基础滤波配置派生的联合惯性层参数。
  cooperative.reference_node_id = config_.filter.reference_node_id;
  cooperative.nis_gate = config_.filter.nis_gate;
  cooperative.min_covariance_diagonal =
      config_.filter.min_covariance_diagonal;
  cooperative.max_inertial_state_dimension = max_inertial_state_dimension;
  // optional::emplace在其内部直接构造联合滤波器，使optional从“空”变为“有值”。
  inertial_filter_.emplace(cooperative, std::move(inertial_config),
                           std::move(initializations));
}

ImuProcessingResult Engine::push_imu(const ImuPacket& packet) {
  processing_started_ = true;
  // IMU入口只做模式和统一时间检查，15维传播及细分错误由惯性滤波器分类。
  if (!inertial_filter_) {
    ImuProcessingResult result{};  // 未启用惯性路径时返回的确定性拒绝结果。
    result.disposition = ImuDisposition::kUnknownNode;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }
  const bool too_far_future =  // 测量时间相对同基准接收时间是否超前过多。
      packet.timestamp_ns > packet.receive_timestamp_ns &&
      packet.timestamp_ns - packet.receive_timestamp_ns >
          config_.max_future_skew_ns;
  const bool too_delayed =  // 本机接收时间相对测量时间是否落后过多。
      packet.receive_timestamp_ns > packet.timestamp_ns &&
      packet.receive_timestamp_ns - packet.timestamp_ns >
          config_.max_receive_delay_ns;
  // 测量与接收时间必须来自同一时基：前者不能明显“来自未来”，后者也不能
  // 晚到超过允许链路延迟。这里不尝试自行校时，校时职责属于上海交大平台侧。
  if (packet.receive_timestamp_ns == 0U || too_far_future || too_delayed) {
    ImuProcessingResult result{};  // 时间语义不合法时返回的确定性拒绝结果。
    result.disposition = ImuDisposition::kInvalidPacket;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }
  // optional重载了operator->；前面已判定有值，因此可像指针一样调用内部对象。
  return inertial_filter_->push_imu(packet);
}

bool Engine::structurally_valid(const RangePacket& packet) const {
  return packet.valid && packet.from_node != packet.to_node &&
         known_nodes_.count(packet.from_node) != 0U &&
         known_nodes_.count(packet.to_node) != 0U &&
         std::isfinite(packet.range_m) && packet.range_m > 0.0 &&
         std::isfinite(packet.range_std_m) && packet.range_std_m > 0.0;
}

bool Engine::packet_is_nlos(const RangePacket& packet) const {
  return packet.nlos_flag ||
         (packet.has_nlos_probability &&
          std::isfinite(static_cast<double>(packet.nlos_probability)) &&
          static_cast<double>(packet.nlos_probability) >=
              config_.degradation.nlos_probability_threshold);
}

std::size_t Engine::DirectedLinkHash::operator()(
    const DirectedLinkKey& link) const noexcept {
  const std::size_t from_hash =  // 有向链路发送端编号的基础散列。
      std::hash<std::uint32_t>{}(link.from);
  const std::size_t to_hash =  // 有向链路接收端编号的基础散列。
      std::hash<std::uint32_t>{}(link.to);
  return from_hash ^ (to_hash + static_cast<std::size_t>(0x9e3779b9U) +
                      (from_hash << 6U) + (from_hash >> 2U));
}

bool Engine::duplicate_and_remember(const RangePacket& packet) {
  // 重复判定按有向链路保存(sequence,timestamp)，反向合法测距不会互相覆盖。
  auto& cache =  // 当前发送端到接收端独占的近期包键FIFO。
      // unordered_map的operator[]：键不存在时自动值初始化一个空deque并返回其引用。
      duplicate_cache_[{packet.from_node, packet.to_node}];
  const auto duplicate = std::find_if(  // 与输入序号和测量时间同时相等的缓存位置。
      cache.begin(), cache.end(), [&packet](const DuplicateKey& existing) {
        // `[&packet]`按引用捕获当前输入，Lambda调用期间packet仍有效且不会复制整包。
        // 捕获packet以读取当前输入键；existing为逐项比较的历史包键。
        return existing.sequence == packet.sequence &&
               existing.timestamp_ns == packet.timestamp_ns;
      });
  if (duplicate != cache.end()) {
    return true;
  }
  cache.push_back({packet.sequence, packet.timestamp_ns});
  // 有界FIFO只用于近期重传诊断，不承担无限历史去重或无线链路可靠传输。
  if (cache.size() > config_.duplicate_cache_per_link) {
    cache.pop_front();
  }
  return false;
}

RangeProcessingResult Engine::push_range(const RangePacket& packet) {
  processing_started_ = true;
  // 阶段1：结构、节点、时间和重复包检查必须先于质量统计与滤波更新。
  RangeProcessingResult result{};  // 逐阶段填充并返回调用方的引擎处置结果。
  result.edge = EdgeKey(packet.from_node, packet.to_node);
  if (packet.from_node == packet.to_node ||
      known_nodes_.count(packet.from_node) == 0U ||
      known_nodes_.count(packet.to_node) == 0U) {
    result.update.disposition = UpdateDisposition::InvalidPacket;
    return result;
  }
  if (packet.receive_timestamp_ns == 0U) {
    result.disposition = ProcessingDisposition::TimeRejected;
    time_sync_faults_.insert(result.edge);
    return result;
  }
  const bool too_far_future =  // 测量时间相对同基准接收时间是否超前过多。
      packet.timestamp_ns > packet.receive_timestamp_ns &&
      packet.timestamp_ns - packet.receive_timestamp_ns >
          config_.max_future_skew_ns;
  const bool too_delayed =  // 本机接收时间相对测量时间是否落后过多。
      packet.receive_timestamp_ns > packet.timestamp_ns &&
      packet.receive_timestamp_ns - packet.timestamp_ns >
          config_.max_receive_delay_ns;
  if (too_far_future || too_delayed) {
    result.disposition = ProcessingDisposition::TimeRejected;
    time_sync_faults_.insert(result.edge);
    return result;
  }
  if (duplicate_and_remember(packet)) {
    result.disposition = ProcessingDisposition::Duplicate;
    return result;
  }
  if (has_latest_timestamp_ && packet.timestamp_ns < latest_timestamp_ns_) {
    result.disposition = ProcessingDisposition::OutOfOrder;
    result.update.disposition = UpdateDisposition::OutOfOrder;
    return result;
  }
  time_sync_faults_.erase(result.edge);

  // 阶段2：所有到达量测都进入质量窗口，包括无效/NLOS数据，避免只统计好数据。
  const bool valid_for_filter = structurally_valid(packet);  // 是否允许进入数值滤波更新。
  monitor_.record(result.edge, packet.timestamp_ns, valid_for_filter,
                  packet.nlos_flag, packet.has_nlos_probability,
                  static_cast<double>(packet.nlos_probability));
  monitor_.advance(packet.timestamp_ns);
  latest_timestamp_ns_ = packet.timestamp_ns;
  has_latest_timestamp_ = true;
  const ObservationQuality quality =  // 记录当前包并推进状态机后的单边质量快照。
      monitor_.quality(result.edge);
  result.action = quality.action;
  if (!valid_for_filter) {
    result.disposition = ProcessingDisposition::InvalidPacket;
    result.update.disposition = UpdateDisposition::InvalidPacket;
    return result;
  }
  const auto last_valid =  // 当前边最近结构有效测量时间的索引位置。
      last_valid_ns_.find(result.edge);
  if (last_valid == last_valid_ns_.end()) {
    last_valid_ns_.emplace(result.edge, packet.timestamp_ns);
  } else {
    last_valid->second = std::max(last_valid->second, packet.timestamp_ns);
  }

  // 阶段3：质量状态决定正常、降权、暂缓、剔除或试探恢复的融合动作。
  if (result.action == FusionAction::kHold ||
      result.action == FusionAction::kReject) {
    // 多行三目运算把两种质量阻断动作分别映射到Held或Rejected。
    result.disposition = result.action == FusionAction::kHold
                             ? ProcessingDisposition::Held
                             : ProcessingDisposition::Rejected;
    return result;
  }
  const bool downweighted =  // 原始NLOS或状态机要求是否触发协方差放大。
                            packet_is_nlos(packet) ||
                             result.action == FusionAction::kUseDownweighted ||
                             result.action == FusionAction::kTrialRecovery;
  const double covariance_scale =  // 本次数值更新实际采用的测距协方差倍率。
      downweighted ? config_.degradation.nlos_covariance_scale : 1.0;
  // 阶段4：默认惯性路径与兼容仅测距路径二选一，绝不同时更新两套在线状态。
  result.update = inertial_filter_
                      // optional可在布尔上下文判断是否有值；有值走15N路径，否则走4M回退路径。
                      ? inertial_filter_->update_range(packet,
                                                       covariance_scale)
                      : filter_.update(packet, covariance_scale);
  result.filter_updated =
      result.update.disposition == UpdateDisposition::Accepted ||
      result.update.disposition == UpdateDisposition::NisRejected;
  if (result.update.disposition == UpdateDisposition::NisRejected) {
    monitor_.record_residual_rejection(result.edge, packet.timestamp_ns);
    monitor_.advance(packet.timestamp_ns);
  }
  result.disposition = ProcessingDisposition::Processed;
  return result;
}

EngineSnapshot Engine::step(std::uint64_t now_ns) {
  processing_started_ = true;
  const std::uint64_t effective_now =  // 不早于既有测距或step进度的统一step时刻。
      // 三目运算：已有内部时间时取两者最大值，否则直接采用调用方时刻。
      has_latest_timestamp_ ? std::max(now_ns, latest_timestamp_ns_) : now_ns;
  // 惯性模式的位置只能由真实IMU推进，step不得叠加旧的恒速预测模型。
  if (!inertial_filter_) {
    filter_.predict_to(effective_now);
  }
  // 阶段1：把质量窗口推进到统一输出时刻，再选择仍处于有效期的协同边。
  monitor_.advance(effective_now);
  latest_timestamp_ns_ = effective_now;
  has_latest_timestamp_ = true;

  EngineSnapshot snapshot{};  // 本轮统一时刻逐项组装的原子输出。
  snapshot.timestamp_ns = effective_now;
  snapshot.observations.reserve(all_edges_.size());
  std::vector<Edge> active_edges;  // 同时满足新鲜度和质量状态的拓扑约束。
  active_edges.reserve(all_edges_.size());
  bool degraded_quality = false;  // 任一边质量异常或同步故障的汇总标志。
  bool timed_out = false;         // 任一曾有效边是否超过新鲜度门限。
  for (const EdgeKey edge : all_edges_) {  // edge为完全图中待生成观测并筛选活动性的无向边。
    const ObservationQuality quality = monitor_.quality(edge);  // 该边在effective_now的质量快照。
    snapshot.observations.push_back(quality);
    snapshot.network.reason_mask |= quality.reason_mask;
    degraded_quality = degraded_quality ||
                       (quality.state != ObservationState::kUnknown &&
                        quality.state != ObservationState::kNormal) ||
                       quality.action != FusionAction::kUseNormal;
    const auto last_valid = last_valid_ns_.find(edge);  // 该边最近结构有效测量的索引位置。
    if (last_valid == last_valid_ns_.end()) {
      continue;
    }
    // 活动边必须同时满足“近期有结构有效量测”和“质量状态允许使用”；
    // 仅在质量窗口中出现过并不代表此刻仍能承担拓扑约束。
    const bool expired =  // 最近有效测量距统一输出时刻是否超过边超时门限。
        effective_now - last_valid->second > config_.edge_timeout_ns;
    if (expired) {
      timed_out = true;
      continue;
    }
    if (quality.state == ObservationState::kSuspended ||
        quality.state == ObservationState::kRejected) {
      continue;
    }
    active_edges.push_back({edge.first, edge.second});
  }

  const std::vector<NodeEstimate> estimates =  // 当前唯一在线滤波路径导出的全部节点估计。
      // 两个分支返回同一类型vector，因此可用三目运算一次初始化const结果。
      inertial_filter_ ? inertial_filter_->estimates() : filter_.estimates();
  std::vector<Point2> positions;  // 与node_ids_同序、供刚度分析使用的二维位置。
  positions.reserve(estimates.size());
  for (const auto& estimate : estimates) {  // estimate为待投影到二维刚度输入的节点估计。
    positions.push_back({estimate.x, estimate.y});
  }
  // 阶段2：基于当前活动边实时分析主参考可达性和几何可观性。
  const RigidityResult rigidity = analyze_rigidity(  // 当前活动拓扑的可达性和刚度秩结论。
      node_ids_, positions, active_edges, config_.filter.reference_node_id,
      config_.rigidity_tolerance);

  snapshot.network.timestamp_ns = effective_now;
  snapshot.network.node_count = node_ids_.size();
  snapshot.network.reachable_node_count = rigidity.reachable_count;
  snapshot.network.active_edge_count = rigidity.effective_edge_count;
  snapshot.network.connected = rigidity.connected;
  snapshot.network.observable = rigidity.observable;
  if (timed_out) {
    snapshot.network.reason_mask |= ReasonMask::LINK_TIMEOUT;
  }
  if (!time_sync_faults_.empty()) {
    snapshot.network.reason_mask |= ReasonMask::TIME_SYNC_TIMEOUT;
    degraded_quality = true;
  }
  if (!rigidity.connected) {
    snapshot.network.reason_mask |= ReasonMask::NODE_UNREACHABLE;
  }
  // 状态优先级：几何不可观最高，其次是质量/同步退化；只有两者都正常才Normal。
  if (!rigidity.observable) {
    snapshot.network.reason_mask |= ReasonMask::GRAPH_GEOMETRY_DEGENERATE;
  }
  if (!rigidity.observable) {
    snapshot.network.state = LocalizationState::kUnobservable;
  } else if (degraded_quality) {
    snapshot.network.state = LocalizationState::kDegraded;
  } else {
    snapshot.network.state = LocalizationState::kNormal;
  }

  // 阶段3：定位、网络和观测状态来自同一时刻，整体作为原子快照返回。
  snapshot.localizations.reserve(estimates.size());
  for (const auto& estimate : estimates) {  // estimate为待转换成公开相对定位快照的节点估计。
    LocalizationSnapshot localization{};  // 当前节点的输出结构。
    localization.node_id = estimate.node_id;
    localization.reference_node_id = config_.filter.reference_node_id;
    localization.timestamp_ns = estimate.timestamp_ns;
    localization.x = estimate.x;
    localization.y = estimate.y;
    localization.yaw_rad = estimate.yaw_rad;
    localization.vx = estimate.vx;
    localization.vy = estimate.vy;
    localization.cov_xx = estimate.cov_xx;
    localization.cov_xy = estimate.cov_xy;
    localization.cov_yy = estimate.cov_yy;
    localization.valid = estimate.valid;
    localization.yaw_valid = inertial_filter_ && estimate.yaw_valid;
    localization.z_valid = false;
    // 节点估计有效时继承同批网络状态，否则明确标记未初始化。
    localization.state = estimate.valid ? snapshot.network.state
                                        : LocalizationState::kUninitialized;
    snapshot.localizations.push_back(localization);
  }
  return snapshot;
}

Pose2dSnapshot Engine::pose2d_snapshot() const {
  Pose2dSnapshot snapshot{};
  snapshot.reference_node_id = config_.filter.reference_node_id;
  const std::vector<NodeEstimate> estimates =
      inertial_filter_ ? inertial_filter_->estimates() : filter_.estimates();
  snapshot.vehicles.reserve(estimates.size());

  bool common_inertial_epoch = inertial_filter_.has_value() &&
                               !estimates.empty();
  std::uint64_t common_timestamp = 0U;
  if (common_inertial_epoch) {
    common_timestamp = estimates.front().pose_timestamp_ns;
    common_inertial_epoch = common_timestamp != 0U;
    for (const auto& estimate : estimates) {
      common_inertial_epoch = common_inertial_epoch &&
                              estimate.pose_timestamp_ns == common_timestamp;
    }
  }
  if (inertial_filter_) {
    snapshot.timestamp_ns = common_inertial_epoch ? common_timestamp : 0U;
  } else if (!estimates.empty()) {
    snapshot.timestamp_ns = estimates.front().timestamp_ns;
  }

  for (const auto& estimate : estimates) {
    VehiclePose2dSnapshot vehicle{};
    vehicle.node_id = estimate.node_id;
    vehicle.x_m = estimate.x;
    vehicle.y_m = estimate.y;
    vehicle.yaw_rad = estimate.yaw_rad;
    vehicle.position_valid = estimate.valid &&
                             (!inertial_filter_ || common_inertial_epoch);
    vehicle.yaw_valid = inertial_filter_ && common_inertial_epoch &&
                        estimate.yaw_valid;
    snapshot.vehicles.push_back(vehicle);
  }
  return snapshot;
}

// 返回只读引用，不复制滤波器内部状态和协方差。
const RangeEkf& Engine::filter() const noexcept { return filter_; }

bool Engine::inertial_enabled() const noexcept {
  return inertial_filter_.has_value();
}

const CooperativeInertialEkf* Engine::inertial_filter() const noexcept {
  // `*optional`取得内部对象，前缀`&`再取得地址；空optional返回nullptr。
  return inertial_filter_ ? &*inertial_filter_ : nullptr;
}

// 返回只读引用供外部查询质量快照，不允许绕过Engine直接修改状态机。
const DegradationMonitor& Engine::monitor() const noexcept { return monitor_; }

}  // namespace zju::coop

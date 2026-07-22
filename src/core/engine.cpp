// 模块实现：把IMU/测距输入校验、质量监测、15维联合滤波、动态拓扑和输出状态串成闭环。
// 关键原则：惯性模式只由真实IMU推进，测距仅做观测更新；所有时间、重复包和质量结论
// 在进入滤波器前确定，step只生成一致快照，不承担无线协议或车辆控制。
#include "core/engine.hpp"

#include "core/rigidity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

constexpr std::size_t kMaximumDuplicateCachePerLink = 4096U;
constexpr std::size_t kMaximumNodes = 64U;
constexpr std::size_t kMaximumEdges = 2016U;
constexpr std::size_t kMaximumStateDimension = 252U;
constexpr std::size_t kMaximumTrackedEdges = 1'000'000U;

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

  const std::size_t node_count = config.nodes.size();
  if (node_count == 0U || node_count > config.max_nodes) {
    throw std::invalid_argument("engine node count exceeds resource limit");
  }
  const std::size_t nonreference_count = node_count - 1U;
  if (nonreference_count >
      std::numeric_limits<std::size_t>::max() / 4U) {
    throw std::invalid_argument("engine state dimension overflows");
  }
  const std::size_t state_dimension = nonreference_count * 4U;
  if (state_dimension > config.max_state_dimension) {
    throw std::invalid_argument("engine state dimension exceeds limit");
  }
  if (node_count > 1U &&
      node_count >
          std::numeric_limits<std::size_t>::max() / (node_count - 1U)) {
    throw std::invalid_argument("engine edge count overflows");
  }
  const std::size_t edge_count = node_count * (node_count - 1U) / 2U;
  if (edge_count > config.max_edges ||
      edge_count > config.degradation.max_tracked_edges) {
    throw std::invalid_argument("engine edge count exceeds resource limit");
  }

  std::unordered_set<std::uint32_t> identifiers;
  identifiers.reserve(node_count);
  bool has_reference = false;
  for (const auto& node : config.nodes) {
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
    : config_(validate_config(std::move(config))),
      filter_(config_.filter, config_.nodes),
      monitor_(config_.degradation) {
  // 阶段2：固定节点集合并预注册全部可能无向边，运行中只改变边的有效状态。
  node_ids_.reserve(config_.nodes.size());
  known_nodes_.reserve(config_.nodes.size());
  for (const auto& node : config_.nodes) {
    node_ids_.push_back(node.node_id);
    known_nodes_.insert(node.node_id);
  }
  for (std::size_t first = 0U; first < node_ids_.size(); ++first) {
    for (std::size_t second = first + 1U; second < node_ids_.size(); ++second) {
      const EdgeKey edge(node_ids_[first], node_ids_[second]);
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
  std::unordered_set<std::uint32_t> inertial_nodes;
  for (const auto& node : initializations) {
    inertial_nodes.insert(node.node_id);
  }
  if (inertial_nodes != known_nodes_) {
    throw std::invalid_argument("inertial node set does not match engine");
  }
  CooperativeInertialConfig cooperative{};
  cooperative.reference_node_id = config_.filter.reference_node_id;
  cooperative.nis_gate = config_.filter.nis_gate;
  cooperative.min_covariance_diagonal =
      config_.filter.min_covariance_diagonal;
  cooperative.max_inertial_state_dimension = max_inertial_state_dimension;
  inertial_filter_.emplace(cooperative, std::move(inertial_config),
                           std::move(initializations));
}

ImuProcessingResult Engine::push_imu(const ImuPacket& packet) {
  processing_started_ = true;
  // IMU入口只做模式和统一时间检查，15维传播及细分错误由惯性滤波器分类。
  if (!inertial_filter_) {
    ImuProcessingResult result{};
    result.disposition = ImuDisposition::kUnknownNode;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }
  const bool too_far_future =
      packet.timestamp_ns > packet.receive_timestamp_ns &&
      packet.timestamp_ns - packet.receive_timestamp_ns >
          config_.max_future_skew_ns;
  const bool too_delayed =
      packet.receive_timestamp_ns > packet.timestamp_ns &&
      packet.receive_timestamp_ns - packet.timestamp_ns >
          config_.max_receive_delay_ns;
  // 测量与接收时间必须来自同一时基：前者不能明显“来自未来”，后者也不能
  // 晚到超过允许链路延迟。这里不尝试自行校时，校时职责属于上交平台。
  if (packet.receive_timestamp_ns == 0U || too_far_future || too_delayed) {
    ImuProcessingResult result{};
    result.disposition = ImuDisposition::kInvalidPacket;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }
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
  const std::size_t from_hash = std::hash<std::uint32_t>{}(link.from);
  const std::size_t to_hash = std::hash<std::uint32_t>{}(link.to);
  return from_hash ^ (to_hash + static_cast<std::size_t>(0x9e3779b9U) +
                      (from_hash << 6U) + (from_hash >> 2U));
}

bool Engine::duplicate_and_remember(const RangePacket& packet) {
  // 重复判定按有向链路保存(sequence,timestamp)，反向合法测距不会互相覆盖。
  auto& cache = duplicate_cache_[{packet.from_node, packet.to_node}];
  const auto duplicate = std::find_if(
      cache.begin(), cache.end(), [&packet](const DuplicateKey& existing) {
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
  RangeProcessingResult result{};
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
  const bool too_far_future =
      packet.timestamp_ns > packet.receive_timestamp_ns &&
      packet.timestamp_ns - packet.receive_timestamp_ns >
          config_.max_future_skew_ns;
  const bool too_delayed =
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
  const bool valid_for_filter = structurally_valid(packet);
  monitor_.record(result.edge, packet.timestamp_ns, valid_for_filter,
                  packet.nlos_flag, packet.has_nlos_probability,
                  static_cast<double>(packet.nlos_probability));
  monitor_.advance(packet.timestamp_ns);
  latest_timestamp_ns_ = packet.timestamp_ns;
  has_latest_timestamp_ = true;
  const ObservationQuality quality = monitor_.quality(result.edge);
  result.action = quality.action;
  if (!valid_for_filter) {
    result.disposition = ProcessingDisposition::InvalidPacket;
    result.update.disposition = UpdateDisposition::InvalidPacket;
    return result;
  }
  const auto last_valid = last_valid_ns_.find(result.edge);
  if (last_valid == last_valid_ns_.end()) {
    last_valid_ns_.emplace(result.edge, packet.timestamp_ns);
  } else {
    last_valid->second = std::max(last_valid->second, packet.timestamp_ns);
  }

  // 阶段3：质量状态决定正常、降权、暂缓、剔除或试探恢复的融合动作。
  if (result.action == FusionAction::kHold ||
      result.action == FusionAction::kReject) {
    result.disposition = result.action == FusionAction::kHold
                             ? ProcessingDisposition::Held
                             : ProcessingDisposition::Rejected;
    return result;
  }
  const bool downweighted = packet_is_nlos(packet) ||
                            result.action == FusionAction::kUseDownweighted ||
                            result.action == FusionAction::kTrialRecovery;
  const double covariance_scale =
      downweighted ? config_.degradation.nlos_covariance_scale : 1.0;
  // 阶段4：默认惯性路径与兼容仅测距路径二选一，绝不同时更新两套在线状态。
  result.update = inertial_filter_
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
  const std::uint64_t effective_now =
      has_latest_timestamp_ ? std::max(now_ns, latest_timestamp_ns_) : now_ns;
  // 惯性模式的位置只能由真实IMU推进，step不得叠加旧的恒速预测模型。
  if (!inertial_filter_) {
    filter_.predict_to(effective_now);
  }
  // 阶段1：把质量窗口推进到统一输出时刻，再选择仍处于有效期的协同边。
  monitor_.advance(effective_now);
  latest_timestamp_ns_ = effective_now;
  has_latest_timestamp_ = true;

  EngineSnapshot snapshot{};
  snapshot.timestamp_ns = effective_now;
  snapshot.observations.reserve(all_edges_.size());
  std::vector<Edge> active_edges;
  active_edges.reserve(all_edges_.size());
  bool degraded_quality = false;
  bool timed_out = false;
  for (const EdgeKey edge : all_edges_) {
    const ObservationQuality quality = monitor_.quality(edge);
    snapshot.observations.push_back(quality);
    snapshot.network.reason_mask |= quality.reason_mask;
    degraded_quality = degraded_quality ||
                       (quality.state != ObservationState::kUnknown &&
                        quality.state != ObservationState::kNormal) ||
                       quality.action != FusionAction::kUseNormal;
    const auto last_valid = last_valid_ns_.find(edge);
    if (last_valid == last_valid_ns_.end()) {
      continue;
    }
    // 活动边必须同时满足“近期有结构有效量测”和“质量状态允许使用”；
    // 仅在质量窗口中出现过并不代表此刻仍能承担拓扑约束。
    const bool expired = effective_now - last_valid->second >
                         config_.edge_timeout_ns;
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

  const std::vector<NodeEstimate> estimates =
      inertial_filter_ ? inertial_filter_->estimates() : filter_.estimates();
  std::vector<Point2> positions;
  positions.reserve(estimates.size());
  for (const auto& estimate : estimates) {
    positions.push_back({estimate.x, estimate.y});
  }
  // 阶段2：基于当前活动边实时分析主参考可达性和几何可观性。
  const RigidityResult rigidity = analyze_rigidity(
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
  for (const auto& estimate : estimates) {
    LocalizationSnapshot localization{};
    localization.node_id = estimate.node_id;
    localization.reference_node_id = config_.filter.reference_node_id;
    localization.timestamp_ns = estimate.timestamp_ns;
    localization.x = estimate.x;
    localization.y = estimate.y;
    localization.vx = estimate.vx;
    localization.vy = estimate.vy;
    localization.cov_xx = estimate.cov_xx;
    localization.cov_xy = estimate.cov_xy;
    localization.cov_yy = estimate.cov_yy;
    localization.valid = estimate.valid;
    localization.yaw_valid = false;
    localization.z_valid = false;
    localization.state = estimate.valid ? snapshot.network.state
                                        : LocalizationState::kUninitialized;
    snapshot.localizations.push_back(localization);
  }
  return snapshot;
}

const RangeEkf& Engine::filter() const noexcept { return filter_; }

bool Engine::inertial_enabled() const noexcept {
  return inertial_filter_.has_value();
}

const CooperativeInertialEkf* Engine::inertial_filter() const noexcept {
  return inertial_filter_ ? &*inertial_filter_ : nullptr;
}

const DegradationMonitor& Engine::monitor() const noexcept { return monitor_; }

}  // namespace zju::coop

// 模块实现：按无向协同边维护定长时间滑窗，并运行带保持时间的观测退化状态机。
// 输入证据包括NLOS、有效率、实际频率、残差拒绝和缓存溢出；输出动作只影响量测
// 协方差或是否融合，不修改底层通信链路状态。
//
// 初学者阅读顺序：先看record()怎样把样本放进deque，再看evaluate()怎样删除窗外样本并
// 计算比例，最后看状态转换和action_for()。deque可理解为“两端都能增删的队列”，
// unordered_map则按无向边编号快速找到各自的样本队列。
#include "core/degradation_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace zju::coop {
namespace {

constexpr std::size_t kMaximumSamplesPerEdge = 1'000'000U;  // 单边缓存及期望计数计算的绝对保护上限。
constexpr std::size_t kCapacityMultiplier = 8U;  // 相对期望样本数预留的到达抖动倍率。
constexpr std::size_t kCapacityOverhead = 16U;   // 低频窗口仍保留的固定样本余量。

/** @param value 待验证为有限闭区间[0,1]数值的配置项。 */
bool unit_interval(double value) {
  // 三个`&&`要求有限、非负且不大于1，正好定义闭区间[0,1]。
  return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

/** @param now 当前统一时间；@param since 持续区间起点。 */
std::uint64_t elapsed(std::uint64_t now, std::uint64_t since) {
  // 无符号整数相减可能下溢，所以用三目运算符在时间倒退时饱和为0。
  return now >= since ? now - since : 0U;
}

}  // namespace

EdgeKey::EdgeKey(std::uint32_t node_a, std::uint32_t node_b) noexcept
    // 成员初始化列表用min/max强制first<=second，使反向输入得到同一个键。
    : first(std::min(node_a, node_b)), second(std::max(node_a, node_b)) {}

bool EdgeKey::operator==(const EdgeKey& other) const noexcept {
  // 即使有人通过默认构造后手工交换成员，比较时也再次规范化两边。
  return std::min(first, second) == std::min(other.first, other.second) &&
         std::max(first, second) == std::max(other.first, other.second);
}

bool EdgeKey::operator!=(const EdgeKey& other) const noexcept {
  // `*this`取得当前EdgeKey对象，再复用已定义的==并用!取反。
  return !(*this == other);
}

std::size_t EdgeKeyHash::operator()(const EdgeKey& edge) const noexcept {
  const std::size_t first_hash =  // 规范化较小端点编号的基础散列。
      // `std::hash<T>{}`先值初始化一个函数对象，后面的(...)调用其operator()。
      std::hash<std::uint32_t>{}(std::min(edge.first, edge.second));
  const std::size_t second_hash =  // 规范化较大端点编号的基础散列。
      std::hash<std::uint32_t>{}(std::max(edge.first, edge.second));
  // `^`为按位异或，配合移位混合两个端点散列；这只影响哈希桶，不改变相等判断。
  return first_hash ^ (second_hash + static_cast<std::size_t>(0x9e3779b9U) +
                       (first_hash << 6U) + (first_hash >> 2U));
}

DegradationMonitor::DegradationMonitor(DegradationConfig config)
    // 按值接收后复制到config_，确保调用者后续修改原配置不会改变监视器行为。
    : config_(config) {
  if (config_.window_ns == 0U ||
      !(config_.nominal_rate_hz > 0.0) ||
      !std::isfinite(config_.nominal_rate_hz) ||
      !unit_interval(config_.nlos_ratio_threshold) ||
      !unit_interval(config_.valid_ratio_threshold) ||
      !unit_interval(config_.rate_ratio_threshold) ||
      !unit_interval(config_.nlos_probability_threshold) ||
      !std::isfinite(config_.nlos_covariance_scale) ||
      config_.nlos_covariance_scale < 1.0 ||
      config_.max_tracked_edges == 0U ||
      config_.max_tracked_edges > kMaximumSamplesPerEdge ||
      config_.reject_duration_ns < config_.suspend_duration_ns) {
    throw std::invalid_argument("invalid degradation configuration");
  }
  // 根据窗口长度和标称频率推导期望样本数，并预留抖动余量限制内存。
  const long double raw_expected =  // 窗宽乘标称频率得到的未取整期望样本数。
      static_cast<long double>(config_.nominal_rate_hz) *
      static_cast<long double>(config_.window_ns) / 1.0e9L;
  const long double rounded_expected =  // 按最近整数取整后的期望样本数。
      std::floor(raw_expected + 0.5L);
  constexpr std::size_t maximum_expected =  // 在容量倍率和余量下仍不越界的最大期望数。
      (kMaximumSamplesPerEdge - kCapacityOverhead) / kCapacityMultiplier;
  if (!std::isfinite(raw_expected) || !std::isfinite(rounded_expected) ||
      rounded_expected < 1.0L ||
      rounded_expected > static_cast<long double>(maximum_expected)) {
    throw std::invalid_argument("degradation expected count is invalid");
  }
  expected_count_ = static_cast<std::size_t>(rounded_expected);
  // 上面的maximum_expected检查保证下面乘法和加法都不会超过硬上限。
  max_samples_ =
      expected_count_ * kCapacityMultiplier + kCapacityOverhead;
}

void DegradationMonitor::track(EdgeKey edge) {
  // 只有新边才占用资源；已跟踪边重复track不受数量上限影响。
  if (edges_.find(edge) == edges_.end() &&
      edges_.size() >= config_.max_tracked_edges) {
    throw std::invalid_argument("too many tracked degradation edges");
  }
  // iterator定位本边记录；inserted区分新建记录与已存在记录。
  // 结构化绑定把pair拆成iterator和inserted；try_emplace仅在键不存在时构造空EdgeRecord。
  auto [iterator, inserted] = edges_.try_emplace(edge);
  if (inserted) {
    // map迭代器指向pair<const EdgeKey,EdgeRecord>，second才是可修改记录。
    iterator->second.quality.edge = edge;
    iterator->second.quality.expected_count = expected_count_;
    if (started_) {
      iterator->second.start_timestamp_ns =
          std::max(now_ns_, start_timestamp_ns_);
      iterator->second.started = true;
    }
  }
}

void DegradationMonitor::start_all(std::uint64_t timestamp_ns) {
  if (started_) {
    // 全局起点只允许建立一次，后续更晚输入不能重置成熟计时。
    return;
  }
  started_ = true;
  start_timestamp_ns_ = timestamp_ns;
  for (auto& entry : edges_) {  // entry为待同步启动成熟计时的边键/记录。
    // auto&使用可写引用，直接修改unordered_map内部EdgeRecord而不复制。
    entry.second.start_timestamp_ns = timestamp_ns;
    entry.second.started = true;
  }
}

void DegradationMonitor::record(const RangePacket& packet) {
  // 该重载只负责把完整包拆成质量字段，再转发给统一的逐字段实现。
  record(EdgeKey(packet.from_node, packet.to_node), packet.timestamp_ns,
         packet.valid, packet.nlos_flag, packet.has_nlos_probability,
         // 概率在包内是float，这里显式提升为double与配置阈值同精度比较。
         static_cast<double>(packet.nlos_probability));
}

void DegradationMonitor::record(EdgeKey edge, std::uint64_t timestamp_ns,
                                bool valid, bool nlos_flag,
                                bool has_nlos_probability,
                                double nlos_probability) {
  // 记录阶段只追加质量证据；状态转换集中在evaluate，避免不同入口采用不同口径。
  if (edges_.find(edge) == edges_.end() &&
      edges_.size() >= config_.max_tracked_edges) {
    throw std::invalid_argument("too many tracked degradation edges");
  }
  // iterator定位本边记录；inserted指示是否需初始化公开快照。
  // 若边尚不存在就原位创建；结构化绑定同时得到位置和“是否新建”标志。
  auto [iterator, inserted] = edges_.try_emplace(edge);
  auto& record = iterator->second;  // 当前样本所属边的可变滑窗/状态记录。
  if (inserted) {
    record.quality.edge = edge;
    record.quality.expected_count = expected_count_;
  }
  start_all(timestamp_ns);
  if (!record.started) {
    if (timestamp_ns < start_timestamp_ns_) {
      // 新边收到的历史样本早于全局起点，不允许倒置该边成熟窗口。
      return;
    }
    record.start_timestamp_ns = timestamp_ns;
    record.started = true;
  }
  if (timestamp_ns < record.start_timestamp_ns ||
      (record.has_latest_processed_timestamp &&
       timestamp_ns < record.latest_processed_timestamp_ns)) {
    return;
  }
  record.latest_processed_timestamp_ns = timestamp_ns;
  record.has_latest_processed_timestamp = true;
  const bool probability_nlos =  // 有效概率字段是否达到配置的NLOS判定门限。
      // `&&`短路：没有概率字段时不会继续把默认概率与阈值比较。
      has_nlos_probability && std::isfinite(nlos_probability) &&
      nlos_probability >= config_.nlos_probability_threshold;
  const Sample sample{timestamp_ns, valid, nlos_flag || probability_nlos,
                      false};  // 追加到滑窗的最小质量证据。
  const auto insertion = std::upper_bound(  // 保持同时间样本稳定追加的插入位置。
      record.samples.begin(), record.samples.end(), timestamp_ns,
      [](std::uint64_t timestamp, const Sample& existing) {
        // `[]`为空捕获Lambda，不读取外部变量；upper_bound用它完成二分比较。
        // timestamp为待插入测量时间；existing为二分探查的既有样本。
        return timestamp < existing.timestamp_ns;
      });
  record.samples.insert(insertion, sample);
  // 超过硬容量时从最旧端持续弹出，直至deque重新满足上限。
  while (record.samples.size() > max_samples_) {
    record.samples.pop_front();
    ++record.dropped_count;
    record.last_overflow_ns = timestamp_ns;
    record.has_overflow_event = true;
  }
}

void DegradationMonitor::record_residual_rejection(
    EdgeKey edge, std::uint64_t timestamp_ns) {
  const auto found = edges_.find(edge);  // 待回写残差拒绝标志的边记录位置。
  if (found == edges_.end()) {
    return;
  }
  for (auto sample = found->second.samples.rbegin();  // sample从最新样本反向寻找同时间量测。
       sample != found->second.samples.rend(); ++sample) {
    // 反向迭代器的`->`访问其指向Sample成员；优先命中同时间的最新记录。
    if (sample->timestamp_ns == timestamp_ns) {
      sample->residual_rejected = true;
      return;
    }
  }
}

void DegradationMonitor::advance(std::uint64_t now_ns) {
  start_all(now_ns);
  // max保证监视器时间单调不倒退，即使调用者传入较早时刻也只重新评估当前时间。
  now_ns_ = std::max(now_ns_, now_ns);
  for (auto& entry : edges_) {  // entry为本次统一时刻需要评估的边记录。
    evaluate(entry.second, now_ns_);
  }
}

ObservationQuality DegradationMonitor::quality(EdgeKey edge) const {
  const auto found = edges_.find(edge);  // 查询边在监视表中的位置。
  if (found == edges_.end()) {
    ObservationQuality empty{};  // 未跟踪边仍返回带窗口边界和期望数的空快照。
    empty.edge = edge;
    empty.expected_count = expected_count_;
    empty.window_start_ns =
        // 三目运算避免now_ns_-window_ns在尚未满窗时发生uint64下溢。
        now_ns_ >= config_.window_ns ? now_ns_ - config_.window_ns : 0U;
    empty.window_end_ns = now_ns_;
    return empty;
  }
  return found->second.quality;
}

const DegradationConfig& DegradationMonitor::config() const noexcept {
  // const引用避免复制多个阈值，同时禁止调用者通过返回值修改内部配置。
  return config_;
}

FusionAction DegradationMonitor::action_for(
    ObservationState state) const noexcept {
  switch (state) {
    // switch按枚举值选择唯一动作；每个return直接结束函数，因此不需要break。
    case ObservationState::kDegraded:
      return FusionAction::kUseDownweighted;
    case ObservationState::kSuspended:
      return FusionAction::kHold;
    case ObservationState::kRejected:
      return FusionAction::kReject;
    case ObservationState::kRecovering:
      return FusionAction::kTrialRecovery;
    case ObservationState::kUnknown:
    case ObservationState::kNormal:
    default:
      // default为将来出现未知枚举值提供安全回退：正常使用但仍由其他入口检查有效性。
      return FusionAction::kUseNormal;
  }
}

void DegradationMonitor::evaluate(EdgeRecord& record,
                                  std::uint64_t now_ns) {
  // 阶段1：移除窗口外样本，再由剩余证据计算计数、比例和实际到达频率。
  const std::uint64_t window_start =  // 当前固定窗宽对应的左边界，避免无符号下溢。
      now_ns >= config_.window_ns ? now_ns - config_.window_ns : 0U;
  const bool full_window = now_ns >= config_.window_ns;  // 左边界是否已脱离时间原点。
  while (full_window && !record.samples.empty() &&
         record.samples.front().timestamp_ns <= window_start) {
    // front取得最旧Sample；成熟窗口采用左开右闭语义，等于左边界的样本也移除。
    record.samples.pop_front();
  }

  ObservationQuality next{};  // 由当前剩余样本重算并最终替换旧快照的候选结果。
  next.edge = record.quality.edge;
  next.window_start_ns = window_start;
  next.window_end_ns = now_ns;
  next.expected_count = expected_count_;
  next.dropped_count = record.dropped_count;
  next.input_overflow =
      // 最近溢出事件必须不晚于now，且仍落在当前窗内，才标记本快照overflow。
      record.has_overflow_event && record.last_overflow_ns <= now_ns &&
      (!full_window || record.last_overflow_ns > window_start);
  for (const auto& sample : record.samples) {  // sample为窗内待累计的单条质量证据。
    if (sample.timestamp_ns > now_ns) {
      // record允许按时间有序插入未来样本，但当前评估不能提前统计它。
      continue;
    }
    ++next.received_count;
    // 三目运算把bool转换成0或1，再用+=累计计数。
    next.valid_count += sample.valid ? 1U : 0U;
    next.nlos_count += sample.nlos ? 1U : 0U;
    next.residual_rejected_count += sample.residual_rejected ? 1U : 0U;
  }
  if (next.received_count != 0U) {
    // 仅在分母非0时计算NLOS比例；无样本时保留默认0。
    next.nlos_ratio = static_cast<double>(next.nlos_count) /
                      static_cast<double>(next.received_count);
  }
  next.valid_ratio = static_cast<double>(next.valid_count) /
                     static_cast<double>(expected_count_);
  next.actual_rate_hz =
      static_cast<double>(next.received_count) * 1.0e9 /
      static_cast<double>(config_.window_ns);

  const bool mature =  // 本边是否已积累至少一个完整配置窗宽。
      record.started && now_ns >= record.start_timestamp_ns &&
      now_ns - record.start_timestamp_ns >= config_.window_ns;
  if (!mature) {
    // 未积累完整窗宽时统计值可以展示，但不能据此进入退化状态。
    next.state = ObservationState::kUnknown;
    next.action = FusionAction::kUseNormal;
    record.quality = next;
    return;
  }

  // 阶段2：多个退化原因通过位图并存，GCS可同时展示全部触发依据。
  if (next.nlos_ratio >= config_.nlos_ratio_threshold) {
    next.reason_mask |= ReasonMask::NLOS_RATIO_HIGH;
  }
  if (next.valid_ratio < config_.valid_ratio_threshold) {
    next.reason_mask |= ReasonMask::VALID_RATIO_LOW;
  }
  const double minimum_rate =  // 标称频率乘允许比例得到的实际频率下限。
      config_.nominal_rate_hz * config_.rate_ratio_threshold;
  if (next.actual_rate_hz < minimum_rate) {
    next.reason_mask |= ReasonMask::RATE_LOW;
  }
  if (next.residual_rejected_count != 0U) {
    next.reason_mask |= ReasonMask::RANGE_RESIDUAL_HIGH;
  }
  if (next.input_overflow) {
    next.reason_mask |= ReasonMask::INPUT_OVERFLOW;
  }
  const bool bad =  // 本轮是否至少存在一种滑窗退化证据。
      next.reason_mask != ReasonMask::NONE;

  // 阶段3：保持时间抑制状态抖动；坏数据持续越久，状态逐级升级到暂缓和剔除。
  if (bad) {
    // 坏证据出现时取消连续好数据计时，两个保持计时器不会同时活动。
    record.has_good_since = false;
    if (!record.has_bad_since) {
      // 只在本段坏区间第一帧记录起点，后续坏帧不重置计时。
      record.bad_since_ns = now_ns;
      record.has_bad_since = true;
    }
    const std::uint64_t bad_duration =  // 当前连续坏证据已经维持的饱和时长。
        elapsed(now_ns, record.bad_since_ns);
    if (bad_duration >= config_.reject_duration_ns) {
      next.state = ObservationState::kRejected;
    } else if (bad_duration >= config_.suspend_duration_ns) {
      next.state = ObservationState::kSuspended;
    } else {
      next.state = ObservationState::kDegraded;
    }
  } else {
    // 本轮无退化原因，终止连续坏数据计时。
    record.has_bad_since = false;
    // 剔除或降级后的好数据必须经过试探恢复期，不能一帧即恢复正常融合。
    if (record.quality.state == ObservationState::kUnknown ||
        record.quality.state == ObservationState::kNormal) {
      next.state = ObservationState::kNormal;
      record.has_good_since = false;
    } else {
      if (!record.has_good_since) {
        // 从退化状态首次看到好窗口时启动恢复保持计时。
        record.good_since_ns = now_ns;
        record.has_good_since = true;
      }
      next.state = elapsed(now_ns, record.good_since_ns) >=
                           config_.recovery_duration_ns
                       // 三目运算：好状态持续够久转Normal，否则维持Recovering。
                       ? ObservationState::kNormal
                       : ObservationState::kRecovering;
      if (next.state == ObservationState::kNormal) {
        record.has_good_since = false;
      }
    }
  }
  next.action = action_for(next.state);
  // 只有仍融合的退化/恢复状态放大R；Hold/Reject不调用滤波更新，无需倍率。
  if (next.state == ObservationState::kDegraded ||
      next.state == ObservationState::kRecovering) {
    next.covariance_scale = config_.nlos_covariance_scale;
  }
  record.quality = next;
}

}  // namespace zju::coop

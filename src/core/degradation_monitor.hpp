// 模块职责：按协同边维护观测质量滑窗，并把NLOS、有效率、频率、残差等证据
// 转换为正常、降权、暂缓、剔除和试探恢复状态；该模块只决定融合动作，
// 不直接修改滤波状态，也不承担无线链路维护。
//
// 初学者可把它看成“每条测距边的质量记分员”：
// record()只保存最近一段时间的样本；evaluate()统计NLOS比例、有效率、频率和残差；
// 状态机根据坏状态持续多久决定正常使用、降低权重、暂缓、剔除或试探恢复。
// 它只给出建议动作，真正的Kalman更新仍由滤波器完成。
#pragma once

#include "zju_coop/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>

namespace zju::coop {

/** 无向协同边键；构造时规范化节点顺序，使1-2与2-1共享质量状态。 */
struct EdgeKey {
  std::uint32_t first{};   ///< 规范化后编号较小的端点。
  std::uint32_t second{};  ///< 规范化后编号较大的端点。

  /** `= default`要求编译器生成默认构造函数，两个整数仍按成员后的{}初始化为0。 */
  EdgeKey() = default;
  /** @param node_a 边的任一端点；@param node_b 边的另一端点。 */
  EdgeKey(std::uint32_t node_a, std::uint32_t node_b) noexcept;

  /** @param other 待与本无向边比较的键。 */
  [[nodiscard]] bool operator==(const EdgeKey& other) const noexcept;
  /** @param other 待与本无向边比较的键。 */
  [[nodiscard]] bool operator!=(const EdgeKey& other) const noexcept;
};

struct EdgeKeyHash {
  /** @param edge 待散列的规范化无向边键。 */
  [[nodiscard]] std::size_t operator()(const EdgeKey& edge) const noexcept;
};

enum class ReasonMask : std::uint32_t {
  NONE = 0U,                         ///< 没有退化原因，所有位均为0。
  NLOS_RATIO_HIGH = 1U << 0U,       ///< 窗口NLOS比例达到或超过阈值。
  VALID_RATIO_LOW = 1U << 1U,       ///< 窗口有效样本数相对期望数不足。
  RATE_LOW = 1U << 2U,              ///< 实际到达频率低于标称频率比例门限。
  RANGE_RESIDUAL_HIGH = 1U << 3U,   ///< 窗内至少有一次测距被NIS残差门限拒绝。
  TIME_SYNC_TIMEOUT = 1U << 4U,     ///< 外部时间同步状态超时，由上层合并。
  LINK_TIMEOUT = 1U << 5U,          ///< 通信链路状态超时，由上层合并。
  GRAPH_GEOMETRY_DEGENERATE = 1U << 6U, ///< 当前刚度矩阵秩不足。
  NODE_UNREACHABLE = 1U << 7U,      ///< 至少一个节点无法从主参考沿有效边到达。
  INITIALIZATION_MISSING = 1U << 8U,///< 必要初始状态尚未建立。
  INPUT_OVERFLOW = 1U << 9U,        ///< 单边样本缓存超过安全上限并发生丢弃。
};

/** @param left 已有原因位；@param right 待合并的原因位。 */
[[nodiscard]] constexpr ReasonMask operator|(ReasonMask left,
                                             ReasonMask right) noexcept {
  return static_cast<ReasonMask>(static_cast<std::uint32_t>(left) |
                                 static_cast<std::uint32_t>(right));
}

/** @param left 原位累加原因的位图；@param right 待并入的原因位。 */
constexpr ReasonMask& operator|=(ReasonMask& left, ReasonMask right) noexcept {
  // left是可写引用，因此赋值会直接修改调用方原变量；返回引用支持连续组合。
  left = left | right;
  return left;
}

/** @param mask 待检查的综合原因位图；@param reason 要求完整包含的原因位。 */
[[nodiscard]] constexpr bool has_reason(ReasonMask mask,
                                        ReasonMask reason) noexcept {
  // 先排除NONE，再用按位与确认reason中的所有1位都出现在mask中。
  return reason != ReasonMask::NONE &&
         (static_cast<std::uint32_t>(mask) &
          static_cast<std::uint32_t>(reason)) ==
             static_cast<std::uint32_t>(reason);
}

/**
 * 退化判据、状态保持时间和资源上限，全部由配置文件提供。
 * 比率阈值取[0,1]；三个duration是“坏/好证据持续多久才换状态”，
 * window_ns则决定统计样本范围，两类时间不能互相替代。
 */
struct DegradationConfig {
  std::uint64_t window_ns{2'000'000'000ULL};  ///< 质量统计滑窗宽度，单位ns。
  double nominal_rate_hz{20.0};               ///< 单边量测的标称到达频率，单位Hz。
  double nlos_ratio_threshold{0.30};          ///< 触发NLOS比例过高原因的下限。
  double valid_ratio_threshold{0.80};         ///< 触发有效率过低原因的下限。
  double rate_ratio_threshold{0.80};          ///< 实际频率相对标称频率的最低允许比例。
  double nlos_probability_threshold{0.50};    ///< 将概率量测判作NLOS的概率下限。
  double nlos_covariance_scale{4.0};          ///< 降权或试探恢复时的测距协方差倍率。
  std::uint64_t suspend_duration_ns{1'000'000'000ULL};  ///< 坏证据升级为暂缓所需时长。
  std::uint64_t reject_duration_ns{3'000'000'000ULL};   ///< 坏证据升级为剔除所需时长。
  std::uint64_t recovery_duration_ns{1'000'000'000ULL}; ///< 好证据恢复正常所需时长。
  std::size_t max_tracked_edges{2016U};  ///< 监视器可持有的无向边记录上限。
};

/** 当前滑窗的可审计统计快照，也是GCS观测状态输出的数据来源。 */
struct ObservationQuality {
  EdgeKey edge{};                    ///< 本快照对应的无向协同边。
  std::uint64_t window_start_ns{};   ///< 当前统计滑窗左边界。
  std::uint64_t window_end_ns{};     ///< 当前统计滑窗的统一评估时刻。
  std::size_t expected_count{};      ///< 按标称频率推导的窗口期望样本数。
  std::size_t received_count{};      ///< 窗内不晚于评估时刻的全部样本数。
  std::size_t valid_count{};         ///< 窗内通过结构有效性检查的样本数。
  std::size_t nlos_count{};          ///< 窗内由标志或概率判作NLOS的样本数。
  std::size_t residual_rejected_count{};  ///< 窗内被滤波残差门限拒绝的样本数。
  std::size_t dropped_count{};       ///< 因单边缓存容量限制累计丢弃的样本数。
  double nlos_ratio{};               ///< NLOS样本占已接收样本的比例。
  double valid_ratio{};              ///< 有效样本占窗口期望样本的比例。
  double actual_rate_hz{};           ///< 由窗内接收数和固定窗宽计算的实际频率。
  bool input_overflow{};             ///< 当前窗口内是否发生过样本缓存溢出。
  ObservationState state{ObservationState::kUnknown};  ///< 保持时间状态机的当前状态。
  FusionAction action{FusionAction::kUseNormal};       ///< 当前状态映射出的融合动作。
  ReasonMask reason_mask{ReasonMask::NONE};            ///< 本次评估同时成立的退化原因位图。
  double covariance_scale{1.0};      ///< 当前快照建议的测距协方差倍率。
};

class DegradationMonitor {
 public:
  /** @param config 退化判据、保持时长及缓存资源限制。 */
  // explicit禁止把DegradationConfig隐式当作DegradationMonitor；`={}`允许无参时使用默认配置。
  explicit DegradationMonitor(DegradationConfig config = {});

  /** 预注册可能出现的无向边；超过资源上限时抛错而不是静默丢弃。
   * @param edge 要预注册并持续统计的无向边。 */
  void track(EdgeKey edge);
  /** 记录原始量测质量证据；无效包也进入窗口，避免只统计成功量测。
   * @param packet 提供边端点、测量时间及原始质量证据的测距包。 */
  void record(const RangePacket& packet);
  /**
   * @param edge 样本所属无向边；@param timestamp_ns 样本的统一测量时间。
   * @param valid 是否通过上层结构有效性检查；@param nlos_flag 设备给出的NLOS标志。
   * @param has_nlos_probability 概率字段是否有效；@param nlos_probability NLOS概率值。
   */
  void record(EdgeKey edge, std::uint64_t timestamp_ns, bool valid,
              bool nlos_flag = false, bool has_nlos_probability = false,
              double nlos_probability = 0.0);
  /** 把同一时刻的NIS拒绝回写到已有样本，形成残差退化证据。
   * @param edge 被拒绝样本所属边；@param timestamp_ns 用于反查样本的测量时间。 */
  void record_residual_rejection(EdgeKey edge,
                                 std::uint64_t timestamp_ns);
  /** 推进所有边的时间窗口和状态机；内部时间只单调取最大值。
   * @param now_ns 用于裁剪滑窗并评估状态的统一当前时间。 */
  void advance(std::uint64_t now_ns);

  /** @param edge 要查询的无向边。 */
  [[nodiscard]] ObservationQuality quality(EdgeKey edge) const;
  /** 返回只读引用避免复制配置；引用不能在监视器对象销毁后继续使用。 */
  [[nodiscard]] const DegradationConfig& config() const noexcept;

 private:
  struct Sample {
    // 样本只保存重算滑窗统计所需的最小证据，避免长期保存完整测距包。
    std::uint64_t timestamp_ns{};  ///< 样本的统一测量时间，用于排序和滑窗裁剪。
    bool valid{};                  ///< 样本是否通过结构有效性检查。
    bool nlos{};                   ///< 样本是否由标志或概率判作NLOS。
    bool residual_rejected{};      ///< 样本是否随后被残差门限拒绝。
  };

  struct EdgeRecord {
    std::deque<Sample> samples;            ///< 按测量时间排序的单边滑窗样本缓存。
    ObservationQuality quality{};          ///< 最近一次完成评估的公开快照。
    std::uint64_t start_timestamp_ns{};    ///< 本边成熟窗口的计时起点。
    std::uint64_t latest_processed_timestamp_ns{};  ///< 已接受样本的最大测量时间。
    // bad/good_since分别实现恶化和恢复保持时间，二者不会同时有效。
    std::uint64_t bad_since_ns{};           ///< 当前连续坏证据区间的起点。
    std::uint64_t good_since_ns{};          ///< 当前连续好证据区间的起点。
    bool started{};                         ///< 本边是否已建立成熟窗口计时起点。
    bool has_latest_processed_timestamp{};  ///< 最大测量时间字段是否已初始化。
    bool has_bad_since{};                   ///< 坏证据持续计时是否处于活动状态。
    bool has_good_since{};                  ///< 好证据持续计时是否处于活动状态。
    std::size_t dropped_count{};            ///< 单边缓存溢出的累计丢样数。
    std::uint64_t last_overflow_ns{};       ///< 最近一次缓存溢出的样本时间。
    bool has_overflow_event{};              ///< 是否至少记录过一次缓存溢出。
  };

  /** @param record 待原位更新的单边状态；@param now_ns 本次统一评估时刻。 */
  void evaluate(EdgeRecord& record, std::uint64_t now_ns);
  /** @param timestamp_ns 全局及既有边的成熟窗口计时起点。 */
  void start_all(std::uint64_t timestamp_ns);
  /** @param state 待映射为融合策略的观测状态。 */
  [[nodiscard]] FusionAction action_for(ObservationState state) const noexcept;

  DegradationConfig config_;       ///< 经构造函数校验后固定使用的退化配置。
  std::size_t expected_count_{};   ///< 每窗期望样本数的预计算整数值。
  std::size_t max_samples_{};      ///< 单边缓存允许保留的样本数硬上限。
  std::uint64_t now_ns_{};         ///< 监视器已推进到的最大统一时间。
  std::uint64_t start_timestamp_ns_{};  ///< 首次record或advance建立的全局成熟窗口起点。
  bool started_{};                 ///< 全局成熟窗口计时是否已经启动。
  std::unordered_map<EdgeKey, EdgeRecord, EdgeKeyHash> edges_;  ///< 无向边到其滑窗/状态记录的索引。
};

}  // namespace zju::coop

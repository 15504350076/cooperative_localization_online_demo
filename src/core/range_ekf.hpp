// 模块职责：提供缺少IMU时的二维恒速+平台间测距兼容滤波路径。
// 当前交付默认使用IMU+测距的15维联合滤波；本模块保留用于故障回退、历史数据
// 回归和无IMU环境演示，Engine保证两种预测模型不会同时推进同一状态。
// 维度记号：M为非参考节点数，参考节点固定为零状态且不占四维状态块。
#pragma once

#include "core/dense_matrix.hpp"
#include "zju_coop/types.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace zju::coop {

/** 仅测距兼容滤波器的平面位置、速度及初始标准差。 */
struct NodeInitialization {
  std::uint32_t node_id{};  ///< 平台编号，可不连续但在初始化数组中必须唯一。
  double x{};               ///< 初始平面x位置，单位m；构造时转换为相对参考节点坐标。
  double y{};               ///< 初始平面y位置，单位m；构造时转换为相对参考节点坐标。
  double vx{};              ///< 初始x方向速度，单位m/s；构造时转换为相对速度。
  double vy{};              ///< 初始y方向速度，单位m/s；构造时转换为相对速度。
  double position_std_m{};  ///< x、y位置共用的初始标准差，单位m。
  double velocity_std_mps{};  ///< vx、vy速度共用的初始标准差，单位m/s。
};

/** 恒速过程模型、NIS门限和数值稳定参数。 */
struct FilterConfig {
  std::uint32_t reference_node_id{};  ///< 固定在零状态、定义相对坐标原点的主参考平台编号。
  double process_accel_std_mps2{};    ///< 各平面轴独立白噪声加速度标准差，单位m/s²。
  double nis_gate{};                  ///< 一维距离创新的无量纲NIS接纳上限。
  double max_prediction_step_s{};     ///< 推导分段恒速过程噪声时采用的最大等效子步，单位s。
  double min_covariance_diagonal{};   ///< 协方差正定稳定化使用的对角线数值下限。
};

/**
 * UWB-only兼容路径的主参考二维相对状态。
 * valid只表示该节点存在于已初始化状态，三边测距仍不能给出全球平移、
 * 旋转、镜像和绝对航向；这些自由度由参考节点和初始几何固定。
 */
struct NodeEstimate {
  std::uint32_t node_id{};    ///< 本估计所属的平台编号。
  std::uint64_t timestamp_ns{};  ///< 滤波状态对应的统一传感器时间，单位ns。
  double x{};                 ///< 相对参考节点的平面x位置，单位m。
  double y{};                 ///< 相对参考节点的平面y位置，单位m。
  double vx{};                ///< 相对参考节点的x方向速度，单位m/s。
  double vy{};                ///< 相对参考节点的y方向速度，单位m/s。
  double cov_xx{};            ///< 相对位置x方差，单位m²。
  double cov_xy{};            ///< 相对位置x-y协方差，单位m²。
  double cov_yy{};            ///< 相对位置y方差，单位m²。
  bool valid{};               ///< `true`表示节点存在且所返回状态/协方差可供下游使用。
};

/** 拒绝原因保持互斥，便于区分输入问题、统计门限和矩阵数值失败。 */
enum class UpdateDisposition {
  Accepted,
  InvalidPacket,
  UnknownNode,
  SelfRange,
  NonPositiveRange,
  OutOfOrder,
  NisRejected,
  NumericalFailure,
};

/** 单次测距更新诊断；即使拒绝量测也保留创新、方差和NIS便于告警分析。 */
struct UpdateResult {
  UpdateDisposition disposition{UpdateDisposition::InvalidPacket};  ///< 本次测距更新的互斥处理分类。
  double innovation_m{};       ///< 实测距离减预测距离的创新，单位m。
  double innovation_variance{};  ///< S=HPHᵀ+R的一维创新方差，单位m²。
  double nis{};                ///< innovation²/S得到的无量纲归一化创新平方。
  double covariance_scale{1.0};  ///< 实际用于放大量测方差R的不小于1的质量降权倍数。
};

class RangeEkf {
 public:
  /**
   * 二维恒速UWB距离EKF，仅用于不提供IMU时的兼容演示。
   * 生产惯性路径由CooperativeInertialEkf实现，二者不同时预测。
   * `config`给出参考节点、过程噪声、NIS与数值门限；
   * `initializations`的顺序固定节点记录，每项给出共同平面坐标下的初值及标准差。
   */
  RangeEkf(FilterConfig config,
           std::vector<NodeInitialization> initializations);

  /** `timestamp_ns`为目标统一传感器时间，单位ns；时间不前进时保持原状态。 */
  void predict_to(std::uint64_t timestamp_ns);
  /**
   * `packet`给出平台间平面欧氏距离与标准差，单位m；`covariance_scale`
   * 是不小于1的量测方差放大倍数。
   */
  [[nodiscard]] UpdateResult update(const RangePacket& packet,
                                    double covariance_scale = 1.0);

  /** `node_id`指定待查询的平台；未知编号返回`valid=false`及NaN数值。 */
  [[nodiscard]] NodeEstimate estimate(std::uint32_t node_id) const;
  [[nodiscard]] std::vector<NodeEstimate> estimates() const;
  [[nodiscard]] const DenseMatrix& covariance() const noexcept;

 private:
  struct NodeRecord {
    std::uint32_t node_id{};  ///< 本记录对应的平台编号。
    std::size_t offset{};     ///< 非参考节点在`state_`中[x,y,vx,vy]四维块的起始下标。
    bool reference{};         ///< `true`表示该节点固定为相对坐标零状态，不占用四维块。
  };

  /** `node_id`为查找键；返回指针借用`nodes_`存储，仅在对象不变更节点表时有效。 */
  [[nodiscard]] const NodeRecord* find_node(std::uint32_t node_id) const;
  /** `node`为`nodes_`中的有效记录，用于组装对应相对状态输出。 */
  [[nodiscard]] NodeEstimate make_estimate(const NodeRecord& node) const;
  [[nodiscard]] bool finite_state_and_covariance() const;
  /** `total_seconds`为正预测区间；`step_count`为闭式过程噪声对应的等效等长子步数。 */
  void predict_interval(double total_seconds, long double step_count);

  FilterConfig config_;  ///< 构造时校验后固定的参考节点、过程噪声与数值门限。
  std::vector<NodeRecord> nodes_;  ///< 按初始化顺序持有节点记录，生命周期与滤波器相同。
  std::unordered_map<std::uint32_t, std::size_t> node_lookup_;  ///< 平台编号到`nodes_`下标的查找表。
  std::vector<double> state_;  ///< 仅含非参考节点的4M维相对状态，块内顺序[x,y,vx,vy]。
  DenseMatrix covariance_;     ///< 与`state_`同序的4M×4M联合协方差，含跨节点块。
  std::uint64_t last_timestamp_ns_{};  ///< 当前预测状态对应的统一传感器时间，单位ns。
  bool has_timebase_{};  ///< `true`表示`last_timestamp_ns_`已由首个预测或测距建立。
};

}  // namespace zju::coop

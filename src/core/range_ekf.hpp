// 模块职责：提供缺少IMU时的二维恒速+平台间测距兼容滤波路径。
// 当前交付默认使用IMU+测距的15维联合滤波；本模块保留用于故障回退、历史数据
// 回归和无IMU环境演示，Engine保证两种预测模型不会同时推进同一状态。
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
  std::uint32_t node_id{};
  double x{};
  double y{};
  double vx{};
  double vy{};
  double position_std_m{};
  double velocity_std_mps{};
};

/** 恒速过程模型、NIS门限和数值稳定参数。 */
struct FilterConfig {
  std::uint32_t reference_node_id{};
  double process_accel_std_mps2{};
  double nis_gate{};
  double max_prediction_step_s{};
  double min_covariance_diagonal{};
};

/**
 * UWB-only兼容路径的主参考二维相对状态。
 * valid只表示该节点存在于已初始化状态，三边测距仍不能给出全球平移、
 * 旋转、镜像和绝对航向；这些自由度由参考节点和初始几何固定。
 */
struct NodeEstimate {
  std::uint32_t node_id{};
  std::uint64_t timestamp_ns{};
  double x{};
  double y{};
  double vx{};
  double vy{};
  double cov_xx{};
  double cov_xy{};
  double cov_yy{};
  bool valid{};
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
  UpdateDisposition disposition{UpdateDisposition::InvalidPacket};
  double innovation_m{};
  double innovation_variance{};
  double nis{};
  double covariance_scale{1.0};
};

class RangeEkf {
 public:
  /**
   * 二维恒速UWB距离EKF，仅用于不提供IMU时的兼容演示。
   * 生产惯性路径由CooperativeInertialEkf实现，二者不同时预测。
   */
  RangeEkf(FilterConfig config,
           std::vector<NodeInitialization> initializations);

  /** 以分段恒速模型预测到目标时刻；时间不前进时保持原状态。 */
  void predict_to(std::uint64_t timestamp_ns);
  [[nodiscard]] UpdateResult update(const RangePacket& packet,
                                    double covariance_scale = 1.0);

  [[nodiscard]] NodeEstimate estimate(std::uint32_t node_id) const;
  [[nodiscard]] std::vector<NodeEstimate> estimates() const;
  [[nodiscard]] const DenseMatrix& covariance() const noexcept;

 private:
  struct NodeRecord {
    std::uint32_t node_id{};
    std::size_t offset{};
    bool reference{};
  };

  [[nodiscard]] const NodeRecord* find_node(std::uint32_t node_id) const;
  [[nodiscard]] NodeEstimate make_estimate(const NodeRecord& node) const;
  [[nodiscard]] bool finite_state_and_covariance() const;
  void predict_interval(double total_seconds, long double step_count);

  FilterConfig config_;
  std::vector<NodeRecord> nodes_;
  std::unordered_map<std::uint32_t, std::size_t> node_lookup_;
  std::vector<double> state_;
  DenseMatrix covariance_;
  std::uint64_t last_timestamp_ns_{};
  bool has_timebase_{};
};

}  // namespace zju::coop

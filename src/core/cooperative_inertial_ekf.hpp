// 模块职责：在各节点独立IMU名义状态之上维护15N联合协方差，并融合节点间测距。
// 关键设计：节点编号可以不连续，状态块顺序由初始化数组确定；交叉协方差不能丢弃，
// 否则一次平台间测距无法把约束正确传播到协同网络中的其他相关节点。
#pragma once

#include "core/dense_matrix.hpp"
#include "core/inertial_eskf15.hpp"
#include "core/range_ekf.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace zju::coop {

/** 多节点15维误差状态联合滤波器的资源、参考节点和量测门限配置。 */
struct CooperativeInertialConfig {
  std::uint32_t reference_node_id{};
  double nis_gate{9.0};
  double min_covariance_diagonal{1.0e-12};
  std::size_t max_inertial_state_dimension{300U};
};

/**
 * 多平台15N联合ESKF。
 * 每节点名义状态由独立IMU传播器维护，联合协方差保留节点间相关性，
 * 状态块顺序严格跟随构造时的节点数组，不要求节点编号连续。
 */
class CooperativeInertialEkf {
 public:
  CooperativeInertialEkf(
      CooperativeInertialConfig cooperative_config,
      InertialConfig inertial_config,
      std::vector<InertialNodeInitialization> initializations);

  /**
   * 输入一帧标准化IMU瞬时量。名义状态只传播所属节点；联合协方差执行
   * P_ii=Phi*P_ii*Phi^T+Qd，同时用Phi更新该节点与其他节点的交叉块。
   */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);

  /**
   * 使用节点间三维欧氏距离更新完整15N状态，并返回创新/NIS诊断。
   * H只在两端节点的位置误差块非零，但K=P*H^T会把约束传播到所有相关节点。
   */
  [[nodiscard]] UpdateResult update_range(const RangePacket& packet,
                                          double covariance_scale = 1.0);

  /**
   * 输出相对主参考节点的平面位置、速度和相对位置协方差。
   * 协方差使用Pii+Prr-Pir-Pri，不能简单相加两个节点的边缘方差。
   */
  [[nodiscard]] NodeEstimate estimate(std::uint32_t node_id) const;
  [[nodiscard]] std::vector<NodeEstimate> estimates() const;

  [[nodiscard]] const InertialNominalState& state(
      std::uint32_t node_id) const;
  [[nodiscard]] const DenseMatrix& covariance() const noexcept;
  [[nodiscard]] std::size_t state_dimension() const noexcept;
  [[nodiscard]] const std::vector<std::uint32_t>& node_ids() const noexcept;

 private:
  [[nodiscard]] bool valid_covariance(const DenseMatrix& covariance) const;
  [[nodiscard]] std::uint64_t latest_timestamp_ns() const noexcept;

  CooperativeInertialConfig config_;
  std::vector<InertialEskf15> filters_;
  std::vector<std::uint32_t> node_ids_;
  std::unordered_map<std::uint32_t, std::size_t> node_lookup_;
  DenseMatrix covariance_;
  std::uint64_t last_range_timestamp_ns_{};
  bool has_range_timebase_{};
};

}  // namespace zju::coop

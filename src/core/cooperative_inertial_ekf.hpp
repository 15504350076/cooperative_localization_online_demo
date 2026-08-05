// 模块职责：在各节点独立IMU名义状态之上维护15N联合协方差，并融合节点间测距。
// 关键设计：节点编号可以不连续，状态块顺序由初始化数组确定；交叉协方差不能丢弃，
// 否则一次平台间测距无法把约束正确传播到协同网络中的其他相关节点。
//
// 初学者阅读提示：
// - N辆车各有15维误差，因此联合协方差是15N×15N；对角块描述单车不确定度，
//   非对角块描述车辆之间“误差是否一起变化”的相关性。
// - 每车IMU只直接传播本车名义状态，但与本车相关的协方差行列也必须一起传播。
// - 一条测距同时涉及两车位置，Kalman增益再借助交叉协方差把修正传给相关状态。
// 建议先读对应测试，再进入update_range()中的大矩阵计算。
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
  std::uint32_t reference_node_id{};  ///< 相对估计输出所采用的主参考平台编号。
  double nis_gate{9.0};               ///< 一维距离创新的无量纲NIS接纳上限。
  double min_covariance_diagonal{1.0e-12};  ///< 15N联合协方差对角线允许的数值下限。
  std::size_t max_inertial_state_dimension{300U};  ///< 可分配的15N联合误差状态最大维数。
};

/**
 * 多平台15N联合ESKF。
 * 每节点名义状态由独立IMU传播器维护，联合协方差保留节点间相关性，
 * 状态块顺序严格跟随构造时的节点数组，不要求节点编号连续。
 */
class CooperativeInertialEkf {
 public:
  /**
   * `cooperative_config`给出参考节点、NIS门限和15N资源上限；
   * `inertial_config`由所有单节点传播器共享；`initializations`的顺序固定
   * 节点15维块布局，且每项提供对应名义初值和五组三轴标准差。
   */
  CooperativeInertialEkf(
      CooperativeInertialConfig cooperative_config,
      InertialConfig inertial_config,
      std::vector<InertialNodeInitialization> initializations);

  /**
   * `packet`输入一帧标准化ROS 2 IMU瞬时量：角速度为车体FLU瞬时量，
   * 单位rad/s；线加速度按车体FLU瞬时比力解释，单位m/s²，含重力影响，
   * 传播时旋转到导航ENU并加入-z方向重力。名义状态只传播所属节点；联合协方差执行
   * P_ii=Phi*P_ii*Phi^T+Qd，同时用Phi更新该节点与其他节点的交叉块。
   */
  [[nodiscard]] ImuProcessingResult push_imu(const ImuPacket& packet);

  /**
   * `packet`给出节点间三维欧氏距离及标准差，单位m；`covariance_scale`
   * 是不小于1的量测方差放大倍数。更新完整15N状态，并返回创新/NIS诊断。
   * H只在两端节点的位置误差块非零，但K=P*H^T会把约束传播到所有相关节点。
   */
  [[nodiscard]] UpdateResult update_range(const RangePacket& packet,
                                          double covariance_scale = 1.0);

  /**
   * `node_id`指定待输出的平台。输出相对主参考节点的平面位置、速度和相对位置协方差。
   * 协方差使用Pii+Prr-Pir-Pri，不能简单相加两个节点的边缘方差。
   */
  [[nodiscard]] NodeEstimate estimate(std::uint32_t node_id) const;
  [[nodiscard]] std::vector<NodeEstimate> estimates() const;

  /** `node_id`指定要查询的已初始化平台，未知编号抛出异常。 */
  [[nodiscard]] const InertialNominalState& state(
      std::uint32_t node_id) const;
  /** 返回15N联合协方差只读引用，避免复制大矩阵。 */
  [[nodiscard]] const DenseMatrix& covariance() const noexcept;
  /** 返回联合误差状态维数15N，不是名义状态对象数量。 */
  [[nodiscard]] std::size_t state_dimension() const noexcept;
  /** 返回与15维状态块顺序一致的节点编号vector只读引用。 */
  [[nodiscard]] const std::vector<std::uint32_t>& node_ids() const noexcept;

 private:
  /** `covariance`为待检查有限值与对角下限的15N×15N联合协方差候选。 */
  [[nodiscard]] bool valid_covariance(const DenseMatrix& covariance) const;
  /** 遍历各节点传播器并返回最大的已接纳IMU时间。 */
  [[nodiscard]] std::uint64_t latest_timestamp_ns() const noexcept;

  CooperativeInertialConfig config_;  ///< 构造后固定的参考节点、NIS门限和资源上限。
  std::vector<InertialEskf15> filters_;  ///< 按初始化顺序持有各节点名义状态传播器。
  std::vector<std::uint32_t> node_ids_;  ///< 与15维状态块顺序一一对应的平台编号。
  std::unordered_map<std::uint32_t, std::size_t> node_lookup_;  ///< 平台编号到`filters_`及15维块序号的映射。
  DenseMatrix covariance_;  ///< 15N×15N联合误差协方差，行列均按节点及δp/δv/δθ/δbg/δba排列。
  std::uint64_t last_range_timestamp_ns_{};  ///< 已消费测距的最新统一传感器时间，单位ns。
  bool has_range_timebase_{};  ///< `true`表示`last_range_timestamp_ns_`已由接纳或NIS拒绝的测距建立。
};

}  // namespace zju::coop

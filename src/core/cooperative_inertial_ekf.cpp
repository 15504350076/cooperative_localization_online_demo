// 模块实现：维护多节点名义惯性状态和完整15N联合协方差，融合平台间三维距离观测。
// 关键原则：IMU只传播所属节点的状态块和相关交叉块；测距更新作用于完整联合状态，
// 误差注入或协方差数值检查失败时整次更新回滚，不留下半更新状态。
//
// 初学者阅读主线：
// 构造函数建立节点顺序和初始15N协方差；push_imu()把某一节点的Phi/Qd嵌入联合矩阵；
// update_range()计算“实测距离-预测距离”，通过NIS判断异常，再用Kalman增益修正所有相关状态；
// estimate()最后把节点状态转换成“相对主参考节点”的输出。
// 大矩阵循环本质仍是课本公式P=Phi*P*Phi^T+Q和K=P*H^T/(H*P*H^T+R)。
#include "core/cooperative_inertial_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

// `value`是应严格大于零的门限或资源配置标量。
bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

// `value`是允许为零的初始标准差分量。
bool finite_nonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

// `covariance`是15N×15N联合协方差；`offset`是目标三维误差块起始下标；
// `standard_deviation`给出该块三轴初始标准差；`minimum_diagonal`是方差下限。
void set_initial_variance(DenseMatrix& covariance, std::size_t offset,
                          const Vec3& standard_deviation,
                          double minimum_diagonal) {
  // `values`按x/y/z排列，便于沿目标3×3块对角线逐轴写入方差。
  const double values[3]{standard_deviation.x, standard_deviation.y,
                         standard_deviation.z};
  // `axis`同步遍历三维误差块的x/y/z行列下标。
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    covariance(offset + axis, offset + axis) =
        std::max(values[axis] * values[axis], minimum_diagonal);
  }
}

}  // namespace

CooperativeInertialEkf::CooperativeInertialEkf(
    CooperativeInertialConfig cooperative_config,
    InertialConfig inertial_config,
    std::vector<InertialNodeInitialization> initializations)
    // cooperative_config按值传入后移动进成员；covariance_先构造成合法0×0矩阵，校验后再分配。
    : config_(std::move(cooperative_config)), covariance_(0U, 0U) {
  if (initializations.empty() || !positive_finite(config_.nis_gate) ||
      !positive_finite(config_.min_covariance_diagonal) ||
      config_.max_inertial_state_dimension < kInertialErrorStateSize ||
      initializations.size() >
          std::numeric_limits<std::size_t>::max() /
              kInertialErrorStateSize) {
    throw std::invalid_argument("invalid cooperative inertial configuration");
  }
  // 初始化阶段固定节点到15维块的映射，并在分配矩阵前检查状态资源上限。
  // `dimension`是节点数乘15得到的联合误差状态及协方差行列维数。
  const std::size_t dimension =
      initializations.size() * kInertialErrorStateSize;
  if (dimension > config_.max_inertial_state_dimension) {
    throw std::invalid_argument("inertial state dimension exceeds limit");
  }

  filters_.reserve(initializations.size());
  node_ids_.reserve(initializations.size());
  // `reference_found`为true表示初始化数组已包含配置的主参考平台。
  bool reference_found = false;
  // `index`遍历初始化顺序，同时固定节点的15维块序号。
  for (std::size_t index = 0U; index < initializations.size(); ++index) {
    // `node`借用当前节点初值，生命周期限于本次循环。
    const auto& node = initializations[index];
    if (!finite_nonnegative(node.position_std_m.x) ||
        !finite_nonnegative(node.position_std_m.y) ||
        !finite_nonnegative(node.position_std_m.z) ||
        !finite_nonnegative(node.velocity_std_mps.x) ||
        !finite_nonnegative(node.velocity_std_mps.y) ||
        !finite_nonnegative(node.velocity_std_mps.z) ||
        !finite_nonnegative(node.attitude_std_rad.x) ||
        !finite_nonnegative(node.attitude_std_rad.y) ||
        !finite_nonnegative(node.attitude_std_rad.z) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.x) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.y) ||
        !finite_nonnegative(node.gyro_bias_std_rad_s.z) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.x) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.y) ||
        !finite_nonnegative(node.accel_bias_std_m_s2.z)) {
      throw std::invalid_argument("invalid inertial initial standard deviation");
    }
    // emplace返回pair；second=false说明键已存在，可在一次查找中完成插入和重复判断。
    if (!node_lookup_.emplace(node.node_id, index).second) {
      throw std::invalid_argument("duplicate inertial node id");
    }
    reference_found = reference_found ||
                      node.node_id == config_.reference_node_id;
    node_ids_.push_back(node.node_id);
    // emplace_back直接在vector尾部调用InertialEskf15构造函数，避免先造临时对象再复制。
    filters_.emplace_back(node, inertial_config);
  }
  if (!reference_found) {
    throw std::invalid_argument("reference inertial node is missing");
  }

  // 初始节点互不相关，只填充各自15×15对角块；后续协同量测会建立交叉相关。
  covariance_ = DenseMatrix(dimension, dimension);
  // `index`再次遍历节点块，为每个15×15对角块写入五组三轴初始方差。
  for (std::size_t index = 0U; index < initializations.size(); ++index) {
    // `offset`是当前节点15维块在联合协方差中的起始行列下标；
    // `node`借用对应节点的五组三轴标准差。
    const std::size_t offset = index * kInertialErrorStateSize;
    const auto& node = initializations[index];
    set_initial_variance(covariance_, offset, node.position_std_m,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 3U, node.velocity_std_mps,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 6U, node.attitude_std_rad,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 9U, node.gyro_bias_std_rad_s,
                         config_.min_covariance_diagonal);
    set_initial_variance(covariance_, offset + 12U, node.accel_bias_std_m_s2,
                         config_.min_covariance_diagonal);
  }
}

ImuProcessingResult CooperativeInertialEkf::push_imu(
    const ImuPacket& packet) {
  // 阶段1：用固定节点映射定位传播器，未知节点不能在运行中扩大状态维度。
  // `found`是输入平台编号到传播器及15维块序号的查找结果。
  const auto found = node_lookup_.find(packet.node_id);
  if (found == node_lookup_.end()) {
    // `result`构造未知节点诊断，并返回15×15单位Phi以保持接口形状。
    ImuProcessingResult result{};
    result.disposition = ImuDisposition::kUnknownNode;
    result.phi = DenseMatrix::identity(kInertialErrorStateSize);
    return result;
  }

  // 在副本上完成名义状态和协方差传播；任何数值异常都不会污染在线状态。
  // `candidate_filters`是全部节点传播器的事务副本；`result`是目标节点IMU传播诊断。
  std::vector<InertialEskf15> candidate_filters = filters_;
  ImuProcessingResult result = candidate_filters[found->second].push_imu(packet);
  // unordered_map迭代器的second是节点块序号，用它索引对应单车传播器。
  if (!result.propagated) {
    if (result.disposition == ImuDisposition::kBaselineEstablished ||
        result.disposition == ImuDisposition::kIntervalRejected) {
      filters_ = std::move(candidate_filters);
      // 首帧和间隔拒绝都会更新该节点时间基准，因此即使没有传播也要提交传播器副本。
    }
    return result;
  }

  // 阶段2：在协方差副本上更新目标块及全部交叉块，保证数值失败时可回滚。
  // `candidate_covariance`是待提交的15N×15N联合协方差副本；
  // `node_offset`是被传播节点15维块起始下标；`dimension`为联合状态维数15N。
  DenseMatrix candidate_covariance = covariance_;
  const std::size_t node_offset =
      found->second * kInertialErrorStateSize;
  const std::size_t dimension = covariance_.rows();

  // P_ii=Phi*P_ii*Phi^T+Q；P_ij=Phi*P_ij；P_ji=P_ji*Phi^T。
  // `other_offset`按15维步长遍历其余节点交叉协方差块的起始下标。
  for (std::size_t other_offset = 0U; other_offset < dimension;
       other_offset += kInertialErrorStateSize) {
    if (other_offset == node_offset) {
      continue;
    }
    // `row`与`col`遍历当前15×15跨节点块的目标行列；
    // `inner`遍历Phi与原跨节点块的共享15维分量。
    for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
      for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
        // `left_value`累计P_ij新元素；`right_value`累计对称方向P_ji新元素。
        double left_value = 0.0;
        double right_value = 0.0;
        for (std::size_t inner = 0U; inner < kInertialErrorStateSize;
             ++inner) {
          left_value += result.phi(row, inner) *
                        covariance_(node_offset + inner,
                                    other_offset + col);
          right_value += covariance_(other_offset + row,
                                     node_offset + inner) *
                         result.phi(col, inner);
        }
        candidate_covariance(node_offset + row, other_offset + col) =
            left_value;
        candidate_covariance(other_offset + row, node_offset + col) =
            right_value;
      }
    }
  }

  // `row`与`col`遍历被传播节点自身15×15协方差块；
  // `left`、`right`分别遍历Phi*P*Phiᵀ的两个收缩维。
  for (std::size_t row = 0U; row < kInertialErrorStateSize; ++row) {
    for (std::size_t col = 0U; col < kInertialErrorStateSize; ++col) {
      // `value`从Qd(row,col)开始累计目标节点传播后的块内协方差元素。
      double value = result.qd(row, col);
      for (std::size_t left = 0U; left < kInertialErrorStateSize; ++left) {
        for (std::size_t right = 0U; right < kInertialErrorStateSize;
             ++right) {
          value += result.phi(row, left) *
                   covariance_(node_offset + left, node_offset + right) *
                   result.phi(col, right);
        }
      }
      candidate_covariance(node_offset + row, node_offset + col) = value;
    }
  }
  candidate_covariance = candidate_covariance.symmetrized();
  if (!valid_covariance(candidate_covariance)) {
    result.disposition = ImuDisposition::kNumericalFailure;
    result.propagated = false;
    return result;
  }

  filters_ = std::move(candidate_filters);
  // 名义状态和联合协方差都验证成功后才一起移动提交，形成事务边界。
  covariance_ = std::move(candidate_covariance);
  return result;
}

UpdateResult CooperativeInertialEkf::update_range(const RangePacket& packet,
                                                   double covariance_scale) {
  // `result`保存本次三维距离量测的创新、S、NIS、降权倍数及最终处置。
  UpdateResult result{};
  // 阶段1：结构、节点和时间检查先于几何线性化，拒绝结果不修改滤波状态。
  result.covariance_scale = covariance_scale;
  if (!packet.valid || packet.receive_timestamp_ns == 0U ||
      packet.status > 2U || !std::isfinite(packet.range_m) ||
      !std::isfinite(packet.range_std_m) || packet.range_m <= 0.0 ||
      packet.range_std_m <= 0.0 || !std::isfinite(covariance_scale) ||
      covariance_scale < 1.0) {
    result.disposition = packet.range_m <= 0.0
                             ? UpdateDisposition::NonPositiveRange
                             : UpdateDisposition::InvalidPacket;
    return result;
  }
  if (packet.from_node == packet.to_node) {
    result.disposition = UpdateDisposition::SelfRange;
    return result;
  }
  // `from`与`to`分别定位测距起点、终点的平台传播器及15维块序号。
  const auto from = node_lookup_.find(packet.from_node);
  const auto to = node_lookup_.find(packet.to_node);
  if (from == node_lookup_.end() || to == node_lookup_.end()) {
    result.disposition = UpdateDisposition::UnknownNode;
    return result;
  }
  if (has_range_timebase_ && packet.timestamp_ns < last_range_timestamp_ns_) {
    result.disposition = UpdateDisposition::OutOfOrder;
    return result;
  }

  // 阶段2：以两节点当前三维位置构造预测距离和单位方向；零基线不可线性化。
  // `difference`是从from指向to的导航ENU三维位置差向量，单位m；
  // `predicted_range`是其欧氏模长，即当前预测距离，单位m。
  const Vec3 difference = filters_[to->second].state().position_n_m -
                          filters_[from->second].state().position_n_m;
  const double predicted_range = norm(difference);
  if (!positive_finite(predicted_range)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  // `direction`是位置差归一化后的ENU视线单位向量。
  const Vec3 direction = (1.0 / predicted_range) * difference;
  // `dimension`是联合误差状态维数15N；`jacobian`是一维距离对完整状态的1×15N雅可比。
  const std::size_t dimension = covariance_.rows();
  // H只在两个节点的位置块非零，但P*H^T会把约束传播到全部相关状态块。
  std::vector<double> jacobian(dimension, 0.0);
  // `from_offset`与`to_offset`分别是两端节点15维块的起始列下标。
  const std::size_t from_offset = from->second * kInertialErrorStateSize;
  const std::size_t to_offset = to->second * kInertialErrorStateSize;
  jacobian[from_offset] = -direction.x;
  jacobian[from_offset + 1U] = -direction.y;
  jacobian[from_offset + 2U] = -direction.z;
  jacobian[to_offset] = direction.x;
  jacobian[to_offset + 1U] = direction.y;
  jacobian[to_offset + 2U] = direction.z;

  // `covariance_times_jacobian`是P*Hᵀ的15N列向量，保留跨节点相关性；
  // `projected_variance`累计H*P*Hᵀ，单位m²。
  const std::vector<double> covariance_times_jacobian =
      covariance_ * jacobian;
  double projected_variance = 0.0;
  // `index`遍历完整15N状态分量以完成标量二次型。
  for (std::size_t index = 0U; index < dimension; ++index) {
    projected_variance +=
        jacobian[index] * covariance_times_jacobian[index];
  }
  // `measurement_variance`是经质量倍数放大的距离量测方差R，单位m²；
  // `innovation_variance`是S=HPHᵀ+R，单位m²；`innovation`是实测减预测距离，单位m。
  const double measurement_variance =
      packet.range_std_m * packet.range_std_m * covariance_scale;
  const double innovation_variance =
      projected_variance + measurement_variance;
  const double innovation = packet.range_m - predicted_range;
  if (!positive_finite(measurement_variance) ||
      !positive_finite(innovation_variance) || !std::isfinite(innovation)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  result.innovation_m = innovation;
  result.innovation_variance = innovation_variance;
  // 阶段3：用NIS在状态更新前隔离异常量测，同时保留创新诊断供质量监测使用。
  result.nis = innovation * innovation / innovation_variance;
  if (!std::isfinite(result.nis)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  if (result.nis > config_.nis_gate) {
    // 统计拒绝仍消费该测距时间：否则后续更早的包可能被错误地当成新量测。
    // 这里只推进测距时间基准，不改变名义状态或联合协方差。
    has_range_timebase_ = true;
    last_range_timestamp_ns_ =
        std::max(last_range_timestamp_ns_, packet.timestamp_ns);
    result.disposition = UpdateDisposition::NisRejected;
    return result;
  }

  // 阶段4：计算联合Kalman增益和完整15N误差向量。
  // `gain`是15N×1 Kalman增益；`error`是待按节点分块注入的15N误差状态。
  std::vector<double> gain(dimension, 0.0);
  std::vector<double> error(dimension, 0.0);
  // `index`遍历联合误差状态的全部15N分量。
  for (std::size_t index = 0U; index < dimension; ++index) {
    gain[index] = covariance_times_jacobian[index] / innovation_variance;
    error[index] = gain[index] * innovation;
  }

  // Joseph形式P+=(I-KH)P(I-KH)^T+KRK^T；相比简式(I-KH)P，
  // 在15N大矩阵和弱几何条件下更能保持对称性与半正定性。
  // `identity_minus_gain_h`是15N×15N矩阵I-KH，行对应新误差、列对应旧误差。
  DenseMatrix identity_minus_gain_h = DenseMatrix::identity(dimension);
  // `row`遍历Kalman增益分量及矩阵行；`col`遍历H分量及矩阵列。
  for (std::size_t row = 0U; row < dimension; ++row) {
    for (std::size_t col = 0U; col < dimension; ++col) {
      identity_minus_gain_h(row, col) -= gain[row] * jacobian[col];
    }
  }
  // `candidate_covariance`是Joseph公式在事务副本上生成的15N×15N后验协方差。
  DenseMatrix candidate_covariance =
      identity_minus_gain_h * covariance_ * identity_minus_gain_h.transpose();
  // `row`与`col`遍历Joseph公式KRKᵀ项的全部15N×15N元素。
  for (std::size_t row = 0U; row < dimension; ++row) {
    for (std::size_t col = 0U; col < dimension; ++col) {
      candidate_covariance(row, col) +=
          gain[row] * measurement_variance * gain[col];
    }
  }
  candidate_covariance = candidate_covariance.symmetrized();
  // `index`同步遍历后验协方差主对角，施加对应状态平方量纲下的数值下限。
  for (std::size_t index = 0U; index < dimension; ++index) {
    candidate_covariance(index, index) =
        std::max(candidate_covariance(index, index),
                 config_.min_covariance_diagonal);
  }

  // 阶段5：先向所有传播器副本注入对应15维误差，全部成功后再原子提交。
  // `candidate_filters`是所有节点名义状态的事务副本。
  std::vector<InertialEskf15> candidate_filters = filters_;
  // `node`遍历传播器及联合误差向量中对应的15维节点块。
  for (std::size_t node = 0U; node < candidate_filters.size(); ++node) {
    // `node_error`按δp/δv/δθ/δbg/δba复制当前节点的15个后验误差分量。
    std::array<double, kInertialErrorStateSize> node_error{};
    std::copy_n(error.begin() +
                    // vector迭代器支持加偏移；ptrdiff_t是迭代器差值使用的有符号整数类型。
                    static_cast<std::ptrdiff_t>(node *
                                                kInertialErrorStateSize),
                kInertialErrorStateSize, node_error.begin());
    if (!candidate_filters[node].inject_error(node_error)) {
      result.disposition = UpdateDisposition::NumericalFailure;
      return result;
    }
  }
  if (!valid_covariance(candidate_covariance)) {
    result.disposition = UpdateDisposition::NumericalFailure;
    return result;
  }
  filters_ = std::move(candidate_filters);
  // 所有节点误差注入都成功后，才把候选传播器和候选协方差同时替换在线状态。
  covariance_ = std::move(candidate_covariance);
  has_range_timebase_ = true;
  last_range_timestamp_ns_ =
      std::max(last_range_timestamp_ns_, packet.timestamp_ns);
  result.disposition = UpdateDisposition::Accepted;
  return result;
}

NodeEstimate CooperativeInertialEkf::estimate(std::uint32_t node_id) const {
  // `found`定位目标节点；`reference`定位配置的主参考节点。
  const auto found = node_lookup_.find(node_id);
  const auto reference = node_lookup_.find(config_.reference_node_id);
  if (found == node_lookup_.end() || reference == node_lookup_.end()) {
    throw std::out_of_range("unknown inertial node id");
  }
  // 输出采用相对主参考状态，不把任一节点本地ENU初值误当成GCS绝对坐标。
  // `node_state`与`reference_state`分别借用目标和参考平台当前名义状态。
  const auto& node_state = filters_[found->second].state();
  const auto& reference_state = filters_[reference->second].state();
  // `relative_position`是目标减参考的ENU位置差(m)；
  // `relative_velocity`是目标减参考的ENU速度差(m/s)。
  const Vec3 relative_position =
      node_state.position_n_m - reference_state.position_n_m;
  const Vec3 relative_velocity =
      node_state.velocity_n_mps - reference_state.velocity_n_mps;
  // `node_offset`与`reference_offset`分别是目标、参考节点15维协方差块起始下标。
  const std::size_t node_offset =
      found->second * kInertialErrorStateSize;
  const std::size_t reference_offset =
      reference->second * kInertialErrorStateSize;

  // `output`汇总相对参考节点的平面状态、相对位置协方差和最新统一时间。
  NodeEstimate output{};
  output.node_id = node_id;
  output.timestamp_ns = latest_timestamp_ns();
  output.x = relative_position.x;
  output.y = relative_position.y;
  output.vx = relative_velocity.x;
  output.vy = relative_velocity.y;
  // Cov(p_i-p_r)=P_ii+P_rr-P_ir-P_ri。两个交叉项来自协同量测建立的
  // 节点相关性，忽略它们会系统性高估或低估相对位置不确定度。
  output.cov_xx = covariance_(node_offset, node_offset) +
                  covariance_(reference_offset, reference_offset) -
                  covariance_(node_offset, reference_offset) -
                  covariance_(reference_offset, node_offset);
  output.cov_xy = covariance_(node_offset, node_offset + 1U) +
                  covariance_(reference_offset, reference_offset + 1U) -
                  covariance_(node_offset, reference_offset + 1U) -
                  covariance_(reference_offset, node_offset + 1U);
  output.cov_yy = covariance_(node_offset + 1U, node_offset + 1U) +
                  covariance_(reference_offset + 1U,
                              reference_offset + 1U) -
                  covariance_(node_offset + 1U, reference_offset + 1U) -
                  covariance_(reference_offset + 1U, node_offset + 1U);
  output.valid = std::isfinite(output.x) && std::isfinite(output.y) &&
                 std::isfinite(output.vx) && std::isfinite(output.vy) &&
                 std::isfinite(output.cov_xx) &&
                 std::isfinite(output.cov_xy) &&
                 std::isfinite(output.cov_yy);
  return output;
}

std::vector<NodeEstimate> CooperativeInertialEkf::estimates() const {
  // `output`按构造时节点块顺序收集所有相对主参考估计。
  std::vector<NodeEstimate> output;
  output.reserve(node_ids_.size());
  // `node_id`依次取固定节点列表中的平台编号。
  for (const std::uint32_t node_id : node_ids_) {
    output.push_back(estimate(node_id));
  }
  return output;
}

const InertialNominalState& CooperativeInertialEkf::state(
    std::uint32_t node_id) const {
  // `found`定位目标平台传播器，返回引用的有效期随本滤波器对象。
  const auto found = node_lookup_.find(node_id);
  if (found == node_lookup_.end()) {
    throw std::out_of_range("unknown inertial node id");
  }
  return filters_[found->second].state();
}

const DenseMatrix& CooperativeInertialEkf::covariance() const noexcept {
  return covariance_;
}

std::size_t CooperativeInertialEkf::state_dimension() const noexcept {
  return covariance_.rows();
}

const std::vector<std::uint32_t>& CooperativeInertialEkf::node_ids() const
    noexcept {
  return node_ids_;
}

bool CooperativeInertialEkf::valid_covariance(
    const DenseMatrix& covariance) const {
  // `row`遍历联合协方差行并先检查对应对角线；
  // `col`遍历该行全部15N列以拒绝任意非有限跨节点元素。
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    if (!std::isfinite(covariance(row, row)) ||
        covariance(row, row) < config_.min_covariance_diagonal) {
      return false;
    }
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      if (!std::isfinite(covariance(row, col))) {
        return false;
      }
    }
  }
  return true;
}

std::uint64_t CooperativeInertialEkf::latest_timestamp_ns() const noexcept {
  // `timestamp`累计测距与各IMU传播器已提交统一传感器时间的最大值，单位ns。
  std::uint64_t timestamp =
      has_range_timebase_ ? last_range_timestamp_ns_ : 0U;
  // `filter`依次借用各节点传播器以合并其最新IMU时间。
  for (const auto& filter : filters_) {
    timestamp = std::max(timestamp, filter.timestamp_ns());
  }
  return timestamp;
}

}  // namespace zju::coop

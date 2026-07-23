// 模块实现：把稳定C ABI结构转换为C++ Engine输入，并将原子快照转换回调用方缓冲区。
// 关键原则：所有版本、结构大小、数组步长、对齐、保留字段和有限值在边界集中校验；
// C++异常不得穿越C ABI，输出缓冲区在完整转换成功前不得出现部分写入。
#include "zju_coop/c_api.h"

#include "core/engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <iterator>
#include <memory>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

// 每个句柄拥有一个独立算法会话；C ABI本身不加锁，由ROS 2适配层保证串行调用。
struct zju_coop_handle {
  // config移交给独占Engine；configured_node_count/configured_edge_count冻结v1输出数组所需元素数。
  zju_coop_handle(zju::coop::EngineConfig config,
                  std::uint32_t configured_node_count,
                  std::uint32_t configured_edge_count)
      : engine(std::make_unique<zju::coop::Engine>(std::move(config))),
        localization_count(configured_node_count),
        observation_count(configured_edge_count) {}

  std::unique_ptr<zju::coop::Engine> engine; /* 句柄独占的算法会话，随句柄销毁。 */
  std::uint32_t localization_count{}; /* 每次step输出的全部配置节点定位条数，包含参考节点。 */
  std::uint32_t observation_count{};  /* 每次step必须输出的完全图无向边质量条数。 */
  bool processing_started{}; /* true表示已有输入或step，之后禁止惯性重配置。 */
  bool inertial_configured{}; /* true表示15N状态、节点初值和IMU参数已冻结。 */
};

namespace {

// kMaximumNodes限制平台数；kMaximumEdges限制64节点完全图边数；kMaximumStateDimension限制4×63平面状态。
constexpr std::uint32_t kMaximumNodes = 64U;
constexpr std::uint32_t kMaximumEdges = 2016U;
constexpr std::uint32_t kMaximumStateDimension = 252U;
// kMaximumDuplicateCachePerLink限制单向sequence缓存；kMaximumTrackedEdges限制质量状态总边数。
constexpr std::uint32_t kMaximumDuplicateCachePerLink = 4096U;
constexpr std::uint32_t kMaximumTrackedEdges = 1'000'000U;

template <typename Structure>
// value借用调用方结构；Structure同时确定v1最小布局，并以struct_size/abi_version判定兼容性。
zju_coop_error_code_t validate_header(const Structure* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (value->struct_size < sizeof(Structure)) {
    return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  }
  if (value->abi_version != ZJU_COOP_ABI_VERSION_V1) {
    return ZJU_COOP_ABI_MISMATCH;
  }
  return ZJU_COOP_OK;
}

// value为跨C ABI传入的单字节布尔编码，仅0和1合法。
bool valid_boolean(zju_coop_bool_t value) {
  return value == ZJU_COOP_FALSE || value == ZJU_COOP_TRUE;
}

// value为进入算法前待排除NaN/Inf的浮点字段。
bool finite(double value) { return std::isfinite(value); }

template <std::size_t Size>
// values是字段布局确定的Size个浮点量；无捕获lambda的value逐项排除NaN/Inf。
bool finite_array(const double (&values)[Size]) {
  return std::all_of(std::begin(values), std::end(values),
                     [](double value) { return finite(value); });
}

// value为固定容量字符缓冲首地址；capacity为可搜索NUL的最大字节数，内存由调用方持有。
bool nul_terminated_nonempty(const char* value, std::size_t capacity) {
  return value != nullptr && value[0] != '\0' &&
         std::memchr(value, '\0', capacity) != nullptr;
}

template <typename Structure>
// base借用调用方数组，count/stride给出元素数和字节步长；Structure确定最小大小与对齐。
bool valid_array_span(const void* base, std::uint32_t count,
                      std::uint32_t stride) {
  // 在做指针运算前验证步长、对齐和地址溢出，避免跨语言数组导致越界访问。
  if (count == 0U) {
    return true;
  }
  if (base == nullptr || stride < sizeof(Structure) ||
      stride % alignof(Structure) != 0U) {
    return false;
  }
  // base_address把首地址转为整数；maximum、last_index、last_offset、last_address和last_byte
  // 共同证明最后一个Structure元素的末字节仍落在可表示地址范围内。
  const auto base_address = reinterpret_cast<std::uintptr_t>(base);
  if (base_address % alignof(Structure) != 0U) {
    return false;
  }
  const auto maximum = std::numeric_limits<std::uintptr_t>::max();
  const std::uintptr_t last_index = count - 1U;
  if (last_index != 0U && stride > maximum / last_index) {
    return false;
  }
  const std::uintptr_t last_offset = last_index * stride;
  if (base_address > maximum - last_offset) {
    return false;
  }
  const std::uintptr_t last_address = base_address + last_offset;
  constexpr std::uintptr_t last_byte = sizeof(Structure) - 1U;
  return last_address <= maximum - last_byte;
}

template <typename Structure>
// base为已验证数组首地址，index为目标0起始元素号，stride为相邻元素字节步长。
const Structure* array_element(const void* base, std::uint32_t index,
                               std::uint32_t stride) {
  // bytes保留调用方只读所有权；offset为目标元素相对首地址的字节偏移。
  const auto* bytes = static_cast<const unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<const Structure*>(bytes + offset);
}

template <typename Structure>
// base为已验证可写数组首地址，index为目标0起始元素号，stride为相邻元素字节步长。
Structure* array_element(void* base, std::uint32_t index,
                         std::uint32_t stride) {
  // bytes借用调用方可写缓冲；offset为目标元素相对首地址的字节偏移。
  auto* bytes = static_cast<unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<Structure*>(bytes + offset);
}

// node为待进入二维状态初始化的C ABI节点，检查所有物理量均有限。
bool finite_node(const zju_coop_node_initialization_t& node) {
  return finite(node.x) && finite(node.y) && finite(node.vx) &&
         finite(node.vy) && finite(node.position_std_m) &&
         finite(node.velocity_std_mps);
}

// config为待转换的基础配置，仅检查其中全部浮点阈值和噪声参数有限。
bool finite_config(const zju_coop_config_t& config) {
  return finite(config.process_accel_std_mps2) && finite(config.nis_gate) &&
         finite(config.max_prediction_step_s) &&
         finite(config.min_covariance_diagonal) &&
         finite(config.nominal_rate_hz) &&
         finite(config.nlos_ratio_threshold) &&
         finite(config.valid_ratio_threshold) &&
         finite(config.rate_ratio_threshold) &&
         finite(config.nlos_probability_threshold) &&
         finite(config.nlos_covariance_scale) &&
         finite(config.rigidity_tolerance);
}

// input为已通过v1头校验的调用方配置；output为成功时才完整覆盖的内部配置目标。
zju_coop_error_code_t convert_config(const zju_coop_config_t& input,
                                     zju::coop::EngineConfig& output) {
  // 阶段1：先检查资源上限与数组跨度，再逐节点深拷贝，库不保存调用方内存指针。
  // 只有完整转换成功才覆盖output，避免异常调用方留下半构造EngineConfig。
  if (!finite_config(input) || input.node_count == 0U ||
      input.nodes == nullptr || input.node_count > kMaximumNodes ||
      input.max_nodes == 0U || input.max_nodes > kMaximumNodes ||
      input.node_count > input.max_nodes || input.max_edges == 0U ||
      input.max_edges > kMaximumEdges ||
      input.max_state_dimension == 0U ||
      input.max_state_dimension > kMaximumStateDimension ||
      input.duplicate_cache_per_link == 0U ||
      input.duplicate_cache_per_link > kMaximumDuplicateCachePerLink ||
      input.max_tracked_edges == 0U ||
      input.max_tracked_edges > kMaximumTrackedEdges ||
      !valid_array_span<zju_coop_node_initialization_t>(
          input.nodes, input.node_count, input.node_stride)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  // node_count提升到64位避免组合计算溢出；edge_count为完全图边数，state_dimension为4×非参考节点数。
  const std::uint64_t node_count = input.node_count;
  const std::uint64_t edge_count = node_count * (node_count - 1U) / 2U;
  const std::uint64_t state_dimension = (node_count - 1U) * 4U;
  if (edge_count > input.max_edges || edge_count > input.max_tracked_edges ||
      state_dimension > input.max_state_dimension) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  // nodes为深拷贝后的内部二维初值，局部构造完成后整体移交output。
  std::vector<zju::coop::NodeInitialization> nodes;
  nodes.reserve(input.node_count);
  // index遍历调用方nodes数组中每个配置节点的0起始位置。
  for (std::uint32_t index = 0U; index < input.node_count; ++index) {
    // source借用当前跨步元素，仅在本次迭代和input数组生命周期内有效。
    const auto* source = array_element<zju_coop_node_initialization_t>(
        input.nodes, index, input.node_stride);
    // header_status保留当前节点v1大小/版本校验的稳定ABI错误码。
    const auto header_status = validate_header(source);
    if (header_status != ZJU_COOP_OK) {
      return header_status;
    }
    if (source->struct_size > input.node_stride || !finite_node(*source)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    nodes.push_back({source->node_id, source->x, source->y, source->vx,
                     source->vy, source->position_std_m,
                     source->velocity_std_mps});
  }

  output.filter.reference_node_id = input.reference_node_id;
  output.filter.process_accel_std_mps2 = input.process_accel_std_mps2;
  output.filter.nis_gate = input.nis_gate;
  output.filter.max_prediction_step_s = input.max_prediction_step_s;
  output.filter.min_covariance_diagonal = input.min_covariance_diagonal;
  output.nodes = std::move(nodes);
  output.degradation.window_ns = input.degradation_window_ns;
  output.degradation.nominal_rate_hz = input.nominal_rate_hz;
  output.degradation.nlos_ratio_threshold = input.nlos_ratio_threshold;
  output.degradation.valid_ratio_threshold = input.valid_ratio_threshold;
  output.degradation.rate_ratio_threshold = input.rate_ratio_threshold;
  output.degradation.nlos_probability_threshold =
      input.nlos_probability_threshold;
  output.degradation.nlos_covariance_scale = input.nlos_covariance_scale;
  output.degradation.suspend_duration_ns = input.suspend_duration_ns;
  output.degradation.reject_duration_ns = input.reject_duration_ns;
  output.degradation.recovery_duration_ns = input.recovery_duration_ns;
  output.degradation.max_tracked_edges = input.max_tracked_edges;
  output.edge_timeout_ns = input.edge_timeout_ns;
  output.max_future_skew_ns = input.max_future_skew_ns;
  output.max_receive_delay_ns = input.max_receive_delay_ns;
  output.duplicate_cache_per_link = input.duplicate_cache_per_link;
  output.max_nodes = input.max_nodes;
  output.max_edges = input.max_edges;
  output.max_state_dimension = input.max_state_dimension;
  output.rigidity_tolerance = input.rigidity_tolerance;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t convert_inertial_config(
    // input为已通过v1头校验的调用方惯性配置；inertial/nodes为成功时交给Engine的内部输出。
    const zju_coop_inertial_config_t& input,
    zju::coop::InertialConfig& inertial,
    std::vector<zju::coop::InertialNodeInitialization>& nodes) {
  // 惯性配置同时冻结15N状态维度、节点顺序和坐标约定；全部节点初值、
  // 四元数、标准差、帧名及连续时间噪声参数必须在首个输入前一次性通过。
  // 节点数组执行深拷贝，frame_id必须在固定容量内以NUL结束。
  if (input.node_count == 0U || input.nodes == nullptr ||
      input.max_inertial_state_dimension == 0U ||
      !valid_array_span<zju_coop_inertial_node_initialization_t>(
          input.nodes, input.node_count, input.node_stride) ||
      !finite(input.gravity_mps2) || !finite(input.min_imu_dt_s) ||
      !finite(input.max_imu_dt_s) ||
      !finite(input.max_propagation_substep_s) ||
      !finite(input.gyro_noise_density_rad_s_sqrt_hz) ||
      !finite(input.accel_noise_density_m_s2_sqrt_hz) ||
      !finite(input.gyro_bias_random_walk_rad_s2_sqrt_hz) ||
      !finite(input.accel_bias_random_walk_m_s3_sqrt_hz) ||
      !finite(input.min_covariance_diagonal) ||
      !finite(input.quaternion_norm_tolerance) ||
      !finite(input.covariance_symmetry_tolerance) ||
      !valid_boolean(input.use_message_covariance) ||
      !valid_boolean(input.use_orientation_for_initialization) ||
      !nul_terminated_nonempty(input.expected_frame_id,
                               sizeof(input.expected_frame_id))) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  nodes.clear();
  nodes.reserve(input.node_count);
  // index遍历调用方惯性初值数组中每个平台的0起始位置。
  for (std::uint32_t index = 0U; index < input.node_count; ++index) {
    // source借用当前跨步初值元素，不保存到返回对象。
    const auto* source =
        array_element<zju_coop_inertial_node_initialization_t>(
            input.nodes, index, input.node_stride);
    // header_status为当前惯性节点结构的v1头校验结论。
    const auto header_status = validate_header(source);
    if (header_status != ZJU_COOP_OK) {
      return header_status;
    }
    if (source->struct_size > input.node_stride || source->reserved0 != 0U ||
        !finite_array(source->position_n_m) ||
        !finite_array(source->velocity_n_mps) ||
        !finite_array(source->orientation_xyzw) ||
        !finite_array(source->gyro_bias_rad_s) ||
        !finite_array(source->accel_bias_m_s2) ||
        !finite_array(source->position_std_m) ||
        !finite_array(source->velocity_std_mps) ||
        !finite_array(source->attitude_std_rad) ||
        !finite_array(source->gyro_bias_std_rad_s) ||
        !finite_array(source->accel_bias_std_m_s2)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    // node为当前平台的15维内部初值副本，四元数由xyzw重排为内部wxyz。
    zju::coop::InertialNodeInitialization node{};
    node.node_id = source->node_id;
    node.position_n_m = {source->position_n_m[0], source->position_n_m[1],
                         source->position_n_m[2]};
    node.velocity_n_mps = {source->velocity_n_mps[0],
                           source->velocity_n_mps[1],
                           source->velocity_n_mps[2]};
    node.orientation_b_to_n = {source->orientation_xyzw[3],
                               source->orientation_xyzw[0],
                               source->orientation_xyzw[1],
                               source->orientation_xyzw[2]};
    node.gyro_bias_rad_s = {source->gyro_bias_rad_s[0],
                            source->gyro_bias_rad_s[1],
                            source->gyro_bias_rad_s[2]};
    node.accel_bias_m_s2 = {source->accel_bias_m_s2[0],
                            source->accel_bias_m_s2[1],
                            source->accel_bias_m_s2[2]};
    node.position_std_m = {source->position_std_m[0],
                           source->position_std_m[1],
                           source->position_std_m[2]};
    node.velocity_std_mps = {source->velocity_std_mps[0],
                             source->velocity_std_mps[1],
                             source->velocity_std_mps[2]};
    node.attitude_std_rad = {source->attitude_std_rad[0],
                             source->attitude_std_rad[1],
                             source->attitude_std_rad[2]};
    node.gyro_bias_std_rad_s = {source->gyro_bias_std_rad_s[0],
                                source->gyro_bias_std_rad_s[1],
                                source->gyro_bias_std_rad_s[2]};
    node.accel_bias_std_m_s2 = {source->accel_bias_std_m_s2[0],
                                source->accel_bias_std_m_s2[1],
                                source->accel_bias_std_m_s2[2]};
    nodes.push_back(node);
  }

  inertial.gravity_mps2 = input.gravity_mps2;
  inertial.min_imu_dt_s = input.min_imu_dt_s;
  inertial.max_imu_dt_s = input.max_imu_dt_s;
  inertial.max_propagation_substep_s =
      input.max_propagation_substep_s;
  inertial.gyro_noise_density_rad_s_sqrt_hz =
      input.gyro_noise_density_rad_s_sqrt_hz;
  inertial.accel_noise_density_m_s2_sqrt_hz =
      input.accel_noise_density_m_s2_sqrt_hz;
  inertial.gyro_bias_random_walk_rad_s2_sqrt_hz =
      input.gyro_bias_random_walk_rad_s2_sqrt_hz;
  inertial.accel_bias_random_walk_m_s3_sqrt_hz =
      input.accel_bias_random_walk_m_s3_sqrt_hz;
  inertial.min_covariance_diagonal = input.min_covariance_diagonal;
  inertial.quaternion_norm_tolerance = input.quaternion_norm_tolerance;
  inertial.covariance_symmetry_tolerance =
      input.covariance_symmetry_tolerance;
  inertial.use_message_covariance =
      input.use_message_covariance == ZJU_COOP_TRUE;
  inertial.use_orientation_for_initialization =
      input.use_orientation_for_initialization == ZJU_COOP_TRUE;
  inertial.expected_frame_id = input.expected_frame_id;
  return ZJU_COOP_OK;
}

// value为Engine整包处理结论，返回其固定数值的C ABI等价值；未知值保守映射为无效包。
zju_coop_processing_disposition_t processing_disposition(
    zju::coop::ProcessingDisposition value) {
  switch (value) {
    case zju::coop::ProcessingDisposition::Processed:
      return ZJU_COOP_PROCESSING_PROCESSED;
    case zju::coop::ProcessingDisposition::InvalidPacket:
      return ZJU_COOP_PROCESSING_INVALID_PACKET;
    case zju::coop::ProcessingDisposition::OutOfOrder:
      return ZJU_COOP_PROCESSING_OUT_OF_ORDER;
    case zju::coop::ProcessingDisposition::TimeRejected:
      return ZJU_COOP_PROCESSING_TIME_REJECTED;
    case zju::coop::ProcessingDisposition::Duplicate:
      return ZJU_COOP_PROCESSING_DUPLICATE;
    case zju::coop::ProcessingDisposition::Held:
      return ZJU_COOP_PROCESSING_HELD;
    case zju::coop::ProcessingDisposition::Rejected:
      return ZJU_COOP_PROCESSING_REJECTED;
  }
  return ZJU_COOP_PROCESSING_INVALID_PACKET;
}

// value为滤波量测更新结论，返回稳定C ABI错误枚举；未知值按数值失败处理。
zju_coop_update_disposition_t update_disposition(
    zju::coop::UpdateDisposition value) {
  switch (value) {
    case zju::coop::UpdateDisposition::Accepted:
      return ZJU_COOP_UPDATE_ACCEPTED;
    case zju::coop::UpdateDisposition::InvalidPacket:
      return ZJU_COOP_UPDATE_INVALID_PACKET;
    case zju::coop::UpdateDisposition::UnknownNode:
      return ZJU_COOP_UPDATE_UNKNOWN_NODE;
    case zju::coop::UpdateDisposition::SelfRange:
      return ZJU_COOP_UPDATE_SELF_RANGE;
    case zju::coop::UpdateDisposition::NonPositiveRange:
      return ZJU_COOP_UPDATE_NON_POSITIVE_RANGE;
    case zju::coop::UpdateDisposition::OutOfOrder:
      return ZJU_COOP_UPDATE_OUT_OF_ORDER;
    case zju::coop::UpdateDisposition::NisRejected:
      return ZJU_COOP_UPDATE_NIS_REJECTED;
    case zju::coop::UpdateDisposition::NumericalFailure:
      return ZJU_COOP_UPDATE_NUMERICAL_FAILURE;
  }
  return ZJU_COOP_UPDATE_NUMERICAL_FAILURE;
}

// value为内部IMU处理结论，返回固定C ABI数值；未知值按无效包处理。
zju_coop_imu_disposition_t imu_disposition(
    zju::coop::ImuDisposition value) {
  switch (value) {
    case zju::coop::ImuDisposition::kBaselineEstablished:
      return ZJU_COOP_IMU_BASELINE_ESTABLISHED;
    case zju::coop::ImuDisposition::kPropagated:
      return ZJU_COOP_IMU_PROPAGATED;
    case zju::coop::ImuDisposition::kInvalidPacket:
      return ZJU_COOP_IMU_INVALID_PACKET;
    case zju::coop::ImuDisposition::kUnknownNode:
      return ZJU_COOP_IMU_UNKNOWN_NODE;
    case zju::coop::ImuDisposition::kDuplicate:
      return ZJU_COOP_IMU_DUPLICATE;
    case zju::coop::ImuDisposition::kOutOfOrder:
      return ZJU_COOP_IMU_OUT_OF_ORDER;
    case zju::coop::ImuDisposition::kIntervalRejected:
      return ZJU_COOP_IMU_INTERVAL_REJECTED;
    case zju::coop::ImuDisposition::kFrameMismatch:
      return ZJU_COOP_IMU_FRAME_MISMATCH;
    case zju::coop::ImuDisposition::kNumericalFailure:
      return ZJU_COOP_IMU_NUMERICAL_FAILURE;
  }
  return ZJU_COOP_IMU_INVALID_PACKET;
}

// value为质量状态机动作，转换为稳定C ABI动作；未知值保守拒绝融合。
zju_coop_fusion_action_t fusion_action(zju::coop::FusionAction value) {
  switch (value) {
    case zju::coop::FusionAction::kUseNormal:
      return ZJU_COOP_FUSION_USE_NORMAL;
    case zju::coop::FusionAction::kUseDownweighted:
      return ZJU_COOP_FUSION_USE_DOWNWEIGHTED;
    case zju::coop::FusionAction::kHold:
      return ZJU_COOP_FUSION_HOLD;
    case zju::coop::FusionAction::kReject:
      return ZJU_COOP_FUSION_REJECT;
    case zju::coop::FusionAction::kTrialRecovery:
      return ZJU_COOP_FUSION_TRIAL_RECOVERY;
  }
  return ZJU_COOP_FUSION_REJECT;
}

// value为内部单边长期质量状态，转换为稳定C ABI状态；未知值保留为UNKNOWN。
zju_coop_observation_state_t observation_state(
    zju::coop::ObservationState value) {
  switch (value) {
    case zju::coop::ObservationState::kUnknown:
      return ZJU_COOP_OBSERVATION_UNKNOWN;
    case zju::coop::ObservationState::kNormal:
      return ZJU_COOP_OBSERVATION_NORMAL;
    case zju::coop::ObservationState::kDegraded:
      return ZJU_COOP_OBSERVATION_DEGRADED;
    case zju::coop::ObservationState::kSuspended:
      return ZJU_COOP_OBSERVATION_SUSPENDED;
    case zju::coop::ObservationState::kRejected:
      return ZJU_COOP_OBSERVATION_REJECTED;
    case zju::coop::ObservationState::kRecovering:
      return ZJU_COOP_OBSERVATION_RECOVERING;
  }
  return ZJU_COOP_OBSERVATION_UNKNOWN;
}

// value为内部综合定位状态，转换为稳定C ABI状态；未知值回退到未初始化。
zju_coop_localization_state_t localization_state(
    zju::coop::LocalizationState value) {
  switch (value) {
    case zju::coop::LocalizationState::kUninitialized:
      return ZJU_COOP_LOCALIZATION_UNINITIALIZED;
    case zju::coop::LocalizationState::kNormal:
      return ZJU_COOP_LOCALIZATION_NORMAL;
    case zju::coop::LocalizationState::kDegraded:
      return ZJU_COOP_LOCALIZATION_DEGRADED;
    case zju::coop::LocalizationState::kUnobservable:
      return ZJU_COOP_LOCALIZATION_UNOBSERVABLE;
    case zju::coop::LocalizationState::kStale:
      return ZJU_COOP_LOCALIZATION_STALE;
  }
  return ZJU_COOP_LOCALIZATION_UNINITIALIZED;
}

// source为Engine快照中的单节点内部输出，返回不借用source的v1 C结构副本。
zju_coop_localization_t localization_output(
    const zju::coop::LocalizationSnapshot& source) {
  // output为完整初始化后按值返回的C ABI定位元素。
  zju_coop_localization_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;
  output.timestamp_ns = source.timestamp_ns;
  output.node_id = source.node_id;
  output.reference_node_id = source.reference_node_id;
  output.x = source.x;
  output.y = source.y;
  output.vx = source.vx;
  output.vy = source.vy;
  output.cov_xx = source.cov_xx;
  output.cov_xy = source.cov_xy;
  output.cov_yy = source.cov_yy;
  output.valid = source.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.yaw_valid = source.yaw_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.z_valid = source.z_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.state = localization_state(source.state);
  return output;
}

// source为单条规范化无向边的内部质量快照，返回计数安全收窄后的v1 C结构。
zju_coop_observation_t observation_output(
    const zju::coop::ObservationQuality& source) {
  // output为完整初始化后按值返回的C ABI边质量元素。
  zju_coop_observation_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;
  output.from_node = source.edge.first;
  output.to_node = source.edge.second;
  output.window_start_ns = source.window_start_ns;
  output.window_end_ns = source.window_end_ns;
  output.expected_count = static_cast<std::uint32_t>(source.expected_count);
  output.received_count = static_cast<std::uint32_t>(source.received_count);
  output.valid_count = static_cast<std::uint32_t>(source.valid_count);
  output.nlos_count = static_cast<std::uint32_t>(source.nlos_count);
  output.residual_rejected_count =
      static_cast<std::uint32_t>(source.residual_rejected_count);
  output.dropped_count = static_cast<std::uint32_t>(source.dropped_count);
  output.nlos_ratio = source.nlos_ratio;
  output.valid_ratio = source.valid_ratio;
  output.actual_rate_hz = source.actual_rate_hz;
  output.state = observation_state(source.state);
  output.fusion_action = fusion_action(source.action);
  output.reason_mask = static_cast<std::uint32_t>(source.reason_mask);
  output.input_overflow =
      source.input_overflow ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.covariance_scale = source.covariance_scale;
  return output;
}

// source为Engine的当前拓扑快照，返回不借用source的v1 C网络结构。
zju_coop_network_t network_output(const zju::coop::NetworkSnapshot& source) {
  // output为完成全部计数收窄与状态转换后按值返回的网络元素。
  zju_coop_network_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;
  output.timestamp_ns = source.timestamp_ns;
  output.node_count = static_cast<std::uint32_t>(source.node_count);
  output.reachable_node_count =
      static_cast<std::uint32_t>(source.reachable_node_count);
  output.active_edge_count =
      static_cast<std::uint32_t>(source.active_edge_count);
  output.connected = source.connected ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.observable = source.observable ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.reason_mask = static_cast<std::uint32_t>(source.reason_mask);
  output.state = localization_state(source.state);
  return output;
}

// snapshot为待导出快照；检查所有size_t计数均可无损放入v1的uint32_t字段。
bool counts_fit_v1(const zju::coop::EngineSnapshot& snapshot) {
  // maximum是v1所有公开计数字段允许的最大值。
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  if (snapshot.localizations.size() > maximum ||
      snapshot.observations.size() > maximum ||
      snapshot.network.node_count > maximum ||
      snapshot.network.reachable_node_count > maximum ||
      snapshot.network.active_edge_count > maximum) {
    return false;
  }
  // quality依次引用快照中的每条边质量记录，核查其六个累计计数。
  for (const auto& quality : snapshot.observations) {
    if (quality.expected_count > maximum || quality.received_count > maximum ||
        quality.valid_count > maximum || quality.nlos_count > maximum ||
        quality.residual_rejected_count > maximum ||
        quality.dropped_count > maximum) {
      return false;
    }
  }
  return true;
}

zju_coop_error_code_t exception_code() {
  // ABI只暴露稳定错误码：内存耗尽和参数错误单独报告，其余C++异常折叠为
  // 内部错误，防止异常对象、RTTI或编译器运行库穿越MSVC/GCC及语言边界。
  try {
    throw;
  } catch (const std::bad_alloc&) {
    return ZJU_COOP_OUT_OF_MEMORY;
  } catch (const std::invalid_argument&) {
    return ZJU_COOP_INVALID_ARGUMENT;
  } catch (...) {
    return ZJU_COOP_INTERNAL_ERROR;
  }
}

}  // namespace

extern "C" {

uint32_t ZJU_COOP_CALL zju_coop_abi_version(void) {
  return ZJU_COOP_ABI_VERSION_V1;
}

const char* ZJU_COOP_CALL zju_coop_version_string(void) { return "0.1.0"; }

const char* ZJU_COOP_CALL zju_coop_error_string(
    zju_coop_error_code_t code) {
  switch (code) {
    case ZJU_COOP_OK:
      return "success";
    case ZJU_COOP_INVALID_ARGUMENT:
      return "invalid argument";
    case ZJU_COOP_ABI_MISMATCH:
      return "ABI version mismatch";
    case ZJU_COOP_STRUCT_SIZE_MISMATCH:
      return "structure size mismatch";
    case ZJU_COOP_BUFFER_TOO_SMALL:
      return "output buffer too small";
    case ZJU_COOP_OUT_OF_MEMORY:
      return "out of memory";
    case ZJU_COOP_INTERNAL_ERROR:
      return "internal error";
    default:
      return "unknown error";
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_node_initialization_init(
    zju_coop_node_initialization_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->position_std_m = 0.5;
  value->velocity_std_mps = 0.5;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_config_init(
    zju_coop_config_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->reference_node_id = 1U;
  value->node_stride = sizeof(zju_coop_node_initialization_t);
  value->process_accel_std_mps2 = 0.5;
  value->nis_gate = 9.0;
  value->max_prediction_step_s = 0.05;
  value->min_covariance_diagonal = 1.0e-9;
  value->degradation_window_ns = 2'000'000'000ULL;
  value->nominal_rate_hz = 20.0;
  value->nlos_ratio_threshold = 0.30;
  value->valid_ratio_threshold = 0.80;
  value->rate_ratio_threshold = 0.80;
  value->nlos_probability_threshold = 0.50;
  value->nlos_covariance_scale = 4.0;
  value->suspend_duration_ns = 1'000'000'000ULL;
  value->reject_duration_ns = 3'000'000'000ULL;
  value->recovery_duration_ns = 1'000'000'000ULL;
  value->max_tracked_edges = 2016U;
  value->duplicate_cache_per_link = 128U;
  value->edge_timeout_ns = 500'000'000ULL;
  value->max_future_skew_ns = 100'000'000ULL;
  value->max_receive_delay_ns = 500'000'000ULL;
  value->max_nodes = 64U;
  value->max_edges = 2016U;
  value->max_state_dimension = 252U;
  value->rigidity_tolerance = 1.0e-9;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_range_packet_init(
    zju_coop_range_packet_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->range_std_m = 0.1;
  value->status = ZJU_COOP_RANGE_STATUS_OK;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_range_processing_result_init(
    zju_coop_range_processing_result_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->disposition = ZJU_COOP_PROCESSING_INVALID_PACKET;
  value->fusion_action = ZJU_COOP_FUSION_USE_NORMAL;
  value->update_disposition = ZJU_COOP_UPDATE_INVALID_PACKET;
  value->covariance_scale = 1.0;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_localization_init(
    zju_coop_localization_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->state = ZJU_COOP_LOCALIZATION_UNINITIALIZED;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_network_init(
    zju_coop_network_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->state = ZJU_COOP_LOCALIZATION_UNINITIALIZED;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_observation_init(
    zju_coop_observation_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->state = ZJU_COOP_OBSERVATION_UNKNOWN;
  value->fusion_action = ZJU_COOP_FUSION_USE_NORMAL;
  value->covariance_scale = 1.0;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_inertial_node_initialization_init(
    zju_coop_inertial_node_initialization_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->orientation_xyzw[3] = 1.0;
  // axis遍历ENU/FLU三个轴，为五个15维误差状态块写入默认1σ。
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    value->position_std_m[axis] = 0.5;
    value->velocity_std_mps[axis] = 0.5;
    value->attitude_std_rad[axis] = 0.1;
    value->gyro_bias_std_rad_s[axis] = 0.01;
    value->accel_bias_std_m_s2[axis] = 0.1;
  }
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_inertial_config_init(
    zju_coop_inertial_config_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->node_stride = sizeof(zju_coop_inertial_node_initialization_t);
  value->max_inertial_state_dimension = 960U;
  value->gravity_mps2 = 9.80665;
  value->min_imu_dt_s = 1.0e-6;
  value->max_imu_dt_s = 0.1;
  value->max_propagation_substep_s = 0.01;
  value->gyro_noise_density_rad_s_sqrt_hz = 1.0e-4;
  value->accel_noise_density_m_s2_sqrt_hz = 1.0e-3;
  value->gyro_bias_random_walk_rad_s2_sqrt_hz = 1.0e-6;
  value->accel_bias_random_walk_m_s3_sqrt_hz = 1.0e-5;
  value->min_covariance_diagonal = 1.0e-12;
  value->quaternion_norm_tolerance = 1.0e-3;
  value->covariance_symmetry_tolerance = 1.0e-9;
  std::memcpy(value->expected_frame_id, "imu_link", 9U);
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_imu_packet_init(
    zju_coop_imu_packet_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->orientation_xyzw[3] = 1.0;
  value->status = ZJU_COOP_RANGE_STATUS_OK;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_imu_processing_result_init(
    zju_coop_imu_processing_result_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->disposition = ZJU_COOP_IMU_INVALID_PACKET;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_create(
    const zju_coop_config_t* config, zju_coop_handle_t** out_handle) {
  // 创建失败时先把输出置空，调用方可无歧义判断句柄所有权是否建立。
  if (out_handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *out_handle = nullptr;
  // header_status为输入基础配置的大小/ABI校验结果，转换前直接透传。
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  try {
    // converted为即将移交Engine的内部配置；conversion_status保留字段/资源约束转换结论。
    zju::coop::EngineConfig converted{};
    const auto conversion_status = convert_config(*config, converted);
    if (conversion_status != ZJU_COOP_OK) {
      return conversion_status;
    }
    // node_count提升到64位；edge_count为固定输出的完全图无向边数量。
    const std::uint64_t node_count = config->node_count;
    const std::uint64_t edge_count =
        node_count * (node_count > 0U ? node_count - 1U : 0U) / 2U;
    // created在句柄交给调用方前临时独占对象，异常时自动销毁，release后所有权转给out_handle。
    std::unique_ptr<zju_coop_handle_t> created(new zju_coop_handle_t(
        std::move(converted), config->node_count,
        static_cast<std::uint32_t>(edge_count)));
    *out_handle = created.release();
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_destroy(
    zju_coop_handle_t* handle) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    delete handle;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_configure_inertial(
    zju_coop_handle_t* handle, const zju_coop_inertial_config_t* config) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // header_status为惯性配置公开头校验结论，失败时不改变句柄。
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  // 15N状态维度只能在首个输入前设置一次，运行中重配会破坏已有协方差。
  if (handle->processing_started || handle->inertial_configured) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    // converted保存传播参数，nodes保存全部平台深拷贝初值，二者仅在完整成功后移交Engine。
    zju::coop::InertialConfig converted{};
    std::vector<zju::coop::InertialNodeInitialization> nodes;
    // conversion_status保留惯性字段、数组跨度与物理值的转换结论。
    const auto conversion_status =
        convert_inertial_config(*config, converted, nodes);
    if (conversion_status != ZJU_COOP_OK) {
      return conversion_status;
    }
    handle->engine->configure_inertial(
        std::move(converted), std::move(nodes),
        config->max_inertial_state_dimension);
    handle->inertial_configured = true;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_imu(
    zju_coop_handle_t* handle, const zju_coop_imu_packet_t* packet,
    zju_coop_imu_processing_result_t* result) {
  // 阶段1：验证调用方结构和保留字段，再转换到不含ROS类型的内部ImuPacket。
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // packet_status/result_status分别验证只读输入与调用方输出结构的v1头部。
  const auto packet_status = validate_header(packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;
  }
  const auto result_status = validate_header(result);
  if (result_status != ZJU_COOP_OK) {
    return result_status;
  }
  if (!handle->inertial_configured || packet->reserved0 != 0U ||
      !valid_boolean(packet->orientation_valid) ||
      !valid_boolean(packet->valid) ||
      packet->status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !finite_array(packet->orientation_xyzw) ||
      !finite_array(packet->orientation_covariance) ||
      !finite_array(packet->angular_velocity_rad_s) ||
      !finite_array(packet->angular_velocity_covariance) ||
      !finite_array(packet->linear_acceleration_m_s2) ||
      !finite_array(packet->linear_acceleration_covariance) ||
      !nul_terminated_nonempty(packet->frame_id, sizeof(packet->frame_id))) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (!std::all_of(std::begin(packet->reserved1), std::end(packet->reserved1),
                   // lambda的[]明确不捕获外部对象；value为reserved1中当前保留字节，v1要求逐字节为零。
                   [](std::uint8_t value) { return value == 0U; })) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    // converted为不借用调用方缓冲的内部IMU副本，数组布局在此保持ROS行主序。
    zju::coop::ImuPacket converted{};
    converted.node_id = packet->node_id;
    converted.sequence = packet->sequence;
    converted.timestamp_ns = packet->timestamp_ns;
    converted.receive_timestamp_ns = packet->receive_timestamp_ns;
    std::copy(std::begin(packet->orientation_xyzw),
              std::end(packet->orientation_xyzw),
              converted.orientation_xyzw.begin());
    std::copy(std::begin(packet->orientation_covariance),
              std::end(packet->orientation_covariance),
              converted.orientation_covariance.begin());
    std::copy(std::begin(packet->angular_velocity_rad_s),
              std::end(packet->angular_velocity_rad_s),
              converted.angular_velocity_rad_s.begin());
    std::copy(std::begin(packet->angular_velocity_covariance),
              std::end(packet->angular_velocity_covariance),
              converted.angular_velocity_covariance.begin());
    std::copy(std::begin(packet->linear_acceleration_m_s2),
              std::end(packet->linear_acceleration_m_s2),
              converted.linear_acceleration_m_s2.begin());
    std::copy(std::begin(packet->linear_acceleration_covariance),
              std::end(packet->linear_acceleration_covariance),
              converted.linear_acceleration_covariance.begin());
    std::copy(std::begin(packet->frame_id), std::end(packet->frame_id),
              converted.frame_id.begin());
    converted.orientation_valid =
        packet->orientation_valid == ZJU_COOP_TRUE;
    converted.valid = packet->valid == ZJU_COOP_TRUE;
    converted.status = packet->status;

    // 阶段2：算法处理完成后先构造局部结果，最后一次性覆盖调用方输出结构。
    // processed是Engine内部诊断；output是待完整构造后一次性提交的C ABI结果。
    const auto processed = handle->engine->push_imu(converted);
    zju_coop_imu_processing_result_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.disposition = imu_disposition(processed.disposition);
    output.propagated =
        processed.propagated ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    output.dt_s = processed.dt_s;
    // caller_size保留调用方声明的实际结构容量，覆盖v1字段后原样恢复以兼容尾部扩展。
    const std::uint32_t caller_size = result->struct_size;
    *result = output;
    result->struct_size = caller_size;
    handle->processing_started = true;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_range(
    zju_coop_handle_t* handle, const zju_coop_range_packet_t* packet,
    zju_coop_range_processing_result_t* result) {
  // 测距入口沿用“边界校验→普通结构转换→Engine处理→诊断结果转换”的固定顺序。
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // packet_status/result_status分别验证测距输入与调用方诊断输出的v1头部。
  const auto packet_status = validate_header(packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;
  }
  const auto result_status = validate_header(result);
  if (result_status != ZJU_COOP_OK) {
    return result_status;
  }
  if (!valid_boolean(packet->nlos_flag) ||
      !valid_boolean(packet->has_nlos_probability) ||
      !valid_boolean(packet->valid) ||
      packet->status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !finite(packet->range_m) || !finite(packet->range_std_m) ||
      !finite(static_cast<double>(packet->nlos_probability)) ||
      packet->nlos_probability < 0.0F || packet->nlos_probability > 1.0F) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    // converted为不借用C缓冲的内部测距副本，保留有向节点号、sequence和两种统一时间。
    zju::coop::RangePacket converted{};
    converted.from_node = packet->from_node;
    converted.to_node = packet->to_node;
    converted.sequence = packet->sequence;
    converted.timestamp_ns = packet->timestamp_ns;
    converted.receive_timestamp_ns = packet->receive_timestamp_ns;
    converted.range_m = packet->range_m;
    converted.range_std_m = packet->range_std_m;
    converted.nlos_probability = packet->nlos_probability;
    converted.nlos_flag = packet->nlos_flag == ZJU_COOP_TRUE;
    converted.has_nlos_probability =
        packet->has_nlos_probability == ZJU_COOP_TRUE;
    converted.valid = packet->valid == ZJU_COOP_TRUE;
    converted.status = packet->status;

    // processed为Engine对该包的质量/融合诊断；output为原子写回前的C ABI临时值。
    const auto processed = handle->engine->push_range(converted);
    zju_coop_range_processing_result_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.from_node = processed.edge.first;
    output.to_node = processed.edge.second;
    output.disposition = processing_disposition(processed.disposition);
    output.fusion_action = fusion_action(processed.action);
    output.update_disposition =
        update_disposition(processed.update.disposition);
    output.filter_updated =
        processed.filter_updated ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    output.innovation_m = processed.update.innovation_m;
    output.innovation_variance = processed.update.innovation_variance;
    output.nis = processed.update.nis;
    output.covariance_scale = processed.update.covariance_scale;
    // caller_size保留调用方结构容量，避免覆盖未来版本尾部大小声明。
    const std::uint32_t caller_size = result->struct_size;
    *result = output;
    result->struct_size = caller_size;
    handle->processing_started = true;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_step(
    zju_coop_handle_t* handle, uint64_t now_ns,
    zju_coop_localization_t* localizations,
    uint32_t localization_capacity, uint32_t localization_stride,
    uint32_t* localization_count, zju_coop_observation_t* observations,
    uint32_t observation_capacity, uint32_t observation_stride,
    uint32_t* observation_count, zju_coop_network_t* network) {
  if (handle == nullptr || localization_count == nullptr ||
      observation_count == nullptr ||
      (localization_capacity != 0U && localizations == nullptr) ||
      (observation_capacity != 0U && observations == nullptr) ||
      (localization_capacity == 0U && localizations != nullptr) ||
      (observation_capacity == 0U && observations != nullptr)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    // 阶段1：支持NULL/0容量查询所需数量；查询本身不能推进算法时间。
    // required_localizations/required_observations是句柄创建时冻结的两个输出数组元素需求。
    const auto required_localizations = handle->localization_count;
    const auto required_observations = handle->observation_count;

    if (localization_capacity < required_localizations ||
        observation_capacity < required_observations) {
      *localization_count = required_localizations;
      *observation_count = required_observations;
      return ZJU_COOP_BUFFER_TOO_SMALL;
    }
    if (network == nullptr) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }

    if (!valid_array_span<zju_coop_localization_t>(
            localizations, required_localizations, localization_stride) ||
        !valid_array_span<zju_coop_observation_t>(
            observations, required_observations, observation_stride)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }

    // index遍历调用方localizations数组，写入前逐元素验证头部与stride容量。
    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      // output借用当前定位输出元素，只读检查阶段不修改调用方内存。
      const auto* output = array_element<zju_coop_localization_t>(
          localizations, index, localization_stride);
      // status为当前定位元素的v1头部校验码。
      const auto status = validate_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > localization_stride) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }
    // index遍历调用方observations数组，写入前逐元素验证头部与stride容量。
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      // output借用当前边质量输出元素，只读检查阶段不修改调用方内存。
      const auto* output = array_element<zju_coop_observation_t>(
          observations, index, observation_stride);
      // status为当前边质量元素的v1头部校验码。
      const auto status = validate_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > observation_stride) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }
    // network_status验证单个网络输出结构可安全写入v1字段。
    const auto network_status = validate_header(network);
    if (network_status != ZJU_COOP_OK) {
      return network_status;
    }

    // 阶段2：在Engine副本和临时输出数组上完成step与全部转换，保证失败可回滚。
    // localization_values/observation_values暂存完整C数组；candidate为可回滚的Engine副本。
    std::vector<zju_coop_localization_t> localization_values(
        required_localizations);
    std::vector<zju_coop_observation_t> observation_values(
        required_observations);
    auto candidate = std::make_unique<zju::coop::Engine>(*handle->engine);
    // snapshot是candidate推进到now_ns后的原子内部快照，全部转换成功前不替换正式Engine。
    const auto snapshot = candidate->step(now_ns);
    if (!counts_fit_v1(snapshot) ||
        snapshot.localizations.size() != required_localizations ||
        snapshot.observations.size() != required_observations) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    // index遍历内部定位快照并转换到同下标的临时C数组元素。
    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      localization_values[index] =
          localization_output(snapshot.localizations[index]);
    }
    // index遍历内部边质量快照并转换到同下标的临时C数组元素。
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      observation_values[index] =
          observation_output(snapshot.observations[index]);
    }
    // network_value为完成计数收窄和状态映射的临时C网络快照。
    const auto network_value = network_output(snapshot.network);

    // 阶段3：全部成功后先提交Engine，再整体写入调用方缓冲区，避免半快照。
    handle->engine.swap(candidate);
    handle->processing_started = true;
    // index遍历已验证的调用方定位数组，提交对应临时元素。
    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      // output借用当前可写元素；caller_size保留其调用方声明容量。
      auto* output = array_element<zju_coop_localization_t>(
          localizations, index, localization_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = localization_values[index];
      output->struct_size = caller_size;
    }
    // index遍历已验证的调用方边质量数组，提交对应临时元素。
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      // output借用当前可写元素；caller_size保留其调用方声明容量。
      auto* output = array_element<zju_coop_observation_t>(
          observations, index, observation_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = observation_values[index];
      output->struct_size = caller_size;
    }
    // caller_size保留调用方network结构容量，再以临时快照整体覆盖v1字段。
    const std::uint32_t caller_size = network->struct_size;
    *network = network_value;
    network->struct_size = caller_size;
    *localization_count = required_localizations;
    *observation_count = required_observations;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

}  // extern "C"

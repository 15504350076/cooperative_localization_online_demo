// 模块实现：把稳定C ABI结构转换为C++ Engine输入，并将原子快照转换回调用方缓冲区。
// 关键原则：所有版本、结构大小、数组步长、对齐、保留字段和有限值在边界集中校验；
// C++异常不得穿越C ABI，输出缓冲区在完整转换成功前不得出现部分写入。
//
// C++初学者可把本文件看成“翻译员”，按下面顺序阅读：
// 1. validate_*函数先检查C调用者传来的指针、版本号、长度、布尔值和浮点数。
// 2. to_*函数把C结构复制/转换成types.hpp中的C++结构。
// 3. 公开zju_coop_*函数调用Engine完成实际处理。
// 4. fill_*函数把Engine结果写回调用者提供的C缓冲区。
// 5. 每个公开函数末端捕获异常并返回错误码，绝不让C++异常穿过动态库边界。
//
// 本文件常见难点语法：
// - `template <typename Structure>`：同一套检查逻辑可由不同C结构类型实例化，Structure由实参推导；
// - `sizeof(T)`/`alignof(T)`：分别取得编译后类型的字节大小和对齐要求；
// - `reinterpret_cast`：只在已完成地址、长度和对齐验证后，把字节地址重新解释成结构指针；
// - `unique_ptr`：独占对象所有权并在离开作用域时自动delete；
// - `try/catch (...)`：把任何C++异常拦在ABI内部，再转换成稳定整数错误码。
#include "zju_coop/c_api.h"

#include "core/distributed_fusion2d.hpp"
#include "core/engine.hpp"
#include "core/gnss_reference.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <iterator>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// 每个句柄拥有一个独立算法会话；C ABI本身不加锁，由调用方ROS 2适配层保证串行调用。
struct zju_coop_handle {
  // config移交给独占Engine；configured_node_count/configured_edge_count冻结v1输出数组所需元素数。
  zju_coop_handle(zju::coop::EngineConfig config,
                  std::uint32_t configured_node_count,
                  std::uint32_t configured_edge_count)
      // make_unique在堆上构造Engine并返回独占智能指针；move避免再次复制较大的配置容器。
      : engine(std::make_unique<zju::coop::Engine>(std::move(config))),
        localization_count(configured_node_count),
        observation_count(configured_edge_count) {}

  /* unique_ptr表示句柄独占Engine；句柄销毁时Engine自动释放，不需要手写delete。 */
  std::unique_ptr<zju::coop::Engine> engine;
  std::uint32_t localization_count{}; /* 每次step输出的全部配置节点定位条数，包含参考节点。 */
  std::uint32_t observation_count{};  /* 每次step必须输出的完全图无向边质量条数。 */
  bool processing_started{}; /* true表示已有输入或step，之后禁止惯性重配置。 */
  bool inertial_configured{}; /* true表示15N状态、节点初值和IMU参数已冻结。 */
};

// 分布式句柄只持有低带宽二维修正器，与既有集中式Engine状态完全隔离。
struct zju_coop_distributed_handle {
  zju_coop_distributed_handle(zju::coop::DistributedFusionConfig config,
                              std::uint32_t configured_vehicle_count)
      : fusion(std::make_unique<zju::coop::DistributedFusion2D>(
            std::move(config))),
        vehicle_count(configured_vehicle_count) {}

  std::unique_ptr<zju::coop::DistributedFusion2D> fusion;
  std::uint32_t vehicle_count{};
};

// GNSS上下文与主Engine分开分配，类型结构本身保证RTK真值不能调用滤波更新。
struct zju_coop_gnss_context {
  explicit zju_coop_gnss_context(zju::coop::GnssReferenceConfig config)
      : node_count(static_cast<std::uint32_t>(config.nodes.size())),
        reference(std::make_unique<zju::coop::GnssReference>(
            std::move(config))) {}

  std::uint32_t node_count{};
  std::unique_ptr<zju::coop::GnssReference> reference;
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
  // 模板使config、packet、result等结构共用一份头部检查代码，编译器为实际类型生成对应版本。
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // 指针用`->`访问结构成员；sizeof在编译期得到当前ABI版本要求的最小字节数。
  if (value->struct_size < sizeof(Structure)) {
    return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  }
  if (value->abi_version != ZJU_COOP_ABI_VERSION_V1) {
    return ZJU_COOP_ABI_MISMATCH;
  }
  return ZJU_COOP_OK;
}

template <typename Structure>
// Pose2D采用独立v2版本域，不能复用上面的v1头部校验，否则会把合法v2结构误判为版本不匹配。
zju_coop_error_code_t validate_pose2d_header(const Structure* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (value->struct_size < sizeof(Structure)) {
    return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  }
  if (value->abi_version != ZJU_COOP_POSE2D_ABI_VERSION_V2) {
    return ZJU_COOP_ABI_MISMATCH;
  }
  return ZJU_COOP_OK;
}

// value为跨C ABI传入的单字节布尔编码，仅0和1合法。
bool valid_boolean(zju_coop_bool_t value) {
  return value == ZJU_COOP_FALSE || value == ZJU_COOP_TRUE;
}

// value为进入算法前待排除NaN/Inf的浮点字段。
bool is_finite_value(double value) { return std::isfinite(value); }

template <std::size_t Size>
// values是字段布局确定的Size个浮点量；无捕获lambda的value逐项排除NaN/Inf。
bool finite_array(const double (&values)[Size]) {
  // `const double (&)[Size]`是固定长C数组的只读引用，调用时Size可自动从数组长度推导。
  return std::all_of(std::begin(values), std::end(values),
                     // 空捕获Lambda逐项调用本文件finite包装函数。
                     [](double value) { return is_finite_value(value); });
}

// value为固定容量字符缓冲首地址；capacity为可搜索NUL的最大字节数，内存由调用方持有。
bool nul_terminated_nonempty(const char* value, std::size_t capacity) {
  return value != nullptr && value[0] != '\0' &&
         // memchr在最多capacity字节内找NUL；返回非空只说明找到终止符，不读取其后内存。
         std::memchr(value, '\0', capacity) != nullptr;
}

template <std::size_t Size>
// values是跨ABI结构中的保留字节；v1要求全部为零，避免未来字段被旧代码误解释。
bool all_zero(const std::uint8_t (&values)[Size]) {
  return std::all_of(std::begin(values), std::end(values),
                     [](std::uint8_t value) { return value == 0U; });
}

template <typename Structure>
// base借用调用方数组，count/stride给出元素数和字节步长；Structure确定最小大小与对齐。
bool valid_array_span(const void* base, std::uint32_t count,
                      std::uint32_t stride) {
  // 在做指针运算前验证步长、对齐和地址溢出，避免跨语言数组导致越界访问。
  if (count == 0U) {
    return true;
  }
  // alignof给出Structure首地址必须满足的字节倍数；stride也必须保持每个元素对齐。
  if (base == nullptr || stride < sizeof(Structure) ||
      stride % alignof(Structure) != 0U) {
    return false;
  }
  // base_address把首地址转为整数；maximum、last_index、last_offset、last_address和last_byte
  // 共同证明最后一个Structure元素的末字节仍落在可表示地址范围内。
  // uintptr_t是能够无损保存对象地址的无符号整数类型；这里只用于溢出/对齐算术，不解引用。
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
  // void*可用static_cast转成字节指针；unsigned char允许逐字节查看任意对象表示。
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  // 调用前已由valid_array_span证明大小和对齐安全，这里才把目标字节地址解释为Structure指针。
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

// datatype为ROS PointField兼容的1至8类型码；返回一个标量占用的字节数，0表示未知类型。
std::uint32_t point_field_element_size(
    zju_coop_point_field_datatype_t datatype) {
  switch (datatype) {
    case ZJU_COOP_POINT_FIELD_INT8:
    case ZJU_COOP_POINT_FIELD_UINT8:
      return 1U;
    case ZJU_COOP_POINT_FIELD_INT16:
    case ZJU_COOP_POINT_FIELD_UINT16:
      return 2U;
    case ZJU_COOP_POINT_FIELD_INT32:
    case ZJU_COOP_POINT_FIELD_UINT32:
    case ZJU_COOP_POINT_FIELD_FLOAT32:
      return 4U;
    case ZJU_COOP_POINT_FIELD_FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

// packet为待校验的PointCloud2映射；函数只读元数据，不解析或复制点云大缓冲。
zju_coop_error_code_t validate_point_cloud_packet(
    const zju_coop_point_cloud_packet_t& packet) {
  if (packet.node_id == 0U || packet.sensor_id == 0U ||
      packet.sequence == 0U || packet.timestamp_ns == 0U ||
      packet.receive_timestamp_ns == 0U ||
      !nul_terminated_nonempty(packet.frame_id, sizeof(packet.frame_id)) ||
      packet.height == 0U || packet.width == 0U ||
      packet.field_count == 0U || packet.point_step == 0U ||
      packet.row_step == 0U || packet.data == nullptr ||
      !valid_boolean(packet.is_bigendian) ||
      !valid_boolean(packet.is_dense) || !valid_boolean(packet.valid) ||
      packet.status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !all_zero(packet.reserved0)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  // uint64_t乘法容纳两个uint32_t的最大乘积；下面分别验证行跨度和整块data大小。
  const std::uint64_t minimum_row_step =
      static_cast<std::uint64_t>(packet.point_step) * packet.width;
  const std::uint64_t expected_data_size =
      static_cast<std::uint64_t>(packet.row_step) * packet.height;
  if (packet.row_step < minimum_row_step ||
      packet.data_size != expected_data_size ||
      !valid_array_span<zju_coop_point_field_t>(
          packet.fields, packet.field_count, packet.field_stride)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  // index逐项验证字段头、名称、保留位和“offset+元素宽度”没有越过一个点记录。
  for (std::uint32_t index = 0U; index < packet.field_count; ++index) {
    const auto* field = array_element<zju_coop_point_field_t>(
        packet.fields, index, packet.field_stride);
    const auto header_status = validate_header(field);
    if (header_status != ZJU_COOP_OK) {
      return header_status;
    }
    const std::uint32_t element_size =
        point_field_element_size(field->datatype);
    const std::uint64_t field_end =
        static_cast<std::uint64_t>(field->offset) +
        static_cast<std::uint64_t>(element_size) * field->count;
    if (field->struct_size > packet.field_stride ||
        !nul_terminated_nonempty(field->name, sizeof(field->name)) ||
        element_size == 0U || field->count == 0U ||
        !all_zero(field->reserved0) || field->reserved1 != 0U ||
        field_end > packet.point_step) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
  }
  return ZJU_COOP_OK;
}

// packet为待校验的sensor_msgs/Image映射；当前不根据encoding解码像素。
zju_coop_error_code_t validate_camera_image_packet(
    const zju_coop_camera_image_packet_t& packet) {
  const std::uint64_t expected_data_size =
      static_cast<std::uint64_t>(packet.step) * packet.height;
  if (packet.node_id == 0U || packet.camera_id == 0U ||
      packet.sequence == 0U || packet.timestamp_ns == 0U ||
      packet.receive_timestamp_ns == 0U ||
      !nul_terminated_nonempty(packet.frame_id, sizeof(packet.frame_id)) ||
      !nul_terminated_nonempty(packet.encoding, sizeof(packet.encoding)) ||
      packet.height == 0U || packet.width == 0U ||
      packet.step < packet.width || packet.reserved0 != 0U ||
      packet.data == nullptr || packet.data_size != expected_data_size ||
      !valid_boolean(packet.is_bigendian) || !valid_boolean(packet.valid) ||
      packet.status > ZJU_COOP_RANGE_STATUS_INVALID ||
      !all_zero(packet.reserved1)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  return ZJU_COOP_OK;
}

// fill_raw_input_result先在局部变量中构造完整回执，再原子覆盖调用方结构。
void fill_raw_input_result(zju_coop_raw_input_result_t& destination,
                           zju_coop_raw_input_type_t input_type,
                           std::uint32_t node_id, std::uint32_t sensor_id,
                           std::uint64_t sequence,
                           std::uint64_t timestamp_ns) {
  zju_coop_raw_input_result_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;
  output.input_type = input_type;
  output.disposition = ZJU_COOP_RAW_INPUT_VALIDATED_NOT_USED;
  output.node_id = node_id;
  output.sensor_id = sensor_id;
  output.sequence = sequence;
  output.timestamp_ns = timestamp_ns;
  // caller_size允许未来调用方在v1结构尾部追加字段；当前库只覆盖已知v1部分。
  const std::uint32_t caller_size = destination.struct_size;
  destination = output;
  destination.struct_size = caller_size;
}

// node为待进入二维状态初始化的C ABI节点，检查所有物理量均有限。
bool finite_node(const zju_coop_node_initialization_t& node) {
  return is_finite_value(node.x) && is_finite_value(node.y) &&
         is_finite_value(node.vx) && is_finite_value(node.vy) &&
         is_finite_value(node.position_std_m) &&
         is_finite_value(node.velocity_std_mps);
}

// config为待转换的基础配置，仅检查其中全部浮点阈值和噪声参数有限。
bool finite_config(const zju_coop_config_t& config) {
  return is_finite_value(config.process_accel_std_mps2) &&
         is_finite_value(config.nis_gate) &&
         is_finite_value(config.max_prediction_step_s) &&
         is_finite_value(config.min_covariance_diagonal) &&
         is_finite_value(config.nominal_rate_hz) &&
         is_finite_value(config.nlos_ratio_threshold) &&
         is_finite_value(config.valid_ratio_threshold) &&
         is_finite_value(config.rate_ratio_threshold) &&
         is_finite_value(config.nlos_probability_threshold) &&
         is_finite_value(config.nlos_covariance_scale) &&
         is_finite_value(config.rigidity_tolerance);
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

  // 下面逐字段复制，而不直接memcpy，是因为C ABI结构与内部C++结构的布局不保证相同。
  output.filter.reference_node_id = input.reference_node_id;  // 指定所有相对位置所依附的参考节点。
  output.filter.process_accel_std_mps2 = input.process_accel_std_mps2;  // 设置二维运动模型的过程噪声。
  output.filter.nis_gate = input.nis_gate;  // 设置测距创新的统计门限，用于拒绝明显异常量测。
  output.filter.max_prediction_step_s = input.max_prediction_step_s;  // 限制单次预测步长，避免大时间间隔造成数值发散。
  output.filter.min_covariance_diagonal = input.min_covariance_diagonal;  // 给协方差对角线设下限，避免矩阵退化。
  output.nodes = std::move(nodes);  // move转移节点容器所有权，避免再次复制全部初值。
  output.degradation.window_ns = input.degradation_window_ns;  // 质量评价只统计该滑动时间窗内的观测。
  output.degradation.nominal_rate_hz = input.nominal_rate_hz;  // 用名义频率判断实际数据是否掉频。
  output.degradation.nlos_ratio_threshold = input.nlos_ratio_threshold;  // NLOS比例超过该值时认为链路退化。
  output.degradation.valid_ratio_threshold = input.valid_ratio_threshold;  // 有效率低于该值时认为输入质量不足。
  output.degradation.rate_ratio_threshold = input.rate_ratio_threshold;  // 实际/名义频率低于该比例时触发掉频原因。
  output.degradation.nlos_probability_threshold =
      input.nlos_probability_threshold;
  // 上一项规定“多大的NLOS概率算一次NLOS观测”，与统计窗口的NLOS比例门限不是同一个概念。
  output.degradation.nlos_covariance_scale = input.nlos_covariance_scale;  // 退化但仍可用时放大量测方差，降低该观测权重。
  output.degradation.suspend_duration_ns = input.suspend_duration_ns;  // 持续退化达到此时长后暂缓融合。
  output.degradation.reject_duration_ns = input.reject_duration_ns;  // 持续严重异常达到此时长后拒绝融合。
  output.degradation.recovery_duration_ns = input.recovery_duration_ns;  // 连续恢复达到此时长后才回到正常，避免状态抖动。
  output.degradation.max_tracked_edges = input.max_tracked_edges;  // 限制质量状态表大小，避免异常节点号耗尽内存。
  output.edge_timeout_ns = input.edge_timeout_ns;  // 超时未更新的协同边不再参与拓扑。
  output.max_future_skew_ns = input.max_future_skew_ns;  // 拒绝时间戳明显超前于处理时刻的数据。
  output.max_receive_delay_ns = input.max_receive_delay_ns;  // 拒绝接收延迟过大的旧数据。
  output.duplicate_cache_per_link = input.duplicate_cache_per_link;  // 每条有向链路缓存有限个序号用于去重。
  output.max_nodes = input.max_nodes;  // 把调用方承诺的节点资源上限传入Engine。
  output.max_edges = input.max_edges;  // 把调用方承诺的协同边资源上限传入Engine。
  output.max_state_dimension = input.max_state_dimension;  // 限制二维协同滤波器可分配的总状态维数。
  output.rigidity_tolerance = input.rigidity_tolerance;  // 设置几何秩判定时使用的数值容差。
  return ZJU_COOP_OK;  // 到这里说明所有字段均已验证并完成深拷贝。
}

// 分布式模式沿用现有基础配置，避免为三车最小演示再维护一套重复参数结构。
zju_coop_error_code_t convert_distributed_config(
    const zju_coop_config_t& input,
    zju::coop::DistributedFusionConfig& output) {
  zju::coop::EngineConfig base{};
  const auto status = convert_config(input, base);
  if (status != ZJU_COOP_OK) {
    return status;
  }
  if (base.nodes.size() < 2U || !(base.filter.process_accel_std_mps2 > 0.0) ||
      !(base.filter.nis_gate > 0.0) ||
      !(base.filter.max_prediction_step_s > 0.0) ||
      !(base.filter.min_covariance_diagonal > 0.0)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  const long double extrapolation_ns =
      static_cast<long double>(base.filter.max_prediction_step_s) * 1.0e9L;
  if (!std::isfinite(extrapolation_ns) || extrapolation_ns < 1.0L ||
      extrapolation_ns >
          static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  zju::coop::DistributedFusionConfig converted{};
  converted.reference_node_id = base.filter.reference_node_id;
  converted.process_accel_std_mps2 = base.filter.process_accel_std_mps2;
  converted.nis_gate = base.filter.nis_gate;
  converted.min_covariance_diagonal =
      base.filter.min_covariance_diagonal;
  converted.max_extrapolation_ns =
      static_cast<std::uint64_t>(extrapolation_ns);
  converted.node_timeout_ns = base.edge_timeout_ns;
  converted.max_future_skew_ns = base.max_future_skew_ns;
  converted.max_receive_delay_ns = base.max_receive_delay_ns;

  double largest_position_std = 0.0;
  converted.node_ids.reserve(base.nodes.size());
  for (const auto& node : base.nodes) {
    if (node.node_id > std::numeric_limits<std::uint16_t>::max() ||
        !(node.position_std_m > 0.0) || !(node.velocity_std_mps > 0.0)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    converted.node_ids.push_back(node.node_id);
    largest_position_std =
        std::max(largest_position_std, node.position_std_m);
  }
  converted.initial_correction_std_m = largest_position_std;
  output = std::move(converted);
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
      !is_finite_value(input.gravity_mps2) ||
      !is_finite_value(input.min_imu_dt_s) ||
      !is_finite_value(input.max_imu_dt_s) ||
      !is_finite_value(input.max_propagation_substep_s) ||
      !is_finite_value(input.gyro_noise_density_rad_s_sqrt_hz) ||
      !is_finite_value(input.accel_noise_density_m_s2_sqrt_hz) ||
      !is_finite_value(input.gyro_bias_random_walk_rad_s2_sqrt_hz) ||
      !is_finite_value(input.accel_bias_random_walk_m_s3_sqrt_hz) ||
      !is_finite_value(input.min_covariance_diagonal) ||
      !is_finite_value(input.quaternion_norm_tolerance) ||
      !is_finite_value(input.covariance_symmetry_tolerance) ||
      !valid_boolean(input.use_message_covariance) ||
      !valid_boolean(input.use_orientation_for_initialization) ||
      !nul_terminated_nonempty(input.expected_frame_id,
                               sizeof(input.expected_frame_id))) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  nodes.clear();  // 清除调用者可能遗留的旧结果，保证本次转换从空容器开始。
  nodes.reserve(input.node_count);  // 一次预留足够空间，避免push_back过程中反复扩容和复制。
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
    node.node_id = source->node_id;  // 节点编号是后续按平台查找15维状态的主键。
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
    nodes.push_back(node);  // 复制完整节点初值；局部node随后离开本轮循环仍不会悬空。
  }

  // 惯性配置也逐字段复制，便于边界层明确单位，并避免把C结构的填充字节带入算法。
  inertial.gravity_mps2 = input.gravity_mps2;  // ENU导航系中用于抵消重力的标量大小。
  inertial.min_imu_dt_s = input.min_imu_dt_s;  // 小于该间隔通常是重复时间戳或时钟异常。
  inertial.max_imu_dt_s = input.max_imu_dt_s;  // 大于该间隔的数据不连续，不能直接一次传播。
  inertial.max_propagation_substep_s =
      input.max_propagation_substep_s;
  // 上一项把较长但仍合法的IMU间隔切成小步积分，以减小离散化误差。
  inertial.gyro_noise_density_rad_s_sqrt_hz =
      input.gyro_noise_density_rad_s_sqrt_hz;
  // 上一项是陀螺白噪声密度，用于建立姿态误差的连续时间过程噪声。
  inertial.accel_noise_density_m_s2_sqrt_hz =
      input.accel_noise_density_m_s2_sqrt_hz;
  // 上一项是加速度计白噪声密度，用于建立速度和位置误差的过程噪声。
  inertial.gyro_bias_random_walk_rad_s2_sqrt_hz =
      input.gyro_bias_random_walk_rad_s2_sqrt_hz;
  // 上一项描述陀螺零偏随时间随机漂移的强度。
  inertial.accel_bias_random_walk_m_s3_sqrt_hz =
      input.accel_bias_random_walk_m_s3_sqrt_hz;
  // 上一项描述加速度计零偏随时间随机漂移的强度。
  inertial.min_covariance_diagonal = input.min_covariance_diagonal;  // 防止15维协方差出现零或负对角项。
  inertial.quaternion_norm_tolerance = input.quaternion_norm_tolerance;  // 判断输入姿态是否接近单位四元数。
  inertial.covariance_symmetry_tolerance =
      input.covariance_symmetry_tolerance;
  // 上一项允许浮点舍入造成的微小非对称，但拒绝真正损坏的协方差。
  inertial.use_message_covariance =
      input.use_message_covariance == ZJU_COOP_TRUE;
  // 上一项把C的0/1布尔量显式转换为C++ bool，决定是否采用ROS消息自带协方差。
  inertial.use_orientation_for_initialization =
      input.use_orientation_for_initialization == ZJU_COOP_TRUE;
  // 上一项只允许消息姿态参与“首次初始化”，不会把它当作每帧姿态量测更新。
  inertial.expected_frame_id = input.expected_frame_id;  // 深拷贝期望坐标系名，运行时用来阻止FLU/其他坐标系混用。
  return ZJU_COOP_OK;  // 到这里说明15维惯性配置已可安全交给Engine。
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

zju_coop_gnss_disposition_t gnss_disposition(
    zju::coop::GnssPushDisposition value) {
  switch (value) {
    case zju::coop::GnssPushDisposition::kStored:
      return ZJU_COOP_GNSS_STORED;
    case zju::coop::GnssPushDisposition::kInvalidPacket:
      return ZJU_COOP_GNSS_INVALID_PACKET;
    case zju::coop::GnssPushDisposition::kUnknownNode:
      return ZJU_COOP_GNSS_UNKNOWN_NODE;
    case zju::coop::GnssPushDisposition::kDuplicate:
      return ZJU_COOP_GNSS_DUPLICATE;
    case zju::coop::GnssPushDisposition::kOutOfOrder:
      return ZJU_COOP_GNSS_OUT_OF_ORDER;
    case zju::coop::GnssPushDisposition::kTimeRejected:
      return ZJU_COOP_GNSS_TIME_REJECTED;
  }
  return ZJU_COOP_GNSS_INVALID_PACKET;
}

zju_coop_error_code_t convert_gnss_config(
    const zju_coop_gnss_config_t& input,
    zju::coop::GnssReferenceConfig& output) {
  if (input.reference_node_id == 0U || input.node_count == 0U ||
      input.node_count > kMaximumNodes || input.reserved0 != 0U ||
      !valid_boolean(input.require_known_covariance) ||
      !all_zero(input.reserved1) ||
      !valid_array_span<zju_coop_gnss_node_config_t>(
          input.nodes, input.node_count, input.node_stride) ||
      input.max_epoch_skew_ns == 0U || input.max_truth_age_ns == 0U ||
      input.max_future_skew_ns == 0U ||
      input.max_receive_delay_ns == 0U ||
      !is_finite_value(input.min_velocity_dt_s) ||
      !is_finite_value(input.max_velocity_dt_s) ||
      !is_finite_value(input.max_horizontal_std_m) ||
      !is_finite_value(input.max_vertical_std_m)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  output.reference_node_id = input.reference_node_id;
  output.max_epoch_skew_ns = input.max_epoch_skew_ns;
  output.max_truth_age_ns = input.max_truth_age_ns;
  output.max_future_skew_ns = input.max_future_skew_ns;
  output.max_receive_delay_ns = input.max_receive_delay_ns;
  output.min_velocity_dt_s = input.min_velocity_dt_s;
  output.max_velocity_dt_s = input.max_velocity_dt_s;
  output.max_horizontal_std_m = input.max_horizontal_std_m;
  output.max_vertical_std_m = input.max_vertical_std_m;
  output.require_known_covariance =
      input.require_known_covariance == ZJU_COOP_TRUE;
  output.nodes.reserve(input.node_count);
  for (std::uint32_t index = 0U; index < input.node_count; ++index) {
    const auto* source = array_element<zju_coop_gnss_node_config_t>(
        input.nodes, index, input.node_stride);
    const auto header_status = validate_header(source);
    if (header_status != ZJU_COOP_OK ||
        source->struct_size > input.node_stride ||
        source->reserved0 != 0U ||
        !finite_array(source->antenna_lever_arm_body_m) ||
        !finite_array(source->orientation_body_to_enu_xyzw)) {
      return header_status != ZJU_COOP_OK ? header_status
                                          : ZJU_COOP_INVALID_ARGUMENT;
    }
    zju::coop::GnssNodeConfig node{};
    node.node_id = source->node_id;
    node.antenna_lever_arm_body_m =
        {source->antenna_lever_arm_body_m[0],
         source->antenna_lever_arm_body_m[1],
         source->antenna_lever_arm_body_m[2]};
    std::copy(std::begin(source->orientation_body_to_enu_xyzw),
              std::end(source->orientation_body_to_enu_xyzw),
              node.orientation_body_to_enu_xyzw.begin());
    output.nodes.push_back(node);
  }
  return ZJU_COOP_OK;
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
  zju_coop_localization_t output{};  // `{}`先清零未显式赋值字段，防止填充/保留字段泄漏随机内存。
  output.struct_size = sizeof(output);  // 输出携带自身v1结构大小，调用方可做一致性检查。
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;  // 明确下面字段按v1解释。
  output.timestamp_ns = source.timestamp_ns;  // 结果对应的统一算法时刻，而不是网络接收时刻。
  output.node_id = source.node_id;  // 标识这一条结果属于哪辆车。
  output.reference_node_id = source.reference_node_id;  // 标识位置和速度相对于哪个参考节点。
  output.x = source.x;  // ENU相对坐标的东向位置。
  output.y = source.y;  // ENU相对坐标的北向位置。
  output.vx = source.vx;  // 东向相对速度。
  output.vy = source.vy;  // 北向相对速度。
  output.cov_xx = source.cov_xx;  // x位置方差，反映估计不确定度而不是误差真值。
  output.cov_xy = source.cov_xy;  // x和y位置误差的互协方差。
  output.cov_yy = source.cov_yy;  // y位置方差。
  output.valid = source.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;  // 三目运算符把C++ bool映射到规定的C整数布尔值。
  // v1曾明确约定yaw不可用；即使Engine内部已有惯性航向，也必须保持false，
  // 防止旧调用方在结构布局未升级时悄然改变业务行为。真实航向仅由Pose2D v2查询提供。
  output.yaw_valid = ZJU_COOP_FALSE;
  output.z_valid = source.z_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;  // 当前二维UWB闭环通常不会输出有效高度。
  output.state = localization_state(source.state);  // 用显式函数映射枚举，避免依赖两个枚举碰巧同值。
  return output;  // 按值返回独立副本，离开函数后不依赖source生命周期。
}

// source为只读Pose2D快照中的单车元素，返回独立v2 C结构副本。
zju_coop_vehicle_pose2d_v2_t pose2d_vehicle_output(
    const zju::coop::VehiclePose2dSnapshot& source) {
  zju_coop_vehicle_pose2d_v2_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  output.node_id = source.node_id;
  output.x_m = source.x_m;
  output.y_m = source.y_m;
  output.yaw_rad = source.yaw_rad;
  output.position_valid =
      source.position_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.yaw_valid = source.yaw_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  return output;
}

// 分布式核心使用独立普通C++快照类型，但对外复用完全相同的Pose2D v2布局。
zju_coop_vehicle_pose2d_v2_t pose2d_vehicle_output(
    const zju::coop::DistributedVehiclePose2D& source) {
  zju_coop_vehicle_pose2d_v2_t output{};
  output.struct_size = sizeof(output);
  output.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  output.node_id = source.node_id;
  output.x_m = source.x_m;
  output.y_m = source.y_m;
  output.yaw_rad = source.yaw_rad;
  output.position_valid =
      source.position_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.yaw_valid = source.yaw_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  return output;
}

// source为单条规范化无向边的内部质量快照，返回计数安全收窄后的v1 C结构。
zju_coop_observation_t observation_output(
    const zju::coop::ObservationQuality& source) {
  // output为完整初始化后按值返回的C ABI边质量元素。
  zju_coop_observation_t output{};  // 清零全部v1字段和保留空间。
  output.struct_size = sizeof(output);  // 写入库端结构大小。
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;  // 写入库端ABI版本。
  output.from_node = source.edge.first;  // 规范化无向边中编号较小的节点。
  output.to_node = source.edge.second;  // 规范化无向边中编号较大的节点。
  output.window_start_ns = source.window_start_ns;  // 质量统计窗口的起点。
  output.window_end_ns = source.window_end_ns;  // 质量统计窗口的终点。
  output.expected_count = static_cast<std::uint32_t>(source.expected_count);  // 显式收窄为ABI规定的32位计数。
  output.received_count = static_cast<std::uint32_t>(source.received_count);  // 窗口内收到的总包数。
  output.valid_count = static_cast<std::uint32_t>(source.valid_count);  // 窗口内通过字段有效性检查的包数。
  output.nlos_count = static_cast<std::uint32_t>(source.nlos_count);  // 窗口内被标记为非视距的包数。
  output.residual_rejected_count =
      static_cast<std::uint32_t>(source.residual_rejected_count);
  // 上一项记录因NIS残差门限而未进入滤波器的包数。
  output.dropped_count = static_cast<std::uint32_t>(source.dropped_count);  // 记录去重、乱序或时延等原因丢弃的包数。
  output.nlos_ratio = source.nlos_ratio;  // NLOS包数除以收到包数。
  output.valid_ratio = source.valid_ratio;  // 有效包数除以收到包数。
  output.actual_rate_hz = source.actual_rate_hz;  // 根据窗口时长和收到包数估计的实际频率。
  output.state = observation_state(source.state);  // 显式映射长期质量状态。
  output.fusion_action = fusion_action(source.action);  // 显式映射当前应正常、降权、暂缓或拒绝融合。
  output.reason_mask = static_cast<std::uint32_t>(source.reason_mask);  // 位掩码可同时携带NLOS、低有效率、低频率等多个原因。
  output.input_overflow =
      source.input_overflow ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  output.covariance_scale = source.covariance_scale;  // 实际应用到量测方差上的倍率。
  return output;  // 返回不依赖内部容器的C结构副本。
}

// source为Engine的当前拓扑快照，返回不借用source的v1 C网络结构。
zju_coop_network_t network_output(const zju::coop::NetworkSnapshot& source) {
  // output为完成全部计数收窄与状态转换后按值返回的网络元素。
  zju_coop_network_t output{};  // 清零网络状态结构，保证未赋值保留字段确定。
  output.struct_size = sizeof(output);  // 写入当前结构大小。
  output.abi_version = ZJU_COOP_ABI_VERSION_V1;  // 写入当前ABI版本。
  output.timestamp_ns = source.timestamp_ns;  // 网络判断对应的统一算法时刻。
  output.node_count = static_cast<std::uint32_t>(source.node_count);  // 配置中的总节点数。
  output.reachable_node_count =
      static_cast<std::uint32_t>(source.reachable_node_count);
  output.active_edge_count =
      static_cast<std::uint32_t>(source.active_edge_count);
  // 上一项统计从主参考节点经当前有效边能够到达的节点数。
  // active_edge_count只统计当前通过质量与超时判断的协同边。
  output.connected = source.connected ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;  // 全部节点均从参考节点可达时才为真。
  output.observable = source.observable ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;  // 几何约束达到当前算法定义的可观条件时才为真。
  output.reason_mask = static_cast<std::uint32_t>(source.reason_mask);  // 同时报告断连、欠约束或过期等原因位。
  output.state = localization_state(source.state);  // 将内部综合状态显式映射到C ABI。
  return output;  // 按值返回网络快照副本。
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
  return ZJU_COOP_ABI_VERSION_V1;  // 调用方可先比较此值，再决定是否能安全传入本版结构。
}

uint32_t ZJU_COOP_CALL zju_coop_pose2d_abi_version(void) {
  // Pose2D查询独立演进；保留v1函数与结构布局，使既有调用方无需同步升级。
  return ZJU_COOP_POSE2D_ABI_VERSION_V2;
}

// 返回指向库内静态字符串的只读指针；调用方只能读取，不能修改或释放。
const char* ZJU_COOP_CALL zju_coop_version_string(void) { return "0.3.0"; }

const char* ZJU_COOP_CALL zju_coop_error_string(
    zju_coop_error_code_t code) {
  switch (code) {  // switch比连续if更直接地表达“一个枚举值对应一个字符串”。
    case ZJU_COOP_OK:
      return "success";  // 操作成功。
    case ZJU_COOP_INVALID_ARGUMENT:
      return "invalid argument";  // 空指针、非法数值、越界资源或错误调用顺序。
    case ZJU_COOP_ABI_MISMATCH:
      return "ABI version mismatch";  // 调用方与动态库使用了不同结构版本。
    case ZJU_COOP_STRUCT_SIZE_MISMATCH:
      return "structure size mismatch";  // 调用方结构比本版必需字段更短。
    case ZJU_COOP_BUFFER_TOO_SMALL:
      return "output buffer too small";  // 输出数组容量不足，同时函数会返回所需数量。
    case ZJU_COOP_OUT_OF_MEMORY:
      return "out of memory";  // 堆内存分配失败。
    case ZJU_COOP_INTERNAL_ERROR:
      return "internal error";  // 未归类的库内异常或一致性失败。
    case ZJU_COOP_NOT_READY:
      return "not ready";  // 合法调用尚未收齐生成初始化所需的同步样本。
    default:
      return "unknown error";  // 防御未来或损坏的枚举数值，保证总能得到字符串。
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_node_initialization_init(
    zju_coop_node_initialization_t* value) {
  if (value == nullptr) {  // 初始化函数必须有可写目标，nullptr表示调用方没有提供内存。
    return ZJU_COOP_INVALID_ARGUMENT;  // 立即返回，避免下一行解引用空指针。
  }
  *value = {};  // `{}`值初始化整份结构：数值和保留字段全部清零，避免未初始化垃圾值。
  value->struct_size = sizeof(*value);  // 记录调用方实际结构大小，供动态库做向前兼容检查。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 明确该结构遵循v1字段布局。
  value->position_std_m = 0.5;  // 给位置初值设置保守的默认1σ，不假定初值绝对精确。
  value->velocity_std_mps = 0.5;  // 给速度初值设置保守的默认1σ。
  return ZJU_COOP_OK;  // 返回成功仅表示默认值已写好，节点号和状态仍需调用方填写。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_config_init(
    zju_coop_config_t* value) {
  if (value == nullptr) {  // 所有init函数都先做相同空指针防护。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 先清零包括reserved在内的全部字节，满足ABI对保留字段的要求。
  value->struct_size = sizeof(*value);  // sizeof(*value)随当前头文件的真实结构布局自动变化。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 标记默认值属于v1 ABI。
  value->reference_node_id = 1U;  // 默认把1号车作为相对坐标系原点；实车联调时可配置。
  value->node_stride = sizeof(zju_coop_node_initialization_t);  // 默认认为节点数组紧密连续排列。
  value->process_accel_std_mps2 = 0.5;  // 二维常速度模型允许的随机加速度1σ。
  value->nis_gate = 9.0;  // 一维量测约3σ门限的平方，用于统计一致性检验。
  value->max_prediction_step_s = 0.05;  // 把较长预测间隔拆小，改善离散模型精度。
  value->min_covariance_diagonal = 1.0e-9;  // 防止协方差因舍入误差失去正定性。
  value->degradation_window_ns = 2'000'000'000ULL;  // 使用2秒滑动窗口统计质量；数字分隔符只增强可读性。
  value->nominal_rate_hz = 20.0;  // 默认期望每条测距链路20 Hz。
  value->nlos_ratio_threshold = 0.30;  // 窗口内NLOS比例超过30%判为退化。
  value->valid_ratio_threshold = 0.80;  // 窗口内有效包比例低于80%判为退化。
  value->rate_ratio_threshold = 0.80;  // 实际频率低于名义频率80%判为掉频。
  value->nlos_probability_threshold = 0.50;  // 单包NLOS概率达到50%即计为NLOS。
  value->nlos_covariance_scale = 4.0;  // 可疑量测方差扩大4倍，即降低其滤波权重。
  value->suspend_duration_ns = 1'000'000'000ULL;  // 连续退化1秒后暂缓使用。
  value->reject_duration_ns = 3'000'000'000ULL;  // 连续严重异常3秒后拒绝使用。
  value->recovery_duration_ns = 1'000'000'000ULL;  // 连续正常1秒后才恢复，避免频繁切换。
  value->max_tracked_edges = 2016U;  // 64个节点完全图共有64×63÷2=2016条无向边。
  value->duplicate_cache_per_link = 128U;  // 每个有向来源保存最近128个序号用于去重。
  value->edge_timeout_ns = 500'000'000ULL;  // 0.5秒没有新量测就认为边已失活。
  value->max_future_skew_ns = 100'000'000ULL;  // 最多容忍消息时间比当前时刻超前0.1秒。
  value->max_receive_delay_ns = 500'000'000ULL;  // 最多容忍0.5秒端到端接收延迟。
  value->max_nodes = 64U;  // 给内存和计算量设置明确上限，拒绝无限制扩容。
  value->max_edges = 2016U;  // 与64节点完全图的最大无向边数一致。
  value->max_state_dimension = 252U;  // 二维相对滤波为4×(64-1)=252维。
  value->rigidity_tolerance = 1.0e-9;  // 几何秩判断中把极小量视为零，抑制浮点噪声。
  return ZJU_COOP_OK;  // 默认配置可作为起点，但部署前仍应通过INI按传感器实测值标定。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_range_packet_init(
    zju_coop_range_packet_t* value) {
  if (value == nullptr) {  // 没有可写结构就不能建立默认测距包。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零序号、时间戳、节点号、布尔量和保留字段。
  value->struct_size = sizeof(*value);  // 供库校验调用方编译时看到的结构大小。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 供库校验双方ABI版本。
  value->range_std_m = 0.1;  // 默认测距标准差0.1米，实机必须按设备与场景调整。
  value->status = ZJU_COOP_RANGE_STATUS_OK;  // 默认设备状态正常；无效包必须由适配层明确改写。
  return ZJU_COOP_OK;  // 初始化完成后调用方还需填写节点、时间、距离和有效标志。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_range_processing_result_init(
    zju_coop_range_processing_result_t* value) {
  if (value == nullptr) {  // 输出结构同样由调用方分配，所以先检查地址。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零全部诊断量，避免失败路径残留上一次结果。
  value->struct_size = sizeof(*value);  // 告诉动态库该缓冲至少能容纳当前结构。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 固定当前字段语义为v1。
  value->disposition = ZJU_COOP_PROCESSING_INVALID_PACKET;  // 未真正处理前，最安全的默认结论是“无效”。
  value->fusion_action = ZJU_COOP_FUSION_USE_NORMAL;  // 协方差默认不缩放，实际处理会覆盖。
  value->update_disposition = ZJU_COOP_UPDATE_INVALID_PACKET;  // 未更新滤波器前不声称接受量测。
  value->covariance_scale = 1.0;  // 比例1表示保持原始量测方差。
  return ZJU_COOP_OK;  // 只完成结果缓冲初始化，并不代表任何测距已处理。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_localization_init(
    zju_coop_localization_t* value) {
  if (value == nullptr) {  // 防止对空输出指针写入。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 数值清零不能代表有效位置，所以下面还要明确状态。
  value->struct_size = sizeof(*value);  // 保存当前结构容量。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 保存当前ABI版本。
  value->state = ZJU_COOP_LOCALIZATION_UNINITIALIZED;  // 防止调用方把默认零坐标误当成有效结果。
  return ZJU_COOP_OK;  // 后续zju_coop_step成功时才覆盖为真实定位状态。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_network_init(
    zju_coop_network_t* value) {
  if (value == nullptr) {  // network必须指向调用方可写内存。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零节点数、边数、连通/可观标志和原因位。
  value->struct_size = sizeof(*value);  // 保存可写缓冲大小。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 固定字段解释版本。
  value->state = ZJU_COOP_LOCALIZATION_UNINITIALIZED;  // 尚未step前不能声称网络可用。
  return ZJU_COOP_OK;  // 真正拓扑状态由后续step计算。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_observation_init(
    zju_coop_observation_t* value) {
  if (value == nullptr) {  // observation必须指向调用方可写内存。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零窗口计数、比例、节点号和告警原因位。
  value->struct_size = sizeof(*value);  // 记录当前结构容量。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 记录当前字段版本。
  value->state = ZJU_COOP_OBSERVATION_UNKNOWN;  // 没有统计样本前质量只能是未知。
  value->fusion_action = ZJU_COOP_FUSION_USE_NORMAL;  // 默认不修改权重，首个质量判断会覆盖。
  value->covariance_scale = 1.0;  // 1.0表示不放大量测方差。
  return ZJU_COOP_OK;  // 初始化完成不等于该边已经存在。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_vehicle_pose2d_v2_init(
    zju_coop_vehicle_pose2d_v2_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // 整体清零使有效标志和保留字段具有安全默认值；随后写入独立v2头部。
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_pose2d_snapshot_v2_init(
    zju_coop_pose2d_snapshot_v2_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  // timestamp/reference/frame在真正查询前均保持零或空串，防止误作有效快照。
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_inertial_node_initialization_init(
    zju_coop_inertial_node_initialization_t* value) {
  if (value == nullptr) {  // 惯性初值结构必须由调用方提供可写空间。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 先把位置、速度、零偏、标准差及保留字段全部清零。
  value->struct_size = sizeof(*value);  // 保存当前结构大小，供动态库验证。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 指明数组字段遵循v1语义和单位。
  value->orientation_xyzw[3] = 1.0;  // ROS顺序为x,y,z,w；(0,0,0,1)表示无旋转单位四元数。
  // axis遍历ENU/FLU三个轴，为五个15维误差状态块写入默认1σ。
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    value->position_std_m[axis] = 0.5;  // 当前轴位置初值的1σ不确定度。
    value->velocity_std_mps[axis] = 0.5;  // 当前轴速度初值的1σ不确定度。
    value->attitude_std_rad[axis] = 0.1;  // 当前轴小角度误差的1σ不确定度。
    value->gyro_bias_std_rad_s[axis] = 0.01;  // 当前轴陀螺零偏初值的1σ不确定度。
    value->accel_bias_std_m_s2[axis] = 0.1;  // 当前轴加速度计零偏初值的1σ不确定度。
  }
  return ZJU_COOP_OK;  // 调用方随后应按实际初始状态和标定结果覆盖这些默认值。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_inertial_config_init(
    zju_coop_inertial_config_t* value) {
  if (value == nullptr) {  // 惯性配置需要写入调用方提供的结构。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零开关、节点指针、计数和保留字段，先得到确定状态。
  value->struct_size = sizeof(*value);  // 记录当前惯性配置结构的字节大小。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 指明参数单位和布局遵循v1。
  value->node_stride = sizeof(zju_coop_inertial_node_initialization_t);  // 默认节点初值数组紧密排列。
  value->max_inertial_state_dimension = 960U;  // 最多支持64节点×每节点15维误差状态。
  value->gravity_mps2 = 9.80665;  // 使用标准重力作为默认值，部署地点可按需要标定。
  value->min_imu_dt_s = 1.0e-6;  // 小于1微秒通常不是有效采样间隔。
  value->max_imu_dt_s = 0.1;  // 大于0.1秒说明IMU明显断流，拒绝直接传播。
  value->max_propagation_substep_s = 0.01;  // 每个积分子步不超过10毫秒，提高数值稳定性。
  value->gyro_noise_density_rad_s_sqrt_hz = 1.0e-4;  // 陀螺白噪声密度占位默认值，实机应读取规格/标定。
  value->accel_noise_density_m_s2_sqrt_hz = 1.0e-3;  // 加速度计白噪声密度占位默认值。
  value->gyro_bias_random_walk_rad_s2_sqrt_hz = 1.0e-6;  // 陀螺零偏随机游走默认值。
  value->accel_bias_random_walk_m_s3_sqrt_hz = 1.0e-5;  // 加速度计零偏随机游走默认值。
  value->min_covariance_diagonal = 1.0e-12;  // 15维协方差对角线的数值安全下限。
  value->quaternion_norm_tolerance = 1.0e-3;  // 输入四元数范数允许偏离1的最大量。
  value->covariance_symmetry_tolerance = 1.0e-9;  // 检查消息协方差对称性时的浮点容差。
  std::memcpy(value->expected_frame_id, "imu_link", 9U);  // 连同末尾NUL复制9字节，建立默认FLU机体系名称。
  return ZJU_COOP_OK;  // 调用方仍需设置节点数组、节点数及按实测标定噪声。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_imu_packet_init(
    zju_coop_imu_packet_t* value) {
  if (value == nullptr) {  // IMU包必须写入有效结构地址。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零时间、三轴瞬时量、协方差、帧名、有效标志和保留字段。
  value->struct_size = sizeof(*value);  // 声明调用方缓冲大小。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 声明采用v1字段布局。
  value->orientation_xyzw[3] = 1.0;  // 即使orientation_valid为假，也保持四元数数值有限且归一。
  value->status = ZJU_COOP_RANGE_STATUS_OK;  // 复用通用传感器状态枚举；适配层应按设备诊断覆盖。
  return ZJU_COOP_OK;  // 调用方随后填写瞬时角速度、线加速度、时间戳和frame_id。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_imu_processing_result_init(
    zju_coop_imu_processing_result_t* value) {
  if (value == nullptr) {  // 处理结果也必须由调用方预先分配。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零propagated和dt等诊断字段。
  value->struct_size = sizeof(*value);  // 记录结果缓冲容量。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 记录结果字段版本。
  value->disposition = ZJU_COOP_IMU_INVALID_PACKET;  // 未处理前默认无效，避免误报成功。
  return ZJU_COOP_OK;  // push_imu成功返回后才应读取真实处理结论。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_node_state_init(
    zju_coop_node_state_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->orientation_flu_to_enu_xyzw[3] = 1.0;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_point_field_init(
    zju_coop_point_field_t* value) {
  if (value == nullptr) {  // PointField描述必须由调用方提供可写空间。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零name、offset、datatype、count以及全部保留字段。
  value->struct_size = sizeof(*value);  // 声明调用方按照当前v1头文件分配了完整结构。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 固定后续字段的ROS PointField兼容语义。
  return ZJU_COOP_OK;  // 名称、偏移、类型和数量仍需wrapper按真实PointCloud2填写。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_point_cloud_packet_init(
    zju_coop_point_cloud_packet_t* value) {
  if (value == nullptr) {  // 点云包初始化不能写入空指针。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 先清零所有指针、尺寸、标志和保留字段，建立确定的失败安全默认值。
  value->struct_size = sizeof(*value);  // 记录PointCloud2映射结构的当前字节数。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 表示字段遵循v1布局。
  value->field_stride = sizeof(zju_coop_point_field_t);  // 默认fields是紧密连续的结构数组。
  value->status = ZJU_COOP_RANGE_STATUS_OK;  // 默认设备诊断正常，wrapper可按驱动状态覆盖。
  return ZJU_COOP_OK;  // 返回成功只表示默认值已建立，不表示点云数据本身有效。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_camera_image_packet_init(
    zju_coop_camera_image_packet_t* value) {
  if (value == nullptr) {  // 图像包需要有效可写地址。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零图像尺寸、编码、数据指针、标志及保留字段。
  value->struct_size = sizeof(*value);  // 保存当前Image映射结构容量。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 固定字段和单位为v1约定。
  value->status = ZJU_COOP_RANGE_STATUS_OK;  // 默认设备状态正常，但valid仍保持false。
  return ZJU_COOP_OK;  // wrapper随后必须填写时间、帧名、编码、布局和借用数据指针。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_raw_input_result_init(
    zju_coop_raw_input_result_t* value) {
  if (value == nullptr) {  // 回执缓冲必须由调用方分配。
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};  // 清零类型、节点、传感器、序号和时间，避免把旧内容误当成新结果。
  value->struct_size = sizeof(*value);  // 声明结果缓冲至少可容纳v1字段。
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;  // 声明回执字段遵循v1语义。
  value->input_type = ZJU_COOP_RAW_INPUT_UNKNOWN;  // 尚未校验任何数据时类型未知。
  value->disposition = ZJU_COOP_RAW_INPUT_UNINITIALIZED;  // 明确尚无处理结论。
  return ZJU_COOP_OK;  // 只有raw push成功后才能读取VALIDATED_NOT_USED回执。
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_node_config_init(
    zju_coop_gnss_node_config_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->orientation_body_to_enu_xyzw[3] = 1.0;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_config_init(
    zju_coop_gnss_config_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->reference_node_id = 1U;
  value->node_stride = sizeof(zju_coop_gnss_node_config_t);
  value->max_epoch_skew_ns = 100'000'000ULL;
  value->max_truth_age_ns = 500'000'000ULL;
  value->max_future_skew_ns = 100'000'000ULL;
  value->max_receive_delay_ns = 500'000'000ULL;
  value->min_velocity_dt_s = 0.2;
  value->max_velocity_dt_s = 5.0;
  value->max_horizontal_std_m = 0.2;
  value->max_vertical_std_m = 0.5;
  value->require_known_covariance = ZJU_COOP_TRUE;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_fix_packet_init(
    zju_coop_gnss_fix_packet_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->navsat_status = -1;
  value->position_covariance_type =
      ZJU_COOP_GNSS_COVARIANCE_TYPE_UNKNOWN;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_gnss_processing_result_init(
    zju_coop_gnss_processing_result_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->disposition = ZJU_COOP_GNSS_INVALID_PACKET;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_relative_truth_init(
    zju_coop_gnss_relative_truth_t* value) {
  if (value == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *value = {};
  value->struct_size = sizeof(*value);
  value->abi_version = ZJU_COOP_ABI_VERSION_V1;
  value->stale = ZJU_COOP_TRUE;
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

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_distributed_create(
    const zju_coop_config_t* config,
    zju_coop_distributed_handle_t** out_handle) {
  if (out_handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *out_handle = nullptr;
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  try {
    zju::coop::DistributedFusionConfig converted{};
    const auto conversion_status = convert_distributed_config(*config,
                                                               converted);
    if (conversion_status != ZJU_COOP_OK) {
      return conversion_status;
    }
    const auto vehicle_count = static_cast<std::uint32_t>(
        converted.node_ids.size());
    std::unique_ptr<zju_coop_distributed_handle_t> created(
        new zju_coop_distributed_handle_t(std::move(converted),
                                          vehicle_count));
    *out_handle = created.release();
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_distributed_destroy(
    zju_coop_distributed_handle_t* handle) {
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

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_create(
    const zju_coop_gnss_config_t* config,
    zju_coop_gnss_context_t** out_context) {
  if (out_context == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *out_context = nullptr;
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  try {
    zju::coop::GnssReferenceConfig converted{};
    const auto conversion_status = convert_gnss_config(*config, converted);
    if (conversion_status != ZJU_COOP_OK) {
      return conversion_status;
    }
    std::unique_ptr<zju_coop_gnss_context_t> created(
        new zju_coop_gnss_context_t(std::move(converted)));
    *out_context = created.release();
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_destroy(
    zju_coop_gnss_context_t* context) {
  if (context == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    delete context;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_gnss_push_fix(
    zju_coop_gnss_context_t* context,
    const zju_coop_gnss_fix_packet_t* packet,
    zju_coop_gnss_processing_result_t* result) {
  if (context == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto packet_header = validate_header(packet);
  if (packet_header != ZJU_COOP_OK) {
    return packet_header;
  }
  const auto result_header = validate_header(result);
  if (result_header != ZJU_COOP_OK) {
    return result_header;
  }
  if (packet->reserved0 != 0U || packet->reserved1 != 0U ||
      packet->reserved2 != 0U || !all_zero(packet->reserved3) ||
      !valid_boolean(packet->valid) ||
      packet->position_covariance_type >
          ZJU_COOP_GNSS_COVARIANCE_TYPE_KNOWN ||
      !nul_terminated_nonempty(packet->frame_id,
                               sizeof(packet->frame_id)) ||
      !is_finite_value(packet->latitude_deg) ||
      !is_finite_value(packet->longitude_deg) ||
      !is_finite_value(packet->altitude_m) ||
      !finite_array(packet->position_covariance_m2)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    zju::coop::GnssFix converted{};
    converted.node_id = packet->node_id;
    converted.sequence = packet->sequence;
    converted.timestamp_ns = packet->timestamp_ns;
    converted.receive_timestamp_ns = packet->receive_timestamp_ns;
    converted.frame_id = packet->frame_id;
    converted.status = packet->navsat_status;
    converted.service = packet->service;
    converted.latitude_deg = packet->latitude_deg;
    converted.longitude_deg = packet->longitude_deg;
    converted.altitude_m = packet->altitude_m;
    std::copy(std::begin(packet->position_covariance_m2),
              std::end(packet->position_covariance_m2),
              converted.position_covariance_m2.begin());
    converted.covariance_type =
        static_cast<zju::coop::GnssCovarianceType>(
            packet->position_covariance_type);
    converted.valid = packet->valid == ZJU_COOP_TRUE;
    const auto disposition = context->reference->push(converted);

    zju_coop_gnss_processing_result_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.disposition = gnss_disposition(disposition);
    output.node_id = packet->node_id;
    output.sequence = packet->sequence;
    output.timestamp_ns = packet->timestamp_ns;
    const std::uint32_t caller_size = result->struct_size;
    *result = output;
    result->struct_size = caller_size;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_gnss_build_initializations(
    zju_coop_gnss_context_t* context,
    zju_coop_node_initialization_t* base_nodes, uint32_t base_capacity,
    uint32_t base_stride,
    zju_coop_inertial_node_initialization_t* inertial_nodes,
    uint32_t inertial_capacity, uint32_t inertial_stride,
    uint32_t* node_count) {
  if (context == nullptr || node_count == nullptr ||
      (base_capacity == 0U) != (base_nodes == nullptr) ||
      (inertial_capacity == 0U) != (inertial_nodes == nullptr)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    const auto built = context->reference->build_initializations();
    if (!built) {
      *node_count = 0U;
      return ZJU_COOP_NOT_READY;
    }
    const std::uint32_t required =
        static_cast<std::uint32_t>(built->size());
    if (base_capacity < required || inertial_capacity < required) {
      *node_count = required;
      return ZJU_COOP_BUFFER_TOO_SMALL;
    }
    if (!valid_array_span<zju_coop_node_initialization_t>(
            base_nodes, required, base_stride) ||
        !valid_array_span<zju_coop_inertial_node_initialization_t>(
            inertial_nodes, required, inertial_stride)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto* base = array_element<zju_coop_node_initialization_t>(
          base_nodes, index, base_stride);
      const auto* inertial =
          array_element<zju_coop_inertial_node_initialization_t>(
              inertial_nodes, index, inertial_stride);
      const auto base_header = validate_header(base);
      const auto inertial_header = validate_header(inertial);
      if (base_header != ZJU_COOP_OK ||
          inertial_header != ZJU_COOP_OK ||
          base->struct_size > base_stride ||
          inertial->struct_size > inertial_stride) {
        return base_header != ZJU_COOP_OK
                   ? base_header
                   : (inertial_header != ZJU_COOP_OK
                          ? inertial_header
                          : ZJU_COOP_INVALID_ARGUMENT);
      }
    }

    std::vector<zju_coop_node_initialization_t> base_values(required);
    std::vector<zju_coop_inertial_node_initialization_t> inertial_values(
        required);
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto& source = (*built)[index];
      (void)zju_coop_node_initialization_init(&base_values[index]);
      (void)zju_coop_inertial_node_initialization_init(
          &inertial_values[index]);
      auto& base = base_values[index];
      auto& inertial = inertial_values[index];
      base.node_id = source.node_id;
      base.x = source.position_enu_m.x;
      base.y = source.position_enu_m.y;
      base.vx = source.velocity_enu_mps.x;
      base.vy = source.velocity_enu_mps.y;
      base.position_std_m =
          std::sqrt(std::max(source.position_covariance_m2[0],
                             source.position_covariance_m2[4]));
      base.velocity_std_mps =
          std::sqrt(std::max(source.velocity_covariance_m2ps2[0],
                             source.velocity_covariance_m2ps2[4]));
      inertial.node_id = source.node_id;
      inertial.position_n_m[0] = source.position_enu_m.x;
      inertial.position_n_m[1] = source.position_enu_m.y;
      inertial.position_n_m[2] = source.position_enu_m.z;
      inertial.velocity_n_mps[0] = source.velocity_enu_mps.x;
      inertial.velocity_n_mps[1] = source.velocity_enu_mps.y;
      inertial.velocity_n_mps[2] = source.velocity_enu_mps.z;
      std::copy(source.orientation_body_to_enu_xyzw.begin(),
                source.orientation_body_to_enu_xyzw.end(),
                std::begin(inertial.orientation_xyzw));
      for (std::size_t axis = 0U; axis < 3U; ++axis) {
        inertial.position_std_m[axis] =
            std::sqrt(source.position_covariance_m2[axis * 3U + axis]);
        inertial.velocity_std_mps[axis] =
            std::sqrt(
                source.velocity_covariance_m2ps2[axis * 3U + axis]);
      }
    }
    for (std::uint32_t index = 0U; index < required; ++index) {
      auto* base = array_element<zju_coop_node_initialization_t>(
          base_nodes, index, base_stride);
      auto* inertial =
          array_element<zju_coop_inertial_node_initialization_t>(
              inertial_nodes, index, inertial_stride);
      const std::uint32_t base_size = base->struct_size;
      const std::uint32_t inertial_size = inertial->struct_size;
      *base = base_values[index];
      *inertial = inertial_values[index];
      base->struct_size = base_size;
      inertial->struct_size = inertial_size;
    }
    *node_count = required;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_gnss_get_relative_truth(
    zju_coop_gnss_context_t* context, uint64_t now_ns,
    zju_coop_gnss_relative_truth_t* truths, uint32_t truth_capacity,
    uint32_t truth_stride, uint32_t* truth_count) {
  if (context == nullptr || truth_count == nullptr ||
      (truth_capacity == 0U) != (truths == nullptr)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    const auto values = context->reference->relative_truth(now_ns);
    const std::uint32_t required =
        static_cast<std::uint32_t>(values.size());
    if (truth_capacity < required) {
      *truth_count = required;
      return ZJU_COOP_BUFFER_TOO_SMALL;
    }
    if (!valid_array_span<zju_coop_gnss_relative_truth_t>(
            truths, required, truth_stride)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto* destination =
          array_element<zju_coop_gnss_relative_truth_t>(
              truths, index, truth_stride);
      const auto header = validate_header(destination);
      if (header != ZJU_COOP_OK ||
          destination->struct_size > truth_stride) {
        return header != ZJU_COOP_OK ? header
                                     : ZJU_COOP_INVALID_ARGUMENT;
      }
    }
    std::vector<zju_coop_gnss_relative_truth_t> converted(required);
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto& source = values[index];
      auto& destination = converted[index];
      (void)zju_coop_gnss_relative_truth_init(&destination);
      destination.node_id = source.node_id;
      destination.reference_node_id = source.reference_node_id;
      destination.node_timestamp_ns = source.node_timestamp_ns;
      destination.reference_timestamp_ns =
          source.reference_timestamp_ns;
      destination.position_enu_m[0] = source.position_enu_m.x;
      destination.position_enu_m[1] = source.position_enu_m.y;
      destination.position_enu_m[2] = source.position_enu_m.z;
      std::copy(source.position_covariance_m2.begin(),
                source.position_covariance_m2.end(),
                std::begin(destination.position_covariance_m2));
      destination.valid =
          source.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
      destination.stale =
          source.stale ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    }
    for (std::uint32_t index = 0U; index < required; ++index) {
      auto* destination =
          array_element<zju_coop_gnss_relative_truth_t>(
              truths, index, truth_stride);
      const std::uint32_t caller_size = destination->struct_size;
      *destination = converted[index];
      destination->struct_size = caller_size;
    }
    *truth_count = required;
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

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_get_node_state(
    zju_coop_handle_t* handle, uint32_t node_id,
    zju_coop_node_state_t* state) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto header_status = validate_header(state);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  if (state->reserved0 != 0U || !all_zero(state->reserved1)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    const auto* filter = handle->engine->inertial_filter();
    if (filter == nullptr) {
      return ZJU_COOP_NOT_READY;
    }
    const auto& node_ids = filter->node_ids();
    if (std::find(node_ids.begin(), node_ids.end(), node_id) ==
        node_ids.end()) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    const auto& source = filter->state(node_id);
    if (!zju::coop::finite(source.position_n_m) ||
        !zju::coop::finite(source.velocity_n_mps) ||
        !source.orientation_b_to_n.finite()) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    const auto estimate = filter->estimate(node_id);

    zju_coop_node_state_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.node_id = node_id;
    output.timestamp_ns = estimate.pose_timestamp_ns;
    output.position_enu_m[0] = source.position_n_m.x;
    output.position_enu_m[1] = source.position_n_m.y;
    output.position_enu_m[2] = source.position_n_m.z;
    output.velocity_enu_mps[0] = source.velocity_n_mps.x;
    output.velocity_enu_mps[1] = source.velocity_n_mps.y;
    output.velocity_enu_mps[2] = source.velocity_n_mps.z;
    output.orientation_flu_to_enu_xyzw[0] =
        source.orientation_b_to_n.x;
    output.orientation_flu_to_enu_xyzw[1] =
        source.orientation_b_to_n.y;
    output.orientation_flu_to_enu_xyzw[2] =
        source.orientation_b_to_n.z;
    output.orientation_flu_to_enu_xyzw[3] =
        source.orientation_b_to_n.w;
    output.valid = output.timestamp_ns != 0U ? ZJU_COOP_TRUE
                                             : ZJU_COOP_FALSE;

    const std::uint32_t caller_size = state->struct_size;
    *state = output;
    state->struct_size = caller_size;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_distributed_push_node_state(
    zju_coop_distributed_handle_t* handle,
    const zju_coop_node_state_t* state) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto header_status = validate_header(state);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  if (state->reserved0 != 0U || !all_zero(state->reserved1) ||
      !valid_boolean(state->valid) ||
      !finite_array(state->position_enu_m) ||
      !finite_array(state->velocity_enu_mps) ||
      !finite_array(state->orientation_flu_to_enu_xyzw)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    zju::coop::NodeState converted{};
    converted.node_id = state->node_id;
    converted.timestamp_ns = state->timestamp_ns;
    converted.receive_timestamp_ns = state->receive_timestamp_ns;
    converted.position_enu_m = {state->position_enu_m[0],
                                state->position_enu_m[1],
                                state->position_enu_m[2]};
    converted.velocity_enu_mps = {state->velocity_enu_mps[0],
                                  state->velocity_enu_mps[1],
                                  state->velocity_enu_mps[2]};
    converted.orientation_flu_to_enu = {
        state->orientation_flu_to_enu_xyzw[3],
        state->orientation_flu_to_enu_xyzw[0],
        state->orientation_flu_to_enu_xyzw[1],
        state->orientation_flu_to_enu_xyzw[2]};
    converted.valid = state->valid == ZJU_COOP_TRUE;
    return handle->fusion->push_node_state(std::move(converted))
               ? ZJU_COOP_OK
               : ZJU_COOP_INVALID_ARGUMENT;
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
      !is_finite_value(packet->range_m) ||
      !is_finite_value(packet->range_std_m) ||
      !is_finite_value(static_cast<double>(packet->nlos_probability)) ||
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

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_distributed_push_range(
    zju_coop_distributed_handle_t* handle,
    const zju_coop_range_packet_t* packet,
    zju_coop_range_processing_result_t* result) {
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
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
      packet->receive_timestamp_ns == 0U ||
      !is_finite_value(packet->range_m) ||
      !is_finite_value(packet->range_std_m) ||
      !is_finite_value(static_cast<double>(packet->nlos_probability)) ||
      packet->nlos_probability < 0.0F || packet->nlos_probability > 1.0F) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
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

    const auto update = handle->fusion->push_range(converted);
    zju_coop_range_processing_result_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.from_node = std::min<std::uint32_t>(packet->from_node,
                                              packet->to_node);
    output.to_node = std::max<std::uint32_t>(packet->from_node,
                                            packet->to_node);
    if (update.disposition == zju::coop::UpdateDisposition::OutOfOrder) {
      output.disposition = ZJU_COOP_PROCESSING_OUT_OF_ORDER;
    } else if (update.disposition ==
                   zju::coop::UpdateDisposition::InvalidPacket ||
               update.disposition ==
                   zju::coop::UpdateDisposition::UnknownNode ||
               update.disposition ==
                   zju::coop::UpdateDisposition::SelfRange ||
               update.disposition ==
                   zju::coop::UpdateDisposition::NonPositiveRange) {
      output.disposition = ZJU_COOP_PROCESSING_INVALID_PACKET;
    } else {
      output.disposition = ZJU_COOP_PROCESSING_PROCESSED;
    }
    output.fusion_action = ZJU_COOP_FUSION_USE_NORMAL;
    output.update_disposition = update_disposition(update.disposition);
    output.filter_updated =
        update.disposition == zju::coop::UpdateDisposition::Accepted ||
                update.disposition ==
                    zju::coop::UpdateDisposition::NisRejected
            ? ZJU_COOP_TRUE
            : ZJU_COOP_FALSE;
    output.innovation_m = update.innovation_m;
    output.innovation_variance = update.innovation_variance;
    output.nis = update.nis;
    output.covariance_scale = update.covariance_scale;

    const std::uint32_t caller_size = result->struct_size;
    *result = output;
    result->struct_size = caller_size;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_point_cloud(
    zju_coop_handle_t* handle,
    const zju_coop_point_cloud_packet_t* packet,
    zju_coop_raw_input_result_t* result) {
  // 阶段1先校验三个顶层指针；原始大缓冲只借用，不进入Engine或任何持久容器。
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto packet_header_status = validate_header(packet);
  if (packet_header_status != ZJU_COOP_OK) {
    return packet_header_status;
  }
  const auto result_header_status = validate_header(result);
  if (result_header_status != ZJU_COOP_OK) {
    return result_header_status;
  }
  // packet_status覆盖PointCloud2行跨度、字段数组、每个字段范围以及data总长度。
  const auto packet_status = validate_point_cloud_packet(*packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;  // 失败时不调用fill，调用方result保持进入函数前的原值。
  }

  try {
    // 阶段2只生成“已校验但未使用”回执；当前版本故意不设置processing_started。
    fill_raw_input_result(*result, ZJU_COOP_RAW_INPUT_POINT_CLOUD,
                          packet->node_id, packet->sensor_id,
                          packet->sequence, packet->timestamp_ns);
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();  // 防御未来扩展；C++异常绝不能穿越C ABI边界。
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_camera_image(
    zju_coop_handle_t* handle,
    const zju_coop_camera_image_packet_t* packet,
    zju_coop_raw_input_result_t* result) {
  // 图像入口与点云入口保持相同的“头部→业务布局→原子回执”处理顺序。
  if (handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto packet_header_status = validate_header(packet);
  if (packet_header_status != ZJU_COOP_OK) {
    return packet_header_status;
  }
  const auto result_header_status = validate_header(result);
  if (result_header_status != ZJU_COOP_OK) {
    return result_header_status;
  }
  // packet_status验证帧名、编码、图像行跨度、data长度、布尔值和保留字段。
  const auto packet_status = validate_camera_image_packet(*packet);
  if (packet_status != ZJU_COOP_OK) {
    return packet_status;  // 不产生半写结果，也不会冻结随后允许的一次惯性配置。
  }

  try {
    // camera_id统一映射到通用回执sensor_id，方便wrapper使用同一套日志代码。
    fill_raw_input_result(*result, ZJU_COOP_RAW_INPUT_CAMERA_IMAGE,
                          packet->node_id, packet->camera_id,
                          packet->sequence, packet->timestamp_ns);
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

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_get_pose2d_v2(
    zju_coop_handle_t* handle, zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t vehicle_capacity,
    uint32_t vehicle_stride, uint32_t* vehicle_count) {
  // 指针与容量必须成对出现；NULL/0/0只用于第一次查询所需车辆数。
  if (handle == nullptr || snapshot == nullptr || vehicle_count == nullptr ||
      (vehicle_capacity != 0U && vehicles == nullptr) ||
      (vehicle_capacity == 0U && vehicles != nullptr) ||
      (vehicle_capacity == 0U && vehicle_stride != 0U)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  const auto snapshot_status = validate_pose2d_header(snapshot);
  if (snapshot_status != ZJU_COOP_OK) {
    return snapshot_status;
  }
  if (snapshot->reserved0 != 0U) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    const std::uint32_t required = handle->localization_count;
    if (vehicle_capacity < required) {
      // 与v1 step保持相同的两次调用契约：容量不足只写回需求数量，不读取数组、不推进Engine。
      *vehicle_count = required;
      return ZJU_COOP_BUFFER_TOO_SMALL;
    }
    if (!valid_array_span<zju_coop_vehicle_pose2d_v2_t>(
            vehicles, required, vehicle_stride)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }

    // 写入前逐元素验证v2头、stride和保留字段，任何失败均保持全部输出不变。
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto* output = array_element<zju_coop_vehicle_pose2d_v2_t>(
          vehicles, index, vehicle_stride);
      const auto status = validate_pose2d_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > vehicle_stride ||
          output->reserved0 != 0U || !all_zero(output->reserved)) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }

    // pose2d_snapshot是const只读查询，不调用step，也不改变时间、质量窗、去重缓存或滤波状态。
    const auto current = handle->engine->pose2d_snapshot();
    if (current.vehicles.size() != required) {
      return ZJU_COOP_INTERNAL_ERROR;
    }

    zju_coop_pose2d_snapshot_v2_t snapshot_value{};
    snapshot_value.struct_size = sizeof(snapshot_value);
    snapshot_value.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
    snapshot_value.timestamp_ns = current.timestamp_ns;
    snapshot_value.reference_node_id = current.reference_node_id;
    const std::string frame_id =
        "coop_ref_" + std::to_string(current.reference_node_id) + "_enu";
    if (frame_id.size() >= sizeof(snapshot_value.frame_id)) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    std::memcpy(snapshot_value.frame_id, frame_id.c_str(),
                frame_id.size() + 1U);

    std::vector<zju_coop_vehicle_pose2d_v2_t> vehicle_values(required);
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto& source = current.vehicles[index];
      // 无论有效位如何，跨ABI浮点字段都必须保持有限，避免NaN污染ROS 2/GCS。
      if (!is_finite_value(source.x_m) ||
          !is_finite_value(source.y_m) ||
          !is_finite_value(source.yaw_rad)) {
        return ZJU_COOP_INTERNAL_ERROR;
      }
      vehicle_values[index] = pose2d_vehicle_output(source);
    }

    // 所有验证与转换成功后再一次性提交，保留调用方可能大于当前版本的struct_size。
    const std::uint32_t snapshot_caller_size = snapshot->struct_size;
    *snapshot = snapshot_value;
    snapshot->struct_size = snapshot_caller_size;
    for (std::uint32_t index = 0U; index < required; ++index) {
      auto* output = array_element<zju_coop_vehicle_pose2d_v2_t>(
          vehicles, index, vehicle_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = vehicle_values[index];
      output->struct_size = caller_size;
    }
    *vehicle_count = required;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_distributed_get_pose2d_v2(
    zju_coop_distributed_handle_t* handle, uint64_t now_ns,
    zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t vehicle_capacity,
    uint32_t vehicle_stride, uint32_t* vehicle_count) {
  if (handle == nullptr || snapshot == nullptr || vehicle_count == nullptr ||
      (vehicle_capacity != 0U && vehicles == nullptr) ||
      (vehicle_capacity == 0U && vehicles != nullptr) ||
      (vehicle_capacity == 0U && vehicle_stride != 0U)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  const auto snapshot_status = validate_pose2d_header(snapshot);
  if (snapshot_status != ZJU_COOP_OK) {
    return snapshot_status;
  }
  if (snapshot->reserved0 != 0U) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
    const std::uint32_t required = handle->vehicle_count;
    if (vehicle_capacity < required) {
      *vehicle_count = required;
      return ZJU_COOP_BUFFER_TOO_SMALL;
    }
    if (!valid_array_span<zju_coop_vehicle_pose2d_v2_t>(
            vehicles, required, vehicle_stride)) {
      return ZJU_COOP_INVALID_ARGUMENT;
    }
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto* output = array_element<zju_coop_vehicle_pose2d_v2_t>(
          vehicles, index, vehicle_stride);
      const auto status = validate_pose2d_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > vehicle_stride ||
          output->reserved0 != 0U || !all_zero(output->reserved)) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }

    const auto current = handle->fusion->pose2d_snapshot(now_ns);
    if (current.vehicles.size() != required) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    zju_coop_pose2d_snapshot_v2_t snapshot_value{};
    snapshot_value.struct_size = sizeof(snapshot_value);
    snapshot_value.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
    snapshot_value.timestamp_ns = current.timestamp_ns;
    snapshot_value.reference_node_id = current.reference_node_id;
    const std::string frame_id =
        "coop_ref_" + std::to_string(current.reference_node_id) + "_enu";
    if (frame_id.size() >= sizeof(snapshot_value.frame_id)) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    std::memcpy(snapshot_value.frame_id, frame_id.c_str(),
                frame_id.size() + 1U);

    std::vector<zju_coop_vehicle_pose2d_v2_t> vehicle_values(required);
    for (std::uint32_t index = 0U; index < required; ++index) {
      const auto& source = current.vehicles[index];
      if (!is_finite_value(source.x_m) || !is_finite_value(source.y_m) ||
          !is_finite_value(source.yaw_rad)) {
        return ZJU_COOP_INTERNAL_ERROR;
      }
      vehicle_values[index] = pose2d_vehicle_output(source);
    }

    const std::uint32_t snapshot_caller_size = snapshot->struct_size;
    *snapshot = snapshot_value;
    snapshot->struct_size = snapshot_caller_size;
    for (std::uint32_t index = 0U; index < required; ++index) {
      auto* output = array_element<zju_coop_vehicle_pose2d_v2_t>(
          vehicles, index, vehicle_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = vehicle_values[index];
      output->struct_size = caller_size;
    }
    *vehicle_count = required;
    return ZJU_COOP_OK;
  } catch (...) {
    return exception_code();
  }
}

}  // extern "C"

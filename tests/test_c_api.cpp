// 模块职责：从调用方视角验证C ABI版本、结构步长/对齐、缓冲查询、异常转换和原子输出。
// 关键用例故意提供坏指针跨度、错误版本和不足容量，确认失败不会推进或部分写入状态。
#include "test_support.hpp"
#include "zju_coop/c_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace {

// kTimestampNs：C ABI输入场景统一使用的50 ms有效时刻；两个Stride常量是标准输出结构的紧密步长。
constexpr std::uint64_t kTimestampNs = 50'000'000ULL;
constexpr std::uint32_t kLocalizationStride =
    static_cast<std::uint32_t>(sizeof(zju_coop_localization_t));
constexpr std::uint32_t kObservationStride =
    static_cast<std::uint32_t>(sizeof(zju_coop_observation_t));

struct ExtendedNode {
  // value是ABI可见前缀，tail是验证自定义大步长不会被库越界改写的调用方扩展区。
  zju_coop_node_initialization_t value{};
  std::uint64_t tail{};
};

struct ExtendedLocalization {
  // value是定位输出前缀，tail是跨zju_coop_step保持不变的调用方私有尾部。
  zju_coop_localization_t value{};
  std::uint64_t tail{};
};

struct ExtendedObservation {
  // value是观测输出前缀，tail是跨zju_coop_step保持不变的调用方私有尾部。
  zju_coop_observation_t value{};
  std::uint64_t tail{};
};

struct TestEngine {
  // handle：由构造函数创建、析构函数销毁的独占C句柄，在辅助对象生命周期内保持有效。
  zju_coop_handle_t* handle{};

  TestEngine() {
    // nodes：构造器栈内三节点3-4布局，create必须在其销毁前深拷贝；node逐项初始化结构头和标准差。
    std::array<zju_coop_node_initialization_t, 3U> nodes{};
    for (auto& node : nodes) {
      EXPECT_EQ(zju_coop_node_initialization_init(&node), ZJU_COOP_OK);
      node.position_std_m = 0.1;
      node.velocity_std_mps = 0.1;
    }
    nodes[0U].node_id = 1U;
    nodes[1U].node_id = 2U;
    nodes[1U].x = 3.0;
    nodes[2U].node_id = 3U;
    nodes[2U].y = 4.0;

    // config：只在create调用期间有效的C配置，handle不得借用其节点指针。
    zju_coop_config_t config{};
    EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
    config.reference_node_id = 1U;
    config.nodes = nodes.data();
    config.node_count = static_cast<std::uint32_t>(nodes.size());
    config.nis_gate = 1.0e9;
    EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_OK);
    EXPECT_TRUE(handle != nullptr);
  }

  ~TestEngine() {
    if (handle != nullptr) {
      static_cast<void>(zju_coop_destroy(handle));
    }
  }

  TestEngine(const TestEngine&) = delete;
  TestEngine& operator=(const TestEngine&) = delete;
};

// from/to/range_m/timestamp_ns分别指定边端点、距离和采样/接收时刻。
zju_coop_range_packet_t range_packet(std::uint16_t from, std::uint16_t to,
                                     double range_m,
                                     std::uint64_t timestamp_ns) {
  // packet：已初始化结构头、0.1 m标准差和有效状态的测距输入。
  zju_coop_range_packet_t packet{};
  EXPECT_EQ(zju_coop_range_packet_init(&packet), ZJU_COOP_OK);
  packet.from_node = from;
  packet.to_node = to;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns;
  packet.range_m = range_m;
  packet.range_std_m = 0.1;
  packet.valid = ZJU_COOP_TRUE;
  packet.status = ZJU_COOP_RANGE_STATUS_OK;
  return packet;
}

zju_coop_range_processing_result_t initialized_push_result() {
  // result：带正确结构大小和ABI版本的空测距处理输出缓冲。
  zju_coop_range_processing_result_t result{};
  EXPECT_EQ(zju_coop_range_processing_result_init(&result), ZJU_COOP_OK);
  return result;
}

// localizations/observations/network均由调用方持有，本函数只初始化结构头；循环引用仅在遍历期间有效。
void initialize_outputs(std::vector<zju_coop_localization_t>& localizations,
                        std::vector<zju_coop_observation_t>& observations,
                        zju_coop_network_t& network) {
  for (auto& localization : localizations) {
    EXPECT_EQ(zju_coop_localization_init(&localization), ZJU_COOP_OK);
  }
  for (auto& observation : observations) {
    EXPECT_EQ(zju_coop_observation_init(&observation), ZJU_COOP_OK);
  }
  EXPECT_EQ(zju_coop_network_init(&network), ZJU_COOP_OK);
}

}  // namespace

// 契约组锁定版本、结构默认值和公开原因位，避免跨语言调用方读取漂移枚举。
TEST_CASE(c_api_exposes_stable_version_defaults_and_error_strings) {
  EXPECT_EQ(zju_coop_abi_version(), ZJU_COOP_ABI_VERSION_V1);
  EXPECT_TRUE(zju_coop_version_string() != nullptr);
  EXPECT_TRUE(std::strlen(zju_coop_version_string()) != 0U);
  EXPECT_TRUE(std::strlen(zju_coop_error_string(ZJU_COOP_OK)) != 0U);
  EXPECT_TRUE(std::strlen(zju_coop_error_string(
                  static_cast<zju_coop_error_code_t>(999))) != 0U);

  EXPECT_EQ(zju_coop_config_init(nullptr), ZJU_COOP_INVALID_ARGUMENT);
  // config：公开初始化函数写入的默认配置，期望版本、容量和步长均稳定。
  zju_coop_config_t config{};
  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  EXPECT_EQ(config.struct_size, sizeof(config));
  EXPECT_EQ(config.abi_version, ZJU_COOP_ABI_VERSION_V1);
  EXPECT_TRUE(config.process_accel_std_mps2 > 0.0);
  EXPECT_TRUE(config.nis_gate > 0.0);
  EXPECT_TRUE(config.max_prediction_step_s > 0.0);
  EXPECT_EQ(config.max_nodes, 64U);
  EXPECT_EQ(config.node_stride, sizeof(zju_coop_node_initialization_t));
}

TEST_CASE(c_api_reason_mask_bits_are_public_and_stable) {
  EXPECT_EQ(ZJU_COOP_REASON_NONE, UINT32_C(0));
  EXPECT_EQ(ZJU_COOP_REASON_NLOS_RATIO_HIGH, UINT32_C(1) << 0U);
  EXPECT_EQ(ZJU_COOP_REASON_VALID_RATIO_LOW, UINT32_C(1) << 1U);
  EXPECT_EQ(ZJU_COOP_REASON_RATE_LOW, UINT32_C(1) << 2U);
  EXPECT_EQ(ZJU_COOP_REASON_RANGE_RESIDUAL_HIGH, UINT32_C(1) << 3U);
  EXPECT_EQ(ZJU_COOP_REASON_TIME_SYNC_TIMEOUT, UINT32_C(1) << 4U);
  EXPECT_EQ(ZJU_COOP_REASON_LINK_TIMEOUT, UINT32_C(1) << 5U);
  EXPECT_EQ(ZJU_COOP_REASON_GRAPH_GEOMETRY_DEGENERATE,
            UINT32_C(1) << 6U);
  EXPECT_EQ(ZJU_COOP_REASON_NODE_UNREACHABLE, UINT32_C(1) << 7U);
  EXPECT_EQ(ZJU_COOP_REASON_INITIALIZATION_MISSING, UINT32_C(1) << 8U);
  EXPECT_EQ(ZJU_COOP_REASON_INPUT_OVERFLOW, UINT32_C(1) << 9U);
}

// 创建/跨度组故意提供空指针、坏版本、未对齐stride和溢出地址，校验前不得解引用。
TEST_CASE(c_api_create_rejects_null_bad_size_and_bad_version) {
  // handle：预置非空哨兵以确认所有创建失败路径都会清零输出；config依次注入坏大小和坏ABI版本。
  zju_coop_handle_t* handle = reinterpret_cast<zju_coop_handle_t*>(1U);
  EXPECT_EQ(zju_coop_create(nullptr, &handle), ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_TRUE(handle == nullptr);

  zju_coop_config_t config{};
  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  EXPECT_EQ(zju_coop_create(&config, nullptr), ZJU_COOP_INVALID_ARGUMENT);

  handle = reinterpret_cast<zju_coop_handle_t*>(1U);
  config.struct_size = sizeof(config) - 1U;
  EXPECT_EQ(zju_coop_create(&config, &handle),
            ZJU_COOP_STRUCT_SIZE_MISMATCH);
  EXPECT_TRUE(handle == nullptr);

  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  config.abi_version = 0U;
  handle = reinterpret_cast<zju_coop_handle_t*>(1U);
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_ABI_MISMATCH);
  EXPECT_TRUE(handle == nullptr);
}

TEST_CASE(c_api_create_deep_copies_initialization_and_step_queries_buffers) {
  // 三个Tail常量：节点输入、定位输出和观测输出私有尾部的不同哨兵位型。
  constexpr std::uint64_t kNodeTail = UINT64_C(0x1122334455667788);
  constexpr std::uint64_t kLocalizationTail =
      UINT64_C(0x8877665544332211);
  constexpr std::uint64_t kObservationTail =
      UINT64_C(0xA5A55A5AF0F00F0F);
  // nodes/node：带扩展尾部的三节点输入及逐项引用；config/handle：自定义stride配置与创建结果。
  std::array<ExtendedNode, 3U> nodes{};
  for (auto& node : nodes) {
    EXPECT_EQ(zju_coop_node_initialization_init(&node.value), ZJU_COOP_OK);
    node.value.struct_size = sizeof(node);
    node.value.position_std_m = 0.1;
    node.value.velocity_std_mps = 0.1;
    node.tail = kNodeTail;
  }
  nodes[0U].value.node_id = 1U;
  nodes[1U].value.node_id = 2U;
  nodes[1U].value.x = 3.0;
  nodes[2U].value.node_id = 3U;
  nodes[2U].value.y = 4.0;

  zju_coop_config_t config{};
  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  config.reference_node_id = 1U;
  config.nodes = reinterpret_cast<const zju_coop_node_initialization_t*>(
      nodes.data());
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  config.node_stride = sizeof(ExtendedNode);

  zju_coop_handle_t* handle{};
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_OK);
  nodes[1U].value.x = 3000.0;
  for (const auto& node : nodes) {
    EXPECT_EQ(node.tail, kNodeTail);
  }

  // localization_count/observation_count：先接收容量查询所需数量，随后作为实际写入计数。
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  EXPECT_EQ(zju_coop_step(handle, 0U, nullptr, 0U, 0U,
                          &localization_count, nullptr, 0U, 0U,
                          &observation_count, nullptr),
            ZJU_COOP_BUFFER_TOO_SMALL);
  EXPECT_EQ(localization_count, 3U);
  EXPECT_EQ(observation_count, 3U);

  // localizations/observations/network：带私有尾部的输出缓冲和网络输出；循环引用只初始化各元素。
  std::vector<ExtendedLocalization> localizations(localization_count);
  std::vector<ExtendedObservation> observations(observation_count);
  zju_coop_network_t network{};
  for (auto& localization : localizations) {
    EXPECT_EQ(zju_coop_localization_init(&localization.value), ZJU_COOP_OK);
    localization.value.struct_size = sizeof(localization);
    localization.tail = kLocalizationTail;
  }
  for (auto& observation : observations) {
    EXPECT_EQ(zju_coop_observation_init(&observation.value), ZJU_COOP_OK);
    observation.value.struct_size = sizeof(observation);
    observation.tail = kObservationTail;
  }
  EXPECT_EQ(zju_coop_network_init(&network), ZJU_COOP_OK);
  EXPECT_EQ(zju_coop_step(
                handle, 0U,
                reinterpret_cast<zju_coop_localization_t*>(
                    localizations.data()),
                localization_count, sizeof(ExtendedLocalization),
                &localization_count,
                reinterpret_cast<zju_coop_observation_t*>(
                    observations.data()),
                observation_count, sizeof(ExtendedObservation),
                &observation_count, &network),
            ZJU_COOP_OK);

  // second：定位输出中节点2的迭代器；lambda参数value仅在单次谓词调用中借用当前扩展元素。
  const auto second = std::find_if(
      localizations.begin(), localizations.end(),
      [](const ExtendedLocalization& value) {
        return value.value.node_id == 2U;
      });
  EXPECT_TRUE(second != localizations.end());
  EXPECT_TRUE(std::abs(second->value.x - 3.0) < 1.0e-12);
  EXPECT_EQ(second->value.reference_node_id, 1U);
  EXPECT_EQ(second->value.yaw_valid, ZJU_COOP_FALSE);
  EXPECT_EQ(second->value.z_valid, ZJU_COOP_FALSE);
  EXPECT_EQ(observations[1U].value.from_node, 1U);
  EXPECT_EQ(observations[1U].value.to_node, 3U);
  for (const auto& localization : localizations) {
    EXPECT_EQ(localization.tail, kLocalizationTail);
  }
  for (const auto& observation : observations) {
    EXPECT_EQ(observation.tail, kObservationTail);
  }
  EXPECT_EQ(network.node_count, 3U);
  EXPECT_EQ(zju_coop_destroy(handle), ZJU_COOP_OK);
}

TEST_CASE(c_api_rejects_invalid_node_and_output_strides_without_dereference) {
  // nodes/node/config/handle：标准三节点输入、逐项初始化引用、待变步长配置及创建输出。
  std::array<zju_coop_node_initialization_t, 3U> nodes{};
  for (auto& node : nodes) {
    EXPECT_EQ(zju_coop_node_initialization_init(&node), ZJU_COOP_OK);
    node.position_std_m = 0.1;
    node.velocity_std_mps = 0.1;
  }
  nodes[0U].node_id = 1U;
  nodes[1U].node_id = 2U;
  nodes[1U].x = 3.0;
  nodes[2U].node_id = 3U;
  nodes[2U].y = 4.0;

  zju_coop_config_t config{};
  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  config.nodes = nodes.data();
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  zju_coop_handle_t* handle{};

  config.node_stride = sizeof(zju_coop_node_initialization_t) - 1U;
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_TRUE(handle == nullptr);

  config.node_stride = sizeof(zju_coop_node_initialization_t);
  nodes[1U].struct_size = config.node_stride + 1U;
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_TRUE(handle == nullptr);
  nodes[1U].struct_size = sizeof(nodes[1U]);

  // maximum_address/invalid_address：对齐到结构要求的近地址空间末端伪指针，验证跨度溢出检查先于解引用。
  const auto maximum_address = std::numeric_limits<std::uintptr_t>::max();
  const auto invalid_address =
      maximum_address -
      maximum_address % alignof(zju_coop_node_initialization_t);
  config.nodes = reinterpret_cast<const zju_coop_node_initialization_t*>(
      invalid_address);
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_TRUE(handle == nullptr);

  // engine：有效句柄对照；localizations/observations/network：标准输出缓冲；两个count预置哨兵确认失败不改写。
  TestEngine engine;
  std::vector<zju_coop_localization_t> localizations(3U);
  std::vector<zju_coop_observation_t> observations(3U);
  zju_coop_network_t network{};
  initialize_outputs(localizations, observations, network);
  std::uint32_t localization_count{71U};
  std::uint32_t observation_count{72U};

  EXPECT_EQ(zju_coop_step(
                engine.handle, 0U, localizations.data(), 3U,
                sizeof(zju_coop_localization_t) - 1U, &localization_count,
                observations.data(), 3U, kObservationStride,
                &observation_count, &network),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(localization_count, 71U);
  EXPECT_EQ(observation_count, 72U);

  localizations[1U].struct_size = kLocalizationStride + 1U;
  localization_count = 73U;
  observation_count = 74U;
  EXPECT_EQ(zju_coop_step(
                engine.handle, 0U, localizations.data(), 3U,
                kLocalizationStride, &localization_count,
                observations.data(), 3U, kObservationStride,
                &observation_count, &network),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(localization_count, 73U);
  EXPECT_EQ(observation_count, 74U);
  localizations[1U].struct_size = sizeof(localizations[1U]);

  // invalid_output：同一近地址末端伪指针，用于验证输出跨度溢出拒绝。
  auto* invalid_output = reinterpret_cast<zju_coop_localization_t*>(
      invalid_address);
  localization_count = 75U;
  observation_count = 76U;
  EXPECT_EQ(zju_coop_step(
                engine.handle, 0U, invalid_output, 3U,
                kLocalizationStride, &localization_count,
                observations.data(), 3U, kObservationStride,
                &observation_count, &network),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(localization_count, 75U);
  EXPECT_EQ(observation_count, 76U);
}

TEST_CASE(c_api_push_range_returns_processing_and_filter_diagnostics) {
  // engine/packet/result：有效测试句柄、序号17测距包及已初始化回执，期望返回完整处理和滤波诊断。
  TestEngine engine;
  auto packet = range_packet(1U, 2U, 3.0, kTimestampNs);
  packet.sequence = 17U;
  auto result = initialized_push_result();
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_OK);
  EXPECT_EQ(result.from_node, 1U);
  EXPECT_EQ(result.to_node, 2U);
  EXPECT_EQ(result.disposition, ZJU_COOP_PROCESSING_PROCESSED);
  EXPECT_EQ(result.fusion_action, ZJU_COOP_FUSION_USE_NORMAL);
  EXPECT_EQ(result.filter_updated, ZJU_COOP_TRUE);
  EXPECT_EQ(result.update_disposition, ZJU_COOP_UPDATE_ACCEPTED);
  EXPECT_TRUE(std::isfinite(result.innovation_m));
  EXPECT_TRUE(std::isfinite(result.innovation_variance));
  EXPECT_TRUE(std::isfinite(result.nis));
  EXPECT_EQ(result.covariance_scale, 1.0);
}

TEST_CASE(c_api_validates_range_headers_flags_status_and_finite_values) {
  // engine/packet/result：复用于空指针、坏结构头、非法布尔/状态和非有限字段的输入与输出缓冲。
  TestEngine engine;
  auto packet = range_packet(1U, 2U, 3.0, kTimestampNs);
  auto result = initialized_push_result();

  EXPECT_EQ(zju_coop_push_range(nullptr, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(zju_coop_push_range(engine.handle, nullptr, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, nullptr),
            ZJU_COOP_INVALID_ARGUMENT);

  packet.struct_size = sizeof(packet) - 1U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_STRUCT_SIZE_MISMATCH);
  packet.struct_size = sizeof(packet);
  packet.abi_version = 0U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_ABI_MISMATCH);
  packet.abi_version = ZJU_COOP_ABI_VERSION_V1;

  packet.nlos_flag = 2U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  packet.nlos_flag = ZJU_COOP_FALSE;
  packet.has_nlos_probability = 2U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  packet.has_nlos_probability = ZJU_COOP_FALSE;
  packet.valid = 2U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  packet.valid = ZJU_COOP_TRUE;
  packet.status = 255U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  packet.status = ZJU_COOP_RANGE_STATUS_OK;
  packet.range_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);
  packet.range_m = 3.0;
  packet.nlos_probability = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_INVALID_ARGUMENT);

  packet.nlos_probability = 0.0F;
  result.abi_version = 0U;
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_ABI_MISMATCH);
}

// 输出事务组验证容量查询不推进时间，坏缓冲不部分写入，Engine失败也不提交候选状态。
TEST_CASE(c_api_step_never_partially_writes_when_capacity_or_headers_fail) {
  // engine：有效句柄；localizations/observations/network：预置可辨内容的输出；两个count接收需求或保留哨兵。
  TestEngine engine;
  std::vector<zju_coop_localization_t> localizations(2U);
  std::vector<zju_coop_observation_t> observations(3U);
  zju_coop_network_t network{};
  initialize_outputs(localizations, observations, network);
  localizations[0U].x = 12345.0;
  observations[0U].nlos_ratio = 0.75;
  network.timestamp_ns = 999U;

  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  EXPECT_EQ(zju_coop_step(engine.handle, 0U, localizations.data(), 2U,
                          kLocalizationStride, &localization_count,
                          observations.data(), 3U, kObservationStride,
                          &observation_count, &network),
            ZJU_COOP_BUFFER_TOO_SMALL);
  EXPECT_EQ(localization_count, 3U);
  EXPECT_EQ(observation_count, 3U);
  EXPECT_EQ(localizations[0U].x, 12345.0);
  EXPECT_EQ(observations[0U].nlos_ratio, 0.75);
  EXPECT_EQ(network.timestamp_ns, 999U);

  localizations.resize(3U);
  EXPECT_EQ(zju_coop_localization_init(&localizations[2U]), ZJU_COOP_OK);
  localizations[1U].abi_version = 0U;
  localization_count = 77U;
  observation_count = 88U;
  EXPECT_EQ(zju_coop_step(engine.handle, 0U, localizations.data(), 3U,
                          kLocalizationStride, &localization_count,
                          observations.data(), 3U, kObservationStride,
                          &observation_count, &network),
            ZJU_COOP_ABI_MISMATCH);
  EXPECT_EQ(localizations[0U].x, 12345.0);
  EXPECT_EQ(network.timestamp_ns, 999U);
  EXPECT_EQ(localization_count, 77U);
  EXPECT_EQ(observation_count, 88U);

  localizations[1U].abi_version = ZJU_COOP_ABI_VERSION_V1;
  localization_count = 91U;
  observation_count = 92U;
  EXPECT_EQ(zju_coop_step(engine.handle, 0U, localizations.data(), 3U,
                          kLocalizationStride, &localization_count,
                          observations.data(), 3U, kObservationStride,
                          &observation_count, nullptr),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(localizations[0U].x, 12345.0);
  EXPECT_EQ(observations[0U].nlos_ratio, 0.75);
  EXPECT_EQ(localization_count, 91U);
  EXPECT_EQ(observation_count, 92U);
}

TEST_CASE(c_api_failed_engine_step_is_transactional_for_outputs_and_handle) {
  // nodes/node/config/handle：极大过程噪声的三节点引擎输入、逐项引用、配置和句柄。
  std::array<zju_coop_node_initialization_t, 3U> nodes{};
  for (auto& node : nodes) {
    EXPECT_EQ(zju_coop_node_initialization_init(&node), ZJU_COOP_OK);
    node.position_std_m = 0.1;
    node.velocity_std_mps = 0.1;
  }
  nodes[0U].node_id = 1U;
  nodes[1U].node_id = 2U;
  nodes[1U].x = 3.0;
  nodes[2U].node_id = 3U;
  nodes[2U].y = 4.0;

  zju_coop_config_t config{};
  EXPECT_EQ(zju_coop_config_init(&config), ZJU_COOP_OK);
  config.nodes = nodes.data();
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  config.process_accel_std_mps2 = 1.0e150;
  zju_coop_handle_t* handle{};
  EXPECT_EQ(zju_coop_create(&config, &handle), ZJU_COOP_OK);

  // localizations/observations/network及两个count：失败前写入哨兵的输出事务缓冲。
  std::vector<zju_coop_localization_t> localizations(3U);
  std::vector<zju_coop_observation_t> observations(3U);
  zju_coop_network_t network{};
  initialize_outputs(localizations, observations, network);
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  EXPECT_EQ(zju_coop_step(handle, 0U, localizations.data(), 3U,
                          kLocalizationStride, &localization_count,
                          observations.data(), 3U, kObservationStride,
                          &observation_count, &network),
            ZJU_COOP_OK);

  localizations[0U].x = 12345.0;
  observations[0U].nlos_ratio = 0.75;
  network.timestamp_ns = 999U;
  localization_count = 81U;
  observation_count = 82U;
  EXPECT_EQ(zju_coop_step(
                handle, std::numeric_limits<std::uint64_t>::max(),
                localizations.data(), 3U, kLocalizationStride,
                &localization_count, observations.data(), 3U,
                kObservationStride, &observation_count, &network),
            ZJU_COOP_INTERNAL_ERROR);
  EXPECT_EQ(localizations[0U].x, 12345.0);
  EXPECT_EQ(observations[0U].nlos_ratio, 0.75);
  EXPECT_EQ(network.timestamp_ns, 999U);
  EXPECT_EQ(localization_count, 81U);
  EXPECT_EQ(observation_count, 82U);

  EXPECT_EQ(zju_coop_step(handle, 1U, localizations.data(), 3U,
                          kLocalizationStride, &localization_count,
                          observations.data(), 3U, kObservationStride,
                          &observation_count, &network),
            ZJU_COOP_OK);
  EXPECT_EQ(network.timestamp_ns, 1U);
  EXPECT_EQ(zju_coop_destroy(handle), ZJU_COOP_OK);
}

TEST_CASE(c_api_step_validates_null_contract_and_destroy_rejects_null) {
  // engine：有效句柄对照；localization_count/observation_count：验证空契约时的计数指针组合。
  TestEngine engine;
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  EXPECT_EQ(zju_coop_step(nullptr, 0U, nullptr, 0U, 0U,
                          &localization_count, nullptr, 0U, 0U,
                          &observation_count, nullptr),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(zju_coop_step(engine.handle, 0U, nullptr, 0U, 0U, nullptr,
                          nullptr, 0U, 0U, &observation_count, nullptr),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(zju_coop_step(engine.handle, 0U, nullptr, 1U,
                          kLocalizationStride, &localization_count, nullptr,
                          0U, 0U, &observation_count, nullptr),
            ZJU_COOP_INVALID_ARGUMENT);
  EXPECT_EQ(zju_coop_destroy(nullptr), ZJU_COOP_INVALID_ARGUMENT);
}

TEST_CASE(c_api_buffer_query_does_not_advance_engine_timebase) {
  // engine与两个count执行仅查询容量；packet/result随后验证查询时间不曾提交到引擎。
  TestEngine engine;
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  EXPECT_EQ(zju_coop_step(engine.handle, 5'000'000'000ULL, nullptr, 0U, 0U,
                          &localization_count, nullptr, 0U, 0U,
                          &observation_count, nullptr),
            ZJU_COOP_BUFFER_TOO_SMALL);

  auto packet = range_packet(1U, 2U, 3.0, kTimestampNs);
  auto result = initialized_push_result();
  EXPECT_EQ(zju_coop_push_range(engine.handle, &packet, &result),
            ZJU_COOP_OK);
  EXPECT_EQ(result.disposition, ZJU_COOP_PROCESSING_PROCESSED);
}

// 惯性集成组证明标准ROS字段可映射为普通C结构，核心库本身不链接ROS 2。
TEST_CASE(c_api_configures_and_pushes_standard_imu_without_ros_dependency) {
  // engine：有效C句柄；nodes/index：三节点惯导初值及逐项编号索引；config：仅在配置调用期间借用节点数组。
  TestEngine engine;
  std::array<zju_coop_inertial_node_initialization_t, 3U> nodes{};
  for (std::size_t index = 0U; index < nodes.size(); ++index) {
    EXPECT_EQ(zju_coop_inertial_node_initialization_init(&nodes[index]),
              ZJU_COOP_OK);
    nodes[index].node_id = static_cast<std::uint32_t>(index + 1U);
  }
  nodes[1U].position_n_m[0] = 3.0;
  nodes[2U].position_n_m[1] = 4.0;

  zju_coop_inertial_config_t config{};
  EXPECT_EQ(zju_coop_inertial_config_init(&config), ZJU_COOP_OK);
  config.nodes = nodes.data();
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  EXPECT_EQ(zju_coop_configure_inertial(engine.handle, &config), ZJU_COOP_OK);

  // packet：节点2的标准ROS字段布局静止IMU首帧；result：已初始化的处理输出，期望只建立时基。
  zju_coop_imu_packet_t packet{};
  EXPECT_EQ(zju_coop_imu_packet_init(&packet), ZJU_COOP_OK);
  packet.node_id = 2U;
  packet.sequence = 1U;
  packet.timestamp_ns = kTimestampNs;
  packet.receive_timestamp_ns = kTimestampNs;
  packet.linear_acceleration_m_s2[2] = 9.80665;
  std::memcpy(packet.frame_id, "imu_link", 9U);
  packet.valid = ZJU_COOP_TRUE;
  zju_coop_imu_processing_result_t result{};
  EXPECT_EQ(zju_coop_imu_processing_result_init(&result), ZJU_COOP_OK);

  EXPECT_EQ(zju_coop_push_imu(engine.handle, &packet, &result), ZJU_COOP_OK);
  EXPECT_EQ(result.disposition, ZJU_COOP_IMU_BASELINE_ESTABLISHED);
  EXPECT_EQ(result.propagated, ZJU_COOP_FALSE);
}

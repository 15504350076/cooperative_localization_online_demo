// 模块实现：用三车静止IMU和3-4-5测距从公开C ABI完成创建、传播、引擎级Processed路径、
// 输出与恢复验证；测距项不单独断言底层update_disposition为ACCEPTED。
// 所有输入均为内存构造的确定性数据，不打开端口和硬件，因此可与后续ROS 2实机节点并存。
//
// C++初学者阅读顺序：
// 1. Recorder把每个布尔判断记录为PASS/FAIL；HandleGuard保证提前返回时也能destroy句柄。
// 2. 构造三车初值和惯性配置，创建公开C ABI会话。
// 3. 为每辆车输入两帧静止IMU：第一帧建立时间基线，第二帧真正执行预测。
// 4. 输入3 m、4 m、5 m三条测距，随后step检查三车输出、网络状态和恢复行为。
// 5. 所有数据固定，因而同一程序在Windows、Ubuntu和RK3588应得到可重复结果。
// HandleGuard使用RAII，`= delete`禁止复制句柄所有权；ostringstream把多次`<<`输出累计成字符串；
// 所有C结构先调用init，避免初学者把零初始化误当作完整ABI版本握手。
#include "self_check/self_check.hpp"

#include "zju_coop/c_api.h"

#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace zju::coop::self_check {
namespace {

constexpr std::uint64_t kInitialTimeNs = 1'000'000'000ULL;  // 三车首批IMU共享的确定性统一时间。
constexpr std::uint64_t kImuIntervalNs = 10'000'000ULL;     // 第二批IMU与恢复测距使用的10 ms步长。
constexpr double kPositionToleranceM = 1.0e-5;              // 3-4-5几何位置比较的默认绝对容差。

class Recorder {
 public:
  /**
   * @param condition 当前检查是否通过；@param name 报告中的稳定检查项名称。
   * @param detail 失败时附加的API返回或数值上下文。
   */
  void check(bool condition, const char* name,
             const std::string& detail = {}) {
    if (condition) {
      ++passed_;
      report_ << "[PASS] " << name << '\n';
      return;
    }
    ++failed_;
    report_ << "[FAIL] " << name;
    if (!detail.empty()) {
      report_ << ": " << detail;
    }
    report_ << '\n';
  }

  [[nodiscard]] SelfCheckResult result() const {
    return {failed_ == 0U, passed_, failed_, report_.str()};
  }

 private:
  std::uint32_t passed_{};   ///< 已记录为通过的检查项数。
  std::uint32_t failed_{};   ///< 已记录为失败的检查项数。
  std::ostringstream report_;  ///< 按执行顺序累积PASS/FAIL文本。
};

class HandleGuard {
 public:
  // 任何中途失败或异常都销毁临时会话；自检不会遗留可被在线程序复用的状态。
  ~HandleGuard() {
    if (value != nullptr) {
      static_cast<void>(zju_coop_destroy(value));
    }
  }

  /** 未命名源守卫参数刻意禁用临时句柄所有权复制。 */
  HandleGuard(const HandleGuard&) = delete;
  /** 未命名源守卫参数刻意禁用临时句柄所有权复制赋值。 */
  HandleGuard& operator=(const HandleGuard&) = delete;
  HandleGuard() = default;

  zju_coop_handle_t* value{};  ///< 本守卫独占、析构时兜底销毁的C ABI句柄。
};

struct Snapshot {
  std::vector<zju_coop_localization_t> localizations;  ///< 两阶段step读出的三车定位数组。
  std::vector<zju_coop_observation_t> observations;    ///< 两阶段step读出的三条边质量数组。
  zju_coop_network_t network{};                        ///< 与两数组同次生成的网络状态。
};

/** @param code 待转换为稳定诊断文本的C ABI错误码。 */
[[nodiscard]] std::string api_error(zju_coop_error_code_t code) {
  const char* text = zju_coop_error_string(code);  // C ABI返回的静态错误字符串或空指针。
  return text == nullptr ? std::to_string(code) : text;
}

/**
 * @param lhs 实际数值；@param rhs 期望数值；@param tolerance 允许的绝对误差。
 */
[[nodiscard]] bool approximately(double lhs, double rhs,
                                 double tolerance = kPositionToleranceM) {
  return std::isfinite(lhs) && std::abs(lhs - rhs) <= tolerance;
}

/** @param snapshot 待查找的定位快照；@param node_id 目标节点编号。 */
[[nodiscard]] const zju_coop_localization_t* find_localization(
    const Snapshot& snapshot, std::uint32_t node_id) {
  for (const auto& value : snapshot.localizations) {  // value为逐项匹配编号的定位输出。
    if (value.node_id == node_id) {
      return &value;
    }
  }
  return nullptr;
}

/**
 * @param handle 待执行step的有效C ABI会话；@param now_ns 本次统一输出时间。
 * @param snapshot 先按查询数量扩容、再写入结果的调用方缓冲。
 * @param detail 失败时回填的诊断文本。
 */
[[nodiscard]] bool take_snapshot(zju_coop_handle_t* handle,
                                 std::uint64_t now_ns, Snapshot& snapshot,
                                 std::string& detail) {
  // C ABI快照采用“两次调用”：先查询数量，再初始化调用方缓冲并读取原子结果。
  std::uint32_t localization_count{};  // 第一次查询、第二次确认的定位元素数。
  std::uint32_t observation_count{};   // 第一次查询、第二次确认的观测元素数。
  auto code = zju_coop_step(handle, now_ns, nullptr, 0U, 0U,  // 两阶段step当前返回码。
                            &localization_count, nullptr, 0U, 0U,
                            &observation_count, nullptr);
  if (code != ZJU_COOP_BUFFER_TOO_SMALL) {
    detail = "size query returned " + api_error(code);
    return false;
  }

  snapshot.localizations.resize(localization_count);
  snapshot.observations.resize(observation_count);
  // 两个value分别是待设置版本握手字段的定位缓冲槽和观测缓冲槽。
  for (auto& value : snapshot.localizations) {
    if (zju_coop_localization_init(&value) != ZJU_COOP_OK) {
      detail = "localization initialization failed";
      return false;
    }
  }
  for (auto& value : snapshot.observations) {
    if (zju_coop_observation_init(&value) != ZJU_COOP_OK) {
      detail = "observation initialization failed";
      return false;
    }
  }
  if (zju_coop_network_init(&snapshot.network) != ZJU_COOP_OK) {
    detail = "network initialization failed";
    return false;
  }

  code = zju_coop_step(
      handle, now_ns, snapshot.localizations.data(), localization_count,
      sizeof(zju_coop_localization_t), &localization_count,
      snapshot.observations.data(), observation_count,
      sizeof(zju_coop_observation_t), &observation_count, &snapshot.network);
  if (code != ZJU_COOP_OK) {
    detail = "step returned " + api_error(code);
    return false;
  }
  snapshot.localizations.resize(localization_count);
  snapshot.observations.resize(observation_count);
  return true;
}

/**
 * @param from_node 3-4-5三角形边的发送端；@param to_node 该边接收端。
 * @param range_m 两端确定性欧氏距离；@param sequence 该有向链路的发送序号。
 * @param timestamp_ns 同时写入测量和接收字段的统一时间。
 */
[[nodiscard]] zju_coop_range_packet_t make_range(
    std::uint16_t from_node, std::uint16_t to_node, double range_m,
    std::uint64_t sequence, std::uint64_t timestamp_ns) {
  zju_coop_range_packet_t packet{};  // 已初始化并填入一条三角形边的确定性测距包。
  static_cast<void>(zju_coop_range_packet_init(&packet));
  packet.from_node = from_node;
  packet.to_node = to_node;
  packet.sequence = sequence;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns;
  packet.range_m = range_m;
  packet.range_std_m = 0.05;
  packet.valid = ZJU_COOP_TRUE;
  packet.status = ZJU_COOP_RANGE_STATUS_OK;
  return packet;
}

/**
 * @param node_id 三辆自检车之一的节点编号；@param sequence 该节点IMU发送序号。
 * @param timestamp_ns 同时写入测量和接收字段的统一时间。
 */
[[nodiscard]] zju_coop_imu_packet_t make_stationary_imu(
    std::uint32_t node_id, std::uint64_t sequence,
    std::uint64_t timestamp_ns) {
  zju_coop_imu_packet_t packet{};  // 已初始化并填入静止比力的确定性IMU包。
  static_cast<void>(zju_coop_imu_packet_init(&packet));
  packet.node_id = node_id;
  packet.sequence = sequence;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns;
  // ENU/FLU 下静止 IMU 的比力为 +g；惯导传播会扣除重力。
  packet.linear_acceleration_m_s2[2] = 9.80665;
  std::memcpy(packet.frame_id, "imu_link", 9U);
  packet.valid = ZJU_COOP_TRUE;
  return packet;
}

/**
 * @param handle 接收三条边输入的有效C ABI会话；@param packets 3-4-5三角形整批测距。
 * @param expected_disposition 每条边均应返回的引擎处置。
 * @param detail 首条失败边及API结果的诊断输出。
 */
[[nodiscard]] bool push_ranges(
    zju_coop_handle_t* handle,
    const std::array<zju_coop_range_packet_t, 3U>& packets,
    zju_coop_processing_disposition_t expected_disposition,
    std::string& detail) {
  for (const auto& packet : packets) {  // packet为本批待按顺序推送的一条三角形边。
    zju_coop_range_processing_result_t result{};  // C ABI回填的单边处理结果。
    auto code =  // 结果初始化及随后push_range共用的返回码。
        zju_coop_range_processing_result_init(&result);
    if (code == ZJU_COOP_OK) {
      code = zju_coop_push_range(handle, &packet, &result);
    }
    if (code != ZJU_COOP_OK || result.disposition != expected_disposition) {
      detail = "edge " + std::to_string(packet.from_node) + "-" +
               std::to_string(packet.to_node) + " returned " +
               api_error(code) + ", disposition=" +
               std::to_string(result.disposition);
      return false;
    }
  }
  return true;
}

}  // namespace

SelfCheckResult run_self_check() noexcept {
  Recorder recorder;   // 即使中途返回也保留已执行检查的计数和报告。
  HandleGuard handle;  // 任意异常或早退时销毁临时C ABI会话的RAII守卫。

  try {
    // 阶段1：验证ABI版本、初始化器、会话创建及资源所有权。
    const char* version = zju_coop_version_string();  // 公开ABI报告的静态版本字符串。
    recorder.check(zju_coop_abi_version() == ZJU_COOP_ABI_VERSION_V1 &&
                       version != nullptr && version[0] != '\0',
                   "abi_version", "public ABI/version query failed");

    // nodes保存三车二维初值；initialized合并初始化器结果；index映射连续节点编号1..3。
    std::array<zju_coop_node_initialization_t, 3U> nodes{};
    bool initialized = true;
    for (std::size_t index = 0U; index < nodes.size(); ++index) {
      initialized =
          initialized &&
          zju_coop_node_initialization_init(&nodes[index]) == ZJU_COOP_OK;
      nodes[index].node_id = static_cast<std::uint32_t>(index + 1U);
      nodes[index].position_std_m = 0.1;
      nodes[index].velocity_std_mps = 0.1;
    }
    nodes[1].x = 3.0;
    nodes[2].y = 4.0;

    zju_coop_config_t config{};  // 创建三车会话的版本化基础配置。
    initialized = initialized &&
                  zju_coop_config_init(&config) == ZJU_COOP_OK;
    config.reference_node_id = 1U;
    config.nodes = nodes.data();
    config.node_count = static_cast<std::uint32_t>(nodes.size());
    // 自检测距与初值严格一致，放宽门限避免浮点微扰影响跨平台重复性。
    config.nis_gate = 1.0e9;
    const auto create_code =  // 仅在所有初始化器成功后调用create的结果。
        initialized
                                 ? zju_coop_create(&config, &handle.value)
                                 : ZJU_COOP_INTERNAL_ERROR;
    recorder.check(create_code == ZJU_COOP_OK && handle.value != nullptr,
                   "create_session", api_error(create_code));
    if (create_code != ZJU_COOP_OK || handle.value == nullptr) {
      return recorder.result();
    }

    // 阶段2：配置三节点15维惯性状态；位置组成3-4-5三角形以提供非退化几何。
    // inertial_nodes保存三车15维初值；inertial_initialized合并初始化器结果；
    // index映射连续惯性节点编号1..3。
    std::array<zju_coop_inertial_node_initialization_t, 3U>
        inertial_nodes{};
    bool inertial_initialized = true;
    for (std::size_t index = 0U; index < inertial_nodes.size(); ++index) {
      inertial_initialized =
          inertial_initialized &&
          zju_coop_inertial_node_initialization_init(
              &inertial_nodes[index]) == ZJU_COOP_OK;
      inertial_nodes[index].node_id =
          static_cast<std::uint32_t>(index + 1U);
    }
    inertial_nodes[1].position_n_m[0] = 3.0;
    inertial_nodes[2].position_n_m[1] = 4.0;

    zju_coop_inertial_config_t inertial_config{};  // 首个输入前启用15维路径的版本化配置。
    inertial_initialized =
        inertial_initialized &&
        zju_coop_inertial_config_init(&inertial_config) == ZJU_COOP_OK;
    inertial_config.nodes = inertial_nodes.data();
    inertial_config.node_count =
        static_cast<std::uint32_t>(inertial_nodes.size());
    const auto configure_code =  // 仅在惯性初始化完整时调用configure的结果。
        inertial_initialized
            ? zju_coop_configure_inertial(handle.value, &inertial_config)
            : ZJU_COOP_INTERNAL_ERROR;
    recorder.check(configure_code == ZJU_COOP_OK, "configure_inertial",
                   api_error(configure_code));
    if (configure_code != ZJU_COOP_OK) {
      return recorder.result();
    }

    // 阶段3：第一批静止IMU建立各节点时间基准，第二批完成真实10 ms传播。
    bool baselines_ok = true;  // 三车首包均只建立时间基准的合并结论。
    for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {  // node_id依次覆盖三辆自检车。
      // packet是当前车基准IMU包；result接收处置；code串联初始化与push返回码。
      auto packet = make_stationary_imu(node_id, 1U, kInitialTimeNs);
      zju_coop_imu_processing_result_t result{};
      auto code =
          zju_coop_imu_processing_result_init(&result);
      if (code == ZJU_COOP_OK) {
        code = zju_coop_push_imu(handle.value, &packet, &result);
      }
      baselines_ok = baselines_ok && code == ZJU_COOP_OK &&
                     result.disposition == ZJU_COOP_IMU_BASELINE_ESTABLISHED &&
                     result.propagated == ZJU_COOP_FALSE;
    }
    recorder.check(baselines_ok, "imu_baseline");

    const auto propagated_time_ns =  // 三车第二批IMU及首批测距共享的统一时间。
        kInitialTimeNs + kImuIntervalNs;
    bool propagation_ok = true;  // 三车第二包均完成10 ms传播的合并结论。
    for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {  // node_id依次覆盖三辆自检车。
      // packet是当前车传播IMU包；result接收处置；code串联初始化与push返回码。
      auto packet = make_stationary_imu(node_id, 2U, propagated_time_ns);
      zju_coop_imu_processing_result_t result{};
      auto code =
          zju_coop_imu_processing_result_init(&result);
      if (code == ZJU_COOP_OK) {
        code = zju_coop_push_imu(handle.value, &packet, &result);
      }
      propagation_ok = propagation_ok && code == ZJU_COOP_OK &&
                       result.disposition == ZJU_COOP_IMU_PROPAGATED &&
                       result.propagated == ZJU_COOP_TRUE &&
                       approximately(result.dt_s, 0.01, 1.0e-12);
    }
    recorder.check(propagation_ok, "imu_propagation");

    // 阶段4：输入三条一致测距，验证引擎级Processed处理路径、动态拓扑和主参考相对输出；
    // 这里不单独断言底层update_disposition为ACCEPTED。
    const std::array<zju_coop_range_packet_t, 3U> nominal_ranges{  // 边1-2=3 m、1-3=4 m、2-3=5 m的首批约束。
        make_range(1U, 2U, 3.0, 1U, propagated_time_ns),
        make_range(1U, 3U, 4.0, 1U, propagated_time_ns),
        make_range(2U, 3U, 5.0, 1U, propagated_time_ns)};
    std::string detail;  // 各后续检查复用的首个失败诊断文本。
    const bool ranges_ok =  // 三条名义边均由引擎处置为Processed。
        push_ranges(handle.value, nominal_ranges,
                    ZJU_COOP_PROCESSING_PROCESSED, detail);
    recorder.check(ranges_ok, "range_updates", detail);

    Snapshot nominal;  // 首批IMU和3-4-5测距后的两阶段输出缓冲。
    detail.clear();
    const bool nominal_step_ok =  // 名义统一时刻的两阶段step是否成功。
        take_snapshot(handle.value, propagated_time_ns, nominal, detail);
    const auto* node1 = find_localization(nominal, 1U);  // 主参考节点1的定位输出或空。
    const auto* node2 = find_localization(nominal, 2U);  // 3 m横向节点2的定位输出或空。
    const auto* node3 = find_localization(nominal, 3U);  // 4 m纵向节点3的定位输出或空。
    const bool nominal_geometry_ok =  // 输出数量及三车相对位置是否保持3-4-5初始几何。
        nominal_step_ok && nominal.localizations.size() == 3U &&
        nominal.observations.size() == 3U && node1 != nullptr &&
        node2 != nullptr && node3 != nullptr &&
        approximately(node1->x, 0.0) && approximately(node1->y, 0.0) &&
        approximately(node2->x, 3.0) && approximately(node2->y, 0.0) &&
        approximately(node3->x, 0.0) && approximately(node3->y, 4.0);
    recorder.check(nominal_geometry_ok, "nominal_snapshot", detail);

    const bool topology_ok =  // 三节点三活动边是否连通且达到二维目标刚度秩。
        nominal_step_ok && nominal.network.node_count == 3U &&
        nominal.network.reachable_node_count == 3U &&
        nominal.network.active_edge_count == 3U &&
        nominal.network.connected == ZJU_COOP_TRUE &&
        nominal.network.observable == ZJU_COOP_TRUE;
    recorder.check(topology_ok, "network_topology");

    // 数值位置正确仍不够：能力有效位是给GCS/实车消费者的安全契约，必须单独验。
    const bool output_flags_ok =  // 三车参考编号及未提供yaw/z的能力标志是否一致。
        node1 != nullptr && node2 != nullptr && node3 != nullptr &&
        node1->reference_node_id == 1U && node2->reference_node_id == 1U &&
        node3->reference_node_id == 1U && node1->yaw_valid == ZJU_COOP_FALSE &&
        node2->yaw_valid == ZJU_COOP_FALSE &&
        node3->yaw_valid == ZJU_COOP_FALSE &&
        node1->z_valid == ZJU_COOP_FALSE && node2->z_valid == ZJU_COOP_FALSE &&
        node3->z_valid == ZJU_COOP_FALSE;
    recorder.check(output_flags_ok, "output_validity_flags");

    // 阶段5：重复包、链路超时和新量测恢复覆盖在线状态机的关键异常路径。
    zju_coop_range_processing_result_t duplicate_result{};  // 重推首条边时C ABI回填的结果。
    auto duplicate_code =  // 结果初始化及随后重复包push共用的返回码。
        zju_coop_range_processing_result_init(&duplicate_result);
    if (duplicate_code == ZJU_COOP_OK) {
      duplicate_code = zju_coop_push_range(handle.value, &nominal_ranges[0],
                                           &duplicate_result);
    }
    recorder.check(duplicate_code == ZJU_COOP_OK &&
                       duplicate_result.disposition ==
                           ZJU_COOP_PROCESSING_DUPLICATE,
                   "duplicate_rejection", api_error(duplicate_code));

    const auto timeout_time_ns =  // 严格越过边有效期1 ns的统一step时间。
        propagated_time_ns + config.edge_timeout_ns + 1U;
    Snapshot timed_out;  // 无新测距越过超时门限后的两阶段输出缓冲。
    detail.clear();
    const bool timeout_step_ok =  // 超时时刻的两阶段step是否成功。
        take_snapshot(handle.value, timeout_time_ns, timed_out, detail);
    const bool timeout_ok =  // 三条边均失效且网络报告链路超时原因。
        timeout_step_ok && timed_out.network.active_edge_count == 0U &&
        timed_out.network.connected == ZJU_COOP_FALSE &&
        (timed_out.network.reason_mask & ZJU_COOP_REASON_LINK_TIMEOUT) != 0U;
    recorder.check(timeout_ok, "link_timeout", detail);

    const auto recovery_time_ns =  // 超时后注入新鲜测距并取恢复快照的统一时间。
        timeout_time_ns + kImuIntervalNs;
    const std::array<zju_coop_range_packet_t, 3U> recovery_ranges{  // 序号更新后的第二批3-4-5约束。
        make_range(1U, 2U, 3.0, 2U, recovery_time_ns),
        make_range(1U, 3U, 4.0, 2U, recovery_time_ns),
        make_range(2U, 3U, 5.0, 2U, recovery_time_ns)};
    detail.clear();
    const bool recovery_input_ok =  // 三条恢复边均由引擎处置为Processed。
        push_ranges(handle.value, recovery_ranges,
                    ZJU_COOP_PROCESSING_PROCESSED, detail);
    Snapshot recovered;  // 新鲜3-4-5测距后的两阶段恢复输出缓冲。
    const bool recovery_step_ok =  // 恢复输入和随后step均成功。
        recovery_input_ok &&
        take_snapshot(handle.value, recovery_time_ns, recovered, detail);
    const bool recovery_ok =  // 活动边、可达节点、连通性和可观性是否全部恢复。
        recovery_step_ok && recovered.network.active_edge_count == 3U &&
        recovered.network.reachable_node_count == 3U &&
        recovered.network.connected == ZJU_COOP_TRUE &&
        recovered.network.observable == ZJU_COOP_TRUE;
    recorder.check(recovery_ok, "network_recovery", detail);

    const auto destroy_code = zju_coop_destroy(handle.value);  // 显式销毁路径的C ABI返回码。
    handle.value = nullptr;
    recorder.check(destroy_code == ZJU_COOP_OK, "destroy_session",
                   api_error(destroy_code));
  } catch (const std::exception& error) {  // error为转换成失败检查的标准异常。
    recorder.check(false, "unexpected_exception", error.what());
  } catch (...) {
    recorder.check(false, "unexpected_exception", "unknown exception");
  }

  return recorder.result();
}

}  // namespace zju::coop::self_check

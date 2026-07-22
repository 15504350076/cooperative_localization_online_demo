// 模块实现：用三车静止IMU和3-4-5测距从公开C ABI完成创建、传播、更新、输出与恢复验证。
// 所有输入均为内存构造的确定性数据，不打开端口和硬件，因此可与后续ROS 2实机节点并存。
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

constexpr std::uint64_t kInitialTimeNs = 1'000'000'000ULL;
constexpr std::uint64_t kImuIntervalNs = 10'000'000ULL;
constexpr double kPositionToleranceM = 1.0e-5;

class Recorder {
 public:
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
  std::uint32_t passed_{};
  std::uint32_t failed_{};
  std::ostringstream report_;
};

class HandleGuard {
 public:
  // 任何中途失败或异常都销毁临时会话；自检不会遗留可被在线程序复用的状态。
  ~HandleGuard() {
    if (value != nullptr) {
      static_cast<void>(zju_coop_destroy(value));
    }
  }

  HandleGuard(const HandleGuard&) = delete;
  HandleGuard& operator=(const HandleGuard&) = delete;
  HandleGuard() = default;

  zju_coop_handle_t* value{};
};

struct Snapshot {
  std::vector<zju_coop_localization_t> localizations;
  std::vector<zju_coop_observation_t> observations;
  zju_coop_network_t network{};
};

[[nodiscard]] std::string api_error(zju_coop_error_code_t code) {
  const char* text = zju_coop_error_string(code);
  return text == nullptr ? std::to_string(code) : text;
}

[[nodiscard]] bool approximately(double lhs, double rhs,
                                 double tolerance = kPositionToleranceM) {
  return std::isfinite(lhs) && std::abs(lhs - rhs) <= tolerance;
}

[[nodiscard]] const zju_coop_localization_t* find_localization(
    const Snapshot& snapshot, std::uint32_t node_id) {
  for (const auto& value : snapshot.localizations) {
    if (value.node_id == node_id) {
      return &value;
    }
  }
  return nullptr;
}

[[nodiscard]] bool take_snapshot(zju_coop_handle_t* handle,
                                 std::uint64_t now_ns, Snapshot& snapshot,
                                 std::string& detail) {
  // C ABI快照采用“两次调用”：先查询数量，再初始化调用方缓冲并读取原子结果。
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  auto code = zju_coop_step(handle, now_ns, nullptr, 0U, 0U,
                            &localization_count, nullptr, 0U, 0U,
                            &observation_count, nullptr);
  if (code != ZJU_COOP_BUFFER_TOO_SMALL) {
    detail = "size query returned " + api_error(code);
    return false;
  }

  snapshot.localizations.resize(localization_count);
  snapshot.observations.resize(observation_count);
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

[[nodiscard]] zju_coop_range_packet_t make_range(
    std::uint16_t from_node, std::uint16_t to_node, double range_m,
    std::uint64_t sequence, std::uint64_t timestamp_ns) {
  zju_coop_range_packet_t packet{};
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

[[nodiscard]] zju_coop_imu_packet_t make_stationary_imu(
    std::uint32_t node_id, std::uint64_t sequence,
    std::uint64_t timestamp_ns) {
  zju_coop_imu_packet_t packet{};
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

[[nodiscard]] bool push_ranges(
    zju_coop_handle_t* handle,
    const std::array<zju_coop_range_packet_t, 3U>& packets,
    zju_coop_processing_disposition_t expected_disposition,
    std::string& detail) {
  for (const auto& packet : packets) {
    zju_coop_range_processing_result_t result{};
    auto code = zju_coop_range_processing_result_init(&result);
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
  Recorder recorder;
  HandleGuard handle;

  try {
    // 阶段1：验证ABI版本、初始化器、会话创建及资源所有权。
    const char* version = zju_coop_version_string();
    recorder.check(zju_coop_abi_version() == ZJU_COOP_ABI_VERSION_V1 &&
                       version != nullptr && version[0] != '\0',
                   "abi_version", "public ABI/version query failed");

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

    zju_coop_config_t config{};
    initialized = initialized &&
                  zju_coop_config_init(&config) == ZJU_COOP_OK;
    config.reference_node_id = 1U;
    config.nodes = nodes.data();
    config.node_count = static_cast<std::uint32_t>(nodes.size());
    // 自检测距与初值严格一致，放宽门限避免浮点微扰影响跨平台重复性。
    config.nis_gate = 1.0e9;
    const auto create_code = initialized
                                 ? zju_coop_create(&config, &handle.value)
                                 : ZJU_COOP_INTERNAL_ERROR;
    recorder.check(create_code == ZJU_COOP_OK && handle.value != nullptr,
                   "create_session", api_error(create_code));
    if (create_code != ZJU_COOP_OK || handle.value == nullptr) {
      return recorder.result();
    }

    // 阶段2：配置三节点15维惯性状态；位置组成3-4-5三角形以提供非退化几何。
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

    zju_coop_inertial_config_t inertial_config{};
    inertial_initialized =
        inertial_initialized &&
        zju_coop_inertial_config_init(&inertial_config) == ZJU_COOP_OK;
    inertial_config.nodes = inertial_nodes.data();
    inertial_config.node_count =
        static_cast<std::uint32_t>(inertial_nodes.size());
    const auto configure_code =
        inertial_initialized
            ? zju_coop_configure_inertial(handle.value, &inertial_config)
            : ZJU_COOP_INTERNAL_ERROR;
    recorder.check(configure_code == ZJU_COOP_OK, "configure_inertial",
                   api_error(configure_code));
    if (configure_code != ZJU_COOP_OK) {
      return recorder.result();
    }

    // 阶段3：第一批静止IMU建立各节点时间基准，第二批完成真实10 ms传播。
    bool baselines_ok = true;
    for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
      auto packet = make_stationary_imu(node_id, 1U, kInitialTimeNs);
      zju_coop_imu_processing_result_t result{};
      auto code = zju_coop_imu_processing_result_init(&result);
      if (code == ZJU_COOP_OK) {
        code = zju_coop_push_imu(handle.value, &packet, &result);
      }
      baselines_ok = baselines_ok && code == ZJU_COOP_OK &&
                     result.disposition == ZJU_COOP_IMU_BASELINE_ESTABLISHED &&
                     result.propagated == ZJU_COOP_FALSE;
    }
    recorder.check(baselines_ok, "imu_baseline");

    const auto propagated_time_ns = kInitialTimeNs + kImuIntervalNs;
    bool propagation_ok = true;
    for (std::uint32_t node_id = 1U; node_id <= 3U; ++node_id) {
      auto packet = make_stationary_imu(node_id, 2U, propagated_time_ns);
      zju_coop_imu_processing_result_t result{};
      auto code = zju_coop_imu_processing_result_init(&result);
      if (code == ZJU_COOP_OK) {
        code = zju_coop_push_imu(handle.value, &packet, &result);
      }
      propagation_ok = propagation_ok && code == ZJU_COOP_OK &&
                       result.disposition == ZJU_COOP_IMU_PROPAGATED &&
                       result.propagated == ZJU_COOP_TRUE &&
                       approximately(result.dt_s, 0.01, 1.0e-12);
    }
    recorder.check(propagation_ok, "imu_propagation");

    // 阶段4：输入三条一致测距，验证滤波更新、动态拓扑和主参考相对输出。
    const std::array<zju_coop_range_packet_t, 3U> nominal_ranges{
        make_range(1U, 2U, 3.0, 1U, propagated_time_ns),
        make_range(1U, 3U, 4.0, 1U, propagated_time_ns),
        make_range(2U, 3U, 5.0, 1U, propagated_time_ns)};
    std::string detail;
    const bool ranges_ok =
        push_ranges(handle.value, nominal_ranges,
                    ZJU_COOP_PROCESSING_PROCESSED, detail);
    recorder.check(ranges_ok, "range_updates", detail);

    Snapshot nominal;
    detail.clear();
    const bool nominal_step_ok =
        take_snapshot(handle.value, propagated_time_ns, nominal, detail);
    const auto* node1 = find_localization(nominal, 1U);
    const auto* node2 = find_localization(nominal, 2U);
    const auto* node3 = find_localization(nominal, 3U);
    const bool nominal_geometry_ok =
        nominal_step_ok && nominal.localizations.size() == 3U &&
        nominal.observations.size() == 3U && node1 != nullptr &&
        node2 != nullptr && node3 != nullptr &&
        approximately(node1->x, 0.0) && approximately(node1->y, 0.0) &&
        approximately(node2->x, 3.0) && approximately(node2->y, 0.0) &&
        approximately(node3->x, 0.0) && approximately(node3->y, 4.0);
    recorder.check(nominal_geometry_ok, "nominal_snapshot", detail);

    const bool topology_ok =
        nominal_step_ok && nominal.network.node_count == 3U &&
        nominal.network.reachable_node_count == 3U &&
        nominal.network.active_edge_count == 3U &&
        nominal.network.connected == ZJU_COOP_TRUE &&
        nominal.network.observable == ZJU_COOP_TRUE;
    recorder.check(topology_ok, "network_topology");

    // 数值位置正确仍不够：能力有效位是给GCS/实车消费者的安全契约，必须单独验。
    const bool output_flags_ok =
        node1 != nullptr && node2 != nullptr && node3 != nullptr &&
        node1->reference_node_id == 1U && node2->reference_node_id == 1U &&
        node3->reference_node_id == 1U && node1->yaw_valid == ZJU_COOP_FALSE &&
        node2->yaw_valid == ZJU_COOP_FALSE &&
        node3->yaw_valid == ZJU_COOP_FALSE &&
        node1->z_valid == ZJU_COOP_FALSE && node2->z_valid == ZJU_COOP_FALSE &&
        node3->z_valid == ZJU_COOP_FALSE;
    recorder.check(output_flags_ok, "output_validity_flags");

    // 阶段5：重复包、链路超时和新量测恢复覆盖在线状态机的关键异常路径。
    zju_coop_range_processing_result_t duplicate_result{};
    auto duplicate_code =
        zju_coop_range_processing_result_init(&duplicate_result);
    if (duplicate_code == ZJU_COOP_OK) {
      duplicate_code = zju_coop_push_range(handle.value, &nominal_ranges[0],
                                           &duplicate_result);
    }
    recorder.check(duplicate_code == ZJU_COOP_OK &&
                       duplicate_result.disposition ==
                           ZJU_COOP_PROCESSING_DUPLICATE,
                   "duplicate_rejection", api_error(duplicate_code));

    const auto timeout_time_ns =
        propagated_time_ns + config.edge_timeout_ns + 1U;
    Snapshot timed_out;
    detail.clear();
    const bool timeout_step_ok =
        take_snapshot(handle.value, timeout_time_ns, timed_out, detail);
    const bool timeout_ok =
        timeout_step_ok && timed_out.network.active_edge_count == 0U &&
        timed_out.network.connected == ZJU_COOP_FALSE &&
        (timed_out.network.reason_mask & ZJU_COOP_REASON_LINK_TIMEOUT) != 0U;
    recorder.check(timeout_ok, "link_timeout", detail);

    const auto recovery_time_ns = timeout_time_ns + kImuIntervalNs;
    const std::array<zju_coop_range_packet_t, 3U> recovery_ranges{
        make_range(1U, 2U, 3.0, 2U, recovery_time_ns),
        make_range(1U, 3U, 4.0, 2U, recovery_time_ns),
        make_range(2U, 3U, 5.0, 2U, recovery_time_ns)};
    detail.clear();
    const bool recovery_input_ok =
        push_ranges(handle.value, recovery_ranges,
                    ZJU_COOP_PROCESSING_PROCESSED, detail);
    Snapshot recovered;
    const bool recovery_step_ok =
        recovery_input_ok &&
        take_snapshot(handle.value, recovery_time_ns, recovered, detail);
    const bool recovery_ok =
        recovery_step_ok && recovered.network.active_edge_count == 3U &&
        recovered.network.reachable_node_count == 3U &&
        recovered.network.connected == ZJU_COOP_TRUE &&
        recovered.network.observable == ZJU_COOP_TRUE;
    recorder.check(recovery_ok, "network_recovery", detail);

    const auto destroy_code = zju_coop_destroy(handle.value);
    handle.value = nullptr;
    recorder.check(destroy_code == ZJU_COOP_OK, "destroy_session",
                   api_error(destroy_code));
  } catch (const std::exception& error) {
    recorder.check(false, "unexpected_exception", error.what());
  } catch (...) {
    recorder.check(false, "unexpected_exception", "unknown exception");
  }

  return recorder.result();
}

}  // namespace zju::coop::self_check

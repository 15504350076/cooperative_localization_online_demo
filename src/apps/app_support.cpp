// 模块实现：把强类型INI配置转换为C ABI初始化结构，并将算法输出编码为临时遥测帧。
// 关键原则：会话创建失败立即释放资源；step先查询数组容量再填充；告警按激活/恢复成对输出，
// 使在线运行与日志回放复用完全相同的算法调用和GCS输出口径。
#include "apps/app_support.hpp"

#include "zju_coop/types.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace zju::coop::apps {
namespace {

void require_ok(zju_coop_error_code_t code, const char* operation) {
  if (code != ZJU_COOP_OK) {
    throw std::runtime_error(std::string(operation) + ": " +
                             zju_coop_error_string(code));
  }
}

std::uint16_t wire_node(std::uint32_t value) {
  if (value > std::numeric_limits<std::uint16_t>::max()) {
    throw std::runtime_error("node id exceeds the temporary wire protocol");
  }
  return static_cast<std::uint16_t>(value);
}

protocol::AlgorithmRunState run_state(
    zju_coop_localization_state_t state) {
  switch (state) {
    case ZJU_COOP_LOCALIZATION_UNINITIALIZED:
      return protocol::AlgorithmRunState::kInitializing;
    case ZJU_COOP_LOCALIZATION_NORMAL:
      return protocol::AlgorithmRunState::kRunning;
    case ZJU_COOP_LOCALIZATION_DEGRADED:
    case ZJU_COOP_LOCALIZATION_UNOBSERVABLE:
    case ZJU_COOP_LOCALIZATION_STALE:
      return protocol::AlgorithmRunState::kDegraded;
    default:
      throw std::runtime_error("algorithm returned an unknown network state");
  }
}

protocol::Frame output_frame(protocol::MessageType type,
                             std::uint64_t sequence,
                             std::uint64_t timestamp_ns,
                             std::uint16_t source_node,
                             std::uint16_t target_node,
                             std::vector<std::uint8_t> payload) {
  protocol::Frame frame{};
  frame.header.message_type = type;
  frame.header.flags = 0U;
  frame.header.sequence = sequence;
  frame.header.timestamp_ns = timestamp_ns;
  frame.header.source_node = source_node;
  frame.header.target_node = target_node;
  frame.payload = std::move(payload);
  return frame;
}

}  // namespace

AlgorithmSession::AlgorithmSession(const config::DemoConfig& demo_config) {
  // 阶段1：构造基础节点配置并创建句柄，C API在调用期间深拷贝临时数组。
  std::vector<zju_coop_node_initialization_t> nodes(
      demo_config.engine.nodes.size());
  for (std::size_t index = 0U; index < nodes.size(); ++index) {
    require_ok(zju_coop_node_initialization_init(&nodes[index]),
               "initialize node");
    const auto& source = demo_config.engine.nodes[index];
    nodes[index].node_id = source.node_id;
    nodes[index].x = source.x;
    nodes[index].y = source.y;
    nodes[index].vx = source.vx;
    nodes[index].vy = source.vy;
    nodes[index].position_std_m = source.position_std_m;
    nodes[index].velocity_std_mps = source.velocity_std_mps;
  }

  zju_coop_config_t config{};
  require_ok(zju_coop_config_init(&config), "initialize configuration");
  config.reference_node_id = demo_config.engine.filter.reference_node_id;
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  config.node_stride = sizeof(zju_coop_node_initialization_t);
  config.nodes = nodes.data();
  config.process_accel_std_mps2 =
      demo_config.engine.filter.process_accel_std_mps2;
  config.nis_gate = demo_config.engine.filter.nis_gate;
  config.max_prediction_step_s =
      demo_config.engine.filter.max_prediction_step_s;
  config.min_covariance_diagonal =
      demo_config.engine.filter.min_covariance_diagonal;
  config.degradation_window_ns = demo_config.engine.degradation.window_ns;
  config.nominal_rate_hz = demo_config.engine.degradation.nominal_rate_hz;
  config.nlos_ratio_threshold =
      demo_config.engine.degradation.nlos_ratio_threshold;
  config.valid_ratio_threshold =
      demo_config.engine.degradation.valid_ratio_threshold;
  config.rate_ratio_threshold =
      demo_config.engine.degradation.rate_ratio_threshold;
  config.nlos_probability_threshold =
      demo_config.engine.degradation.nlos_probability_threshold;
  config.nlos_covariance_scale =
      demo_config.engine.degradation.nlos_covariance_scale;
  config.suspend_duration_ns =
      demo_config.engine.degradation.suspend_duration_ns;
  config.reject_duration_ns =
      demo_config.engine.degradation.reject_duration_ns;
  config.recovery_duration_ns =
      demo_config.engine.degradation.recovery_duration_ns;
  config.max_tracked_edges = static_cast<std::uint32_t>(
      demo_config.engine.degradation.max_tracked_edges);
  config.duplicate_cache_per_link = static_cast<std::uint32_t>(
      demo_config.engine.duplicate_cache_per_link);
  config.edge_timeout_ns = demo_config.engine.edge_timeout_ns;
  config.max_future_skew_ns = demo_config.engine.max_future_skew_ns;
  config.max_receive_delay_ns = demo_config.engine.max_receive_delay_ns;
  config.max_nodes = static_cast<std::uint32_t>(demo_config.engine.max_nodes);
  config.max_edges = static_cast<std::uint32_t>(demo_config.engine.max_edges);
  config.max_state_dimension =
      static_cast<std::uint32_t>(demo_config.engine.max_state_dimension);
  config.rigidity_tolerance = demo_config.engine.rigidity_tolerance;
  require_ok(zju_coop_create(&config, &handle_), "create algorithm session");

  // 阶段2：默认配置存在[inertial]时，在首个输入前一次性启用15维联合滤波。
  if (demo_config.inertial) {
    std::vector<zju_coop_inertial_node_initialization_t> inertial_nodes(
        demo_config.inertial->nodes.size());
    for (std::size_t index = 0U; index < inertial_nodes.size(); ++index) {
      require_ok(zju_coop_inertial_node_initialization_init(
                     &inertial_nodes[index]),
                 "initialize inertial node");
      const auto& source = demo_config.inertial->nodes[index];
      auto& target = inertial_nodes[index];
      target.node_id = source.node_id;
      const double position[3]{source.position_n_m.x, source.position_n_m.y,
                               source.position_n_m.z};
      const double velocity[3]{source.velocity_n_mps.x,
                               source.velocity_n_mps.y,
                               source.velocity_n_mps.z};
      const double orientation[4]{source.orientation_b_to_n.x,
                                  source.orientation_b_to_n.y,
                                  source.orientation_b_to_n.z,
                                  source.orientation_b_to_n.w};
      const double gyro_bias[3]{source.gyro_bias_rad_s.x,
                                source.gyro_bias_rad_s.y,
                                source.gyro_bias_rad_s.z};
      const double accel_bias[3]{source.accel_bias_m_s2.x,
                                 source.accel_bias_m_s2.y,
                                 source.accel_bias_m_s2.z};
      const double position_std[3]{source.position_std_m.x,
                                   source.position_std_m.y,
                                   source.position_std_m.z};
      const double velocity_std[3]{source.velocity_std_mps.x,
                                   source.velocity_std_mps.y,
                                   source.velocity_std_mps.z};
      const double attitude_std[3]{source.attitude_std_rad.x,
                                   source.attitude_std_rad.y,
                                   source.attitude_std_rad.z};
      const double gyro_bias_std[3]{source.gyro_bias_std_rad_s.x,
                                    source.gyro_bias_std_rad_s.y,
                                    source.gyro_bias_std_rad_s.z};
      const double accel_bias_std[3]{source.accel_bias_std_m_s2.x,
                                     source.accel_bias_std_m_s2.y,
                                     source.accel_bias_std_m_s2.z};
      std::copy(std::begin(position), std::end(position), target.position_n_m);
      std::copy(std::begin(velocity), std::end(velocity), target.velocity_n_mps);
      std::copy(std::begin(orientation), std::end(orientation),
                target.orientation_xyzw);
      std::copy(std::begin(gyro_bias), std::end(gyro_bias),
                target.gyro_bias_rad_s);
      std::copy(std::begin(accel_bias), std::end(accel_bias),
                target.accel_bias_m_s2);
      std::copy(std::begin(position_std), std::end(position_std),
                target.position_std_m);
      std::copy(std::begin(velocity_std), std::end(velocity_std),
                target.velocity_std_mps);
      std::copy(std::begin(attitude_std), std::end(attitude_std),
                target.attitude_std_rad);
      std::copy(std::begin(gyro_bias_std), std::end(gyro_bias_std),
                target.gyro_bias_std_rad_s);
      std::copy(std::begin(accel_bias_std), std::end(accel_bias_std),
                target.accel_bias_std_m_s2);
    }
    zju_coop_inertial_config_t inertial{};
    require_ok(zju_coop_inertial_config_init(&inertial),
               "initialize inertial configuration");
    inertial.nodes = inertial_nodes.data();
    inertial.node_count = static_cast<std::uint32_t>(inertial_nodes.size());
    inertial.node_stride = sizeof(zju_coop_inertial_node_initialization_t);
    inertial.max_inertial_state_dimension = static_cast<std::uint32_t>(
        demo_config.inertial->max_inertial_state_dimension);
    const auto& source = demo_config.inertial->filter;
    inertial.gravity_mps2 = source.gravity_mps2;
    inertial.min_imu_dt_s = source.min_imu_dt_s;
    inertial.max_imu_dt_s = source.max_imu_dt_s;
    inertial.max_propagation_substep_s = source.max_propagation_substep_s;
    inertial.gyro_noise_density_rad_s_sqrt_hz =
        source.gyro_noise_density_rad_s_sqrt_hz;
    inertial.accel_noise_density_m_s2_sqrt_hz =
        source.accel_noise_density_m_s2_sqrt_hz;
    inertial.gyro_bias_random_walk_rad_s2_sqrt_hz =
        source.gyro_bias_random_walk_rad_s2_sqrt_hz;
    inertial.accel_bias_random_walk_m_s3_sqrt_hz =
        source.accel_bias_random_walk_m_s3_sqrt_hz;
    inertial.min_covariance_diagonal = source.min_covariance_diagonal;
    inertial.quaternion_norm_tolerance = source.quaternion_norm_tolerance;
    inertial.covariance_symmetry_tolerance =
        source.covariance_symmetry_tolerance;
    inertial.use_message_covariance =
        source.use_message_covariance ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    inertial.use_orientation_for_initialization =
        source.use_orientation_for_initialization ? ZJU_COOP_TRUE
                                                  : ZJU_COOP_FALSE;
    std::memcpy(inertial.expected_frame_id, source.expected_frame_id.c_str(),
                source.expected_frame_id.size() + 1U);
    require_ok(zju_coop_configure_inertial(handle_, &inertial),
               "configure inertial path");
  }
}

AlgorithmSession::~AlgorithmSession() {
  if (handle_ != nullptr) {
    (void)zju_coop_destroy(handle_);
  }
}

zju_coop_range_processing_result_t AlgorithmSession::push_range(
    const protocol::Frame& frame, const protocol::RangePayload& payload,
    std::uint64_t receive_timestamp_ns) {
  // 帧头提供节点、序号和测量时间，接收时间由本机在线/回放入口补充。
  zju_coop_range_packet_t packet{};
  zju_coop_range_processing_result_t result{};
  require_ok(zju_coop_range_packet_init(&packet), "initialize range packet");
  require_ok(zju_coop_range_processing_result_init(&result),
             "initialize range result");
  packet.from_node = frame.header.source_node;
  packet.to_node = frame.header.target_node;
  packet.sequence = frame.header.sequence;
  packet.timestamp_ns = frame.header.timestamp_ns;
  packet.receive_timestamp_ns = receive_timestamp_ns;
  packet.range_m = payload.range_m;
  packet.range_std_m = payload.range_std_m;
  packet.nlos_probability = payload.nlos_probability;
  packet.nlos_flag = payload.nlos_flag ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  packet.has_nlos_probability =
      payload.has_nlos_probability ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  packet.valid = payload.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  packet.status = payload.status;
  require_ok(zju_coop_push_range(handle_, &packet, &result), "push range");
  return result;
}

zju_coop_imu_processing_result_t AlgorithmSession::push_imu(
    const protocol::Frame& frame, const protocol::ImuPayload& payload,
    std::uint64_t receive_timestamp_ns) {
  // 保持ROS 2 Imu瞬时量与协方差语义，不在适配层预积分或添加温度字段。
  zju_coop_imu_packet_t packet{};
  zju_coop_imu_processing_result_t result{};
  require_ok(zju_coop_imu_packet_init(&packet), "initialize IMU packet");
  require_ok(zju_coop_imu_processing_result_init(&result),
             "initialize IMU result");
  packet.node_id = frame.header.source_node;
  packet.sequence = frame.header.sequence;
  packet.timestamp_ns = frame.header.timestamp_ns;
  packet.receive_timestamp_ns = receive_timestamp_ns;
  std::copy(payload.orientation_xyzw.begin(), payload.orientation_xyzw.end(),
            packet.orientation_xyzw);
  std::copy(payload.orientation_covariance.begin(),
            payload.orientation_covariance.end(),
            packet.orientation_covariance);
  std::copy(payload.angular_velocity_rad_s.begin(),
            payload.angular_velocity_rad_s.end(),
            packet.angular_velocity_rad_s);
  std::copy(payload.angular_velocity_covariance.begin(),
            payload.angular_velocity_covariance.end(),
            packet.angular_velocity_covariance);
  std::copy(payload.linear_acceleration_m_s2.begin(),
            payload.linear_acceleration_m_s2.end(),
            packet.linear_acceleration_m_s2);
  std::copy(payload.linear_acceleration_covariance.begin(),
            payload.linear_acceleration_covariance.end(),
            packet.linear_acceleration_covariance);
  std::copy(payload.frame_id.begin(), payload.frame_id.end(), packet.frame_id);
  packet.orientation_valid =
      payload.orientation_valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  packet.valid = payload.valid ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  packet.status = payload.status;
  require_ok(zju_coop_push_imu(handle_, &packet, &result), "push IMU");
  return result;
}

StepSnapshot AlgorithmSession::step(std::uint64_t now_ns) {
  // 阶段1：先用NULL缓冲查询固定输出数量，查询不会推进Engine时间。
  std::uint32_t localization_count = 0U;
  std::uint32_t observation_count = 0U;
  const auto query = zju_coop_step(
      handle_, now_ns, nullptr, 0U, 0U, &localization_count, nullptr, 0U, 0U,
      &observation_count, nullptr);
  if (query != ZJU_COOP_BUFFER_TOO_SMALL) {
    require_ok(query, "query algorithm output sizes");
    throw std::runtime_error("algorithm size query returned no sizes");
  }

  // 阶段2：按查询数量初始化每个版本化结构，再执行真正的原子step。
  StepSnapshot snapshot{};
  snapshot.localizations.resize(localization_count);
  snapshot.observations.resize(observation_count);
  for (auto& value : snapshot.localizations) {
    require_ok(zju_coop_localization_init(&value),
               "initialize localization output");
  }
  for (auto& value : snapshot.observations) {
    require_ok(zju_coop_observation_init(&value),
               "initialize observation output");
  }
  require_ok(zju_coop_network_init(&snapshot.network),
             "initialize network output");
  require_ok(
      zju_coop_step(
          handle_, now_ns,
          snapshot.localizations.empty() ? nullptr
                                         : snapshot.localizations.data(),
          localization_count, sizeof(zju_coop_localization_t),
          &localization_count,
          snapshot.observations.empty() ? nullptr : snapshot.observations.data(),
          observation_count, sizeof(zju_coop_observation_t),
          &observation_count, &snapshot.network),
      "step algorithm");
  snapshot.localizations.resize(localization_count);
  snapshot.observations.resize(observation_count);
  return snapshot;
}

std::vector<EncodedOutput> encode_snapshot(
    const StepSnapshot& snapshot, std::uint32_t reference_node_id,
    std::uint64_t& next_sequence, std::size_t max_payload_size) {
  // 每个定位节点和观测边独立成帧，网络状态每个快照只发送一帧。
  std::vector<EncodedOutput> result;
  result.reserve(snapshot.localizations.size() + snapshot.observations.size() +
                 1U);
  constexpr std::uint32_t capabilities =
      static_cast<std::uint32_t>(Capability::kUwbRange) |
      static_cast<std::uint32_t>(Capability::kPlanarPosition) |
      static_cast<std::uint32_t>(Capability::kVelocity);

  for (const auto& source : snapshot.localizations) {
    if (source.yaw_valid != ZJU_COOP_FALSE ||
        source.z_valid != ZJU_COOP_FALSE) {
      throw std::runtime_error("UWB-only algorithm claimed yaw or altitude");
    }
    protocol::LocalizationPayload payload{};
    payload.x = source.x;
    payload.y = source.y;
    payload.vx = source.vx;
    payload.vy = source.vy;
    payload.cov_xx = source.cov_xx;
    payload.cov_xy = source.cov_xy;
    payload.cov_yy = source.cov_yy;
    payload.state = static_cast<LocalizationState>(source.state);
    payload.valid = source.valid == ZJU_COOP_TRUE;
    payload.yaw_valid = false;
    payload.z_valid = false;
    payload.capability_mask = capabilities;
    auto frame = output_frame(
        protocol::MessageType::kLocalization, next_sequence++,
        source.timestamp_ns, wire_node(source.node_id),
        wire_node(source.reference_node_id),
        protocol::encode_localization_payload(payload));
    result.push_back({protocol::MessageType::kLocalization,
                      protocol::encode_frame(frame, max_payload_size)});
  }

  protocol::NetworkPayload network{};
  network.node_count = snapshot.network.node_count;
  network.reachable_node_count = snapshot.network.reachable_node_count;
  network.active_edge_count = snapshot.network.active_edge_count;
  network.connected = snapshot.network.connected == ZJU_COOP_TRUE;
  network.observable = snapshot.network.observable == ZJU_COOP_TRUE;
  network.state = static_cast<LocalizationState>(snapshot.network.state);
  network.reason_mask = snapshot.network.reason_mask;
  auto network_frame = output_frame(
      protocol::MessageType::kNetwork, next_sequence++,
      snapshot.network.timestamp_ns, wire_node(reference_node_id), 0U,
      protocol::encode_network_payload(network));
  result.push_back({protocol::MessageType::kNetwork,
                    protocol::encode_frame(network_frame, max_payload_size)});

  for (const auto& source : snapshot.observations) {
    protocol::ObservationPayload payload{};
    payload.window_start_ns = source.window_start_ns;
    payload.window_end_ns = source.window_end_ns;
    payload.expected_count = source.expected_count;
    payload.received_count = source.received_count;
    payload.valid_count = source.valid_count;
    payload.nlos_count = source.nlos_count;
    payload.residual_rejected_count = source.residual_rejected_count;
    payload.dropped_count = source.dropped_count;
    payload.nlos_ratio = source.nlos_ratio;
    payload.valid_ratio = source.valid_ratio;
    payload.actual_rate_hz = source.actual_rate_hz;
    payload.covariance_scale = source.covariance_scale;
    payload.state = static_cast<ObservationState>(source.state);
    payload.action = static_cast<FusionAction>(source.fusion_action);
    payload.input_overflow = source.input_overflow == ZJU_COOP_TRUE;
    payload.reason_mask = source.reason_mask;
    auto frame = output_frame(
        protocol::MessageType::kObservation, next_sequence++,
        snapshot.network.timestamp_ns, wire_node(source.from_node),
        wire_node(source.to_node),
        protocol::encode_observation_payload(payload));
    result.push_back({protocol::MessageType::kObservation,
                      protocol::encode_frame(frame, max_payload_size)});
  }
  return result;
}

std::vector<EncodedOutput> TelemetryEncoder::encode(
    const zju_coop_network_t& network,
    const TelemetryCounters& counters, std::uint64_t uptime_ns,
    std::uint32_t reference_node_id, std::uint64_t& next_sequence,
    std::size_t max_payload_size, protocol::AlgorithmMode mode) {
  // 先发布周期算法状态，再根据原因位图变化发布告警激活或恢复事件。
  std::vector<EncodedOutput> result;
  result.reserve(2U);

  protocol::AlgorithmStatusPayload status{};
  status.mode = mode;
  status.run_state = run_state(network.state);
  status.accepted_ranges = counters.accepted_ranges;
  status.rejected_ranges = counters.rejected_ranges;
  status.protocol_errors = counters.protocol_errors;
  status.uptime_ns = uptime_ns;
  auto status_frame = output_frame(
      protocol::MessageType::kAlgorithmStatus, next_sequence++,
      network.timestamp_ns, wire_node(reference_node_id), 0U,
      protocol::encode_algorithm_status_payload(status));
  result.push_back({
      protocol::MessageType::kAlgorithmStatus,
      protocol::encode_frame(status_frame, max_payload_size)});

  const bool normal = network.state == ZJU_COOP_LOCALIZATION_NORMAL;
  if (!normal && network.reason_mask != 0U) {
    if (!alert_active_) {
      alert_active_ = true;
      alert_first_timestamp_ns_ = network.timestamp_ns;
    }
    protocol::AlertPayload alert{};
    alert.level = protocol::AlertLevel::kWarning;
    alert.lifecycle = protocol::AlertLifecycle::kActive;
    alert.reason_mask = network.reason_mask;
    alert.first_timestamp_ns = alert_first_timestamp_ns_;
    alert.last_timestamp_ns = network.timestamp_ns;
    auto alert_frame = output_frame(
        protocol::MessageType::kAlert, next_sequence++, network.timestamp_ns,
        wire_node(reference_node_id), 0U,
        protocol::encode_alert_payload(alert));
    result.push_back({protocol::MessageType::kAlert,
                      protocol::encode_frame(alert_frame, max_payload_size)});
  } else if (normal && alert_active_) {
    protocol::AlertPayload alert{};
    alert.level = protocol::AlertLevel::kInfo;
    alert.lifecycle = protocol::AlertLifecycle::kCleared;
    alert.reason_mask = 0U;
    alert.first_timestamp_ns = alert_first_timestamp_ns_;
    alert.last_timestamp_ns = network.timestamp_ns;
    auto alert_frame = output_frame(
        protocol::MessageType::kAlert, next_sequence++, network.timestamp_ns,
        wire_node(reference_node_id), 0U,
        protocol::encode_alert_payload(alert));
    result.push_back({protocol::MessageType::kAlert,
                      protocol::encode_frame(alert_frame, max_payload_size)});
    alert_active_ = false;
    alert_first_timestamp_ns_ = 0U;
  }
  return result;
}

std::uint64_t system_time_ns() {
  const auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(since_epoch)
          .count());
}

std::uint64_t elapsed_ns(std::uint64_t timestamp_ns,
                         std::uint64_t start_timestamp_ns) noexcept {
  return timestamp_ns >= start_timestamp_ns
             ? timestamp_ns - start_timestamp_ns
             : 0U;
}

}  // namespace zju::coop::apps

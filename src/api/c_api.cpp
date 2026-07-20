// 将稳定 C ABI 转换到 C++ Engine，并集中完成版本、步长、布尔值和数值合法性校验。
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

struct zju_coop_handle {
  zju_coop_handle(zju::coop::EngineConfig config,
                  std::uint32_t configured_node_count,
                  std::uint32_t configured_edge_count)
      : engine(std::make_unique<zju::coop::Engine>(std::move(config))),
        localization_count(configured_node_count),
        observation_count(configured_edge_count) {}

  std::unique_ptr<zju::coop::Engine> engine;
  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  bool processing_started{};
  bool inertial_configured{};
};

namespace {

constexpr std::uint32_t kMaximumNodes = 64U;
constexpr std::uint32_t kMaximumEdges = 2016U;
constexpr std::uint32_t kMaximumStateDimension = 252U;
constexpr std::uint32_t kMaximumDuplicateCachePerLink = 4096U;
constexpr std::uint32_t kMaximumTrackedEdges = 1'000'000U;

template <typename Structure>
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

bool valid_boolean(zju_coop_bool_t value) {
  return value == ZJU_COOP_FALSE || value == ZJU_COOP_TRUE;
}

bool finite(double value) { return std::isfinite(value); }

template <std::size_t Size>
bool finite_array(const double (&values)[Size]) {
  return std::all_of(std::begin(values), std::end(values),
                     [](double value) { return finite(value); });
}

bool nul_terminated_nonempty(const char* value, std::size_t capacity) {
  return value != nullptr && value[0] != '\0' &&
         std::memchr(value, '\0', capacity) != nullptr;
}

template <typename Structure>
bool valid_array_span(const void* base, std::uint32_t count,
                      std::uint32_t stride) {
  if (count == 0U) {
    return true;
  }
  if (base == nullptr || stride < sizeof(Structure) ||
      stride % alignof(Structure) != 0U) {
    return false;
  }
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
const Structure* array_element(const void* base, std::uint32_t index,
                               std::uint32_t stride) {
  const auto* bytes = static_cast<const unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<const Structure*>(bytes + offset);
}

template <typename Structure>
Structure* array_element(void* base, std::uint32_t index,
                         std::uint32_t stride) {
  auto* bytes = static_cast<unsigned char*>(base);
  const auto offset = static_cast<std::uintptr_t>(index) * stride;
  return reinterpret_cast<Structure*>(bytes + offset);
}

bool finite_node(const zju_coop_node_initialization_t& node) {
  return finite(node.x) && finite(node.y) && finite(node.vx) &&
         finite(node.vy) && finite(node.position_std_m) &&
         finite(node.velocity_std_mps);
}

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

zju_coop_error_code_t convert_config(const zju_coop_config_t& input,
                                     zju::coop::EngineConfig& output) {
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

  const std::uint64_t node_count = input.node_count;
  const std::uint64_t edge_count = node_count * (node_count - 1U) / 2U;
  const std::uint64_t state_dimension = (node_count - 1U) * 4U;
  if (edge_count > input.max_edges || edge_count > input.max_tracked_edges ||
      state_dimension > input.max_state_dimension) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  std::vector<zju::coop::NodeInitialization> nodes;
  nodes.reserve(input.node_count);
  for (std::uint32_t index = 0U; index < input.node_count; ++index) {
    const auto* source = array_element<zju_coop_node_initialization_t>(
        input.nodes, index, input.node_stride);
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
    const zju_coop_inertial_config_t& input,
    zju::coop::InertialConfig& inertial,
    std::vector<zju::coop::InertialNodeInitialization>& nodes) {
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
  for (std::uint32_t index = 0U; index < input.node_count; ++index) {
    const auto* source =
        array_element<zju_coop_inertial_node_initialization_t>(
            input.nodes, index, input.node_stride);
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

zju_coop_localization_t localization_output(
    const zju::coop::LocalizationSnapshot& source) {
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

zju_coop_observation_t observation_output(
    const zju::coop::ObservationQuality& source) {
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

zju_coop_network_t network_output(const zju::coop::NetworkSnapshot& source) {
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

bool counts_fit_v1(const zju::coop::EngineSnapshot& snapshot) {
  constexpr auto maximum = std::numeric_limits<std::uint32_t>::max();
  if (snapshot.localizations.size() > maximum ||
      snapshot.observations.size() > maximum ||
      snapshot.network.node_count > maximum ||
      snapshot.network.reachable_node_count > maximum ||
      snapshot.network.active_edge_count > maximum) {
    return false;
  }
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
  if (out_handle == nullptr) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  *out_handle = nullptr;
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  try {
    zju::coop::EngineConfig converted{};
    const auto conversion_status = convert_config(*config, converted);
    if (conversion_status != ZJU_COOP_OK) {
      return conversion_status;
    }
    const std::uint64_t node_count = config->node_count;
    const std::uint64_t edge_count =
        node_count * (node_count > 0U ? node_count - 1U : 0U) / 2U;
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
  const auto header_status = validate_header(config);
  if (header_status != ZJU_COOP_OK) {
    return header_status;
  }
  if (handle->processing_started || handle->inertial_configured) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    zju::coop::InertialConfig converted{};
    std::vector<zju::coop::InertialNodeInitialization> nodes;
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
                   [](std::uint8_t value) { return value == 0U; })) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }

  try {
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

    const auto processed = handle->engine->push_imu(converted);
    zju_coop_imu_processing_result_t output{};
    output.struct_size = sizeof(output);
    output.abi_version = ZJU_COOP_ABI_VERSION_V1;
    output.disposition = imu_disposition(processed.disposition);
    output.propagated =
        processed.propagated ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
    output.dt_s = processed.dt_s;
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
      !finite(packet->range_m) || !finite(packet->range_std_m) ||
      !finite(static_cast<double>(packet->nlos_probability)) ||
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

    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      const auto* output = array_element<zju_coop_localization_t>(
          localizations, index, localization_stride);
      const auto status = validate_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > localization_stride) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      const auto* output = array_element<zju_coop_observation_t>(
          observations, index, observation_stride);
      const auto status = validate_header(output);
      if (status != ZJU_COOP_OK || output->struct_size > observation_stride) {
        return status != ZJU_COOP_OK ? status : ZJU_COOP_INVALID_ARGUMENT;
      }
    }
    const auto network_status = validate_header(network);
    if (network_status != ZJU_COOP_OK) {
      return network_status;
    }

    std::vector<zju_coop_localization_t> localization_values(
        required_localizations);
    std::vector<zju_coop_observation_t> observation_values(
        required_observations);
    auto candidate = std::make_unique<zju::coop::Engine>(*handle->engine);
    const auto snapshot = candidate->step(now_ns);
    if (!counts_fit_v1(snapshot) ||
        snapshot.localizations.size() != required_localizations ||
        snapshot.observations.size() != required_observations) {
      return ZJU_COOP_INTERNAL_ERROR;
    }
    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      localization_values[index] =
          localization_output(snapshot.localizations[index]);
    }
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      observation_values[index] =
          observation_output(snapshot.observations[index]);
    }
    const auto network_value = network_output(snapshot.network);

    handle->engine.swap(candidate);
    handle->processing_started = true;
    for (std::uint32_t index = 0U; index < required_localizations; ++index) {
      auto* output = array_element<zju_coop_localization_t>(
          localizations, index, localization_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = localization_values[index];
      output->struct_size = caller_size;
    }
    for (std::uint32_t index = 0U; index < required_observations; ++index) {
      auto* output = array_element<zju_coop_observation_t>(
          observations, index, observation_stride);
      const std::uint32_t caller_size = output->struct_size;
      *output = observation_values[index];
      output->struct_size = caller_size;
    }
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

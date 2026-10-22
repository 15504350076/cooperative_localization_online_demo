#include "zju_coop/relative_transform_api.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

// Release构建也必须执行检查，不能把有副作用的API调用放入assert。
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)
using Session = std::unique_ptr<zju_coop_rte_handle_t, decltype(&zju_coop_destroy_rte)>;
using Truth = std::array<double, 4>;
using Sample = zju_coop_sample_rte_t;

Session create(zju_coop_config_rte_t config) {
  zju_coop_rte_handle_t* handle = nullptr;
  CHECK(zju_coop_create_rte(&config, &handle) == ZJU_COOP_OK);
  return Session(handle, zju_coop_destroy_rte);
}

double range(const Sample& s, const Truth& truth) {
  const double c = std::cos(truth[3]), sn = std::sin(truth[3]);
  const double x = truth[0] + c * s.target_antenna_position_m[0] - sn * s.target_antenna_position_m[1] - s.host_antenna_position_m[0];
  const double y = truth[1] + sn * s.target_antenna_position_m[0] + c * s.target_antenna_position_m[1] - s.host_antenna_position_m[1];
  const double z = truth[2] + s.target_antenna_position_m[2] - s.host_antenna_position_m[2];
  return std::sqrt(x * x + y * y + z * z);
}

std::vector<Sample> trajectory(const Truth& truth, int count, bool noisy = false, bool planar = false) {
  std::mt19937 generator(205);
  std::uniform_real_distribution<double> position(-2.5, 2.5);
  std::normal_distribution<double> noise(0.0, 1.0);
  std::vector<Sample> samples;
  for (int i = 0; i < count; ++i) {
    Sample sample;
    CHECK(zju_coop_sample_init_rte(&sample) == ZJU_COOP_OK);
    sample.timestamp_ns = 1000000000ULL + static_cast<uint64_t>(i) * 100000000ULL;
    for (int j = 0; j < 3; ++j) {
      sample.host_antenna_position_m[j] = planar && j == 2 ? 0.0 : position(generator);
      sample.target_antenna_position_m[j] = planar && j == 2 ? 0.0 : position(generator);
    }
    sample.range_std_m = noisy ? 0.02 + (i % 3) * 0.01 : 0.001;
    sample.range_m = range(sample, truth) + (noisy ? sample.range_std_m * noise(generator) : 0.0);
    samples.push_back(sample);
  }
  return samples;
}

zju_coop_estimate_rte_t estimate(Session& session) {
  zju_coop_estimate_rte_t result;
  CHECK(zju_coop_estimate_init_rte(&result) == ZJU_COOP_OK);
  CHECK(zju_coop_step_rte(session.get(), &result) == ZJU_COOP_OK);
  return result;
}

void accuracy_and_objective() {
  zju_coop_config_rte_t config;
  CHECK(zju_coop_config_init_rte(&config) == ZJU_COOP_OK);
  for (const Truth truth : {Truth{2.4, -1.1, 0.7, 0.46}, Truth{-3.0, 2.2, -1.4, -3.10},
                            Truth{1.0, 4.2, 2.1, 2.9}, Truth{7.0, -3.0, -2.1, -1.8}}) {
    for (bool noisy : {false, true}) {
      auto session = create(config);
      const auto samples = trajectory(truth, 80, noisy);
      for (const auto& sample : samples) CHECK(zju_coop_push_sample_rte(session.get(), &sample) == ZJU_COOP_OK);
      const auto out = estimate(session);
      CHECK(out.status == ZJU_COOP_RTE_READY);
      const double tolerance = noisy ? 0.08 : 1e-4;
      for (int i = 0; i < 3; ++i) CHECK(std::abs(out.translation_m[i] - truth[i]) < tolerance);
      CHECK(std::abs(std::remainder(out.yaw_rad - truth[3], 2.0 * 3.141592653589793)) < tolerance);
      CHECK(out.rms_residual_m < (noisy ? 0.06 : 1e-4));
      CHECK(out.covariance_valid && out.covariance[0] > 0 && std::isfinite(out.condition_number));
      CHECK(out.timestamp_ns == samples.back().timestamp_ns);
      double objective = 0;
      const Truth solved{out.translation_m[0], out.translation_m[1], out.translation_m[2], out.yaw_rad};
      for (const auto& sample : samples) {
        const double predicted = range(sample, solved);
        const double variance = sample.range_std_m * sample.range_std_m;
        const double residual = predicted * predicted - sample.range_m * sample.range_m + variance;
        objective += 0.5 * residual * residual / (variance * (4 * sample.range_m * sample.range_m + 2 * variance));
      }
      CHECK(std::abs(objective - out.squared_distance_cost) < 1e-7 * (1.0 + objective));
      std::printf("RTE accuracy: yaw=%+.2f, noisy=%d, rms=%.6f, cost=%.6f, iterations=%u\n",
                  truth[3], noisy, out.rms_residual_m, out.squared_distance_cost, out.iterations);
    }
  }
}

void degenerate_and_window() {
  zju_coop_config_rte_t config;
  zju_coop_config_init_rte(&config);
  {
    auto session = create(config);
    Sample s;
    zju_coop_sample_init_rte(&s);
    s.range_m = 5.0;
    for (uint64_t t = 1; t <= 20; ++t) {
      s.timestamp_ns = t;
      CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_OK);
    }
    const auto out = estimate(session);
    CHECK(out.status == ZJU_COOP_RTE_UNOBSERVABLE);
    CHECK(!out.covariance_valid && std::isfinite(out.condition_number));
  }
  {
    auto session = create(config);
    for (const auto& s : trajectory({2, -1, 1.3, 0.7}, 40, false, true)) {
      CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_OK);
    }
    CHECK(estimate(session).status == ZJU_COOP_RTE_AMBIGUOUS);
  }
  {
    config.max_samples = 20;
    auto session = create(config);
    for (const auto& s : trajectory({2, -1, 1.3, 0.7}, 40)) {
      CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_OK);
    }
    const auto out = estimate(session);
    CHECK(out.status == ZJU_COOP_RTE_READY && out.sample_count == 20);
  }
}

void invalid_input() {
  zju_coop_config_rte_t config;
  zju_coop_config_init_rte(&config);
  auto session = create(config);
  CHECK(zju_coop_config_init_rte(nullptr) == ZJU_COOP_INVALID_ARGUMENT);
  zju_coop_rte_handle_t* unused = session.get();
  config.abi_version++;
  CHECK(zju_coop_create_rte(&config, &unused) == ZJU_COOP_ABI_MISMATCH && unused == nullptr);
  config.struct_size = 4;
  CHECK(zju_coop_create_rte(&config, &unused) == ZJU_COOP_STRUCT_SIZE_MISMATCH);
  Sample s = trajectory({2, -1, 0.5, 0.2}, 1).front();
  const auto saved = s;
  s.range_m = std::numeric_limits<double>::quiet_NaN();
  CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_INVALID_ARGUMENT);
  s = saved;
  CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_OK);
  CHECK(zju_coop_push_sample_rte(session.get(), &s) == ZJU_COOP_INVALID_ARGUMENT);
  zju_coop_estimate_rte_t out;
  zju_coop_estimate_init_rte(&out);
  CHECK(zju_coop_step_rte(session.get(), &out) == ZJU_COOP_NOT_READY);
  CHECK(out.status == ZJU_COOP_RTE_NO_ESTIMATE && out.sample_count == 1);
  out.struct_size = 4;
  const auto before = out;
  CHECK(zju_coop_step_rte(session.get(), &out) == ZJU_COOP_STRUCT_SIZE_MISMATCH);
  CHECK(std::memcmp(&before, &out, sizeof(out)) == 0);
}

void existing_interface_adapter() {
  zju_coop_config_rte_t config;
  zju_coop_config_init_rte(&config);
  config.host_node_id = 201;
  config.target_node_id = 202;
  config.host_lever_arm_body_m[0] = 0.21;
  config.target_lever_arm_body_m[1] = -0.16;
  auto session = create(config);
  const Truth truth{2.4, -1.1, 0.7, 0.46};
  const auto samples = trajectory(truth, 40);
  zju_coop_range_processing_result_t receipt{};
  receipt.struct_size = sizeof(receipt);
  receipt.abi_version = ZJU_COOP_ABI_VERSION_V1;
  zju_coop_range_packet_t packet{};
  packet.struct_size = sizeof(packet);
  packet.abi_version = ZJU_COOP_ABI_VERSION_V1;
  packet.from_node = 201;
  packet.to_node = 202;
  packet.valid = ZJU_COOP_TRUE;
  packet.range_m = 2;
  packet.range_std_m = 0.001;
  packet.timestamp_ns = samples[0].timestamp_ns;
  CHECK(zju_coop_push_range_rte(session.get(), &packet, &receipt) == ZJU_COOP_OK);
  CHECK(receipt.disposition == ZJU_COOP_PROCESSING_HELD);
  packet.range_m = std::numeric_limits<double>::quiet_NaN();
  CHECK(zju_coop_push_range_rte(session.get(), &packet, &receipt) == ZJU_COOP_OK);
  CHECK(receipt.disposition == ZJU_COOP_PROCESSING_INVALID_PACKET);
  packet.range_m = 2;
  for (const auto& s : samples) {
    Sample antenna = s;
    for (int i = 0; i < 2; ++i) {
      zju_coop_node_state_t state{};
      state.struct_size = sizeof(state);
      state.abi_version = ZJU_COOP_ABI_VERSION_V1;
      state.node_id = i == 0 ? 201 : 202;
      const double body_yaw = i == 0 ? 0.3 : -0.2;
      state.orientation_flu_to_enu_xyzw[2] = std::sin(body_yaw / 2);
      state.orientation_flu_to_enu_xyzw[3] = std::cos(body_yaw / 2);
      state.valid = ZJU_COOP_TRUE;
      const double* position = i == 0 ? s.host_antenna_position_m : s.target_antenna_position_m;
      // 测距前后各5ms的状态，库内应线性插值到真实采样点。
      for (int side : {-1, 1}) {
        state.timestamp_ns = side < 0 ? s.timestamp_ns - 5000000 : s.timestamp_ns + 5000000;
        state.orientation_flu_to_enu_xyzw[2] = std::sin((body_yaw + side * 0.02) / 2);
        state.orientation_flu_to_enu_xyzw[3] = std::cos((body_yaw + side * 0.02) / 2);
        for (int axis = 0; axis < 3; ++axis) state.position_enu_m[axis] = position[axis] + side * 0.005 * (axis + 1);
        CHECK(zju_coop_push_node_state_rte(session.get(), &state) == ZJU_COOP_OK);
      }
    }
    antenna.host_antenna_position_m[0] += std::cos(0.3) * config.host_lever_arm_body_m[0];
    antenna.host_antenna_position_m[1] += std::sin(0.3) * config.host_lever_arm_body_m[0];
    antenna.target_antenna_position_m[0] -= std::sin(-0.2) * config.target_lever_arm_body_m[1];
    antenna.target_antenna_position_m[1] += std::cos(-0.2) * config.target_lever_arm_body_m[1];
    packet.timestamp_ns = s.timestamp_ns;
    packet.range_m = range(antenna, truth);
    CHECK(zju_coop_push_range_rte(session.get(), &packet, &receipt) == ZJU_COOP_OK);
    CHECK(receipt.disposition == ZJU_COOP_PROCESSING_PROCESSED && !receipt.filter_updated);
  }
  CHECK(zju_coop_push_range_rte(session.get(), &packet, &receipt) == ZJU_COOP_OK);
  CHECK(receipt.disposition == ZJU_COOP_PROCESSING_DUPLICATE);
  CHECK(estimate(session).status == ZJU_COOP_RTE_READY);
  zju_coop_pose2d_snapshot_v2_t snapshot{};
  snapshot.struct_size = sizeof(snapshot);
  snapshot.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  uint32_t count = 0;
  CHECK(zju_coop_get_pose2d_v2_rte(session.get(), &snapshot, nullptr, 0, 0, &count) == ZJU_COOP_BUFFER_TOO_SMALL && count == 2);
  // 用扩展stride验证适配层缓冲区布局，避免只测紧密数组。
  struct Extended { zju_coop_vehicle_pose2d_v2_t value{}; uint64_t marker{205}; } vehicles[2];
  for (auto& v : vehicles) {
    v.value.struct_size = sizeof(v.value);
    v.value.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  }
  CHECK(zju_coop_get_pose2d_v2_rte(session.get(), &snapshot, &vehicles[0].value, 2, sizeof(Extended), &count) == ZJU_COOP_OK);
  CHECK(std::strcmp(snapshot.frame_id, "coop_rte_201_local") == 0);
  CHECK(vehicles[0].marker == 205 && vehicles[1].marker == 205);
  CHECK(vehicles[1].value.node_id == 202 && vehicles[1].value.position_valid && vehicles[1].value.yaw_valid);
  const auto& last = samples.back();
  const double expected_x = truth[0] + std::cos(truth[3]) * last.target_antenna_position_m[0] -
      std::sin(truth[3]) * last.target_antenna_position_m[1] - last.host_antenna_position_m[0];
  CHECK(std::abs(vehicles[1].value.x_m - expected_x) < 1e-4);
  CHECK(std::abs(vehicles[0].value.yaw_rad - 0.3) < 1e-4);
  CHECK(std::abs(vehicles[1].value.yaw_rad - (truth[3] - 0.2)) < 1e-4);
  const auto saved_snapshot = snapshot;
  const auto saved_first = vehicles[0];
  vehicles[1].value.abi_version++;
  CHECK(zju_coop_get_pose2d_v2_rte(session.get(), &snapshot, &vehicles[0].value, 2, sizeof(Extended), &count) == ZJU_COOP_ABI_MISMATCH);
  CHECK(std::memcmp(&saved_snapshot, &snapshot, sizeof(snapshot)) == 0);
  CHECK(std::memcmp(&saved_first, &vehicles[0], sizeof(saved_first)) == 0);
}

int main() {
  try {
    accuracy_and_objective();
    degenerate_and_window();
    invalid_input();
    existing_interface_adapter();
    std::puts("RTE self-check passed: accuracy, paper objective, degeneracy, mirror ambiguity, window, ABI, NodeState/UWB/Pose2D.");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "RTE self-check failed: %s\n", error.what());
    return 1;
  }
}

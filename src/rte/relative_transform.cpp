// 论文式(7)、(34)-(43)、(16)：平方测距QCQP的可行参数消元与多初值LM。
// Eigen仅在显式启用RTE扩展时需要；默认库不依赖本文件。
#include "zju_coop/relative_transform_api.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <new>
#include <vector>

namespace {
constexpr double kPi = 3.14159265358979323846;
using State = Eigen::Vector4d;
using Matrix4 = Eigen::Matrix4d;
using Vector3 = Eigen::Vector3d;
using Vector9 = Eigen::Matrix<double, 9, 1>;
using Derivative = Eigen::Matrix<double, 9, 4>;
using Design = Eigen::Matrix<double, Eigen::Dynamic, 9>;

template <class T>
zju_coop_error_code_t header(const T* value, uint32_t version = ZJU_COOP_RTE_ABI_VERSION_V1) {
  if (!value || reinterpret_cast<uintptr_t>(value) % alignof(T)) return ZJU_COOP_INVALID_ARGUMENT;
  if (value->struct_size < sizeof(T)) return ZJU_COOP_STRUCT_SIZE_MISMATCH;
  if (value->abi_version != version) return ZJU_COOP_ABI_MISMATCH;
  return ZJU_COOP_OK;
}

template <class T>
zju_coop_error_code_t initialize(T* value) {
  if (!value || reinterpret_cast<uintptr_t>(value) % alignof(T)) return ZJU_COOP_INVALID_ARGUMENT;
  std::memset(value, 0, sizeof(T));
  value->struct_size = sizeof(T);
  value->abi_version = ZJU_COOP_RTE_ABI_VERSION_V1;
  return ZJU_COOP_OK;
}

template <class T, std::size_t N>
bool zeros(const T (&values)[N]) {
  return std::all_of(std::begin(values), std::end(values), [](T x) { return x == 0; });
}

bool finite_position(const double* data) {
  return Eigen::Map<const Vector3>(data).allFinite() &&
         Eigen::Map<const Vector3>(data).cwiseAbs().maxCoeff() <= 1e5;
}

double wrap(double angle) {
  double result = std::remainder(angle, 2.0 * kPi);
  return result >= kPi ? result - 2.0 * kPi : result;
}

Eigen::Matrix3d rotation(double angle) {
  return Eigen::AngleAxisd(angle, Vector3::UnitZ()).toRotationMatrix();
}

struct Pose {
  uint64_t time{};
  Vector3 position{Vector3::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct Sample {
  uint64_t time{};
  Vector3 host{Vector3::Zero()};
  Vector3 target{Vector3::Zero()};
  double range{};
  double sigma{};
  bool body_valid{};
  Pose host_body;
  Pose target_body;
};

double squared_measurement(const Sample& s) { return s.range * s.range - s.sigma * s.sigma; }
double squared_sigma(const Sample& s) {
  // 式(35)，采用独立测距噪声的对角S；不把它误当成sigma^2。
  return s.sigma * std::sqrt(4.0 * s.range * s.range + 2.0 * s.sigma * s.sigma);
}

Design design_matrix(const std::deque<Sample>& samples) {
  Design b(samples.size(), 9);
  Eigen::Index row = 0;
  for (const auto& sample : samples) {
    const Vector3& p = sample.host;
    const Vector3& o = sample.target;
    b.row(row++) << -2.0 * p.x(), -2.0 * p.y(), 2.0 * (o.z() - p.z()),
        -2.0 * (p.x() * o.x() + p.y() * o.y()),
        2.0 * (p.x() * o.y() - o.x() * p.y()), 2.0 * o.x(), 2.0 * o.y(), 1.0,
        p.squaredNorm() + o.squaredNorm() - 2.0 * p.z() * o.z() - squared_measurement(sample);
    b.row(row - 1) /= squared_sigma(sample);
  }
  return b;
}

Vector9 lifted_state(const State& state) {
  const double c = std::cos(state[3]), s = std::sin(state[3]);
  Vector9 x;
  x << state[0], state[1], state[2], c, s,
      state[0] * c + state[1] * s, state[1] * c - state[0] * s,
      state.head<3>().squaredNorm(), 1.0;
  return x;
}

Derivative lifted_derivative(const State& state) {
  const double c = std::cos(state[3]), s = std::sin(state[3]);
  Derivative d = Derivative::Zero();
  d.topLeftCorner<3, 3>().setIdentity();
  d(3, 3) = -s;
  d(4, 3) = c;
  d.row(5) << c, s, 0.0, -state[0] * s + state[1] * c;
  d.row(6) << -s, c, 0.0, -state[1] * s - state[0] * c;
  d.block<1, 3>(7, 0) = 2.0 * state.head<3>().transpose();
  return d;
}

struct Candidate {
  State state{State::Zero()};
  double cost{std::numeric_limits<double>::max()};
  uint32_t iterations{};
  bool converged{};
};

Candidate optimize(const Design& b, State state, const zju_coop_config_rte_t& config) {
  Candidate out;
  if (!state.allFinite()) return out;
  double damping = 1e-3;
  double cost = 0.5 * (b * lifted_state(state)).squaredNorm();
  for (uint32_t iteration = 0; iteration < config.max_iterations; ++iteration) {
    out.iterations = iteration + 1;
    const Eigen::VectorXd residual = b * lifted_state(state);
    const Eigen::MatrixXd jacobian = b * lifted_derivative(state);
    const State gradient = jacobian.transpose() * residual;
    const Matrix4 normal = jacobian.transpose() * jacobian;
    if (!gradient.allFinite() || !normal.allFinite()) break;
    if (gradient.lpNorm<Eigen::Infinity>() < config.convergence_tol * (1.0 + cost)) {
      out.converged = true;
      break;
    }
    bool accepted = false;
    for (int trial = 0; trial < 16; ++trial) {
      Matrix4 system = normal;
      system.diagonal().array() += damping * normal.diagonal().array().max(1.0);
      const State delta = system.ldlt().solve(-gradient);
      State next = state + delta;
      next[3] = wrap(next[3]);
      const double next_cost = 0.5 * (b * lifted_state(next)).squaredNorm();
      if (delta.allFinite() && next.allFinite() && std::isfinite(next_cost) && next_cost <= cost) {
        const double improvement = cost - next_cost;
        state = next;
        cost = next_cost;
        damping = std::max(1e-12, damping * 0.25);
        accepted = true;
        out.converged = delta.norm() < config.convergence_tol * (1.0 + state.norm()) ||
            (improvement < config.convergence_tol * (1.0 + cost) &&
             delta.norm() < std::sqrt(config.convergence_tol) * (1.0 + state.norm()));
        break;
      }
      damping *= 10.0;
    }
    if (!accepted || out.converged) break;
  }
  out.state = state;
  out.cost = cost;
  return out;
}

std::vector<State> starting_points(const Design& b, const std::deque<Sample>& samples,
                                 uint32_t yaw_starts) {
  std::vector<State> seeds;
  const Eigen::VectorXd linear = b.leftCols<8>().colPivHouseholderQr().solve(-b.col(8));
  if (linear.allFinite()) {
    State initial;
    initial << linear.head<3>(), std::atan2(linear[4], linear[3]);
    seeds.push_back(initial);
  }
  // ponytail: 多初值只搜索局部最优，需全局证书时用论文的Gurobi/SDP求解器替换。
  for (uint32_t i = 0; i < yaw_starts; ++i) {
    const double yaw = -kPi + 2.0 * kPi * i / yaw_starts;
    const Eigen::Matrix3d r = rotation(yaw);
    Eigen::MatrixXd q(samples.size(), 3);
    Eigen::VectorXd rhs(samples.size());
    Eigen::Index row = 0;
    for (const auto& sample : samples) {
      const Vector3 delta = r * sample.target - sample.host;
      q.row(row) = delta.transpose();
      rhs[row++] = squared_measurement(sample) - delta.squaredNorm();
    }
    Eigen::MatrixXd a = 2.0 * (q.rowwise() - q.colwise().mean());
    rhs.array() -= rhs.mean();
    row = 0;
    for (const auto& sample : samples) {
      a.row(row) /= squared_sigma(sample);
      rhs[row++] /= squared_sigma(sample);
    }
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeFullV);
    const Vector3 translation = svd.solve(rhs);
    State initial;
    initial << translation, yaw;
    seeds.push_back(initial);
    // 秩亏时补上沿零空间的正负根，显式发现平面轨迹的高度镜像解。
    if (svd.rank() < 3) {
      const Vector3 null = svd.matrixV().col(2);
      double linear_term = 0.0, constant = 0.0;
      row = 0;
      for (const auto& sample : samples) {
        const Vector3 delta = translation + q.row(row++).transpose();
        linear_term += null.dot(delta);
        constant += delta.squaredNorm() - squared_measurement(sample);
      }
      linear_term /= static_cast<double>(samples.size());
      constant /= static_cast<double>(samples.size());
      const double root = std::sqrt(std::max(0.0, linear_term * linear_term - constant));
      for (double sign : {-1.0, 1.0}) {
        initial.head<3>() = translation + (-linear_term + sign * root) * null;
        seeds.push_back(initial);
      }
    }
  }
  return seeds;
}

zju_coop_estimate_rte_t solve(const std::deque<Sample>& samples,
                             const zju_coop_config_rte_t& config) {
  zju_coop_estimate_rte_t out{};
  initialize(&out);
  out.sample_count = static_cast<uint32_t>(samples.size());
  out.timestamp_ns = samples.empty() ? 0 : samples.back().time;
  if (samples.size() < config.minimum_samples) return out;
  const Design b = design_matrix(samples);
  std::vector<Candidate> candidates;
  for (const auto& seed : starting_points(b, samples, config.yaw_starts)) {
    candidates.push_back(optimize(b, seed, config));
  }
  const auto best = std::min_element(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& c) { return a.cost < c.cost; });
  if (best == candidates.end() || !best->state.allFinite() || !std::isfinite(best->cost)) {
    out.status = ZJU_COOP_RTE_NUMERICAL_FAILURE;
    return out;
  }
  const State& state = best->state;
  out.status = best->converged ? ZJU_COOP_RTE_READY : ZJU_COOP_RTE_NOT_CONVERGED;
  out.iterations = best->iterations;
  Eigen::Map<Vector3>(out.translation_m) = state.head<3>();
  out.yaw_rad = wrap(state[3]);
  out.squared_distance_cost = best->cost;
  Matrix4 fim = Matrix4::Zero();
  const Eigen::Matrix3d r = rotation(state[3]);
  double squared_error = 0.0;
  for (const auto& sample : samples) {
    const Vector3 rotated = r * sample.target;
    const Vector3 w = state.head<3>() + rotated - sample.host;
    const double distance = w.norm();
    squared_error += std::pow(distance - sample.range, 2);
    if (distance < 1e-12) continue;
    State jacobian;
    jacobian << w / distance, w.dot(Vector3::UnitZ().cross(rotated)) / distance;
    fim.noalias() += (jacobian * jacobian.transpose()) / (sample.sigma * sample.sigma);
  }
  out.rms_residual_m = std::sqrt(squared_error / samples.size());
  const Eigen::SelfAdjointEigenSolver<Matrix4> eigen(fim);
  out.condition_number = std::numeric_limits<double>::max();
  if (eigen.info() != Eigen::Success || eigen.eigenvalues()[3] <= 0.0 ||
      eigen.eigenvalues()[0] <= eigen.eigenvalues()[3] / config.condition_limit) {
    out.status = ZJU_COOP_RTE_UNOBSERVABLE;
    return out;
  }
  out.condition_number = eigen.eigenvalues()[3] / eigen.eigenvalues()[0];
  const Matrix4 covariance = eigen.eigenvectors() * eigen.eigenvalues().cwiseInverse().asDiagonal() *
                             eigen.eigenvectors().transpose();
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(out.covariance) = covariance;
  out.covariance_valid = ZJU_COOP_TRUE;
  for (const auto& candidate : candidates) {
    if (candidate.converged && std::abs(candidate.cost - best->cost) <= 1e-6 * (1.0 + best->cost) &&
        ((candidate.state.head<3>() - state.head<3>()).norm() > 0.05 ||
         std::abs(wrap(candidate.state[3] - state[3])) > 0.05)) {
      out.status = ZJU_COOP_RTE_AMBIGUOUS;
    }
  }
  return out;
}

uint64_t difference(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }

bool at_time(const std::deque<Pose>& poses, uint64_t time, uint64_t tolerance, Pose& out) {
  if (poses.empty()) return false;
  auto next = std::lower_bound(poses.begin(), poses.end(), time,
      [](const Pose& pose, uint64_t t) { return pose.time < t; });
  if (next != poses.begin() && next != poses.end() && next->time != time &&
      difference(next->time, time) <= tolerance && difference((next - 1)->time, time) <= tolerance) {
    const auto& previous = *(next - 1);
    const double fraction = static_cast<double>(time - previous.time) / (next->time - previous.time);
    out.position = previous.position + fraction * (next->position - previous.position);
    out.orientation = previous.orientation.slerp(fraction, next->orientation);
    out.time = time;
    return true;
  }
  auto nearest = next == poses.end() ? poses.end() - 1 : next;
  if (next != poses.begin() && difference((next - 1)->time, time) < difference(nearest->time, time)) --nearest;
  if (difference(nearest->time, time) > tolerance) return false;
  out = *nearest;
  out.time = time;
  return true;
}
}  // namespace

struct zju_coop_rte_handle {
  zju_coop_config_rte_t config{};
  std::deque<Sample> samples;
  std::array<std::deque<Pose>, 2> poses;
  zju_coop_estimate_rte_t estimate{};
  bool estimate_current{};
};

namespace {
void append(zju_coop_rte_handle_t& handle, const Sample& sample) {
  handle.samples.push_back(sample);
  if (handle.samples.size() > handle.config.max_samples) handle.samples.pop_front();
  handle.estimate_current = false;
}
}  // namespace

extern "C" {
uint32_t ZJU_COOP_CALL zju_coop_abi_version_rte(void) { return ZJU_COOP_RTE_ABI_VERSION_V1; }
const char* ZJU_COOP_CALL zju_coop_version_string_rte(void) { return "1.1.0-rte-qcqp-local"; }

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_config_init_rte(zju_coop_config_rte_t* value) {
  const auto error = initialize(value);
  if (error != ZJU_COOP_OK) return error;
  value->host_node_id = 1;
  value->target_node_id = 2;
  value->max_samples = 512;
  value->minimum_samples = 12;
  value->max_iterations = 100;
  value->yaw_starts = 24;
  value->max_epoch_skew_ns = 20000000;
  value->range_std_m = 0.1;
  value->convergence_tol = 1e-9;
  value->condition_limit = 1e10;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_sample_init_rte(zju_coop_sample_rte_t* value) {
  return initialize(value);
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_estimate_init_rte(zju_coop_estimate_rte_t* value) {
  return initialize(value);
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_create_rte(
    const zju_coop_config_rte_t* config, zju_coop_rte_handle_t** out_handle) {
  if (!out_handle) return ZJU_COOP_INVALID_ARGUMENT;
  *out_handle = nullptr;
  const auto error = header(config);
  if (error != ZJU_COOP_OK) return error;
  if (config->host_node_id == config->target_node_id || config->host_node_id > 65535 ||
      config->target_node_id > 65535 || config->minimum_samples < 8 ||
      config->max_samples < config->minimum_samples || config->max_samples > 4096 ||
      config->max_iterations == 0 || config->max_iterations > 1000 ||
      config->yaw_starts < 4 || config->yaw_starts > 128 ||
      config->max_epoch_skew_ns > 1000000000 ||
      !std::isfinite(config->range_std_m) || config->range_std_m < 1e-6 || config->range_std_m > 1e3 ||
      !std::isfinite(config->convergence_tol) || config->convergence_tol < 1e-14 || config->convergence_tol > 1e-2 ||
      !std::isfinite(config->condition_limit) || config->condition_limit < 1.0 || config->condition_limit > 1e14 ||
      !finite_position(config->host_lever_arm_body_m) || !finite_position(config->target_lever_arm_body_m)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    auto handle = std::make_unique<zju_coop_rte_handle_t>();
    handle->config = *config;
    *out_handle = handle.release();
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_destroy_rte(zju_coop_rte_handle_t* handle) {
  delete handle;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_sample_rte(
    zju_coop_rte_handle_t* handle, const zju_coop_sample_rte_t* sample) {
  if (!handle) return ZJU_COOP_INVALID_ARGUMENT;
  const auto error = header(sample);
  if (error != ZJU_COOP_OK) return error;
  if (!sample->timestamp_ns || !finite_position(sample->host_antenna_position_m) ||
      !finite_position(sample->target_antenna_position_m) || !std::isfinite(sample->range_m) ||
      sample->range_m <= 0.0 || sample->range_m > 2e5 || !std::isfinite(sample->range_std_m) ||
      sample->range_std_m < 0.0 || sample->range_std_m > 1e3 ||
      (sample->range_std_m > 0.0 && sample->range_std_m < 1e-6) ||
      (!handle->samples.empty() && sample->timestamp_ns <= handle->samples.back().time)) {
    return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    Sample converted;
    converted.time = sample->timestamp_ns;
    converted.host = Eigen::Map<const Vector3>(sample->host_antenna_position_m);
    converted.target = Eigen::Map<const Vector3>(sample->target_antenna_position_m);
    converted.range = sample->range_m;
    converted.sigma = sample->range_std_m == 0.0 ? handle->config.range_std_m : sample->range_std_m;
    append(*handle, converted);
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_step_rte(
    zju_coop_rte_handle_t* handle, zju_coop_estimate_rte_t* estimate) {
  if (!handle) return ZJU_COOP_INVALID_ARGUMENT;
  const auto error = header(estimate);
  if (error != ZJU_COOP_OK) return error;
  try {
    auto result = solve(handle->samples, handle->config);
    handle->estimate = result;
    handle->estimate_current = true;
    result.struct_size = estimate->struct_size;
    *estimate = result;
    return handle->samples.size() < handle->config.minimum_samples ? ZJU_COOP_NOT_READY : ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_node_state_rte(
    zju_coop_rte_handle_t* handle, const zju_coop_node_state_t* state) {
  if (!handle) return ZJU_COOP_INVALID_ARGUMENT;
  const auto error = header(state, ZJU_COOP_ABI_VERSION_V1);
  if (error != ZJU_COOP_OK) return error;
  const int index = state->node_id == handle->config.host_node_id ? 0 :
                    state->node_id == handle->config.target_node_id ? 1 : -1;
  if (index < 0 || !state->timestamp_ns || state->valid != ZJU_COOP_TRUE || state->reserved0 ||
      !zeros(state->reserved1) || !finite_position(state->position_enu_m) ||
      !Eigen::Map<const Vector3>(state->velocity_enu_mps).allFinite() ||
      !Eigen::Map<const State>(state->orientation_flu_to_enu_xyzw).allFinite()) return ZJU_COOP_INVALID_ARGUMENT;
  Eigen::Quaterniond q(state->orientation_flu_to_enu_xyzw[3], state->orientation_flu_to_enu_xyzw[0],
                       state->orientation_flu_to_enu_xyzw[1], state->orientation_flu_to_enu_xyzw[2]);
  if (std::abs(q.norm() - 1.0) > 1e-3) return ZJU_COOP_INVALID_ARGUMENT;
  auto& poses = handle->poses[static_cast<std::size_t>(index)];
  if (!poses.empty() && state->timestamp_ns <= poses.back().time) return ZJU_COOP_INVALID_ARGUMENT;
  try {
    poses.push_back({state->timestamp_ns, Eigen::Map<const Vector3>(state->position_enu_m), q.normalized()});
    if (poses.size() > handle->config.max_samples) poses.pop_front();
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_push_range_rte(
    zju_coop_rte_handle_t* handle, const zju_coop_range_packet_t* packet,
    zju_coop_range_processing_result_t* result) {
  if (!handle) return ZJU_COOP_INVALID_ARGUMENT;
  auto error = header(packet, ZJU_COOP_ABI_VERSION_V1);
  if (error != ZJU_COOP_OK) return error;
  error = header(result, ZJU_COOP_ABI_VERSION_V1);
  if (error != ZJU_COOP_OK) return error;
  zju_coop_range_processing_result_t receipt{};
  initialize(&receipt);
  receipt.struct_size = result->struct_size;
  receipt.from_node = std::min(packet->from_node, packet->to_node);
  receipt.to_node = std::max(packet->from_node, packet->to_node);
  receipt.disposition = ZJU_COOP_PROCESSING_REJECTED;
  receipt.fusion_action = ZJU_COOP_FUSION_REJECT;
  receipt.update_disposition = ZJU_COOP_UPDATE_INVALID_PACKET;
  receipt.covariance_scale = 1.0;
  if (receipt.from_node != std::min(handle->config.host_node_id, handle->config.target_node_id) ||
      receipt.to_node != std::max(handle->config.host_node_id, handle->config.target_node_id)) {
    receipt.update_disposition = ZJU_COOP_UPDATE_UNKNOWN_NODE;
  } else if (!packet->timestamp_ns || !std::isfinite(packet->range_m) || packet->range_m <= 0.0 ||
      packet->range_m > 2e5 || !std::isfinite(packet->range_std_m) ||
      packet->range_std_m < 1e-6 || packet->range_std_m > 1e3 ||
      packet->valid != ZJU_COOP_TRUE || packet->reserved0 || packet->nlos_flag ||
      packet->has_nlos_probability > 1 || packet->status != ZJU_COOP_RANGE_STATUS_OK ||
      (packet->has_nlos_probability && (!std::isfinite(packet->nlos_probability) ||
        packet->nlos_probability < 0.0F || packet->nlos_probability > 1.0F || packet->nlos_probability >= 0.5F))) {
    receipt.disposition = ZJU_COOP_PROCESSING_INVALID_PACKET;
  } else if (!handle->samples.empty() && packet->timestamp_ns <= handle->samples.back().time) {
    receipt.disposition = packet->timestamp_ns == handle->samples.back().time ?
        ZJU_COOP_PROCESSING_DUPLICATE : ZJU_COOP_PROCESSING_OUT_OF_ORDER;
    receipt.update_disposition = ZJU_COOP_UPDATE_OUT_OF_ORDER;
  } else {
    Pose host, target;
    if (!at_time(handle->poses[0], packet->timestamp_ns, handle->config.max_epoch_skew_ns, host) ||
        !at_time(handle->poses[1], packet->timestamp_ns, handle->config.max_epoch_skew_ns, target)) {
      receipt.disposition = ZJU_COOP_PROCESSING_HELD;
      receipt.fusion_action = ZJU_COOP_FUSION_HOLD;
    } else {
      zju_coop_sample_rte_t sample{};
      initialize(&sample);
      sample.timestamp_ns = packet->timestamp_ns;
      Eigen::Map<Vector3>(sample.host_antenna_position_m) = host.position + host.orientation *
          Eigen::Map<const Vector3>(handle->config.host_lever_arm_body_m);
      Eigen::Map<Vector3>(sample.target_antenna_position_m) = target.position + target.orientation *
          Eigen::Map<const Vector3>(handle->config.target_lever_arm_body_m);
      sample.range_m = packet->range_m;
      sample.range_std_m = packet->range_std_m;
      error = zju_coop_push_sample_rte(handle, &sample);
      if (error != ZJU_COOP_OK) return error;
      handle->samples.back().host_body = host;
      handle->samples.back().target_body = target;
      handle->samples.back().body_valid = true;
      receipt.disposition = ZJU_COOP_PROCESSING_PROCESSED;
      receipt.fusion_action = ZJU_COOP_FUSION_USE_NORMAL;
      receipt.update_disposition = ZJU_COOP_UPDATE_ACCEPTED;
      // 入窗不等于滤波更新；优化只由step执行，故filter_updated保持false。
    }
  }
  *result = receipt;
  return ZJU_COOP_OK;
}

zju_coop_error_code_t ZJU_COOP_CALL zju_coop_get_pose2d_v2_rte(
    zju_coop_rte_handle_t* handle, zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t vehicle_capacity,
    uint32_t vehicle_stride, uint32_t* vehicle_count) {
  if (!handle || !vehicle_count) return ZJU_COOP_INVALID_ARGUMENT;
  auto error = header(snapshot, ZJU_COOP_POSE2D_ABI_VERSION_V2);
  if (error != ZJU_COOP_OK) return error;
  *vehicle_count = 2;
  if (!vehicles && !vehicle_capacity && !vehicle_stride) return ZJU_COOP_BUFFER_TOO_SMALL;
  if (!vehicles || vehicle_stride < sizeof(*vehicles) ||
      vehicle_stride % alignof(zju_coop_vehicle_pose2d_v2_t) ||
      reinterpret_cast<uintptr_t>(vehicles) % alignof(zju_coop_vehicle_pose2d_v2_t)) return ZJU_COOP_INVALID_ARGUMENT;
  if (vehicle_capacity < 2) return ZJU_COOP_BUFFER_TOO_SMALL;
  auto* second = reinterpret_cast<zju_coop_vehicle_pose2d_v2_t*>(
      reinterpret_cast<unsigned char*>(vehicles) + vehicle_stride);
  for (auto* item : {vehicles, second}) {
    error = header(item, ZJU_COOP_POSE2D_ABI_VERSION_V2);
    if (error != ZJU_COOP_OK) return error;
    if (item->struct_size > vehicle_stride || item->reserved0 || !zeros(item->reserved)) return ZJU_COOP_INVALID_ARGUMENT;
  }
  if (snapshot->reserved0) return ZJU_COOP_INVALID_ARGUMENT;
  if (!handle->estimate_current || handle->estimate.status != ZJU_COOP_RTE_READY ||
      handle->samples.empty() || !handle->samples.back().body_valid) return ZJU_COOP_NOT_READY;
  const auto& sample = handle->samples.back();
  const auto& estimate = handle->estimate;
  const Vector3 relative = Eigen::Map<const Vector3>(estimate.translation_m) +
      rotation(estimate.yaw_rad) * sample.target_body.position - sample.host_body.position;
  zju_coop_pose2d_snapshot_v2_t next{};
  initialize(&next);
  next.abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
  next.struct_size = snapshot->struct_size;
  next.timestamp_ns = estimate.timestamp_ns;
  next.reference_node_id = handle->config.host_node_id;
  std::snprintf(next.frame_id, sizeof(next.frame_id), "coop_rte_%u_local", next.reference_node_id);
  std::array<zju_coop_vehicle_pose2d_v2_t, 2> output{};
  for (std::size_t i = 0; i < 2; ++i) {
    initialize(&output[i]);
    output[i].abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
    output[i].struct_size = (i == 0 ? vehicles : second)->struct_size;
    output[i].node_id = i == 0 ? handle->config.host_node_id : handle->config.target_node_id;
    const auto& q = i == 0 ? sample.host_body.orientation : sample.target_body.orientation;
    const auto body_rotation = q.toRotationMatrix();
    const bool heading_defined = std::hypot(body_rotation(1, 0), body_rotation(0, 0)) > 1e-8;
    output[i].yaw_rad = heading_defined ? wrap(std::atan2(body_rotation(1, 0), body_rotation(0, 0)) +
                             (i == 0 ? 0.0 : estimate.yaw_rad)) : 0.0;
    output[i].position_valid = ZJU_COOP_TRUE;
    output[i].yaw_valid = heading_defined ? ZJU_COOP_TRUE : ZJU_COOP_FALSE;
  }
  output[1].x_m = relative.x();
  output[1].y_m = relative.y();
  *snapshot = next;
  *vehicles = output[0];
  *second = output[1];
  return ZJU_COOP_OK;
}
}  // extern "C"

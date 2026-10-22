#include "zju_coop/three_vehicle_rte_api.h"
#include "rte_detail.hpp"
#include <array>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <stdexcept>
#include <vector>

namespace {
using namespace zju::coop::rte_detail;
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
constexpr std::array<std::array<int, 2>, 3> kEdges{{{0, 1}, {0, 2}, {1, 2}}};
struct Transform {
  Vector3 translation{Vector3::Zero()};
  double yaw{};
  Vector3 point(const Vector3& p) const { return translation + rotation(yaw) * p; }
};
Transform compose(const Transform& a, const Transform& b) {
  return {a.point(b.translation), wrap(a.yaw + b.yaw)};
}
Transform inverse(const Transform& t) {
  return {-rotation(-t.yaw) * t.translation, wrap(-t.yaw)};
}
struct Observation {
  uint64_t time{};
  int edge{};
  Vector3 from{Vector3::Zero()}, to{Vector3::Zero()};
  double range{}, sigma{};
};
struct Edge {
  zju_coop_rte_handle_t* pair{};
  std::deque<Observation> samples;
  std::map<uint64_t, zju_coop_range_packet_t> pending;
  ~Edge() { zju_coop_destroy_rte(pair); }
};
struct Evaluation {
  Vector residual;
  Matrix jacobian;
  double squared_range_error{};
};
struct Fit {
  Vector state;
  double cost{std::numeric_limits<double>::max()};
  bool converged{};
};
}  // namespace

struct zju_coop_three_handle_rte {
  zju_coop_three_config_rte_t config{};
  std::array<std::deque<Pose>, 3> poses;
  std::array<Edge, 3> edges;
  std::array<Transform, 3> transforms;
  Vector state;
  Matrix covariance;
  bool initialized{}, dirty{};
  zju_coop_rte_status_t quality{ZJU_COOP_RTE_NO_ESTIMATE};
  uint64_t attempted_at{}, initialized_at{}, corrected_at{}, corrections{}, accepted{}, rejected{};
  double rms{}, condition{};
};

namespace {
int node_index(const zju_coop_three_handle_rte_t& h, uint32_t id) {
  for (int i = 0; i < 3; ++i) if (h.config.node_ids[i] == id) return i;
  return -1;
}
int edge_index(const zju_coop_three_handle_rte_t& h, uint32_t from, uint32_t to) {
  int a = node_index(h, from), b = node_index(h, to);
  if (a < 0 || b < 0 || a == b) return -1;
  if (a > b) std::swap(a, b);
  for (int e = 0; e < 3; ++e) if (kEdges[e][0] == a && kEdges[e][1] == b) return e;
  return -1;
}
int block_size(const zju_coop_three_handle_rte_t& h) { return h.config.planar ? 3 : 4; }
std::array<Transform, 3> unpack(const zju_coop_three_handle_rte_t& h, const Vector& state) {
  std::array<Transform, 3> out;
  const int size = block_size(h);
  for (int n = 1; n < 3; ++n) {
    out[n].translation.head<2>() = state.segment<2>((n - 1) * size);
    out[n].translation.z() = h.config.planar ?
        h.config.local_origin_heights_m[n] - h.config.local_origin_heights_m[0] : state[(n - 1) * size + 2];
    out[n].yaw = state[n * size - 1];
  }
  return out;
}
Vector pack(const zju_coop_three_handle_rte_t& h, const std::array<Transform, 3>& transforms) {
  const int size = block_size(h);
  Vector out(2 * size);
  for (int n = 1; n < 3; ++n) {
    out.segment((n - 1) * size, size - 1) = transforms[n].translation.head(size - 1);
    out[n * size - 1] = transforms[n].yaw;
  }
  return out;
}
Evaluation evaluate(const zju_coop_three_handle_rte_t& h, const std::vector<Observation>& data,
                    const Vector& state, bool range_fim = false) {
  const auto transforms = unpack(h, state);
  const int size = block_size(h);
  Evaluation out{Vector(data.size()), Matrix::Zero(data.size(), state.size()), 0.0};
  Eigen::Index row = 0;
  for (const auto& s : data) {
    const int i = kEdges[s.edge][0], j = kEdges[s.edge][1];
    const Vector3 ri = rotation(transforms[i].yaw) * s.from;
    const Vector3 rj = rotation(transforms[j].yaw) * s.to;
    const Vector3 w = transforms[j].translation + rj - transforms[i].translation - ri;
    const double distance = w.norm();
    out.squared_range_error += std::pow(distance - s.range, 2);
    const double sd_sigma = s.sigma * std::sqrt(4 * s.range * s.range + 2 * s.sigma * s.sigma);
    const double scale = range_fim ? 1.0 / (std::max(distance, 1e-12) * s.sigma) : 2.0 / sd_sigma;
    out.residual[row] = range_fim ? (distance - s.range) / s.sigma :
        (w.squaredNorm() - s.range * s.range + s.sigma * s.sigma) / sd_sigma;
    for (int n : {i, j}) {
      if (n == 0) continue;
      const double sign = n == j ? 1.0 : -1.0;
      out.jacobian.block(row, (n - 1) * size, 1, size - 1) = sign * scale * w.head(size - 1).transpose();
      out.jacobian(row, n * size - 1) = sign * scale * w.dot(Vector3::UnitZ().cross(n == i ? ri : rj));
    }
    ++row;
  }
  return out;
}
Fit optimize(const zju_coop_three_handle_rte_t& h, const std::vector<Observation>& data, Vector state) {
  Fit out;
  const int size = block_size(h);
  double damping = 1e-3;
  double cost = 0.5 * evaluate(h, data, state).residual.squaredNorm();
  for (uint32_t iteration = 0; iteration < h.config.max_iterations; ++iteration) {
    const auto e = evaluate(h, data, state);
    const Matrix normal = e.jacobian.transpose() * e.jacobian;
    const Vector gradient = e.jacobian.transpose() * e.residual;
    if (!normal.allFinite() || !gradient.allFinite()) break;
    if (gradient.lpNorm<Eigen::Infinity>() < 1e-9 * (1 + cost)) { out.converged = true; break; }
    bool accepted = false;
    for (int attempt = 0; attempt < 16; ++attempt) {
      Matrix system = normal;
      system.diagonal().array() += damping * normal.diagonal().array().max(1.0);
      const Vector delta = system.ldlt().solve(-gradient);
      Vector next = state + delta;
      for (int n = 1; n < 3; ++n) next[n * size - 1] = wrap(next[n * size - 1]);
      const double next_cost = 0.5 * evaluate(h, data, next).residual.squaredNorm();
      if (delta.allFinite() && std::isfinite(next_cost) && next_cost <= cost) {
        const double change = cost - next_cost;
        state = next;
        cost = next_cost;
        damping = std::max(1e-12, damping * 0.25);
        accepted = true;
        out.converged = delta.norm() < 1e-9 * (1 + state.norm()) ||
            (change < 1e-9 * (1 + cost) && delta.norm() < 1e-5 * (1 + state.norm()));
        break;
      }
      damping *= 10;
    }
    if (!accepted || out.converged) break;
  }
  out.state = state;
  out.cost = cost;
  return out;
}
bool fresh(uint64_t time, uint64_t now, uint64_t timeout, uint64_t skew) {
  return time && (time > now ? time - now <= skew : now - time <= timeout);
}
std::vector<Vector> seeds(zju_coop_three_handle_rte_t& h) {
  if (h.initialized) return {h.state};
  std::array<Transform, 3> pair;
  std::array<bool, 3> available{};
  for (int e = 0; e < 3; ++e) {
    zju_coop_estimate_rte_t out;
    zju_coop_estimate_init_rte(&out);
    const auto rc = h.config.planar ? zju_coop_step_planar_rte(h.edges[e].pair,
        h.config.local_origin_heights_m[kEdges[e][1]] - h.config.local_origin_heights_m[kEdges[e][0]], &out) :
        zju_coop_step_rte(h.edges[e].pair, &out);
    available[e] = rc == ZJU_COOP_OK && out.status != ZJU_COOP_RTE_NUMERICAL_FAILURE;
    if (available[e]) pair[e] = {Eigen::Map<Vector3>(out.translation_m), out.yaw_rad};
  }
  std::vector<Vector> result;
  if (available[0] && available[1]) result.push_back(pack(h, {Transform{}, pair[0], pair[1]}));
  if (available[0] && available[2]) result.push_back(pack(h, {Transform{}, pair[0], compose(pair[0], pair[2])}));
  if (available[1] && available[2]) result.push_back(pack(h, {Transform{}, compose(pair[1], inverse(pair[2])), pair[1]}));
  return result;
}
void correct(zju_coop_three_handle_rte_t& h, uint64_t now) {
  std::vector<Observation> data;
  for (auto& edge : h.edges)
    for (const auto& sample : edge.samples)
      if (fresh(sample.time, now, h.config.window_duration_ns, h.config.max_epoch_skew_ns)) data.push_back(sample);
  if (data.size() < 2 * h.config.minimum_samples) { h.quality = ZJU_COOP_RTE_NO_ESTIMATE; return; }
  std::vector<Fit> fits;
  for (const auto& seed : seeds(h)) fits.push_back(optimize(h, data, seed));
  if (fits.empty()) { h.quality = ZJU_COOP_RTE_NO_ESTIMATE; return; }
  const auto best = std::min_element(fits.begin(), fits.end(), [](const Fit& a, const Fit& b) { return a.cost < b.cost; });
  if (!best->converged || !best->state.allFinite()) { h.quality = ZJU_COOP_RTE_NOT_CONVERGED; return; }
  const auto e = evaluate(h, data, best->state, true);
  const Matrix fim = e.jacobian.transpose() * e.jacobian;
  const Eigen::SelfAdjointEigenSolver<Matrix> eigen(fim);
  if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <=
      eigen.eigenvalues().maxCoeff() / h.config.condition_limit) {
    h.quality = ZJU_COOP_RTE_UNOBSERVABLE;
    return;
  }
  const double rms = std::sqrt(e.squared_range_error / data.size());
  if (rms > h.config.max_rms_residual_m) { h.quality = ZJU_COOP_RTE_RESIDUAL_REJECTED; return; }
  const Matrix covariance = eigen.eigenvectors() * eigen.eigenvalues().cwiseInverse().asDiagonal() * eigen.eigenvectors().transpose();
  if (!h.initialized) {
    const int size = block_size(h);
    for (int n = 1; n < 3; ++n) {
      for (int axis = 0; axis < size; ++axis) {
        const double limit = axis == size - 1 ? h.config.max_yaw_std_rad : h.config.max_translation_std_m;
        if (covariance((n - 1) * size + axis, (n - 1) * size + axis) > limit * limit) {
          h.quality = ZJU_COOP_RTE_UNOBSERVABLE;
          return;
        }
      }
    }
  }
  for (const auto& fit : fits) {
    Vector delta = fit.state - best->state;
    const int size = block_size(h);
    for (int n = 1; n < 3; ++n) delta[n * size - 1] = wrap(delta[n * size - 1]);
    if (fit.converged && std::abs(fit.cost - best->cost) < 1e-6 * (1 + best->cost) && delta.norm() > 0.05) {
      h.quality = ZJU_COOP_RTE_AMBIGUOUS;
      return;
    }
  }
  h.state = best->state;
  h.transforms = unpack(h, h.state);
  h.covariance = covariance;
  h.condition = eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
  h.rms = rms;
  h.quality = ZJU_COOP_RTE_READY;
  if (!h.initialized) h.initialized_at = now;
  h.initialized = true;
  h.corrected_at = now;
  ++h.corrections;
}
// 返回PROCESSED、HELD或REJECTED；三车数据与双车初值缓存一起提交。
zju_coop_processing_disposition_t consume(zju_coop_three_handle_rte_t& h, int e,
                                        const zju_coop_range_packet_t& packet) {
  const int i = kEdges[e][0], j = kEdges[e][1];
  Pose a, b;
  if (!at_time(h.poses[i], packet.timestamp_ns, h.config.max_epoch_skew_ns, a) ||
      !at_time(h.poses[j], packet.timestamp_ns, h.config.max_epoch_skew_ns, b)) return ZJU_COOP_PROCESSING_HELD;
  const Vector3 from = a.position + a.orientation * Eigen::Map<const Vector3>(h.config.antenna_lever_arms_body_m + 3 * i);
  const Vector3 to = b.position + b.orientation * Eigen::Map<const Vector3>(h.config.antenna_lever_arms_body_m + 3 * j);
  if (h.initialized && std::abs((h.transforms[j].point(to) - h.transforms[i].point(from)).norm() - packet.range_m) >
      h.config.max_range_innovation_m) return ZJU_COOP_PROCESSING_REJECTED;
  zju_coop_sample_rte_t sample;
  zju_coop_sample_init_rte(&sample);
  sample.timestamp_ns = packet.timestamp_ns;
  Eigen::Map<Vector3>(sample.host_antenna_position_m) = from;
  Eigen::Map<Vector3>(sample.target_antenna_position_m) = to;
  sample.range_m = packet.range_m;
  sample.range_std_m = packet.range_std_m;
  // 先分配容器，异常时不会出现双车缓存已入窗而三车缓存丢失。
  h.edges[e].samples.push_back({packet.timestamp_ns, e, from, to, packet.range_m, packet.range_std_m});
  const auto rc = zju_coop_push_sample_rte(h.edges[e].pair, &sample);
  if (rc != ZJU_COOP_OK) {
    h.edges[e].samples.pop_back();
    if (rc == ZJU_COOP_OUT_OF_MEMORY) throw std::bad_alloc();
    return ZJU_COOP_PROCESSING_REJECTED;
  }
  if (h.edges[e].samples.size() > h.config.max_samples_per_edge) h.edges[e].samples.pop_front();
  h.dirty = true;
  ++h.accepted;
  return ZJU_COOP_PROCESSING_PROCESSED;
}
void drain(zju_coop_three_handle_rte_t& h, uint64_t now) {
  for (int e = 0; e < 3; ++e) {
    auto& edge = h.edges[e];
    for (auto it = edge.pending.begin(); it != edge.pending.end();) {
      const auto time = it->first;
      if ((!edge.samples.empty() && time <= edge.samples.back().time) ||
          !fresh(time, now, h.config.node_timeout_ns, h.config.max_epoch_skew_ns)) {
        ++h.rejected;
        it = edge.pending.erase(it);
        continue;
      }
      const auto status = consume(h, e, it->second);
      if (status == ZJU_COOP_PROCESSING_HELD) { ++it; continue; }
      if (status != ZJU_COOP_PROCESSING_PROCESSED) ++h.rejected;
      it = edge.pending.erase(it);
    }
  }
}
zju_coop_three_snapshot_rte_t view(const zju_coop_three_handle_rte_t& h, uint64_t now) {
  zju_coop_three_snapshot_rte_t out;
  initialize(&out);
  std::copy(std::begin(h.config.node_ids), std::end(h.config.node_ids), out.node_ids);
  out.status = h.quality;
  out.initialized_at_ns = h.initialized_at;
  out.last_correction_ns = h.corrected_at;
  out.correction_count = h.corrections;
  out.accepted_ranges = h.accepted;
  out.rejected_ranges = h.rejected;
  out.rms_residual_m = h.rms;
  out.condition_number = h.condition;
  std::array<bool, 3> reachable{true, false, false};
  for (int e = 0; e < 3; ++e) {
    const auto& samples = h.edges[e].samples;
    out.sample_counts[e] = static_cast<uint32_t>(samples.size());
    if (!samples.empty() && fresh(samples.back().time, now, h.config.range_timeout_ns, h.config.max_epoch_skew_ns)) ++out.active_edges;
  }
  for (int pass = 0; pass < 3; ++pass)
    for (int e = 0; e < 3; ++e) {
      const auto& samples = h.edges[e].samples;
      if (samples.empty() || !fresh(samples.back().time, now, h.config.range_timeout_ns, h.config.max_epoch_skew_ns)) continue;
      const int i = kEdges[e][0], j = kEdges[e][1];
      reachable[i] = reachable[j] = reachable[i] || reachable[j];
    }
  uint64_t common = std::numeric_limits<uint64_t>::max();
  bool nodes_fresh = true;
  for (const auto& poses : h.poses) {
    if (poses.empty()) { nodes_fresh = false; common = 0; break; }
    common = std::min(common, poses.back().time);
    nodes_fresh = nodes_fresh && fresh(poses.back().time, now, h.config.node_timeout_ns, h.config.max_epoch_skew_ns);
  }
  out.timestamp_ns = common;
  if (h.initialized && (!nodes_fresh || !reachable[1] || !reachable[2])) out.status = ZJU_COOP_RTE_STALE;
  std::array<Pose, 3> aligned;
  if (common) {
    for (int i = 0; i < 3; ++i)
      if (!at_time(h.poses[i], common, h.config.max_epoch_skew_ns, aligned[i])) {
        if (h.initialized) out.status = ZJU_COOP_RTE_STALE;
        out.timestamp_ns = 0;
        return out;
      }
  }
  if (!h.initialized || !common) return out;
  const Vector3 reference = aligned[0].position;
  for (int i = 0; i < 3; ++i) {
    Eigen::Map<Vector3>(out.origin_translation_m + 3 * i) = h.transforms[i].translation;
    out.origin_yaw_rad[i] = h.transforms[i].yaw;
    Eigen::Map<Vector3>(out.relative_position_m + 3 * i) = h.transforms[i].point(aligned[i].position) - reference;
    Eigen::Map<Vector3>(out.relative_velocity_mps + 3 * i) = rotation(h.transforms[i].yaw) * aligned[i].velocity - aligned[0].velocity;
    const Eigen::Quaterniond q = Eigen::Quaterniond(rotation(h.transforms[i].yaw)) * aligned[i].orientation;
    std::copy(q.coeffs().data(), q.coeffs().data() + 4, out.body_orientation_xyzw + 4 * i);
    if (i) {
      const int size = block_size(h);
      for (int axis = 0; axis < size - 1; ++axis) out.position_std_m[3 * i + axis] =
          std::sqrt(std::max(0.0, h.covariance((i - 1) * size + axis, (i - 1) * size + axis)));
      out.yaw_std_rad[i] = std::sqrt(std::max(0.0, h.covariance(i * size - 1, i * size - 1)));
    }
  }
  return out;
}
}  // namespace

extern "C" {
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_config_init_rte(zju_coop_three_config_rte_t* c) {
  const auto rc = initialize(c);
  if (rc != ZJU_COOP_OK) return rc;
  c->node_ids[0] = 201; c->node_ids[1] = 202; c->node_ids[2] = 203;
  c->planar = 1;
  c->max_samples_per_edge = 128; c->minimum_samples = 12;
  c->max_iterations = 80; c->yaw_starts = 24;
  c->max_epoch_skew_ns = 50000000;
  c->node_timeout_ns = 300000000; c->range_timeout_ns = 500000000;
  c->solve_interval_ns = 200000000; c->window_duration_ns = 5000000000ULL;
  c->condition_limit = 1e10; c->max_rms_residual_m = 0.3; c->max_range_innovation_m = 1.0;
  c->max_translation_std_m = 0.25; c->max_yaw_std_rad = 0.1;
  return ZJU_COOP_OK;
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_snapshot_init_rte(zju_coop_three_snapshot_rte_t* s) {
  return initialize(s);
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_create_rte(
    const zju_coop_three_config_rte_t* config, zju_coop_three_handle_rte_t** output) {
  if (!output) return ZJU_COOP_INVALID_ARGUMENT;
  *output = nullptr;
  const auto rc = header(config);
  if (rc != ZJU_COOP_OK) return rc;
  for (uint32_t id : config->node_ids) if (!id || id > 65535) return ZJU_COOP_INVALID_ARGUMENT;
  if (config->node_ids[0] == config->node_ids[1] || config->node_ids[0] == config->node_ids[2] ||
      config->node_ids[1] == config->node_ids[2] || config->planar > 1 ||
      config->minimum_samples < 8 || config->minimum_samples > config->max_samples_per_edge ||
      config->max_samples_per_edge > 1024 || !config->max_iterations || config->max_iterations > 1000 ||
      config->yaw_starts < 4 || config->yaw_starts > 128 || config->max_epoch_skew_ns > 1000000000 ||
      !config->node_timeout_ns || !config->range_timeout_ns || !config->solve_interval_ns ||
      config->window_duration_ns < config->range_timeout_ns ||
      !std::isfinite(config->condition_limit) || config->condition_limit <= 1 || config->condition_limit > 1e14 ||
      !std::isfinite(config->max_rms_residual_m) || config->max_rms_residual_m <= 0 ||
      !std::isfinite(config->max_range_innovation_m) || config->max_range_innovation_m <= 0 ||
      !std::isfinite(config->max_translation_std_m) || config->max_translation_std_m <= 0 ||
      !std::isfinite(config->max_yaw_std_rad) || config->max_yaw_std_rad <= 0 ||
      !finite_position(config->local_origin_heights_m)) return ZJU_COOP_INVALID_ARGUMENT;
  for (int i = 0; i < 3; ++i) if (!finite_position(config->antenna_lever_arms_body_m + 3 * i)) return ZJU_COOP_INVALID_ARGUMENT;
  try {
    auto h = std::make_unique<zju_coop_three_handle_rte_t>();
    h->config = *config;
    for (int e = 0; e < 3; ++e) {
      zju_coop_config_rte_t c;
      zju_coop_config_init_rte(&c);
      c.host_node_id = config->node_ids[kEdges[e][0]]; c.target_node_id = config->node_ids[kEdges[e][1]];
      c.minimum_samples = config->minimum_samples; c.max_samples = config->max_samples_per_edge;
      c.yaw_starts = config->yaw_starts; c.max_iterations = config->max_iterations;
      c.condition_limit = config->condition_limit;
      const auto status = zju_coop_create_rte(&c, &h->edges[e].pair);
      if (status != ZJU_COOP_OK) return status;
    }
    *output = h.release();
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_destroy_rte(zju_coop_three_handle_rte_t* h) {
  delete h;
  return ZJU_COOP_OK;
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_push_node_state_rte(
    zju_coop_three_handle_rte_t* h, const zju_coop_node_state_t* s) {
  if (!h) return ZJU_COOP_INVALID_ARGUMENT;
  const auto rc = header(s);
  if (rc != ZJU_COOP_OK) return rc;
  const int i = node_index(*h, s->node_id);
  if (i < 0 || !s->timestamp_ns || s->valid != ZJU_COOP_TRUE || s->reserved0 || !zeros(s->reserved1) ||
      !finite_position(s->position_enu_m) || !Eigen::Map<const Vector3>(s->velocity_enu_mps).allFinite() ||
      !Eigen::Map<const Eigen::Vector4d>(s->orientation_flu_to_enu_xyzw).allFinite()) return ZJU_COOP_INVALID_ARGUMENT;
  const auto* q = s->orientation_flu_to_enu_xyzw;
  Eigen::Quaterniond orientation(q[3], q[0], q[1], q[2]);
  auto& poses = h->poses[i];
  if (std::abs(orientation.norm() - 1) > 1e-3 || (!poses.empty() && s->timestamp_ns <= poses.back().time)) return ZJU_COOP_INVALID_ARGUMENT;
  try {
    Pose pose{s->timestamp_ns, Eigen::Map<const Vector3>(s->position_enu_m), orientation.normalized(),
              Eigen::Map<const Vector3>(s->velocity_enu_mps)};
    if (h->config.planar) { pose.position.z() = 0; pose.velocity.z() = 0; }
    poses.push_back(pose);
    if (poses.size() > 4 * h->config.max_samples_per_edge) poses.pop_front();
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_push_range_rte(
    zju_coop_three_handle_rte_t* h, const zju_coop_range_packet_t* p,
    zju_coop_range_processing_result_t* result) {
  if (!h) return ZJU_COOP_INVALID_ARGUMENT;
  auto rc = header(p); if (rc != ZJU_COOP_OK) return rc;
  rc = header(result); if (rc != ZJU_COOP_OK) return rc;
  zju_coop_range_processing_result_t out;
  initialize(&out); out.struct_size = result->struct_size;
  out.from_node = std::min(p->from_node, p->to_node); out.to_node = std::max(p->from_node, p->to_node);
  out.disposition = ZJU_COOP_PROCESSING_INVALID_PACKET;
  out.update_disposition = ZJU_COOP_UPDATE_INVALID_PACKET;
  out.fusion_action = ZJU_COOP_FUSION_REJECT; out.covariance_scale = 1;
  const int e = edge_index(*h, p->from_node, p->to_node);
  if (e < 0 || !p->timestamp_ns || p->reserved0 || p->valid != ZJU_COOP_TRUE || p->nlos_flag ||
      p->has_nlos_probability > 1 || p->status != ZJU_COOP_RANGE_STATUS_OK ||
      !std::isfinite(p->range_m) || p->range_m <= 0 || p->range_m > 2e5 ||
      !std::isfinite(p->range_std_m) || p->range_std_m < 1e-6 || p->range_std_m > 1e3 ||
      (p->has_nlos_probability && (!std::isfinite(p->nlos_probability) || p->nlos_probability < 0 || p->nlos_probability >= 0.5F))) {
    ++h->rejected; *result = out; return ZJU_COOP_OK;
  }
  auto& edge = h->edges[e];
  if ((!edge.samples.empty() && p->timestamp_ns <= edge.samples.back().time) || edge.pending.count(p->timestamp_ns)) {
    out.disposition = !edge.samples.empty() && p->timestamp_ns < edge.samples.back().time ?
        ZJU_COOP_PROCESSING_OUT_OF_ORDER : ZJU_COOP_PROCESSING_DUPLICATE;
    out.update_disposition = ZJU_COOP_UPDATE_OUT_OF_ORDER;
    ++h->rejected; *result = out; return ZJU_COOP_OK;
  }
  try {
    out.disposition = consume(*h, e, *p);
    if (out.disposition == ZJU_COOP_PROCESSING_HELD) {
      edge.pending.emplace(p->timestamp_ns, *p);
      if (edge.pending.size() > h->config.max_samples_per_edge) { edge.pending.erase(edge.pending.begin()); ++h->rejected; }
      out.fusion_action = ZJU_COOP_FUSION_HOLD;
    } else if (out.disposition == ZJU_COOP_PROCESSING_PROCESSED) {
      out.update_disposition = ZJU_COOP_UPDATE_ACCEPTED;
      out.fusion_action = ZJU_COOP_FUSION_USE_NORMAL;
    } else { ++h->rejected; out.update_disposition = ZJU_COOP_UPDATE_NIS_REJECTED; }
    *result = out;
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_step_rte(
    zju_coop_three_handle_rte_t* h, uint64_t now, zju_coop_three_snapshot_rte_t* output) {
  if (!h || !now) return ZJU_COOP_INVALID_ARGUMENT;
  const auto rc = header(output); if (rc != ZJU_COOP_OK) return rc;
  try {
    drain(*h, now);
    if (h->dirty && (!h->attempted_at || (now >= h->attempted_at && now - h->attempted_at >= h->config.solve_interval_ns))) {
      correct(*h, now); h->attempted_at = now; h->dirty = false;
    }
    auto out = view(*h, now); out.struct_size = output->struct_size; *output = out;
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}
zju_coop_error_code_t ZJU_COOP_CALL zju_coop_three_get_pose2d_v2_rte(
    zju_coop_three_handle_rte_t* h, uint64_t now, zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t capacity, uint32_t stride, uint32_t* count) {
  if (!h || !now || !count) return ZJU_COOP_INVALID_ARGUMENT;
  auto rc = header(snapshot, ZJU_COOP_POSE2D_ABI_VERSION_V2); if (rc != ZJU_COOP_OK) return rc;
  *count = 3;
  if (!vehicles && !capacity && !stride) return ZJU_COOP_BUFFER_TOO_SMALL;
  if (!vehicles || stride < sizeof(*vehicles) || stride % alignof(zju_coop_vehicle_pose2d_v2_t) ||
      reinterpret_cast<uintptr_t>(vehicles) % alignof(zju_coop_vehicle_pose2d_v2_t) || snapshot->reserved0) return ZJU_COOP_INVALID_ARGUMENT;
  if (capacity < 3) return ZJU_COOP_BUFFER_TOO_SMALL;
  std::array<zju_coop_vehicle_pose2d_v2_t*, 3> dst;
  for (int i = 0; i < 3; ++i) {
    dst[i] = reinterpret_cast<zju_coop_vehicle_pose2d_v2_t*>(reinterpret_cast<unsigned char*>(vehicles) + i * static_cast<std::size_t>(stride));
    rc = header(dst[i], ZJU_COOP_POSE2D_ABI_VERSION_V2); if (rc != ZJU_COOP_OK) return rc;
    if (dst[i]->struct_size > stride || dst[i]->reserved0 || !zeros(dst[i]->reserved)) return ZJU_COOP_INVALID_ARGUMENT;
  }
  try {
    const auto state = view(*h, now);
    auto header_out = *snapshot;
    header_out.timestamp_ns = state.timestamp_ns;
    header_out.reference_node_id = h->config.node_ids[0];
    std::memset(header_out.frame_id, 0, sizeof(header_out.frame_id));
    std::snprintf(header_out.frame_id, sizeof(header_out.frame_id), "coop_rte_%u_local", header_out.reference_node_id);
    std::array<zju_coop_vehicle_pose2d_v2_t, 3> out{};
    for (int i = 0; i < 3; ++i) {
      out[i].struct_size = dst[i]->struct_size; out[i].abi_version = ZJU_COOP_POSE2D_ABI_VERSION_V2;
      out[i].node_id = h->config.node_ids[i];
      out[i].x_m = state.relative_position_m[3 * i]; out[i].y_m = state.relative_position_m[3 * i + 1];
      const auto* q = state.body_orientation_xyzw + 4 * i;
      const Eigen::Matrix3d r = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).toRotationMatrix();
      const bool heading_valid = h->initialized && std::hypot(r(0, 0), r(1, 0)) > 1e-8;
      out[i].yaw_rad = heading_valid ? wrap(std::atan2(r(1, 0), r(0, 0))) : 0.0;
      out[i].position_valid = state.status == ZJU_COOP_RTE_READY;
      out[i].yaw_valid = state.status == ZJU_COOP_RTE_READY && heading_valid;
    }
    *snapshot = header_out;
    for (int i = 0; i < 3; ++i) *dst[i] = out[i];
    return ZJU_COOP_OK;
  } catch (const std::bad_alloc&) { return ZJU_COOP_OUT_OF_MEMORY; }
    catch (...) { return ZJU_COOP_INTERNAL_ERROR; }
}
}  // extern "C"

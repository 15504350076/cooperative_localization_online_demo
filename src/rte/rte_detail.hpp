#pragma once
#include "zju_coop/relative_transform_api.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>

namespace zju::coop::rte_detail {
inline constexpr double kPi = 3.14159265358979323846;
using Vector3 = Eigen::Vector3d;
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

inline bool finite_position(const double* data) {
  return Eigen::Map<const Vector3>(data).allFinite() &&
         Eigen::Map<const Vector3>(data).cwiseAbs().maxCoeff() <= 1e5;
}

inline double wrap(double angle) {
  double result = std::remainder(angle, 2.0 * kPi);
  return result >= kPi ? result - 2.0 * kPi : result;
}

inline Eigen::Matrix3d rotation(double angle) {
  return Eigen::AngleAxisd(angle, Vector3::UnitZ()).toRotationMatrix();
}

struct Pose {
  uint64_t time{};
  Vector3 position{Vector3::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Vector3 velocity{Vector3::Zero()};
};

inline uint64_t difference(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }

inline bool at_time(const std::deque<Pose>& poses, uint64_t time, uint64_t tolerance, Pose& out) {
  if (poses.empty()) return false;
  auto next = std::lower_bound(poses.begin(), poses.end(), time,
      [](const Pose& pose, uint64_t t) { return pose.time < t; });
  if (next != poses.begin() && next != poses.end() && next->time != time &&
      difference(next->time, time) <= tolerance && difference((next - 1)->time, time) <= tolerance) {
    const auto& previous = *(next - 1);
    const double fraction = static_cast<double>(time - previous.time) / (next->time - previous.time);
    out.position = previous.position + fraction * (next->position - previous.position);
    out.orientation = previous.orientation.slerp(fraction, next->orientation);
    out.velocity = previous.velocity + fraction * (next->velocity - previous.velocity);
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
}  // namespace zju::coop::rte_detail

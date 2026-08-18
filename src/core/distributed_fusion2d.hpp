// 低带宽分布式模式：各车本地惯导提供航位推算基线，参考车只估计平面位置修正。
#pragma once

#include "core/dense_matrix.hpp"
#include "core/quaternion.hpp"
#include "core/range_ekf.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace zju::coop {

/** 跨车传输的最小惯导状态；完整零偏和15×15协方差保留在各车本机。 */
struct NodeState {
  std::uint32_t node_id{};
  std::uint64_t timestamp_ns{};
  std::uint64_t receive_timestamp_ns{};
  Vec3 position_enu_m{};
  Vec3 velocity_enu_mps{};
  Quaternion orientation_flu_to_enu{};
  bool valid{};
};

struct DistributedFusionConfig {
  std::uint32_t reference_node_id{};
  std::vector<std::uint32_t> node_ids;
  double initial_correction_std_m{1.0};
  double process_accel_std_mps2{0.5};
  double nis_gate{9.0};
  double min_covariance_diagonal{1.0e-9};
  std::uint64_t max_extrapolation_ns{100'000'000ULL};
  std::uint64_t node_timeout_ns{500'000'000ULL};
  std::uint64_t range_timeout_ns{500'000'000ULL};
  std::uint64_t max_future_skew_ns{100'000'000ULL};
  std::uint64_t max_receive_delay_ns{500'000'000ULL};
};

struct DistributedVehiclePose2D {
  std::uint32_t node_id{};
  double x_m{};
  double y_m{};
  double yaw_rad{};
  bool position_valid{};
  bool yaw_valid{};
};

struct DistributedPose2DSnapshot {
  std::uint64_t timestamp_ns{};
  std::uint32_t reference_node_id{};
  std::vector<DistributedVehiclePose2D> vehicles;
};

/**
 * 参考车上的最小二维协同修正器。状态只含每个非参考车的[delta_e,delta_n]；
 * 新NodeState推进航位推算基线，不覆盖已经由UWB形成的修正量。
 */
class DistributedFusion2D {
 public:
  explicit DistributedFusion2D(DistributedFusionConfig config);

  /** 成功存入严格递增的状态时返回true。 */
  [[nodiscard]] bool push_node_state(NodeState state);
  /** 在UWB测量时刻插值/有限外推两端状态并更新平面修正。 */
  [[nodiscard]] UpdateResult push_range(const RangePacket& packet);
  /** 只读输出同一公共时刻的参考车相对二维位姿。 */
  [[nodiscard]] DistributedPose2DSnapshot pose2d_snapshot(
      std::uint64_t now_ns) const;

 private:
  struct NodeRecord {
    std::uint32_t node_id{};
    std::size_t correction_offset{};
    bool reference{};
    std::deque<NodeState> history;
  };

  [[nodiscard]] NodeRecord* find_node(std::uint32_t node_id);
  [[nodiscard]] const NodeRecord* find_node(std::uint32_t node_id) const;
  [[nodiscard]] bool all_nodes_ready() const;
  [[nodiscard]] bool aligned_state(const NodeRecord& node,
                                   std::uint64_t timestamp_ns,
                                   NodeState& output) const;
  [[nodiscard]] bool state_is_fresh(const NodeRecord& node,
                                    std::uint64_t now_ns) const;
  [[nodiscard]] std::vector<bool> fresh_range_connectivity(
      std::uint64_t now_ns) const;
  [[nodiscard]] Vec3 corrected_position(const NodeRecord& node,
                                        const NodeState& state,
                                        const std::vector<double>& correction)
      const;

  DistributedFusionConfig config_;
  std::vector<NodeRecord> nodes_;
  std::unordered_map<std::uint32_t, std::size_t> node_lookup_;
  std::vector<double> correction_;
  DenseMatrix covariance_;
  std::unordered_map<std::uint64_t, std::uint64_t>
      last_range_timestamp_by_edge_;
  std::unordered_map<std::uint64_t, std::uint64_t>
      last_accepted_range_timestamp_by_edge_;
  std::unordered_map<std::uint64_t, std::uint64_t>
      last_accepted_range_receive_timestamp_by_edge_;
  std::uint64_t last_range_timestamp_ns_{};
  bool has_range_timebase_{};
};

}  // namespace zju::coop

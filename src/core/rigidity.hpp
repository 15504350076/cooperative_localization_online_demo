// 动态协同图的连通性、主参考可达性和二维几何可观性分析接口。
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace zju::coop {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

struct Edge {
  std::uint32_t from_node{0U};
  std::uint32_t to_node{0U};
};

struct RigidityResult {
  std::size_t rank{0U};
  std::size_t target_rank{0U};
  std::size_t reachable_count{0U};
  std::size_t effective_edge_count{0U};
  bool connected{false};
  bool observable{false};
};

[[nodiscard]] RigidityResult analyze_rigidity(
    const std::vector<std::uint32_t>& node_ids,
    const std::vector<Point2>& positions, const std::vector<Edge>& edges,
    std::uint32_t reference_node, double tolerance);

}  // namespace zju::coop

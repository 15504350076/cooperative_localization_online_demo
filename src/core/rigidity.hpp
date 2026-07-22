// 模块职责：依据当前有效协同边分析主参考可达性、图连通性和二维几何可观性。
// 拓扑由运行时观测质量动态生成，不假定三车、固定边数或固定约束路径；
// observable表示刚度矩阵达到目标数值秩，不等价于绝对坐标已知。
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

/** 一次拓扑分析结果，供Engine生成网络状态和不可观告警。 */
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

// 模块实现：由运行时有效边构造二维刚度矩阵，同时计算主参考可达节点集合。
// 重复边、自环和未知节点不会被当作额外约束；禁止把三车三边写成固定拓扑假设。
#include "core/rigidity.hpp"

#include "core/dense_matrix.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace zju::coop {

RigidityResult analyze_rigidity(const std::vector<std::uint32_t>& node_ids,
                                const std::vector<Point2>& positions,
                                const std::vector<Edge>& edges,
                                std::uint32_t reference_node,
                                double tolerance) {
  if (node_ids.size() != positions.size()) {
    throw std::invalid_argument("node IDs and positions must have equal sizes");
  }
  if (!(tolerance > 0.0) || !std::isfinite(tolerance)) {
    throw std::invalid_argument("rigidity tolerance must be positive and finite");
  }

  std::unordered_map<std::uint32_t, std::size_t> node_indices;  // 节点编号到位置/矩阵列索引的映射。
  node_indices.reserve(node_ids.size());
  for (std::size_t index = 0U; index < node_ids.size(); ++index) {  // index同步遍历编号及其二维位置。
    if (!std::isfinite(positions[index].x) ||
        !std::isfinite(positions[index].y)) {
      throw std::invalid_argument("positions must be finite");
    }
    const auto insertion =  // 插入结果用于同时建立索引并拒绝重复节点编号。
        node_indices.emplace(node_ids[index], index);
    if (!insertion.second) {
      throw std::invalid_argument("node IDs must be unique");
    }
  }

  const auto reference = node_indices.find(reference_node);  // 主参考在矩阵列顺序中的位置。
  if (reference == node_indices.end()) {
    throw std::invalid_argument("reference node is not present");
  }

  // 阶段1：把有向输入规范化为唯一无向边，重复方向不会增加约束数量。
  std::set<std::pair<std::uint32_t, std::uint32_t>> unique_edges;  // 端点升序的唯一无向边集合。
  for (const Edge& edge : edges) {  // edge为待过滤、规范化的输入约束。
    if (edge.from_node == edge.to_node ||
        node_indices.find(edge.from_node) == node_indices.end() ||
        node_indices.find(edge.to_node) == node_indices.end()) {
      continue;
    }
    unique_edges.emplace(std::min(edge.from_node, edge.to_node),
                         std::max(edge.from_node, edge.to_node));
  }

  // 阶段2：同一组有效边用于图遍历和刚度矩阵，保证连通/可观口径一致。
  std::vector<std::vector<std::size_t>> adjacency(node_ids.size());  // 以节点索引表示的无向邻接表。
  DenseMatrix rigidity(unique_edges.size(), node_ids.size() * 2U);  // 每边一行、每节点x/y两列的刚度矩阵。
  std::size_t edge_row = 0U;  // 当前无向边对应的刚度矩阵行号。
  for (const auto& edge : unique_edges) {  // edge为规范化后的唯一无向约束。
    const std::size_t from = node_indices.at(edge.first);   // 较小端点的节点/列索引。
    const std::size_t to = node_indices.at(edge.second);    // 较大端点的节点/列索引。
    adjacency[from].push_back(to);
    adjacency[to].push_back(from);

    const double dx = positions[from].x - positions[to].x;  // 两端点x坐标差。
    const double dy = positions[from].y - positions[to].y;  // 两端点y坐标差。
    if (!std::isfinite(dx) || !std::isfinite(dy)) {
      throw std::invalid_argument("coordinate differences must be finite");
    }
    // 距离平方的一阶约束在两端节点列上符号相反；每行只含四个非零项。
    // 统一比例不影响秩，因此无需除以实际距离。
    rigidity(edge_row, 2U * from) = dx;
    rigidity(edge_row, 2U * from + 1U) = dy;
    rigidity(edge_row, 2U * to) = -dx;
    rigidity(edge_row, 2U * to + 1U) = -dy;
    ++edge_row;
  }

  // 阶段3：从主参考做BFS，只统计主参考相对坐标系实际可达的平台。
  std::vector<bool> visited(node_ids.size(), false);  // BFS中各节点索引是否已入队。
  std::queue<std::size_t> pending;  // 等待展开邻居的节点索引FIFO。
  pending.push(reference->second);
  visited[reference->second] = true;
  std::size_t reachable_count = 0U;  // 已从队列取出并确认可达的节点数。
  while (!pending.empty()) {
    const std::size_t current = pending.front();  // 本轮展开邻接边的可达节点索引。
    pending.pop();
    ++reachable_count;
    for (const std::size_t neighbor : adjacency[current]) {  // neighbor为current的一跳邻居索引。
      if (!visited[neighbor]) {
        visited[neighbor] = true;
        pending.push(neighbor);
      }
    }
  }

  // 平面相对框架存在2个平移和1个整体旋转自由度，完整目标秩为2N-3。
  // 连通只说明有路径，若节点共线等几何退化使rank不足，observable仍为false。
  const std::size_t target_rank =  // 当前节点数对应的完整二维局部刚度目标秩。
      node_ids.size() >= 2U ? 2U * node_ids.size() - 3U : 0U;
  const std::size_t rank = numeric_rank(rigidity, tolerance);  // 在配置容差下计算的实际数值秩。
  const bool connected = reachable_count == node_ids.size();   // BFS是否覆盖全部配置节点。
  return RigidityResult{rank, target_rank, reachable_count, unique_edges.size(),
                        connected,
                        node_ids.size() >= 2U && connected &&
                            rank >= target_rank};
}

}  // namespace zju::coop

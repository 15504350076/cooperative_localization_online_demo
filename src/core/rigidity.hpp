// 模块职责：依据当前有效协同边分析主参考可达性、图连通性和二维几何可观性。
// 拓扑由运行时观测质量动态生成，不假定三车、固定边数或固定约束路径；
// observable表示刚度矩阵达到目标数值秩，不等价于绝对坐标已知。
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace zju::coop {

/** 主参考相对平面位置，单位m；这里只使用几何差，不解释为全球坐标。 */
struct Point2 {
  double x{0.0};  ///< 主参考平面中的横向坐标，单位m。
  double y{0.0};  ///< 主参考平面中的纵向坐标，单位m。
};

/** 当前允许进入拓扑分析的边；方向会在实现中规范化为唯一无向约束。 */
struct Edge {
  std::uint32_t from_node{0U};  ///< 输入边的起点节点编号。
  std::uint32_t to_node{0U};    ///< 输入边的终点节点编号。
};

/** 一次拓扑分析结果，供Engine生成网络状态和不可观告警。 */
struct RigidityResult {
  std::size_t rank{0U};                 ///< 当前有效边刚度矩阵的数值秩。
  std::size_t target_rank{0U};          ///< 至少2节点时为2N-3，单节点时为0。
  std::size_t reachable_count{0U};      ///< 从主参考沿有效边可达的节点数。
  std::size_t effective_edge_count{0U}; ///< 去除自环、未知边和重复边后的约束数。
  bool connected{false};                ///< 主参考是否能到达全部配置节点。
  bool observable{false};               ///< 至少2节点、连通且刚度矩阵达到目标秩。
};

/**
 * 同时计算主参考可达性和二维距离约束的局部刚度。
 * positions必须与node_ids逐项对应；未知边和自环被忽略，节点/位置配置错误抛出。
 * @param node_ids 配置节点编号及矩阵列的顺序。
 * @param positions 与node_ids逐项对应的当前二维位置。
 * @param edges 本轮允许参与拓扑分析的输入边集合。
 * @param reference_node 主参考节点编号，即可达性遍历起点。
 * @param tolerance 刚度矩阵数值秩判定容差。
 */
[[nodiscard]] RigidityResult analyze_rigidity(
    const std::vector<std::uint32_t>& node_ids,
    const std::vector<Point2>& positions, const std::vector<Edge>& edges,
    std::uint32_t reference_node, double tolerance);

}  // namespace zju::coop

// 把三条实测UWB边长送入工程现有的solve_three_node_formation()，输出JSON。
// 用法：check_three_node_formation node0 node1 node2 d01_m d02_m d12_m
#include "core/range_ekf.hpp"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr << "usage: " << argv[0]
              << " node0 node1 node2 d01_m d02_m d12_m\n";
    return 2;
  }
  try {
    zju::coop::ThreeNodeRanges ranges{};
    for (int i = 0; i < 3; ++i) {
      const auto value = std::stoul(argv[i + 1]);
      if (value > UINT32_MAX) {
        throw std::out_of_range("node id exceeds uint32");
      }
      ranges.node_ids[static_cast<std::size_t>(i)] =
          static_cast<std::uint32_t>(value);
    }
    ranges.range_01_m = std::stod(argv[4]);
    ranges.range_02_m = std::stod(argv[5]);
    ranges.range_12_m = std::stod(argv[6]);
    const auto result = zju::coop::solve_three_node_formation(ranges);
    const double radians_to_degrees = 180.0 / std::acos(-1.0);
    std::cout << std::setprecision(17)
              << "{\"disposition\":" << static_cast<int>(result.disposition)
              << ",\"points\":[";
    for (std::size_t i = 0; i < result.points.size(); ++i) {
      const auto& point = result.points[i];
      std::cout << (i ? "," : "") << "{\"node_id\":" << point.node_id
                << ",\"x_m\":" << point.x_m << ",\"y_m\":"
                << point.y_m << "}";
    }
    std::cout << "],\"interior_angles_deg\":[";
    for (std::size_t i = 0; i < result.interior_angles_rad.size(); ++i) {
      std::cout << (i ? "," : "")
                << result.interior_angles_rad[i] * radians_to_degrees;
    }
    std::cout << "],\"bearings_deg\":["
              << result.bearing_01_rad * radians_to_degrees << ","
              << result.bearing_02_rad * radians_to_degrees << ","
              << result.bearing_12_rad * radians_to_degrees
              << "],\"twice_area_m2\":" << result.twice_area_m2 << "}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}

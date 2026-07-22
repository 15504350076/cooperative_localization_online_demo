// 模块职责：保留早期核心接口回归入口，覆盖三车节点初始化、测距输入和主参考输出。
// 当前CMake不再编译此文件；有效回归已拆入test_engine等注册式用例，阅读时不得把它当成交付测试证据。
#include "zju_coop/engine.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

bool near(double actual, double expected, double tolerance = 1.0e-12) {
  return std::abs(actual - expected) <= tolerance;
}

void test_three_node_initialization_uses_reference_relative_frame() {
  zju::coop::EngineConfig config{};
  config.reference_node_id = 1;

  const std::vector<zju::coop::NodeInitialization> nodes{
      {1, 100.0, 50.0, 0.04},
      {2, 104.0, 50.0, 0.04},
      {3, 100.0, 53.0, 0.04},
  };

  const zju::coop::CooperativeEngine engine(config, nodes);
  const auto estimates = engine.estimates();

  check(estimates.size() == 3U, "three configured nodes must be reported");
  check(estimates.at(0).node_id == 1, "reference node must be first");
  check(near(estimates.at(0).x_m, 0.0), "reference x must be zero");
  check(near(estimates.at(0).y_m, 0.0), "reference y must be zero");
  check(near(estimates.at(1).x_m, 4.0), "node 2 x must be relative to reference");
  check(near(estimates.at(1).y_m, 0.0), "node 2 y must be relative to reference");
  check(near(estimates.at(2).x_m, 0.0), "node 3 x must be relative to reference");
  check(near(estimates.at(2).y_m, 3.0), "node 3 y must be relative to reference");
  check(!estimates.at(1).yaw_valid, "UWB-only output must not claim yaw");
  check(!estimates.at(1).z_valid, "planar UWB-only output must not claim height");
}

}  // namespace

int main() {
  try {
    test_three_node_initialization_uses_reference_relative_frame();
  } catch (const std::exception& error) {
    std::cerr << "UNEXPECTED EXCEPTION: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }

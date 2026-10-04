// 模块职责：保留早期核心接口回归入口，覆盖三车节点初始化、测距输入和主参考输出。
// 当前CMake不再编译此文件；有效回归已拆入test_engine等注册式用例，阅读时不得把它当成交付测试证据。
// C++初学者注意：这是历史代码，只用于理解早期思路；学习和验证应从test_main.cpp、
// test_support.hpp及当前test_*.cpp开始，不要尝试单独编译本文件来判断工程是否可用。
#include "zju_coop/engine.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

// failures：早期独立测试入口共享的失败检查计数，非零时令进程返回EXIT_FAILURE。
int failures = 0;

// condition是待验证状态，message是失败时输出的场景说明；二者只在本次检查调用中使用。
void check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << '\n';
  }
}

// actual/expected分别为实测值和解析期望值，tolerance是允许的绝对误差上限。
bool near(double actual, double expected, double tolerance = 1.0e-12) {
  return std::abs(actual - expected) <= tolerance;
}

void test_three_node_initialization_uses_reference_relative_frame() {
  // config：指定节点1为相对坐标原点的引擎配置。
  zju::coop::EngineConfig config{};
  config.reference_node_id = 1;

  // nodes：输入的三节点全局坐标与测距标准差，构成3-4直角布局。
  const std::vector<zju::coop::NodeInitialization> nodes{
      {1, 100.0, 50.0, 0.04},
      {2, 104.0, 50.0, 0.04},
      {3, 100.0, 53.0, 0.04},
  };

  // engine：由上述输入构造的被测引擎；estimates：其首次相对定位快照，期望主节点归零且从节点保留偏移。
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
    // error：捕获初始化场景的意外异常，内容直接写入独立测试入口诊断。
    std::cerr << "UNEXPECTED EXCEPTION: " << error.what() << '\n';
    return EXIT_FAILURE;
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed\n";
    return EXIT_FAILURE;
  }

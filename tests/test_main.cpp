// 模块职责：聚合全部C++测试模块并输出逐项结果与最终失败计数。
#include "test_support.hpp"

#include <exception>
#include <iostream>

int main() {
  // 每个register_*函数对应一个独立模块，新增测试文件必须在此显式注册。
  unsigned int failures = 0U;

  for (const auto& test_case : zju::coop::test::registry()) {
    try {
      test_case.function();
      std::cout << "[PASS] " << test_case.name << '\n';
    } catch (const std::exception& error) {
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": " << error.what()
                << '\n';
    } catch (...) {
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": unknown exception\n";
    }
  }

  std::cout << "Tests: " << zju::coop::test::registry().size()
            << ", Failures: " << failures << '\n';
  return failures == 0U ? 0 : 1;
}

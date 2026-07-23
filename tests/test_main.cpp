// 模块职责：聚合全部C++测试模块并输出逐项结果与最终失败计数。
#include "test_support.hpp"

#include <exception>
#include <iostream>

int main() {
  // failures：累计未通过用例数，决定汇总输出和进程退出码。
  unsigned int failures = 0U;

  // test_case：依次借用进程级注册表中的用例描述，注册表在整个遍历期间保持稳定。
  for (const auto& test_case : zju::coop::test::registry()) {
    try {
      test_case.function();
      std::cout << "[PASS] " << test_case.name << '\n';
    } catch (const std::exception& error) {
      // error：仅在当前catch块存活的失败异常引用，用于把断言诊断写入CTest日志。
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

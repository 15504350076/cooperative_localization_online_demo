// 模块职责：聚合全部C++测试模块并输出逐项结果与最终失败计数。
// C++初学者阅读提示：各TEST_CASE在程序启动前自动登记到registry；
// main逐项调用函数，异常表示当前用例失败但不会阻止后续用例继续运行。
#include "test_support.hpp"

#include <exception>
#include <iostream>

int main() {
  // failures：累计未通过用例数，决定汇总输出和进程退出码。
  unsigned int failures = 0U;

  // test_case：依次借用进程级注册表中的用例描述，注册表在整个遍历期间保持稳定。
  for (const auto& test_case : zju::coop::test::registry()) {
    // try中的测试函数可能由EXPECT_*抛出runtime_error；通过则执行到PASS输出。
    try {
      // function是函数指针，加括号即可调用其指向的无参测试函数。
      test_case.function();
      // `<<`把右侧内容依次写入标准输出流；'\n'换行但不像std::endl强制刷新。
      std::cout << "[PASS] " << test_case.name << '\n';
    } catch (const std::exception& error) {
      // error：仅在当前catch块存活的失败异常引用，用于把断言诊断写入CTest日志。
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": " << error.what()
                << '\n';
    } catch (...) {
      // 兜底捕获不继承std::exception的异常，避免整个测试程序在首个未知异常处退出。
      ++failures;
      std::cerr << "[FAIL] " << test_case.name << ": unknown exception\n";
    }
  }

  std::cout << "Tests: " << zju::coop::test::registry().size()
            << ", Failures: " << failures << '\n';
  // 三目运算把失败计数转换成进程退出码：全通过为0，任何失败为1。
  return failures == 0U ? 0 : 1;
}

// 模块职责：独立自检可执行程序入口，只调用run_self_check并把报告转换为进程退出码。
// 退出码0表示全部软件检查通过，非0便于部署脚本和上海交大联调脚本自动判定失败。
//
// C++初学者阅读提示：main是可执行程序的起点；std::cout输出报告；最后的三目运算符
// “condition ? A : B”表示condition为真时取A，否则取B，因此通过返回0、失败返回1。
#include "self_check/self_check.hpp"

#include <iostream>

int main() {
  // 自检不读取在线配置、不绑定UDP端口，因此不会影响之后的实机进程。
  const auto result =  // 汇总全部确定性检查计数及逐项报告的返回值。
      zju::coop::self_check::run_self_check();
  std::cout << result.report;
  std::cout << "SELF_CHECK " << (result.passed ? "PASS" : "FAIL")
            << " (passed=" << result.passed_checks
            << ", failed=" << result.failed_checks << ")\n";
  return result.passed ? 0 : 1;
}

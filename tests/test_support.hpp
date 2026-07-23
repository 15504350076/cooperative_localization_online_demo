// 模块职责：提供全部C++单元测试共享的注册、断言、异常检查和浮点比较工具。
// 断言失败会抛出异常并终止当前用例；test_main捕获后继续运行其余用例并最终返回非零退出码。
#pragma once

#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace zju::coop::test {

using TestFunction = void (*)();

struct TestCase {
  // name：指向静态用例名字符串，生命周期覆盖整个测试进程；function：同名测试函数入口，由注册表统一调度。
  const char* name;
  TestFunction function;
};

inline std::vector<TestCase>& registry() {
  // cases：进程级用例注册表，首次调用时构造、进程退出时销毁；Registrar只保存静态字符串和函数地址。
  static std::vector<TestCase> cases;
  return cases;
}

class Registrar {
 public:
  // name在测试进程内保持有效，function指向无参用例；构造期间把二者复制进长生命周期注册表。
  Registrar(const char* name, TestFunction function) {
    registry().push_back({name, function});
  }
};

// expression保存失败断言文本，file/line保存宏展开处源码位置，三者只在本次异常消息组装期间借用。
inline void fail(const char* expression, const char* file, int line) {
  // message：暂存带源位置的失败说明，抛出runtime_error时复制其字符串内容。
  std::ostringstream message;
  message << file << ':' << line << ": expectation failed: " << expression;
  throw std::runtime_error(message.str());
}

// value是被判定的布尔状态；expression/file/line用于失败时还原原始断言及调用位置。
inline void expect_true(bool value, const char* expression, const char* file,
                        int line) {
  if (!value) {
    fail(expression, file, line);
  }
}

template <typename Left, typename Right>
// left/right是在本次比较期间借用的实参；expression/file/line记录比较文本与断言位置。
inline void expect_equal(const Left& left, const Right& right,
                         const char* expression, const char* file, int line) {
  if (!(left == right)) {
    fail(expression, file, line);
  }
}

}  // namespace zju::coop::test

// name同时作为静态测试函数标识符和字符串化用例名，保证注册项与实际入口同名。
// name##_registrar是进程初始化期构造的静态对象，其构造函数把name对应函数登记到registry。
#define TEST_CASE(name)                                                    \
  static void name();                                                       \
  static ::zju::coop::test::Registrar name##_registrar(#name, &name);      \
  static void name()

#define EXPECT_TRUE(expression)                                            \
  ::zju::coop::test::expect_true(static_cast<bool>(expression), #expression, \
                                 __FILE__, __LINE__)

#define EXPECT_FALSE(expression) EXPECT_TRUE(!(expression))

#define EXPECT_EQ(left, right)                                             \
  ::zju::coop::test::expect_equal((left), (right), #left " == " #right,    \
                                 __FILE__, __LINE__)

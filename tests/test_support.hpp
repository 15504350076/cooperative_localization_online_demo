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
  const char* name;
  TestFunction function;
};

inline std::vector<TestCase>& registry() {
  // 函数内静态对象规避跨测试翻译单元的初始化顺序问题；Registrar构造时统一写入该表。
  static std::vector<TestCase> cases;
  return cases;
}

class Registrar {
 public:
  Registrar(const char* name, TestFunction function) {
    registry().push_back({name, function});
  }
};

inline void fail(const char* expression, const char* file, int line) {
  // 保留表达式和源位置，使CTest日志无需调试器即可定位首个失败断言。
  std::ostringstream message;
  message << file << ':' << line << ": expectation failed: " << expression;
  throw std::runtime_error(message.str());
}

inline void expect_true(bool value, const char* expression, const char* file,
                        int line) {
  if (!value) {
    fail(expression, file, line);
  }
}

template <typename Left, typename Right>
inline void expect_equal(const Left& left, const Right& right,
                         const char* expression, const char* file, int line) {
  if (!(left == right)) {
    fail(expression, file, line);
  }
}

}  // namespace zju::coop::test

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

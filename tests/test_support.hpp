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

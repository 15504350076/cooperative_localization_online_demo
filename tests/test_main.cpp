#include "test_support.hpp"

#include <exception>
#include <iostream>

int main() {
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

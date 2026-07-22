// 模块职责：把独立C ABI自检纳入CTest，确认报告、通过计数和最终状态一致。
#include "self_check/self_check.hpp"
#include "test_support.hpp"

#include <string>

TEST_CASE(self_check_validates_public_c_abi_without_external_io) {
  const zju::coop::self_check::SelfCheckResult result =
      zju::coop::self_check::run_self_check();

  EXPECT_TRUE(result.passed);
  EXPECT_EQ(result.failed_checks, 0U);
  EXPECT_TRUE(result.passed_checks >= 11U);
  EXPECT_TRUE(result.report.find("[PASS] imu_propagation") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] nominal_snapshot") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] link_timeout") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] network_recovery") !=
              std::string::npos);
}

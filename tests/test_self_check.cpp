// 模块职责：把独立C ABI自检纳入CTest，确认报告、通过计数和最终状态一致。
// C++初学者阅读提示：本用例只调用一次公开run_self_check，然后检查总状态、失败数和关键文字；
// 具体三车数据怎样进入C ABI，请继续看src/self_check/self_check.cpp。
#include "self_check/self_check.hpp"
#include "test_support.hpp"

#include <string>

TEST_CASE(self_check_validates_public_c_abi_without_external_io) {
  // result：一次无外部I/O自检的完整汇总，期望总状态通过、失败计数为零且关键阶段均留有通过记录。
  const zju::coop::self_check::SelfCheckResult result =
      zju::coop::self_check::run_self_check();

  // 同时检查汇总位、失败计数和关键阶段名称，防止自检因漏跑阶段而出现“空报告假通过”。
  EXPECT_TRUE(result.passed);
  EXPECT_EQ(result.failed_checks, 0U);
  EXPECT_TRUE(result.passed_checks >= 14U);
  EXPECT_TRUE(result.report.find("[PASS] imu_propagation") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] nominal_snapshot") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] link_timeout") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] network_recovery") !=
              std::string::npos);
  EXPECT_TRUE(result.report.find("[PASS] pose2d_snapshot") !=
              std::string::npos);
}

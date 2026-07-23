// 模块职责：声明不依赖ROS 2、网络、配置文件和真实传感器的公开C ABI确定性自检。
// 自检只验证软件闭环与接口契约，不替代实机传感器精度、时钟同步或性能验收。
#pragma once

#include <cstdint>
#include <string>

namespace zju::coop::self_check {

/**
 * 独立自检结果。
 *
 * 自检只通过公开 C ABI 驱动算法，不访问 ROS 2、网络、传感器或配置文件，
 * 因而可在开发机和 AIBrainBox 上重复运行而不干扰在线进程。
 */
struct SelfCheckResult {
  bool passed{};                       ///< 是否所有已执行检查均通过。
  std::uint32_t passed_checks{};       ///< 通过检查项数量。
  std::uint32_t failed_checks{};       ///< 失败检查项数量。
  std::string report;                  ///< 按执行顺序生成的逐项文本报告。
};

/**
 * 运行三车IMU+测距确定性闭环自检；测距项只验证引擎级Processed处理路径，
 * 不单独断言底层update_disposition为ACCEPTED；任何异常均转换为失败结果。
 */
[[nodiscard]] SelfCheckResult run_self_check() noexcept;

}  // namespace zju::coop::self_check

// 模块职责：验证15N联合协方差传播、平台间测距更新、交叉相关和主参考相对输出。
// C++初学者阅读方法：先看测试名称理解要证明的性质，再依次找“初始配置、输入IMU/测距、
// 调用predict/update、EXPECT_*断言”。15N表示N辆车每辆各有15维误差状态。
#include "core/cooperative_inertial_ekf.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

using zju::coop::CooperativeInertialConfig;
using zju::coop::CooperativeInertialEkf;
using zju::coop::ImuDisposition;
using zju::coop::ImuPacket;
using zju::coop::InertialConfig;
using zju::coop::InertialNodeInitialization;
using zju::coop::RangePacket;
using zju::coop::UpdateDisposition;

InertialConfig inertial_config() {
  // config：放宽时间步上限并固定IMU坐标系名的单节点传播配置。
  InertialConfig config{};
  config.max_imu_dt_s = 1.0;
  config.expected_frame_id = "imu_link";
  return config;
}

CooperativeInertialConfig cooperative_config() {
  // config：指定稀疏主参考ID、NIS门限、协方差下限和联合状态资源上限。
  CooperativeInertialConfig config{};
  config.reference_node_id = 7U;
  config.nis_gate = 9.0;
  config.min_covariance_diagonal = 1.0e-12;
  config.max_inertial_state_dimension = 300U;
  return config;
}

std::vector<InertialNodeInitialization> nodes() {
  // result：按42、7、99的非排序ID构造三节点初值，位置形成3-4直角布局。
  std::vector<InertialNodeInitialization> result(3U);
  result[0].node_id = 42U;
  result[0].position_n_m = {3.0, 0.0, 0.0};
  result[1].node_id = 7U;
  result[2].node_id = 99U;
  result[2].position_n_m = {0.0, 4.0, 0.0};
  return result;
}

// node_id/sequence/timestamp_ns分别指定IMU归属、去重序号和采样时刻，供传播与未知节点路径复用。
ImuPacket imu(std::uint32_t node_id, std::uint64_t sequence,
              std::uint64_t timestamp_ns) {
  // packet：有效静止IMU输入包；frame：复制进定长frame_id缓冲的期望坐标系字符串。
  ImuPacket packet{};
  packet.node_id = node_id;
  packet.sequence = sequence;
  packet.timestamp_ns = timestamp_ns;
  packet.receive_timestamp_ns = timestamp_ns + 1U;
  packet.linear_acceleration_m_s2 = {0.0, 0.0, 9.80665};
  const char* frame = "imu_link";
  std::copy(frame, frame + std::strlen(frame) + 1U, packet.frame_id.begin());
  packet.valid = true;
  return packet;
}

// from/to确定测距边端点，range_m是观测距离，standard_deviation控制NIS接受或拒绝强度。
RangePacket range(std::uint16_t from, std::uint16_t to, double range_m,
                  double standard_deviation = 0.5) {
  // packet：带有效时间戳和不确定度的单次测距输入。
  RangePacket packet{};
  packet.from_node = from;
  packet.to_node = to;
  packet.sequence = 1U;
  packet.timestamp_ns = 1'000'000'000ULL;
  packet.receive_timestamp_ns = packet.timestamp_ns + 1U;
  packet.range_m = range_m;
  packet.range_std_m = standard_deviation;
  packet.valid = true;
  return packet;
}

}  // namespace

// 布局/传播组锁定稀疏节点到15维块的映射，以及单节点IMU对边缘块和交叉块的影响。
TEST_CASE(cooperative_inertial_ekf_uses_stable_sparse_node_layout) {
  // filter：按非连续节点ID构造的45维联合滤波器，期望块顺序与输入顺序稳定一致。
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                nodes());

  EXPECT_EQ(filter.state_dimension(), 45U);
  EXPECT_EQ(filter.node_ids().size(), 3U);
  EXPECT_EQ(filter.node_ids()[0], 42U);
  EXPECT_EQ(filter.node_ids()[1], 7U);
  EXPECT_EQ(filter.node_ids()[2], 99U);
  EXPECT_TRUE(std::abs(filter.state(42U).position_n_m.x - 3.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(filter.state(99U).position_n_m.y - 4.0) < 1.0e-12);
}

TEST_CASE(cooperative_inertial_ekf_propagates_only_selected_node_blocks) {
  // filter：仅向节点42注入IMU的联合滤波器；covariance_before/node_99_before：传播前未选节点与协方差基线。
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                nodes());
  (void)filter.push_imu(imu(42U, 1U, 1'000'000'000ULL));
  const auto covariance_before = filter.covariance();
  const auto node_99_before = filter.state(99U);

  // result：第二帧10 ms间隔IMU的处理回执，期望进入传播状态。
  const auto result = filter.push_imu(imu(42U, 2U, 1'010'000'000ULL));

  EXPECT_EQ(result.disposition, ImuDisposition::kPropagated);
  EXPECT_EQ(filter.state(99U).position_n_m.x, node_99_before.position_n_m.x);
  EXPECT_EQ(filter.state(99U).position_n_m.y, node_99_before.position_n_m.y);
  // 节点42的 p-v 块经 Phi 传播后出现非零交叉项；节点99的自身块不变。
  EXPECT_TRUE(std::abs(filter.covariance()(0U, 3U)) > 0.0);
  EXPECT_EQ(filter.covariance()(30U, 30U), covariance_before(30U, 30U));
}

TEST_CASE(cooperative_inertial_ekf_rejects_unknown_imu_without_state_change) {
  // filter/covariance_before：接收未知节点前的被测滤波器及协方差基线；result期望报告kUnknownNode且协方差(0,0)不变。
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                nodes());
  const auto covariance_before = filter.covariance();

  const auto result = filter.push_imu(imu(1234U, 1U, 1'000'000'000ULL));

  EXPECT_EQ(result.disposition, ImuDisposition::kUnknownNode);
  EXPECT_EQ(filter.covariance()(0U, 0U), covariance_before(0U, 0U));
}

// 测距组验证H虽只落在两端位置块，完整K仍建立跨节点相关并降低距离残差。
TEST_CASE(cooperative_inertial_ekf_range_update_reduces_residual_and_couples_nodes) {
  // initializations：放大测距两端先验位置方差以允许明显修正；filter：执行节点7到42测距更新的联合滤波器。
  auto initializations = nodes();
  initializations[0].position_std_m = {2.0, 2.0, 2.0};
  initializations[1].position_std_m = {2.0, 2.0, 2.0};
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                initializations);
  // before/after：更新前后节点间x距离对2.5 m观测的绝对残差；result：期望Accepted的更新回执。
  const double before = std::abs(
      (filter.state(42U).position_n_m - filter.state(7U).position_n_m).x -
      2.5);

  const auto result = filter.update_range(range(7U, 42U, 2.5));
  const double after = std::abs(
      (filter.state(42U).position_n_m - filter.state(7U).position_n_m).x -
      2.5);

  EXPECT_EQ(result.disposition, UpdateDisposition::Accepted);
  EXPECT_TRUE(after < before);
  EXPECT_TRUE(std::abs(filter.state(7U).position_n_m.x) > 0.0);
  EXPECT_TRUE(std::abs(filter.state(42U).position_n_m.x - 3.0) > 0.0);
  EXPECT_TRUE(std::abs(filter.covariance()(0U, 15U)) > 0.0);
}

TEST_CASE(cooperative_inertial_ekf_nis_rejection_does_not_inject_state) {
  // filter/before：极小标准差离群测距注入前的滤波器及节点42状态；result期望NIS拒绝且节点42的position.x不变。
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                nodes());
  const auto before = filter.state(42U);

  const auto result = filter.update_range(range(7U, 42U, 100.0, 0.01));

  EXPECT_EQ(result.disposition, UpdateDisposition::NisRejected);
  EXPECT_EQ(filter.state(42U).position_n_m.x, before.position_n_m.x);
}

// 输出组单独检查Pii+Prr-Pir-Pri，防止把两个边缘方差简单相加。
TEST_CASE(cooperative_inertial_ekf_outputs_reference_relative_state_and_covariance) {
  // initializations：为所有节点赋相同速度的初值集合；node：逐项写入速度的短生命周期引用。
  auto initializations = nodes();
  for (auto& node : initializations) {
    node.velocity_n_mps = {2.0, -1.0, 0.0};
  }
  // filter：生成主参考相对输出；reference/node_42：分别核对归零参考和3 m相对节点快照。
  CooperativeInertialEkf filter(cooperative_config(), inertial_config(),
                                initializations);

  const auto reference = filter.estimate(7U);
  const auto node_42 = filter.estimate(42U);

  EXPECT_EQ(reference.x, 0.0);
  EXPECT_EQ(reference.y, 0.0);
  EXPECT_EQ(reference.vx, 0.0);
  EXPECT_EQ(reference.vy, 0.0);
  EXPECT_TRUE(std::abs(node_42.x - 3.0) < 1.0e-12);
  EXPECT_EQ(node_42.vx, 0.0);
  EXPECT_EQ(node_42.vy, 0.0);
  EXPECT_TRUE(node_42.cov_xx > 0.0);
  EXPECT_TRUE(node_42.valid);
}

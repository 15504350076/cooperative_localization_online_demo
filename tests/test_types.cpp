// 模块职责：验证公共枚举位图、默认值和能力/原因组合的基础类型语义。
#include "test_support.hpp"
#include "zju_coop/types.hpp"

// 能力位必须支持组合且不能把kNone误判为存在，否则GCS会显示未实现的状态维度。
TEST_CASE(capability_mask_reports_present_and_absent_bits) {
  // mask：只声明测距与平面位置的输入能力位；combined：追加速度位后用于验证多位“全包含”语义。
  const auto mask = zju::coop::Capability::kUwbRange |
                    zju::coop::Capability::kPlanarPosition;
  const auto combined = mask | zju::coop::Capability::kVelocity;

  EXPECT_TRUE(
      zju::coop::has_capability(mask, zju::coop::Capability::kUwbRange));
  EXPECT_FALSE(zju::coop::has_capability(mask, zju::coop::Capability::kYaw));
  EXPECT_TRUE(zju::coop::has_capability(
      combined, zju::coop::Capability::kUwbRange |
                    zju::coop::Capability::kVelocity));
  EXPECT_FALSE(zju::coop::has_capability(
      mask, zju::coop::Capability::kUwbRange |
                zju::coop::Capability::kVelocity));
  EXPECT_FALSE(
      zju::coop::has_capability(mask, zju::coop::Capability::kNone));
}

// 默认构造包保持invalid，要求适配层显式填写有效性和时间，而不是零值误入滤波器。
TEST_CASE(range_packet_defaults_are_invalid) {
  // packet：零初始化的空测距包，期望保持无效且所有标识、时间、量测和NLOS字段为安全零值。
  const zju::coop::RangePacket packet{};

  EXPECT_FALSE(packet.valid);
  EXPECT_EQ(packet.from_node, 0U);
  EXPECT_EQ(packet.to_node, 0U);
  EXPECT_EQ(packet.sequence, 0U);
  EXPECT_EQ(packet.timestamp_ns, 0U);
  EXPECT_EQ(packet.receive_timestamp_ns, 0U);
  EXPECT_EQ(packet.range_m, 0.0);
  EXPECT_EQ(packet.range_std_m, 0.0);
  EXPECT_EQ(packet.nlos_probability, 0.0F);
  EXPECT_FALSE(packet.nlos_flag);
  EXPECT_FALSE(packet.has_nlos_probability);
  EXPECT_EQ(packet.status, 0U);
}

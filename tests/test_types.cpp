#include "test_support.hpp"
#include "zju_coop/types.hpp"

TEST_CASE(capability_mask_reports_present_and_absent_bits) {
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

TEST_CASE(range_packet_defaults_are_invalid) {
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

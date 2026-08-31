"""Contract test for the explicitly enabled GNSS-derived range fallback."""

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from cooperative_interfaces.msg import GnssPosition, UwbRange
from cooperative_localization_msgs.msg import NodeState
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
FIX_TOPIC = "/test/gnss/fix"
RANGE_TOPIC = "/test/cooperative_localization/gnss_derived_range"
STATE_TOPIC = "/test/cooperative_localization/node_state"
WGS84_A_M = 6378137.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    fallback = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_gnss_range_fallback_node",
        name="zju_gnss_range_fallback_test",
        output="screen",
        parameters=[{
            "node_ids": [1, 2, 3],
            "common_frame_id": "common_enu",
            "max_stamp_skew_ms": 50,
            "common_time_timeout_ms": 150,
            "max_state_stamp_spread_ms": 100,
            "fix_timeout_ms": 150,
            "max_receive_delay_ms": 500,
            "max_future_skew_ms": 100,
            "max_range_m": 100.0,
            "use_altitude": False,
        }],
        remappings=[
            ("node_state", STATE_TOPIC),
            ("fix", FIX_TOPIC),
            ("range", RANGE_TOPIC),
        ],
    )
    return (
        launch.LaunchDescription([
            fallback,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"fallback": fallback},
    )


class TestGnssRangeFallback(unittest.TestCase):

    def test_sjtu_message_contracts_are_frozen(self):
        self.assertEqual(
            UwbRange.get_fields_and_field_types(),
            {
                "header": "std_msgs/Header",
                "src_id": "uint32",
                "target_id": "uint32",
                "distance": "float",
            },
        )
        self.assertEqual(
            GnssPosition.get_fields_and_field_types(),
            {
                "header": "std_msgs/Header",
                "node_id": "uint32",
                "status": "int8",
                "latitude": "double",
                "longitude": "double",
                "altitude": "double",
                "position_covariance": "double[9]",
                "position_covariance_type": "uint8",
            },
        )

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("gnss_range_fallback_test_driver")
        state_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.state_publisher = self.node.create_publisher(
            NodeState, STATE_TOPIC, state_qos
        )
        self.fix_publisher = self.node.create_publisher(
            GnssPosition, FIX_TOPIC, qos_profile_sensor_data
        )
        self.ranges = []
        self.range_subscription = self.node.create_subscription(
            UwbRange, RANGE_TOPIC, self.ranges.append, qos_profile_sensor_data
        )

    def tearDown(self):
        self.node.destroy_publisher(self.state_publisher)
        self.node.destroy_publisher(self.fix_publisher)
        self.node.destroy_subscription(self.range_subscription)
        self.node.destroy_node()

    def _wait_for_graph(self, timeout_s=8.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.fix_publisher.get_subscription_count() == 1
                and self.state_publisher.get_subscription_count() == 1
                and self.node.count_publishers(RANGE_TOPIC) == 1
            ):
                return
        self.fail("GNSS fallback ROS graph was not ready")

    @staticmethod
    def _fix(
        node_id, stamp_ns, longitude_deg, status=GnssPosition.STATUS_FIX
    ):
        message = GnssPosition()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = "gnss_link"
        message.node_id = node_id
        message.status = status
        message.latitude = 0.0
        message.longitude = longitude_deg
        message.altitude = math.nan
        return message

    def _publish_epoch(self, stamp_ns, offsets_ns=(0, 0, 0), statuses=None):
        # Equatorial WGS-84 chord construction: node 2 is 3 m east and node 3
        # is 4 m west of node 1, so the three horizontal chords are 3/4/~7 m.
        longitude_3m = math.degrees(2.0 * math.asin(3.0 / (2.0 * WGS84_A_M)))
        longitude_4m = -math.degrees(
            2.0 * math.asin(4.0 / (2.0 * WGS84_A_M))
        )
        longitudes = (0.0, longitude_3m, longitude_4m)
        statuses = statuses or (GnssPosition.STATUS_FIX,) * 3
        for node_id, offset_ns, longitude, status in zip(
            (1, 2, 3), offsets_ns, longitudes, statuses
        ):
            self.fix_publisher.publish(
                self._fix(
                    node_id, stamp_ns + offset_ns, longitude, status
                )
            )

    def _publish_common_time(
        self, stamp_ns, valid=True, frame_id="common_enu"
    ):
        for node_id in (1, 2, 3):
            message = NodeState()
            message.header.stamp.sec = stamp_ns // 1_000_000_000
            message.header.stamp.nanosec = stamp_ns % 1_000_000_000
            message.header.frame_id = frame_id
            message.node_id = node_id
            message.orientation_flu_to_enu_xyzw = [0.0, 0.0, 0.0, 1.0]
            message.valid = valid
            self.state_publisher.publish(message)
        time.sleep(0.05)

    def _spin_until_ranges(self, expected, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and len(self.ranges) < expected:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_valid_synchronized_fixes_generate_one_three_edge_epoch(self):
        self._wait_for_graph()
        stamp_ns = time.monotonic_ns()
        self._publish_common_time(stamp_ns)
        self._publish_epoch(
            stamp_ns, offsets_ns=(-20_000_000, -10_000_000, 0)
        )
        self._spin_until_ranges(3)

        self.assertEqual(len(self.ranges), 3)
        by_edge = {
            (message.src_id, message.target_id): message
            for message in self.ranges
        }
        self.assertEqual(set(by_edge), {(1, 2), (1, 3), (2, 3)})
        for message in by_edge.values():
            actual_stamp_ns = (
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            )
            self.assertEqual(actual_stamp_ns, stamp_ns)
            self.assertEqual(message.header.frame_id, "")
        self.assertAlmostEqual(by_edge[(1, 2)].distance, 3.0, places=5)
        self.assertAlmostEqual(by_edge[(1, 3)].distance, 4.0, places=5)
        self.assertAlmostEqual(by_edge[(2, 3)].distance, 7.0, places=5)

        # Updating only one vehicle must not reuse the other two cached fixes.
        self.ranges.clear()
        self.fix_publisher.publish(
            self._fix(1, stamp_ns + 1_000_000, 0.0)
        )
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # A newer NO_FIX must invalidate this node's previously cached fix.
        next_stamp_ns = time.monotonic_ns()
        self._publish_common_time(next_stamp_ns)
        longitude_3m = math.degrees(
            2.0 * math.asin(3.0 / (2.0 * WGS84_A_M))
        )
        longitude_4m = -math.degrees(
            2.0 * math.asin(4.0 / (2.0 * WGS84_A_M))
        )
        self.fix_publisher.publish(
            self._fix(2, next_stamp_ns, longitude_3m)
        )
        self._spin_until_ranges(1, timeout_s=0.1)
        self.assertEqual(self.ranges, [])
        self.fix_publisher.publish(self._fix(
            2,
            next_stamp_ns + 1_000_000,
            longitude_3m,
            GnssPosition.STATUS_NO_FIX,
        ))
        self._spin_until_ranges(1, timeout_s=0.1)
        self.assertEqual(self.ranges, [])
        self.fix_publisher.publish(self._fix(1, next_stamp_ns, 0.0))
        self.fix_publisher.publish(
            self._fix(3, next_stamp_ns, longitude_4m)
        )
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # Replaying the same three fixes must not emit a duplicate epoch.
        self._publish_epoch(
            stamp_ns, offsets_ns=(-20_000_000, -10_000_000, 0)
        )
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

    def test_bad_fix_skew_and_receive_timeout_do_not_emit_ranges(self):
        self._wait_for_graph()

        # A valid-looking trio is ignored until all three NodeState streams
        # establish a live UWB_SYSTEM_TIME reference.
        now_ns = time.monotonic_ns()
        self._publish_common_time(now_ns, valid=False)
        self._publish_epoch(now_ns)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # Wrong-frame and stale NodeState samples cannot establish the clock.
        now_ns = time.monotonic_ns()
        self._publish_common_time(now_ns, frame_id="map")
        self._publish_epoch(now_ns)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        now_ns = time.monotonic_ns()
        self._publish_common_time(now_ns)
        time.sleep(0.2)
        self._publish_epoch(now_ns + 200_000_000)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        now_ns = time.monotonic_ns()
        self._publish_common_time(now_ns)
        self._publish_epoch(now_ns - 600_000_000)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        now_ns = time.monotonic_ns()
        self._publish_common_time(now_ns)
        future_ns = now_ns + 200_000_000
        self._publish_epoch(future_ns)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        self._publish_epoch(0)
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # A real-UTC GnssPosition cannot pass a UWB_SYSTEM_TIME reference gate.
        self._publish_common_time(time.monotonic_ns())
        self._publish_epoch(time.time_ns())
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        stamp_ns = time.monotonic_ns()
        self._publish_common_time(stamp_ns)
        # One NO_FIX invalidates the whole synchronized epoch.
        self._publish_epoch(
            stamp_ns,
            statuses=(
                GnssPosition.STATUS_FIX,
                GnssPosition.STATUS_NO_FIX,
                GnssPosition.STATUS_FIX,
            ),
        )
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # An 80 ms sensor-stamp spread exceeds the configured 50 ms slop.
        stamp_ns = time.monotonic_ns()
        self._publish_common_time(stamp_ns)
        self._publish_epoch(stamp_ns, offsets_ns=(-80_000_000, -70_000_000, 0))
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])

        # Cached fixes older than the receive-time timeout are not combined.
        stamp_ns = time.monotonic_ns()
        self._publish_common_time(stamp_ns)
        self.fix_publisher.publish(self._fix(1, stamp_ns, 0.0))
        longitude_3m = math.degrees(
            2.0 * math.asin(3.0 / (2.0 * WGS84_A_M))
        )
        longitude_4m = -math.degrees(
            2.0 * math.asin(4.0 / (2.0 * WGS84_A_M))
        )
        self.fix_publisher.publish(self._fix(2, stamp_ns, longitude_3m))
        time.sleep(0.2)
        self.fix_publisher.publish(self._fix(3, stamp_ns, longitude_4m))
        self._spin_until_ranges(1, timeout_s=0.3)
        self.assertEqual(self.ranges, [])


@launch_testing.post_shutdown_test()
class TestGnssRangeFallbackShutdown(unittest.TestCase):

    def test_clean_shutdown(self, proc_info, fallback):
        launch_testing.asserts.assertExitCodes(proc_info, process=fallback)

"""Verify that three same-epoch UWB edges produce GCS output without IMU."""

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
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data

from cooperative_interfaces.msg import UwbRange
from cooperative_localization_msgs.msg import CooperativePose2DArray


UWB_TOPIC = "/uwb/range"
POSE_TOPIC = "/cooperative_localization/poses_2d"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    fusion = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_cooperative_fusion_node",
        name="zju_uwb_only_fusion_test",
        output="screen",
        parameters=[{
            "input_mode": "uwb_only",
            "node_ids": [1, 2, 3],
            "reference_node_id": 1,
            "enable_follower_feedback": False,
            "uwb_epoch_wait_ms": 5,
            "uwb_epoch_timeout_ms": 30,
            "uwb_reciprocal_max_difference_m": 0.5,
            "uwb_maximum_range_rate_mps": 10.0,
            "uwb_range_jump_tolerance_m": 0.5,
        }],
        remappings=[
            ("uwb_range", UWB_TOPIC),
            ("poses_2d", POSE_TOPIC),
        ],
    )
    return (
        launch.LaunchDescription([
            fusion,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"fusion": fusion},
    )


class TestUwbOnlyPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_uwb_only_test_driver")
        self.publisher = self.node.create_publisher(
            UwbRange, UWB_TOPIC, qos_profile_sensor_data
        )
        self.outputs = []
        self.subscription = self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.outputs.append,
            10,
        )

    def tearDown(self):
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()

    def _wait_for_graph(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.publisher.get_subscription_count() == 1
                and self.node.count_publishers(POSE_TOPIC) == 1
            ):
                return
        self.fail("UWB-only ROS graph did not become ready")

    def _publish_range(self, stamp_ns, src_id, target_id, distance_m):
        message = UwbRange()
        message.header.stamp = Time(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )
        message.src_id = src_id
        message.target_id = target_id
        message.distance = distance_m
        self.publisher.publish(message)

    def _wait_for_output(self, previous_count):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if len(self.outputs) > previous_count:
                return self.outputs[-1]
        self.fail("no UWB-only GCS output was published")

    def _assert_no_output(self, previous_count, duration_s=0.15):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(len(self.outputs), previous_count)

    def test_three_edges_form_canonical_triangle_without_node_state(self):
        self._wait_for_graph()
        stamp_ns = 12_345_678_900
        self._publish_range(stamp_ns, 1, 2, 3.0)
        self._publish_range(stamp_ns, 1, 3, 4.0)
        self._publish_range(stamp_ns, 2, 3, 5.0)
        output = self._wait_for_output(0)

        self.assertEqual(output.header.frame_id, "coop_ref_1_relative")
        self.assertEqual(output.reference_node_id, 1)
        self.assertEqual(len(output.vehicles), 3)
        poses = {pose.node_id: pose for pose in output.vehicles}
        self.assertAlmostEqual(poses[1].x_m, 0.0, places=9)
        self.assertAlmostEqual(poses[1].y_m, 0.0, places=9)
        self.assertAlmostEqual(poses[2].x_m, 3.0, places=6)
        self.assertAlmostEqual(poses[2].y_m, 0.0, places=6)
        self.assertAlmostEqual(poses[3].x_m, 0.0, places=6)
        self.assertAlmostEqual(poses[3].y_m, 4.0, places=6)
        self.assertTrue(all(pose.position_valid for pose in poses.values()))
        self.assertTrue(all(not pose.yaw_valid for pose in poses.values()))

        invalid_stamp_ns = stamp_ns + 100_000_000
        previous_count = len(self.outputs)
        self._publish_range(invalid_stamp_ns, 1, 2, 1.0)
        self._publish_range(invalid_stamp_ns, 1, 3, 1.0)
        self._publish_range(invalid_stamp_ns, 2, 3, 3.0)
        invalid_output = self._wait_for_output(previous_count)
        self.assertTrue(
            all(not pose.position_valid for pose in invalid_output.vehicles)
        )
        self.assertTrue(
            all(not pose.yaw_valid for pose in invalid_output.vehicles)
        )

        reciprocal_stamp_ns = stamp_ns + 200_000_000
        previous_count = len(self.outputs)
        self._publish_range(reciprocal_stamp_ns, 1, 2, 3.0)
        self._publish_range(reciprocal_stamp_ns, 2, 1, 3.2)
        self._publish_range(reciprocal_stamp_ns, 1, 3, 4.0)
        self._publish_range(reciprocal_stamp_ns, 3, 1, 4.0)
        self._publish_range(reciprocal_stamp_ns, 2, 3, 5.0)
        self._publish_range(reciprocal_stamp_ns, 3, 2, 5.0)
        reciprocal_output = self._wait_for_output(previous_count)
        reciprocal_poses = {
            pose.node_id: pose for pose in reciprocal_output.vehicles
        }
        self.assertTrue(
            all(pose.position_valid for pose in reciprocal_poses.values())
        )
        self.assertAlmostEqual(reciprocal_poses[2].x_m, 3.1, places=6)

        mismatch_stamp_ns = stamp_ns + 300_000_000
        previous_count = len(self.outputs)
        self._publish_range(mismatch_stamp_ns, 1, 2, 3.0)
        self._publish_range(mismatch_stamp_ns, 2, 1, 4.0)
        self._publish_range(mismatch_stamp_ns, 1, 3, 4.0)
        self._publish_range(mismatch_stamp_ns, 3, 1, 4.0)
        self._publish_range(mismatch_stamp_ns, 2, 3, 5.0)
        self._publish_range(mismatch_stamp_ns, 3, 2, 5.0)
        mismatch_output = self._wait_for_output(previous_count)
        # 两个方向都到达时取均值；方向差异不再由ROS适配层单独拦截，
        # 最终只由三角形几何条件决定是否有效。
        self.assertTrue(
            all(pose.position_valid for pose in mismatch_output.vehicles)
        )

        incomplete_stamp_ns = stamp_ns + 400_000_000
        previous_count = len(self.outputs)
        self._publish_range(incomplete_stamp_ns, 1, 2, 3.1)
        self._publish_range(incomplete_stamp_ns, 1, 3, 4.0)
        self._assert_no_output(previous_count)


@launch_testing.post_shutdown_test()
class TestUwbOnlyPipelineShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

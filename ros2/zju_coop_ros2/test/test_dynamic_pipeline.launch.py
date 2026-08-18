"""Deterministic dynamic test for the three-vehicle ROS 2 pipeline."""

import math
import statistics
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState
from zju_coop_test_msgs.msg import UwbRange


POSE_TOPIC = "/cooperative_localization/poses_2d"
NODE_STATE_TOPIC = "/cooperative_localization/node_state"
UWB_TOPIC = "/uwb/range"
NODE_IDS = (1, 2, 3)
IMU_DT_S = 0.01
UWB_PERIOD_SAMPLES = 5
UWB_DELAY_SAMPLES = 5
RUN_DURATION_S = 6.0
GRAVITY_MPS2 = 9.80665


def _truth(node_id, elapsed_s):
    """Return ENU position, velocity, yaw and yaw rate at elapsed_s."""
    if node_id == 1:
        return (
            (0.6 * elapsed_s, 0.0, 0.0),
            (0.6, 0.0, 0.0),
            0.0,
            0.0,
        )
    if node_id == 2:
        radius_m = 3.0
        yaw_rate_rad_s = 0.15
        yaw = yaw_rate_rad_s * elapsed_s
        return (
            (
                4.0 + radius_m * math.sin(yaw),
                radius_m * (1.0 - math.cos(yaw)),
                0.0,
            ),
            (
                radius_m * yaw_rate_rad_s * math.cos(yaw),
                radius_m * yaw_rate_rad_s * math.sin(yaw),
                0.0,
            ),
            yaw,
            yaw_rate_rad_s,
        )
    if node_id == 3:
        return (
            (0.0, 3.0 + 0.3 * elapsed_s, 0.0),
            (0.0, 0.3, 0.0),
            math.pi / 2.0,
            0.0,
        )
    raise ValueError(f"unknown node_id {node_id}")


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _local_node(node_id, initial_position):
    _, velocity, yaw, _ = _truth(node_id, 0.0)
    return launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_dynamic_local_inertial_node_{node_id}",
        output="screen",
        parameters=[{
            "node_id": node_id,
            "publish_rate_hz": 20.0,
            "expected_imu_frame_id": "imu_link",
            "common_enu_frame_id": "common_enu",
            "initial_position_enu_m": list(initial_position),
            "initial_velocity_enu_mps": list(velocity),
            "initial_orientation_flu_to_enu_xyzw": list(
                _yaw_quaternion(yaw)
            ),
            "initial_gyro_bias_flu_rad_s": [0.0, 0.0, 0.0],
            "initial_accel_bias_flu_m_s2": [0.0, 0.0, 0.0],
        }],
        remappings=[
            ("imu", f"/vehicle_{node_id}/imu/data"),
            ("node_state", NODE_STATE_TOPIC),
        ],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Node 2 deliberately starts 0.5 m east of truth. Its local INS keeps this
    # baseline error; pairwise UWB must remove it from the GCS result.
    filter_initial_positions = (
        (0.0, 0.0, 0.0),
        (4.5, 0.0, 0.0),
        (0.0, 3.0, 0.0),
    )
    local_nodes = [
        _local_node(node_id, position)
        for node_id, position in zip(NODE_IDS, filter_initial_positions)
    ]
    fusion_node = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_cooperative_fusion_node",
        name="zju_dynamic_cooperative_fusion_node",
        output="screen",
        parameters=[{
            "node_ids": list(NODE_IDS),
            "reference_node_id": 1,
            "publish_rate_hz": 10.0,
            "range_std_m": 0.1,
            "node_state_timeout_ms": 300,
            "common_enu_frame_id": "common_enu",
        }],
        remappings=[
            ("node_state", NODE_STATE_TOPIC),
            ("uwb_range", UWB_TOPIC),
            ("poses_2d", POSE_TOPIC),
        ],
    )

    return (
        launch.LaunchDescription([
            *local_nodes,
            fusion_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"local_nodes": local_nodes, "fusion_node": fusion_node},
    )


class TestDynamicPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_coop_dynamic_pipeline_test_driver")
        self.imu_publishers = {
            node_id: self.node.create_publisher(
                Imu, f"/vehicle_{node_id}/imu/data", qos_profile_sensor_data
            )
            for node_id in NODE_IDS
        }
        self.uwb_publisher = self.node.create_publisher(
            UwbRange, UWB_TOPIC, qos_profile_sensor_data
        )
        self.node_states = {node_id: [] for node_id in NODE_IDS}
        self.node_state_subscription = self.node.create_subscription(
            NodeState,
            NODE_STATE_TOPIC,
            self._record_node_state,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.pose_messages = []
        self.pose_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.pose_messages.append,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

    def tearDown(self):
        for publisher in self.imu_publishers.values():
            self.node.destroy_publisher(publisher)
        self.node.destroy_publisher(self.uwb_publisher)
        self.node.destroy_subscription(self.node_state_subscription)
        self.node.destroy_subscription(self.pose_subscription)
        self.node.destroy_node()

    def _record_node_state(self, message):
        if message.node_id in self.node_states:
            self.node_states[message.node_id].append(message)

    def _wait_for_graph(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                all(
                    publisher.get_subscription_count() >= 1
                    for publisher in self.imu_publishers.values()
                )
                and self.uwb_publisher.get_subscription_count() >= 1
                and self.node.count_publishers(NODE_STATE_TOPIC) >= 3
                and self.node.count_publishers(POSE_TOPIC) >= 1
            ):
                return
        self.fail("ROS graph did not expose the dynamic pipeline endpoints")

    @staticmethod
    def _set_stamp(message, timestamp_ns):
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000

    @classmethod
    def _imu(cls, node_id, timestamp_ns, elapsed_s):
        _, _, yaw, yaw_rate = _truth(node_id, elapsed_s)
        message = Imu()
        cls._set_stamp(message, timestamp_ns)
        message.header.frame_id = "imu_link"
        quaternion = _yaw_quaternion(yaw)
        message.orientation.x = quaternion[0]
        message.orientation.y = quaternion[1]
        message.orientation.z = quaternion[2]
        message.orientation.w = quaternion[3]
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.z = yaw_rate
        if node_id == 2:
            message.linear_acceleration.y = 3.0 * yaw_rate * yaw_rate
        message.linear_acceleration.z = GRAVITY_MPS2
        return message

    def _publish_ranges(self, timestamp_ns, elapsed_s):
        positions = {
            node_id: _truth(node_id, elapsed_s)[0] for node_id in NODE_IDS
        }
        for source, target in ((1, 2), (1, 3), (2, 3)):
            delta = tuple(
                positions[target][axis] - positions[source][axis]
                for axis in range(3)
            )
            message = UwbRange()
            self._set_stamp(message, timestamp_ns)
            message.header.frame_id = "common_enu"
            message.src_id = source
            message.target_id = target
            message.distance = math.sqrt(sum(value * value for value in delta))
            self.uwb_publisher.publish(message)

    @staticmethod
    def _timestamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _complete_pose(message):
        return (
            message.header.frame_id == "coop_ref_1_enu"
            and message.reference_node_id == 1
            and len(message.vehicles) == 3
            and {vehicle.node_id for vehicle in message.vehicles}
            == set(NODE_IDS)
            and all(vehicle.position_valid for vehicle in message.vehicles)
            and all(vehicle.yaw_valid for vehicle in message.vehicles)
        )

    @staticmethod
    def _angle_error(left, right):
        return abs(math.atan2(math.sin(left - right), math.cos(left - right)))

    def test_dynamic_motion_time_alignment_and_uwb_correction(self):
        self._wait_for_graph()
        start_ns = self.node.get_clock().now().nanoseconds
        sample_count = int(RUN_DURATION_S / IMU_DT_S)
        start_wall = time.monotonic()

        for sample_index in range(sample_count + 1):
            target_wall = start_wall + sample_index * IMU_DT_S
            remaining = target_wall - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)

            elapsed_s = sample_index * IMU_DT_S
            timestamp_ns = start_ns + int(round(elapsed_s * 1.0e9))
            for node_id, publisher in self.imu_publishers.items():
                publisher.publish(self._imu(node_id, timestamp_ns, elapsed_s))

            if (
                sample_index >= 2 * UWB_DELAY_SAMPLES
                and sample_index % UWB_PERIOD_SAMPLES == 0
            ):
                range_index = sample_index - UWB_DELAY_SAMPLES
                range_elapsed_s = range_index * IMU_DT_S
                range_timestamp_ns = start_ns + int(
                    round(range_elapsed_s * 1.0e9)
                )
                self._publish_ranges(range_timestamp_ns, range_elapsed_s)

            # Four subscriptions can become ready together (three NodeState
            # streams plus GCS), so drain a bounded batch every simulator tick.
            for _ in range(4):
                rclpy.spin_once(self.node, timeout_sec=0.0)

        collect_deadline = time.monotonic() + 0.5
        while time.monotonic() < collect_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        complete = [
            message for message in self.pose_messages
            if self._complete_pose(message)
        ]
        self.assertGreaterEqual(len(complete), 30)
        final_pose = complete[-1]
        final_elapsed_s = (
            self._timestamp_ns(final_pose.header.stamp) - start_ns
        ) * 1.0e-9
        self.assertGreater(final_elapsed_s, RUN_DURATION_S - 0.5)

        vehicles = {vehicle.node_id: vehicle for vehicle in final_pose.vehicles}
        reference_truth = _truth(1, final_elapsed_s)[0]
        for node_id, vehicle in vehicles.items():
            position, _, yaw, _ = _truth(node_id, final_elapsed_s)
            expected_x = position[0] - reference_truth[0]
            expected_y = position[1] - reference_truth[1]
            position_error = math.hypot(
                vehicle.x_m - expected_x, vehicle.y_m - expected_y
            )
            self.assertLess(position_error, 0.15)
            self.assertLess(self._angle_error(vehicle.yaw_rad, yaw), 0.01)

        self.assertAlmostEqual(vehicles[1].x_m, 0.0, places=9)
        self.assertAlmostEqual(vehicles[1].y_m, 0.0, places=9)

        for node_id, messages in self.node_states.items():
            timestamps = [
                self._timestamp_ns(message.header.stamp) for message in messages
            ]
            self.assertGreaterEqual(len(timestamps), 70)
            self.assertTrue(
                all(left < right for left, right in zip(timestamps, timestamps[1:]))
            )
            intervals_s = [
                (right - left) * 1.0e-9
                for left, right in zip(timestamps, timestamps[1:])
            ]
            self.assertGreater(statistics.median(intervals_s), 0.03)
            self.assertLess(statistics.median(intervals_s), 0.08)

        latest_node_1 = self.node_states[1][-1]
        latest_node_2 = self.node_states[2][-1]
        node_time_s = (
            min(
                self._timestamp_ns(latest_node_1.header.stamp),
                self._timestamp_ns(latest_node_2.header.stamp),
            ) - start_ns
        ) * 1.0e-9
        node_1_truth = _truth(1, node_time_s)[0]
        node_2_truth = _truth(2, node_time_s)[0]
        raw_relative_x = (
            latest_node_2.position_enu_m[0]
            - latest_node_1.position_enu_m[0]
        )
        true_relative_x = node_2_truth[0] - node_1_truth[0]
        raw_error = abs(raw_relative_x - true_relative_x)
        fused_error = abs(
            vehicles[2].x_m
            - (_truth(2, final_elapsed_s)[0][0] - reference_truth[0])
        )
        self.assertGreater(raw_error, 0.4)
        self.assertLess(fused_error, 0.15)

        dropout_start = len(self.pose_messages)
        dropout_samples = int(0.5 / IMU_DT_S)
        dropout_wall = time.monotonic()
        dropout_start_ns = self.node.get_clock().now().nanoseconds
        for offset in range(1, dropout_samples + 1):
            target_wall = dropout_wall + offset * IMU_DT_S
            remaining = target_wall - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            timestamp_ns = dropout_start_ns + int(round(offset * IMU_DT_S * 1.0e9))
            elapsed_s = (timestamp_ns - start_ns) * 1.0e-9
            for node_id, publisher in self.imu_publishers.items():
                publisher.publish(self._imu(node_id, timestamp_ns, elapsed_s))
            for _ in range(4):
                rclpy.spin_once(self.node, timeout_sec=0.0)

        collect_deadline = time.monotonic() + 0.2
        while time.monotonic() < collect_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stale = next(
            (
                message for message in reversed(self.pose_messages[dropout_start:])
                if len(message.vehicles) == 3
                and {vehicle.node_id for vehicle in message.vehicles}
                == set(NODE_IDS)
            ),
            None,
        )
        self.assertIsNotNone(stale)
        stale_vehicles = {vehicle.node_id: vehicle for vehicle in stale.vehicles}
        self.assertTrue(stale_vehicles[1].position_valid)
        self.assertFalse(stale_vehicles[2].position_valid)
        self.assertFalse(stale_vehicles[3].position_valid)
        self.assertTrue(all(vehicle.yaw_valid for vehicle in stale.vehicles))


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

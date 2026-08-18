"""End-to-end smoke test for the minimal three-vehicle ROS 2 pipeline."""

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
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState
from zju_coop_test_msgs.msg import UwbRange


POSE_TOPIC = "/cooperative_localization/poses_2d"
NODE_STATE_TOPIC = "/cooperative_localization/node_state"
UWB_TOPIC = "/uwb/range"
NODE_IDS = (1, 2, 3)
INITIAL_POSITIONS = (
    (0.0, 0.0, 0.0),
    (5.0, 0.0, 0.0),
    (0.0, 5.0, 0.0),
)
INITIAL_ORIENTATIONS = (
    (0.0, 0.0, 0.0, 1.0),
    (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    (0.0, 0.0, math.sin(-math.pi / 8.0), math.cos(-math.pi / 8.0)),
)


def _local_node(node_id, initial_position, initial_orientation):
    return launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_local_inertial_node_{node_id}",
        output="screen",
        parameters=[{
            "node_id": node_id,
            "publish_rate_hz": 10.0,
            "expected_imu_frame_id": "imu_link",
            "initial_position_enu_m": list(initial_position),
            "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
            "initial_orientation_flu_to_enu_xyzw": list(initial_orientation),
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
    local_nodes = [
        _local_node(node_id, position, orientation)
        for node_id, position, orientation in zip(
            NODE_IDS, INITIAL_POSITIONS, INITIAL_ORIENTATIONS
        )
    ]
    fusion_node = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_cooperative_fusion_node",
        name="zju_cooperative_fusion_node_test",
        output="screen",
        parameters=[{
            "node_ids": list(NODE_IDS),
            "reference_node_id": 1,
            "publish_rate_hz": 10.0,
            "range_std_m": 0.1,
            "node_state_timeout_ms": 2000,
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


class TestMinimalPipeline(unittest.TestCase):

    def test_node_state_v1_contract_is_frozen(self):
        self.assertEqual(
            NodeState.get_fields_and_field_types(),
            {
                "header": "std_msgs/Header",
                "node_id": "uint32",
                "position_enu_m": "double[3]",
                "velocity_enu_mps": "double[3]",
                "orientation_flu_to_enu_xyzw": "double[4]",
                "valid": "boolean",
            },
        )

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_coop_pipeline_test_driver")
        self.imu_publishers = [
            self.node.create_publisher(
                Imu, f"/vehicle_{node_id}/imu/data", qos_profile_sensor_data
            )
            for node_id in NODE_IDS
        ]
        self.uwb_publisher = self.node.create_publisher(
            UwbRange, UWB_TOPIC, qos_profile_sensor_data
        )
        self.pose_messages = []
        self.pose_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.pose_messages.append,
            qos_profile_sensor_data,
        )

    def tearDown(self):
        for publisher in self.imu_publishers:
            self.node.destroy_publisher(publisher)
        self.node.destroy_publisher(self.uwb_publisher)
        self.node.destroy_subscription(self.pose_subscription)
        self.node.destroy_node()

    def _wait_for_graph(self, timeout_sec=10.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            imu_ready = all(
                publisher.get_subscription_count() >= 1
                for publisher in self.imu_publishers
            )
            uwb_ready = self.uwb_publisher.get_subscription_count() >= 1
            pose_ready = self.node.count_publishers(POSE_TOPIC) >= 1
            if imu_ready and uwb_ready and pose_ready:
                return
        self.fail("ROS graph did not expose all IMU, UWB and Pose2D endpoints")

    @staticmethod
    def _imu(stamp):
        message = Imu()
        message.header.stamp.sec = stamp.sec
        message.header.stamp.nanosec = stamp.nanosec
        message.header.frame_id = "imu_link"
        message.orientation.w = 1.0
        message.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        message.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001,
        ]
        message.linear_acceleration.z = 9.80665
        message.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        return message

    def _publish_ranges(self, stamp):
        for source, target, distance in (
            (1, 2, 4.0),
            (1, 3, 5.0),
            (2, 3, math.sqrt(41.0)),
        ):
            message = UwbRange()
            message.header.stamp.sec = stamp.sec
            message.header.stamp.nanosec = stamp.nanosec
            message.header.frame_id = "common_enu"
            message.src_id = source
            message.target_id = target
            message.distance = float(distance)
            self.uwb_publisher.publish(message)

    @staticmethod
    def _complete_pose(message):
        return (
            message.reference_node_id == 1
            and len(message.vehicles) == 3
            and {vehicle.node_id for vehicle in message.vehicles} == set(NODE_IDS)
            and all(vehicle.position_valid for vehicle in message.vehicles)
            and all(vehicle.yaw_valid for vehicle in message.vehicles)
        )

    @classmethod
    def _uwb_corrected_pose(cls, message):
        if not cls._complete_pose(message):
            return False
        vehicle_2 = next(
            vehicle for vehicle in message.vehicles if vehicle.node_id == 2
        )
        return abs(vehicle_2.x_m - 4.0) < 0.5

    def test_standard_imu_and_uwb_produce_gcs_pose_array(self):
        self._wait_for_graph()

        accepted = None
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and accepted is None:
            stamp = self.node.get_clock().now().to_msg()
            for publisher in self.imu_publishers:
                publisher.publish(self._imu(stamp))
            # Give local callbacks and timers time to forward this common-stamp
            # batch before sending ranges; the loop tolerates timer phase skew.
            time.sleep(0.06)
            self._publish_ranges(stamp)

            rclpy.spin_once(self.node, timeout_sec=0.05)
            accepted = next(
                (message for message in reversed(self.pose_messages)
                 if self._uwb_corrected_pose(message)),
                None,
            )

        self.assertIsNotNone(accepted, "no complete valid three-vehicle pose received")
        self.assertEqual(accepted.header.frame_id, "coop_ref_1_enu")
        self.assertNotEqual(accepted.header.stamp.sec, 0)

        vehicles = {vehicle.node_id: vehicle for vehicle in accepted.vehicles}
        reference = vehicles[1]
        self.assertAlmostEqual(reference.x_m, 0.0, places=9)
        self.assertAlmostEqual(reference.y_m, 0.0, places=9)
        self.assertLess(abs(vehicles[2].x_m - 4.0), 0.5)
        expected_yaws = {1: 0.0, 2: math.pi / 2.0, 3: -math.pi / 4.0}
        for node_id, vehicle in vehicles.items():
            self.assertTrue(math.isfinite(vehicle.x_m))
            self.assertTrue(math.isfinite(vehicle.y_m))
            self.assertTrue(math.isfinite(vehicle.yaw_rad))
            self.assertAlmostEqual(
                vehicle.yaw_rad, expected_yaws[node_id], places=6
            )


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

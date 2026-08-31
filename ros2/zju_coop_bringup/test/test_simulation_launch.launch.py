"""Acceptance test for the directly runnable three-vehicle simulation."""

import math
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from launch_ros.substitutions import FindPackageShare
from sensor_msgs.msg import Imu

from cooperative_localization_msgs.msg import CooperativePose2DArray
from cooperative_interfaces.msg import UwbRange


POSE_TOPIC = "/cooperative_localization/poses_2d"
TRUTH_TOPIC = "/simulation/ground_truth/poses_2d"
NODE_IDS = {1, 2, 3}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = PathJoinSubstitution([
        FindPackageShare("zju_coop_bringup"),
        "launch",
        "simulation.launch.py",
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "uwb_noise_std_m": "0.02",
            "uwb_drop_every_n": "10",
            "uwb_nlos_every_n": "25",
            "gyro_noise_std_rad_s": "0.0002",
            "accel_noise_std_m_s2": "0.01",
        }.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimulationLaunch(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_coop_simulation_launch_test")
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.poses = []
        self.truth = []
        self.uwb_messages = []
        self.imu_messages = []
        self.pose_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.poses.append,
            qos,
        )
        self.truth_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            TRUTH_TOPIC,
            self.truth.append,
            qos,
        )
        self.uwb_subscription = self.node.create_subscription(
            UwbRange,
            "/uwb/range",
            self.uwb_messages.append,
            qos_profile_sensor_data,
        )
        self.imu_subscription = self.node.create_subscription(
            Imu,
            "/vehicle_1/imu/data",
            self.imu_messages.append,
            qos_profile_sensor_data,
        )

    def tearDown(self):
        self.node.destroy_subscription(self.pose_subscription)
        self.node.destroy_subscription(self.truth_subscription)
        self.node.destroy_subscription(self.uwb_subscription)
        self.node.destroy_subscription(self.imu_subscription)
        self.node.destroy_node()

    @staticmethod
    def _stamp_ns(message):
        return (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )

    @staticmethod
    def _complete(message):
        return (
            message.header.frame_id == "coop_ref_1_enu"
            and message.reference_node_id == 1
            and {vehicle.node_id for vehicle in message.vehicles} == NODE_IDS
            and all(vehicle.position_valid for vehicle in message.vehicles)
            and all(vehicle.yaw_valid for vehicle in message.vehicles)
        )

    @staticmethod
    def _angle_error(left, right):
        return abs(math.atan2(math.sin(left - right), math.cos(left - right)))

    def test_simulation_produces_truth_and_corrected_gcs_output(self):
        graph_deadline = time.monotonic() + 15.0
        while time.monotonic() < graph_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.node.count_publishers(POSE_TOPIC) >= 1
                and self.node.count_publishers(TRUTH_TOPIC) >= 1
            ):
                break
        else:
            self.fail("simulation launch did not expose GCS and truth topics")

        collect_deadline = time.monotonic() + 7.0
        while time.monotonic() < collect_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            complete_poses = [message for message in self.poses if self._complete(message)]
            complete_truth = [message for message in self.truth if self._complete(message)]
            if len(complete_poses) >= 30 and len(complete_truth) >= 60:
                break

        complete_poses = [message for message in self.poses if self._complete(message)]
        complete_truth = [message for message in self.truth if self._complete(message)]
        self.assertGreaterEqual(len(complete_poses), 30)
        self.assertGreaterEqual(len(complete_truth), 60)

        final_pose = complete_poses[-1]
        pose_time = self._stamp_ns(final_pose)
        matching_truth = min(
            complete_truth,
            key=lambda message: abs(self._stamp_ns(message) - pose_time),
        )
        self.assertLess(abs(self._stamp_ns(matching_truth) - pose_time), 80_000_000)

        poses = {vehicle.node_id: vehicle for vehicle in final_pose.vehicles}
        truth = {vehicle.node_id: vehicle for vehicle in matching_truth.vehicles}
        for node_id in NODE_IDS:
            position_error = math.hypot(
                poses[node_id].x_m - truth[node_id].x_m,
                poses[node_id].y_m - truth[node_id].y_m,
            )
            self.assertLess(position_error, 0.15)
            self.assertLess(
                self._angle_error(poses[node_id].yaw_rad, truth[node_id].yaw_rad),
                0.01,
            )

        pose_stamps = [self._stamp_ns(message) for message in complete_poses]
        truth_stamps = [self._stamp_ns(message) for message in complete_truth]
        self.assertGreater(pose_stamps[-1] - pose_stamps[0], 2_000_000_000)
        self.assertGreater(truth_stamps[-1] - truth_stamps[0], 2_000_000_000)

        edge_12_stamps = [
            self._stamp_ns(message)
            for message in self.uwb_messages
            if {message.src_id, message.target_id} == {1, 2}
        ]
        self.assertTrue(
            any(
                right - left > 75_000_000
                for left, right in zip(edge_12_stamps, edge_12_stamps[1:])
            ),
            "periodic UWB drop did not create a timestamp gap",
        )

        nlos_residuals = []
        for message in self.uwb_messages:
            if {message.src_id, message.target_id} != {2, 3}:
                continue
            message_time = self._stamp_ns(message)
            nearest = min(
                complete_truth,
                key=lambda candidate: abs(self._stamp_ns(candidate) - message_time),
            )
            if abs(self._stamp_ns(nearest) - message_time) >= 80_000_000:
                continue
            truth_by_id = {
                vehicle.node_id: vehicle for vehicle in nearest.vehicles
            }
            expected = math.hypot(
                truth_by_id[3].x_m - truth_by_id[2].x_m,
                truth_by_id[3].y_m - truth_by_id[2].y_m,
            )
            nlos_residuals.append(message.distance - expected)
        self.assertTrue(
            any(residual > 1.5 for residual in nlos_residuals),
            "periodic NLOS did not add the configured positive range bias",
        )
        self.assertTrue(
            any(
                abs(message.angular_velocity.x) > 1.0e-8
                or abs(message.linear_acceleration.x) > 1.0e-6
                for message in self.imu_messages
            ),
            "configured IMU white noise was not observed",
        )


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

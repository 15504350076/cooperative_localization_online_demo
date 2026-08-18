"""Headless acceptance test for the optional Gazebo Fortress pipeline."""

import math
from pathlib import Path
import tempfile
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_ros.substitutions import FindPackageShare
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
import yaml

from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState
from zju_coop_test_msgs.msg import UwbRange


NODE_IDS = {1, 2, 3}
POSE_TOPIC = "/cooperative_localization/poses_2d"
TRUTH_TOPIC = "/simulation/ground_truth/poses_2d"
NODE_STATE_TOPIC = "/cooperative_localization/node_state"
RECORDED_TOPICS = {
    "/clock",
    "/vehicle_1/imu/data",
    "/vehicle_2/imu/data",
    "/vehicle_3/imu/data",
    "/uwb/range",
    "/simulation/ground_truth/poses_2d",
    NODE_STATE_TOPIC,
    POSE_TOPIC,
}
BAG_TEMP_DIRECTORY = tempfile.TemporaryDirectory(
    prefix="zju_coop_gazebo_bag_test_"
)
BAG_OUTPUT = Path(BAG_TEMP_DIRECTORY.name) / "recording"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_file = PathJoinSubstitution([
        FindPackageShare("zju_coop_gazebo"),
        "launch",
        "three_vehicle_gazebo.launch.py",
    ])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            "headless": "true",
            "node_2_initial_east_m": "4.5",
            "record_bag": "true",
            "bag_output": str(BAG_OUTPUT),
        }.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestGazeboPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_gazebo_pipeline_test")
        sensor_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        result_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.imus = {node_id: [] for node_id in NODE_IDS}
        self.node_states = {node_id: [] for node_id in NODE_IDS}
        self.ranges = []
        self.poses = []
        self.truth = []
        self.subscriptions = []
        for node_id in NODE_IDS:
            self.subscriptions.append(self.node.create_subscription(
                Imu,
                f"/vehicle_{node_id}/imu/data",
                lambda message, source=node_id: self.imus[source].append(message),
                sensor_qos,
            ))
        self.subscriptions.extend([
            self.node.create_subscription(
                NodeState,
                NODE_STATE_TOPIC,
                self._record_node_state,
                sensor_qos,
            ),
            self.node.create_subscription(
                UwbRange, "/uwb/range", self.ranges.append, sensor_qos
            ),
            self.node.create_subscription(
                CooperativePose2DArray,
                POSE_TOPIC,
                self.poses.append,
                result_qos,
            ),
            self.node.create_subscription(
                CooperativePose2DArray,
                TRUTH_TOPIC,
                self.truth.append,
                result_qos,
            ),
        ])

    def tearDown(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_node()

    def _record_node_state(self, message):
        if message.node_id in self.node_states:
            self.node_states[message.node_id].append(message)

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

    @staticmethod
    def _roll_pitch(quaternion):
        roll = math.atan2(
            2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
            1.0 - 2.0 * (quaternion.x ** 2 + quaternion.y ** 2),
        )
        pitch = math.asin(max(-1.0, min(
            1.0,
            2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x),
        )))
        return roll, pitch

    def test_physics_imu_uwb_and_gcs_pipeline(self):
        graph_deadline = time.monotonic() + 25.0
        while time.monotonic() < graph_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                all(
                    self.node.count_publishers(f"/vehicle_{node_id}/imu/data")
                    >= 1
                    for node_id in NODE_IDS
                )
                and self.node.count_publishers("/uwb/range") >= 1
                and self.node.count_publishers(NODE_STATE_TOPIC) >= 3
                and self.node.count_publishers(POSE_TOPIC) >= 1
                and self.node.count_publishers(TRUTH_TOPIC) >= 1
            ):
                break
        else:
            self.fail("Gazebo pipeline did not expose all ROS endpoints")

        collect_deadline = time.monotonic() + 25.0
        while time.monotonic() < collect_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            complete_poses = [item for item in self.poses if self._complete(item)]
            complete_truth = [item for item in self.truth if self._complete(item)]
            if (
                len(complete_poses) >= 60
                and len(complete_truth) >= 120
                and all(len(messages) >= 400 for messages in self.imus.values())
                and all(
                    len(messages) >= 60 for messages in self.node_states.values()
                )
            ):
                if (
                    self._stamp_ns(complete_poses[-1])
                    - self._stamp_ns(complete_poses[0])
                    >= 12_000_000_000
                ):
                    break

        complete_poses = [item for item in self.poses if self._complete(item)]
        complete_truth = [item for item in self.truth if self._complete(item)]
        self.assertGreaterEqual(len(complete_poses), 60)
        self.assertGreaterEqual(len(complete_truth), 120)
        self.assertGreaterEqual(len(self.ranges), 300)
        for node_id in NODE_IDS:
            self.assertGreaterEqual(len(self.imus[node_id]), 400)
            self.assertGreaterEqual(len(self.node_states[node_id]), 60)
            self.assertTrue(all(
                message.header.frame_id == "imu_link"
                and self._stamp_ns(message) > 0
                for message in self.imus[node_id]
            ))
            tilts = [
                self._roll_pitch(message.orientation)
                for message in self.imus[node_id]
            ]
            self.assertLess(max(abs(roll) for roll, _ in tilts), 0.10)
            self.assertLess(max(abs(pitch) for _, pitch in tilts), 0.10)
            state_stamps = [
                self._stamp_ns(message) for message in self.node_states[node_id]
            ]
            self.assertTrue(all(
                left < right
                for left, right in zip(state_stamps, state_stamps[1:])
            ))

        first_truth_stamp = self._stamp_ns(complete_truth[0])
        stationary_truth = min(
            complete_truth,
            key=lambda message: abs(
                self._stamp_ns(message) - first_truth_stamp - 800_000_000
            ),
        )
        first_truth_by_id = {
            vehicle.node_id: vehicle for vehicle in complete_truth[0].vehicles
        }
        stationary_truth_by_id = {
            vehicle.node_id: vehicle for vehicle in stationary_truth.vehicles
        }
        for node_id in NODE_IDS:
            self.assertLess(math.hypot(
                stationary_truth_by_id[node_id].x_m
                - first_truth_by_id[node_id].x_m,
                stationary_truth_by_id[node_id].y_m
                - first_truth_by_id[node_id].y_m,
            ), 0.02)
            self.assertLess(self._angle_error(
                stationary_truth_by_id[node_id].yaw_rad,
                first_truth_by_id[node_id].yaw_rad,
            ), 0.01)
            stationary_imu = [
                message for message in self.imus[node_id]
                if first_truth_stamp
                <= self._stamp_ns(message)
                <= first_truth_stamp + 800_000_000
            ]
            self.assertGreaterEqual(len(stationary_imu), 30)
            mean_wz = sum(
                message.angular_velocity.z for message in stationary_imu
            ) / len(stationary_imu)
            mean_ax = sum(
                message.linear_acceleration.x for message in stationary_imu
            ) / len(stationary_imu)
            mean_ay = sum(
                message.linear_acceleration.y for message in stationary_imu
            ) / len(stationary_imu)
            mean_az = sum(
                message.linear_acceleration.z for message in stationary_imu
            ) / len(stationary_imu)
            self.assertLess(abs(mean_wz), 0.01)
            self.assertLess(abs(mean_ax), 0.05)
            self.assertLess(abs(mean_ay), 0.05)
            self.assertLess(abs(mean_az - 9.80665), 0.10)

        ranges_by_edge = {}
        for message in self.ranges:
            edge = tuple(sorted((message.src_id, message.target_id)))
            self.assertIn(edge, {(1, 2), (1, 3), (2, 3)})
            self.assertEqual(message.header.frame_id, "common_enu")
            self.assertGreater(self._stamp_ns(message), 0)
            self.assertTrue(math.isfinite(message.distance))
            self.assertGreater(message.distance, 0.0)
            ranges_by_edge.setdefault(edge, []).append(self._stamp_ns(message))
        self.assertEqual(set(ranges_by_edge), {(1, 2), (1, 3), (2, 3)})
        for stamps in ranges_by_edge.values():
            self.assertGreaterEqual(len(stamps), 180)
            self.assertTrue(all(
                left < right for left, right in zip(stamps, stamps[1:])
            ))

        final_pose = complete_poses[-1]
        pose_stamp = self._stamp_ns(final_pose)
        last_truth_by_id = {
            vehicle.node_id: vehicle for vehicle in complete_truth[-1].vehicles
        }
        for snapshot in complete_truth:
            vehicles = {
                vehicle.node_id: vehicle for vehicle in snapshot.vehicles
            }
            for first, second in ((1, 2), (1, 3), (2, 3)):
                self.assertGreater(math.hypot(
                    vehicles[first].x_m - vehicles[second].x_m,
                    vehicles[first].y_m - vehicles[second].y_m,
                ), 1.20)
        self.assertGreater(
            math.hypot(
                last_truth_by_id[2].x_m - first_truth_by_id[2].x_m,
                last_truth_by_id[2].y_m - first_truth_by_id[2].y_m,
            ),
            0.20,
        )
        node_2_truth = [
            next(
                vehicle for vehicle in snapshot.vehicles
                if vehicle.node_id == 2
            )
            for snapshot in complete_truth
        ]
        node_2_yaw_deltas = [
            math.atan2(
                math.sin(vehicle.yaw_rad - first_truth_by_id[2].yaw_rad),
                math.cos(vehicle.yaw_rad - first_truth_by_id[2].yaw_rad),
            )
            for vehicle in node_2_truth
        ]
        peak_index = max(
            range(len(node_2_yaw_deltas)),
            key=node_2_yaw_deltas.__getitem__,
        )
        peak_truth_stamp = self._stamp_ns(complete_truth[peak_index])
        self.assertGreater(node_2_yaw_deltas[peak_index], 0.30)
        self.assertLess(node_2_yaw_deltas[peak_index], 0.55)
        self.assertLess(abs(node_2_yaw_deltas[-1]), 0.12)
        self.assertGreater(
            node_2_yaw_deltas[peak_index] - node_2_yaw_deltas[-1],
            0.25,
        )
        self.assertGreater(peak_truth_stamp - first_truth_stamp, 4_300_000_000)
        self.assertLess(peak_truth_stamp - first_truth_stamp, 5_700_000_000)
        self.assertGreater(
            last_truth_by_id[2].y_m - first_truth_by_id[2].y_m,
            0.25,
        )
        turning_imu = sorted((
            message for message in self.imus[2]
            if first_truth_stamp + 1_000_000_000
            <= self._stamp_ns(message)
            <= first_truth_stamp + 9_200_000_000
        ), key=self._stamp_ns)
        positive_stamps = [
            self._stamp_ns(message) for message in turning_imu
            if message.angular_velocity.z > 0.05
        ]
        negative_stamps = [
            self._stamp_ns(message) for message in turning_imu
            if message.angular_velocity.z < -0.05
        ]
        self.assertGreaterEqual(len(positive_stamps), 150)
        self.assertGreaterEqual(len(negative_stamps), 150)
        self.assertLess(
            positive_stamps[len(positive_stamps) // 2],
            negative_stamps[len(negative_stamps) // 2],
        )
        angular_accelerations = []
        for left, right in zip(turning_imu, turning_imu[1:]):
            dt_s = (
                self._stamp_ns(right) - self._stamp_ns(left)
            ) * 1.0e-9
            if dt_s > 0.0:
                angular_accelerations.append(abs(
                    right.angular_velocity.z - left.angular_velocity.z
                ) / dt_s)
        angular_accelerations.sort()
        self.assertGreaterEqual(len(angular_accelerations), 500)
        self.assertLess(
            angular_accelerations[
                int(0.95 * (len(angular_accelerations) - 1))
            ],
            0.55,
        )
        peak_pose = min(
            complete_poses,
            key=lambda message: abs(
                self._stamp_ns(message) - peak_truth_stamp
            ),
        )
        self.assertLess(
            abs(self._stamp_ns(peak_pose) - peak_truth_stamp),
            100_000_000,
        )
        peak_pose_2 = next(
            vehicle for vehicle in peak_pose.vehicles if vehicle.node_id == 2
        )
        self.assertLess(
            self._angle_error(
                peak_pose_2.yaw_rad,
                node_2_truth[peak_index].yaw_rad,
            ),
            0.20,
        )
        matching_truth = min(
            complete_truth,
            key=lambda message: abs(self._stamp_ns(message) - pose_stamp),
        )
        self.assertLess(abs(self._stamp_ns(matching_truth) - pose_stamp), 100_000_000)
        pose_by_id = {vehicle.node_id: vehicle for vehicle in final_pose.vehicles}
        truth_by_id = {
            vehicle.node_id: vehicle for vehicle in matching_truth.vehicles
        }
        fused_errors = {}
        for node_id in NODE_IDS:
            fused_errors[node_id] = math.hypot(
                pose_by_id[node_id].x_m - truth_by_id[node_id].x_m,
                pose_by_id[node_id].y_m - truth_by_id[node_id].y_m,
            )
            self.assertLess(fused_errors[node_id], 0.50)
            self.assertLess(
                self._angle_error(
                    pose_by_id[node_id].yaw_rad,
                    truth_by_id[node_id].yaw_rad,
                ),
                0.25,
            )
        self.assertAlmostEqual(pose_by_id[1].x_m, 0.0, places=9)
        self.assertAlmostEqual(pose_by_id[1].y_m, 0.0, places=9)

        latest_1 = min(
            self.node_states[1],
            key=lambda message: abs(self._stamp_ns(message) - pose_stamp),
        )
        latest_2 = min(
            self.node_states[2],
            key=lambda message: abs(self._stamp_ns(message) - pose_stamp),
        )
        raw_relative = (
            latest_2.position_enu_m[0] - latest_1.position_enu_m[0],
            latest_2.position_enu_m[1] - latest_1.position_enu_m[1],
        )
        raw_error = math.hypot(
            raw_relative[0] - truth_by_id[2].x_m,
            raw_relative[1] - truth_by_id[2].y_m,
        )
        self.assertGreater(raw_error, 0.30)
        self.assertLess(fused_errors[2], 0.8 * raw_error)


@launch_testing.post_shutdown_test()
class TestRosbagWasSaved(unittest.TestCase):

    def test_recording_contains_all_pipeline_topics(self):
        metadata_path = BAG_OUTPUT / "metadata.yaml"
        self.assertTrue(metadata_path.is_file())
        self.assertTrue(any(BAG_OUTPUT.glob("*.db3")))

        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        topic_counts = {
            item["topic_metadata"]["name"]: item["message_count"]
            for item in metadata["rosbag2_bagfile_information"][
                "topics_with_message_count"
            ]
        }
        self.assertEqual(RECORDED_TOPICS - set(topic_counts), set())
        for topic in RECORDED_TOPICS:
            self.assertGreater(topic_counts[topic], 0, topic)

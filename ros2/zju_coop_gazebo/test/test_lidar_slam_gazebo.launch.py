"""Headless contract for the opt-in three-vehicle lidar SLAM pipeline."""

import math
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
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from cooperative_localization_msgs.msg import CooperativePose2DArray


NODE_IDS = (1, 2, 3)
MAP_TOPICS = {
    node_id: f"/vehicle_{node_id}/lidar_slam/map"
    for node_id in NODE_IDS
}
POSE_TOPIC = "/cooperative_localization/poses_2d"
SLAM_DATABASE_DIRECTORY = tempfile.TemporaryDirectory(
    prefix="zju_coop_lidar_slam_test_"
)


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
            "software_rendering": "true",
            "record_bag": "false",
            "enable_lidar_slam": "true",
            "lidar_slam_database_directory": SLAM_DATABASE_DIRECTORY.name,
        }.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestLidarSlamGazebo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_gazebo_lidar_slam_test")
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        pose_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.maps = {node_id: [] for node_id in NODE_IDS}
        self.poses = []
        self.subscriptions = [
            self.node.create_subscription(
                OccupancyGrid,
                topic,
                lambda message, source=node_id: self.maps[source].append(
                    message
                ),
                map_qos,
            )
            for node_id, topic in MAP_TOPICS.items()
        ]
        self.subscriptions.append(self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.poses.append,
            pose_qos,
        ))

    def tearDown(self):
        for subscription in self.subscriptions:
            self.node.destroy_subscription(subscription)
        self.node.destroy_node()

    @staticmethod
    def _complete_pose(message):
        return (
            message.reference_node_id == 1
            and {vehicle.node_id for vehicle in message.vehicles}
            == set(NODE_IDS)
            and all(vehicle.position_valid for vehicle in message.vehicles)
            and all(vehicle.yaw_valid for vehicle in message.vehicles)
        )

    def test_three_maps_and_cooperative_pose_are_available(self):
        deadline = time.monotonic() + 75.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (
                all(len(messages) >= 3 for messages in self.maps.values())
                and all(
                    (
                        messages[-1].header.stamp.sec
                        - messages[0].header.stamp.sec
                    )
                    + 1.0e-9 * (
                        messages[-1].header.stamp.nanosec
                        - messages[0].header.stamp.nanosec
                    )
                    >= 2.0
                    for messages in self.maps.values()
                )
                and any(self._complete_pose(pose) for pose in self.poses)
            ):
                break

        self.assertTrue(
            all(len(messages) >= 3 for messages in self.maps.values())
        )
        self.assertTrue(any(self._complete_pose(pose) for pose in self.poses))

        frame_ids = set()
        for node_id, messages in self.maps.items():
            topic = MAP_TOPICS[node_id]
            first_grid = messages[0]
            grid = messages[-1]
            self.assertGreaterEqual(
                self.node.count_publishers(topic), 1, topic
            )
            first_stamp_ns = (
                first_grid.header.stamp.sec * 1_000_000_000
                + first_grid.header.stamp.nanosec
            )
            last_stamp_ns = (
                grid.header.stamp.sec * 1_000_000_000
                + grid.header.stamp.nanosec
            )
            self.assertGreaterEqual(
                last_stamp_ns - first_stamp_ns,
                2_000_000_000,
                topic,
            )
            self.assertEqual(
                grid.header.frame_id,
                f"vehicle_{node_id}/lidar_map",
            )
            frame_ids.add(grid.header.frame_id)
            self.assertGreater(grid.info.resolution, 0.0)
            self.assertGreater(grid.info.width, 0)
            self.assertGreater(grid.info.height, 0)
            self.assertEqual(
                len(grid.data),
                grid.info.width * grid.info.height,
            )
            quaternion = grid.info.origin.orientation
            self.assertTrue(all(math.isfinite(value) for value in (
                quaternion.x,
                quaternion.y,
                quaternion.z,
                quaternion.w,
            )))
            self.assertTrue(all(-1 <= value <= 100 for value in grid.data))
            self.assertIn(-1, grid.data)
            self.assertIn(0, grid.data)
            self.assertTrue(any(value > 0 for value in grid.data))
            self.assertNotEqual(
                (
                    first_grid.info.width,
                    first_grid.info.height,
                    first_grid.info.origin.position.x,
                    first_grid.info.origin.position.y,
                    tuple(first_grid.data),
                ),
                (
                    grid.info.width,
                    grid.info.height,
                    grid.info.origin.position.x,
                    grid.info.origin.position.y,
                    tuple(grid.data),
                ),
                topic,
            )

        self.assertEqual(
            frame_ids,
            {f"vehicle_{node_id}/lidar_map" for node_id in NODE_IDS},
        )

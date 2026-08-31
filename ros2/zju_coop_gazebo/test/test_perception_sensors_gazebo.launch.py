"""Headless contract for the opt-in three-vehicle perception sensors."""

import math
import statistics
import struct
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
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState
from cooperative_interfaces.msg import UwbRange


NODE_IDS = (1, 2, 3)
NODE_ID_SET = set(NODE_IDS)
NODE_STATE_TOPIC = "/cooperative_localization/node_state"
POSE_TOPIC = "/cooperative_localization/poses_2d"


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
            "enable_lidar": "true",
            "enable_camera": "true",
            "enable_lidar_frontend": "true",
            "enable_visual_frontend": "true",
        }.items(),
    )
    return launch.LaunchDescription([
        simulation,
        launch_testing.actions.ReadyToTest(),
    ])


class TestPerceptionSensorsGazebo(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("zju_gazebo_perception_test")
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
        self.clouds = {node_id: [] for node_id in NODE_IDS}
        self.images = {node_id: [] for node_id in NODE_IDS}
        self.camera_infos = {node_id: [] for node_id in NODE_IDS}
        self.imus = {node_id: [] for node_id in NODE_IDS}
        self.node_states = {node_id: [] for node_id in NODE_IDS}
        self.lidar_odometry = {node_id: [] for node_id in NODE_IDS}
        self.visual_odometry = {node_id: [] for node_id in NODE_IDS}
        self.gazebo_poses = {node_id: [] for node_id in NODE_IDS}
        self.ranges = []
        self.poses = []
        self.subscriptions = []
        for node_id in NODE_IDS:
            self.subscriptions.extend([
                self.node.create_subscription(
                    PointCloud2,
                    f"/vehicle_{node_id}/lidar/points",
                    lambda message, source=node_id: self.clouds[source].append(
                        message
                    ),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    Image,
                    f"/vehicle_{node_id}/camera/image_raw",
                    lambda message, source=node_id: self.images[source].append(
                        message
                    ),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    CameraInfo,
                    f"/vehicle_{node_id}/camera/camera_info",
                    lambda message, source=node_id: self.camera_infos[
                        source
                    ].append(message),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    Imu,
                    f"/vehicle_{node_id}/imu/data",
                    lambda message, source=node_id: self.imus[source].append(
                        message
                    ),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    Odometry,
                    f"/vehicle_{node_id}/lidar/odometry",
                    lambda message, source=node_id: self.lidar_odometry[
                        source
                    ].append(message),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    Odometry,
                    f"/vehicle_{node_id}/visual/odometry",
                    lambda message, source=node_id: self.visual_odometry[
                        source
                    ].append(message),
                    sensor_qos,
                ),
                self.node.create_subscription(
                    TFMessage,
                    f"/simulation/gazebo/vehicle_{node_id}/pose",
                    lambda message, source=node_id: self._record_gazebo_pose(
                        source, message
                    ),
                    sensor_qos,
                ),
            ])
        self.subscriptions.extend([
            self.node.create_subscription(
                NodeState,
                NODE_STATE_TOPIC,
                self._record_node_state,
                sensor_qos,
            ),
            self.node.create_subscription(
                UwbRange,
                "/uwb/range",
                self.ranges.append,
                sensor_qos,
            ),
            self.node.create_subscription(
                CooperativePose2DArray,
                POSE_TOPIC,
                self.poses.append,
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

    def _record_gazebo_pose(self, node_id, message):
        if message.transforms:
            self.gazebo_poses[node_id].append(message.transforms[0])

    @staticmethod
    def _stamp_ns(message):
        return (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _angle_error(left, right):
        return abs(math.atan2(
            math.sin(left - right), math.cos(left - right)
        ))

    @classmethod
    def _nearest_transform(cls, transforms, stamp_ns):
        return min(
            transforms,
            key=lambda transform: abs(
                cls._stamp_ns(transform) - stamp_ns
            ),
        )

    def _assert_stamps_and_rate(self, messages, minimum_hz, maximum_hz):
        raw_stamps = [self._stamp_ns(message) for message in messages]
        # The bridge can expose one initial sample while Gazebo is still
        # paused at simulation time zero. Operational samples must be nonzero.
        self.assertLessEqual(sum(stamp == 0 for stamp in raw_stamps), 1)
        stamps = [stamp for stamp in raw_stamps if stamp > 0]
        self.assertGreaterEqual(len(stamps), len(raw_stamps) - 1)
        self.assertTrue(all(
            left < right for left, right in zip(stamps, stamps[1:])
        ))
        intervals_s = [
            (right - left) * 1.0e-9
            for left, right in zip(stamps, stamps[1:])
        ]
        median_rate_hz = 1.0 / statistics.median(intervals_s)
        self.assertGreaterEqual(median_rate_hz, minimum_hz)
        self.assertLessEqual(median_rate_hz, maximum_hz)

    @staticmethod
    def _cloud_has_usable_geometry(cloud):
        offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            return False
        byte_order = ">" if cloud.is_bigendian else "<"
        points = []
        for index in range(cloud.width * cloud.height):
            base = index * cloud.point_step
            xyz = tuple(
                struct.unpack_from(
                    byte_order + "f", cloud.data, base + offsets[axis]
                )[0]
                for axis in ("x", "y", "z")
            )
            if (all(math.isfinite(value) for value in xyz)
                    and math.hypot(xyz[0], xyz[1]) > 0.25):
                points.append(xyz)
        if len(points) < 80:
            return False
        return (
            max(point[0] for point in points)
            - min(point[0] for point in points) > 1.0
            and max(point[1] for point in points)
            - min(point[1] for point in points) > 1.0
        )

    @staticmethod
    def _complete_pose(message):
        return (
            message.header.frame_id == "coop_ref_1_enu"
            and message.reference_node_id == 1
            and {vehicle.node_id for vehicle in message.vehicles}
            == NODE_ID_SET
            and all(vehicle.position_valid for vehicle in message.vehicles)
            and all(vehicle.yaw_valid for vehicle in message.vehicles)
        )

    def test_perception_messages_and_pipeline_remain_healthy(self):
        graph_deadline = time.monotonic() + 25.0
        while time.monotonic() < graph_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            perception_topics = [
                topic
                for node_id in NODE_IDS
                for topic in (
                    f"/vehicle_{node_id}/lidar/points",
                    f"/vehicle_{node_id}/camera/image_raw",
                    f"/vehicle_{node_id}/camera/camera_info",
                )
            ]
            if (
                all(self.node.count_publishers(topic) >= 1
                    for topic in perception_topics)
                and all(
                    self.node.count_subscribers(
                        f"/vehicle_{node_id}/lidar/points"
                    ) >= 2
                    and self.node.count_subscribers(
                        f"/vehicle_{node_id}/camera/image_raw"
                    ) >= 2
                    for node_id in NODE_IDS
                )
                and all(
                    self.node.count_publishers(
                        f"/vehicle_{node_id}/imu/data"
                    ) >= 1
                    for node_id in NODE_IDS
                )
                and all(
                    self.node.count_publishers(
                        f"/vehicle_{node_id}/lidar/odometry"
                    ) >= 1
                    and self.node.count_publishers(
                        f"/vehicle_{node_id}/visual/odometry"
                    ) >= 1
                    for node_id in NODE_IDS
                )
                and self.node.count_publishers("/uwb/range") >= 1
                and self.node.count_publishers(NODE_STATE_TOPIC) >= 3
                and self.node.count_publishers(POSE_TOPIC) >= 1
            ):
                break
        else:
            self.fail(
                "enabled perception and localization graph was incomplete"
            )

        collect_deadline = time.monotonic() + 35.0
        while time.monotonic() < collect_deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            complete_poses = [
                message for message in self.poses
                if self._complete_pose(message)
            ]
            if (
                all(len(self.clouds[node_id]) >= 12 for node_id in NODE_IDS)
                and all(
                    len(self.images[node_id]) >= 24
                    for node_id in NODE_IDS
                )
                and all(
                    len(self.camera_infos[node_id]) >= 24
                    for node_id in NODE_IDS
                )
                and all(len(self.imus[node_id]) >= 150 for node_id in NODE_IDS)
                and all(
                    len(self.lidar_odometry[node_id]) >= 12
                    and len(self.visual_odometry[node_id]) >= 5
                    and len(self.gazebo_poses[node_id]) >= 30
                    for node_id in NODE_IDS
                )
                and all(
                    len(self.node_states[node_id]) >= 20
                    for node_id in NODE_IDS
                )
                and len(self.ranges) >= 90
                and len(complete_poses) >= 15
            ):
                break

        complete_poses = [
            message for message in self.poses if self._complete_pose(message)
        ]
        self.assertGreaterEqual(len(complete_poses), 15)
        pose_stamps = [self._stamp_ns(message) for message in complete_poses]
        self.assertTrue(all(stamp > 0 for stamp in pose_stamps))
        self.assertTrue(all(
            left < right for left, right in zip(pose_stamps, pose_stamps[1:])
        ))

        for node_id in NODE_IDS:
            clouds = self.clouds[node_id]
            images = self.images[node_id]
            camera_infos = self.camera_infos[node_id]
            imus = self.imus[node_id]
            states = self.node_states[node_id]
            self.assertGreaterEqual(len(clouds), 12)
            self.assertGreaterEqual(len(images), 24)
            self.assertGreaterEqual(len(camera_infos), 24)
            self.assertGreaterEqual(len(imus), 150)
            self.assertGreaterEqual(len(states), 20)
            self.assertGreaterEqual(len(self.lidar_odometry[node_id]), 12)
            self.assertGreaterEqual(len(self.visual_odometry[node_id]), 5)
            self.assertGreaterEqual(len(self.gazebo_poses[node_id]), 30)

            self._assert_stamps_and_rate(clouds, 3.0, 7.0)
            self._assert_stamps_and_rate(images, 7.0, 13.0)
            self._assert_stamps_and_rate(camera_infos, 7.0, 13.0)

            for cloud in clouds:
                self.assertEqual(
                    cloud.header.frame_id,
                    f"vehicle_{node_id}/base_link/front_lidar",
                )
                self.assertEqual(cloud.width, 360)
                self.assertEqual(cloud.height, 1)
                self.assertTrue({"x", "y", "z"}.issubset(
                    {field.name for field in cloud.fields}
                ))
                self.assertGreater(cloud.point_step, 0)
                self.assertGreaterEqual(
                    cloud.row_step, cloud.width * cloud.point_step
                )
                self.assertGreaterEqual(
                    len(cloud.data), cloud.height * cloud.row_step
                )
            self.assertTrue(any(
                self._cloud_has_usable_geometry(cloud) for cloud in clouds
            ))

            info_stamps = [self._stamp_ns(info) for info in camera_infos]
            for image in images:
                self.assertEqual(
                    image.header.frame_id,
                    f"vehicle_{node_id}/base_link/front_camera",
                )
                self.assertEqual(image.width, 320)
                self.assertEqual(image.height, 240)
                self.assertNotEqual(image.encoding, "")
                self.assertGreaterEqual(image.step, image.width)
                self.assertGreaterEqual(
                    len(image.data), image.height * image.step
                )
                self.assertLessEqual(
                    min(abs(self._stamp_ns(image) - stamp)
                        for stamp in info_stamps),
                    100_000_000,
                )
            self.assertTrue(any(
                image.data and min(image.data) < max(image.data)
                for image in images
            ))
            for info in camera_infos:
                self.assertEqual(
                    info.header.frame_id,
                    f"vehicle_{node_id}/base_link/front_camera",
                )
                self.assertEqual(info.width, 320)
                self.assertEqual(info.height, 240)
                self.assertTrue(math.isfinite(info.k[0]))
                self.assertTrue(math.isfinite(info.k[4]))
                self.assertGreater(info.k[0], 0.0)
                self.assertGreater(info.k[4], 0.0)

            imu_stamps = [self._stamp_ns(message) for message in imus]
            self.assertTrue(all(stamp > 0 for stamp in imu_stamps))
            self.assertTrue(all(
                left < right
                for left, right in zip(imu_stamps, imu_stamps[1:])
            ))
            self.assertTrue(all(
                message.header.frame_id == "imu_link" for message in imus
            ))
            state_stamps = [self._stamp_ns(message) for message in states]
            self.assertTrue(all(stamp > 0 for stamp in state_stamps))
            self.assertTrue(all(
                left < right
                for left, right in zip(state_stamps, state_stamps[1:])
            ))
            self.assertTrue(all(
                message.header.frame_id == "common_enu" for message in states
            ))

            visual_errors = []
            visual_yaw_errors = []
            for estimate in self.visual_odometry[node_id]:
                estimate_stamp = self._stamp_ns(estimate)
                truth = self._nearest_transform(
                    self.gazebo_poses[node_id], estimate_stamp
                )
                self.assertLessEqual(
                    abs(self._stamp_ns(truth) - estimate_stamp),
                    100_000_000,
                )
                self.assertEqual(estimate.header.frame_id, "common_enu")
                self.assertEqual(
                    estimate.child_frame_id,
                    f"vehicle_{node_id}/base_link",
                )
                visual_errors.append(math.hypot(
                    estimate.pose.pose.position.x
                    - truth.transform.translation.x,
                    estimate.pose.pose.position.y
                    - truth.transform.translation.y,
                ))
                visual_yaw_errors.append(self._angle_error(
                    self._yaw(estimate.pose.pose.orientation),
                    self._yaw(truth.transform.rotation),
                ))
            self.assertLess(
                max(visual_errors), 0.35,
                (node_id, max(visual_errors)),
            )
            self.assertLess(
                max(visual_yaw_errors), 0.10,
                (node_id, max(visual_yaw_errors)),
            )

            lidar = self.lidar_odometry[node_id]
            lidar_stamps = [self._stamp_ns(message) for message in lidar]
            self.assertTrue(all(stamp > 0 for stamp in lidar_stamps))
            self.assertTrue(all(
                left < right
                for left, right in zip(lidar_stamps, lidar_stamps[1:])
            ))
            self.assertTrue(all(
                message.header.frame_id == f"vehicle_{node_id}/lidar_odom"
                and message.child_frame_id
                == f"vehicle_{node_id}/base_link/front_lidar"
                for message in lidar
            ))
            first_lidar = lidar[0]
            last_lidar = lidar[-1]
            first_truth = self._nearest_transform(
                self.gazebo_poses[node_id], self._stamp_ns(first_lidar)
            )
            last_truth = self._nearest_transform(
                self.gazebo_poses[node_id], self._stamp_ns(last_lidar)
            )
            self.assertLessEqual(
                abs(self._stamp_ns(first_truth) - self._stamp_ns(first_lidar)),
                100_000_000,
            )
            self.assertLessEqual(
                abs(self._stamp_ns(last_truth) - self._stamp_ns(last_lidar)),
                100_000_000,
            )
            first_truth_yaw = self._yaw(first_truth.transform.rotation)
            last_truth_yaw = self._yaw(last_truth.transform.rotation)
            first_sensor_x = (
                first_truth.transform.translation.x
                + 0.20 * math.cos(first_truth_yaw)
            )
            first_sensor_y = (
                first_truth.transform.translation.y
                + 0.20 * math.sin(first_truth_yaw)
            )
            last_sensor_x = (
                last_truth.transform.translation.x
                + 0.20 * math.cos(last_truth_yaw)
            )
            last_sensor_y = (
                last_truth.transform.translation.y
                + 0.20 * math.sin(last_truth_yaw)
            )
            world_dx = last_sensor_x - first_sensor_x
            world_dy = last_sensor_y - first_sensor_y
            truth_dx = (
                math.cos(first_truth_yaw) * world_dx
                + math.sin(first_truth_yaw) * world_dy
            )
            truth_dy = (
                -math.sin(first_truth_yaw) * world_dx
                + math.cos(first_truth_yaw) * world_dy
            )

            first_odom_yaw = self._yaw(
                first_lidar.pose.pose.orientation
            )
            odom_world_dx = (
                last_lidar.pose.pose.position.x
                - first_lidar.pose.pose.position.x
            )
            odom_world_dy = (
                last_lidar.pose.pose.position.y
                - first_lidar.pose.pose.position.y
            )
            odom_dx = (
                math.cos(first_odom_yaw) * odom_world_dx
                + math.sin(first_odom_yaw) * odom_world_dy
            )
            odom_dy = (
                -math.sin(first_odom_yaw) * odom_world_dx
                + math.cos(first_odom_yaw) * odom_world_dy
            )
            self.assertGreater(math.hypot(truth_dx, truth_dy), 0.20)
            self.assertGreater(math.hypot(odom_dx, odom_dy), 0.10)
            self.assertLess(
                math.hypot(odom_dx - truth_dx, odom_dy - truth_dy),
                0.65,
            )
            self.assertLess(
                self._angle_error(
                    self._yaw(last_lidar.pose.pose.orientation)
                    - first_odom_yaw,
                    last_truth_yaw - first_truth_yaw,
                ),
                0.25,
            )

        ranges_by_edge = {}
        for message in self.ranges:
            edge = tuple(sorted((message.src_id, message.target_id)))
            self.assertIn(edge, {(1, 2), (1, 3), (2, 3)})
            self.assertGreater(self._stamp_ns(message), 0)
            self.assertTrue(math.isfinite(message.distance))
            self.assertGreater(message.distance, 0.0)
            ranges_by_edge.setdefault(edge, []).append(
                self._stamp_ns(message)
            )
        self.assertEqual(set(ranges_by_edge), {(1, 2), (1, 3), (2, 3)})
        for stamps in ranges_by_edge.values():
            self.assertGreaterEqual(len(stamps), 15)
            self.assertTrue(all(
                left < right for left, right in zip(stamps, stamps[1:])
            ))

        # Re-check the graph after collection so an early frontend exit cannot
        # pass merely because its publisher was visible during discovery.
        for node_id in NODE_IDS:
            self.assertEqual(self.node.count_publishers(
                f"/vehicle_{node_id}/lidar/odometry"
            ), 1)
            self.assertEqual(self.node.count_publishers(
                f"/vehicle_{node_id}/visual/odometry"
            ), 1)

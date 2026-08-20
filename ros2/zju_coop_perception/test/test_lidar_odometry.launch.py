"""Contract test for the optional planar lidar odometry frontend."""

import math
import struct
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


CLOUD_TOPIC = "/test/lidar/points"
ODOMETRY_TOPIC = "/test/lidar/odometry"
LIDAR_FRAME = "test_lidar"
ODOMETRY_FRAME = "test_lidar_odom"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    frontend = launch_ros.actions.Node(
        package="zju_coop_perception",
        executable="zju_lidar_odometry_node",
        name="zju_lidar_odometry_test",
        output="screen",
        parameters=[{
            "odom_frame_id": ODOMETRY_FRAME,
            "min_range_m": 0.1,
            "max_range_m": 20.0,
            "minimum_points": 20,
            "minimum_geometry_variance_m2": 0.01,
            "minimum_geometry_ratio": 0.02,
            "max_correspondence_distance_m": 0.5,
            "maximum_iterations": 60,
            "max_fitness_score_m2": 0.01,
            "max_translation_per_scan_m": 0.5,
            "max_yaw_per_scan_rad": 0.3,
        }],
        remappings=[
            ("point_cloud", CLOUD_TOPIC),
            ("lidar_odometry", ODOMETRY_TOPIC),
        ],
    )
    return (
        launch.LaunchDescription([
            frontend,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"frontend": frontend},
    )


class TestLidarOdometry(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("lidar_odometry_test_driver")
        self.cloud_publisher = self.node.create_publisher(
            PointCloud2, CLOUD_TOPIC, qos_profile_sensor_data
        )
        self.odometry = []
        self.odometry_subscription = self.node.create_subscription(
            Odometry,
            ODOMETRY_TOPIC,
            self.odometry.append,
            10,
        )

    def tearDown(self):
        self.node.destroy_publisher(self.cloud_publisher)
        self.node.destroy_subscription(self.odometry_subscription)
        self.node.destroy_node()

    def _wait_for_graph(self, timeout_s=8.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.cloud_publisher.get_subscription_count() == 1
                and self.node.count_publishers(ODOMETRY_TOPIC) == 1
            ):
                return
        self.fail("lidar odometry ROS graph was not ready")

    @staticmethod
    def _cloud(stamp_ns, points):
        message = PointCloud2()
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        message.header.frame_id = LIDAR_FRAME
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name="x", offset=0,
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name="y", offset=4,
                datatype=PointField.FLOAT32, count=1,
            ),
            PointField(
                name="z", offset=8,
                datatype=PointField.FLOAT32, count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = b"".join(
            struct.pack("<fff", float(x), float(y), float(z))
            for x, y, z in points
        )
        message.is_dense = all(
            math.isfinite(value)
            for point in points
            for value in point
        )
        return message

    @staticmethod
    def _static_scene():
        points = []
        for index in range(51):
            points.append((
                -2.0 + 0.08 * index,
                2.0 + 0.10 * math.sin(0.37 * index),
                0.0,
            ))
        for index in range(37):
            points.append((
                -1.45 + 0.08 * math.sin(0.51 * index),
                -1.0 + 0.09 * index,
                0.0,
            ))
        for index in range(29):
            points.append((
                0.55 + 0.045 * index,
                -1.65 + 0.0022 * index * index,
                0.0,
            ))
        return points

    @staticmethod
    def _points_in_moved_sensor_frame(points, x_m, y_m, yaw_rad):
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        output = []
        for x, y, z in points:
            translated_x = x - x_m
            translated_y = y - y_m
            output.append((
                cos_yaw * translated_x + sin_yaw * translated_y,
                -sin_yaw * translated_x + cos_yaw * translated_y,
                z,
            ))
        return output

    def _spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _spin_until_odometry(self, expected, timeout_s=3.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and len(self.odometry) < expected:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    @staticmethod
    def _yaw(message):
        orientation = message.pose.pose.orientation
        return math.atan2(
            2.0 * orientation.w * orientation.z,
            1.0 - 2.0 * orientation.z * orientation.z,
        )

    def test_planar_motion_and_rejection_contract(self):
        self._wait_for_graph()

        zero_points = [(0.0, 0.0, 0.0)] * 64
        self.cloud_publisher.publish(self._cloud(1_000_000_000, zero_points))
        self._spin_for(0.3)
        self.assertEqual(self.odometry, [])

        line_points = [
            (-2.0 + 0.08 * index, 1.0, 0.0)
            for index in range(51)
        ]
        self.cloud_publisher.publish(self._cloud(2_000_000_000, line_points))
        self._spin_for(0.3)
        self.assertEqual(self.odometry, [])

        scene = self._static_scene()
        self.cloud_publisher.publish(self._cloud(3_000_000_000, scene))
        self._spin_until_odometry(1)
        self.assertEqual(len(self.odometry), 1)
        origin = self.odometry[-1]
        self.assertEqual(origin.header.frame_id, ODOMETRY_FRAME)
        self.assertEqual(origin.child_frame_id, LIDAR_FRAME)
        self.assertAlmostEqual(origin.pose.pose.position.x, 0.0, places=9)
        self.assertAlmostEqual(origin.pose.pose.position.y, 0.0, places=9)
        self.assertAlmostEqual(self._yaw(origin), 0.0, places=9)

        expected_x_m = 0.08
        expected_y_m = -0.04
        expected_yaw_rad = 0.04
        moved_scene = self._points_in_moved_sensor_frame(
            scene, expected_x_m, expected_y_m, expected_yaw_rad
        )
        self.cloud_publisher.publish(
            self._cloud(3_200_000_000, moved_scene)
        )
        self._spin_until_odometry(2)
        self.assertEqual(len(self.odometry), 2)
        estimate = self.odometry[-1]
        self.assertEqual(estimate.header.stamp.sec, 3)
        self.assertEqual(estimate.header.stamp.nanosec, 200_000_000)
        self.assertAlmostEqual(
            estimate.pose.pose.position.x, expected_x_m, delta=0.02
        )
        self.assertAlmostEqual(
            estimate.pose.pose.position.y, expected_y_m, delta=0.02
        )
        self.assertAlmostEqual(
            self._yaw(estimate), expected_yaw_rad, delta=0.01
        )
        self.assertGreater(estimate.pose.covariance[14], 1.0e5)
        self.assertGreater(estimate.pose.covariance[21], 1.0e5)
        self.assertGreater(estimate.pose.covariance[28], 1.0e5)

        accepted_count = len(self.odometry)
        self.cloud_publisher.publish(
            self._cloud(3_400_000_000, zero_points)
        )
        self.cloud_publisher.publish(
            self._cloud(3_600_000_000, line_points)
        )
        self._spin_for(0.5)
        self.assertEqual(len(self.odometry), accepted_count)


@launch_testing.post_shutdown_test()
class TestLidarOdometryShutdown(unittest.TestCase):

    def test_frontend_exits_cleanly(self, proc_info, frontend):
        launch_testing.asserts.assertExitCodes(proc_info, process=frontend)

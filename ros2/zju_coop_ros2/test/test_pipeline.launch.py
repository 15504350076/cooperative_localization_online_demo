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
from builtin_interfaces.msg import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState
from cooperative_interfaces.msg import UwbRange


POSE_TOPIC = "/cooperative_localization/poses_2d"
FEEDBACK_TOPIC = "/cooperative_localization/feedback/poses_2d"
NODE_STATE_TOPIC = "/cooperative_localization/node_state"
UWB_TOPIC = "/uwb/range"
NODE_IDS = (1, 2, 3)
# Deliberately not ROS SystemTime: production sensor headers use
# UWB_SYSTEM_TIME (here represented as elapsed time since the UWB epoch).
UWB_START_NS = 10_000_000_000
LONG_CAMERA_FRAME_ID = "vehicle_2/base_link/front_camera_optical_frame"
LONG_POINT_CLOUD_FRAME_ID = "vehicle_2/base_link/front_lidar_sensor_frame"
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


def _local_node(
    node_id, initial_position, initial_orientation, enable_raw_input=False
):
    parameters = {
        "node_id": node_id,
        "publish_rate_hz": 10.0,
        "expected_imu_frame_id": "imu_link",
        "initial_position_enu_m": list(initial_position),
        "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
        "initial_orientation_flu_to_enu_xyzw": list(initial_orientation),
        "initial_gyro_bias_flu_rad_s": [0.0, 0.0, 0.0],
        "initial_accel_bias_flu_m_s2": [0.0, 0.0, 0.0],
    }
    if enable_raw_input:
        parameters.update({
            "enable_camera_input": True,
            "enable_point_cloud_input": True,
            "camera_id": 1,
            "point_cloud_sensor_id": 1,
            "camera_frame_alias": "camera_front",
            "point_cloud_frame_alias": "lidar_link",
        })

    return launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_local_inertial_node_{node_id}",
        output="screen",
        parameters=[parameters],
        remappings=[
            ("imu", f"/vehicle_{node_id}/imu/data"),
            ("node_state", NODE_STATE_TOPIC),
            ("camera_image", f"/vehicle_{node_id}/camera/image_raw"),
            ("point_cloud", f"/vehicle_{node_id}/lidar/points"),
        ],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    local_nodes = [
        _local_node(
            node_id,
            position,
            orientation,
            enable_raw_input=(node_id == 2),
        )
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
            "enable_follower_feedback": True,
            "use_sim_time": False,
        }],
        remappings=[
            ("node_state", NODE_STATE_TOPIC),
            ("uwb_range", UWB_TOPIC),
            ("poses_2d", POSE_TOPIC),
            ("feedback_poses_2d", FEEDBACK_TOPIC),
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
        self.node_state_publisher = self.node.create_publisher(
            NodeState, NODE_STATE_TOPIC, qos_profile_sensor_data
        )
        self.camera_publishers = {
            node_id: self.node.create_publisher(
                Image,
                f"/vehicle_{node_id}/camera/image_raw",
                qos_profile_sensor_data,
            )
            for node_id in NODE_IDS
        }
        self.point_cloud_publishers = {
            node_id: self.node.create_publisher(
                PointCloud2,
                f"/vehicle_{node_id}/lidar/points",
                qos_profile_sensor_data,
            )
            for node_id in NODE_IDS
        }
        self.pose_messages = []
        self.pose_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            POSE_TOPIC,
            self.pose_messages.append,
            qos_profile_sensor_data,
        )
        self.feedback_messages = []
        self.feedback_subscription = self.node.create_subscription(
            CooperativePose2DArray,
            FEEDBACK_TOPIC,
            self.feedback_messages.append,
            qos_profile_sensor_data,
        )

    def tearDown(self):
        for publisher in self.imu_publishers:
            self.node.destroy_publisher(publisher)
        self.node.destroy_publisher(self.uwb_publisher)
        self.node.destroy_publisher(self.node_state_publisher)
        for publisher in self.camera_publishers.values():
            self.node.destroy_publisher(publisher)
        for publisher in self.point_cloud_publishers.values():
            self.node.destroy_publisher(publisher)
        self.node.destroy_subscription(self.pose_subscription)
        self.node.destroy_subscription(self.feedback_subscription)
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
            node_state_ready = (
                self.node_state_publisher.get_subscription_count() >= 1
            )
            pose_ready = self.node.count_publishers(POSE_TOPIC) >= 1
            feedback_ready = self.node.count_publishers(FEEDBACK_TOPIC) == 1
            raw_input_ready = (
                self.camera_publishers[2].get_subscription_count() >= 1
                and self.point_cloud_publishers[2].get_subscription_count()
                >= 1
            )
            if (
                imu_ready
                and uwb_ready
                and node_state_ready
                and pose_ready
                and feedback_ready
                and raw_input_ready
            ):
                return
        self.fail(
            "ROS graph did not expose all IMU, UWB, raw-input and Pose2D "
            "endpoints"
        )

    def _assert_optional_raw_input_graph(self):
        for node_id in (1, 3):
            self.assertEqual(
                self.camera_publishers[node_id].get_subscription_count(),
                0,
                f"vehicle {node_id} created a default-disabled camera "
                "subscription",
            )
            self.assertEqual(
                self.point_cloud_publishers[node_id].get_subscription_count(),
                0,
                f"vehicle {node_id} created a default-disabled point-cloud "
                "subscription",
            )

    @staticmethod
    def _camera_image(stamp):
        message = Image()
        message.header.stamp.sec = stamp.sec
        message.header.stamp.nanosec = stamp.nanosec
        message.header.frame_id = LONG_CAMERA_FRAME_ID
        message.height = 2
        message.width = 2
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = 6
        message.data = bytes(12)
        return message

    @staticmethod
    def _point_cloud(stamp):
        message = PointCloud2()
        message.header.stamp.sec = stamp.sec
        message.header.stamp.nanosec = stamp.nanosec
        message.header.frame_id = LONG_POINT_CLOUD_FRAME_ID
        message.height = 1
        message.width = 2
        message.fields = [
            PointField(
                name=name,
                offset=offset,
                datatype=PointField.FLOAT32,
                count=1,
            )
            for name, offset in (("x", 0), ("y", 4), ("z", 8))
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 24
        message.data = bytes(24)
        message.is_dense = True
        return message

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

    @staticmethod
    def _uwb_stamp(elapsed_s):
        timestamp_ns = UWB_START_NS + int(elapsed_s * 1_000_000_000)
        return Time(
            sec=timestamp_ns // 1_000_000_000,
            nanosec=timestamp_ns % 1_000_000_000,
        )

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

    def _publish_reference_state(self, stamp, valid):
        message = NodeState()
        message.header.stamp = stamp
        message.header.frame_id = "common_enu"
        message.node_id = 1
        message.orientation_flu_to_enu_xyzw[3] = 1.0
        message.valid = valid
        self.node_state_publisher.publish(message)

    @staticmethod
    def _complete_pose(message):
        return (
            message.reference_node_id == 1
            and len(message.vehicles) == 3
            and {vehicle.node_id for vehicle in message.vehicles}
            == set(NODE_IDS)
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

    def test_standard_imu_and_uwb_produce_gcs_pose_array(
        self, proc_output, local_nodes, fusion_node
    ):
        self._wait_for_graph()
        self._assert_optional_raw_input_graph()
        self.assertGreater(len(LONG_CAMERA_FRAME_ID), 31)
        self.assertGreater(len(LONG_POINT_CLOUD_FRAME_ID), 31)

        raw_stamp = self.node.get_clock().now().to_msg()
        for node_id in (1, 3):
            self.camera_publishers[node_id].publish(
                self._camera_image(raw_stamp)
            )
            self.point_cloud_publishers[node_id].publish(
                self._point_cloud(raw_stamp)
            )
        self.camera_publishers[2].publish(self._camera_image(raw_stamp))
        self.point_cloud_publishers[2].publish(self._point_cloud(raw_stamp))
        proc_output.assertWaitFor(
            "camera input: VALIDATED_NOT_USED",
            process=local_nodes[1],
            timeout=5,
        )
        proc_output.assertWaitFor(
            "point cloud input: VALIDATED_NOT_USED",
            process=local_nodes[1],
            timeout=5,
        )

        # An invalid reference state must not establish the UWB clock anchor.
        self._publish_reference_state(self._uwb_stamp(100.0), False)
        time.sleep(0.05)

        accepted = None
        deadline = time.monotonic() + 15.0
        start_wall = time.monotonic()
        while time.monotonic() < deadline and accepted is None:
            stamp = self._uwb_stamp(time.monotonic() - start_wall)
            for publisher in self.imu_publishers:
                publisher.publish(self._imu(stamp))
            # Keep consecutive IMU samples safely inside the SDK's 0.1 s
            # continuity limit while still letting callbacks and timers run.
            time.sleep(0.02)
            self._publish_ranges(stamp)

            rclpy.spin_once(self.node, timeout_sec=0.02)
            accepted = next(
                (message for message in reversed(self.pose_messages)
                 if self._uwb_corrected_pose(message)),
                None,
            )

        self.assertIsNotNone(
            accepted, "no complete valid three-vehicle pose received"
        )
        self.assertEqual(accepted.header.frame_id, "coop_ref_1_enu")
        self.assertNotEqual(accepted.header.stamp.sec, 0)

        feedback_deadline = time.monotonic() + 2.0
        matching_feedback = None
        while time.monotonic() < feedback_deadline:
            matching_feedback = next(
                (
                    message
                    for message in reversed(self.feedback_messages)
                    if message.header.stamp == accepted.header.stamp
                ),
                None,
            )
            if matching_feedback is not None:
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIsNotNone(matching_feedback)
        self.assertEqual(
            serialize_message(matching_feedback),
            serialize_message(accepted),
        )

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

        self._publish_reference_state(self._uwb_stamp(100.0), True)
        proc_output.assertWaitFor(
            "reference NodeState UWB time discontinuity",
            process=fusion_node,
            timeout=5,
        )

        self._publish_reference_state(self._uwb_stamp(-0.001), True)
        proc_output.assertWaitFor(
            "reference NodeState UWB time rolled back",
            process=fusion_node,
            timeout=5,
        )


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

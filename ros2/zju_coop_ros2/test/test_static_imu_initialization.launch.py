"""Verify startup static alignment and frozen gyro-bias initialization online."""

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

from cooperative_localization_msgs.msg import NodeState


IMU_TOPIC = "/static_initialization/imu"
STATE_TOPIC = "/static_initialization/node_state"
YAW_RAD = math.radians(40.0)
GYRO_BIAS = (0.02, -0.01, 0.03)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    local_node = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name="zju_static_imu_initialization_test",
        output="screen",
        parameters=[{
            "node_id": 1,
            "publish_rate_hz": 100.0,
            "expected_imu_frame_id": "imu_link",
            "common_enu_frame_id": "common_enu",
            "initial_position_enu_m": [0.0, 0.0, 0.0],
            "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
            "initial_orientation_flu_to_enu_xyzw": [
                0.0, 0.0, math.sin(YAW_RAD / 2.0), math.cos(YAW_RAD / 2.0)
            ],
            "initial_gyro_bias_flu_rad_s": [0.0, 0.0, 0.0],
            "initial_accel_bias_flu_m_s2": [0.0, 0.0, 0.0],
            "enable_static_imu_initialization": True,
            "static_imu_initialization_duration_s": 0.1,
            "static_imu_minimum_sample_count": 10,
        }],
        remappings=[("imu", IMU_TOPIC), ("node_state", STATE_TOPIC)],
    )
    return (
        launch.LaunchDescription([
            local_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"local_node": local_node},
    )


class TestStaticImuInitialization(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("static_imu_initialization_test_driver")
        self.publisher = self.node.create_publisher(
            Imu, IMU_TOPIC, qos_profile_sensor_data
        )
        self.states = []
        self.subscription = self.node.create_subscription(
            NodeState, STATE_TOPIC, self.states.append, qos_profile_sensor_data
        )

    def tearDown(self):
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()

    @staticmethod
    def _imu(timestamp_ns):
        message = Imu()
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        message.header.frame_id = "imu_link"
        message.orientation_covariance[0] = -1.0
        message.angular_velocity.x = GYRO_BIAS[0]
        message.angular_velocity.y = GYRO_BIAS[1]
        message.angular_velocity.z = GYRO_BIAS[2]
        message.linear_acceleration.z = 9.80665
        return message

    def test_static_window_initializes_and_removes_constant_rate(
        self, proc_output, local_node
    ):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() >= 1:
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        else:
            self.fail("local inertial IMU subscription was not discovered")

        start_ns = self.node.get_clock().now().nanoseconds
        for index in range(80):
            self.publisher.publish(self._imu(start_ns + index * 10_000_000))
            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.01)

        proc_output.assertWaitFor(
            "static IMU initialization ready", process=local_node, timeout=5
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not self.states:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(self.states, "no NodeState after static initialization")

        state = self.states[-1]
        x, y, z, w = state.orientation_flu_to_enu_xyzw
        yaw = math.atan2(2.0 * (w * z + x * y),
                         1.0 - 2.0 * (y * y + z * z))
        self.assertAlmostEqual(yaw, YAW_RAD, delta=1.0e-3)
        for value in (*state.position_enu_m, *state.velocity_enu_mps):
            self.assertAlmostEqual(value, 0.0, delta=1.0e-6)
        self.assertTrue(state.valid)


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

"""Regression test for the local INS latched IMU time fault."""

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


IMU_TOPIC = "/imu_time_fail_closed/imu"
NODE_STATE_TOPIC = "/imu_time_fail_closed/node_state"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    local_node = launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name="zju_imu_time_fail_closed_test",
        output="screen",
        parameters=[{
            "node_id": 1,
            "publish_rate_hz": 100.0,
            "expected_imu_frame_id": "imu_link",
            "common_enu_frame_id": "common_enu",
            "initial_position_enu_m": [0.0, 0.0, 0.0],
            "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
            "initial_orientation_flu_to_enu_xyzw": [0.0, 0.0, 0.0, 1.0],
            "initial_gyro_bias_flu_rad_s": [0.0, 0.0, 0.0],
            "initial_accel_bias_flu_m_s2": [0.0, 0.0, 0.0],
        }],
        remappings=[
            ("imu", IMU_TOPIC),
            ("node_state", NODE_STATE_TOPIC),
        ],
    )
    return (
        launch.LaunchDescription([
            local_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"local_node": local_node},
    )


class TestImuTimeFailClosed(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("imu_time_fail_closed_test_driver")
        self.publisher = self.node.create_publisher(
            Imu, IMU_TOPIC, qos_profile_sensor_data
        )
        self.states = []
        self.subscription = self.node.create_subscription(
            NodeState,
            NODE_STATE_TOPIC,
            self.states.append,
            qos_profile_sensor_data,
        )

    def tearDown(self):
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()

    @staticmethod
    def _stamp_ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _imu(timestamp_ns):
        message = Imu()
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        message.header.frame_id = "imu_link"
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1.0
        message.linear_acceleration.z = 9.80665
        return message

    def _spin_until_stamp(self, expected_ns, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if any(
                self._stamp_ns(state.header.stamp) == expected_ns
                for state in self.states
            ):
                return
        self.fail(f"NodeState stamp {expected_ns} was not published")

    def test_rollback_latches_and_forward_samples_cannot_recover(
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
        stamps = [start_ns + index * 10_000_000 for index in range(6)]
        for stamp in stamps:
            self.publisher.publish(self._imu(stamp))
            time.sleep(0.01)
        self._spin_until_stamp(stamps[-1])

        # An exact duplicate is discarded but must not latch the fault.
        self.publisher.publish(self._imu(stamps[-1]))
        next_stamp = stamps[-1] + 10_000_000
        time.sleep(0.01)
        self.publisher.publish(self._imu(next_stamp))
        self._spin_until_stamp(next_stamp)
        state_count_before_fault = len(self.states)

        self.publisher.publish(self._imu(next_stamp - 5_000_000))
        proc_output.assertWaitFor(
            "IMU time fault latched (timestamp rollback)",
            process=local_node,
            timeout=5,
        )

        for index in range(1, 11):
            self.publisher.publish(
                self._imu(next_stamp + index * 10_000_000)
            )
            time.sleep(0.01)
            rclpy.spin_once(self.node, timeout_sec=0.0)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

        self.assertEqual(len(self.states), state_count_before_fault)
        self.assertTrue(all(state.valid for state in self.states))


@launch_testing.post_shutdown_test()
class TestProcessesExitCleanly(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

"""Verify that vehicle bringup defaults to UWB and explicitly switches to GNSS."""

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
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from zju_coop_test_msgs.msg import UwbRange


GNSS_TOPICS = tuple(f"/test/vehicle_{node_id}/gnss/fix" for node_id in (1, 2, 3))
DERIVED_RANGE_TOPIC = "/cooperative_localization/gnss_derived_range"


def _vehicle_launch(namespace, fallback=None):
    launch_file = PathJoinSubstitution([
        FindPackageShare("zju_coop_bringup"),
        "launch",
        "vehicle.launch.py",
    ])
    arguments = {
        "namespace": namespace,
        "run_fusion": "true",
    }
    if fallback is not None:
        arguments.update({
            "use_gnss_range_fallback": "true" if fallback else "false",
            "gnss_topic_1": GNSS_TOPICS[0],
            "gnss_topic_2": GNSS_TOPICS[1],
            "gnss_topic_3": GNSS_TOPICS[2],
        })
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=arguments.items(),
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch.LaunchDescription([
        # Omit the fallback argument to lock its declared default of false.
        _vehicle_launch("normal_ref"),
        _vehicle_launch("fallback_ref", fallback=True),
        launch_testing.actions.ReadyToTest(),
    ])


class TestGnssFallbackModes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("gnss_fallback_modes_test_driver")
        self.uwb_publisher = self.node.create_publisher(
            UwbRange, "/uwb/range", qos_profile_sensor_data
        )
        self.gnss_publishers = [
            self.node.create_publisher(NavSatFix, topic, qos_profile_sensor_data)
            for topic in GNSS_TOPICS
        ]

    def tearDown(self):
        self.node.destroy_publisher(self.uwb_publisher)
        for publisher in self.gnss_publishers:
            self.node.destroy_publisher(publisher)
        self.node.destroy_node()

    def _wait_for_graph(self, timeout_s=10.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.uwb_publisher.get_subscription_count() == 1
                and all(publisher.get_subscription_count() == 1
                        for publisher in self.gnss_publishers)
                and self.node.count_publishers(DERIVED_RANGE_TOPIC) == 1
                and self.node.count_subscribers(DERIVED_RANGE_TOPIC) == 1
                and any(
                    name == "zju_gnss_range_fallback_node"
                    and namespace == "/fallback_ref"
                    for name, namespace
                    in self.node.get_node_names_and_namespaces()
                )
            ):
                return
        self.fail("normal and GNSS-fallback bringup graphs were not ready")

    def _range_std_m(self, node_name):
        client = self.node.create_client(
            GetParameters, f"{node_name}/get_parameters"
        )
        self.assertTrue(client.wait_for_service(timeout_sec=5.0), node_name)
        request = GetParameters.Request()
        request.names = ["range_std_m"]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done(), node_name)
        value = future.result().values[0].double_value
        self.node.destroy_client(client)
        return value

    def test_modes_are_mutually_exclusive_and_use_distinct_uncertainty(self):
        self._wait_for_graph()
        fallback_nodes = [
            namespace
            for name, namespace in self.node.get_node_names_and_namespaces()
            if name == "zju_gnss_range_fallback_node"
        ]
        self.assertEqual(fallback_nodes, ["/fallback_ref"])
        self.assertAlmostEqual(
            self._range_std_m("/normal_ref/zju_cooperative_fusion_node"),
            0.1,
        )
        self.assertAlmostEqual(
            self._range_std_m("/fallback_ref/zju_cooperative_fusion_node"),
            3.0,
        )

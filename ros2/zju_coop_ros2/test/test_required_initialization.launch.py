"""Verify that all required local INS initial values are fail-fast parameters."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest


REQUIRED_PARAMETERS = (
    "initial_position_enu_m",
    "initial_velocity_enu_mps",
    "initial_orientation_flu_to_enu_xyzw",
)


def _invalid_local_node(index, missing_parameter):
    parameters = {
        "node_id": index + 1,
        "initial_position_enu_m": [0.0, 0.0, 0.0],
        "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
        "initial_orientation_flu_to_enu_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    del parameters[missing_parameter]
    return launch_ros.actions.Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_local_inertial_node_missing_{index}",
        output="screen",
        parameters=[parameters],
    )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    invalid_nodes = [
        _invalid_local_node(index, parameter)
        for index, parameter in enumerate(REQUIRED_PARAMETERS)
    ]
    return (
        launch.LaunchDescription([
            *invalid_nodes,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"invalid_nodes": invalid_nodes},
    )


class TestRequiredInitialization(unittest.TestCase):

    def test_missing_parameter_exits_with_failure(self, proc_info, invalid_nodes):
        for invalid_node in invalid_nodes:
            proc_info.assertWaitForShutdown(process=invalid_node, timeout=10)
            launch_testing.asserts.assertExitCodes(
                proc_info,
                process=invalid_node,
                allowable_exit_codes=[1],
            )

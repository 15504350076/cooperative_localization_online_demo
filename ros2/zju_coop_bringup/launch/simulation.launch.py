"""Run the complete three-vehicle IMU/UWB simulation and GCS pipeline."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


NODE_STATE_TOPIC = "/cooperative_localization/node_state"


def _local_node(node_id, position, velocity, yaw):
    return Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_sim_local_inertial_node_{node_id}",
        output="screen",
        parameters=[{
            "node_id": node_id,
            "publish_rate_hz": 20.0,
            "expected_imu_frame_id": "imu_link",
            "common_enu_frame_id": "common_enu",
            "initial_position_enu_m": list(position),
            "initial_velocity_enu_mps": list(velocity),
            "initial_orientation_flu_to_enu_xyzw": [
                0.0,
                0.0,
                math.sin(0.5 * yaw),
                math.cos(0.5 * yaw),
            ],
            "initial_gyro_bias_flu_rad_s": [0.0, 0.0, 0.0],
            "initial_accel_bias_flu_m_s2": [0.0, 0.0, 0.0],
        }],
        remappings=[
            ("imu", f"/vehicle_{node_id}/imu/data"),
            ("node_state", NODE_STATE_TOPIC),
        ],
    )


def generate_launch_description():
    uwb_noise = LaunchConfiguration("uwb_noise_std_m")
    uwb_drop = LaunchConfiguration("uwb_drop_every_n")
    uwb_nlos = LaunchConfiguration("uwb_nlos_every_n")
    uwb_nlos_bias = LaunchConfiguration("uwb_nlos_bias_m")
    gyro_noise = LaunchConfiguration("gyro_noise_std_rad_s")
    accel_noise = LaunchConfiguration("accel_noise_std_m_s2")

    local_nodes = [
        _local_node(1, (0.0, 0.0, 0.0), (0.6, 0.0, 0.0), 0.0),
        # The 0.5 m east error is deliberate: UWB should remove it only in GCS.
        _local_node(2, (4.5, 0.0, 0.0), (0.45, 0.0, 0.0), 0.0),
        _local_node(3, (0.0, 3.0, 0.0), (0.0, 0.3, 0.0), math.pi / 2.0),
    ]
    fusion_node = Node(
        package="zju_coop_ros2",
        executable="zju_cooperative_fusion_node",
        name="zju_sim_cooperative_fusion_node",
        output="screen",
        parameters=[{
            "node_ids": [1, 2, 3],
            "reference_node_id": 1,
            "publish_rate_hz": 10.0,
            "range_std_m": 0.1,
            "node_state_timeout_ms": 500,
            "common_enu_frame_id": "common_enu",
        }],
        remappings=[
            ("node_state", NODE_STATE_TOPIC),
            ("uwb_range", "/uwb/range"),
            ("poses_2d", "/cooperative_localization/poses_2d"),
        ],
    )
    simulator = Node(
        package="zju_coop_ros2",
        executable="zju_three_vehicle_simulator",
        name="zju_three_vehicle_simulator",
        output="screen",
        parameters=[{
            "imu_rate_hz": 100.0,
            "uwb_rate_hz": 20.0,
            "truth_rate_hz": 20.0,
            "uwb_delay_s": 0.05,
            "uwb_noise_std_m": ParameterValue(uwb_noise, value_type=float),
            "uwb_drop_every_n": ParameterValue(uwb_drop, value_type=int),
            "uwb_nlos_every_n": ParameterValue(uwb_nlos, value_type=int),
            "uwb_nlos_bias_m": ParameterValue(uwb_nlos_bias, value_type=float),
            "gyro_noise_std_rad_s": ParameterValue(
                gyro_noise, value_type=float
            ),
            "accel_noise_std_m_s2": ParameterValue(
                accel_noise, value_type=float
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "uwb_noise_std_m",
            default_value="0.0",
            description="UWB single-sample Gaussian noise standard deviation (m)",
        ),
        DeclareLaunchArgument(
            "uwb_drop_every_n",
            default_value="0",
            description="Drop all three UWB edges every N epochs; 0 disables",
        ),
        DeclareLaunchArgument(
            "uwb_nlos_every_n",
            default_value="0",
            description="Bias the 2-3 UWB edge every N epochs; 0 disables",
        ),
        DeclareLaunchArgument(
            "uwb_nlos_bias_m",
            default_value="2.0",
            description="Positive bias added to an enabled NLOS epoch (m)",
        ),
        DeclareLaunchArgument(
            "gyro_noise_std_rad_s",
            default_value="0.0",
            description="IMU gyro single-sample Gaussian standard deviation (rad/s)",
        ),
        DeclareLaunchArgument(
            "accel_noise_std_m_s2",
            default_value="0.0",
            description="IMU accel single-sample Gaussian standard deviation (m/s^2)",
        ),
        *local_nodes,
        fusion_node,
        simulator,
    ])

"""Gazebo Fortress three-vehicle IMU/UWB cooperative localization demo."""

from datetime import datetime
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


NODE_STATE_TOPIC = "/cooperative_localization/node_state"
BAG_TOPICS = (
    "/clock",
    "/vehicle_1/imu/data",
    "/vehicle_2/imu/data",
    "/vehicle_3/imu/data",
    "/uwb/range",
    "/simulation/ground_truth/poses_2d",
    NODE_STATE_TOPIC,
    "/cooperative_localization/poses_2d",
)


def _local_node(node_id, position, yaw):
    return Node(
        package="zju_coop_ros2",
        executable="zju_local_inertial_node",
        name=f"zju_gazebo_local_inertial_node_{node_id}",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "node_id": node_id,
            "publish_rate_hz": 20.0,
            "expected_imu_frame_id": "imu_link",
            "common_enu_frame_id": "common_enu",
            "initial_position_enu_m": list(position),
            "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
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
    package_share = FindPackageShare("zju_coop_gazebo")
    ros_gz_share = FindPackageShare("ros_gz_sim")
    world = PathJoinSubstitution([package_share, "worlds", "three_vehicle.sdf"])
    bridge_config = PathJoinSubstitution([package_share, "config", "bridge.yaml"])
    model_path = PathJoinSubstitution([package_share, "models"])
    headless = LaunchConfiguration("headless")
    bag_output = LaunchConfiguration("bag_output")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(
                    headless,
                    if_value="-s ",
                    else_value="--render-engine-gui ogre ",
                ),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zju_gazebo_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    local_nodes = [
        _local_node(1, (0.0, 0.0, 0.25), 0.0),
        _local_node(
            2,
            (LaunchConfiguration("node_2_initial_east_m"), 0.0, 0.25),
            0.0,
        ),
        _local_node(3, (0.0, 3.0, 0.25), math.pi / 2.0),
    ]
    fusion = Node(
        package="zju_coop_ros2",
        executable="zju_cooperative_fusion_node",
        name="zju_gazebo_cooperative_fusion_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "node_ids": [1, 2, 3],
            "reference_node_id": 1,
            "publish_rate_hz": 10.0,
            "range_std_m": 0.20,
            "node_state_timeout_ms": 1000,
            "common_enu_frame_id": "common_enu",
        }],
        remappings=[
            ("node_state", NODE_STATE_TOPIC),
            ("uwb_range", "/uwb/range"),
            ("poses_2d", "/cooperative_localization/poses_2d"),
        ],
    )
    scenario = Node(
        package="zju_coop_gazebo",
        executable="zju_gazebo_scenario",
        name="zju_gazebo_scenario",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--use-sim-time",
            "-o", bag_output,
            *BAG_TOPICS,
        ],
        condition=IfCondition(LaunchConfiguration("record_bag")),
        output="screen",
    )
    start_simulation = TimerAction(
        period=2.5,
        actions=[ExecuteProcess(
            cmd=[
                "ign", "service",
                "-s", "/world/zju_coop_gazebo/control",
                "--reqtype", "ignition.msgs.WorldControl",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "5000",
                "--req", "pause: false",
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo without the graphical client",
        ),
        DeclareLaunchArgument(
            "node_2_initial_east_m",
            default_value="4.0",
            description="Node 2 INS east initialization; use 4.5 only for tests",
        ),
        DeclareLaunchArgument(
            "record_bag",
            default_value="true",
            description="Record simulation inputs, truth and algorithm outputs",
        ),
        DeclareLaunchArgument(
            "bag_output",
            default_value=str(
                Path.cwd()
                / "bags"
                / datetime.now().strftime("gazebo_three_vehicle_%Y%m%d_%H%M%S_%f")
            ),
            description="Rosbag output directory",
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [model_path, ":", EnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", default_value=""
            )],
        ),
        SetEnvironmentVariable(
            "IGN_PARTITION",
            ["zju_coop_gazebo_", EnvironmentVariable(
                "ROS_DOMAIN_ID", default_value="0"
            )],
        ),
        gazebo,
        bridge,
        *local_nodes,
        fusion,
        scenario,
        recorder,
        start_simulation,
    ])

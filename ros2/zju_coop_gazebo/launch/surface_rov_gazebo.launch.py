"""ROS 2 Humble + Gazebo Fortress near-surface ROV scenario."""

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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


def generate_launch_description():
    package_share = FindPackageShare("zju_coop_gazebo")
    ros_gz_share = FindPackageShare("ros_gz_sim")
    world = PathJoinSubstitution([package_share, "worlds", "surface_rov.sdf"])
    model_path = PathJoinSubstitution([package_share, "models"])
    bridge_config = PathJoinSubstitution([
        package_share, "config", "surface_rov_bridge.yaml"
    ])
    headless = LaunchConfiguration("headless")
    software_rendering = LaunchConfiguration("software_rendering")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(
                    headless,
                    if_value="-r -s --render-engine-server ogre2 ",
                    else_value="-r --render-engine-server ogre2 ",
                ),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="surface_rov_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config}],
    )
    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--use-sim-time", "-o", bag_output,
            "/clock", "/surface_rov/imu/data",
            "/surface_rov/camera/overhead/image_raw",
            "/surface_rov/camera/overhead/camera_info",
            "/simulation/ground_truth/poses",
        ],
        output="screen",
        condition=IfCondition(record_bag),
    )
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("software_rendering", default_value="false"),
        DeclareLaunchArgument("record_bag", default_value="false"),
        DeclareLaunchArgument(
            "bag_output",
            default_value=str(
                Path.cwd() / "bags" / datetime.now().strftime(
                    "surface_rov_%Y%m%d_%H%M%S"
                )
            ),
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [
                model_path,
                ":",
                EnvironmentVariable(
                    "IGN_GAZEBO_RESOURCE_PATH", default_value=""
                ),
            ],
        ),
        SetEnvironmentVariable(
            "LIBGL_ALWAYS_SOFTWARE",
            IfElseSubstitution(
                software_rendering,
                if_value="1",
                else_value=EnvironmentVariable(
                    "LIBGL_ALWAYS_SOFTWARE", default_value="0"
                ),
            ),
        ),
        gazebo,
        bridge,
        recorder,
    ])

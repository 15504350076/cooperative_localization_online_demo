from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("zju_coop_bringup")

    namespace = LaunchConfiguration("namespace")
    local_config = LaunchConfiguration("local_config")
    fusion_config = LaunchConfiguration("fusion_config")
    imu_topic = LaunchConfiguration("imu_topic")
    run_fusion = LaunchConfiguration("run_fusion")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="vehicle_1"),
            DeclareLaunchArgument(
                "local_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "vehicle_1.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "fusion_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "fusion.yaml"]
                ),
            ),
            DeclareLaunchArgument("imu_topic", default_value="imu/data"),
            DeclareLaunchArgument("run_fusion", default_value="true"),
            Node(
                package="zju_coop_ros2",
                executable="zju_local_inertial_node",
                name="zju_local_inertial_node",
                namespace=namespace,
                output="screen",
                parameters=[local_config],
                remappings=[
                    ("imu", imu_topic),
                    ("node_state", "/cooperative_localization/node_state"),
                ],
            ),
            Node(
                package="zju_coop_ros2",
                executable="zju_cooperative_fusion_node",
                name="zju_cooperative_fusion_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(run_fusion),
                parameters=[fusion_config],
                remappings=[
                    ("node_state", "/cooperative_localization/node_state"),
                    ("uwb_range", "/uwb/range"),
                    ("poses_2d", "/cooperative_localization/poses_2d"),
                ],
            ),
        ]
    )

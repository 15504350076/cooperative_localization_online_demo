from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("zju_coop_bringup")
    fusion_config = LaunchConfiguration("fusion_config")
    reference_node_id = LaunchConfiguration("reference_node_id")
    uwb_topic = LaunchConfiguration("uwb_topic")
    poses_topic = LaunchConfiguration("poses_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "fusion_config",
            default_value=PathJoinSubstitution([
                package_share,
                "config",
                "fusion.yaml",
            ]),
        ),
        DeclareLaunchArgument("reference_node_id", default_value="1"),
        DeclareLaunchArgument("uwb_topic", default_value="/uwb/range"),
        DeclareLaunchArgument(
            "poses_topic",
            default_value="/cooperative_localization/poses_2d",
        ),
        Node(
            package="zju_coop_ros2",
            executable="zju_cooperative_fusion_node",
            name="zju_uwb_only_fusion",
            output="screen",
            parameters=[
                fusion_config,
                {
                    "input_mode": "uwb_only",
                    "reference_node_id": ParameterValue(
                        reference_node_id, value_type=int
                    ),
                },
            ],
            remappings=[
                ("uwb_range", uwb_topic),
                ("poses_2d", poses_topic),
            ],
        ),
    ])

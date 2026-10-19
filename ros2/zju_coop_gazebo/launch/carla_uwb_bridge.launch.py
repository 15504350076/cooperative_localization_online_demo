"""Bridge CARLA/AWSIM odometry to this project's UWB and truth topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic_1", default_value="/carla/vehicle_1/odometry"
        ),
        DeclareLaunchArgument(
            "odom_topic_2", default_value="/carla/vehicle_2/odometry"
        ),
        DeclareLaunchArgument(
            "odom_topic_3", default_value="/carla/vehicle_3/odometry"
        ),
        DeclareLaunchArgument("range_noise_std_m", default_value="0.02"),
        DeclareLaunchArgument("dropout_probability", default_value="0.0"),
        Node(
            package="zju_coop_gazebo",
            executable="zju_carla_uwb_bridge",
            output="screen",
            parameters=[{
                "odom_topic_1": LaunchConfiguration("odom_topic_1"),
                "odom_topic_2": LaunchConfiguration("odom_topic_2"),
                "odom_topic_3": LaunchConfiguration("odom_topic_3"),
                "range_noise_std_m": LaunchConfiguration("range_noise_std_m"),
                "dropout_probability": LaunchConfiguration(
                    "dropout_probability"
                ),
            }],
        ),
    ])

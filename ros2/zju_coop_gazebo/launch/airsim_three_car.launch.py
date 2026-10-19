"""Start the AirSim three-car odometry and project UWB adapters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="127.0.0.1"),
        DeclareLaunchArgument("port", default_value="41461"),
        DeclareLaunchArgument("airsim_python_path", default_value=""),
        DeclareLaunchArgument(
            "vehicle_names", default_value="Car_1,Car_2,Car_3"
        ),
        DeclareLaunchArgument(
            "initial_positions_enu", default_value="0,0;12,0;0,12"
        ),
        DeclareLaunchArgument("range_noise_std_m", default_value="0.02"),
        DeclareLaunchArgument("dropout_probability", default_value="0.0"),
        Node(
            package="zju_coop_gazebo",
            executable="zju_airsim_odometry_bridge",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("host"),
                "port": LaunchConfiguration("port"),
                "airsim_python_path": LaunchConfiguration("airsim_python_path"),
                "vehicle_names": LaunchConfiguration("vehicle_names"),
                "initial_positions_enu": LaunchConfiguration(
                    "initial_positions_enu"
                ),
            }],
        ),
        Node(
            package="zju_coop_gazebo",
            executable="zju_sim_uwb_bridge",
            output="screen",
            parameters=[{
                "odom_topic_1": "/airsim/Car_1/odometry",
                "odom_topic_2": "/airsim/Car_2/odometry",
                "odom_topic_3": "/airsim/Car_3/odometry",
                "range_noise_std_m": LaunchConfiguration("range_noise_std_m"),
                "dropout_probability": LaunchConfiguration(
                    "dropout_probability"
                ),
            }],
        ),
    ])

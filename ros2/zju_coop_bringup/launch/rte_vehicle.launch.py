"""显式启用单车局部惯导与可选三车RTE融合；使用独立话题和DDS桥端口。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, IfElseSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("zju_coop_bringup")
    node_id = LaunchConfiguration("node_id")
    bridge = LaunchConfiguration("use_localhost_imu_bridge")
    local_domain = IfElseSubstitution(bridge, if_value="1", else_value=EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="0"))
    shared_domain = IfElseSubstitution(bridge, if_value="0", else_value=EnvironmentVariable("ROS_LOCALHOST_ONLY", default_value="0"))
    local_topic = "/cooperative_localization/rte/node_state_local"
    shared_topic = LaunchConfiguration("node_state_topic")
    state_topic = IfElseSubstitution(bridge, if_value=local_topic, else_value=shared_topic)
    namespace = PythonExpression(["'rte_vehicle_' + '", node_id, "'"])
    return LaunchDescription([
        DeclareLaunchArgument("node_id", default_value="1"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/raw"),
        DeclareLaunchArgument("local_config", default_value=PathJoinSubstitution([share, "config", "vehicle_1.yaml"])),
        DeclareLaunchArgument("fusion_config", default_value=PathJoinSubstitution([share, "config", "rte_fusion.yaml"])),
        DeclareLaunchArgument("run_fusion", default_value="false"),
        DeclareLaunchArgument("use_localhost_imu_bridge", default_value="false"),
        DeclareLaunchArgument("node_state_loopback_port", default_value="45130"),
        DeclareLaunchArgument("node_state_topic", default_value="/cooperative_localization/rte/node_state"),
        DeclareLaunchArgument("uwb_topic", default_value="/uwb/range"),
        DeclareLaunchArgument("poses_topic", default_value="/cooperative_localization/rte/poses_2d"),
        DeclareLaunchArgument("status_topic", default_value="/cooperative_localization/rte/status"),
        Node(package="zju_coop_ros2", executable="zju_local_inertial_node", name="zju_local_inertial_node",
             namespace=namespace, output="screen", additional_env={"ROS_LOCALHOST_ONLY": local_domain},
             parameters=[LaunchConfiguration("local_config"), {
                 "node_id": ParameterValue(node_id, value_type=int),
                 "common_enu_frame_id": PythonExpression(["'rte_odom_' + '", node_id, "'"]),
                 "initial_position_enu_m": [0.0, 0.0, 0.0], "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
                 "initial_orientation_flu_to_enu_xyzw": [0.0, 0.0, 0.0, 1.0],
                 "enable_static_imu_initialization": True, "publish_rate_hz": 50.0,
             }], remappings=[("imu", LaunchConfiguration("imu_topic")), ("node_state", state_topic)]),
        Node(package="zju_coop_ros2", executable="zju_node_state_local_sender", namespace=namespace,
             condition=IfCondition(bridge), additional_env={"ROS_LOCALHOST_ONLY": "1"},
             parameters=[{"loopback_port": ParameterValue(LaunchConfiguration("node_state_loopback_port"), value_type=int)}],
             remappings=[("node_state_local", local_topic)]),
        Node(package="zju_coop_ros2", executable="zju_node_state_shared_relay", namespace=namespace,
             condition=IfCondition(bridge), additional_env={"ROS_LOCALHOST_ONLY": "0"},
             parameters=[{"loopback_port": ParameterValue(LaunchConfiguration("node_state_loopback_port"), value_type=int)}],
             remappings=[("node_state_shared", shared_topic)]),
        Node(package="zju_coop_ros2", executable="zju_three_vehicle_rte_node", name="zju_three_vehicle_rte_node",
             condition=IfCondition(LaunchConfiguration("run_fusion")), output="screen",
             additional_env={"ROS_LOCALHOST_ONLY": shared_domain}, parameters=[LaunchConfiguration("fusion_config")],
             remappings=[("node_state", shared_topic), ("uwb_range", LaunchConfiguration("uwb_topic")),
                         ("poses_2d", LaunchConfiguration("poses_topic")), ("status", LaunchConfiguration("status_topic"))]),
    ])

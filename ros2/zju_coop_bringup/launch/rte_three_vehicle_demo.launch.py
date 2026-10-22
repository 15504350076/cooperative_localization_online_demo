"""独立命名空间的三车原始IMU+UWB演示，不向既有定位话题发布。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    actions = [DeclareLaunchArgument("duration_s", default_value="30.0")]
    for i in range(1, 4):
        actions.append(Node(
            package="zju_coop_ros2", executable="zju_local_inertial_node",
            name="zju_local_inertial_node", namespace=f"rte_demo/vehicle_{i}", output="screen",
            parameters=[{
                "node_id": i, "publish_rate_hz": 50.0, "expected_imu_frame_id": "imu_link",
                "common_enu_frame_id": f"rte_odom_{i}",
                "initial_position_enu_m": [0.0, 0.0, 0.0], "initial_velocity_enu_mps": [0.0, 0.0, 0.0],
                "initial_orientation_flu_to_enu_xyzw": [0.0, 0.0, 0.0, 1.0],
                "enable_static_imu_initialization": True, "static_imu_initialization_duration_s": .5,
                "static_imu_minimum_sample_count": 30,
            }], remappings=[("imu", f"/rte_demo/imu_{i}"), ("node_state", "/rte_demo/node_state")]))
    actions.append(Node(
        package="zju_coop_ros2", executable="zju_three_vehicle_rte_node", namespace="rte_demo", output="screen",
        parameters=[{"node_ids": [1, 2, 3], "range_std_m": .02}],
        remappings=[("node_state", "/rte_demo/node_state"), ("uwb_range", "/rte_demo/uwb")]))
    actions.append(TimerAction(period=1.0, actions=[Node(
        package="zju_coop_ros2", executable="zju_three_vehicle_rte_simulator", namespace="rte_demo", output="screen",
        parameters=[{"duration_s": ParameterValue(LaunchConfiguration("duration_s"), value_type=float)}])]))
    return LaunchDescription(actions)

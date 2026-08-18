from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("zju_coop_bringup")

    namespace = LaunchConfiguration("namespace")
    local_config = LaunchConfiguration("local_config")
    fusion_config = LaunchConfiguration("fusion_config")
    imu_topic = LaunchConfiguration("imu_topic")
    run_fusion = LaunchConfiguration("run_fusion")
    use_gnss_range_fallback = LaunchConfiguration("use_gnss_range_fallback")
    uwb_range_std_m = LaunchConfiguration("uwb_range_std_m")
    gnss_range_std_m = LaunchConfiguration("gnss_range_std_m")
    gnss_sync_slop_ms = LaunchConfiguration("gnss_sync_slop_ms")
    range_input_topic = IfElseSubstitution(
        use_gnss_range_fallback,
        if_value="/cooperative_localization/gnss_derived_range",
        else_value="/uwb/range",
    )
    range_std_m = IfElseSubstitution(
        use_gnss_range_fallback,
        if_value=gnss_range_std_m,
        else_value=uwb_range_std_m,
    )

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
            DeclareLaunchArgument(
                "use_gnss_range_fallback",
                default_value="false",
                description="Use GNSS-derived ranges instead of /uwb/range",
            ),
            DeclareLaunchArgument(
                "gnss_topic_1", default_value="/vehicle_1/gnss/fix"
            ),
            DeclareLaunchArgument(
                "gnss_topic_2", default_value="/vehicle_2/gnss/fix"
            ),
            DeclareLaunchArgument(
                "gnss_topic_3", default_value="/vehicle_3/gnss/fix"
            ),
            DeclareLaunchArgument(
                "uwb_range_std_m",
                default_value="0.1",
                description="UWB range standard deviation used by fusion",
            ),
            DeclareLaunchArgument(
                "gnss_range_std_m",
                default_value="3.0",
                description="Conservative GNSS-derived range standard deviation",
            ),
            DeclareLaunchArgument(
                "gnss_sync_slop_ms",
                default_value="50",
                description="Maximum stamp spread of one three-fix epoch",
            ),
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
                parameters=[
                    fusion_config,
                    {"range_std_m": ParameterValue(range_std_m, value_type=float)},
                ],
                remappings=[
                    ("node_state", "/cooperative_localization/node_state"),
                    ("uwb_range", range_input_topic),
                    ("poses_2d", "/cooperative_localization/poses_2d"),
                ],
            ),
            Node(
                package="zju_coop_ros2",
                executable="zju_gnss_range_fallback_node",
                name="zju_gnss_range_fallback_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(
                    AndSubstitution(use_gnss_range_fallback, run_fusion)
                ),
                parameters=[{
                    "node_ids": [1, 2, 3],
                    "max_stamp_skew_ms": ParameterValue(
                        gnss_sync_slop_ms, value_type=int
                    ),
                }],
                remappings=[
                    ("fix_1", LaunchConfiguration("gnss_topic_1")),
                    ("fix_2", LaunchConfiguration("gnss_topic_2")),
                    ("fix_3", LaunchConfiguration("gnss_topic_3")),
                    (
                        "range",
                        "/cooperative_localization/gnss_derived_range",
                    ),
                ],
            ),
        ]
    )

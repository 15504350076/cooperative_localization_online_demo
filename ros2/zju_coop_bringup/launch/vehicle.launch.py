from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution,
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PythonExpression,
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
    point_cloud_topic = LaunchConfiguration("point_cloud_topic")
    camera_image_topic = LaunchConfiguration("camera_image_topic")
    enable_point_cloud_input = LaunchConfiguration("enable_point_cloud_input")
    enable_camera_input = LaunchConfiguration("enable_camera_input")
    enable_lidar_frontend = LaunchConfiguration("enable_lidar_frontend")
    enable_lidar_slam = LaunchConfiguration("enable_lidar_slam")
    lidar_frontend_enabled = PythonExpression([
        "'", enable_lidar_frontend, "'.lower() == 'true' or '",
        enable_lidar_slam, "'.lower() == 'true'",
    ])
    run_fusion = LaunchConfiguration("run_fusion")
    use_gnss_range_fallback = LaunchConfiguration("use_gnss_range_fallback")
    enable_follower_feedback = LaunchConfiguration(
        "enable_follower_feedback"
    )
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
            DeclareLaunchArgument(
                "point_cloud_topic", default_value="lidar/points"
            ),
            DeclareLaunchArgument(
                "camera_image_topic", default_value="camera/image_raw"
            ),
            DeclareLaunchArgument(
                "enable_point_cloud_input",
                default_value="false",
                description="Validate local PointCloud2 input without fusion",
            ),
            DeclareLaunchArgument(
                "enable_camera_input",
                default_value="false",
                description="Validate local Image input without fusion",
            ),
            DeclareLaunchArgument(
                "enable_lidar_frontend",
                default_value="false",
                description=(
                    "Run local planar ICP lidar odometry without fusion"
                ),
            ),
            DeclareLaunchArgument(
                "enable_lidar_slam",
                default_value="false",
                description=(
                    "Run local RTAB-Map 2D lidar SLAM; this automatically "
                    "enables ICP odometry and does not feed the cooperative "
                    "filter"
                ),
            ),
            DeclareLaunchArgument(
                "lidar_odometry_topic", default_value="lidar/odometry"
            ),
            DeclareLaunchArgument(
                "lidar_frame_id",
                default_value=PythonExpression([
                    "'", namespace, "' + '/base_link/front_lidar'",
                ]),
            ),
            DeclareLaunchArgument(
                "lidar_odom_frame_id",
                default_value=PythonExpression([
                    "'", namespace, "' + '/lidar_odom'",
                ]),
            ),
            DeclareLaunchArgument(
                "lidar_map_frame_id",
                default_value=PythonExpression([
                    "'", namespace, "' + '/lidar_map'",
                ]),
            ),
            DeclareLaunchArgument(
                "lidar_slam_database_path",
                default_value=PathJoinSubstitution([
                    EnvironmentVariable("HOME"),
                    ".ros",
                    "zju_coop_lidar_slam.db",
                ]),
            ),
            DeclareLaunchArgument(
                "point_cloud_sensor_id", default_value="1"
            ),
            DeclareLaunchArgument("camera_id", default_value="1"),
            DeclareLaunchArgument(
                "point_cloud_frame_alias", default_value=""
            ),
            DeclareLaunchArgument("camera_frame_alias", default_value=""),
            DeclareLaunchArgument("run_fusion", default_value="true"),
            DeclareLaunchArgument(
                "enable_follower_feedback",
                default_value="false",
                description=(
                    "Publish the cooperative pose array on the dedicated "
                    "follower feedback topic"
                ),
            ),
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
                parameters=[
                    local_config,
                    {
                        "enable_point_cloud_input": ParameterValue(
                            enable_point_cloud_input, value_type=bool
                        ),
                        "enable_camera_input": ParameterValue(
                            enable_camera_input, value_type=bool
                        ),
                        "point_cloud_sensor_id": ParameterValue(
                            LaunchConfiguration("point_cloud_sensor_id"),
                            value_type=int,
                        ),
                        "camera_id": ParameterValue(
                            LaunchConfiguration("camera_id"), value_type=int
                        ),
                        "point_cloud_frame_alias": LaunchConfiguration(
                            "point_cloud_frame_alias"
                        ),
                        "camera_frame_alias": LaunchConfiguration(
                            "camera_frame_alias"
                        ),
                    },
                ],
                remappings=[
                    ("imu", imu_topic),
                    ("point_cloud", point_cloud_topic),
                    ("camera_image", camera_image_topic),
                    ("node_state", "/cooperative_localization/node_state"),
                ],
            ),
            Node(
                package="zju_coop_perception",
                executable="zju_lidar_odometry_node",
                name="zju_lidar_odometry_node",
                namespace=namespace,
                output="screen",
                condition=IfCondition(lidar_frontend_enabled),
                parameters=[{
                    "odom_frame_id": LaunchConfiguration(
                        "lidar_odom_frame_id"
                    ),
                    "publish_tf": ParameterValue(
                        enable_lidar_slam, value_type=bool
                    ),
                }],
                remappings=[
                    ("point_cloud", point_cloud_topic),
                    (
                        "lidar_odometry",
                        LaunchConfiguration("lidar_odometry_topic"),
                    ),
                ],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="zju_lidar_slam",
                namespace=namespace,
                output="screen",
                condition=IfCondition(enable_lidar_slam),
                parameters=[{
                    "subscribe_depth": False,
                    "subscribe_rgb": False,
                    "subscribe_scan": False,
                    "subscribe_scan_cloud": True,
                    "scan_cloud_is_2d": True,
                    "subscribe_odom_info": False,
                    "frame_id": LaunchConfiguration("lidar_frame_id"),
                    "map_frame_id": LaunchConfiguration(
                        "lidar_map_frame_id"
                    ),
                    "odom_frame_id": LaunchConfiguration(
                        "lidar_odom_frame_id"
                    ),
                    "publish_tf": True,
                    "wait_for_transform": 0.5,
                    "topic_queue_size": 1,
                    "qos_scan": 2,
                    "database_path": LaunchConfiguration(
                        "lidar_slam_database_path"
                    ),
                    "Reg/Strategy": "1",
                    "Reg/Force3DoF": "true",
                    "Grid/3D": "false",
                    "Grid/FromDepth": "false",
                    "Mem/NotLinkedNodesKept": "false",
                }],
                remappings=[
                    ("scan_cloud", point_cloud_topic),
                    ("map", "lidar_slam/map"),
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
                    {
                        "range_std_m": ParameterValue(
                            range_std_m, value_type=float
                        ),
                        "enable_follower_feedback": ParameterValue(
                            AndSubstitution(
                                enable_follower_feedback,
                                NotSubstitution(use_gnss_range_fallback),
                            ),
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[
                    ("node_state", "/cooperative_localization/node_state"),
                    ("uwb_range", range_input_topic),
                    ("poses_2d", "/cooperative_localization/poses_2d"),
                    (
                        "feedback_poses_2d",
                        "/cooperative_localization/feedback/poses_2d",
                    ),
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

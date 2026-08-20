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
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


NODE_STATE_TOPIC = "/cooperative_localization/node_state"
BAG_TOPICS = (
    "/clock",
    "/tf",
    "/vehicle_1/imu/data",
    "/vehicle_2/imu/data",
    "/vehicle_3/imu/data",
    "/uwb/range",
    "/simulation/ground_truth/poses_2d",
    NODE_STATE_TOPIC,
    "/cooperative_localization/poses_2d",
    "/cooperative_localization/feedback/poses_2d",
    "/vehicle_1/lidar/points",
    "/vehicle_2/lidar/points",
    "/vehicle_3/lidar/points",
    "/vehicle_1/camera/image_raw",
    "/vehicle_2/camera/image_raw",
    "/vehicle_3/camera/image_raw",
    "/vehicle_1/camera/camera_info",
    "/vehicle_2/camera/camera_info",
    "/vehicle_3/camera/camera_info",
    "/vehicle_1/lidar/odometry",
    "/vehicle_2/lidar/odometry",
    "/vehicle_3/lidar/odometry",
    "/vehicle_1/lidar_slam/map",
    "/vehicle_2/lidar_slam/map",
    "/vehicle_3/lidar_slam/map",
    "/vehicle_1/visual/odometry",
    "/vehicle_2/visual/odometry",
    "/vehicle_3/visual/odometry",
)


def _local_node(node_id, position, yaw, enable_lidar, enable_camera):
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
            "enable_point_cloud_input": ParameterValue(
                enable_lidar, value_type=bool
            ),
            "enable_camera_input": ParameterValue(
                enable_camera, value_type=bool
            ),
            # The public raw-input ABI v1 has a 31-byte frame capacity.
            # These aliases are local to node_id and are never written back
            # into the ROS messages or advertised as a TF tree.
            "point_cloud_frame_alias": "front_lidar",
            "camera_frame_alias": "front_camera",
        }],
        remappings=[
            ("imu", f"/vehicle_{node_id}/imu/data"),
            ("point_cloud", f"/vehicle_{node_id}/lidar/points"),
            ("camera_image", f"/vehicle_{node_id}/camera/image_raw"),
            ("node_state", NODE_STATE_TOPIC),
        ],
    )


def generate_launch_description():
    package_share = FindPackageShare("zju_coop_gazebo")
    ros_gz_share = FindPackageShare("ros_gz_sim")
    world = PathJoinSubstitution(
        [package_share, "worlds", "three_vehicle.sdf"]
    )
    bridge_config = PathJoinSubstitution(
        [package_share, "config", "bridge.yaml"]
    )
    lidar_bridge_config = PathJoinSubstitution(
        [package_share, "config", "lidar_bridge.yaml"]
    )
    camera_bridge_config = PathJoinSubstitution(
        [package_share, "config", "camera_bridge.yaml"]
    )
    gazebo_gui_config = PathJoinSubstitution(
        [package_share, "config", "gazebo_gui.config"]
    )
    slam_rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "vehicle_1_lidar_slam.rviz"]
    )
    model_path = PathJoinSubstitution([package_share, "models"])
    headless = LaunchConfiguration("headless")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_lidar_frontend = LaunchConfiguration("enable_lidar_frontend")
    enable_lidar_slam = LaunchConfiguration("enable_lidar_slam")
    enable_slam_rviz = LaunchConfiguration("enable_slam_rviz")
    lidar_slam_database_directory = LaunchConfiguration(
        "lidar_slam_database_directory"
    )
    enable_visual_frontend = LaunchConfiguration("enable_visual_frontend")
    enable_follower_feedback = LaunchConfiguration(
        "enable_follower_feedback"
    )
    bag_output = LaunchConfiguration("bag_output")
    ign_partition = LaunchConfiguration("ign_partition")
    software_rendering = LaunchConfiguration("software_rendering")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_share, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                IfElseSubstitution(
                    headless,
                    if_value="-s --render-engine-server ogre2 ",
                    else_value=[
                        "--render-engine-gui ogre "
                        "--render-engine-server ogre2 ",
                        "--gui-config ",
                        gazebo_gui_config,
                        " ",
                    ],
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
    lidar_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zju_gazebo_lidar_bridge",
        output="screen",
        parameters=[{"config_file": lidar_bridge_config}],
        condition=IfCondition(PythonExpression([
            "'", enable_lidar, "'.lower() == 'true' or '",
            enable_lidar_frontend, "'.lower() == 'true' or '",
            enable_lidar_slam, "'.lower() == 'true'",
        ])),
    )
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zju_gazebo_camera_bridge",
        output="screen",
        parameters=[{"config_file": camera_bridge_config}],
        condition=IfCondition(PythonExpression([
            "'", enable_camera, "'.lower() == 'true' or '",
            enable_visual_frontend, "'.lower() == 'true'",
        ])),
    )
    lidar_frontends = [
        Node(
            package="zju_coop_perception",
            executable="zju_lidar_odometry_node",
            name=f"zju_lidar_odometry_node_{node_id}",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "odom_frame_id": f"vehicle_{node_id}/lidar_odom",
                "minimum_points": 80,
                "minimum_geometry_ratio": 0.02,
                "max_range_m": 40.0,
                "max_correspondence_distance_m": 0.6,
                "max_fitness_score_m2": 0.05,
                "publish_tf": ParameterValue(
                    enable_lidar_slam, value_type=bool
                ),
            }],
            remappings=[
                ("point_cloud", f"/vehicle_{node_id}/lidar/points"),
                ("lidar_odometry", f"/vehicle_{node_id}/lidar/odometry"),
            ],
            condition=IfCondition(PythonExpression([
                "'", enable_lidar_frontend, "'.lower() == 'true' or '",
                enable_lidar_slam, "'.lower() == 'true'",
            ])),
        )
        for node_id in (1, 2, 3)
    ]
    lidar_slam_nodes = [
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace=f"vehicle_{node_id}/lidar_slam",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "subscribe_scan": False,
                "subscribe_scan_cloud": True,
                "scan_cloud_is_2d": True,
                "subscribe_odom_info": False,
                "frame_id": f"vehicle_{node_id}/base_link/front_lidar",
                "map_frame_id": f"vehicle_{node_id}/lidar_map",
                "odom_frame_id": f"vehicle_{node_id}/lidar_odom",
                "publish_tf": True,
                "wait_for_transform": 0.5,
                "topic_queue_size": 1,
                "qos_scan": 2,
                "database_path": PathJoinSubstitution([
                    lidar_slam_database_directory,
                    f"zju_coop_vehicle_{node_id}_lidar_slam.db",
                ]),
                "Reg/Strategy": "1",
                "Reg/Force3DoF": "true",
                "Grid/3D": "false",
                "Grid/FromDepth": "false",
                "Mem/NotLinkedNodesKept": "false",
            }],
            remappings=[
                ("scan_cloud", f"/vehicle_{node_id}/lidar/points"),
            ],
            condition=IfCondition(enable_lidar_slam),
        )
        for node_id in (1, 2, 3)
    ]
    slam_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="zju_reference_lidar_slam_rviz",
        output="screen",
        arguments=["-d", slam_rviz_config],
        parameters=[{"use_sim_time": True}],
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
        condition=IfCondition(PythonExpression([
            "'", enable_lidar_slam, "'.lower() == 'true' and '",
            enable_slam_rviz, "'.lower() == 'true'",
        ])),
    )

    reference_landmark_points = [
        2.97, -0.55, 1.20,
        2.97, -1.15, 1.20,
        2.97, -1.15, 0.50,
        2.97, -0.55, 0.50,
    ]
    east_landmark_points = [
        11.82, 3.20, 2.00,
        11.82, 1.80, 2.00,
        11.82, 1.80, 0.40,
        11.82, 3.20, 0.40,
    ]
    north_landmark_points = [
        -0.60, 9.82, 1.60,
        0.60, 9.82, 1.60,
        0.60, 9.82, 0.40,
        -0.60, 9.82, 0.40,
    ]
    visual_frontends = [
        Node(
            package="zju_coop_perception",
            executable="zju_colored_landmark_localizer",
            name=f"zju_visual_landmark_localizer_{node_id}",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "output_frame_id": "common_enu",
                "child_frame_id": f"vehicle_{node_id}/base_link",
                "expected_yaw_rad": expected_yaw,
                "expected_base_z_m": 0.25,
                "initial_position_enu_m": initial_position,
                "landmark_points_enu_m": landmark_points,
                "base_to_camera_translation_flu_m": [0.41, 0.0, 0.10],
                "max_position_jump_m": 0.15,
                "max_initial_position_error_m": 3.0,
                "max_yaw_jump_rad": 0.08,
            }],
            remappings=[
                ("camera_image", f"/vehicle_{node_id}/camera/image_raw"),
                ("camera_info", f"/vehicle_{node_id}/camera/camera_info"),
                ("visual_odometry", f"/vehicle_{node_id}/visual/odometry"),
            ],
            condition=IfCondition(enable_visual_frontend),
        )
        for node_id, expected_yaw, initial_position, landmark_points in (
            (1, 0.0, [0.0, 0.0, 0.25], reference_landmark_points),
            (2, 0.0, [4.0, 0.0, 0.25], east_landmark_points),
            (3, math.pi / 2.0, [0.0, 3.0, 0.25], north_landmark_points),
        )
    ]
    local_nodes = [
        _local_node(
            1, (0.0, 0.0, 0.25), 0.0, enable_lidar, enable_camera
        ),
        _local_node(
            2,
            (LaunchConfiguration("node_2_initial_east_m"), 0.0, 0.25),
            0.0,
            enable_lidar,
            enable_camera,
        ),
        _local_node(
            3,
            (0.0, 3.0, 0.25),
            math.pi / 2.0,
            enable_lidar,
            enable_camera,
        ),
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
            "enable_follower_feedback": ParameterValue(
                enable_follower_feedback, value_type=bool
            ),
        }],
        remappings=[
            ("node_state", NODE_STATE_TOPIC),
            ("uwb_range", "/uwb/range"),
            ("poses_2d", "/cooperative_localization/poses_2d"),
            (
                "feedback_poses_2d",
                "/cooperative_localization/feedback/poses_2d",
            ),
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
        period=5.0,
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
            description=(
                "Node 2 INS east initialization; use 4.5 only for tests"
            ),
        ),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="false",
            description=(
                "Bridge and validate the three optional Gazebo PointCloud2 "
                "streams; they are not fused into localization"
            ),
        ),
        DeclareLaunchArgument(
            "enable_camera",
            default_value="false",
            description=(
                "Bridge Image/CameraInfo and validate each Image stream; "
                "they are not fused into localization"
            ),
        ),
        DeclareLaunchArgument(
            "enable_lidar_frontend",
            default_value="false",
            description=(
                "Run three local planar ICP odometry frontends; outputs are "
                "diagnostic and are not fused"
            ),
        ),
        DeclareLaunchArgument(
            "enable_lidar_slam",
            default_value="false",
            description=(
                "Run three independent RTAB-Map 2D lidar SLAM backends; "
                "this automatically enables the lidar bridge and ICP "
                "odometry, and does not feed the cooperative filter"
            ),
        ),
        DeclareLaunchArgument(
            "enable_slam_rviz",
            default_value="false",
            description=(
                "Open RViz with the reference vehicle lidar map; requires "
                "enable_lidar_slam=true"
            ),
        ),
        DeclareLaunchArgument(
            "lidar_slam_database_directory",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"),
                ".ros",
            ]),
            description=(
                "Directory for the three persistent RTAB-Map databases"
            ),
        ),
        DeclareLaunchArgument(
            "enable_visual_frontend",
            default_value="false",
            description=(
                "Run three known-landmark metric visual localizers; outputs "
                "are diagnostic and are not fused"
            ),
        ),
        DeclareLaunchArgument(
            "enable_follower_feedback",
            default_value="false",
            description=(
                "Publish the corrected cooperative pose array on the "
                "dedicated follower feedback topic"
            ),
        ),
        DeclareLaunchArgument(
            "software_rendering",
            default_value="false",
            description=(
                "Use Mesa software rendering; leave disabled for a smoother "
                "GUI and enable explicitly for headless sensor tests"
            ),
        ),
        DeclareLaunchArgument(
            "record_bag",
            default_value="true",
            description=(
                "Record simulation inputs, truth and algorithm outputs"
            ),
        ),
        DeclareLaunchArgument(
            "bag_output",
            default_value=str(
                Path.cwd()
                / "bags"
                / datetime.now().strftime(
                    "gazebo_three_vehicle_%Y%m%d_%H%M%S_%f"
                )
            ),
            description="Rosbag output directory",
        ),
        DeclareLaunchArgument(
            "ign_partition",
            default_value=(
                "zju_coop_gazebo_"
                + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            ),
            description=(
                "Gazebo Transport partition; unique by default so a stale "
                "server cannot produce a black GUI on the next launch"
            ),
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            [model_path, ":", EnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", default_value=""
            )],
        ),
        SetEnvironmentVariable(
            "IGN_PARTITION",
            ign_partition,
        ),
        SetEnvironmentVariable(
            "LIBGL_ALWAYS_SOFTWARE",
            IfElseSubstitution(
                software_rendering,
                if_value="1",
                else_value="0",
            ),
        ),
        gazebo,
        bridge,
        lidar_bridge,
        camera_bridge,
        *lidar_frontends,
        *lidar_slam_nodes,
        slam_rviz,
        *visual_frontends,
        *local_nodes,
        fusion,
        scenario,
        recorder,
        start_simulation,
    ])

"""Static contract for the optional Gazebo Fortress demo assets."""

import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


class GazeboAssetTest(unittest.TestCase):

    def test_assets_form_one_complete_three_vehicle_pipeline(self):
        model_path = PACKAGE_ROOT / "models/zju_diff_vehicle/model.sdf"
        reference_model_path = (
            PACKAGE_ROOT / "models/zju_reference_vehicle/model.sdf"
        )
        world_path = PACKAGE_ROOT / "worlds/three_vehicle.sdf"
        bridge_path = PACKAGE_ROOT / "config/bridge.yaml"
        lidar_bridge_path = PACKAGE_ROOT / "config/lidar_bridge.yaml"
        camera_bridge_path = PACKAGE_ROOT / "config/camera_bridge.yaml"
        gazebo_gui_config_path = (
            PACKAGE_ROOT / "config/gazebo_gui.config"
        )
        rviz_config_path = (
            PACKAGE_ROOT / "rviz/vehicle_1_lidar_slam.rviz"
        )
        scenario_path = PACKAGE_ROOT / "scripts/gazebo_scenario.py"
        launch_path = PACKAGE_ROOT / "launch/three_vehicle_gazebo.launch.py"
        launch_test_path = (
            PACKAGE_ROOT / "test/test_three_vehicle_gazebo.launch.py"
        )
        slam_test_path = (
            PACKAGE_ROOT / "test/test_lidar_slam_gazebo.launch.py"
        )

        paths = (
            model_path,
            reference_model_path,
            world_path,
            bridge_path,
            lidar_bridge_path,
            camera_bridge_path,
            gazebo_gui_config_path,
            rviz_config_path,
            scenario_path,
            launch_path,
            launch_test_path,
            slam_test_path,
        )
        for path in paths:
            self.assertTrue(
                path.is_file(),
                f"missing Gazebo asset: {path.relative_to(PACKAGE_ROOT)}",
            )

        model = ET.parse(model_path).getroot().find("model")
        self.assertIsNotNone(model)
        self.assertEqual(
            model.find(".//sensor[@type='imu']/update_rate").text,
            "100",
        )
        self._assert_perception_sensors(model)
        plugin_names = {
            plugin.attrib["name"] for plugin in model.findall("plugin")
        }
        self.assertIn("ignition::gazebo::systems::DiffDrive", plugin_names)
        self.assertIn(
            "ignition::gazebo::systems::PosePublisher", plugin_names
        )
        self.assertEqual(
            model.find("joint[@name='caster_joint']").attrib["type"],
            "fixed",
        )
        reference_model = (
            ET.parse(reference_model_path).getroot().find("model")
        )
        self.assertIsNotNone(reference_model)
        self.assertEqual(
            reference_model.find(".//sensor[@type='imu']/update_rate").text,
            "100",
        )
        self._assert_perception_sensors(reference_model)
        reference_plugin_names = {
            plugin.attrib["name"]
            for plugin in reference_model.findall("plugin")
        }
        self.assertEqual(reference_plugin_names, plugin_names)
        body_diffuse = model.find(
            "link[@name='base_link']/visual[@name='body_visual']"
            "/material/diffuse"
        ).text
        reference_diffuse = reference_model.find(
            "link[@name='base_link']/visual[@name='body_visual']"
            "/material/diffuse"
        ).text
        self.assertNotEqual(body_diffuse, reference_diffuse)
        diff_drive = next(
            plugin
            for plugin in model.findall("plugin")
            if plugin.attrib["name"] == "ignition::gazebo::systems::DiffDrive"
        )
        self.assertEqual(
            diff_drive.findtext("max_linear_acceleration"), "0.30"
        )
        self.assertEqual(
            diff_drive.findtext("min_linear_acceleration"), "-0.30"
        )
        pose_publisher = next(
            plugin
            for plugin in model.findall("plugin")
            if plugin.attrib["name"]
            == "ignition::gazebo::systems::PosePublisher"
        )
        self.assertEqual(pose_publisher.findtext("publish_model_pose"), "true")
        self.assertEqual(
            pose_publisher.findtext("publish_nested_model_pose"), "true"
        )

        world = ET.parse(world_path).getroot().find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.attrib["name"], "zju_coop_gazebo")
        included = {
            include.findtext("name"): include.findtext("uri")
            for include in world.findall("include")
        }
        self.assertEqual(included, {
            "vehicle_1": "model://zju_reference_vehicle",
            "vehicle_2": "model://zju_diff_vehicle",
            "vehicle_3": "model://zju_diff_vehicle",
        })
        sensors_plugins = [
            plugin for plugin in world.findall("plugin")
            if plugin.attrib.get("name")
            == "ignition::gazebo::systems::Sensors"
        ]
        self.assertEqual(len(sensors_plugins), 1)
        self.assertEqual(
            sensors_plugins[0].attrib.get("filename"),
            "libignition-gazebo-sensors-system.so",
        )
        self.assertEqual(
            sensors_plugins[0].findtext("render_engine"), "ogre2"
        )
        self.assertEqual(
            world.find(
                "model[@name='ground_plane']/link/visual[@name='visual']"
            ).findtext("pose"),
            "0 0 -0.005 0 0 0",
        )
        self.assertEqual(world.findtext("scene/grid"), "false")
        arena_link = world.find("model[@name='arena_walls']/link")
        self.assertIsNotNone(arena_link)
        expected_walls = {
            "west": ("-3 11 0.6 0 0 0", "0.15 28 1.2"),
            "east": ("27 11 0.6 0 0 0", "0.15 28 1.2"),
            "south": ("12 -3 0.6 0 0 0", "30 0.15 1.2"),
            "north": ("12 25 0.6 0 0 0", "30 0.15 1.2"),
        }
        for name, (pose, size) in expected_walls.items():
            collision = arena_link.find(f"collision[@name='{name}_collision']")
            visual = arena_link.find(f"visual[@name='{name}_visual']")
            self.assertIsNotNone(collision)
            self.assertIsNotNone(visual)
            self.assertEqual(collision.findtext("pose"), pose)
            self.assertEqual(collision.findtext("geometry/box/size"), size)
            self.assertEqual(visual.findtext("pose"), pose)
            self.assertEqual(visual.findtext("geometry/box/size"), size)

        bridge = yaml.safe_load(bridge_path.read_text(encoding="utf-8"))
        ros_topics = {entry["ros_topic_name"] for entry in bridge}
        self.assertIn("/clock", ros_topics)
        self.assertNotIn("/simulation/gazebo/poses", ros_topics)
        for node_id in (1, 2, 3):
            self.assertIn(
                f"/simulation/gazebo/vehicle_{node_id}/pose",
                ros_topics,
            )
            self.assertIn(f"/vehicle_{node_id}/imu/data", ros_topics)
            self.assertIn(f"/vehicle_{node_id}/cmd_vel", ros_topics)

        lidar_bridge = yaml.safe_load(
            lidar_bridge_path.read_text(encoding="utf-8")
        )
        self._assert_bridge_contract(
            lidar_bridge,
            {
                f"/vehicle_{node_id}/lidar/points": (
                    "sensor_msgs/msg/PointCloud2",
                    "ignition.msgs.PointCloudPacked",
                )
                for node_id in (1, 2, 3)
            },
        )
        camera_contract = {}
        for node_id in (1, 2, 3):
            camera_contract[f"/vehicle_{node_id}/camera/image_raw"] = (
                "sensor_msgs/msg/Image",
                "ignition.msgs.Image",
            )
            camera_contract[f"/vehicle_{node_id}/camera/camera_info"] = (
                "sensor_msgs/msg/CameraInfo",
                "ignition.msgs.CameraInfo",
            )
        camera_bridge = yaml.safe_load(
            camera_bridge_path.read_text(encoding="utf-8")
        )
        self._assert_bridge_contract(camera_bridge, camera_contract)

        gazebo_gui_source = gazebo_gui_config_path.read_text(
            encoding="utf-8"
        )
        self.assertIn('filename="MinimalScene"', gazebo_gui_source)
        self.assertIn('filename="GridConfig"', gazebo_gui_source)
        self.assertIn(
            "<horizontal_cell_count>40</horizontal_cell_count>",
            gazebo_gui_source,
        )
        self.assertIn(
            "<vertical_cell_count>0</vertical_cell_count>",
            gazebo_gui_source,
        )
        self.assertIn("<cell_length>1</cell_length>", gazebo_gui_source)
        self.assertIn(
            "<pose>12 11 0.015 0 0 0</pose>", gazebo_gui_source
        )

        rviz_config = yaml.safe_load(
            rviz_config_path.read_text(encoding="utf-8")
        )
        manager = rviz_config["Visualization Manager"]
        self.assertEqual(
            manager["Global Options"]["Fixed Frame"],
            "vehicle_1/lidar_map",
        )
        displays = {
            display["Name"]: display for display in manager["Displays"]
        }
        map_display = displays["Reference vehicle map"]
        self.assertEqual(
            map_display["Topic"]["Value"],
            "/vehicle_1/lidar_slam/map",
        )
        self.assertEqual(
            map_display["Topic"]["Reliability Policy"], "Reliable"
        )
        self.assertEqual(
            map_display["Topic"]["Durability Policy"], "Transient Local"
        )
        cloud_display = displays["Current lidar scan"]
        self.assertEqual(
            cloud_display["Topic"]["Value"],
            "/vehicle_1/lidar/points",
        )
        self.assertEqual(
            cloud_display["Topic"]["Reliability Policy"], "Best Effort"
        )
        self.assertEqual(
            displays["Reference vehicle pose"]["Reference Frame"],
            "vehicle_1/base_link/front_lidar",
        )
        current_view = manager["Views"]["Current"]
        self.assertEqual(
            current_view["Target Frame"],
            "vehicle_1/base_link/front_lidar",
        )
        self.assertEqual(current_view["X"], 0)
        self.assertEqual(current_view["Y"], 0)

        scenario_source = scenario_path.read_text(encoding="utf-8")
        scenario_tree = ast.parse(scenario_source)
        constants = {}
        for statement in scenario_tree.body:
            if (
                isinstance(statement, ast.Assign)
                and len(statement.targets) == 1
                and isinstance(statement.targets[0], ast.Name)
            ):
                try:
                    constants[statement.targets[0].id] = ast.literal_eval(
                        statement.value
                    )
                except (TypeError, ValueError):
                    pass
        self.assertNotIn("MOTION_DURATION_NS", constants)
        self.assertEqual(constants["LOOP_START_NS"], 15_000_000_000)
        self.assertEqual(constants["LOOP_YAW_RATE_RAD_S"], 0.25)
        self.assertEqual(
            constants["LINEAR_SPEEDS_MPS"],
            {1: 0.30, 2: 0.30, 3: 0.25},
        )
        self.assertIn(
            "motion_elapsed_ns >= LOOP_START_NS", scenario_source
        )
        self.assertIn(
            "command.angular.z = LOOP_YAW_RATE_RAD_S", scenario_source
        )
        self._assert_cyclic_trajectory_is_safe(constants)
        launch_source = launch_path.read_text(encoding="utf-8")
        ast.parse(launch_source)
        self.assertIn('"--render-engine-gui ogre "', launch_source)
        self.assertGreaterEqual(
            launch_source.count('"--render-engine-server ogre2 "'),
            1,
        )
        self.assertIn(
            'if_value="-s --render-engine-server ogre2 "', launch_source
        )
        self.assertIn(
            '"--render-engine-gui ogre "\n'
            '                        "--render-engine-server ogre2 "',
            launch_source,
        )
        self.assertIn('default_value="4.0"', launch_source)
        self.assertIn(
            '"enable_lidar",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"enable_camera",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"enable_lidar_frontend",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"enable_lidar_slam",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"enable_slam_rviz",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"enable_visual_frontend",\n            default_value="false"',
            launch_source,
        )
        self.assertIn(
            '"software_rendering",\n            default_value="false"',
            launch_source,
        )
        self.assertIn('package="rtabmap_slam"', launch_source)
        self.assertIn('package="rviz2"', launch_source)
        self.assertIn(
            'additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"}',
            launch_source,
        )
        self.assertIn('"--gui-config "', launch_source)
        self.assertIn('"rviz", "vehicle_1_lidar_slam.rviz"', launch_source)
        self.assertIn('"max_range_m": 40.0', launch_source)
        self.assertNotIn("--delete_db_on_start", launch_source)
        self.assertIn('"lidar_slam_database_directory"', launch_source)
        for node_id in (1, 2, 3):
            self.assertIn(
                f'"/vehicle_{node_id}/lidar_slam/map"', launch_source
            )
        self.assertIn(
            'condition=IfCondition(enable_visual_frontend)', launch_source
        )
        self.assertIn('"config", "lidar_bridge.yaml"', launch_source)
        self.assertIn('"config", "camera_bridge.yaml"', launch_source)
        ast.parse(launch_test_path.read_text(encoding="utf-8"))
        ast.parse(slam_test_path.read_text(encoding="utf-8"))

    def _assert_perception_sensors(self, model):
        lidar = model.find(".//sensor[@name='front_lidar'][@type='gpu_lidar']")
        self.assertIsNotNone(lidar)
        self.assertEqual(lidar.findtext("update_rate"), "5")
        self.assertEqual(lidar.findtext("always_on"), "false")
        self.assertIsNone(lidar.find("ignition_frame_id"))
        self.assertEqual(
            lidar.findtext("lidar/scan/horizontal/samples"), "360"
        )
        self.assertEqual(
            lidar.findtext("lidar/scan/vertical/samples"), "1"
        )
        self.assertEqual(lidar.findtext("lidar/range/max"), "40.0")

        camera = model.find(".//sensor[@name='front_camera'][@type='camera']")
        self.assertIsNotNone(camera)
        self.assertEqual(camera.findtext("update_rate"), "10")
        self.assertEqual(camera.findtext("always_on"), "false")
        self.assertIsNone(camera.find("ignition_frame_id"))
        self.assertEqual(camera.findtext("camera/image/width"), "320")
        self.assertEqual(camera.findtext("camera/image/height"), "240")

    def _assert_cyclic_trajectory_is_safe(self, constants):
        positions = {
            1: [0.0, 0.0, 0.0],
            2: [4.0, 0.0, 0.0],
            3: [0.0, 3.0, 0.5 * math.pi],
        }
        linear_velocities = {node_id: 0.0 for node_id in positions}
        angular_velocities = {node_id: 0.0 for node_id in positions}
        speeds = constants["LINEAR_SPEEDS_MPS"]
        loop_start_s = constants["LOOP_START_NS"] * 1.0e-9
        loop_rate = constants["LOOP_YAW_RATE_RAD_S"]
        s_curve_duration_s = constants["S_CURVE_DURATION_NS"] * 1.0e-9
        s_curve_peak_rate = constants["S_CURVE_PEAK_YAW_RATE_RAD_S"]
        self.assertAlmostEqual(speeds[1] / loop_rate, 1.20)
        self.assertAlmostEqual(speeds[2] / loop_rate, 1.20)
        self.assertAlmostEqual(speeds[3] / loop_rate, 1.00)
        vehicle_radius_m = 0.525
        obstacles = (
            (8.5, 4.8, (0.35 ** 2 + 0.60 ** 2) ** 0.5),
            (1.8, 8.2, (0.70 ** 2 + 0.25 ** 2) ** 0.5),
        )
        dt_s = 0.01
        for step in range(int(300.0 / dt_s) + 1):
            elapsed_s = step * dt_s
            for node_id, position in positions.items():
                target_yaw_rate = 0.0
                if node_id == 2 and elapsed_s < s_curve_duration_s:
                    target_yaw_rate = s_curve_peak_rate * math.sin(
                        2.0 * math.pi * elapsed_s / s_curve_duration_s
                    )
                elif elapsed_s >= loop_start_s:
                    target_yaw_rate = loop_rate
                linear_velocities[node_id] += max(
                    -0.30 * dt_s,
                    min(
                        0.30 * dt_s,
                        speeds[node_id] - linear_velocities[node_id],
                    ),
                )
                angular_velocities[node_id] += max(
                    -0.50 * dt_s,
                    min(
                        0.50 * dt_s,
                        target_yaw_rate - angular_velocities[node_id],
                    ),
                )
                position[2] += angular_velocities[node_id] * dt_s
                position[0] += (
                    linear_velocities[node_id]
                    * math.cos(position[2]) * dt_s
                )
                position[1] += (
                    linear_velocities[node_id]
                    * math.sin(position[2]) * dt_s
                )
                self.assertGreater(position[0] - vehicle_radius_m, -2.625)
                self.assertLess(position[0] + vehicle_radius_m, 26.625)
                self.assertGreater(position[1] - vehicle_radius_m, -2.625)
                self.assertLess(position[1] + vehicle_radius_m, 24.625)
                for obstacle_x, obstacle_y, obstacle_radius in obstacles:
                    clearance = (
                        (position[0] - obstacle_x) ** 2
                        + (position[1] - obstacle_y) ** 2
                    ) ** 0.5 - vehicle_radius_m - obstacle_radius
                    self.assertGreater(clearance, 0.50)
            for left, right in ((1, 2), (1, 3), (2, 3)):
                separation = (
                    (positions[left][0] - positions[right][0]) ** 2
                    + (positions[left][1] - positions[right][1]) ** 2
                ) ** 0.5
                self.assertGreater(separation, 1.20)

    def _assert_bridge_contract(self, entries, expected):
        self.assertIsInstance(entries, list)
        self.assertEqual(len(entries), len(expected))
        by_topic = {entry["ros_topic_name"]: entry for entry in entries}
        self.assertEqual(set(by_topic), set(expected))
        self.assertEqual(
            len({entry["gz_topic_name"] for entry in entries}),
            len(entries),
        )
        for topic, (ros_type, gz_type) in expected.items():
            entry = by_topic[topic]
            self.assertTrue(entry["gz_topic_name"].startswith("/"))
            self.assertEqual(entry["ros_type_name"], ros_type)
            self.assertEqual(entry["gz_type_name"], gz_type)
            self.assertEqual(entry["direction"], "GZ_TO_ROS")


if __name__ == "__main__":
    unittest.main()

"""Static contract for the optional Gazebo Fortress demo assets."""

import ast
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
        warehouse_rack_path = (
            PACKAGE_ROOT / "models/zju_warehouse_rack/model.sdf"
        )
        loading_pallet_path = (
            PACKAGE_ROOT / "models/zju_loading_pallet/model.sdf"
        )
        warehouse_frame_path = (
            PACKAGE_ROOT / "models/zju_warehouse_frame/model.sdf"
        )
        warehouse_shell_path = (
            PACKAGE_ROOT / "models/zju_warehouse_shell/model.sdf"
        )
        conveyor_path = PACKAGE_ROOT / "models/zju_conveyor/model.sdf"
        safety_barrier_path = (
            PACKAGE_ROOT / "models/zju_safety_barrier/model.sdf"
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
            warehouse_rack_path,
            loading_pallet_path,
            warehouse_frame_path,
            warehouse_shell_path,
            conveyor_path,
            safety_barrier_path,
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
        self.assertEqual({
            name: uri for name, uri in included.items()
            if name.startswith("vehicle_")
        }, {
            "vehicle_1": "model://zju_reference_vehicle",
            "vehicle_2": "model://zju_diff_vehicle",
            "vehicle_3": "model://zju_diff_vehicle",
        })
        self.assertEqual(
            included["rack_row_a"], "model://zju_warehouse_rack"
        )
        self.assertEqual(
            included["loading_pallet_west"], "model://zju_loading_pallet"
        )
        self.assertEqual(
            included["warehouse_frame_back"], "model://zju_warehouse_frame"
        )
        self.assertEqual(
            included["warehouse_shell"], "model://zju_warehouse_shell"
        )
        self.assertEqual(
            included["conveyor_station"], "model://zju_conveyor"
        )
        self.assertEqual(
            included["safety_barrier_east"], "model://zju_safety_barrier"
        )
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
        self.assertNotIn('filename="GridConfig"', gazebo_gui_source)

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
        self.assertEqual(
            constants["LINEAR_SPEEDS_MPS"],
            {1: 0.75, 2: 0.80, 3: 0.72},
        )
        self.assertEqual(constants["VEHICLE_AVOIDANCE_RADIUS_M"], 2.2)
        self.assertEqual(constants["VEHICLE_MIN_CLEARANCE_M"], 1.15)
        self.assertGreaterEqual(len(constants["STATIC_OBSTACLES"]), 10)
        self.assertIn("_avoid_static_obstacles", scenario_source)
        self.assertIn("_avoid_other_vehicles", scenario_source)
        waypoints = constants["WAYPOINTS"]
        self.assertEqual(set(waypoints), {1, 2, 3})
        for node_id, route in waypoints.items():
            self.assertGreaterEqual(len(route), 8)
            self.assertEqual(route[0][:2], {
                1: (0.0, 0.0),
                2: (4.0, 0.0),
                3: (0.0, 3.0),
            }[node_id])
            for x, y in route:
                self.assertGreater(x, -2.0)
                self.assertLess(x, 26.0)
                self.assertGreater(y, -2.0)
                self.assertLess(y, 24.0)
        # The routes must visibly enter the rack aisles/loading area rather
        # than remain in the open starting area.
        self.assertIn((9.5, 16.5), waypoints[1])
        self.assertIn((19.3, 17.0), waypoints[2])
        self.assertIn((14.5, 13.0), waypoints[3])
        launch_source = launch_path.read_text(encoding="utf-8")
        ast.parse(launch_source)
        self.assertIn('"--render-engine-gui ogre2 "', launch_source)
        self.assertGreaterEqual(
            launch_source.count('"--render-engine-server ogre2 "'),
            1,
        )
        self.assertIn(
            'if_value="-s --render-engine-server ogre2 "', launch_source
        )
        self.assertIn(
            '"--render-engine-gui ogre2 "\n'
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
            '"software_rendering",\n            default_value="true"',
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

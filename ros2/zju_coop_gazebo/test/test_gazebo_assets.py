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
        world_path = PACKAGE_ROOT / "worlds/three_vehicle.sdf"
        bridge_path = PACKAGE_ROOT / "config/bridge.yaml"
        scenario_path = PACKAGE_ROOT / "scripts/gazebo_scenario.py"
        launch_path = PACKAGE_ROOT / "launch/three_vehicle_gazebo.launch.py"
        launch_test_path = (
            PACKAGE_ROOT / "test/test_three_vehicle_gazebo.launch.py"
        )

        paths = (
            model_path,
            reference_model_path,
            world_path,
            bridge_path,
            scenario_path,
            launch_path,
            launch_test_path,
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
        self.assertEqual(diff_drive.findtext("max_linear_acceleration"), "0.30")
        self.assertEqual(diff_drive.findtext("min_linear_acceleration"), "-0.30")
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

        ast.parse(scenario_path.read_text(encoding="utf-8"))
        launch_source = launch_path.read_text(encoding="utf-8")
        ast.parse(launch_source)
        self.assertIn('else_value="--render-engine-gui ogre "', launch_source)
        self.assertIn('default_value="4.0"', launch_source)
        ast.parse(launch_test_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()

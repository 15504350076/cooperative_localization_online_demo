#!/usr/bin/env python3
"""Publish three AirSim car poses as ENU ROS 2 Odometry.

AirSim is NED and exposes its state through RPC.  This small adapter converts
the car pose to ENU and publishes only the common Odometry contract; the
existing simulator UWB adapter then generates ``/uwb/range``.
"""

import importlib
import math
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def _stamp(timestamp_ns, fallback):
    if timestamp_ns > 0:
        return fallback.__class__(
            seconds=int(timestamp_ns // 1_000_000_000),
            nanoseconds=int(timestamp_ns % 1_000_000_000),
        ).to_msg()
    return fallback.to_msg()


class AirsimOdometryBridge(Node):
    """Poll AirSim car RPC state and publish one Odometry topic per car."""

    def __init__(self):
        super().__init__("zju_airsim_odometry_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 41461)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("vehicle_names", "Car_1,Car_2,Car_3")
        self.declare_parameter("initial_positions_enu", "0,0;12,0;0,12")
        self.declare_parameter("airsim_python_path", "")

        python_path = str(self.get_parameter("airsim_python_path").value)
        if python_path:
            if python_path not in sys.path:
                sys.path.insert(0, python_path)
            # AirSimAG keeps msgpack-rpc in its repository-local venv.  Make
            # the ROS system Python able to reuse that existing dependency.
            site_packages = Path(python_path).parent / ".venv/lib/python3.10/site-packages"
            if site_packages.is_dir() and str(site_packages) not in sys.path:
                sys.path.insert(0, str(site_packages))
        try:
            airsim = importlib.import_module("airsim")
        except ModuleNotFoundError as error:
            raise RuntimeError(
                f"AirSim Python客户端或依赖不可用：{error}；设置airsim_python_path"
                "为AirSimAG/PythonClient"
            ) from error

        self._airsim = airsim
        self._names = tuple(
            name.strip()
            for name in str(self.get_parameter("vehicle_names").value).split(",")
            if name.strip()
        )
        if not self._names:
            raise RuntimeError("vehicle_names不能为空")
        self._initial_poses = {}
        self._client = airsim.CarClient(
            ip=str(self.get_parameter("host").value),
            port=int(self.get_parameter("port").value),
            timeout_value=5,
        )
        self._client.confirmConnection()
        available = set(self._client.listVehicles())
        missing = [name for name in self._names if name not in available]
        if missing:
            raise RuntimeError(
                f"AirSim未找到车辆{missing}；当前车辆={sorted(available)}"
            )
        self._set_initial_positions()

        self._publishers = {
            name: self.create_publisher(
                Odometry, f"/airsim/{name}/odometry", qos_profile_sensor_data
            )
            for name in self._names
        }
        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            "AirSim odometry ready: vehicles="
            f"{','.join(self._names)}, output=/airsim/<vehicle>/odometry, "
            "frame=common_enu"
        )

    def _set_initial_positions(self):
        """Place the three cars in a common ENU formation.

        Some Blocks maps already contain vehicle pawns at the player start and
        ignore the per-vehicle X/Y settings.  Setting the pose through RPC
        makes the formation deterministic while retaining the map's visuals.
        """
        text = str(self.get_parameter("initial_positions_enu").value).strip()
        if not text:
            return
        try:
            values = []
            for item in text.split(";"):
                x_enu, y_enu = (float(part.strip()) for part in item.split(","))
                values.append((x_enu, y_enu))
            if len(values) != len(self._names):
                raise ValueError("位置数量与vehicle_names不一致")
            for name, (x_enu, y_enu) in zip(self._names, values):
                # AirSim uses NED; ENU x/y maps to NED y/x.  ENU yaw=0 is
                # represented by a NED yaw of +90 degrees.
                pose = self._airsim.Pose(
                    self._airsim.Vector3r(y_enu, x_enu, 0.0),
                    self._airsim.to_quaternion(0.0, 0.0, math.pi / 2.0),
                )
                self._initial_poses[name] = pose
                self._client.simSetVehiclePose(pose, True, name)
                # PhysX cars otherwise keep the map's previous controls after
                # a teleport and may roll/fall before a controller takes over.
                self._client.setCarControls(
                    self._airsim.CarControls(brake=1.0, handbrake=True), name
                )
        except (TypeError, ValueError) as error:
            raise RuntimeError(
                "initial_positions_enu应为x,y;x,y;x,y，例如0,0;12,0;0,12"
            ) from error

    def _publish(self):
        now = self.get_clock().now()
        for name in self._names:
            state = self._client.getCarState(vehicle_name=name)
            # getCarState().kinematics_estimated is local to each car's home
            # frame.  The cooperative UWB adapter needs one common world
            # frame, so use simGetVehiclePose for position and attitude.
            world_pose = self._client.simGetVehiclePose(vehicle_name=name)
            kinematics = state.kinematics_estimated
            position = world_pose.position
            orientation = world_pose.orientation
            roll, pitch, yaw_ned = self._airsim.to_eularian_angles(orientation)
            yaw_enu = math.pi / 2.0 - yaw_ned

            message = Odometry()
            message.header.stamp = _stamp(int(state.timestamp), now)
            message.header.frame_id = "common_enu"
            message.child_frame_id = f"{name}/base_link"
            message.pose.pose.position.x = float(position.y_val)
            message.pose.pose.position.y = float(position.x_val)
            message.pose.pose.position.z = float(-position.z_val)
            message.pose.pose.orientation.w = math.cos(yaw_enu / 2.0)
            message.pose.pose.orientation.z = math.sin(yaw_enu / 2.0)
            message.twist.twist.linear.x = float(kinematics.linear_velocity.y_val)
            message.twist.twist.linear.y = float(kinematics.linear_velocity.x_val)
            message.twist.twist.linear.z = float(-kinematics.linear_velocity.z_val)
            message.twist.twist.angular.x = float(kinematics.angular_velocity.y_val)
            message.twist.twist.angular.y = float(kinematics.angular_velocity.x_val)
            message.twist.twist.angular.z = float(-kinematics.angular_velocity.z_val)
            self._publishers[name].publish(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AirsimOdometryBridge()
    except Exception as error:
        print(f"AirSim里程计适配启动失败：{error}", file=sys.stderr)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

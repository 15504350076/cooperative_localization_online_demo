#!/usr/bin/env python3
"""Drive three Gazebo vehicles and derive ideal UWB/truth from world poses."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from cooperative_localization_msgs.msg import (
    CooperativePose2DArray,
    VehiclePose2D,
)
from zju_coop_test_msgs.msg import UwbRange


NODE_IDS = (1, 2, 3)
UWB_EDGES = ((1, 2), (1, 3), (2, 3))
STATIONARY_DURATION_NS = 1_000_000_000
S_CURVE_DURATION_NS = 8_000_000_000
LOOP_START_NS = 15_000_000_000
S_CURVE_PEAK_YAW_RATE_RAD_S = 0.15
LOOP_YAW_RATE_RAD_S = 0.25
LINEAR_SPEEDS_MPS = {
    1: 0.30,
    2: 0.30,
    3: 0.25,
}


def _stamp_ns(stamp):
    if stamp.sec < 0 or stamp.nanosec >= 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _node_id_from_model_frame(frame_id):
    normalized = frame_id.replace("::", "/").strip("/")
    parts = [part for part in normalized.split("/") if part]
    if not parts:
        return None
    for node_id in NODE_IDS:
        if parts[-1] == f"vehicle_{node_id}":
            return node_id
    return None


def _yaw_from_xyzw(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("invalid Gazebo pose quaternion")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


class GazeboScenario(Node):
    def __init__(self):
        super().__init__("zju_gazebo_scenario")
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._command_publishers = {
            node_id: self.create_publisher(
                Twist, f"/vehicle_{node_id}/cmd_vel", 1
            )
            for node_id in NODE_IDS
        }
        self._uwb_publisher = self.create_publisher(
            UwbRange, "/uwb/range", sensor_qos
        )
        self._truth_publisher = self.create_publisher(
            CooperativePose2DArray,
            "/simulation/ground_truth/poses_2d",
            result_qos,
        )
        self._pose_subscriptions = [
            self.create_subscription(
                TFMessage,
                f"/simulation/gazebo/vehicle_{node_id}/pose",
                lambda message, source=node_id: self._on_pose(source, message),
                sensor_qos,
            )
            for node_id in NODE_IDS
        ]
        self._poses_by_stamp = {}
        self._latest_snapshot = None
        self._first_snapshot_stamp_ns = None
        self._last_published_stamp_ns = 0
        self._timer = self.create_timer(0.05, self._tick)

    def _on_pose(self, node_id, message):
        for transform in message.transforms:
            source = _node_id_from_model_frame(transform.child_frame_id)
            if source != node_id:
                continue
            translation = transform.transform.translation
            values = (translation.x, translation.y, translation.z)
            if not all(math.isfinite(value) for value in values):
                continue
            try:
                yaw = _yaw_from_xyzw(transform.transform.rotation)
            except ValueError:
                continue
            candidate_stamp_ns = _stamp_ns(transform.header.stamp)
            if candidate_stamp_ns <= 0:
                continue
            bucket = self._poses_by_stamp.setdefault(
                candidate_stamp_ns,
                {"stamp": transform.header.stamp, "poses": {}},
            )
            bucket["poses"][node_id] = (values, yaw)
            if set(bucket["poses"]) == set(NODE_IDS):
                self._latest_snapshot = (
                    candidate_stamp_ns,
                    bucket["stamp"],
                    dict(bucket["poses"]),
                )
            if len(self._poses_by_stamp) > 20:
                for old_stamp in sorted(self._poses_by_stamp)[:-20]:
                    del self._poses_by_stamp[old_stamp]
            return

    def _publish_commands(self, motion_elapsed_ns):
        for node_id, linear_speed in LINEAR_SPEEDS_MPS.items():
            command = Twist()
            if motion_elapsed_ns is not None:
                command.linear.x = linear_speed
                if node_id == 2 and motion_elapsed_ns < S_CURVE_DURATION_NS:
                    phase = (
                        2.0 * math.pi * motion_elapsed_ns
                        / S_CURVE_DURATION_NS
                    )
                    command.angular.z = (
                        S_CURVE_PEAK_YAW_RATE_RAD_S * math.sin(phase)
                    )
                elif motion_elapsed_ns >= LOOP_START_NS:
                    command.angular.z = LOOP_YAW_RATE_RAD_S
            self._command_publishers[node_id].publish(command)

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return

        snapshot = self._latest_snapshot
        if snapshot is None:
            self._publish_commands(None)
            return
        stamp_ns, stamp, poses = snapshot
        if self._first_snapshot_stamp_ns is None:
            self._first_snapshot_stamp_ns = stamp_ns
        elapsed_ns = stamp_ns - self._first_snapshot_stamp_ns
        motion_elapsed_ns = (
            None
            if elapsed_ns < STATIONARY_DURATION_NS
            else elapsed_ns - STATIONARY_DURATION_NS
        )
        self._publish_commands(motion_elapsed_ns)
        if stamp_ns <= self._last_published_stamp_ns:
            return

        for source, target in UWB_EDGES:
            source_position = poses[source][0]
            target_position = poses[target][0]
            difference = tuple(
                source_position[index] - target_position[index]
                for index in range(3)
            )
            distance = math.sqrt(sum(value * value for value in difference))
            message = UwbRange()
            message.header.stamp = stamp
            message.header.frame_id = "common_enu"
            message.src_id = source
            message.target_id = target
            message.distance = float(distance)
            self._uwb_publisher.publish(message)

        reference = poses[1][0]
        truth = CooperativePose2DArray()
        truth.header.stamp = stamp
        truth.header.frame_id = "coop_ref_1_enu"
        truth.reference_node_id = 1
        for node_id in NODE_IDS:
            position, yaw = poses[node_id]
            vehicle = VehiclePose2D()
            vehicle.node_id = node_id
            vehicle.x_m = position[0] - reference[0]
            vehicle.y_m = position[1] - reference[1]
            vehicle.yaw_rad = yaw
            vehicle.position_valid = True
            vehicle.yaw_valid = True
            truth.vehicles.append(vehicle)
        self._truth_publisher.publish(truth)
        self._last_published_stamp_ns = stamp_ns


def main(args=None):
    rclpy.init(args=args)
    node = GazeboScenario()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # ROS 2 Humble can race a subscription take with context shutdown.
        # Preserve real conversion failures while allowing a clean stop.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

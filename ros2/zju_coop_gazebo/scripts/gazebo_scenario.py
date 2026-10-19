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
from cooperative_interfaces.msg import UwbRange


NODE_IDS = (1, 2, 3)
UWB_EDGES = ((1, 2), (1, 3), (2, 3))
STATIONARY_DURATION_NS = 1_000_000_000
# The routes deliberately pass through the warehouse aisles and loading
# area.  They are closed loops, so a long run repeatedly exercises the same
# obstacle geometry instead of leaving all cars in the open starting area.
WAYPOINTS = {
    1: (
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 9.0),
        (9.5, 9.0),
        (9.5, 16.5),
        (9.5, 18.0),
        (4.5, 18.0),
        (4.5, 11.0),
        (0.0, 11.0),
    ),
    2: (
        (4.0, 0.0),
        (18.0, 0.0),
        (18.0, 7.0),
        (24.0, 7.0),
        (24.0, 11.0),
        (19.3, 11.0),
        (19.3, 17.0),
        (22.8, 16.0),
        (24.0, 8.0),
        (18.0, 5.0),
        (4.0, 5.0),
    ),
    3: (
        (0.0, 3.0),
        (4.0, 3.0),
        (4.0, 9.0),
        (9.5, 9.0),
        (9.5, 13.0),
        (9.5, 18.0),
        (14.5, 18.0),
        (14.5, 13.0),
        (19.3, 13.0),
        (19.3, 18.0),
        (24.0, 18.0),
        (24.0, 5.0),
        (0.0, 5.0),
    ),
}
WAYPOINT_REACHED_M = 0.65
MAX_ANGULAR_SPEED_RAD_S = 0.85
HEADING_GAIN = 1.6
# The body is about 0.9 m across diagonally.  Start braking well before
# Gazebo's collision geometry can overlap.
VEHICLE_AVOIDANCE_RADIUS_M = 3.6
VEHICLE_MIN_CLEARANCE_M = 1.80
VEHICLE_CONTACT_DISTANCE_M = 0.95
STATIC_CLEARANCE_M = 0.85
# Approximate 2-D footprints of the static SDF objects.  This is a small
# local safety layer for the scripted test route; it is not a replacement for
# a production LiDAR planner.
STATIC_OBSTACLES = (
    (8.5, 4.8, 1.05),
    (1.8, 8.2, 0.90),
    (7.0, 14.0, 1.85),
    (12.0, 14.0, 1.85),
    (17.0, 14.0, 1.85),
    (7.0, 20.2, 1.85),
    (12.0, 20.2, 1.85),
    (17.0, 20.2, 1.85),
    (4.8, 10.8, 1.05),
    (22.5, 10.8, 1.05),
    (22.5, 18.0, 1.05),
    (20.0, 8.0, 2.85),
    (22.0, 6.2, 1.30),
    (7.5, 10.6, 1.20),
)
LINEAR_SPEEDS_MPS = {
    1: 0.75,
    2: 0.80,
    3: 0.72,
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
        self._waypoint_index = {node_id: 0 for node_id in NODE_IDS}
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

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _blend_angles(first, second, weight):
        return math.atan2(
            (1.0 - weight) * math.sin(first) + weight * math.sin(second),
            (1.0 - weight) * math.cos(first) + weight * math.cos(second),
        )

    def _avoid_static_obstacles(self, position, desired_yaw, speed):
        repulse_x = 0.0
        repulse_y = 0.0
        strongest = 0.0
        for obstacle_x, obstacle_y, obstacle_radius in STATIC_OBSTACLES:
            dx = position[0] - obstacle_x
            dy = position[1] - obstacle_y
            distance = math.hypot(dx, dy)
            influence = obstacle_radius + STATIC_CLEARANCE_M
            if distance >= influence:
                continue
            if distance < 1.0e-6:
                dx, dy, distance = 1.0, 0.0, 1.0
            weight = (influence - distance) / influence
            repulse_x += dx / distance * weight
            repulse_y += dy / distance * weight
            strongest = max(strongest, weight)
        if strongest <= 0.0:
            return desired_yaw, speed
        repulse_yaw = math.atan2(repulse_y, repulse_x)
        desired_yaw = self._blend_angles(
            desired_yaw, repulse_yaw, min(0.85, strongest)
        )
        speed *= max(0.08, 1.0 - 0.85 * strongest)
        return desired_yaw, speed

    def _avoid_other_vehicles(self, node_id, position, desired_yaw, speed,
                               poses):
        nearest_id = None
        nearest_distance = float("inf")
        for other_id, (other_position, _other_yaw) in poses.items():
            if other_id == node_id:
                continue
            distance = math.hypot(
                position[0] - other_position[0],
                position[1] - other_position[1],
            )
            if distance < nearest_distance:
                nearest_id = other_id
                nearest_distance = distance
        if nearest_id is None or nearest_distance >= VEHICLE_AVOIDANCE_RADIUS_M:
            return desired_yaw, speed

        other_position = poses[nearest_id][0]
        away_yaw = math.atan2(
            position[1] - other_position[1],
            position[0] - other_position[0],
        )
        weight = min(
            1.0,
            max(
                0.0,
                (VEHICLE_AVOIDANCE_RADIUS_M - nearest_distance)
                / (VEHICLE_AVOIDANCE_RADIUS_M - VEHICLE_MIN_CLEARANCE_M),
            ),
        )
        to_other_error = self._wrap_angle(
            math.atan2(
                other_position[1] - position[1],
                other_position[0] - position[0],
            ) - desired_yaw
        )
        approaching = abs(to_other_error) < 0.75 * math.pi

        if node_id < nearest_id:
            # Lower node id has priority, but it must not drive into a
            # stationary yielding vehicle.  Turn slightly away and hard-stop
            # while the other vehicle clears the envelope.
            if approaching:
                desired_yaw = self._blend_angles(
                    desired_yaw, away_yaw, 0.35 * weight
                )
                if nearest_distance <= VEHICLE_MIN_CLEARANCE_M:
                    speed = 0.0
                else:
                    speed *= max(0.25, 1.0 - 0.75 * weight)
            return desired_yaw, speed

        # Higher node id yields on a deterministic side.  At close range it
        # is explicitly allowed to move away at a crawl; setting speed to
        # zero here caused two cars to remain nose-to-nose indefinitely.
        side = 1.0 if node_id > nearest_id else -1.0
        bypass_yaw = away_yaw + side * 0.5 * math.pi
        desired_yaw = self._blend_angles(desired_yaw, bypass_yaw, weight)
        if nearest_distance <= VEHICLE_CONTACT_DISTANCE_M:
            speed = 0.0
        elif nearest_distance <= VEHICLE_MIN_CLEARANCE_M:
            speed = max(0.22, min(speed, 0.30))
        else:
            speed *= max(0.20, 1.0 - 0.80 * weight)
        return desired_yaw, speed

    def _publish_commands(self, motion_elapsed_ns, poses):
        for node_id, cruise_speed in LINEAR_SPEEDS_MPS.items():
            command = Twist()
            if motion_elapsed_ns is not None and poses is not None:
                position, yaw = poses[node_id]
                waypoints = WAYPOINTS[node_id]
                index = self._waypoint_index[node_id]
                target_x, target_y = waypoints[index]
                dx = target_x - position[0]
                dy = target_y - position[1]
                distance = math.hypot(dx, dy)
                if distance < WAYPOINT_REACHED_M:
                    index = (index + 1) % len(waypoints)
                    self._waypoint_index[node_id] = index
                    target_x, target_y = waypoints[index]
                    dx = target_x - position[0]
                    dy = target_y - position[1]
                    distance = math.hypot(dx, dy)

                desired_yaw = math.atan2(dy, dx)
                command_speed = min(
                    cruise_speed,
                    max(0.0, 0.18 + 0.35 * distance),
                )
                desired_yaw, command_speed = self._avoid_static_obstacles(
                    position, desired_yaw, command_speed
                )
                desired_yaw, command_speed = self._avoid_other_vehicles(
                    node_id, position, desired_yaw, command_speed, poses
                )
                heading_error = self._wrap_angle(desired_yaw - yaw)
                command.angular.z = max(
                    -MAX_ANGULAR_SPEED_RAD_S,
                    min(
                        MAX_ANGULAR_SPEED_RAD_S,
                        HEADING_GAIN * heading_error,
                    ),
                )
                command.linear.x = command_speed
                if abs(heading_error) > 0.75:
                    command.linear.x *= 0.35
                if abs(heading_error) > 1.45:
                    command.linear.x = 0.0
            self._command_publishers[node_id].publish(command)

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return

        snapshot = self._latest_snapshot
        if snapshot is None:
            self._publish_commands(None, None)
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
        self._publish_commands(motion_elapsed_ns, poses)
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

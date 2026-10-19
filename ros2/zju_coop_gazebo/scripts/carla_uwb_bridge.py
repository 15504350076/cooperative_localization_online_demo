#!/usr/bin/env python3
"""Convert CARLA/AWSIM vehicle odometry into project UWB and truth topics.

The simulator remains responsible for cameras, LiDAR and IMU.  This small
ROS 2 node only provides the project-specific UWB contract and a truth topic;
no CARLA Python package is required on the ROS computer.
"""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cooperative_interfaces.msg import UwbRange
from cooperative_localization_msgs.msg import CooperativePose2DArray, VehiclePose2D


NODE_IDS = (1, 2, 3)
UWB_EDGES = ((1, 2), (1, 3), (2, 3))


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class CarlaUwbBridge(Node):
    """Publish noisy pairwise UWB ranges and simulator ground truth."""

    def __init__(self):
        super().__init__("zju_carla_uwb_bridge")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.declare_parameter("odom_topic_1", "/carla/vehicle_1/odometry")
        self.declare_parameter("odom_topic_2", "/carla/vehicle_2/odometry")
        self.declare_parameter("odom_topic_3", "/carla/vehicle_3/odometry")
        self.declare_parameter("range_noise_std_m", 0.02)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("max_odom_age_ms", 200.0)
        self.declare_parameter("max_stamp_span_ms", 20.0)
        self.declare_parameter("random_seed", 7)

        self._poses = {}
        self._noise_std = float(self.get_parameter("range_noise_std_m").value)
        self._dropout = float(self.get_parameter("dropout_probability").value)
        self._max_age_ns = int(
            float(self.get_parameter("max_odom_age_ms").value) * 1e6
        )
        self._max_span_ns = int(
            float(self.get_parameter("max_stamp_span_ms").value) * 1e6
        )
        self._rng = random.Random(
            int(self.get_parameter("random_seed").value)
        )
        self._range_pub = self.create_publisher(UwbRange, "/uwb/range", qos)
        self._truth_pub = self.create_publisher(
            CooperativePose2DArray,
            "/simulation/ground_truth/poses_2d",
            1,
        )
        self._subscriptions = []
        for node_id in NODE_IDS:
            topic = str(
                self.get_parameter(f"odom_topic_{node_id}").value
            )
            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    topic,
                    lambda message, node_id=node_id: self._on_odom(
                        node_id, message
                    ),
                    qos,
                )
            )
        self._timer = self.create_timer(0.05, self._publish)

    def _on_odom(self, node_id, message):
        pose = message.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z)
        if not all(math.isfinite(value) for value in values):
            return
        self._poses[node_id] = (
            _stamp_ns(message.header.stamp),
            message.header,
            values,
            _yaw_from_quaternion(pose.orientation),
        )

    def _publish(self):
        if set(self._poses) != set(NODE_IDS):
            return
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        stamps = [self._poses[node_id][0] for node_id in NODE_IDS]
        if max(stamps) - min(stamps) > self._max_span_ns:
            return
        if any(now_ns - stamp > self._max_age_ns for stamp in stamps):
            return

        stamp = self._poses[1][1]
        positions = {node_id: self._poses[node_id][2] for node_id in NODE_IDS}
        for source, target in UWB_EDGES:
            if self._rng.random() < self._dropout:
                continue
            distance = math.dist(positions[source], positions[target])
            if self._noise_std > 0.0:
                distance += self._rng.gauss(0.0, self._noise_std)
            message = UwbRange()
            message.header = stamp
            message.header.frame_id = "common_enu"
            message.src_id = source
            message.target_id = target
            message.rolling = 0
            message.distance = max(0.0, float(distance))
            self._range_pub.publish(message)

        reference = positions[1]
        truth = CooperativePose2DArray()
        truth.header = stamp
        truth.header.frame_id = "carla_enu"
        truth.reference_node_id = 1
        for node_id in NODE_IDS:
            vehicle = VehiclePose2D()
            vehicle.node_id = node_id
            vehicle.x_m = positions[node_id][0] - reference[0]
            vehicle.y_m = positions[node_id][1] - reference[1]
            vehicle.yaw_rad = self._poses[node_id][3]
            vehicle.position_valid = True
            vehicle.yaw_valid = True
            truth.vehicles.append(vehicle)
        self._truth_pub.publish(truth)


def main(args=None):
    rclpy.init(args=args)
    node = CarlaUwbBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

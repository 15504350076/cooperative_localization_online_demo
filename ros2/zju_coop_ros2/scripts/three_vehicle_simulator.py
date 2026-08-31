#!/usr/bin/env python3
"""Short-duration analytic IMU/UWB simulator for the three-vehicle demo."""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from cooperative_localization_msgs.msg import (
    CooperativePose2DArray,
    VehiclePose2D,
)
from cooperative_interfaces.msg import UwbRange


NODE_IDS = (1, 2, 3)
UWB_EDGES = ((1, 2), (1, 3), (2, 3))
STREAM_RADIX = 256
IMU_STREAM_CODES = {1: 1, 2: 2, 3: 3}
UWB_STREAM_CODES = {(1, 2): 129, (1, 3): 130, (2, 3): 131}


def stream_seed(base_seed, stream_code):
    """Derive a stable, non-overlapping seed for one sensor stream."""
    return base_seed * STREAM_RADIX + stream_code


def noise_sample(random_stream, standard_deviation):
    """Draw one sample and advance the stream even when sigma is zero."""
    return random_stream.gauss(0.0, 1.0) * standard_deviation


def truth(node_id, elapsed_s):
    """Return ENU position, velocity, yaw and yaw rate for one vehicle."""
    if node_id == 1:
        return (
            (0.6 * elapsed_s, 0.0, 0.0),
            (0.6, 0.0, 0.0),
            0.0,
            0.0,
        )
    if node_id == 2:
        radius_m = 3.0
        yaw_rate_rad_s = 0.15
        yaw = yaw_rate_rad_s * elapsed_s
        return (
            (
                4.0 + radius_m * math.sin(yaw),
                radius_m * (1.0 - math.cos(yaw)),
                0.0,
            ),
            (
                radius_m * yaw_rate_rad_s * math.cos(yaw),
                radius_m * yaw_rate_rad_s * math.sin(yaw),
                0.0,
            ),
            yaw,
            yaw_rate_rad_s,
        )
    if node_id == 3:
        return (
            (0.0, 3.0 + 0.3 * elapsed_s, 0.0),
            (0.0, 0.3, 0.0),
            math.pi / 2.0,
            0.0,
        )
    raise ValueError(f"unknown node_id {node_id}")


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def finite_vector(value, expected_size, name):
    if len(value) != expected_size or not all(math.isfinite(item) for item in value):
        raise ValueError(f"{name} must contain {expected_size} finite values")
    return tuple(value)


class ThreeVehicleSimulator(Node):

    def __init__(self):
        super().__init__("zju_three_vehicle_simulator")
        self.imu_rate_hz = self.declare_parameter("imu_rate_hz", 100.0).value
        self.uwb_rate_hz = self.declare_parameter("uwb_rate_hz", 20.0).value
        self.truth_rate_hz = self.declare_parameter("truth_rate_hz", 20.0).value
        self.uwb_delay_s = self.declare_parameter("uwb_delay_s", 0.05).value
        self.gravity_mps2 = self.declare_parameter("gravity_mps2", 9.80665).value
        self.gyro_noise_std_rad_s = self.declare_parameter(
            "gyro_noise_std_rad_s", 0.0
        ).value
        self.accel_noise_std_m_s2 = self.declare_parameter(
            "accel_noise_std_m_s2", 0.0
        ).value
        self.uwb_noise_std_m = self.declare_parameter(
            "uwb_noise_std_m", 0.0
        ).value
        self.uwb_drop_every_n = self.declare_parameter(
            "uwb_drop_every_n", 0
        ).value
        self.uwb_nlos_every_n = self.declare_parameter(
            "uwb_nlos_every_n", 0
        ).value
        self.uwb_nlos_bias_m = self.declare_parameter(
            "uwb_nlos_bias_m", 2.0
        ).value
        random_seed = self.declare_parameter("random_seed", 20260817).value
        self.gyro_bias = finite_vector(
            self.declare_parameter("gyro_bias_flu_rad_s", [0.0] * 9).value,
            9,
            "gyro_bias_flu_rad_s",
        )
        self.accel_bias = finite_vector(
            self.declare_parameter("accel_bias_flu_m_s2", [0.0] * 9).value,
            9,
            "accel_bias_flu_m_s2",
        )
        self._validate_parameters(random_seed)
        # A fixed stream per sensor keeps a sample sequence stable even when
        # another sensor is dropped or its timer happens to run first.
        self.imu_random = {
            node_id: random.Random(
                stream_seed(random_seed, IMU_STREAM_CODES[node_id])
            )
            for node_id in NODE_IDS
        }
        self.uwb_random = {
            edge: random.Random(
                stream_seed(random_seed, UWB_STREAM_CODES[edge])
            )
            for edge in UWB_EDGES
        }

        self.imu_publishers = {
            node_id: self.create_publisher(
                Imu, f"/vehicle_{node_id}/imu/data", qos_profile_sensor_data
            )
            for node_id in NODE_IDS
        }
        self.uwb_publisher = self.create_publisher(
            UwbRange,
            "/uwb/range",
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.truth_publisher = self.create_publisher(
            CooperativePose2DArray,
            "/simulation/ground_truth/poses_2d",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.start_time_ns = None
        self.next_uwb_s = self.uwb_delay_s
        self.next_truth_s = 0.0
        self.uwb_epoch = 0
        self.timer = self.create_timer(1.0 / self.imu_rate_hz, self._tick)

    def _validate_parameters(self, random_seed):
        positive = (
            self.imu_rate_hz,
            self.uwb_rate_hz,
            self.truth_rate_hz,
            self.gravity_mps2,
        )
        nonnegative = (
            self.gyro_noise_std_rad_s,
            self.accel_noise_std_m_s2,
            self.uwb_noise_std_m,
            self.uwb_nlos_bias_m,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("simulation rates and gravity must be positive")
        if (
            self.uwb_rate_hz > self.imu_rate_hz
            or self.truth_rate_hz > self.imu_rate_hz
        ):
            raise ValueError("UWB and truth rates cannot exceed the IMU timer rate")
        if not all(math.isfinite(value) and value >= 0.0 for value in nonnegative):
            raise ValueError("simulation noise and NLOS bias must be nonnegative")
        if not math.isfinite(self.uwb_delay_s) or not 0.0 <= self.uwb_delay_s < 0.5:
            raise ValueError("uwb_delay_s must be in [0, 0.5)")
        if (
            type(self.uwb_drop_every_n) is not int
            or self.uwb_drop_every_n < 0
            or type(self.uwb_nlos_every_n) is not int
            or self.uwb_nlos_every_n < 0
            or type(random_seed) is not int
            or not 0 <= random_seed <= 2**63 - 1
        ):
            raise ValueError(
                "drop/NLOS periods must be nonnegative integers and "
                "random_seed must be an unsigned 63-bit integer"
            )

    @staticmethod
    def _set_stamp(message, timestamp_ns):
        message.header.stamp.sec = timestamp_ns // 1_000_000_000
        message.header.stamp.nanosec = timestamp_ns % 1_000_000_000

    def _graph_ready(self):
        return (
            all(
                publisher.get_subscription_count() >= 1
                for publisher in self.imu_publishers.values()
            )
            and self.uwb_publisher.get_subscription_count() >= 1
            and self.count_publishers("/cooperative_localization/node_state") >= 3
            and self.count_publishers("/cooperative_localization/poses_2d") >= 1
        )

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        if self.start_time_ns is None:
            if not self._graph_ready():
                return
            self.start_time_ns = now_ns
            self.get_logger().info(
                "three-vehicle simulation started: "
                f"IMU {self.imu_rate_hz:.1f} Hz, UWB {self.uwb_rate_hz:.1f} Hz"
            )

        elapsed_s = (now_ns - self.start_time_ns) * 1.0e-9
        self._publish_imu(now_ns, elapsed_s)

        if elapsed_s + 1.0e-9 >= self.next_truth_s:
            self._publish_truth(now_ns, elapsed_s)
            self.next_truth_s += 1.0 / self.truth_rate_hz
            if self.next_truth_s <= elapsed_s:
                self.next_truth_s = elapsed_s + 1.0 / self.truth_rate_hz

        if elapsed_s + 1.0e-9 >= self.next_uwb_s:
            measurement_s = max(0.0, elapsed_s - self.uwb_delay_s)
            measurement_ns = now_ns - int(round(self.uwb_delay_s * 1.0e9))
            self._publish_uwb(measurement_ns, measurement_s)
            self.next_uwb_s += 1.0 / self.uwb_rate_hz
            if self.next_uwb_s <= elapsed_s:
                self.next_uwb_s = elapsed_s + 1.0 / self.uwb_rate_hz

    def _publish_imu(self, timestamp_ns, elapsed_s):
        for node_index, node_id in enumerate(NODE_IDS):
            _, _, yaw, yaw_rate = truth(node_id, elapsed_s)
            random_stream = self.imu_random[node_id]
            message = Imu()
            self._set_stamp(message, timestamp_ns)
            message.header.frame_id = "imu_link"
            quaternion = yaw_quaternion(yaw)
            message.orientation.x = quaternion[0]
            message.orientation.y = quaternion[1]
            message.orientation.z = quaternion[2]
            message.orientation.w = quaternion[3]
            message.orientation_covariance[0] = -1.0

            gyro_ideal = (0.0, 0.0, yaw_rate)
            accel_ideal = (
                0.0,
                3.0 * yaw_rate * yaw_rate if node_id == 2 else 0.0,
                self.gravity_mps2,
            )
            gyro = [
                gyro_ideal[axis]
                + self.gyro_bias[3 * node_index + axis]
                + noise_sample(random_stream, self.gyro_noise_std_rad_s)
                for axis in range(3)
            ]
            accel = [
                accel_ideal[axis]
                + self.accel_bias[3 * node_index + axis]
                + noise_sample(random_stream, self.accel_noise_std_m_s2)
                for axis in range(3)
            ]
            message.angular_velocity.x = gyro[0]
            message.angular_velocity.y = gyro[1]
            message.angular_velocity.z = gyro[2]
            message.linear_acceleration.x = accel[0]
            message.linear_acceleration.y = accel[1]
            message.linear_acceleration.z = accel[2]
            gyro_variance = self.gyro_noise_std_rad_s**2
            accel_variance = self.accel_noise_std_m_s2**2
            for axis in range(3):
                message.angular_velocity_covariance[4 * axis] = gyro_variance
                message.linear_acceleration_covariance[4 * axis] = accel_variance
            self.imu_publishers[node_id].publish(message)

    def _publish_uwb(self, timestamp_ns, elapsed_s):
        self.uwb_epoch += 1
        edge_noise = {
            edge: noise_sample(self.uwb_random[edge], self.uwb_noise_std_m)
            for edge in UWB_EDGES
        }
        if (
            self.uwb_drop_every_n > 0
            and self.uwb_epoch % self.uwb_drop_every_n == 0
        ):
            return

        positions = {node_id: truth(node_id, elapsed_s)[0] for node_id in NODE_IDS}
        nlos_epoch = (
            self.uwb_nlos_every_n > 0
            and self.uwb_epoch % self.uwb_nlos_every_n == 0
        )
        for source, target in UWB_EDGES:
            delta = tuple(
                positions[target][axis] - positions[source][axis]
                for axis in range(3)
            )
            distance = math.sqrt(sum(value * value for value in delta))
            distance += edge_noise[(source, target)]
            if nlos_epoch and (source, target) == (2, 3):
                distance += self.uwb_nlos_bias_m
            message = UwbRange()
            self._set_stamp(message, timestamp_ns)
            message.header.frame_id = "common_enu"
            message.src_id = source
            message.target_id = target
            message.distance = max(distance, 1.0e-3)
            self.uwb_publisher.publish(message)

    def _publish_truth(self, timestamp_ns, elapsed_s):
        reference_position = truth(1, elapsed_s)[0]
        output = CooperativePose2DArray()
        self._set_stamp(output, timestamp_ns)
        output.header.frame_id = "coop_ref_1_enu"
        output.reference_node_id = 1
        for node_id in NODE_IDS:
            position, _, yaw, _ = truth(node_id, elapsed_s)
            vehicle = VehiclePose2D()
            vehicle.node_id = node_id
            vehicle.x_m = position[0] - reference_position[0]
            vehicle.y_m = position[1] - reference_position[1]
            vehicle.yaw_rad = math.atan2(math.sin(yaw), math.cos(yaw))
            vehicle.position_valid = True
            vehicle.yaw_valid = True
            output.vehicles.append(vehicle)
        self.truth_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThreeVehicleSimulator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        return_code = 0
    except (ValueError, TypeError) as error:
        rclpy.logging.get_logger("zju_three_vehicle_simulator").fatal(str(error))
        return_code = 1
    else:
        return_code = 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(return_code)


if __name__ == "__main__":
    main()

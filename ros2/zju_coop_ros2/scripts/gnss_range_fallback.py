#!/usr/bin/env python3
"""Explicitly enabled SJTU GnssPosition-to-range fallback."""

from itertools import combinations
import math
import time

import rclpy
from cooperative_interfaces.msg import GnssPosition, UwbRange
from cooperative_localization_msgs.msg import NodeState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


_WGS84_SEMI_MAJOR_M = 6378137.0
_WGS84_ECCENTRICITY_SQUARED = 6.6943799901413165e-3
_NANOSECONDS_PER_MILLISECOND = 1_000_000
_NANOSECONDS_PER_SECOND = 1_000_000_000


def _geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m):
    """Convert WGS-84 geodetic coordinates to ECEF metres."""
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = _WGS84_SEMI_MAJOR_M / math.sqrt(
        1.0 - _WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
    )
    return (
        (radius + altitude_m) * cos_latitude * math.cos(longitude),
        (radius + altitude_m) * cos_latitude * math.sin(longitude),
        (radius * (1.0 - _WGS84_ECCENTRICITY_SQUARED) + altitude_m)
        * sin_latitude,
    )


class GnssRangeFallbackNode(Node):
    """Publish one three-edge range epoch from each synchronized GNSS trio."""

    def __init__(self):
        super().__init__("zju_gnss_range_fallback_node")
        self._node_ids = tuple(
            int(value)
            for value in self.declare_parameter("node_ids", [1, 2, 3]).value
        )
        self._common_frame_id = str(
            self.declare_parameter("common_frame_id", "common_enu").value
        )
        self._max_stamp_skew_ns = self._positive_milliseconds(
            "max_stamp_skew_ms", 50
        )
        self._common_time_timeout_ns = self._positive_milliseconds(
            "common_time_timeout_ms", 500
        )
        # This is latest asynchronous sample spread, not UWB clock accuracy.
        self._max_state_stamp_spread_ns = self._positive_milliseconds(
            "max_state_stamp_spread_ms", 100
        )
        self._fix_timeout_ns = self._positive_milliseconds(
            "fix_timeout_ms", 500
        )
        self._max_receive_delay_ns = self._positive_milliseconds(
            "max_receive_delay_ms", 500
        )
        self._max_future_skew_ns = self._positive_milliseconds(
            "max_future_skew_ms", 100
        )
        self._max_range_m = float(
            self.declare_parameter("max_range_m", 200.0).value
        )
        self._use_altitude = bool(
            self.declare_parameter("use_altitude", False).value
        )
        if (
            len(self._node_ids) != 3
            or len(set(self._node_ids)) != 3
            or any(node_id <= 0 or node_id > 65535 for node_id in self._node_ids)
            or not self._common_frame_id
            or not math.isfinite(self._max_range_m)
            or self._max_range_m <= 0.0
        ):
            raise ValueError("invalid GNSS range fallback parameters")

        self._latest = {}
        self._common_time = {}
        self._last_common_stamp = {node_id: 0 for node_id in self._node_ids}
        self._last_input_stamp = {node_id: 0 for node_id in self._node_ids}
        self._last_emitted_stamp = {node_id: 0 for node_id in self._node_ids}
        self._fix_subscription = self.create_subscription(
            GnssPosition, "fix", self._on_fix, qos_profile_sensor_data
        )
        state_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._state_subscription = self.create_subscription(
            NodeState, "node_state", self._on_node_state, state_qos
        )
        range_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(UwbRange, "range", range_qos)
        self.get_logger().warning(
            "GNSS-derived range fallback ACTIVE: output is not UWB and must "
            "not be used for UWB timing or accuracy acceptance; GnssPosition "
            "header stamps must already use UWB_SYSTEM_TIME; use_sim_time "
            "selects /clock, otherwise live NodeState advances with steady time"
        )

    def _positive_milliseconds(self, name, default):
        value = int(self.declare_parameter(name, default).value)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value * _NANOSECONDS_PER_MILLISECOND

    @staticmethod
    def _stamp_ns(message):
        if (
            message.header.stamp.sec < 0
            or message.header.stamp.nanosec >= _NANOSECONDS_PER_SECOND
        ):
            return 0
        return (
            message.header.stamp.sec * _NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec
        )

    def _valid_fix(self, message, stamp_ns):
        altitude_valid = math.isfinite(message.altitude) or not self._use_altitude
        return (
            stamp_ns > 0
            and message.status >= GnssPosition.STATUS_FIX
            and math.isfinite(message.latitude)
            and -90.0 <= message.latitude <= 90.0
            and math.isfinite(message.longitude)
            and -180.0 <= message.longitude <= 180.0
            and altitude_valid
        )

    def _on_node_state(self, message):
        if message.node_id not in self._node_ids:
            return
        stamp_ns = self._stamp_ns(message)
        if stamp_ns <= self._last_common_stamp[message.node_id]:
            return
        self._last_common_stamp[message.node_id] = stamp_ns
        if not message.valid or message.header.frame_id != self._common_frame_id:
            self._common_time.pop(message.node_id, None)
            self._latest.clear()
            return
        self._common_time[message.node_id] = (stamp_ns, time.monotonic_ns())

    def _common_now_ns(self):
        if len(self._common_time) != len(self._node_ids):
            return None
        receive_now_ns = time.monotonic_ns()
        states = [self._common_time[node_id] for node_id in self._node_ids]
        if any(
            receive_now_ns < received_ns
            or receive_now_ns - received_ns > self._common_time_timeout_ns
            for _, received_ns in states
        ):
            return None
        stamps = [stamp_ns for stamp_ns, _ in states]
        if max(stamps) - min(stamps) > self._max_state_stamp_spread_ns:
            return None
        if bool(self.get_parameter("use_sim_time").value):
            ros_now_ns = self.get_clock().now().nanoseconds
            return ros_now_ns if ros_now_ns > 0 else None
        return max(
            stamp_ns + receive_now_ns - received_ns
            for stamp_ns, received_ns in states
        )

    def _on_fix(self, message):
        node_id = int(message.node_id)
        if node_id not in self._node_ids:
            return
        stamp_ns = self._stamp_ns(message)
        if stamp_ns <= self._last_input_stamp[node_id]:
            return
        if not self._valid_fix(message, stamp_ns):
            if stamp_ns > 0:
                self._last_input_stamp[node_id] = stamp_ns
                self._latest.pop(node_id, None)
            return

        now_ns = self._common_now_ns()
        if now_ns is None:
            self._latest.clear()
            return
        too_future = stamp_ns > now_ns + self._max_future_skew_ns
        too_delayed = now_ns > stamp_ns + self._max_receive_delay_ns
        if now_ns <= 0 or too_future or too_delayed:
            self._latest.pop(node_id, None)
            if not too_future:
                self._last_input_stamp[node_id] = stamp_ns
            return

        altitude_m = message.altitude if self._use_altitude else 0.0
        position_ecef_m = _geodetic_to_ecef(
            message.latitude, message.longitude, altitude_m
        )
        if not all(math.isfinite(value) for value in position_ecef_m):
            self._last_input_stamp[node_id] = stamp_ns
            self._latest.pop(node_id, None)
            return
        self._last_input_stamp[node_id] = stamp_ns
        self._latest[node_id] = (stamp_ns, time.monotonic_ns(), position_ecef_m)
        self._publish_if_ready()

    def _publish_if_ready(self):
        if len(self._latest) != len(self._node_ids):
            return
        if any(
            self._latest[node_id][0] <= self._last_emitted_stamp[node_id]
            for node_id in self._node_ids
        ):
            return

        stamps = [self._latest[node_id][0] for node_id in self._node_ids]
        if max(stamps) - min(stamps) > self._max_stamp_skew_ns:
            return
        common_now_ns = self._common_now_ns()
        if common_now_ns is None:
            self._latest.clear()
            return
        if any(
            stamp_ns > common_now_ns + self._max_future_skew_ns
            or common_now_ns > stamp_ns + self._max_receive_delay_ns
            for stamp_ns in stamps
        ):
            return
        receive_now_ns = time.monotonic_ns()
        if any(
            receive_now_ns - self._latest[node_id][1] > self._fix_timeout_ns
            for node_id in self._node_ids
        ):
            return

        ranges = []
        for source_id, target_id in combinations(sorted(self._node_ids), 2):
            source = self._latest[source_id][2]
            target = self._latest[target_id][2]
            distance_m = math.sqrt(sum(
                (target[index] - source[index]) ** 2 for index in range(3)
            ))
            if (
                not math.isfinite(distance_m)
                or distance_m <= 0.0
                or distance_m > self._max_range_m
            ):
                return
            ranges.append((source_id, target_id, distance_m))

        epoch_stamp_ns = max(stamps)
        for source_id, target_id, distance_m in ranges:
            output = UwbRange()
            output.header.stamp.sec = epoch_stamp_ns // _NANOSECONDS_PER_SECOND
            output.header.stamp.nanosec = epoch_stamp_ns % _NANOSECONDS_PER_SECOND
            output.src_id = source_id
            output.target_id = target_id
            output.distance = float(distance_m)
            self._publisher.publish(output)
        self._last_emitted_stamp = {
            node_id: self._latest[node_id][0] for node_id in self._node_ids
        }


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GnssRangeFallbackNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # Keep Python exceptions out of launch internals.
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"GNSS range fallback initialization failed: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

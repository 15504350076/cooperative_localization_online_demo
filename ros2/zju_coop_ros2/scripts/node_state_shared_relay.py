#!/usr/bin/env python3
"""Republish validated loopback NodeState packets on the shared DDS graph."""

import socket

import rclpy
from cooperative_localization_msgs.msg import NodeState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message

from node_state_loopback_protocol import MAX_PACKET_BYTES, decode_packet


class NodeStateSharedRelay(Node):
    def __init__(self):
        super().__init__("zju_node_state_shared_relay")
        port = int(self.declare_parameter("loopback_port", 45120).value)
        if not 1024 <= port <= 65535:
            raise ValueError("loopback_port must be in [1024, 65535]")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(("127.0.0.1", port))
        self._socket.setblocking(False)
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            NodeState, "node_state_shared", qos
        )
        self._timer = self.create_timer(0.005, self._drain_socket)
        self.get_logger().info(
            f"shared-DDS NodeState relay ready: 127.0.0.1:{port}"
        )

    def _drain_socket(self):
        while True:
            try:
                packet, source = self._socket.recvfrom(MAX_PACKET_BYTES + 1)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().warning(
                    str(error), throttle_duration_sec=2.0
                )
                return
            try:
                if source[0] != "127.0.0.1":
                    raise ValueError("NodeState datagram is not from loopback")
                node_id, payload = decode_packet(packet)
                message = deserialize_message(payload, NodeState)
                if message.node_id != node_id:
                    raise ValueError("NodeState node_id does not match envelope")
                self._publisher.publish(message)
            except (ValueError, RuntimeError) as error:
                self.get_logger().warning(
                    str(error), throttle_duration_sec=2.0
                )

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NodeStateSharedRelay()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"NodeState shared relay initialization failed: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

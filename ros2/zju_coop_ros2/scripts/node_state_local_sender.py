#!/usr/bin/env python3
"""Forward localhost-only NodeState CDR bytes to a loopback UDP socket."""

import socket

import rclpy
from cooperative_localization_msgs.msg import NodeState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message

from node_state_loopback_protocol import encode_packet


class NodeStateLocalSender(Node):
    def __init__(self):
        super().__init__("zju_node_state_local_sender")
        port = int(self.declare_parameter("loopback_port", 45120).value)
        if not 1024 <= port <= 65535:
            raise ValueError("loopback_port must be in [1024, 65535]")
        self._destination = ("127.0.0.1", port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            NodeState, "node_state_local", self._on_state, qos
        )
        self.get_logger().info(
            f"localhost-only NodeState sender ready: 127.0.0.1:{port}"
        )

    def _on_state(self, message):
        try:
            packet = encode_packet(message.node_id, serialize_message(message))
            self._socket.sendto(packet, self._destination)
        except (OSError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = NodeStateLocalSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"NodeState local sender initialization failed: {error}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

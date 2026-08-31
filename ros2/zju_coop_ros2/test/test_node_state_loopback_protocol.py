#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(
    0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts")
)

from node_state_loopback_protocol import decode_packet, encode_packet  # noqa: E402


class TestNodeStateLoopbackProtocol(unittest.TestCase):
    def test_round_trip(self):
        packet = encode_packet(3, b"serialized-node-state")
        self.assertEqual(decode_packet(packet), (3, b"serialized-node-state"))

    def test_corruption_is_rejected(self):
        packet = bytearray(encode_packet(1, b"payload"))
        packet[-1] ^= 0x01
        with self.assertRaises(ValueError):
            decode_packet(packet)

    def test_zero_node_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_packet(0, b"payload")


if __name__ == "__main__":
    unittest.main()

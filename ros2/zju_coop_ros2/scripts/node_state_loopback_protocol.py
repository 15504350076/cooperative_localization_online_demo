"""Small validated datagram envelope for the localhost-only NodeState bridge."""

import struct
import zlib


_MAGIC = b"ZJNS"
_VERSION = 1
_HEADER = struct.Struct("!4sB3xIII")
MAX_PAYLOAD_BYTES = 4096
MAX_PACKET_BYTES = _HEADER.size + MAX_PAYLOAD_BYTES


def encode_packet(node_id, payload):
    payload = bytes(payload)
    if not 0 < int(node_id) <= 0xFFFFFFFF:
        raise ValueError("node_id is outside uint32")
    if not 0 < len(payload) <= MAX_PAYLOAD_BYTES:
        raise ValueError("invalid NodeState payload length")
    checksum = zlib.crc32(payload) & 0xFFFFFFFF
    return _HEADER.pack(
        _MAGIC, _VERSION, int(node_id), len(payload), checksum
    ) + payload


def decode_packet(packet):
    packet = bytes(packet)
    if len(packet) < _HEADER.size:
        raise ValueError("truncated NodeState packet")
    magic, version, node_id, payload_size, checksum = _HEADER.unpack_from(packet)
    payload = packet[_HEADER.size:]
    if magic != _MAGIC or version != _VERSION:
        raise ValueError("unsupported NodeState packet")
    if node_id == 0 or payload_size == 0 or payload_size > MAX_PAYLOAD_BYTES:
        raise ValueError("invalid NodeState packet header")
    if payload_size != len(payload):
        raise ValueError("NodeState packet length mismatch")
    if zlib.crc32(payload) & 0xFFFFFFFF != checksum:
        raise ValueError("NodeState packet checksum mismatch")
    return node_id, payload

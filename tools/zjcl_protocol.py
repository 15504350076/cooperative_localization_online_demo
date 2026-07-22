"""临时ZJCL/UDP协议的严格Python标准库编解码器。

本模块与C++ wire_protocol保持相同的小端固定布局、CRC范围、枚举和数值约束，供模拟器、
冒烟测试和演示GCS复用；它不是最终ROS 2接口，也不承担网络路由或安全认证。
"""

from dataclasses import dataclass
import math
import struct
import zlib


MAGIC = b"ZJCL"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0
HEADER_SIZE = 40
MAX_PAYLOAD_SIZE = 1024 * 1024
MAX_UDP_DATAGRAM = 65507
MAX_UDP_PAYLOAD = MAX_UDP_DATAGRAM - HEADER_SIZE

MSG_RANGE = 1
MSG_IMU = 2
MSG_LOCALIZATION = 100
MSG_NETWORK = 101
MSG_OBSERVATION = 102
MSG_ALERT = 103
MSG_ALGORITHM_STATUS = 104

RANGE_STATUS_OK = 0
RANGE_STATUS_DEGRADED = 1
RANGE_STATUS_INVALID = 2

LOCALIZATION_UNINITIALIZED = 0
LOCALIZATION_NORMAL = 1
LOCALIZATION_DEGRADED = 2
LOCALIZATION_UNOBSERVABLE = 3
LOCALIZATION_STALE = 4

OBSERVATION_UNKNOWN = 0
OBSERVATION_NORMAL = 1
OBSERVATION_DEGRADED = 2
OBSERVATION_SUSPENDED = 3
OBSERVATION_REJECTED = 4
OBSERVATION_RECOVERING = 5

FUSION_USE_NORMAL = 0
FUSION_USE_DOWNWEIGHTED = 1
FUSION_HOLD = 2
FUSION_REJECT = 3
FUSION_TRIAL_RECOVERY = 4

ALGORITHM_STATUS_ABI_VERSION = 0x00010000
SOFTWARE_VERSION_PACKED = 0x00000100
ALGORITHM_MODE_UWB_ONLY_PLANAR = 1
ALGORITHM_MODE_IMU_UWB_15_STATE = 2
ALGORITHM_RUN_INITIALIZING = 0
ALGORITHM_RUN_RUNNING = 1
ALGORITHM_RUN_DEGRADED = 2
ALGORITHM_RUN_ERROR = 3
ALGORITHM_RUN_STOPPED = 4

ALERT_CODE_NETWORK_STATE = 1
ALERT_LEVEL_INFO = 0
ALERT_LEVEL_WARNING = 1
ALERT_LEVEL_ERROR = 2
ALERT_LEVEL_CRITICAL = 3
ALERT_LIFECYCLE_ACTIVE = 0
ALERT_LIFECYCLE_CLEARED = 1
ALERT_SOURCE_ALGORITHM = 0

CAPABILITY_UWB_RANGE = 1 << 0
CAPABILITY_PLANAR_POSITION = 1 << 1
CAPABILITY_VELOCITY = 1 << 2

_KNOWN_TYPES = {
    MSG_RANGE,
    MSG_IMU,
    MSG_LOCALIZATION,
    MSG_NETWORK,
    MSG_OBSERVATION,
    MSG_ALERT,
    MSG_ALGORITHM_STATUS,
}
_FIXED_PAYLOAD_SIZES = {
    MSG_RANGE: 24,
    MSG_IMU: 332,
    MSG_LOCALIZATION: 64,
    MSG_NETWORK: 20,
    MSG_OBSERVATION: 80,
    MSG_ALERT: 40,
    MSG_ALGORITHM_STATUS: 48,
}
_HEADER_PREFIX = struct.Struct("<4sBBHHHIQQHH")
# “<”明确固定小端，不依赖运行Python的Windows/x86或RK3588/ARM64主机字节序。
_RANGE = struct.Struct("<ddfBBBB")
_IMU = struct.Struct("<4d9d3d9d3d9d32s4B")
_LOCALIZATION = struct.Struct("<dddddddBBBBI")
_NETWORK = struct.Struct("<IIIBBBBI")
_OBSERVATION = struct.Struct("<QQIIIIIIddddBBBBI")
_ALERT = struct.Struct("<IBBBBIHHHHQQI")
_ALGORITHM_STATUS = struct.Struct("<IIBBHIQQQQ")


class ProtocolError(ValueError):
    """Raised when a frame or payload violates the frozen v1 contract."""


@dataclass(frozen=True)
class Frame:
    message_type: int
    flags: int
    sequence: int
    timestamp_ns: int
    source_node: int
    target_node: int
    payload: bytes


@dataclass(frozen=True)
class RangePayload:
    range_m: float
    range_std_m: float
    nlos_probability: float
    nlos_flag: bool
    has_nlos_probability: bool
    valid: bool
    status: int


@dataclass(frozen=True)
class ImuPayload:
    """ROS 2 Imu瞬时量的临时UDP表示；不包含温度。"""
    orientation_xyzw: tuple
    orientation_covariance: tuple
    angular_velocity_rad_s: tuple
    angular_velocity_covariance: tuple
    linear_acceleration_m_s2: tuple
    linear_acceleration_covariance: tuple
    frame_id: str
    orientation_valid: bool
    valid: bool
    status: int


@dataclass(frozen=True)
class LocalizationPayload:
    x: float
    y: float
    vx: float
    vy: float
    cov_xx: float
    cov_xy: float
    cov_yy: float
    state: int
    valid: bool
    yaw_valid: bool
    z_valid: bool
    capability_mask: int


@dataclass(frozen=True)
class NetworkPayload:
    node_count: int
    reachable_node_count: int
    active_edge_count: int
    connected: bool
    observable: bool
    state: int
    reason_mask: int


@dataclass(frozen=True)
class ObservationPayload:
    window_start_ns: int
    window_end_ns: int
    expected_count: int
    received_count: int
    valid_count: int
    nlos_count: int
    residual_rejected_count: int
    dropped_count: int
    nlos_ratio: float
    valid_ratio: float
    actual_rate_hz: float
    covariance_scale: float
    state: int
    action: int
    input_overflow: bool
    reason_mask: int


@dataclass(frozen=True)
class AlertPayload:
    alert_code: int
    level: int
    lifecycle: int
    source: int
    reason_mask: int
    node_id: int
    from_node: int
    to_node: int
    first_timestamp_ns: int
    last_timestamp_ns: int


@dataclass(frozen=True)
class AlgorithmStatusPayload:
    abi_version: int
    software_version_packed: int
    mode: int
    run_state: int
    accepted_ranges: int
    rejected_ranges: int
    protocol_errors: int
    uptime_ns: int


def _uint(name, value, bits):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an unsigned {bits}-bit integer")
    if value < 0 or value > (1 << bits) - 1:
        raise ProtocolError(f"{name} is outside uint{bits}")
    return value


def _boolean(name, value):
    if type(value) is not bool:
        raise ProtocolError(f"{name} must be bool")
    return 1 if value else 0


def _decoded_boolean(name, value):
    if value not in (0, 1):
        raise ProtocolError(f"{name} must be encoded as zero or one")
    return value == 1


def _finite(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{name} must be finite")
    return result


def _unit(name, value):
    result = _finite(name, value)
    if result < 0.0 or result > 1.0:
        raise ProtocolError(f"{name} must be in [0,1]")
    return result


def _enum(name, value, maximum):
    result = _uint(name, value, 8)
    if result > maximum:
        raise ProtocolError(f"{name} is unknown")
    return result


def _scaled_product(left, right):
    left_fraction, left_exponent = math.frexp(left)
    right_fraction, right_exponent = math.frexp(right)
    product_fraction, product_exponent = math.frexp(
        left_fraction * right_fraction
    )
    return (
        product_fraction,
        left_exponent + right_exponent + product_exponent,
    )


def _positive_semidefinite_2x2(cov_xx, cov_xy, cov_yy):
    """无直接乘方溢出地检查二维协方差行列式非负。"""
    if cov_xx < 0.0 or cov_yy < 0.0:
        return False
    if cov_xx == 0.0 or cov_yy == 0.0:
        return cov_xy == 0.0
    magnitude = abs(cov_xy)
    if magnitude == 0.0:
        return True
    square_fraction, square_exponent = _scaled_product(magnitude, magnitude)
    product_fraction, product_exponent = _scaled_product(cov_xx, cov_yy)
    if square_exponent != product_exponent:
        return square_exponent < product_exponent
    return square_fraction <= product_fraction


def _bytes(value, name="payload"):
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise ProtocolError(f"{name} must be bytes-like")
    return bytes(value)


def _payload_limit(max_payload_size):
    value = _uint("max_payload_size", max_payload_size, 32)
    if value > MAX_PAYLOAD_SIZE:
        raise ProtocolError("max_payload_size exceeds the 1 MiB hard limit")
    return value


def encode_frame(frame, max_payload_size=MAX_PAYLOAD_SIZE, udp=False):
    """校验业务字段并编码公共帧；CRC覆盖36字节前缀和完整载荷。"""
    # 阶段1：完成类型、范围、固定载荷长度和UDP上限检查后再打包。
    if not isinstance(frame, Frame):
        raise ProtocolError("frame must be a Frame")
    message_type = _uint("message_type", frame.message_type, 16)
    if message_type not in _KNOWN_TYPES:
        raise ProtocolError("message_type is unknown")
    flags = _uint("flags", frame.flags, 16)
    sequence = _uint("sequence", frame.sequence, 64)
    timestamp_ns = _uint("timestamp_ns", frame.timestamp_ns, 64)
    source_node = _uint("source_node", frame.source_node, 16)
    target_node = _uint("target_node", frame.target_node, 16)
    payload = _bytes(frame.payload)
    fixed_size = _FIXED_PAYLOAD_SIZES.get(message_type)
    if fixed_size is not None and len(payload) != fixed_size:
        raise ProtocolError("payload size does not match message type")
    if len(payload) > _payload_limit(max_payload_size):
        raise ProtocolError("payload exceeds the 1 MiB/configured limit")
    if udp and HEADER_SIZE + len(payload) > MAX_UDP_DATAGRAM:
        raise ProtocolError("frame exceeds the UDP datagram limit")

    prefix = _HEADER_PREFIX.pack(
        MAGIC,
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        message_type,
        HEADER_SIZE,
        flags,
        len(payload),
        sequence,
        timestamp_ns,
        source_node,
        target_node,
    )
    # CRC字段位于偏移36，计算时尚未拼入，因此与C++跳过CRC字段的规则一致。
    crc = zlib.crc32(prefix + payload) & 0xFFFFFFFF
    return prefix + struct.pack("<I", crc) + payload


def decode_frame(data, max_payload_size=MAX_PAYLOAD_SIZE, udp=False):
    """按长度、头部、固定载荷长度和CRC顺序严格解码一个完整帧。"""
    # 阶段2：任何校验失败都抛ProtocolError，调用方不得使用部分解析结果。
    encoded = _bytes(data, "frame")
    if udp and len(encoded) > MAX_UDP_DATAGRAM:
        raise ProtocolError("UDP frame exceeds the datagram limit")
    if len(encoded) < HEADER_SIZE:
        raise ProtocolError("frame is shorter than the 40-byte header")
    fields = _HEADER_PREFIX.unpack_from(encoded, 0)
    (magic, major, minor, message_type, header_size, flags, payload_size,
     sequence, timestamp_ns, source_node, target_node) = fields
    if magic != MAGIC:
        raise ProtocolError("bad frame magic")
    if (major, minor) != (PROTOCOL_MAJOR, PROTOCOL_MINOR):
        raise ProtocolError("unsupported protocol version")
    if message_type not in _KNOWN_TYPES:
        raise ProtocolError("message_type is unknown")
    if header_size != HEADER_SIZE:
        raise ProtocolError("header_size is not 40")
    if payload_size > _payload_limit(max_payload_size):
        raise ProtocolError("payload exceeds the 1 MiB/configured limit")
    fixed_size = _FIXED_PAYLOAD_SIZES.get(message_type)
    if fixed_size is not None and payload_size != fixed_size:
        raise ProtocolError("payload size does not match message type")
    expected_size = HEADER_SIZE + payload_size
    if len(encoded) < expected_size:
        raise ProtocolError("frame payload is truncated")
    if len(encoded) > expected_size:
        raise ProtocolError("frame has trailing bytes")
    encoded_crc = struct.unpack_from("<I", encoded, 36)[0]
    payload = encoded[HEADER_SIZE:]
    actual_crc = zlib.crc32(encoded[:36] + payload) & 0xFFFFFFFF
    if encoded_crc != actual_crc:
        raise ProtocolError("frame CRC32 mismatch")
    return Frame(message_type, flags, sequence, timestamp_ns, source_node,
                 target_node, payload)


def encode_range_payload(value):
    if not isinstance(value, RangePayload):
        raise ProtocolError("range value has wrong type")
    range_m = _finite("range_m", value.range_m)
    range_std_m = _finite("range_std_m", value.range_std_m)
    probability = _unit("nlos_probability", value.nlos_probability)
    if range_m <= 0.0 or range_std_m <= 0.0:
        raise ProtocolError("range and standard deviation must be positive")
    return _RANGE.pack(
        range_m,
        range_std_m,
        probability,
        _boolean("nlos_flag", value.nlos_flag),
        _boolean("has_nlos_probability", value.has_nlos_probability),
        _boolean("valid", value.valid),
        _enum("range status", value.status, RANGE_STATUS_INVALID),
    )


def decode_range_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _RANGE.size:
        raise ProtocolError("range payload must be 24 bytes")
    values = _RANGE.unpack(encoded)
    result = RangePayload(
        values[0], values[1], values[2],
        _decoded_boolean("nlos_flag", values[3]),
        _decoded_boolean("has_nlos_probability", values[4]),
        _decoded_boolean("valid", values[5]), values[6],
    )
    encode_range_payload(result)
    return result


def _float_tuple(name, values, size):
    if not isinstance(values, (tuple, list)) or len(values) != size:
        raise ProtocolError(f"{name} must contain {size} values")
    return tuple(_finite(f"{name}[{index}]", value)
                 for index, value in enumerate(values))


def encode_imu_payload(value):
    """编码ROS 2 Imu瞬时量映射，不包含温度或调用方预积分结果。"""
    if not isinstance(value, ImuPayload):
        raise ProtocolError("IMU value has wrong type")
    orientation = _float_tuple("orientation_xyzw", value.orientation_xyzw, 4)
    orientation_covariance = _float_tuple(
        "orientation_covariance", value.orientation_covariance, 9)
    angular_velocity = _float_tuple(
        "angular_velocity_rad_s", value.angular_velocity_rad_s, 3)
    angular_covariance = _float_tuple(
        "angular_velocity_covariance", value.angular_velocity_covariance, 9)
    linear_acceleration = _float_tuple(
        "linear_acceleration_m_s2", value.linear_acceleration_m_s2, 3)
    linear_covariance = _float_tuple(
        "linear_acceleration_covariance", value.linear_acceleration_covariance,
        9)
    if not isinstance(value.frame_id, str) or not value.frame_id:
        raise ProtocolError("frame_id must be a non-empty string")
    try:
        frame = value.frame_id.encode("utf-8")
    except UnicodeEncodeError as error:
        raise ProtocolError("frame_id must be UTF-8") from error
    if len(frame) >= 32 or b"\0" in frame:
        raise ProtocolError("frame_id must fit in 31 UTF-8 bytes")
    # frame_id最多31个UTF-8字节，最后至少保留一个NUL以匹配C ABI固定数组。
    frame += b"\0" * (32 - len(frame))
    status = _enum("IMU status", value.status, RANGE_STATUS_INVALID)
    return _IMU.pack(
        *orientation, *orientation_covariance,
        *angular_velocity, *angular_covariance,
        *linear_acceleration, *linear_covariance, frame,
        _boolean("orientation_valid", value.orientation_valid),
        _boolean("valid", value.valid), status, 0)


def decode_imu_payload(data):
    """解码固定332字节IMU载荷，并复用编码器执行语义级复验。"""
    encoded = _bytes(data)
    if len(encoded) != _IMU.size:
        raise ProtocolError("IMU payload must be 332 bytes")
    values = _IMU.unpack(encoded)
    frame_bytes = values[37]
    terminator = frame_bytes.find(b"\0")
    if terminator <= 0:
        raise ProtocolError("frame_id must be NUL-terminated and non-empty")
    try:
        frame_id = frame_bytes[:terminator].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ProtocolError("frame_id must be UTF-8") from error
    if values[41] != 0:
        raise ProtocolError("IMU reserved byte must be zero")
    result = ImuPayload(
        tuple(values[0:4]), tuple(values[4:13]), tuple(values[13:16]),
        tuple(values[16:25]), tuple(values[25:28]), tuple(values[28:37]),
        frame_id,
        _decoded_boolean("orientation_valid", values[38]),
        _decoded_boolean("valid", values[39]), values[40])
    # 复用编码器完成有限值、枚举和字段长度验证。
    encode_imu_payload(result)
    return result


def encode_localization_payload(value):
    """编码主参考二维相对状态及2×2协方差、有效位和能力位。"""
    if not isinstance(value, LocalizationPayload):
        raise ProtocolError("localization value has wrong type")
    cov_xx = _finite("cov_xx", value.cov_xx)
    cov_xy = _finite("cov_xy", value.cov_xy)
    cov_yy = _finite("cov_yy", value.cov_yy)
    if not _positive_semidefinite_2x2(cov_xx, cov_xy, cov_yy):
        raise ProtocolError("localization covariance is not positive semidefinite")
    numbers = [
        _finite("x", value.x), _finite("y", value.y),
        _finite("vx", value.vx), _finite("vy", value.vy),
        cov_xx, cov_xy, cov_yy,
    ]
    return _LOCALIZATION.pack(
        *numbers,
        _enum("localization state", value.state, LOCALIZATION_STALE),
        _boolean("valid", value.valid),
        _boolean("yaw_valid", value.yaw_valid),
        _boolean("z_valid", value.z_valid),
        _uint("capability_mask", value.capability_mask, 32),
    )


def decode_localization_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _LOCALIZATION.size:
        raise ProtocolError("localization payload must be 64 bytes")
    values = _LOCALIZATION.unpack(encoded)
    result = LocalizationPayload(
        *values[:7],
        _enum("localization state", values[7], LOCALIZATION_STALE),
        _decoded_boolean("valid", values[8]),
        _decoded_boolean("yaw_valid", values[9]),
        _decoded_boolean("z_valid", values[10]), values[11],
    )
    encode_localization_payload(result)
    return result


def encode_network_payload(value):
    """编码动态拓扑，并交叉验证节点数、活动边、连通与可观状态。"""
    if not isinstance(value, NetworkPayload):
        raise ProtocolError("network value has wrong type")
    node_count = _uint("node_count", value.node_count, 32)
    reachable_node_count = _uint(
        "reachable_node_count", value.reachable_node_count, 32
    )
    active_edge_count = _uint(
        "active_edge_count", value.active_edge_count, 32
    )
    connected = _boolean("connected", value.connected)
    observable = _boolean("observable", value.observable)
    if reachable_node_count > node_count:
        raise ProtocolError("reachable_node_count exceeds node_count")
    minimum_edge_count = max(0, reachable_node_count - 1)
    maximum_edge_count = node_count * max(0, node_count - 1) // 2
    if (active_edge_count < minimum_edge_count
            or active_edge_count > maximum_edge_count):
        raise ProtocolError("active_edge_count is inconsistent with topology")
    expected_connected = node_count > 0 and reachable_node_count == node_count
    if (connected == 1) != expected_connected:
        raise ProtocolError("connected is inconsistent with reachability")
    if observable == 1 and connected == 0:
        raise ProtocolError("observable network must be connected")
    return _NETWORK.pack(
        node_count,
        reachable_node_count,
        active_edge_count,
        connected,
        observable,
        _enum("localization state", value.state, LOCALIZATION_STALE),
        0,
        _uint("reason_mask", value.reason_mask, 32),
    )


def decode_network_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _NETWORK.size:
        raise ProtocolError("network payload must be 20 bytes")
    values = _NETWORK.unpack(encoded)
    if values[6] != 0:
        raise ProtocolError("network reserved byte is nonzero")
    result = NetworkPayload(
        values[0], values[1], values[2],
        _decoded_boolean("connected", values[3]),
        _decoded_boolean("observable", values[4]),
        _enum("localization state", values[5], LOCALIZATION_STALE), values[7],
    )
    encode_network_payload(result)
    return result


def encode_observation_payload(value):
    """编码单条协同边的滑窗统计、退化状态、融合动作和原因位图。"""
    if not isinstance(value, ObservationPayload):
        raise ProtocolError("observation value has wrong type")
    nlos_ratio = _unit("nlos_ratio", value.nlos_ratio)
    valid_ratio = _unit("valid_ratio", value.valid_ratio)
    actual_rate_hz = _finite("actual_rate_hz", value.actual_rate_hz)
    covariance_scale = _finite("covariance_scale", value.covariance_scale)
    if actual_rate_hz < 0.0 or covariance_scale <= 0.0:
        raise ProtocolError("rate/scale is outside its valid range")
    window_start_ns = _uint("window_start_ns", value.window_start_ns, 64)
    window_end_ns = _uint("window_end_ns", value.window_end_ns, 64)
    received_count = _uint("received_count", value.received_count, 32)
    valid_count = _uint("valid_count", value.valid_count, 32)
    nlos_count = _uint("nlos_count", value.nlos_count, 32)
    residual_rejected_count = _uint(
        "residual_rejected_count", value.residual_rejected_count, 32
    )
    if window_start_ns > window_end_ns:
        raise ProtocolError("observation window is reversed")
    if valid_count > received_count:
        raise ProtocolError("valid_count exceeds received_count")
    if nlos_count > received_count:
        raise ProtocolError("nlos_count exceeds received_count")
    if residual_rejected_count > received_count:
        raise ProtocolError("residual_rejected_count exceeds received_count")
    if residual_rejected_count > valid_count:
        raise ProtocolError("residual_rejected_count exceeds valid_count")
    return _OBSERVATION.pack(
        window_start_ns,
        window_end_ns,
        _uint("expected_count", value.expected_count, 32),
        received_count,
        valid_count,
        nlos_count,
        residual_rejected_count,
        _uint("dropped_count", value.dropped_count, 32),
        nlos_ratio,
        valid_ratio,
        actual_rate_hz,
        covariance_scale,
        _enum("observation state", value.state, OBSERVATION_RECOVERING),
        _enum("fusion action", value.action, FUSION_TRIAL_RECOVERY),
        _boolean("input_overflow", value.input_overflow),
        0,
        _uint("reason_mask", value.reason_mask, 32),
    )


def decode_observation_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _OBSERVATION.size:
        raise ProtocolError("observation payload must be 80 bytes")
    values = _OBSERVATION.unpack(encoded)
    if values[15] != 0:
        raise ProtocolError("observation reserved byte is nonzero")
    result = ObservationPayload(
        *values[:12],
        _enum("observation state", values[12], OBSERVATION_RECOVERING),
        _enum("fusion action", values[13], FUSION_TRIAL_RECOVERY),
        _decoded_boolean("input_overflow", values[14]), values[16],
    )
    encode_observation_payload(result)
    return result


def encode_alert_payload(value):
    """编码网络状态告警；ACTIVE必须有原因，CLEARED必须清空原因。"""
    if not isinstance(value, AlertPayload):
        raise ProtocolError("alert value has wrong type")
    alert_code = _uint("alert_code", value.alert_code, 32)
    if alert_code != ALERT_CODE_NETWORK_STATE:
        raise ProtocolError("alert_code is unknown")
    level = _enum("alert level", value.level, ALERT_LEVEL_CRITICAL)
    lifecycle = _enum(
        "alert lifecycle", value.lifecycle, ALERT_LIFECYCLE_CLEARED
    )
    source = _uint("alert source", value.source, 8)
    if source != ALERT_SOURCE_ALGORITHM:
        raise ProtocolError("alert source is unknown")
    reason_mask = _uint("reason_mask", value.reason_mask, 32)
    if lifecycle == ALERT_LIFECYCLE_ACTIVE and reason_mask == 0:
        raise ProtocolError("active NETWORK_STATE alert requires a reason")
    if lifecycle == ALERT_LIFECYCLE_CLEARED and reason_mask != 0:
        raise ProtocolError("cleared NETWORK_STATE alert must clear reasons")
    first_timestamp_ns = _uint(
        "first_timestamp_ns", value.first_timestamp_ns, 64
    )
    last_timestamp_ns = _uint(
        "last_timestamp_ns", value.last_timestamp_ns, 64
    )
    if first_timestamp_ns > last_timestamp_ns:
        raise ProtocolError("alert timestamps are reversed")
    return _ALERT.pack(
        alert_code,
        level,
        lifecycle,
        source,
        0,
        reason_mask,
        _uint("node_id", value.node_id, 16),
        _uint("from_node", value.from_node, 16),
        _uint("to_node", value.to_node, 16),
        0,
        first_timestamp_ns,
        last_timestamp_ns,
        0,
    )


def decode_alert_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _ALERT.size:
        raise ProtocolError("alert payload must be 40 bytes")
    values = _ALERT.unpack(encoded)
    if values[4] != 0 or values[9] != 0 or values[12] != 0:
        raise ProtocolError("alert reserved fields are nonzero")
    result = AlertPayload(
        values[0], values[1], values[2], values[3], values[5],
        values[6], values[7], values[8], values[10], values[11],
    )
    encode_alert_payload(result)
    return result


def encode_algorithm_status_payload(value):
    if not isinstance(value, AlgorithmStatusPayload):
        raise ProtocolError("algorithm status value has wrong type")
    abi_version = _uint("abi_version", value.abi_version, 32)
    if abi_version != ALGORITHM_STATUS_ABI_VERSION:
        raise ProtocolError("algorithm status ABI version is unsupported")
    mode = _uint("algorithm mode", value.mode, 8)
    if mode not in (ALGORITHM_MODE_UWB_ONLY_PLANAR,
                    ALGORITHM_MODE_IMU_UWB_15_STATE):
        raise ProtocolError("algorithm mode is unknown")
    run_state = _enum(
        "algorithm run state", value.run_state, ALGORITHM_RUN_STOPPED
    )
    return _ALGORITHM_STATUS.pack(
        abi_version,
        _uint("software_version_packed", value.software_version_packed, 32),
        mode,
        run_state,
        0,
        0,
        _uint("accepted_ranges", value.accepted_ranges, 64),
        _uint("rejected_ranges", value.rejected_ranges, 64),
        _uint("protocol_errors", value.protocol_errors, 64),
        _uint("uptime_ns", value.uptime_ns, 64),
    )


def decode_algorithm_status_payload(data):
    encoded = _bytes(data)
    if len(encoded) != _ALGORITHM_STATUS.size:
        raise ProtocolError("algorithm status payload must be 48 bytes")
    values = _ALGORITHM_STATUS.unpack(encoded)
    if values[4] != 0 or values[5] != 0:
        raise ProtocolError("algorithm status reserved fields are nonzero")
    result = AlgorithmStatusPayload(
        values[0], values[1], values[2], values[3],
        values[6], values[7], values[8], values[9],
    )
    encode_algorithm_status_payload(result)
    return result


def _self_check():
    check = zlib.crc32(b"123456789") & 0xFFFFFFFF
    if (check != 0xCBF43926 or _HEADER_PREFIX.size != 36
            or _ALERT.size != 40 or _ALGORITHM_STATUS.size != 48):
        raise RuntimeError("ZJCL protocol self-check failed")
    print("ZJCL_PROTOCOL_SELFTEST PASS")


if __name__ == "__main__":
    _self_check()

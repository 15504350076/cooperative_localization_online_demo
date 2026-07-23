"""临时ZJCL/UDP协议的严格Python标准库编解码器。

本模块与C++ wire_protocol保持相同的小端固定布局、CRC范围、枚举和数值约束，供模拟器、
冒烟测试和演示GCS复用；它不是最终ROS 2接口，也不承担网络路由或安全认证。
"""

from dataclasses import dataclass
import math
import struct
import zlib


# 公共帧头契约：线缆魔数、协议版本与包含CRC字段在内的固定头长（字节）。
MAGIC = b"ZJCL"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0
HEADER_SIZE = 40
# 传输边界：通用载荷硬上限为1 MiB，UDP载荷还须扣除40字节公共头。
MAX_PAYLOAD_SIZE = 1024 * 1024
MAX_UDP_DATAGRAM = 65507
MAX_UDP_PAYLOAD = MAX_UDP_DATAGRAM - HEADER_SIZE

# message_type线缆枚举；1/2为传感器输入，100以上为算法输出遥测。
MSG_RANGE = 1
MSG_IMU = 2
MSG_LOCALIZATION = 100
MSG_NETWORK = 101
MSG_OBSERVATION = 102
MSG_ALERT = 103
MSG_ALGORITHM_STATUS = 104

# 测距/IMU共用的质量状态枚举，数值必须与C++ v1 ABI保持一致。
RANGE_STATUS_OK = 0
RANGE_STATUS_DEGRADED = 1
RANGE_STATUS_INVALID = 2

# 定位与网络快照共用的解算状态枚举。
LOCALIZATION_UNINITIALIZED = 0
LOCALIZATION_NORMAL = 1
LOCALIZATION_DEGRADED = 2
LOCALIZATION_UNOBSERVABLE = 3
LOCALIZATION_STALE = 4

# 单条协同边在统计窗口内的观测质量状态。
OBSERVATION_UNKNOWN = 0
OBSERVATION_NORMAL = 1
OBSERVATION_DEGRADED = 2
OBSERVATION_SUSPENDED = 3
OBSERVATION_REJECTED = 4
OBSERVATION_RECOVERING = 5

# 算法对该边当前采取的融合策略，而非观测质量本身。
FUSION_USE_NORMAL = 0
FUSION_USE_DOWNWEIGHTED = 1
FUSION_HOLD = 2
FUSION_REJECT = 3
FUSION_TRIAL_RECOVERY = 4

# 算法状态载荷的ABI/软件版本以及运行模式、生命周期枚举。
ALGORITHM_STATUS_ABI_VERSION = 0x00010000
SOFTWARE_VERSION_PACKED = 0x00000100
ALGORITHM_MODE_UWB_ONLY_PLANAR = 1
ALGORITHM_MODE_IMU_UWB_15_STATE = 2
ALGORITHM_RUN_INITIALIZING = 0
ALGORITHM_RUN_RUNNING = 1
ALGORITHM_RUN_DEGRADED = 2
ALGORITHM_RUN_ERROR = 3
ALGORITHM_RUN_STOPPED = 4

# NETWORK_STATE告警的代码、严重度、生命周期和唯一合法来源。
ALERT_CODE_NETWORK_STATE = 1
ALERT_LEVEL_INFO = 0
ALERT_LEVEL_WARNING = 1
ALERT_LEVEL_ERROR = 2
ALERT_LEVEL_CRITICAL = 3
ALERT_LIFECYCLE_ACTIVE = 0
ALERT_LIFECYCLE_CLEARED = 1
ALERT_SOURCE_ALGORITHM = 0

# LocalizationPayload.capability_mask中的可独立组合能力位。
CAPABILITY_UWB_RANGE = 1 << 0
CAPABILITY_PLANAR_POSITION = 1 << 1
CAPABILITY_VELOCITY = 1 << 2

# 解码白名单以及每类v1消息在线缆上的精确载荷字节数。
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
# 11槽依次为magic、major、minor、message_type、header_size、flags、
# payload_size、sequence、timestamp_ns、source_node、target_node。
_HEADER_PREFIX = struct.Struct("<4sBBHHHIQQHH")
# “<”明确固定小端，不依赖运行Python的Windows/x86或RK3588/ARM64主机字节序。
# 各业务载荷的固定小端布局；字段顺序须与对应dataclass逐项一致。
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
    """一份完整ZJCL业务帧；payload不含40字节公共头，节点语义随消息类型解释。"""
    # 16位消息类型，必须属于_KNOWN_TYPES。
    message_type: int
    # 16位协议标志；v1当前透传但仍参与CRC。
    flags: int
    # 发送方维护的64位单调业务序号，通常按边或节点独立计数。
    sequence: int
    # 发送侧采样/产生时刻，Unix纪元纳秒。
    timestamp_ns: int
    # 16位源节点与目标节点；target_node为0时表示非点对点输出。
    source_node: int
    target_node: int
    # 不含公共头的业务载荷原始字节。
    payload: bytes


@dataclass(frozen=True)
class RangePayload:
    """测距值/标准差单位m；NLOS概率只有has_nlos_probability为真时才有业务含义。"""
    # 正的实测距离与1σ测距标准差，单位均为米。
    range_m: float
    range_std_m: float
    # [0,1] NLOS概率；has_nlos_probability声明该数值是否可信。
    nlos_probability: float
    # 本次测距是否被硬判决为NLOS，独立于概率字段是否可用。
    nlos_flag: bool
    has_nlos_probability: bool
    # valid控制该测距样本能否进入后续处理。
    valid: bool
    # 测距质量状态枚举：RANGE_STATUS_OK/DEGRADED/INVALID。
    status: int


@dataclass(frozen=True)
class ImuPayload:
    """ROS 2 Imu瞬时量的临时UDP表示；数组按行主序，不包含温度或预积分量。"""
    # 车体FLU到导航ENU的姿态四元数，按(x,y,z,w)；姿态误差协方差为
    # 3×3行主序，平方单位为rad²。
    orientation_xyzw: tuple
    orientation_covariance: tuple
    # 车体FLU系瞬时三轴角速度（rad/s），未预积分；对应协方差为3×3行主序，
    # 平方单位为(rad/s)²。
    angular_velocity_rad_s: tuple
    angular_velocity_covariance: tuple
    # 车体FLU系瞬时比力（m/s²），含重力响应（静止水平时+Z约为+g），未预积分；
    # 对应协方差为3×3行主序，平方单位为(m/s²)²。
    linear_acceleration_m_s2: tuple
    linear_acceleration_covariance: tuple
    # 非空UTF-8坐标系名，线缆固定槽32字节且必须NUL终止。
    frame_id: str
    # 姿态分量有效位、整帧有效位及共用质量状态枚举。
    orientation_valid: bool
    valid: bool
    status: int


@dataclass(frozen=True)
class LocalizationPayload:
    """主参考平面相对状态；能力位与valid/yaw_valid/z_valid必须联合判断。"""
    # 平面位置（m）与速度（m/s）。
    x: float
    y: float
    vx: float
    vy: float
    # 平面位置2×2协方差的xx/xy/yy独立元素，必须半正定。
    cov_xx: float
    cov_xy: float
    cov_yy: float
    # 解算状态与平面位置、航向、高度三类有效性声明。
    state: int
    valid: bool
    yaw_valid: bool
    z_valid: bool
    # CAPABILITY_*位图，声明该实现实际输出的观测能力。
    capability_mask: int


@dataclass(frozen=True)
class NetworkPayload:
    """同一快照下的活动边、主参考可达性、几何可观性和可组合原因位。"""
    # 拓扑总节点数、从主参考可达节点数和活动无向边数。
    node_count: int
    reachable_node_count: int
    active_edge_count: int
    # connected由可达数推导，observable为更强的几何可观条件。
    connected: bool
    observable: bool
    # 网络对应的定位状态与退化原因位图。
    state: int
    reason_mask: int


@dataclass(frozen=True)
class ObservationPayload:
    """一条无向协同边的滑窗计数、质量状态和实际融合动作。"""
    # 统计窗口起止时刻，Unix纪元纳秒且起点不得晚于终点。
    window_start_ns: int
    window_end_ns: int
    # 当前滑窗内的期望、收到、有效、NLOS和残差拒绝样本统计。
    expected_count: int
    received_count: int
    valid_count: int
    nlos_count: int
    residual_rejected_count: int
    # 监视器生命周期累计的历史容量溢出次数；每次表示淘汰了一个最旧样本。
    dropped_count: int
    # [0,1] NLOS/有效比例，实际输入频率Hz及协方差放大倍数。
    nlos_ratio: float
    valid_ratio: float
    actual_rate_hz: float
    covariance_scale: float
    # 观测状态、融合动作、输入溢出证据以及可组合原因位图。
    state: int
    action: int
    input_overflow: bool
    reason_mask: int


@dataclass(frozen=True)
class AlertPayload:
    """演示层告警生命周期；first保持首次激活时刻，last随活动/清除事件更新。"""
    # 告警代码、严重度、ACTIVE/CLEARED生命周期和来源枚举。
    alert_code: int
    level: int
    lifecycle: int
    source: int
    # 触发原因位图；ACTIVE非零、CLEARED必须归零。
    reason_mask: int
    # 可选的节点或无向边定位信息，0表示不指向具体对象。
    node_id: int
    from_node: int
    to_node: int
    # 首次激活和本次更新的Unix纪元纳秒时间戳。
    first_timestamp_ns: int
    last_timestamp_ns: int


@dataclass(frozen=True)
class AlgorithmStatusPayload:
    """演示进程模式和累计计数；进程重启后计数可以从零开始。"""
    # 固定ABI版本、三段压缩软件版本、算法模式及运行状态。
    abi_version: int
    software_version_packed: int
    mode: int
    run_state: int
    # 自进程启动以来累计接受、拒绝和协议错误计数。
    accepted_ranges: int
    rejected_ranges: int
    protocol_errors: int
    # 当前进程已运行纳秒数，不是Unix时间戳。
    uptime_ns: int


def _uint(name, value, bits):
    """校验无符号整数字段；name用于错误定位，value为候选值，bits为线缆位宽。"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError(f"{name} must be an unsigned {bits}-bit integer")
    if value < 0 or value > (1 << bits) - 1:
        raise ProtocolError(f"{name} is outside uint{bits}")
    return value


def _boolean(name, value):
    """把严格bool编码为0/1；name标识字段，value不得用整数冒充布尔值。"""
    if type(value) is not bool:
        raise ProtocolError(f"{name} must be bool")
    return 1 if value else 0


def _decoded_boolean(name, value):
    """解码0/1布尔字段；name用于错误消息，value是线缆无符号字节。"""
    if value not in (0, 1):
        raise ProtocolError(f"{name} must be encoded as zero or one")
    return value == 1


def _finite(name, value):
    """转成有限浮点数；name标识字段，value不得为bool、NaN或无穷。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f"{name} must be numeric")
    # 统一后的浮点结果供后续范围或物理约束复用。
    result = float(value)
    if not math.isfinite(result):
        raise ProtocolError(f"{name} must be finite")
    return result


def _unit(name, value):
    """校验单位区间；name标识字段，value必须是[0,1]有限数。"""
    # 已通过类型与有限性验证的规范化概率或比例。
    result = _finite(name, value)
    if result < 0.0 or result > 1.0:
        raise ProtocolError(f"{name} must be in [0,1]")
    return result


def _enum(name, value, maximum):
    """校验连续uint8枚举；name标识字段，value上限由maximum给出。"""
    # 先收窄为uint8，再拒绝v1尚未定义的枚举值。
    result = _uint(name, value, 8)
    if result > maximum:
        raise ProtocolError(f"{name} is unknown")
    return result


def _scaled_product(left, right):
    """返回乘积的尾数/指数表示；left、right用于避免直接乘法溢出。"""
    # 两个输入分别拆成二进制尾数和指数，保留极端有限数的可比较性。
    left_fraction, left_exponent = math.frexp(left)
    right_fraction, right_exponent = math.frexp(right)
    # 尾数乘积再次规范化，其指数并入两个原始指数。
    product_fraction, product_exponent = math.frexp(
        left_fraction * right_fraction
    )
    return (
        product_fraction,
        left_exponent + right_exponent + product_exponent,
    )


def _positive_semidefinite_2x2(cov_xx, cov_xy, cov_yy):
    """无直接乘方溢出地检查二维协方差行列式非负。

    cov_xx、cov_xy、cov_yy是对称2×2协方差的三个独立元素。
    """
    if cov_xx < 0.0 or cov_yy < 0.0:
        return False
    if cov_xx == 0.0 or cov_yy == 0.0:
        return cov_xy == 0.0
    # 非负交叉项幅值用于比较cov_xy²与cov_xx*cov_yy。
    magnitude = abs(cov_xy)
    if magnitude == 0.0:
        return True
    # square_fraction/square_exponent是cov_xy²的规范化二进制尾数和指数。
    square_fraction, square_exponent = _scaled_product(magnitude, magnitude)
    # product_fraction/product_exponent是cov_xx*cov_yy的尾数和指数，用于无溢出比较。
    product_fraction, product_exponent = _scaled_product(cov_xx, cov_yy)
    if square_exponent != product_exponent:
        return square_exponent < product_exponent
    return square_fraction <= product_fraction


def _bytes(value, name="payload"):
    """规范化字节类输入；value是候选缓冲区，name用于错误定位。"""
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise ProtocolError(f"{name} must be bytes-like")
    return bytes(value)


def _payload_limit(max_payload_size):
    """验证调用方载荷上限；max_payload_size不得突破协议1 MiB硬边界。"""
    # uint32形式的配置上限，后续与实际payload长度比较。
    value = _uint("max_payload_size", max_payload_size, 32)
    if value > MAX_PAYLOAD_SIZE:
        raise ProtocolError("max_payload_size exceeds the 1 MiB hard limit")
    return value


def encode_frame(frame, max_payload_size=MAX_PAYLOAD_SIZE, udp=False):
    """校验业务字段并编码公共帧；CRC输入是头部[0,36)紧接完整载荷。

    frame是待编码Frame；max_payload_size是调用方可进一步收紧的载荷上限；
    udp为真时额外执行65507字节UDP数据报边界校验。
    """
    # 阶段1：完成类型、范围、固定载荷长度和UDP上限检查后再打包。
    if not isinstance(frame, Frame):
        raise ProtocolError("frame must be a Frame")
    # 规范化后的头部字段，严格对应_HEADER_PREFIX中的线缆槽位。
    message_type = _uint("message_type", frame.message_type, 16)
    if message_type not in _KNOWN_TYPES:
        raise ProtocolError("message_type is unknown")
    flags = _uint("flags", frame.flags, 16)
    sequence = _uint("sequence", frame.sequence, 64)
    timestamp_ns = _uint("timestamp_ns", frame.timestamp_ns, 64)
    source_node = _uint("source_node", frame.source_node, 16)
    target_node = _uint("target_node", frame.target_node, 16)
    payload = _bytes(frame.payload)
    # 已知消息的固定载荷契约，防止类型正确但payload布局错位。
    fixed_size = _FIXED_PAYLOAD_SIZES.get(message_type)
    if fixed_size is not None and len(payload) != fixed_size:
        raise ProtocolError("payload size does not match message type")
    if len(payload) > _payload_limit(max_payload_size):
        raise ProtocolError("payload exceeds the 1 MiB/configured limit")
    if udp and HEADER_SIZE + len(payload) > MAX_UDP_DATAGRAM:
        raise ProtocolError("frame exceeds the UDP datagram limit")

    # 不含CRC的36字节头前缀，随后与payload共同计算CRC32。
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
    # CRC字段位于偏移36；计算时完全排除这4字节，而不是把它们作为零值参与。
    crc = zlib.crc32(prefix + payload) & 0xFFFFFFFF
    return prefix + struct.pack("<I", crc) + payload


def decode_frame(data, max_payload_size=MAX_PAYLOAD_SIZE, udp=False):
    """按长度、头部、固定载荷长度和CRC顺序严格解码一个完整帧。

    data是完整帧字节；max_payload_size是接收侧配置上限；udp为真时先应用
    UDP数据报总长限制。任何失败均不返回部分解析结果。
    """
    # 阶段2：任何校验失败都抛ProtocolError，调用方不得使用部分解析结果。
    encoded = _bytes(data, "frame")
    if udp and len(encoded) > MAX_UDP_DATAGRAM:
        raise ProtocolError("UDP frame exceeds the datagram limit")
    if len(encoded) < HEADER_SIZE:
        raise ProtocolError("frame is shorter than the 40-byte header")
    # 先提取不含CRC的公共头字段，逐项验证后才切分业务payload。
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
    # message_type在v1中约定的固定payload长度，用于与帧头声明的payload_size比对。
    fixed_size = _FIXED_PAYLOAD_SIZES.get(message_type)
    if fixed_size is not None and payload_size != fixed_size:
        raise ProtocolError("payload size does not match message type")
    # 头内声明的唯一合法总帧长，用于同时拒绝截断和尾随数据。
    expected_size = HEADER_SIZE + payload_size
    if len(encoded) < expected_size:
        raise ProtocolError("frame payload is truncated")
    if len(encoded) > expected_size:
        raise ProtocolError("frame has trailing bytes")
    # 线缆CRC与按“头前36字节+完整载荷”重算值组成完整性证据。
    encoded_crc = struct.unpack_from("<I", encoded, 36)[0]
    payload = encoded[HEADER_SIZE:]
    actual_crc = zlib.crc32(encoded[:36] + payload) & 0xFFFFFFFF
    if encoded_crc != actual_crc:
        raise ProtocolError("frame CRC32 mismatch")
    return Frame(message_type, flags, sequence, timestamp_ns, source_node,
                 target_node, payload)


def encode_range_payload(value):
    """编码测距载荷；value必须是满足正距离、正标准差和合法状态的RangePayload。"""
    if not isinstance(value, RangePayload):
        raise ProtocolError("range value has wrong type")
    # 规范化后的物理量与NLOS概率依次对应_RANGE的前三个数值槽。
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
    """解码24字节测距载荷；data经结构解包后会再走编码语义校验。"""
    # 解码三阶段：encoded是固定长度原始payload；values是_RANGE解包槽且布尔字节
    # 尚待校验；result是具名RangePayload，并会重新编码以统一复验语义约束。
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
    """规范化定长浮点数组；name标识字段，values为序列，size为协议元素数。"""
    if not isinstance(values, (tuple, list)) or len(values) != size:
        raise ProtocolError(f"{name} must contain {size} values")
    # index/value分别定位数组元素和候选数值，便于精确报告非有限项。
    return tuple(_finite(f"{name}[{index}]", value)
                 for index, value in enumerate(values))


def encode_imu_payload(value):
    """编码ROS 2 Imu瞬时量映射；value为ImuPayload，不含温度或预积分结果。"""
    if not isinstance(value, ImuPayload):
        raise ProtocolError("IMU value has wrong type")
    # 下列规范化元组按_IMU线缆布局顺序展开，协方差均为3×3行主序。
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
        # 尚未追加NUL填充的frame_id UTF-8原始字节。
        frame = value.frame_id.encode("utf-8")
    # error保留frame_id编码为UTF-8失败的原始UnicodeEncodeError因果链。
    except UnicodeEncodeError as error:
        raise ProtocolError("frame_id must be UTF-8") from error
    if len(frame) >= 32 or b"\0" in frame:
        raise ProtocolError("frame_id must fit in 31 UTF-8 bytes")
    # frame_id最多31个UTF-8字节，最后至少保留一个NUL以匹配C ABI固定数组。
    frame += b"\0" * (32 - len(frame))
    # 规范化后的IMU质量状态字节，写入_IMU布局的尾部状态槽。
    status = _enum("IMU status", value.status, RANGE_STATUS_INVALID)
    return _IMU.pack(
        *orientation, *orientation_covariance,
        *angular_velocity, *angular_covariance,
        *linear_acceleration, *linear_covariance, frame,
        _boolean("orientation_valid", value.orientation_valid),
        _boolean("valid", value.valid), status, 0)


def decode_imu_payload(data):
    """解码固定332字节IMU载荷；data经结构校验后复用编码器执行语义复验。"""
    # 解码三阶段：encoded是固定332字节原始payload；values是_IMU解包后的数值槽、
    # frame_id槽和状态槽；result是具名ImuPayload，随后重新编码复验有限值与枚举。
    encoded = _bytes(data)
    if len(encoded) != _IMU.size:
        raise ProtocolError("IMU payload must be 332 bytes")
    values = _IMU.unpack(encoded)
    frame_bytes = values[37]
    # frame_id槽内第一个NUL的位置；必须至少保留一个非空名称字节。
    terminator = frame_bytes.find(b"\0")
    if terminator <= 0:
        raise ProtocolError("frame_id must be NUL-terminated and non-empty")
    try:
        # 去掉NUL填充后的调用方坐标系名称。
        frame_id = frame_bytes[:terminator].decode("utf-8")
    # error保留线缆frame_id不是合法UTF-8时的原始UnicodeDecodeError因果链。
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
    """编码主参考二维状态；value提供位置/速度、2×2协方差、有效位和能力位。"""
    if not isinstance(value, LocalizationPayload):
        raise ProtocolError("localization value has wrong type")
    # 三个独立协方差元素，编码前须满足对称2×2半正定约束。
    cov_xx = _finite("cov_xx", value.cov_xx)
    cov_xy = _finite("cov_xy", value.cov_xy)
    cov_yy = _finite("cov_yy", value.cov_yy)
    if not _positive_semidefinite_2x2(cov_xx, cov_xy, cov_yy):
        raise ProtocolError("localization covariance is not positive semidefinite")
    # _LOCALIZATION前七个double的精确展开次序。
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
    """解码64字节定位载荷；data转换为LocalizationPayload后复用编码器验证。"""
    # 解码三阶段：encoded是固定64字节原始payload；values是_LOCALIZATION解包槽，
    # 尾部布尔字节尚待校验；result是具名LocalizationPayload和交叉约束复验输入。
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
    """编码动态拓扑；value提供节点/边计数、连通性、可观性和原因位。"""
    if not isinstance(value, NetworkPayload):
        raise ProtocolError("network value has wrong type")
    # 规范化后的拓扑计数，后续用于计算图论上允许的边数区间。
    node_count = _uint("node_count", value.node_count, 32)
    reachable_node_count = _uint(
        "reachable_node_count", value.reachable_node_count, 32
    )
    active_edge_count = _uint(
        "active_edge_count", value.active_edge_count, 32
    )
    # 全部已声明节点均可达的规范化布尔声明。
    connected = _boolean("connected", value.connected)
    # 当前网络几何满足定位可观条件的规范化布尔声明。
    observable = _boolean("observable", value.observable)
    if reachable_node_count > node_count:
        raise ProtocolError("reachable_node_count exceeds node_count")
    # 可达子图连通所需的最少边数，以及全体节点简单无向图的最多边数。
    minimum_edge_count = max(0, reachable_node_count - 1)
    maximum_edge_count = node_count * max(0, node_count - 1) // 2
    if (active_edge_count < minimum_edge_count
            or active_edge_count > maximum_edge_count):
        raise ProtocolError("active_edge_count is inconsistent with topology")
    # connected字段必须与“所有已声明节点均可达”这一协议定义一致。
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
    """解码20字节网络快照；data中的保留位和拓扑交叉关系均严格校验。"""
    # 解码三阶段：encoded是固定20字节原始payload；values是_NETWORK的计数、
    # 布尔/状态及保留槽；result是具名NetworkPayload，随后复验拓扑关系。
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
    """编码单边滑窗统计；value提供窗口、计数、质量比例、融合动作和原因位。"""
    if not isinstance(value, ObservationPayload):
        raise ProtocolError("observation value has wrong type")
    # 规范化后的质量指标，比例限定[0,1]，频率非负，协方差倍数为正。
    nlos_ratio = _unit("nlos_ratio", value.nlos_ratio)
    valid_ratio = _unit("valid_ratio", value.valid_ratio)
    actual_rate_hz = _finite("actual_rate_hz", value.actual_rate_hz)
    covariance_scale = _finite("covariance_scale", value.covariance_scale)
    if actual_rate_hz < 0.0 or covariance_scale <= 0.0:
        raise ProtocolError("rate/scale is outside its valid range")
    # 窗口边界与关键计数，后续逐项检查包含关系和时间顺序。
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
    """解码80字节边观测载荷；data中的保留位和窗口计数关系均严格校验。"""
    # 解码三阶段：encoded是固定80字节原始payload；values是_OBSERVATION的窗口、
    # 计数、质量及状态槽；result是具名ObservationPayload，随后复验比例和计数关系。
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
    """编码网络状态告警；value为AlertPayload，ACTIVE有原因且CLEARED清空原因。"""
    if not isinstance(value, AlertPayload):
        raise ProtocolError("alert value has wrong type")
    # v1仅允许NETWORK_STATE代码及算法来源，拒绝将未知告警静默透传。
    alert_code = _uint("alert_code", value.alert_code, 32)
    if alert_code != ALERT_CODE_NETWORK_STATE:
        raise ProtocolError("alert_code is unknown")
    # 规范化后的告警严重度枚举。
    level = _enum("alert level", value.level, ALERT_LEVEL_CRITICAL)
    # 规范化后的ACTIVE/CLEARED告警生命周期枚举。
    lifecycle = _enum(
        "alert lifecycle", value.lifecycle, ALERT_LIFECYCLE_CLEARED
    )
    source = _uint("alert source", value.source, 8)
    if source != ALERT_SOURCE_ALGORITHM:
        raise ProtocolError("alert source is unknown")
    # 生命周期与原因位必须联动，构成激活/清除事件的最小证据。
    reason_mask = _uint("reason_mask", value.reason_mask, 32)
    if lifecycle == ALERT_LIFECYCLE_ACTIVE and reason_mask == 0:
        raise ProtocolError("active NETWORK_STATE alert requires a reason")
    if lifecycle == ALERT_LIFECYCLE_CLEARED and reason_mask != 0:
        raise ProtocolError("cleared NETWORK_STATE alert must clear reasons")
    # 告警首次激活与本次更新时刻，均为Unix纪元纳秒。
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
    """解码40字节告警载荷；data的所有保留字段、枚举和生命周期关系均校验。"""
    # 解码三阶段：encoded是固定40字节原始payload；values是_ALERT布局含三个保留槽
    # 的解包结果；result是去除保留槽的具名AlertPayload，随后复验生命周期关系。
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
    """编码算法状态；value提供固定ABI、模式/状态、累计计数与运行时长。"""
    if not isinstance(value, AlgorithmStatusPayload):
        raise ProtocolError("algorithm status value has wrong type")
    # ABI版本是解释后续布局的硬门槛，不接受兼容性猜测。
    abi_version = _uint("abi_version", value.abi_version, 32)
    if abi_version != ALGORITHM_STATUS_ABI_VERSION:
        raise ProtocolError("algorithm status ABI version is unsupported")
    # 算法模式仅允许当前实现的UWB平面或IMU/UWB十五状态模式。
    mode = _uint("algorithm mode", value.mode, 8)
    if mode not in (ALGORITHM_MODE_UWB_ONLY_PLANAR,
                    ALGORITHM_MODE_IMU_UWB_15_STATE):
        raise ProtocolError("algorithm mode is unknown")
    # 规范化后的运行生命周期枚举。
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
    """解码48字节算法状态；data的ABI、保留字段、模式和计数均严格校验。"""
    # 解码三阶段：encoded是固定48字节原始payload；values是_ALGORITHM_STATUS的
    # 版本、枚举、保留槽与计数；result是具名AlgorithmStatusPayload和ABI复验输入。
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
    """验证CRC32标准向量和关键Struct尺寸，供直接运行模块时快速自检。"""
    # IEEE CRC32标准测试向量的计算结果。
    check = zlib.crc32(b"123456789") & 0xFFFFFFFF
    if (check != 0xCBF43926 or _HEADER_PREFIX.size != 36
            or _ALERT.size != 40 or _ALGORITHM_STATUS.size != 48):
        raise RuntimeError("ZJCL protocol self-check failed")
    print("ZJCL_PROTOCOL_SELFTEST PASS")


if __name__ == "__main__":
    _self_check()

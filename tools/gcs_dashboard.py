#!/usr/bin/env python3
"""ZJCL协同定位结果的零第三方依赖二维在线演示面板。

模块接收临时UDP输出，在锁保护下维护最新定位、网络、观测、状态和告警快照，
再通过本机HTTP页面展示。它用于当前GCS联调，不是交大最终GCS实现或算法输入端。
"""

import argparse
import copy
import json
import signal
import socket
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import zjcl_protocol as protocol


# 协议枚举到面板中文标签的只读映射；缺失值由严格解码器提前拒绝。
_LOCALIZATION_STATES = {
    protocol.LOCALIZATION_UNINITIALIZED: "未初始化",
    protocol.LOCALIZATION_NORMAL: "正常",
    protocol.LOCALIZATION_DEGRADED: "降级",
    protocol.LOCALIZATION_UNOBSERVABLE: "不可观",
    protocol.LOCALIZATION_STALE: "数据陈旧",
}

# 边观测质量协议枚举到GCS中文展示标签的映射，仅展示而不改变协议值。
_OBSERVATION_STATES = {
    protocol.OBSERVATION_UNKNOWN: "未知",
    protocol.OBSERVATION_NORMAL: "正常",
    protocol.OBSERVATION_DEGRADED: "降级",
    protocol.OBSERVATION_SUSPENDED: "暂缓",
    protocol.OBSERVATION_REJECTED: "剔除",
    protocol.OBSERVATION_RECOVERING: "恢复中",
}

# 融合动作协议枚举到GCS中文展示标签的映射，仅展示而不改变协议值。
_FUSION_ACTIONS = {
    protocol.FUSION_USE_NORMAL: "正常使用",
    protocol.FUSION_USE_DOWNWEIGHTED: "降权使用",
    protocol.FUSION_HOLD: "暂缓",
    protocol.FUSION_REJECT: "剔除",
    protocol.FUSION_TRIAL_RECOVERY: "试探恢复",
}

# 算法运行状态协议枚举到GCS中文展示标签的映射，仅展示而不改变协议值。
_ALGORITHM_RUN_STATES = {
    protocol.ALGORITHM_RUN_INITIALIZING: "初始化中",
    protocol.ALGORITHM_RUN_RUNNING: "运行中",
    protocol.ALGORITHM_RUN_DEGRADED: "降级运行",
    protocol.ALGORITHM_RUN_ERROR: "错误",
    protocol.ALGORITHM_RUN_STOPPED: "已停止",
}

# 算法模式数值与状态帧冻结一致，用于区分仅测距回退和默认15维惯性融合。
_ALGORITHM_MODES = {
    protocol.ALGORITHM_MODE_UWB_ONLY_PLANAR: "仅测距平面",
    protocol.ALGORITHM_MODE_IMU_UWB_15_STATE: "IMU＋测距15维",
}

# 告警严重度协议枚举到GCS中文展示标签的映射，仅展示而不改变协议值。
_ALERT_LEVELS = {
    protocol.ALERT_LEVEL_INFO: "提示",
    protocol.ALERT_LEVEL_WARNING: "警告",
    protocol.ALERT_LEVEL_ERROR: "错误",
    protocol.ALERT_LEVEL_CRITICAL: "严重",
}

# 告警生命周期协议枚举到GCS中文展示标签的映射，仅展示而不改变协议值。
_ALERT_LIFECYCLES = {
    protocol.ALERT_LIFECYCLE_ACTIVE: "活动",
    protocol.ALERT_LIFECYCLE_CLEARED: "已清除",
}

# 观测状态到前端边线颜色的固定调色板。
_QUALITY_COLORS = {
    protocol.OBSERVATION_UNKNOWN: "#94a3b8",
    protocol.OBSERVATION_NORMAL: "#22c55e",
    protocol.OBSERVATION_DEGRADED: "#f59e0b",
    protocol.OBSERVATION_SUSPENDED: "#ef4444",
    protocol.OBSERVATION_REJECTED: "#7f1d1d",
    protocol.OBSERVATION_RECOVERING: "#38bdf8",
}

# reason_mask位序到退化原因文本的协议展示契约。
_REASON_BITS = (
    (0, "非视距比例高"),
    (1, "有效观测比例低"),
    (2, "观测频率低"),
    (3, "测距残差高"),
    (4, "时间同步超时"),
    (5, "链路超时"),
    (6, "图几何退化"),
    (7, "节点不可达"),
    (8, "缺少初始化"),
    (9, "输入队列溢出"),
)


def _reason_texts(mask):
    """展开原因位图；mask是协议uint32 reason_mask，未知位会保留为十六进制证据。"""
    # 已知且置位的原因文本；bit/text分别来自_REASON_BITS位序与标签。
    reasons = [text for bit, text in _REASON_BITS if mask & (1 << bit)]
    # 当前面板认识的全部原因位；bit用于置位，下划线刻意忽略此处不需要的原因文本。
    known_mask = sum(1 << bit for bit, _ in _REASON_BITS)
    # 不在当前映射中的置位部分，不能静默丢弃。
    unknown_mask = mask & ~known_mask
    if unknown_mask:
        reasons.append(f"未知原因位 0x{unknown_mask:08X}")
    return reasons


class DashboardState:
    """线程安全遥测仓库；普通帧逐项更新，Pose2D收齐同批车辆后整体更新。"""

    def __init__(self, expected_pose_nodes=3):
        """建立状态仓库；expected_pose_nodes是组成一个原子Pose2D批次的车辆数。"""
        if (isinstance(expected_pose_nodes, bool)
                or not isinstance(expected_pose_nodes, int)
                or expected_pose_nodes < 1
                or expected_pose_nodes > 64):
            raise ValueError("expected_pose_nodes must be an integer in [1,64]")
        # 所有快照成员的唯一同步锁；UDP写线程与HTTP读线程共同遵守。
        self._lock = threading.Lock()
        # 按节点字符串ID索引的最新定位值，随DashboardState生命周期持续更新。
        self._nodes = {}
        # 按规范化“较小节点-较大节点”键索引的最新边观测值。
        self._edges = {}
        # Pose2D按(统一时间,参考节点)暂存逐车帧；收齐期望车辆数后才原子提交。
        self._expected_pose_nodes = expected_pose_nodes
        self._pose_batches = {}
        self._latest_pose_batch_timestamp = -1
        self._latest_pose_reference = None
        # 最新网络快照；锁内由UDP接收线程整体替换，初值表示未初始化。
        self._network = {
            "node_count": 0,
            "reachable_node_count": 0,
            "active_edge_count": 0,
            "connected": False,
            "observable": False,
            "state": protocol.LOCALIZATION_UNINITIALIZED,
            "state_text": _LOCALIZATION_STATES[protocol.LOCALIZATION_UNINITIALIZED],
            "reason_mask": 0,
            "reasons": [],
            "timestamp_ns": 0,
            "sequence": 0,
            "source_node": 0,
        }
        # 最新算法状态；锁内整体替换，累计计数生命周期由远端进程定义。
        self._algorithm_status = {
            "abi_version": 0,
            "software_version_packed": 0,
            "software_version": "--",
            "mode": 0,
            "mode_text": "--",
            "run_state": protocol.ALGORITHM_RUN_INITIALIZING,
            "run_state_text": _ALGORITHM_RUN_STATES[
                protocol.ALGORITHM_RUN_INITIALIZING
            ],
            "accepted_ranges": 0,
            "rejected_ranges": 0,
            "protocol_errors": 0,
            "uptime_ns": 0,
            "timestamp_ns": 0,
            "sequence": 0,
            "source_node": 0,
        }
        # 最新告警生命周期快照；锁内整体替换，初值显式表示无活动告警。
        self._alert = {
            "alert_code": 0,
            "level": protocol.ALERT_LEVEL_INFO,
            "level_text": "--",
            "lifecycle": protocol.ALERT_LIFECYCLE_CLEARED,
            "lifecycle_text": "无",
            "active": False,
            "reason_mask": 0,
            "reasons": [],
            "node_id": 0,
            "from_node": 0,
            "to_node": 0,
            "first_timestamp_ns": 0,
            "last_timestamp_ns": 0,
            "timestamp_ns": 0,
            "sequence": 0,
            "source_node": 0,
        }
        # 本面板进程生命周期内的接收分类计数和最后接收Unix纳秒时间。
        self._stats = {
            "datagrams": 0,
            "accepted": 0,
            "ignored": 0,
            "rejected": 0,
            "last_receive_time_ns": 0,
        }

    def _count(self, category):
        """原子记录非接受数据报；category必须是ignored或rejected统计键。"""
        with self._lock:
            self._stats["datagrams"] += 1
            self._stats[category] += 1
            self._stats["last_receive_time_ns"] = time.time_ns()

    def _commit_pose2d_batch_locked(self, timestamp_ns, reference_node_id,
                                    batch):
        """在已持有self._lock时，把一个完整同历元批次一次性写入全部车辆。"""
        for source_node, (decoded, sequence, received_at) in batch.items():
            key = str(source_node)
            old = self._nodes.get(key, {})
            position_valid = bool(
                decoded.position_valid
                and decoded.capability_mask
                & protocol.CAPABILITY_PLANAR_POSITION
            )
            yaw_valid = bool(
                decoded.yaw_valid
                and decoded.capability_mask & protocol.CAPABILITY_YAW
            )
            merged = {
                "node_id": source_node,
                "reference_node_id": reference_node_id,
                "vx": 0.0,
                "vy": 0.0,
                "cov_xx": 0.0,
                "cov_xy": 0.0,
                "cov_yy": 0.0,
                "state": protocol.LOCALIZATION_UNINITIALIZED,
                "state_text": _LOCALIZATION_STATES[
                    protocol.LOCALIZATION_UNINITIALIZED
                ],
                **old,
            }
            # 完整批次中的航向状态总是更新；无效时前端立即隐藏方向箭头。
            merged.update({
                "node_id": source_node,
                "reference_node_id": reference_node_id,
                "yaw_rad": decoded.yaw_rad,
                "yaw_valid": yaw_valid,
                "capability_mask": decoded.capability_mask,
                "pose_message_timestamp_ns": timestamp_ns,
                "pose_message_sequence": sequence,
                "receive_time_ns": received_at,
            })
            if position_valid:
                merged.update({
                    "x": decoded.x,
                    "y": decoded.y,
                    "position_valid": True,
                    "valid": True,
                    "timestamp_ns": timestamp_ns,
                    "sequence": sequence,
                    "pose_timestamp_ns": timestamp_ns,
                    "pose_position_authoritative": True,
                })
            elif old.get("pose_position_authoritative", False):
                # 已有105位置时，完整的新无效批次关闭显示并阻止旧100覆盖当前状态。
                merged.update({
                    "position_valid": False,
                    "valid": False,
                    "timestamp_ns": timestamp_ns,
                    "sequence": sequence,
                    "pose_timestamp_ns": timestamp_ns,
                    "pose_position_authoritative": True,
                })
            else:
                # 启动阶段尚无有效105时，允许有效100继续提供临时二维位置。
                merged.update({
                    "position_valid": old.get("position_valid", False),
                    "valid": old.get("valid", False),
                    "pose_position_authoritative": False,
                })
            self._nodes[key] = merged

        self._latest_pose_batch_timestamp = timestamp_ns
        self._latest_pose_reference = reference_node_id
        # 已提交历元及其更早的不完整批次不会再有使用价值，及时释放以限制内存。
        self._pose_batches = {
            key: value for key, value in self._pose_batches.items()
            if key[0] > timestamp_ns
        }

    def ingest_datagram(self, data):
        """严格解码一帧；data是单个UDP数据报，坏帧/非GCS类型不覆盖有效快照。"""
        # 阶段1：公共帧CRC/长度通过后，再按消息类型解析固定载荷。
        try:
            # 已通过公共头长度、类型和CRC验证的完整业务帧。
            frame = protocol.decode_frame(data, udp=True)
        except (protocol.ProtocolError, TypeError, ValueError):
            self._count("rejected")
            return False

        if frame.message_type not in (
            protocol.MSG_LOCALIZATION,
            protocol.MSG_POSE2D,
            protocol.MSG_NETWORK,
            protocol.MSG_OBSERVATION,
            protocol.MSG_ALGORITHM_STATUS,
            protocol.MSG_ALERT,
        ):
            self._count("ignored")
            return False

        try:
            if frame.message_type == protocol.MSG_LOCALIZATION:
                # decoded是类型专属载荷对象，update_kind选择唯一待替换的快照。
                decoded = protocol.decode_localization_payload(frame.payload)
                update_kind = "localization"
            elif frame.message_type == protocol.MSG_POSE2D:
                # decoded是新增105二维位姿；公共头source/target分别是车辆和参考节点。
                decoded = protocol.decode_pose2d_payload(frame.payload)
                update_kind = "pose2d"
            elif frame.message_type == protocol.MSG_NETWORK:
                decoded = protocol.decode_network_payload(frame.payload)
                update_kind = "network"
            elif frame.message_type == protocol.MSG_OBSERVATION:
                decoded = protocol.decode_observation_payload(frame.payload)
                update_kind = "observation"
            elif frame.message_type == protocol.MSG_ALGORITHM_STATUS:
                decoded = protocol.decode_algorithm_status_payload(frame.payload)
                update_kind = "algorithm_status"
            else:
                decoded = protocol.decode_alert_payload(frame.payload)
                update_kind = "alert"
        except (protocol.ProtocolError, TypeError, ValueError):
            self._count("rejected")
            return False

        # 阶段2：载荷完全合法后才获取锁并替换单个最新值；解析异常不会造成半更新。
        # 本机接收完成的Unix纳秒时间，与远端frame.timestamp_ns分开保存。
        received_at = time.time_ns()
        with self._lock:
            if update_kind == "localization":
                # old保存同节点已有Pose2D字段；旧100帧只补充速度、协方差和状态。
                key = str(frame.source_node)
                old = self._nodes.get(key, {})
                merged = dict(old)
                merged.update({
                    "node_id": frame.source_node,
                    "vx": decoded.vx,
                    "vy": decoded.vy,
                    "cov_xx": decoded.cov_xx,
                    "cov_xy": decoded.cov_xy,
                    "cov_yy": decoded.cov_yy,
                    "state": decoded.state,
                    "state_text": _LOCALIZATION_STATES[decoded.state],
                    "localization_valid": decoded.valid,
                    "localization_capability_mask": decoded.capability_mask,
                    "localization_timestamp_ns": frame.timestamp_ns,
                    "localization_sequence": frame.sequence,
                    "receive_time_ns": received_at,
                })
                # 在尚未收到有效105位置时，沿用旧100位置并明确航向无效。
                # 无效105只报告诊断状态，不得永久压住后续有效100位置。
                if not old.get("pose_position_authoritative", False):
                    merged.update({
                        "reference_node_id": frame.target_node,
                        "x": decoded.x,
                        "y": decoded.y,
                        "yaw_rad": 0.0,
                        "position_valid": decoded.valid,
                        "yaw_valid": False,
                        "valid": decoded.valid,
                        "capability_mask": decoded.capability_mask,
                        "timestamp_ns": frame.timestamp_ns,
                        "sequence": frame.sequence,
                    })
                self._nodes[key] = merged
            elif update_kind == "pose2d":
                batch_key = (frame.timestamp_ns, frame.target_node)
                # 已提交时间及更早的UDP乱序帧只计作合法接收，不能回退当前三车快照。
                if frame.timestamp_ns > self._latest_pose_batch_timestamp:
                    batch = self._pose_batches.setdefault(batch_key, {})
                    batch[frame.source_node] = (
                        decoded,
                        frame.sequence,
                        received_at,
                    )
                    if len(batch) == self._expected_pose_nodes:
                        self._commit_pose2d_batch_locked(
                            frame.timestamp_ns,
                            frame.target_node,
                            batch,
                        )
                    elif len(self._pose_batches) > 32:
                        # 极端丢包时仅保留最新32个候选历元，防止无界增长。
                        oldest = min(self._pose_batches)
                        del self._pose_batches[oldest]
            elif update_kind == "network":
                self._network = {
                    "node_count": decoded.node_count,
                    "reachable_node_count": decoded.reachable_node_count,
                    "active_edge_count": decoded.active_edge_count,
                    "connected": decoded.connected,
                    "observable": decoded.observable,
                    "state": decoded.state,
                    "state_text": _LOCALIZATION_STATES[decoded.state],
                    "reason_mask": decoded.reason_mask,
                    "reasons": _reason_texts(decoded.reason_mask),
                    "timestamp_ns": frame.timestamp_ns,
                    "sequence": frame.sequence,
                    "source_node": frame.source_node,
                    "receive_time_ns": received_at,
                }
            elif update_kind == "observation":
                # 无向边的规范端点顺序，保证反向上报仍覆盖同一面板条目。
                first = min(frame.source_node, frame.target_node)
                second = max(frame.source_node, frame.target_node)
                self._edges[f"{first}-{second}"] = {
                    "from_node": frame.source_node,
                    "to_node": frame.target_node,
                    "window_start_ns": decoded.window_start_ns,
                    "window_end_ns": decoded.window_end_ns,
                    "expected_count": decoded.expected_count,
                    "received_count": decoded.received_count,
                    "valid_count": decoded.valid_count,
                    "nlos_count": decoded.nlos_count,
                    "residual_rejected_count": decoded.residual_rejected_count,
                    "dropped_count": decoded.dropped_count,
                    "nlos_ratio": decoded.nlos_ratio,
                    "valid_ratio": decoded.valid_ratio,
                    "actual_rate_hz": decoded.actual_rate_hz,
                    "covariance_scale": decoded.covariance_scale,
                    "state": decoded.state,
                    "state_text": _OBSERVATION_STATES[decoded.state],
                    "action": decoded.action,
                    "action_text": _FUSION_ACTIONS[decoded.action],
                    "quality_color": _QUALITY_COLORS[decoded.state],
                    "input_overflow": decoded.input_overflow,
                    "reason_mask": decoded.reason_mask,
                    "reasons": _reason_texts(decoded.reason_mask),
                    "timestamp_ns": frame.timestamp_ns,
                    "sequence": frame.sequence,
                    "receive_time_ns": received_at,
                }
            elif update_kind == "algorithm_status":
                # 远端压缩版本整数及供人阅读的major.minor.patch文本。
                packed = decoded.software_version_packed
                version = f"{packed >> 16}.{(packed >> 8) & 0xFF}.{packed & 0xFF}"
                self._algorithm_status = {
                    "abi_version": decoded.abi_version,
                    "software_version_packed": packed,
                    "software_version": version,
                    "mode": decoded.mode,
                    "mode_text": _ALGORITHM_MODES[decoded.mode],
                    "run_state": decoded.run_state,
                    "run_state_text": _ALGORITHM_RUN_STATES[decoded.run_state],
                    "accepted_ranges": decoded.accepted_ranges,
                    "rejected_ranges": decoded.rejected_ranges,
                    "protocol_errors": decoded.protocol_errors,
                    "uptime_ns": decoded.uptime_ns,
                    "timestamp_ns": frame.timestamp_ns,
                    "sequence": frame.sequence,
                    "source_node": frame.source_node,
                    "receive_time_ns": received_at,
                }
            else:
                self._alert = {
                    "alert_code": decoded.alert_code,
                    "level": decoded.level,
                    "level_text": _ALERT_LEVELS[decoded.level],
                    "lifecycle": decoded.lifecycle,
                    "lifecycle_text": _ALERT_LIFECYCLES[decoded.lifecycle],
                    "active": decoded.lifecycle == protocol.ALERT_LIFECYCLE_ACTIVE,
                    "reason_mask": decoded.reason_mask,
                    "reasons": _reason_texts(decoded.reason_mask),
                    "node_id": decoded.node_id,
                    "from_node": decoded.from_node,
                    "to_node": decoded.to_node,
                    "first_timestamp_ns": decoded.first_timestamp_ns,
                    "last_timestamp_ns": decoded.last_timestamp_ns,
                    "timestamp_ns": frame.timestamp_ns,
                    "sequence": frame.sequence,
                    "source_node": frame.source_node,
                    "receive_time_ns": received_at,
                }
            self._stats["datagrams"] += 1
            self._stats["accepted"] += 1
            self._stats["last_receive_time_ns"] = received_at
        return True

    def snapshot(self):
        """返回深拷贝JSON快照，避免HTTP线程观察到UDP线程的半更新状态。"""
        with self._lock:
            return {
                "nodes": copy.deepcopy(self._nodes),
                "edges": copy.deepcopy(self._edges),
                "network": copy.deepcopy(self._network),
                "algorithm_status": copy.deepcopy(self._algorithm_status),
                "alert": copy.deepcopy(self._alert),
                "stats": copy.deepcopy(self._stats),
            }


class UdpReceiver(threading.Thread):
    """可由其他线程停止的后台UDP接收器；短超时限制退出等待时间。"""

    def __init__(self, state, bind_address, port):
        """创建后台接收线程。

        state是线程安全快照仓库；bind_address与port是UDP监听端点，port可为0以动态分配。
        """
        super().__init__(name="zjcl-gcs-udp", daemon=True)
        # 外部共享状态引用；只由本接收线程调用ingest_datagram。
        self._state = state
        # 跨线程停止信号，由控制线程设置、接收线程轮询。
        self._stop_event = threading.Event()
        # 本接收线程独占读取、控制线程可在stop中关闭的UDP套接字。
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((bind_address, port))
            self._socket.settimeout(0.2)
        except Exception:
            self._socket.close()
            raise
        # 套接字生命周期内实际绑定端口，支持调用方请求动态端口0。
        self.port = self._socket.getsockname()[1]

    def run(self):
        """线程主体：保持UDP数据报边界并把每个data交给共享state。"""
        # 阶段2：保持数据报边界，把协议语义交给DashboardState集中处理。
        while not self._stop_event.is_set():
            try:
                # data是单个UDP数据报；未使用的地址元组不参与协议语义。
                data, _ = self._socket.recvfrom(protocol.MAX_UDP_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    break
                continue
            self._state.ingest_datagram(data)

    def stop(self):
        """由控制线程请求停止并关闭接收套接字；可在未启动或已退出时调用。"""
        # 先设置事件再关闭socket，使正常stop与运行时OSError可以可靠区分。
        self._stop_event.set()
        try:
            self._socket.close()
        except OSError:
            pass


# 浏览器端单页应用源码；只消费/api/state，不向服务端写入状态。
_DASHBOARD_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>三车协同定位在线演示</title>
<style>
:root { color-scheme: dark; font-family: "Microsoft YaHei", system-ui, sans-serif; }
* { box-sizing: border-box; }
body { margin: 0; background: #07111f; color: #e2e8f0; }
header { padding: 18px 24px 10px; }
h1 { margin: 0; font-size: 24px; }
.sub { margin-top: 5px; color: #94a3b8; font-size: 13px; }
.cards { display: grid; grid-template-columns: repeat(4, minmax(130px, 1fr)); gap: 10px; padding: 10px 24px; }
.card { background: #0f1d2e; border: 1px solid #21334a; border-radius: 10px; padding: 12px; }
.label { color: #94a3b8; font-size: 12px; }
.value { margin-top: 5px; font-size: 20px; font-weight: 700; }
.reasons { padding: 0 24px 10px; color: #fbbf24; min-height: 28px; }
.alert-line { margin: 0 24px 10px; padding: 10px 12px; background: #0f1d2e; border: 1px solid #21334a; border-radius: 8px; color: #cbd5e1; }
.alert-line.active { border-color: #f59e0b; color: #fbbf24; }
.stage { margin: 0 24px; background: #0b1726; border: 1px solid #21334a; border-radius: 12px; overflow: hidden; }
canvas { display: block; width: 100%; height: min(62vh, 620px); }
.legend { display: flex; flex-wrap: wrap; gap: 15px; padding: 12px 24px 20px; color: #cbd5e1; font-size: 13px; }
.dot { width: 10px; height: 10px; display: inline-block; border-radius: 50%; margin-right: 5px; }
@media (max-width: 720px) { .cards { grid-template-columns: repeat(2, 1fr); } }
</style>
</head>
<body>
<header>
  <h1>三车协同定位在线演示</h1>
  <div class="sub">主参考平台 ENU 坐标系 · 二维位置与航向 · 质量状态实时刷新</div>
</header>
<section class="cards">
  <div class="card"><div class="label">网络连通</div><div class="value" id="connected">--</div></div>
  <div class="card"><div class="label">几何可观</div><div class="value" id="observable">--</div></div>
  <div class="card"><div class="label">系统状态</div><div class="value" id="state">--</div></div>
  <div class="card"><div class="label">节点 / 有效边</div><div class="value" id="counts">--</div></div>
</section>
<section class="cards">
  <div class="card"><div class="label">算法版本 / 状态</div><div class="value" id="algorithm-version">--</div></div>
  <div class="card"><div class="label">接收 / 拒绝测距</div><div class="value" id="range-counts">--</div></div>
  <div class="card"><div class="label">协议错误</div><div class="value" id="protocol-errors">--</div></div>
  <div class="card"><div class="label">运行时间</div><div class="value" id="uptime">--</div></div>
</section>
<section class="cards">
  <div class="card"><div class="label">参考节点</div><div class="value" id="reference-node">--</div></div>
  <div class="card"><div class="label">位姿快照时间</div><div class="value" id="pose-time">--</div></div>
  <div class="card"><div class="label">坐标系</div><div class="value" id="pose-frame">--</div></div>
  <div class="card"><div class="label">有效位姿节点</div><div class="value" id="pose-count">--</div></div>
</section>
<div class="reasons" id="reasons">原因：等待数据</div>
<div class="alert-line" id="alert-line">网络告警：等待算法状态</div>
<main class="stage"><canvas id="map"></canvas></main>
<footer class="legend">
  <span><i class="dot" style="background:#22c55e"></i>正常</span>
  <span><i class="dot" style="background:#f59e0b"></i>降级</span>
  <span><i class="dot" style="background:#ef4444"></i>暂缓</span>
  <span><i class="dot" style="background:#7f1d1d"></i>剔除</span>
  <span><i class="dot" style="background:#38bdf8"></i>恢复中</span>
</footer>
<script>
const canvas = document.getElementById("map");
const context = canvas.getContext("2d");

function fitCanvas() {
  const ratio = window.devicePixelRatio || 1;
  const box = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.round(box.width * ratio));
  canvas.height = Math.max(1, Math.round(box.height * ratio));
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  return box;
}

function render(model) {
  const box = fitCanvas();
  const width = box.width;
  const height = box.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#0b1726";
  context.fillRect(0, 0, width, height);

  // 只让位置有效的节点参与缩放、连边和绘图，无效占位值不污染画布范围。
  const nodes = Object.values(model.nodes).filter(node => node.position_valid);
  if (!nodes.length) {
    context.fillStyle = "#94a3b8";
    context.font = "16px Microsoft YaHei";
    context.textAlign = "center";
    context.fillText("等待协同定位输出", width / 2, height / 2);
    return;
  }

  let minX = Math.min(...nodes.map(node => node.x));
  let maxX = Math.max(...nodes.map(node => node.x));
  let minY = Math.min(...nodes.map(node => node.y));
  let maxY = Math.max(...nodes.map(node => node.y));
  const spanX = Math.max(2, maxX - minX);
  const spanY = Math.max(2, maxY - minY);
  minX -= spanX * 0.25; maxX += spanX * 0.25;
  minY -= spanY * 0.25; maxY += spanY * 0.25;
  const point = node => ({
    x: 42 + (node.x - minX) / (maxX - minX) * (width - 84),
    y: height - 42 - (node.y - minY) / (maxY - minY) * (height - 84)
  });

  context.strokeStyle = "#182a40";
  context.lineWidth = 1;
  for (let index = 1; index < 10; ++index) {
    context.beginPath(); context.moveTo(width * index / 10, 0); context.lineTo(width * index / 10, height); context.stroke();
    context.beginPath(); context.moveTo(0, height * index / 10); context.lineTo(width, height * index / 10); context.stroke();
  }

  const byId = new Map(nodes.map(node => [node.node_id, node]));
  Object.values(model.edges).forEach(edge => {
    const first = byId.get(edge.from_node);
    const second = byId.get(edge.to_node);
    if (!first || !second) return;
    const a = point(first), b = point(second);
    context.strokeStyle = edge.quality_color;
    context.lineWidth = 5;
    context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
    context.fillStyle = edge.quality_color;
    context.font = "12px Microsoft YaHei";
    context.textAlign = "center";
    context.fillText(edge.state_text, (a.x + b.x) / 2, (a.y + b.y) / 2 - 8);
  });

  nodes.forEach(node => {
    const current = point(node);
    context.fillStyle = node.position_valid ? "#38bdf8" : "#64748b";
    context.beginPath(); context.arc(current.x, current.y, 14, 0, Math.PI * 2); context.fill();
    context.strokeStyle = "#e0f2fe"; context.lineWidth = 2; context.stroke();
    if (node.yaw_valid) {
      const arrowLength = 36;
      const tipX = current.x + Math.cos(node.yaw_rad) * arrowLength;
      const tipY = current.y - Math.sin(node.yaw_rad) * arrowLength;
      context.strokeStyle = "#f8fafc";
      context.fillStyle = "#f8fafc";
      context.lineWidth = 3;
      context.beginPath(); context.moveTo(current.x, current.y); context.lineTo(tipX, tipY); context.stroke();
      const screenAngle = -node.yaw_rad;
      context.beginPath();
      context.moveTo(tipX, tipY);
      context.lineTo(tipX - 9 * Math.cos(screenAngle - 0.45), tipY - 9 * Math.sin(screenAngle - 0.45));
      context.lineTo(tipX - 9 * Math.cos(screenAngle + 0.45), tipY - 9 * Math.sin(screenAngle + 0.45));
      context.closePath(); context.fill();
    }
    context.fillStyle = "#f8fafc";
    context.font = "bold 13px Microsoft YaHei";
    context.textAlign = "center";
    context.fillText(String(node.node_id), current.x, current.y + 5);
    context.font = "12px Microsoft YaHei";
    const yawText = node.yaw_valid ? `${(node.yaw_rad * 180 / Math.PI).toFixed(1)}°` : "--";
    context.fillText(`节点 ${node.node_id}  (${node.x.toFixed(2)}, ${node.y.toFixed(2)}) m  航向 ${yawText}`, current.x, current.y + 34);
  });
}

async function refresh() {
  try {
    const response = await fetch("/api/state", {cache: "no-store"});
    const model = await response.json();
    const network = model.network;
    const algorithm_status = model.algorithm_status;
    const alert = model.alert;
    document.getElementById("connected").textContent = network.connected ? "是" : "否";
    document.getElementById("observable").textContent = network.observable ? "是" : "否";
    document.getElementById("state").textContent = network.state_text;
    document.getElementById("counts").textContent = `${network.reachable_node_count}/${network.node_count} · ${network.active_edge_count}`;
    document.getElementById("reasons").textContent = `原因：${network.reasons.length ? network.reasons.join("、") : "无"}`;
    document.getElementById("algorithm-version").textContent = `${algorithm_status.software_version} / ${algorithm_status.mode_text} / ${algorithm_status.run_state_text}`;
    document.getElementById("range-counts").textContent = `${algorithm_status.accepted_ranges} / ${algorithm_status.rejected_ranges}`;
    document.getElementById("protocol-errors").textContent = String(algorithm_status.protocol_errors);
    document.getElementById("uptime").textContent = `${(algorithm_status.uptime_ns / 1e9).toFixed(1)} s`;
    const validPoses = Object.values(model.nodes).filter(
      node => node.position_valid && node.pose_position_authoritative
    );
    const referenceIds = [...new Set(validPoses.map(node => node.reference_node_id))];
    const poseTimes = [...new Set(validPoses.map(node => node.pose_timestamp_ns))];
    const referenceId = referenceIds.length === 1 ? referenceIds[0] : null;
    const poseTime = poseTimes.length === 1 ? poseTimes[0] : null;
    document.getElementById("reference-node").textContent = referenceId === null ? "--" : String(referenceId);
    document.getElementById("pose-time").textContent = poseTime === null ? "--" : `${poseTime} ns`;
    document.getElementById("pose-frame").textContent = referenceId === null ? "--" : `coop_ref_${referenceId}_enu`;
    document.getElementById("pose-count").textContent = String(validPoses.length);
    const alertLine = document.getElementById("alert-line");
    const alertReasons = alert.reasons.length ? alert.reasons.join("、") : "无";
    alertLine.textContent = `网络告警：${alert.lifecycle_text} / ${alert.level_text}；原因：${alertReasons}`;
    alertLine.classList.toggle("active", alert.active);
    render(model);
  } catch (error) {
    document.getElementById("reasons").textContent = "原因：数据服务暂不可用";
  }
}

window.addEventListener("resize", refresh);
setInterval(refresh, 300);
refresh();
</script>
</body>
</html>
"""


class _DashboardHttpServer(ThreadingHTTPServer):
    # 每个HTTP请求线程随服务退出，无需阻塞主线程回收。
    daemon_threads = True
    # 测试和短时联调可立即复用刚释放的HTTP监听端口。
    allow_reuse_address = True


def create_http_server(bind_address, port, state):
    """创建但不启动只读HTTP服务。

    bind_address与port指定HTTP监听端点，port可为0；state为线程安全动态快照源。
    """
    """Create, but do not start, the dashboard HTTP server."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            """响应当前请求；self由HTTP服务器线程持有，只提供页面与状态JSON。"""
            # 去除查询参数后的路由路径。
            path = self.path.partition("?")[0]
            if path == "/api/state":
                # 当前锁一致快照的紧凑UTF-8 JSON响应体。
                body = json.dumps(
                    state.snapshot(), ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
            elif path in ("/", "/index.html"):
                # 内嵌单页应用的UTF-8响应体。
                body = _DASHBOARD_PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
            else:
                # 未知路由的固定404响应体。
                body = b"not found"
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *args):
            """关闭默认访问日志；_format和args由BaseHTTPRequestHandler传入但有意忽略。"""
            del args

    return _DashboardHttpServer((bind_address, port), Handler)


def _port(value):
    """argparse端口转换器；value须为0..65535，0表示请求系统动态分配。"""
    try:
        # 命令行词法值转换后的候选端口号。
        result = int(value)
    # error保留端口文本无法转成整数时的原始ValueError。
    except ValueError as error:
        raise argparse.ArgumentTypeError("端口必须是整数") from error
    if result < 0 or result > 65535:
        raise argparse.ArgumentTypeError("端口必须在 0 到 65535 之间")
    return result


def _duration(value):
    """把value解析为浮点秒数并拒绝小于0的值，0表示持续运行。

    当前实现未额外拒绝NaN或正无穷，这是命令行校验的已知限制。
    """
    try:
        # 命令行词法值转换后的运行秒数。
        result = float(value)
    # error保留运行时长文本无法转成浮点秒数时的原始ValueError。
    except ValueError as error:
        raise argparse.ArgumentTypeError("运行时长必须是数字") from error
    if result < 0.0:
        raise argparse.ArgumentTypeError("运行时长不得为负数")
    return result


def _pose_node_count(value):
    """把value解析为1..64的Pose2D批次车辆数。"""
    try:
        result = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("位姿车辆数必须是整数") from error
    if result < 1 or result > 64:
        raise argparse.ArgumentTypeError("位姿车辆数必须在 1 到 64 之间")
    return result


def _arguments(argv):
    """解析CLI；argv为可选参数序列，None表示读取sys.argv。"""
    # 面板UDP/HTTP端口、运行时长和浏览器行为的参数解析器。
    parser = argparse.ArgumentParser(
        description="接收 ZJCL 输出并展示三车二维协同定位结果",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--udp-bind", default="0.0.0.0", help="UDP 监听地址")
    parser.add_argument("--udp-port", type=_port, default=39002, help="UDP 监听端口")
    parser.add_argument("--http-bind", default="127.0.0.1", help="HTTP 监听地址")
    parser.add_argument("--http-port", type=_port, default=8080, help="HTTP 监听端口")
    parser.add_argument(
        "--duration", type=_duration, default=0.0, help="运行秒数，0 表示持续运行"
    )
    parser.add_argument(
        "--expected-pose-nodes",
        type=_pose_node_count,
        default=3,
        help="同一105快照收齐后原子显示的车辆数",
    )
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    return parser.parse_args(argv)


def main(argv=None):
    """启动UDP、HTTP和可选浏览器；argv为可选CLI参数序列。"""
    # 阶段3：先绑定全部端口再启动线程，避免页面可见但数据端口尚不可用。
    # 已解析启动参数和跨线程共享的遥测仓库。
    args = _arguments(argv)
    state = DashboardState(args.expected_pose_nodes)
    # UDP后台接收线程句柄；main发出停止请求并最多等待2秒。
    receiver = None
    # 只读HTTP服务器句柄；由main负责shutdown和server_close。
    server = None
    # 执行server.serve_forever的HTTP后台线程句柄；main最多等待2秒，不保证已终止。
    server_thread = None
    # 主控制流的停止事件，信号处理器和定时等待共同使用。
    stop_event = threading.Event()
    # 被替换的进程信号处理器，退出时逐项恢复。
    previous_handlers = {}

    def request_stop(_signum=None, _frame=None):
        """信号回调；_signum与_frame由signal模块提供，只转换为线程停止事件。"""
        stop_event.set()

    try:
        receiver = UdpReceiver(state, args.udp_bind, args.udp_port)
        server = create_http_server(args.http_bind, args.http_port, state)
        receiver.start()
        # 主函数创建的HTTP服务线程；finally会限时join，但超时后线程仍可能存活。
        server_thread = threading.Thread(
            target=server.serve_forever, name="zjcl-gcs-http", daemon=True
        )
        server_thread.start()

        # signal_name遍历平台可能提供的两种正常终止信号。
        for signal_name in ("SIGINT", "SIGTERM"):
            if hasattr(signal, signal_name):
                # 当前平台上的信号编号及其进入本模块前的处理器。
                selected = getattr(signal, signal_name)
                previous_handlers[selected] = signal.getsignal(selected)
                signal.signal(selected, request_stop)

        # 浏览器可连接的主机名；通配监听地址需改写为本机回环地址。
        browser_host = args.http_bind
        if browser_host in ("", "0.0.0.0", "::"):
            browser_host = "127.0.0.1"
        # 面板最终可访问URL，包含系统可能动态分配的实际HTTP端口。
        url = f"http://{browser_host}:{server.server_port}/"
        print(
            f"GCS_DASHBOARD udp_port={receiver.port} http_port={server.server_port} url={url}",
            flush=True,
        )
        if not args.no_browser:
            webbrowser.open(url)

        if args.duration > 0.0:
            stop_event.wait(args.duration)
        else:
            while not stop_event.wait(0.5):
                pass
    # error捕获端口绑定、服务器创建或启动参数失败，形成进程级错误SUMMARY。
    except (OSError, ValueError) as error:
        print(f"SUMMARY status=ERROR error={error}", file=sys.stderr, flush=True)
        return 2
    finally:
        # selected/previous分别为已替换信号编号和原处理器。
        for selected, previous in previous_handlers.items():
            signal.signal(selected, previous)
        if server is not None:
            server.shutdown()
            server.server_close()
        if receiver is not None:
            receiver.stop()
            if receiver.ident is not None:
                receiver.join(timeout=2.0)
        if server_thread is not None:
            server_thread.join(timeout=2.0)

    # 发出停止并完成限时等待后的接收统计；不据此断言后台线程一定已终止。
    stats = state.snapshot()["stats"]
    print(
        "SUMMARY status=OK "
        f"datagrams={stats['datagrams']} accepted={stats['accepted']} "
        f"ignored={stats['ignored']} rejected={stats['rejected']}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

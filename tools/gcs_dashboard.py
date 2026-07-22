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


_LOCALIZATION_STATES = {
    protocol.LOCALIZATION_UNINITIALIZED: "未初始化",
    protocol.LOCALIZATION_NORMAL: "正常",
    protocol.LOCALIZATION_DEGRADED: "降级",
    protocol.LOCALIZATION_UNOBSERVABLE: "不可观",
    protocol.LOCALIZATION_STALE: "数据陈旧",
}

_OBSERVATION_STATES = {
    protocol.OBSERVATION_UNKNOWN: "未知",
    protocol.OBSERVATION_NORMAL: "正常",
    protocol.OBSERVATION_DEGRADED: "降级",
    protocol.OBSERVATION_SUSPENDED: "暂缓",
    protocol.OBSERVATION_REJECTED: "剔除",
    protocol.OBSERVATION_RECOVERING: "恢复中",
}

_FUSION_ACTIONS = {
    protocol.FUSION_USE_NORMAL: "正常使用",
    protocol.FUSION_USE_DOWNWEIGHTED: "降权使用",
    protocol.FUSION_HOLD: "暂缓",
    protocol.FUSION_REJECT: "剔除",
    protocol.FUSION_TRIAL_RECOVERY: "试探恢复",
}

_ALGORITHM_RUN_STATES = {
    protocol.ALGORITHM_RUN_INITIALIZING: "初始化中",
    protocol.ALGORITHM_RUN_RUNNING: "运行中",
    protocol.ALGORITHM_RUN_DEGRADED: "降级运行",
    protocol.ALGORITHM_RUN_ERROR: "错误",
    protocol.ALGORITHM_RUN_STOPPED: "已停止",
}

_ALERT_LEVELS = {
    protocol.ALERT_LEVEL_INFO: "提示",
    protocol.ALERT_LEVEL_WARNING: "警告",
    protocol.ALERT_LEVEL_ERROR: "错误",
    protocol.ALERT_LEVEL_CRITICAL: "严重",
}

_ALERT_LIFECYCLES = {
    protocol.ALERT_LIFECYCLE_ACTIVE: "活动",
    protocol.ALERT_LIFECYCLE_CLEARED: "已清除",
}

_QUALITY_COLORS = {
    protocol.OBSERVATION_UNKNOWN: "#94a3b8",
    protocol.OBSERVATION_NORMAL: "#22c55e",
    protocol.OBSERVATION_DEGRADED: "#f59e0b",
    protocol.OBSERVATION_SUSPENDED: "#ef4444",
    protocol.OBSERVATION_REJECTED: "#7f1d1d",
    protocol.OBSERVATION_RECOVERING: "#38bdf8",
}

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
    reasons = [text for bit, text in _REASON_BITS if mask & (1 << bit)]
    known_mask = sum(1 << bit for bit, _ in _REASON_BITS)
    unknown_mask = mask & ~known_mask
    if unknown_mask:
        reasons.append(f"未知原因位 0x{unknown_mask:08X}")
    return reasons


class DashboardState:
    """线程安全的遥测状态仓库；一次数据报只更新其对应的业务对象。"""
    """Thread-safe latest-value store for GCS-facing ZJCL output frames."""

    def __init__(self):
        self._lock = threading.Lock()
        self._nodes = {}
        self._edges = {}
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
        self._stats = {
            "datagrams": 0,
            "accepted": 0,
            "ignored": 0,
            "rejected": 0,
            "last_receive_time_ns": 0,
        }

    def _count(self, category):
        with self._lock:
            self._stats["datagrams"] += 1
            self._stats[category] += 1
            self._stats["last_receive_time_ns"] = time.time_ns()

    def ingest_datagram(self, data):
        """严格解码一帧；坏帧只增加错误计数，不覆盖最后有效值。"""
        # 阶段1：公共帧CRC/长度通过后，再按消息类型解析固定载荷。
        """Decode and store one supported UDP frame; return True if accepted."""
        try:
            frame = protocol.decode_frame(data, udp=True)
        except (protocol.ProtocolError, TypeError, ValueError):
            self._count("rejected")
            return False

        if frame.message_type not in (
            protocol.MSG_LOCALIZATION,
            protocol.MSG_NETWORK,
            protocol.MSG_OBSERVATION,
            protocol.MSG_ALGORITHM_STATUS,
            protocol.MSG_ALERT,
        ):
            self._count("ignored")
            return False

        try:
            if frame.message_type == protocol.MSG_LOCALIZATION:
                decoded = protocol.decode_localization_payload(frame.payload)
                update_kind = "localization"
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

        received_at = time.time_ns()
        with self._lock:
            if update_kind == "localization":
                self._nodes[str(frame.source_node)] = {
                    "node_id": frame.source_node,
                    "x": decoded.x,
                    "y": decoded.y,
                    "vx": decoded.vx,
                    "vy": decoded.vy,
                    "cov_xx": decoded.cov_xx,
                    "cov_xy": decoded.cov_xy,
                    "cov_yy": decoded.cov_yy,
                    "state": decoded.state,
                    "state_text": _LOCALIZATION_STATES[decoded.state],
                    "valid": decoded.valid,
                    "capability_mask": decoded.capability_mask,
                    "timestamp_ns": frame.timestamp_ns,
                    "sequence": frame.sequence,
                    "receive_time_ns": received_at,
                }
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
                packed = decoded.software_version_packed
                version = f"{packed >> 16}.{(packed >> 8) & 0xFF}.{packed & 0xFF}"
                self._algorithm_status = {
                    "abi_version": decoded.abi_version,
                    "software_version_packed": packed,
                    "software_version": version,
                    "mode": decoded.mode,
                    "mode_text": "UWB_ONLY_PLANAR",
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
    """后台UDP接收线程；短超时用于及时响应停止事件。"""
    """Bounded-blocking UDP receiver that can be stopped from another thread."""

    def __init__(self, state, bind_address, port):
        super().__init__(name="zjcl-gcs-udp", daemon=True)
        self._state = state
        self._stop_event = threading.Event()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self._socket.bind((bind_address, port))
            self._socket.settimeout(0.2)
        except Exception:
            self._socket.close()
            raise
        self.port = self._socket.getsockname()[1]

    def run(self):
        # 阶段2：保持数据报边界，把协议语义交给DashboardState集中处理。
        while not self._stop_event.is_set():
            try:
                data, _ = self._socket.recvfrom(protocol.MAX_UDP_DATAGRAM + 1)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_event.is_set():
                    break
                continue
            self._state.ingest_datagram(data)

    def stop(self):
        self._stop_event.set()
        try:
            self._socket.close()
        except OSError:
            pass


_DASHBOARD_PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>三车 UWB 协同定位在线演示</title>
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
  <h1>三车 UWB 协同定位在线演示</h1>
  <div class="sub">主参考平台坐标系 · 二维平面位置 · 质量状态实时刷新</div>
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

  const nodes = Object.values(model.nodes);
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
    context.fillStyle = node.valid ? "#38bdf8" : "#64748b";
    context.beginPath(); context.arc(current.x, current.y, 14, 0, Math.PI * 2); context.fill();
    context.strokeStyle = "#e0f2fe"; context.lineWidth = 2; context.stroke();
    context.fillStyle = "#f8fafc";
    context.font = "bold 13px Microsoft YaHei";
    context.textAlign = "center";
    context.fillText(String(node.node_id), current.x, current.y + 5);
    context.font = "12px Microsoft YaHei";
    context.fillText(`节点 ${node.node_id}  (${node.x.toFixed(2)}, ${node.y.toFixed(2)}) m`, current.x, current.y + 34);
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
    document.getElementById("algorithm-version").textContent = `${algorithm_status.software_version} / ${algorithm_status.run_state_text}`;
    document.getElementById("range-counts").textContent = `${algorithm_status.accepted_ranges} / ${algorithm_status.rejected_ranges}`;
    document.getElementById("protocol-errors").textContent = String(algorithm_status.protocol_errors);
    document.getElementById("uptime").textContent = `${(algorithm_status.uptime_ns / 1e9).toFixed(1)} s`;
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
    daemon_threads = True
    allow_reuse_address = True


def create_http_server(bind_address, port, state):
    """创建只读HTTP服务；动态JSON来自state，静态页面内嵌在本模块。"""
    """Create, but do not start, the dashboard HTTP server."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.partition("?")[0]
            if path == "/api/state":
                body = json.dumps(
                    state.snapshot(), ensure_ascii=False, separators=(",", ":")
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
            elif path in ("/", "/index.html"):
                body = _DASHBOARD_PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
            else:
                body = b"not found"
                self.send_response(404)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, _format, *args):
            del args

    return _DashboardHttpServer((bind_address, port), Handler)


def _port(value):
    try:
        result = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("端口必须是整数") from error
    if result < 0 or result > 65535:
        raise argparse.ArgumentTypeError("端口必须在 0 到 65535 之间")
    return result


def _duration(value):
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("运行时长必须是数字") from error
    if result < 0.0:
        raise argparse.ArgumentTypeError("运行时长不得为负数")
    return result


def _arguments(argv):
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
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    return parser.parse_args(argv)


def main(argv=None):
    """启动UDP、HTTP和可选浏览器，并在信号到达时有序停止。"""
    # 阶段3：先绑定全部端口再启动线程，避免页面可见但数据端口尚不可用。
    args = _arguments(argv)
    state = DashboardState()
    receiver = None
    server = None
    server_thread = None
    stop_event = threading.Event()
    previous_handlers = {}

    def request_stop(_signum=None, _frame=None):
        stop_event.set()

    try:
        receiver = UdpReceiver(state, args.udp_bind, args.udp_port)
        server = create_http_server(args.http_bind, args.http_port, state)
        receiver.start()
        server_thread = threading.Thread(
            target=server.serve_forever, name="zjcl-gcs-http", daemon=True
        )
        server_thread.start()

        for signal_name in ("SIGINT", "SIGTERM"):
            if hasattr(signal, signal_name):
                selected = getattr(signal, signal_name)
                previous_handlers[selected] = signal.getsignal(selected)
                signal.signal(selected, request_stop)

        browser_host = args.http_bind
        if browser_host in ("", "0.0.0.0", "::"):
            browser_host = "127.0.0.1"
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
    except (OSError, ValueError) as error:
        print(f"SUMMARY status=ERROR error={error}", file=sys.stderr, flush=True)
        return 2
    finally:
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

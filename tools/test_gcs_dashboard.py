"""GCS面板状态、UDP接收、HTTP快照和命令行启动的自动回归测试。

测试只使用回环网络和内存构造帧，不依赖浏览器人工操作或真实车辆。
"""

import json
import socket
import subprocess
import sys
import threading
import time
import unittest
import urllib.request
from pathlib import Path

import zjcl_protocol as protocol

from gcs_dashboard import DashboardState, UdpReceiver, create_http_server


def _frame(message_type, payload, source=1, target=0, sequence=7):
    """构造固定时间帧。

    message_type/payload决定业务布局；source/target是节点端点；sequence是测试序号。
    """
    return protocol.encode_frame(
        protocol.Frame(
            message_type=message_type,
            flags=0,
            sequence=sequence,
            timestamp_ns=123_456_789,
            source_node=source,
            target_node=target,
            payload=payload,
        ),
        udp=True,
    )


def _localization(source=1, x=1.25, y=-2.5):
    """构造合法平面定位帧；source为节点，x/y为位置米值。"""
    return _frame(
        protocol.MSG_LOCALIZATION,
        protocol.encode_localization_payload(
            protocol.LocalizationPayload(
                x=x,
                y=y,
                vx=0.1,
                vy=-0.2,
                cov_xx=0.04,
                cov_xy=0.0,
                cov_yy=0.09,
                state=protocol.LOCALIZATION_NORMAL,
                valid=True,
                yaw_valid=False,
                z_valid=False,
                capability_mask=(
                    protocol.CAPABILITY_UWB_RANGE
                    | protocol.CAPABILITY_PLANAR_POSITION
                    | protocol.CAPABILITY_VELOCITY
                ),
            )
        ),
        source=source,
    )


def _network(reason_mask=(1 << 4) | (1 << 7)):
    """构造不可观网络帧；reason_mask控制面板原因位展开风险。"""
    return _frame(
        protocol.MSG_NETWORK,
        protocol.encode_network_payload(
            protocol.NetworkPayload(
                node_count=3,
                reachable_node_count=2,
                active_edge_count=2,
                connected=False,
                observable=False,
                state=protocol.LOCALIZATION_UNOBSERVABLE,
                reason_mask=reason_mask,
            )
        ),
        source=1,
    )


def _observation(state=protocol.OBSERVATION_DEGRADED, source=1, target=2):
    """构造边观测帧；state为质量状态，source/target为边端点。"""
    return _frame(
        protocol.MSG_OBSERVATION,
        protocol.encode_observation_payload(
            protocol.ObservationPayload(
                window_start_ns=100,
                window_end_ns=200,
                expected_count=20,
                received_count=18,
                valid_count=15,
                nlos_count=3,
                residual_rejected_count=1,
                dropped_count=2,
                nlos_ratio=0.2,
                valid_ratio=0.75,
                actual_rate_hz=18.0,
                covariance_scale=2.0,
                state=state,
                action=protocol.FUSION_USE_DOWNWEIGHTED,
                input_overflow=False,
                reason_mask=(1 << 0) | (1 << 3),
            )
        ),
        source=source,
        target=target,
    )


def _algorithm_status(run_state=protocol.ALGORITHM_RUN_DEGRADED):
    """构造算法状态帧；run_state控制生命周期标签映射。"""
    return _frame(
        protocol.MSG_ALGORITHM_STATUS,
        protocol.encode_algorithm_status_payload(
            protocol.AlgorithmStatusPayload(
                protocol.ALGORITHM_STATUS_ABI_VERSION,
                protocol.SOFTWARE_VERSION_PACKED,
                protocol.ALGORITHM_MODE_UWB_ONLY_PLANAR,
                run_state,
                120,
                4,
                3,
                5_000_000_000,
            )
        ),
        source=1,
    )


def _alert(lifecycle=protocol.ALERT_LIFECYCLE_ACTIVE, reason_mask=1 << 5):
    """构造网络告警帧；lifecycle和reason_mask必须满足协议联动约束。"""
    return _frame(
        protocol.MSG_ALERT,
        protocol.encode_alert_payload(
            protocol.AlertPayload(
                protocol.ALERT_CODE_NETWORK_STATE,
                protocol.ALERT_LEVEL_WARNING,
                lifecycle,
                protocol.ALERT_SOURCE_ALGORITHM,
                reason_mask,
                0,
                0,
                0,
                100,
                200,
            )
        ),
        source=1,
    )


class DashboardStateTests(unittest.TestCase):
    """验证各类算法输出如何原子更新面板快照及陈旧状态。"""
    def test_supported_frames_update_public_state_without_out_of_scope_axes(self):
        # 接收三类支持帧的面板状态仓库。
        state = DashboardState()

        self.assertTrue(state.ingest_datagram(_localization()))
        self.assertTrue(state.ingest_datagram(_network()))
        self.assertTrue(state.ingest_datagram(_observation()))
        # 三次原子更新后的对外JSON快照。
        snapshot = state.snapshot()

        self.assertEqual(snapshot["nodes"]["1"]["x"], 1.25)
        self.assertEqual(snapshot["nodes"]["1"]["y"], -2.5)
        self.assertEqual(snapshot["network"]["state_text"], "不可观")
        self.assertEqual(
            snapshot["network"]["reasons"], ["时间同步超时", "节点不可达"]
        )
        self.assertEqual(snapshot["edges"]["1-2"]["quality_color"], "#f59e0b")
        self.assertIn("非视距比例高", snapshot["edges"]["1-2"]["reasons"])
        self.assertEqual(snapshot["stats"]["accepted"], 3)

        # 全快照小写序列化文本，用于审计不得泄露未支持航向/高度轴。
        serialized = json.dumps(snapshot, ensure_ascii=False).lower()
        self.assertNotIn("yaw", serialized)
        self.assertNotIn("z_valid", serialized)
        self.assertNotIn("altitude", serialized)
        self.assertNotIn("高度", serialized)

    def test_ignored_and_malformed_datagrams_are_counted_without_state_damage(self):
        # 同时接收不关心类型和畸形字节的空状态仓库。
        state = DashboardState()
        # 协议合法但不属于GCS输出类型的测距帧，应计ignored而非rejected。
        range_frame = _frame(
            protocol.MSG_RANGE,
            protocol.encode_range_payload(
                protocol.RangePayload(3.0, 0.1, 0.0, False, True, True, 0)
            ),
            source=1,
            target=2,
        )

        self.assertFalse(state.ingest_datagram(range_frame))
        self.assertFalse(state.ingest_datagram(b"not-a-zjcl-frame"))
        # 两种失败输入后的状态快照，业务对象应仍为空。
        snapshot = state.snapshot()

        self.assertEqual(snapshot["nodes"], {})
        self.assertEqual(snapshot["edges"], {})
        self.assertEqual(snapshot["stats"]["datagrams"], 2)
        self.assertEqual(snapshot["stats"]["ignored"], 1)
        self.assertEqual(snapshot["stats"]["rejected"], 1)

    def test_algorithm_status_and_network_alert_are_exposed(self):
        # 状态/告警展示映射的隔离仓库。
        state = DashboardState()
        self.assertTrue(state.ingest_datagram(_algorithm_status()))
        self.assertTrue(state.ingest_datagram(_alert()))

        # 同时包含最新状态与告警的公开快照。
        snapshot = state.snapshot()
        # 算法状态子对象，验证压缩版本和累计计数没有丢失。
        status = snapshot["algorithm_status"]
        self.assertEqual(status["software_version"], "0.1.0")
        self.assertEqual(status["accepted_ranges"], 120)
        self.assertEqual(status["rejected_ranges"], 4)
        self.assertEqual(status["protocol_errors"], 3)
        self.assertEqual(status["uptime_ns"], 5_000_000_000)
        self.assertEqual(status["run_state"], protocol.ALGORITHM_RUN_DEGRADED)
        # 告警子对象，验证生命周期、原因位与可读文本。
        alert = snapshot["alert"]
        self.assertEqual(alert["alert_code"], protocol.ALERT_CODE_NETWORK_STATE)
        self.assertEqual(alert["lifecycle"], protocol.ALERT_LIFECYCLE_ACTIVE)
        self.assertEqual(alert["reason_mask"], 1 << 5)
        self.assertTrue(alert["reasons"])

    def test_state_accepts_concurrent_updates(self):
        # 三个写线程共享的DashboardState，专测锁内原子更新与计数不丢失。
        state = DashboardState()

        def feed(node_id):
            """线程任务；node_id固定该线程连续写入的源节点。"""
            # sequence覆盖每个线程的50个独立定位更新序号。
            for sequence in range(50):
                state.ingest_datagram(
                    _frame(
                        protocol.MSG_LOCALIZATION,
                        protocol.encode_localization_payload(
                            protocol.LocalizationPayload(
                                float(sequence),
                                float(node_id),
                                0.0,
                                0.0,
                                1.0,
                                0.0,
                                1.0,
                                protocol.LOCALIZATION_NORMAL,
                                True,
                                False,
                                False,
                                protocol.CAPABILITY_PLANAR_POSITION,
                            )
                        ),
                        source=node_id,
                        sequence=sequence,
                    )
                )

        # node为1..3线程参数；workers持有所有并发写入线程以便完整join。
        workers = [threading.Thread(target=feed, args=(node,)) for node in range(1, 4)]
        # worker逐个启动同一共享状态上的并发写入。
        for worker in workers:
            worker.start()
        # worker逐个等待，确保快照断言发生在全部150次写入之后。
        for worker in workers:
            worker.join()

        # 所有写线程完成后的锁一致快照。
        snapshot = state.snapshot()
        self.assertEqual(snapshot["stats"]["accepted"], 150)
        self.assertEqual(set(snapshot["nodes"]), {"1", "2", "3"})


class DashboardTransportTests(unittest.TestCase):
    """验证UDP线程、HTTP JSON接口和进程级启动/停止边界。"""
    def test_udp_receiver_ingests_a_real_datagram_and_stops(self):
        # UDP后台线程写入的共享状态。
        state = DashboardState()
        # 动态端口接收线程，避免测试间固定端口冲突。
        receiver = UdpReceiver(state, "127.0.0.1", 0)
        receiver.start()
        # 主测试线程拥有的UDP发送套接字，finally中关闭。
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(_localization(source=3), ("127.0.0.1", receiver.port))
            # 等待节点3写入快照的绝对单调截止时刻，限制竞态测试挂起。
            deadline = time.monotonic() + 2.0
            while "3" not in state.snapshot()["nodes"] and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertIn("3", state.snapshot()["nodes"])
        finally:
            sender.close()
            receiver.stop()
            receiver.join(timeout=2.0)

        self.assertFalse(receiver.is_alive())

    def test_http_api_returns_json_and_embedded_canvas_page(self):
        # 预装节点、网络和边观测的HTTP快照源。
        state = DashboardState()
        state.ingest_datagram(_localization())
        state.ingest_datagram(_network(reason_mask=0))
        state.ingest_datagram(_observation())
        # 动态回环端口HTTP服务器及由测试拥有的服务线程。
        server = create_http_server("127.0.0.1", 0, state)
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        # 使用实际动态端口的请求基址。
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            # response是/api/state请求的HTTP响应句柄，由with读取头和JSON后关闭。
            with urllib.request.urlopen(base + "/api/state", timeout=2.0) as response:
                # /api/state返回的解析后JSON快照。
                api = json.loads(response.read().decode("utf-8"))
                self.assertEqual(response.headers.get_content_type(), "application/json")
            # response此处是根页面请求的HTTP响应句柄，由with读取HTML后关闭。
            with urllib.request.urlopen(base + "/", timeout=2.0) as response:
                # 根路由返回的内嵌单页应用源码。
                page = response.read().decode("utf-8")

            self.assertEqual(api["nodes"]["1"]["x"], 1.25)
            self.assertIn("<canvas", page.lower())
            self.assertIn("/api/state", page)
            self.assertIn("quality_color", page)
            self.assertIn("algorithm_status", page)
            self.assertIn("accepted_ranges", page)
            self.assertIn("alert", page)
            self.assertNotIn("https://", page.lower())
            self.assertNotIn("http://", page.lower())
            # forbidden逐项覆盖超出二维UWB能力边界的中英文UI标识。
            for forbidden in ("yaw", "航向", "高度", "altitude", "z_valid"):
                self.assertNotIn(forbidden, page.lower())
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2.0)

        self.assertFalse(worker.is_alive())

    def test_duration_and_no_browser_cli_exit_cleanly(self):
        # 作为真实子进程启动的面板脚本路径。
        script = Path(__file__).with_name("gcs_dashboard.py")
        # 子进程运行耗时的单调起点。
        started = time.monotonic()
        # 动态双端口、禁浏览器、短时运行的进程级结果。
        completed = subprocess.run(
            [
                sys.executable,
                str(script),
                "--udp-bind",
                "127.0.0.1",
                "--udp-port",
                "0",
                "--http-bind",
                "127.0.0.1",
                "--http-port",
                "0",
                "--duration",
                "0.15",
                "--no-browser",
            ],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        # CLI启动到完全退出的墙钟近似耗时（s），用于发现线程回收卡顿。
        elapsed = time.monotonic() - started

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("SUMMARY status=OK", completed.stdout)
        self.assertLess(elapsed, 3.0)


if __name__ == "__main__":
    unittest.main()

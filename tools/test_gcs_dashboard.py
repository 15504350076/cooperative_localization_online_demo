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
    def test_supported_frames_update_public_state_without_out_of_scope_axes(self):
        state = DashboardState()

        self.assertTrue(state.ingest_datagram(_localization()))
        self.assertTrue(state.ingest_datagram(_network()))
        self.assertTrue(state.ingest_datagram(_observation()))
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

        serialized = json.dumps(snapshot, ensure_ascii=False).lower()
        self.assertNotIn("yaw", serialized)
        self.assertNotIn("z_valid", serialized)
        self.assertNotIn("altitude", serialized)
        self.assertNotIn("高度", serialized)

    def test_ignored_and_malformed_datagrams_are_counted_without_state_damage(self):
        state = DashboardState()
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
        snapshot = state.snapshot()

        self.assertEqual(snapshot["nodes"], {})
        self.assertEqual(snapshot["edges"], {})
        self.assertEqual(snapshot["stats"]["datagrams"], 2)
        self.assertEqual(snapshot["stats"]["ignored"], 1)
        self.assertEqual(snapshot["stats"]["rejected"], 1)

    def test_algorithm_status_and_network_alert_are_exposed(self):
        state = DashboardState()
        self.assertTrue(state.ingest_datagram(_algorithm_status()))
        self.assertTrue(state.ingest_datagram(_alert()))

        snapshot = state.snapshot()
        status = snapshot["algorithm_status"]
        self.assertEqual(status["software_version"], "0.1.0")
        self.assertEqual(status["accepted_ranges"], 120)
        self.assertEqual(status["rejected_ranges"], 4)
        self.assertEqual(status["protocol_errors"], 3)
        self.assertEqual(status["uptime_ns"], 5_000_000_000)
        self.assertEqual(status["run_state"], protocol.ALGORITHM_RUN_DEGRADED)
        alert = snapshot["alert"]
        self.assertEqual(alert["alert_code"], protocol.ALERT_CODE_NETWORK_STATE)
        self.assertEqual(alert["lifecycle"], protocol.ALERT_LIFECYCLE_ACTIVE)
        self.assertEqual(alert["reason_mask"], 1 << 5)
        self.assertTrue(alert["reasons"])

    def test_state_accepts_concurrent_updates(self):
        state = DashboardState()

        def feed(node_id):
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

        workers = [threading.Thread(target=feed, args=(node,)) for node in range(1, 4)]
        for worker in workers:
            worker.start()
        for worker in workers:
            worker.join()

        snapshot = state.snapshot()
        self.assertEqual(snapshot["stats"]["accepted"], 150)
        self.assertEqual(set(snapshot["nodes"]), {"1", "2", "3"})


class DashboardTransportTests(unittest.TestCase):
    def test_udp_receiver_ingests_a_real_datagram_and_stops(self):
        state = DashboardState()
        receiver = UdpReceiver(state, "127.0.0.1", 0)
        receiver.start()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(_localization(source=3), ("127.0.0.1", receiver.port))
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
        state = DashboardState()
        state.ingest_datagram(_localization())
        state.ingest_datagram(_network(reason_mask=0))
        state.ingest_datagram(_observation())
        server = create_http_server("127.0.0.1", 0, state)
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urllib.request.urlopen(base + "/api/state", timeout=2.0) as response:
                api = json.loads(response.read().decode("utf-8"))
                self.assertEqual(response.headers.get_content_type(), "application/json")
            with urllib.request.urlopen(base + "/", timeout=2.0) as response:
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
            for forbidden in ("yaw", "航向", "高度", "altitude", "z_valid"):
                self.assertNotIn(forbidden, page.lower())
        finally:
            server.shutdown()
            server.server_close()
            worker.join(timeout=2.0)

        self.assertFalse(worker.is_alive())

    def test_duration_and_no_browser_cli_exit_cleanly(self):
        script = Path(__file__).with_name("gcs_dashboard.py")
        started = time.monotonic()
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
        elapsed = time.monotonic() - started

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("SUMMARY status=OK", completed.stdout)
        self.assertLess(elapsed, 3.0)


if __name__ == "__main__":
    unittest.main()

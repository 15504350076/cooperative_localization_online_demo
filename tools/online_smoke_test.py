#!/usr/bin/env python3
"""仅测距兼容模式的临时 UDP 在线、日志和回放端到端测试。"""

import argparse
import configparser
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import time

import uwb_simulator
import zjcl_protocol as zjcl


EXPECTED_CAPABILITIES = (
    zjcl.CAPABILITY_UWB_RANGE
    | zjcl.CAPABILITY_PLANAR_POSITION
    | zjcl.CAPABILITY_VELOCITY
)
EXPECTED_NODES = {1, 2, 3}
EXPECTED_EDGES = {(1, 2), (1, 3), (2, 3)}


class OutputEvidence:
    def __init__(self):
        self.localization_nodes = set()
        self.observation_edges = set()
        self.good_network_seen = False
        self.status_seen = False
        self.maximum_accepted_ranges = 0
        self.active_alert_seen = False
        self.active_alert_after_good_network = False
        self.inputs_stopped = False
        self.link_timeout_alert_after_stop = False
        self.frame_count = 0

    def mark_inputs_stopped(self):
        self.inputs_stopped = True

    def ingest(self, datagram):
        frame = zjcl.decode_frame(datagram, udp=True)
        self.frame_count += 1
        if frame.message_type == zjcl.MSG_LOCALIZATION:
            value = zjcl.decode_localization_payload(frame.payload)
            if value.yaw_valid or value.z_valid:
                raise AssertionError(
                    "UWB-only output must not claim valid yaw or altitude"
                )
            if (value.capability_mask & EXPECTED_CAPABILITIES) != EXPECTED_CAPABILITIES:
                raise AssertionError("localization capability mask is incomplete")
            if value.valid:
                self.localization_nodes.add(frame.source_node)
        elif frame.message_type == zjcl.MSG_NETWORK:
            value = zjcl.decode_network_payload(frame.payload)
            self.good_network_seen = self.good_network_seen or (
                value.connected and value.observable
            )
        elif frame.message_type == zjcl.MSG_OBSERVATION:
            zjcl.decode_observation_payload(frame.payload)
            self.observation_edges.add(
                tuple(sorted((frame.source_node, frame.target_node)))
            )
        elif frame.message_type == zjcl.MSG_ALGORITHM_STATUS:
            value = zjcl.decode_algorithm_status_payload(frame.payload)
            if value.abi_version != zjcl.ALGORITHM_STATUS_ABI_VERSION:
                raise AssertionError("unexpected algorithm ABI version")
            if value.software_version_packed != zjcl.SOFTWARE_VERSION_PACKED:
                raise AssertionError("unexpected algorithm software version")
            if value.mode != zjcl.ALGORITHM_MODE_UWB_ONLY_PLANAR:
                raise AssertionError("unexpected algorithm mode")
            self.status_seen = True
            self.maximum_accepted_ranges = max(
                self.maximum_accepted_ranges, value.accepted_ranges
            )
        elif frame.message_type == zjcl.MSG_ALERT:
            value = zjcl.decode_alert_payload(frame.payload)
            if value.lifecycle == zjcl.ALERT_LIFECYCLE_ACTIVE:
                self.active_alert_seen = True
                if self.good_network_seen:
                    self.active_alert_after_good_network = True
                if (
                    self.good_network_seen
                    and self.inputs_stopped
                    and value.reason_mask & (1 << 5)
                ):
                    self.link_timeout_alert_after_stop = True

    def assert_complete(self):
        missing_nodes = EXPECTED_NODES - self.localization_nodes
        missing_edges = EXPECTED_EDGES - self.observation_edges
        if missing_nodes:
            raise AssertionError(
                f"missing valid localization nodes: {sorted(missing_nodes)}"
            )
        if missing_edges:
            raise AssertionError(
                f"missing observation edges: {sorted(missing_edges)}"
            )
        if not self.good_network_seen:
            raise AssertionError("connected and observable network was not seen")
        if not self.status_seen:
            raise AssertionError("algorithm status frame was not seen")
        if self.maximum_accepted_ranges == 0:
            raise AssertionError("algorithm status did not report accepted ranges")
        if not self.link_timeout_alert_after_stop:
            raise AssertionError(
                "LINK_TIMEOUT alert after normal inputs stopped was not seen"
            )

    def assert_replay_telemetry(self):
        if not self.status_seen:
            raise AssertionError("replay did not emit algorithm status")
        if not self.active_alert_seen:
            raise AssertionError("replay did not emit an active network alert")


def write_temporary_config(source, target, input_port, output_port, log_path):
    source = Path(source)
    target = Path(target)
    log_path = Path(log_path)
    parser = configparser.ConfigParser(interpolation=None)
    with source.open("r", encoding="utf-8") as input_file:
        parser.read_file(input_file)
    if not parser.has_section("online"):
        raise ValueError("configuration has no [online] section")
    parser.set("online", "input_bind_address", "127.0.0.1")
    parser.set("online", "input_port", str(input_port))
    parser.set("online", "output_address", "127.0.0.1")
    parser.set("online", "output_port", str(output_port))
    parser.set("online", "event_log_enabled", "true")
    parser.set("online", "event_log_path", str(log_path))
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8", newline="\n") as output_file:
        parser.write(output_file)


def _available_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as candidate:
        candidate.bind(("127.0.0.1", 0))
        return candidate.getsockname()[1]


def _hidden_process_options():
    if os.name != "nt":
        return {}
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    return {
        "startupinfo": startup,
        "creationflags": subprocess.CREATE_NO_WINDOW,
    }


def _start_process(arguments):
    return subprocess.Popen(
        [str(item) for item in arguments],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        **_hidden_process_options(),
    )


def _finish_process(process, timeout_s):
    try:
        output, _ = process.communicate(timeout=timeout_s)
        return process.returncode, output
    except subprocess.TimeoutExpired as error:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        raise AssertionError("subprocess exceeded its timeout") from error


def _stop_process(process):
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
    try:
        process.communicate(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate(timeout=1.0)


def _summary_counter(output, name):
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) is None:
        raise AssertionError(f"invalid summary counter name: {name}")
    matches = re.findall(
        rf"(?:^|\s){re.escape(name)}=(\d+)(?=\s|$)", output
    )
    if len(matches) != 1:
        raise AssertionError(
            f"expected one {name} counter in subprocess SUMMARY"
        )
    return int(matches[0])


def _drain_output(receiver, evidence):
    while True:
        try:
            datagram, _ = receiver.recvfrom(zjcl.MAX_UDP_DATAGRAM)
        except socket.timeout:
            return
        evidence.ingest(datagram)


def _exercise_online(process, receiver, target, send_duration, evidence):
    simulator = uwb_simulator.UwbSimulator(
        seed=2026,
        noise_std_m=0.0,
        nlos_probability=0.0,
        drop_probability=0.0,
        range_std_m=0.1,
    )
    start = time.monotonic()
    next_tick = start + 0.10
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while time.monotonic() - start < send_duration:
            now = time.monotonic()
            if now >= next_tick:
                for datagram in simulator.generate_tick(time.time_ns()):
                    sender.sendto(datagram, target)
                next_tick += 0.05
            _drain_output(receiver, evidence)
            if process.poll() is not None:
                break
            time.sleep(0.002)

    evidence.mark_inputs_stopped()
    # The demo edge timeout is 0.5 s. Stop normal ranges, then continue
    # receiving for longer than that so the smoke test proves the timeout
    # transition instead of accepting only a startup alert.
    drain_deadline = time.monotonic() + 0.85
    while time.monotonic() < drain_deadline:
        _drain_output(receiver, evidence)
        time.sleep(0.005)
    result = _finish_process(process, max(3.0, send_duration + 2.0))
    _drain_output(receiver, evidence)
    return result


def run_smoke(args):
    online_exe = Path(args.online_exe).resolve()
    replay_exe = Path(args.replay_exe).resolve()
    config_path = Path(args.config).resolve()
    for path, description in (
        (online_exe, "online executable"),
        (replay_exe, "replay executable"),
        (config_path, "configuration"),
    ):
        if not path.is_file():
            raise AssertionError(f"{description} does not exist: {path}")

    evidence = OutputEvidence()
    online_process = None
    with tempfile.TemporaryDirectory(prefix="zju_coop_smoke_") as directory:
        temporary_root = Path(directory)
        temporary_config = temporary_root / "smoke.ini"
        event_log = temporary_root / "logs" / "smoke.zjlg"

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.01)
            output_port = receiver.getsockname()[1]
            input_port = _available_udp_port()
            write_temporary_config(
                config_path,
                temporary_config,
                input_port,
                output_port,
                event_log,
            )

            online_duration_ms = max(
                1500, int((args.send_duration + 1.2) * 1000.0)
            )
            online_process = _start_process(
                (
                    online_exe,
                    "--config",
                    temporary_config,
                    "--duration-ms",
                    online_duration_ms,
                )
            )
            target = ("127.0.0.1", input_port)
            try:
                online_code, online_output = _exercise_online(
                    online_process,
                    receiver,
                    target,
                    args.send_duration,
                    evidence,
                )
            finally:
                _stop_process(online_process)
            online_process = None

        if online_code != 0:
            raise AssertionError(
                f"online executable failed ({online_code}):\n{online_output}"
            )
        if "SUMMARY status=OK" not in online_output:
            raise AssertionError(
                f"online executable has no successful SUMMARY:\n{online_output}"
            )
        for name in (
            "accepted_ranges",
            "published_localization",
            "published_network",
            "published_observation",
            "published_status",
            "published_alert",
        ):
            if _summary_counter(online_output, name) == 0:
                raise AssertionError(f"online SUMMARY reports zero {name}")
        evidence.assert_complete()
        if not event_log.is_file() or event_log.stat().st_size <= 8:
            raise AssertionError("online event log is missing or empty")

        replay_evidence = OutputEvidence()
        replay_process = None
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as replay_receiver:
            replay_receiver.bind(("127.0.0.1", output_port))
            replay_receiver.settimeout(0.01)
            replay_process = _start_process(
                (
                    replay_exe,
                    "--config",
                    temporary_config,
                    "--log",
                    event_log,
                    "--speed",
                    "0",
                    "--output-mode",
                    "stream",
                )
            )
            try:
                replay_code, replay_output = _finish_process(
                    replay_process, 5.0
                )
            finally:
                _stop_process(replay_process)
            _drain_output(replay_receiver, replay_evidence)
        if replay_code != 0:
            raise AssertionError(
                f"replay executable failed ({replay_code}):\n{replay_output}"
            )
        if "SUMMARY status=OK" not in replay_output:
            raise AssertionError(
                f"replay executable has no successful SUMMARY:\n{replay_output}"
            )
        for name in (
            "input_ranges",
            "output_records_skipped",
            "accepted_ranges",
            "emitted",
        ):
            if _summary_counter(replay_output, name) == 0:
                raise AssertionError(f"replay SUMMARY reports zero {name}")
        replay_evidence.assert_replay_telemetry()

    print(
        "PASS online smoke"
        f" frames={evidence.frame_count}"
        f" nodes={len(evidence.localization_nodes)}"
        f" edges={len(evidence.observation_edges)}"
        f" replay_frames={replay_evidence.frame_count}",
        flush=True,
    )
    return 0


def build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online-exe", required=True)
    parser.add_argument("--replay-exe", required=True)
    # 本脚本专门验证仅测距回退路径；交付默认融合路径由imu_online_smoke_test验证。
    parser.add_argument("--config", default="config/range_only_demo.ini")
    parser.add_argument("--send-duration", type=float, default=1.2)
    return parser


def main(argv=None):
    args = build_argument_parser().parse_args(argv)
    if args.send_duration <= 0.2:
        print("ERROR --send-duration must be greater than 0.2", file=sys.stderr)
        return 2
    try:
        return run_smoke(args)
    except (AssertionError, OSError, ValueError) as error:
        print(f"FAIL {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

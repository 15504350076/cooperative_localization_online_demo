#!/usr/bin/env python3
"""IMU+UWB在线进程、GCS输出和日志回放端到端烟雾测试。"""

import argparse
import configparser
import os
from pathlib import Path
import re
import socket
import subprocess
import tempfile
import time

import imu_uwb_simulator
import zjcl_protocol as zjcl


def _port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _process(args):
    options = {}
    if os.name == "nt":
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        options = {"startupinfo": startup,
                   "creationflags": subprocess.CREATE_NO_WINDOW}
    return subprocess.Popen([str(value) for value in args],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace",
                            **options)


def _counter(output, name):
    match = re.search(rf"(?:^|\s){re.escape(name)}=(\d+)(?=\s|$)", output)
    if match is None:
        raise AssertionError(f"missing SUMMARY counter: {name}\n{output}")
    return int(match.group(1))


def run(args):
    online = Path(args.online_exe).resolve()
    replay = Path(args.replay_exe).resolve()
    source_config = Path(args.config).resolve()
    for path in (online, replay, source_config):
        if not path.is_file():
            raise AssertionError(f"missing file: {path}")

    with tempfile.TemporaryDirectory(prefix="zju_imu_smoke_") as directory:
        root = Path(directory)
        config_path = root / "imu_smoke.ini"
        log_path = root / "imu_smoke.zjlg"
        input_port = _port()
        output_port = _port()
        parser = configparser.ConfigParser(interpolation=None)
        with source_config.open("r", encoding="utf-8") as stream:
            parser.read_file(stream)
        parser.set("online", "input_bind_address", "127.0.0.1")
        parser.set("online", "input_port", str(input_port))
        parser.set("online", "output_address", "127.0.0.1")
        parser.set("online", "output_port", str(output_port))
        parser.set("online", "event_log_enabled", "true")
        parser.set("online", "event_log_path", str(log_path))
        with config_path.open("w", encoding="utf-8", newline="\n") as stream:
            parser.write(stream)

        process = _process((online, "--config", config_path,
                            "--duration-ms", "1800"))
        simulator = imu_uwb_simulator.ImuUwbSimulator()
        modes = set()
        localization_nodes = set()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(("127.0.0.1", output_port))
            receiver.settimeout(0.005)
            start = time.monotonic()
            next_imu = start + 0.10
            next_uwb = start + 0.10
            while time.monotonic() - start < 1.2:
                now = time.monotonic()
                timestamp = time.time_ns()
                if now >= next_imu:
                    for frame in simulator.generate_imu_tick(timestamp):
                        sender.sendto(frame, ("127.0.0.1", input_port))
                    next_imu += 0.01
                if now >= next_uwb:
                    for frame in simulator.generate_uwb_tick(timestamp):
                        sender.sendto(frame, ("127.0.0.1", input_port))
                    next_uwb += 0.05
                try:
                    data, _ = receiver.recvfrom(zjcl.MAX_UDP_DATAGRAM)
                    frame = zjcl.decode_frame(data, udp=True)
                    if frame.message_type == zjcl.MSG_ALGORITHM_STATUS:
                        modes.add(zjcl.decode_algorithm_status_payload(
                            frame.payload).mode)
                    elif frame.message_type == zjcl.MSG_LOCALIZATION:
                        if zjcl.decode_localization_payload(frame.payload).valid:
                            localization_nodes.add(frame.source_node)
                except socket.timeout:
                    pass
            output, _ = process.communicate(timeout=4.0)

        if process.returncode != 0 or "SUMMARY status=OK" not in output:
            raise AssertionError(f"online failed:\n{output}")
        if _counter(output, "imu") == 0 or _counter(output, "propagated_imu") == 0:
            raise AssertionError(f"online did not propagate IMU:\n{output}")
        if _counter(output, "accepted_ranges") == 0:
            raise AssertionError(f"online did not process UWB:\n{output}")
        if zjcl.ALGORITHM_MODE_IMU_UWB_15_STATE not in modes:
            raise AssertionError("GCS status did not report IMU+UWB mode")
        if localization_nodes != {1, 2, 3}:
            raise AssertionError(f"missing localization nodes: {localization_nodes}")
        if not log_path.is_file() or log_path.stat().st_size <= 8:
            raise AssertionError("IMU+UWB event log is empty")

        replay_process = _process((replay, "--config", config_path,
                                   "--log", log_path, "--speed", "0",
                                   "--output-mode", "final"))
        replay_output, _ = replay_process.communicate(timeout=5.0)
        if replay_process.returncode != 0:
            raise AssertionError(f"replay failed:\n{replay_output}")
        if _counter(replay_output, "input_imus") == 0 or \
                _counter(replay_output, "input_ranges") == 0:
            raise AssertionError(f"replay missed inputs:\n{replay_output}")

    print("PASS IMU+UWB online smoke nodes=3 mode=2 replay=OK")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online-exe", required=True)
    parser.add_argument("--replay-exe", required=True)
    parser.add_argument("--config", required=True)
    try:
        return run(parser.parse_args())
    except (AssertionError, OSError, subprocess.TimeoutExpired) as error:
        print(f"FAIL {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

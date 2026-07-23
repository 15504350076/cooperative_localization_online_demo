#!/usr/bin/env python3
"""默认IMU+测距在线进程、GCS输出和日志回放的端到端冒烟测试。

测试在临时目录和回环端口启动真实可执行程序，发送确定性模拟数据，再检查SUMMARY、
输出帧与回放结果。它验证软件闭环可运行，不用于证明真实定位精度或实时性能。
"""

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
    """返回关闭探测socket后得到的回环UDP端口提示。

    端口未被保留，实际绑定前存在TOCTOU竞争；连续两次调用也可能返回同一值。
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def _process(args):
    """启动真实子进程；args为命令序列，Windows隐藏控制台且仍捕获UTF-8输出。"""
    # 当前平台传给Popen的窗口选项，非Windows保持为空。
    options = {}
    if os.name == "nt":
        # Windows隐藏窗口启动配置，不影响合并stdout/stderr证据。
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        options = {"startupinfo": startup,
                   "creationflags": subprocess.CREATE_NO_WINDOW}
    # value为命令序列中可路径化/数值参数，Popen统一接收字符串。
    return subprocess.Popen([str(value) for value in args],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True, encoding="utf-8", errors="replace",
                            **options)


def _counter(output, name):
    """读取SUMMARY计数；output为子进程文本，name为待匹配键。"""
    # 完整键边界后的十进制计数匹配，避免名称前后缀误命中。
    match = re.search(rf"(?:^|\s){re.escape(name)}=(\d+)(?=\s|$)", output)
    if match is None:
        raise AssertionError(f"missing SUMMARY counter: {name}\n{output}")
    return int(match.group(1))


def run(args):
    """完成五阶段端到端验证；args为已解析CLI命名空间。"""
    # 规范化后的在线/回放可执行文件及基线配置路径。
    online = Path(args.online_exe).resolve()
    replay = Path(args.replay_exe).resolve()
    source_config = Path(args.config).resolve()
    # path逐项验证所有外部测试输入真实存在，避免晚期子进程误报。
    for path in (online, replay, source_config):
        if not path.is_file():
            raise AssertionError(f"missing file: {path}")

    # 阶段1：使用回环端口提示与临时日志，降低干扰用户在线实例的概率；探测socket
    # 已关闭，因此端口可能被抢占，input/output两次探测也不保证得到不同值。
    with tempfile.TemporaryDirectory(prefix="zju_imu_smoke_") as directory:
        # 本次冒烟独占的临时根、配置副本和事件日志路径。
        root = Path(directory)
        config_path = root / "imu_smoke.ini"
        log_path = root / "imu_smoke.zjlg"
        # 两个值都是未保留的端口提示；后续在线进程和receiver才分别尝试实际绑定。
        input_port = _port()
        output_port = _port()
        # 禁用插值的INI解析器，确保只替换联调端口和日志路径。
        parser = configparser.ConfigParser(interpolation=None)
        # stream是基线配置的只读UTF-8句柄，读取结束后由with关闭。
        with source_config.open("r", encoding="utf-8") as stream:
            parser.read_file(stream)
        parser.set("online", "input_bind_address", "127.0.0.1")
        parser.set("online", "input_port", str(input_port))
        parser.set("online", "output_address", "127.0.0.1")
        parser.set("online", "output_port", str(output_port))
        parser.set("online", "event_log_enabled", "true")
        parser.set("online", "event_log_path", str(log_path))
        # stream此处是临时配置的独占写句柄，写入刷新后由with关闭。
        with config_path.open("w", encoding="utf-8", newline="\n") as stream:
            parser.write(stream)

        # 阶段2：启动真实在线程序，并向其输入标准IMU和三边测距。
        # 当前在线被测进程句柄；正常路径会communicate等待，但创建后没有统一finally，
        # 因而后续异常或等待超时可能遗留仍运行的子进程。
        process = _process((online, "--config", config_path,
                            "--duration-ms", "1800"))
        # 零故障组合模拟器，以及从真实输出收集的算法模式/有效节点证据。
        simulator = imu_uwb_simulator.ImuUwbSimulator()
        modes = set()
        localization_nodes = set()
        # receiver由本在线阶段独占并绑定output_port，负责收集GCS输出；
        # sender同一作用域独占无绑定UDP套接字，向input_port发送IMU/UWB输入。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(("127.0.0.1", output_port))
            receiver.settimeout(0.005)
            # 送数阶段单调起点，以及100 Hz IMU/20 Hz UWB的首个截止时刻。
            start = time.monotonic()
            next_imu = start + 0.10
            next_uwb = start + 0.10
            while time.monotonic() - start < 1.2:
                # 当前调度单调时刻与该轮输入共享的协议Unix纳秒时间。
                now = time.monotonic()
                timestamp = time.time_ns()
                if now >= next_imu:
                    # frame为单节点完整IMU输入数据报。
                    for frame in simulator.generate_imu_tick(timestamp):
                        sender.sendto(frame, ("127.0.0.1", input_port))
                    next_imu += 0.01
                if now >= next_uwb:
                    # frame为单边完整测距输入数据报。
                    for frame in simulator.generate_uwb_tick(timestamp):
                        sender.sendto(frame, ("127.0.0.1", input_port))
                    next_uwb += 0.05
                try:
                    # data是一个GCS输出数据报；发送端地址不参与证据判定。
                    data, _ = receiver.recvfrom(zjcl.MAX_UDP_DATAGRAM)
                    # 严格验证公共头和CRC后的输出帧。
                    frame = zjcl.decode_frame(data, udp=True)
                    if frame.message_type == zjcl.MSG_ALGORITHM_STATUS:
                        modes.add(zjcl.decode_algorithm_status_payload(
                            frame.payload).mode)
                    elif frame.message_type == zjcl.MSG_LOCALIZATION:
                        if zjcl.decode_localization_payload(frame.payload).valid:
                            localization_nodes.add(frame.source_node)
                except socket.timeout:
                    pass
            # 在线进程合并输出文本；4秒为送数结束后的退出截止时间。
            output, _ = process.communicate(timeout=4.0)

        # 阶段3：退出码本身不足以证明闭环；同时检查IMU确实传播、测距被消费、
        # 状态帧报告15维模式、三个节点均发布有效定位且日志含实际记录。
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

        # 阶段4：speed=0快速回放刚生成的日志，确认IMU和测距Input均被重新消费；
        # 历史Output不会被计入input_imus/input_ranges。
        # 回放被测进程读取刚生成日志；同样未受统一finally保护，异常/超时可能遗留进程。
        replay_process = _process((replay, "--config", config_path,
                                   "--log", log_path, "--speed", "0",
                                   "--output-mode", "final"))
        # 回放合并输出文本，包含输入消费SUMMARY证据。
        replay_output, _ = replay_process.communicate(timeout=5.0)
        if replay_process.returncode != 0:
            raise AssertionError(f"replay failed:\n{replay_output}")
        if _counter(replay_output, "input_imus") == 0 or \
                _counter(replay_output, "input_ranges") == 0:
            raise AssertionError(f"replay missed inputs:\n{replay_output}")

    print("PASS IMU+UWB online smoke nodes=3 mode=2 replay=OK")
    return 0


def main():
    """CLI入口：解析在线程序、回放程序和基线配置路径。"""
    # IMU+UWB冒烟测试命令行参数解析器。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online-exe", required=True)
    parser.add_argument("--replay-exe", required=True)
    parser.add_argument("--config", required=True)
    try:
        return run(parser.parse_args())
    # error汇总证据失败、回环/文件I/O错误或子进程超时，转换为CLI失败输出。
    except (AssertionError, OSError, subprocess.TimeoutExpired) as error:
        print(f"FAIL {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

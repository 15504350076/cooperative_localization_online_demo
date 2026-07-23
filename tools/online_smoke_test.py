#!/usr/bin/env python3
"""仅测距兼容模式的临时UDP在线、日志和回放端到端测试。

本模块通过真实进程和回环UDP收集可审计证据：三节点定位、三条观测边、连通可观网络、
周期状态、输入停止后的链路超时告警以及日志回放遥测，防止仅凭进程退出码假通过。
"""

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


# 仅测距回退模式仍必须显式声明的定位输出能力组合。
EXPECTED_CAPABILITIES = (
    zjcl.CAPABILITY_UWB_RANGE
    | zjcl.CAPABILITY_PLANAR_POSITION
    | zjcl.CAPABILITY_VELOCITY
)
# 3-4-5固定几何应覆盖的完整节点集与无向边集。
EXPECTED_NODES = {1, 2, 3}
EXPECTED_EDGES = {(1, 2), (1, 3), (2, 3)}


class OutputEvidence:
    """聚合输出帧证据；只有几何、状态和告警条件全部满足才判定通过。"""
    def __init__(self):
        # 证据对象生命周期内见过有效定位的节点集合，由接收主线程更新。
        self.localization_nodes = set()
        # 见过合法ObservationPayload的规范化无向边集合。
        self.observation_edges = set()
        # 是否曾到达连通且可观状态，防止启动退化态被误认作稳定闭环。
        self.good_network_seen = False
        # 是否见过ABI/版本/模式均正确的算法状态帧。
        self.status_seen = False
        # 状态帧报告的最大累计接受测距数，证明算法实际消费输入。
        self.maximum_accepted_ranges = 0
        # 告警时序证据：任意活动告警、正常网络后的活动告警。
        self.active_alert_seen = False
        self.active_alert_after_good_network = False
        # 由发送阶段显式置位，限定后续LINK_TIMEOUT必须发生在停输之后。
        self.inputs_stopped = False
        self.link_timeout_alert_after_stop = False
        # 本证据对象累计解码的输出帧数，用于最终审计摘要。
        self.frame_count = 0

    def mark_inputs_stopped(self):
        """记录输入停输边界；之后的链路超时告警才可作为恢复周期证据。"""
        self.inputs_stopped = True

    def ingest(self, datagram):
        """按消息类型提取关键证据；datagram是一个完整ZJCL/UDP输出帧。"""
        # 已通过公共头、载荷长度与CRC校验的输出帧。
        frame = zjcl.decode_frame(datagram, udp=True)
        self.frame_count += 1
        if frame.message_type == zjcl.MSG_LOCALIZATION:
            # 当前定位输出，重点审计有效位和仅测距能力边界。
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
            # 当前网络快照，用于记录是否曾进入正常连通可观状态。
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
            # 当前算法状态，用于ABI、版本、模式和输入消费计数证据。
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
            # 当前告警生命周期事件，用于验证停输后的LINK_TIMEOUT时序。
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
        """要求正常网络和停输后的LINK_TIMEOUT都出现，排除只看到启动告警的假通过。"""
        # 尚未提供有效定位的节点与尚未输出统计的几何边。
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
        """要求回放重新产生状态和活动网络告警，防止只读日志但不重算输出。"""
        if not self.status_seen:
            raise AssertionError("replay did not emit algorithm status")
        if not self.active_alert_seen:
            raise AssertionError("replay did not emit an active network alert")


def write_temporary_config(source, target, input_port, output_port, log_path):
    """写联调配置。

    source是基线配置，target是临时副本；input_port/output_port为回环UDP端口；
    log_path为本次事件日志路径，除这些集成字段外算法参数保持不变。
    """
    # 规范化后的源配置、目标配置和事件日志路径。
    source = Path(source)
    target = Path(target)
    log_path = Path(log_path)
    # 禁用插值的INI解析器，避免含百分号的算法配置被改写。
    parser = configparser.ConfigParser(interpolation=None)
    # input_file是基线INI的只读UTF-8句柄，由with在解析完成后关闭。
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
    # output_file是临时INI的独占写句柄，由with完成刷新并关闭。
    with target.open("w", encoding="utf-8", newline="\n") as output_file:
        parser.write(output_file)


def _available_udp_port():
    """返回关闭探测socket后得到的回环UDP端口提示。

    该端口未被保留，交给子进程前存在TOCTOU竞争，重复调用也可能得到同一端口。
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as candidate:
        candidate.bind(("127.0.0.1", 0))
        return candidate.getsockname()[1]


def _hidden_process_options():
    """返回平台专用Popen选项；Windows隐藏控制台，其他平台为空字典。"""
    if os.name != "nt":
        return {}
    # Windows子进程启动配置，仅影响窗口显示，不改变输出捕获。
    startup = subprocess.STARTUPINFO()
    startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow = subprocess.SW_HIDE
    return {
        "startupinfo": startup,
        "creationflags": subprocess.CREATE_NO_WINDOW,
    }


def _start_process(arguments):
    """启动真实被测进程；arguments为可转成字符串的命令与参数序列。"""
    # item为命令序列中的可路径化/数值参数，统一传给Popen为字符串。
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
    """限时收集输出；process为Popen句柄，timeout_s为正常退出截止秒数。"""
    try:
        # 子进程合并后的UTF-8文本；stderr已重定向所以第二项未使用。
        output, _ = process.communicate(timeout=timeout_s)
        return process.returncode, output
    # error记录首次等待超过timeout_s的TimeoutExpired，触发terminate/kill清理链。
    except subprocess.TimeoutExpired as error:
        process.terminate()
        try:
            output, _ = process.communicate(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=1.0)
        raise AssertionError("subprocess exceeded its timeout") from error


def _stop_process(process):
    """幂等回收Popen进程；process可为None，超时则从terminate升级到kill。"""
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
    """提取唯一SUMMARY十进制计数；output为子进程文本，name为合法标识符键。"""
    if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) is None:
        raise AssertionError(f"invalid summary counter name: {name}")
    # 输出中与完整键边界匹配的全部十进制值，必须恰有一个才可审计。
    matches = re.findall(
        rf"(?:^|\s){re.escape(name)}=(\d+)(?=\s|$)", output
    )
    if len(matches) != 1:
        raise AssertionError(
            f"expected one {name} counter in subprocess SUMMARY"
        )
    return int(matches[0])


def _drain_output(receiver, evidence):
    """排空当前UDP输出；receiver设置短超时，evidence接收每个合法数据报。"""
    while True:
        try:
            # 一个完整输出数据报；发送端地址不参与证据判定。
            datagram, _ = receiver.recvfrom(zjcl.MAX_UDP_DATAGRAM)
        except socket.timeout:
            return
        evidence.ingest(datagram)


def _exercise_online(process, receiver, target, send_duration, evidence):
    """驱动在线阶段。

    process是在线Popen句柄；receiver收集输出；target为输入UDP端点；
    send_duration是送数秒数；evidence聚合正常与停输后的输出证据。
    """
    # 零噪声、零NLOS、零丢包模拟器，隔离在线算法而非输入随机性的风险。
    simulator = uwb_simulator.UwbSimulator(
        seed=2026,
        noise_std_m=0.0,
        nlos_probability=0.0,
        drop_probability=0.0,
        range_std_m=0.1,
    )
    # 送数阶段单调起点与首个20 Hz测距tick截止时刻。
    start = time.monotonic()
    next_tick = start + 0.10
    # sender由本驱动阶段独占，向target发送测距输入并在送数结束后由with关闭。
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while time.monotonic() - start < send_duration:
            # 当前调度单调时刻。
            now = time.monotonic()
            if now >= next_tick:
                # datagram为当前边的完整测距输入帧。
                for datagram in simulator.generate_tick(time.time_ns()):
                    sender.sendto(datagram, target)
                next_tick += 0.05
            _drain_output(receiver, evidence)
            if process.poll() is not None:
                break
            time.sleep(0.002)

    # 输入停止后继续接收，使Engine.step有机会发布链路超时和告警激活帧。
    evidence.mark_inputs_stopped()
    # 默认边超时为0.5 s，继续监听0.85 s可证明“正常→停输→LINK_TIMEOUT”转换，
    # 不能用进程启动阶段原本就可能存在的不可观告警冒充停输告警。
    # 停输后继续收集输出的绝对单调截止时刻。
    drain_deadline = time.monotonic() + 0.85
    while time.monotonic() < drain_deadline:
        _drain_output(receiver, evidence)
        time.sleep(0.005)
    # 在线进程退出码与完整文本输出，供上层SUMMARY交叉验证。
    result = _finish_process(process, max(3.0, send_duration + 2.0))
    _drain_output(receiver, evidence)
    return result


def run_smoke(args):
    """编排在线、证据、日志和回放阶段；args为已解析CLI命名空间。"""
    # 规范化后的在线/回放可执行文件与基线配置路径。
    online_exe = Path(args.online_exe).resolve()
    replay_exe = Path(args.replay_exe).resolve()
    config_path = Path(args.config).resolve()
    # path/description分别是必须存在的输入路径及面向失败消息的角色名。
    for path, description in (
        (online_exe, "online executable"),
        (replay_exe, "replay executable"),
        (config_path, "configuration"),
    ):
        if not path.is_file():
            raise AssertionError(f"{description} does not exist: {path}")

    # 在线阶段证据聚合器及受finally所有权保护的子进程句柄。
    evidence = OutputEvidence()
    online_process = None
    # directory是TemporaryDirectory创建并最终递归清理的临时目录名称字符串。
    with tempfile.TemporaryDirectory(prefix="zju_coop_smoke_") as directory:
        # 本次冒烟独占的临时根、配置副本与事件日志路径。
        temporary_root = Path(directory)
        temporary_config = temporary_root / "smoke.ini"
        event_log = temporary_root / "logs" / "smoke.zjlg"

        # receiver由在线验证阶段独占，绑定动态output_port并接收被测进程输出。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.01)
            # output_port由仍打开的receiver实际占用；input_port只是已释放探测socket
            # 给出的未保留提示，子进程绑定前存在TOCTOU竞争。
            output_port = receiver.getsockname()[1]
            input_port = _available_udp_port()
            write_temporary_config(
                config_path,
                temporary_config,
                input_port,
                output_port,
                event_log,
            )

            # 在线子进程运行预算（ms），覆盖送数和停输告警观察窗口。
            online_duration_ms = max(
                1500, int((args.send_duration + 1.2) * 1000.0)
            )
            # 当前在线被测进程句柄，finally确保任何断言路径都会回收。
            online_process = _start_process(
                (
                    online_exe,
                    "--config",
                    temporary_config,
                    "--duration-ms",
                    online_duration_ms,
                )
            )
            # 测距模拟数据的在线进程回环输入端点。
            target = ("127.0.0.1", input_port)
            try:
                # 在线退出码与合并输出文本，是SUMMARY验证的进程级证据。
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
        # name遍历必须由在线SUMMARY报告为非零的关键工作量计数。
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
        # 同时使用SUMMARY计数和真实UDP帧证据，避免被测程序只打印成功文本。
        evidence.assert_complete()
        if not event_log.is_file() or event_log.stat().st_size <= 8:
            raise AssertionError("online event log is missing or empty")

        # 回放阶段独立证据，避免在线帧掩盖回放未输出的问题。
        replay_evidence = OutputEvidence()
        # 回放子进程句柄先置空，确保启动/等待任一路径都可由finally幂等回收。
        replay_process = None
        # replay_receiver独占已释放的output_port，证明回放遥测来自回放进程而非在线阶段。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as replay_receiver:
            replay_receiver.bind(("127.0.0.1", output_port))
            replay_receiver.settimeout(0.01)
            # 当前回放被测进程句柄，读取刚生成的临时事件日志。
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
                # 回放退出码与合并输出文本，用于SUMMARY和遥测双重验证。
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
        # name遍历回放阶段必须非零的输入消费、跳过历史输出和发射计数。
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
    """构建仅测距端到端冒烟CLI。"""
    # 在线/回放可执行路径、基线配置和送数时长的参数解析器。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--online-exe", required=True)
    parser.add_argument("--replay-exe", required=True)
    # 本脚本专门验证仅测距回退路径；交付默认融合路径由imu_online_smoke_test验证。
    parser.add_argument("--config", default="config/range_only_demo.ini")
    parser.add_argument("--send-duration", type=float, default=1.2)
    return parser


def main(argv=None):
    """CLI入口；argv为可选参数序列，None表示读取sys.argv。"""
    # 已解析冒烟测试参数。
    args = build_argument_parser().parse_args(argv)
    if args.send_duration <= 0.2:
        print("ERROR --send-duration must be greater than 0.2", file=sys.stderr)
        return 2
    try:
        return run_smoke(args)
    # error汇总证据断言、回环I/O或配置值失败，转换为冒烟测试FAIL输出。
    except (AssertionError, OSError, ValueError) as error:
        print(f"FAIL {error}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

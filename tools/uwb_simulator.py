#!/usr/bin/env python3
"""确定性的三车平台间测距数据源，用于验证仅测距兼容模式。

临时联调传输使用UDP/ZJCL；正式AIBrainBox适配仍由上交ROS 2负责。本工具在硬件到位前
提供可复现的3-4-5几何、噪声、NLOS偏置和丢包序列，不属于生产传感器驱动。
"""

import argparse
import math
import random
import socket
import sys
import time

import zjcl_protocol as zjcl


# 固定3-4-5几何：每项为(source_node, target_node, true_range_m)。
DEFAULT_EDGES = (
    (1, 2, 3.0),
    (1, 3, 4.0),
    (2, 3, 5.0),
)


def _finite_nonnegative(name, value):
    """规范化非负有限参数；name用于错误定位，value为用户/调用方输入。"""
    # 统一为float后的仿真物理参数。
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return number


def _probability(name, value):
    """规范化概率参数；name用于错误定位，value必须位于闭区间[0,1]。"""
    # 统一为float后的伯努利事件概率。
    number = float(value)
    if not math.isfinite(number) or number < 0.0 or number > 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
    return number


class UwbSimulator:
    """每个采样时刻为未丢弃的协同边生成一个ZJCL测距数据报。"""

    def __init__(
        self,
        *,
        seed,
        noise_std_m,
        nlos_probability,
        drop_probability,
        range_std_m,
        nlos_bias_m=0.5,
    ):
        """配置可复现测距源。

        seed控制本实例私有伪随机流；noise_std_m为零均值高斯噪声标准差（m）；
        nlos_probability与drop_probability分别控制NLOS/丢包伯努利事件；
        range_std_m写入载荷作为接收方测量标准差（m）；nlos_bias_m是NLOS正偏置（m）。
        """
        # 实例生命周期内只读的噪声幅值（m），由调用generate_tick的线程读取。
        self.noise_std_m = _finite_nonnegative("noise_std_m", noise_std_m)
        # 实例生命周期内只读的NLOS/丢包概率，由私有随机流逐边采样。
        self.nlos_probability = _probability(
            "nlos_probability", nlos_probability
        )
        self.drop_probability = _probability(
            "drop_probability", drop_probability
        )
        # 写入每个RangePayload的1σ测量标准差（m），生命周期与模拟器一致。
        self.range_std_m = _finite_nonnegative("range_std_m", range_std_m)
        if self.range_std_m == 0.0:
            raise ValueError("range_std_m must be positive")
        # NLOS样本叠加的正向距离偏置（m），实例生命周期内不变。
        self.nlos_bias_m = _finite_nonnegative("nlos_bias_m", nlos_bias_m)
        # 实例私有PRNG；仅generate_tick调用线程拥有，seed保证测试流可复现。
        self._random = random.Random(seed)
        # 每条有向边独立维护序号，模拟上交侧每个测距数据流的递增sequence。
        # 实例生命周期内持久的逐边序号表；source/target是固定几何的端点，
        # 下划线刻意忽略该项的true_range_m，因为序号初值只依赖端点。
        self.sequence_by_edge = {
            (source, target): 0 for source, target, _ in DEFAULT_EDGES
        }

    def generate_tick(self, timestamp_ns):
        """使用同一时间戳生成当前三条边。

        timestamp_ns是写入所有帧头的uint64 Unix纪元纳秒；随机序列由构造种子决定。
        """
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            raise ValueError("timestamp_ns must be an integer")
        if timestamp_ns < 0 or timestamp_ns > (1 << 64) - 1:
            raise ValueError("timestamp_ns is outside uint64")

        # 当前采样批次成功生成的UDP数据报；丢包边不会占据列表元素。
        datagrams = []
        # 每条边独立决定丢包、NLOS和高斯噪声，公共时间戳保持同一采样批次。
        for source, target, true_range_m in DEFAULT_EDGES:
            # 当前有向(source,target)数据流键；反向流若存在将独立维护序号。
            edge = (source, target)
            self.sequence_by_edge[edge] += 1
            # 本边本tick的线缆序号，即使后续丢包也已消耗。
            sequence = self.sequence_by_edge[edge]

            # 序号在丢包判定前递增，因此接收端能从序号缺口观察到模拟丢失，
            # 而不是把下一包错误地伪装成连续采样。
            if self._random.random() < self.drop_probability:
                continue

            # NLOS为正偏置而非对称噪声；flag给出本次真值，probability保留配置置信度。
            # 当前边的NLOS真值、零均值噪声（m）及最终正距离观测（m）。
            nlos_flag = self._random.random() < self.nlos_probability
            noise_m = self._random.gauss(0.0, self.noise_std_m)
            measured_range_m = (
                true_range_m
                + noise_m
                + (self.nlos_bias_m if nlos_flag else 0.0)
            )
            measured_range_m = max(measured_range_m, 1.0e-6)
            # 24字节测距payload，携带噪声后距离与配置质量信息。
            payload = zjcl.encode_range_payload(
                zjcl.RangePayload(
                    range_m=measured_range_m,
                    range_std_m=self.range_std_m,
                    nlos_probability=self.nlos_probability,
                    nlos_flag=nlos_flag,
                    has_nlos_probability=True,
                    valid=True,
                    status=0,
                )
            )
            datagrams.append(
                zjcl.encode_frame(
                    zjcl.Frame(
                        message_type=zjcl.MSG_RANGE,
                        flags=0,
                        sequence=sequence,
                        timestamp_ns=timestamp_ns,
                        source_node=source,
                        target_node=target,
                        payload=payload,
                    ),
                    udp=True,
                )
            )
        return datagrams


def _positive_float(text):
    """argparse正有限浮点转换器；text是命令行词法值。"""
    try:
        # 命令行文本转换后的候选频率/标准差。
        value = float(text)
    # error是float(text)失败的原始ValueError，作为argparse错误因果链保留。
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a number") from error
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def _nonnegative_float(text):
    """argparse非负有限浮点转换器；text是命令行词法值。"""
    try:
        # 命令行文本转换后的候选时长/噪声/偏置。
        value = float(text)
    # error是非负参数文本无法转成float时的原始ValueError。
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a number") from error
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("must be finite and non-negative")
    return value


def _unit_float(text):
    """argparse概率转换器；text必须可解析为[0,1]有限数。"""
    # 已通过非负检查的候选概率。
    value = _nonnegative_float(text)
    if value > 1.0:
        raise argparse.ArgumentTypeError("must be in [0, 1]")
    return value


def build_argument_parser():
    """构建模拟器CLI参数契约。"""
    # 命令行解析器，集中定义目的地址、节拍和误差模型参数。
    parser = argparse.ArgumentParser(
        description="Send deterministic 3-4-5 UWB ranges over temporary ZJCL/UDP"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=39001)
    parser.add_argument("--rate-hz", type=_positive_float, default=20.0)
    parser.add_argument(
        "--duration",
        type=_nonnegative_float,
        default=10.0,
        help="seconds; zero runs until interrupted",
    )
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--noise-std-m", type=_nonnegative_float, default=0.02)
    parser.add_argument("--range-std-m", type=_positive_float, default=0.10)
    parser.add_argument(
        "--nlos-probability", type=_unit_float, default=0.0
    )
    parser.add_argument("--nlos-bias-m", type=_nonnegative_float, default=0.5)
    parser.add_argument("--drop-probability", type=_unit_float, default=0.0)
    return parser


def run(args):
    """按标称频率发送测距数据报并输出统计；args为已解析CLI命名空间。"""
    if args.port < 1 or args.port > 65535:
        raise ValueError("port must be in [1, 65535]")
    # 本次运行独占的确定性测距状态机。
    simulator = UwbSimulator(
        seed=args.seed,
        noise_std_m=args.noise_std_m,
        nlos_probability=args.nlos_probability,
        drop_probability=args.drop_probability,
        range_std_m=args.range_std_m,
        nlos_bias_m=args.nlos_bias_m,
    )
    # 每个数据报的UDP目的端点(host, port)。
    target = (args.host, args.port)
    # 标称测距tick周期，单位秒。
    period_s = 1.0 / args.rate_hz
    # 本次运行的单调时钟起点，供时长统计与首轮调度使用。
    start = time.monotonic()
    # 下一次tick的单调时钟截止点，初值使首批数据立即发送。
    next_tick = start
    # 运行期累计调度tick数与实际发送数据报数。
    tick_count = 0
    sent_count = 0

    # output_socket由本run调用栈独占，with退出时负责关闭UDP发送套接字。
    # 单调时钟负责发送周期，系统纳秒时钟写入协议测量时间戳。
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as output_socket:
        try:
            while args.duration == 0.0 or time.monotonic() - start < args.duration:
                # 当前采样批次共享的协议Unix纳秒时间戳。
                timestamp_ns = time.time_ns()
                # 丢包过滤后待发送的数据报批次。
                datagrams = simulator.generate_tick(timestamp_ns)
                # datagram为一条边的完整ZJCL/UDP帧。
                for datagram in datagrams:
                    output_socket.sendto(datagram, target)
                    sent_count += 1
                tick_count += 1
                next_tick += period_s
                # 距离下个标称tick的剩余单调时间（s）。
                delay_s = next_tick - time.monotonic()
                if delay_s > 0.0:
                    time.sleep(delay_s)
                elif delay_s < -period_s:
                    # 落后超过一整个周期时重置调度基准，避免突发补发大量过期tick。
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            pass

    # 最终实际运行时长（s），用于SUMMARY速率审计。
    elapsed_s = time.monotonic() - start
    print(
        "SUMMARY status=OK"
        f" ticks={tick_count} sent={sent_count} elapsed_s={elapsed_s:.3f}"
        f" target={args.host}:{args.port}",
        flush=True,
    )
    return 0


def main(argv=None):
    """CLI入口；argv为可选参数序列，None表示读取sys.argv。"""
    # 仅本次调用使用的参数解析器与解析结果。
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    # error汇总端口/参数校验失败或UDP系统调用失败，转换为CLI错误输出。
    except (OSError, ValueError) as error:
        print(f"ERROR {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

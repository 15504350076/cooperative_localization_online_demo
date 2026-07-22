#!/usr/bin/env python3
"""三车静止IMU加3-4-5平台间测距的临时UDP数据源。

IMU保持标准ROS 2 Imu瞬时量语义：零角速度、ENU/FLU静止比力+g、不提供温度；
测距复用确定性三边发生器。该工具只用于硬件到位前联调，不模拟真实器件误差模型。
"""

import argparse
import math
import socket
import sys
import time

import uwb_simulator
import zjcl_protocol as zjcl


class ImuUwbSimulator:
    """产生三个节点的标准IMU瞬时量，并复用UWB三边测距发生器。"""

    def __init__(self, seed=2026):
        self.uwb = uwb_simulator.UwbSimulator(
            seed=seed, noise_std_m=0.01, nlos_probability=0.0,
            drop_probability=0.0, range_std_m=0.1,
        )
        self.imu_sequence = {1: 0, 2: 0, 3: 0}

    def generate_imu_tick(self, timestamp_ns):
        """为三个节点生成同一统一时间戳的瞬时IMU；每个节点仍有独立序号。"""
        frames = []
        # 每个节点维护独立序号；节点号和时间写入ZJCL公共帧头。
        for node_id in (1, 2, 3):
            self.imu_sequence[node_id] += 1
            # 单位姿态仅占位且orientation_valid=false；静止FLU传感器测得+g比力，
            # 让惯导内部R_nb*f_b+[0,0,-g]得到零导航系加速度。
            payload = zjcl.encode_imu_payload(
                zjcl.ImuPayload(
                    (0.0, 0.0, 0.0, 1.0), (0.0,) * 9,
                    (0.0, 0.0, 0.0), (0.0,) * 9,
                    (0.0, 0.0, 9.80665), (0.0,) * 9,
                    "imu_link", False, True, zjcl.RANGE_STATUS_OK,
                )
            )
            frames.append(zjcl.encode_frame(
                zjcl.Frame(zjcl.MSG_IMU, 0, self.imu_sequence[node_id],
                           timestamp_ns, node_id, 0, payload),
                udp=True,
            ))
        return frames

    def generate_uwb_tick(self, timestamp_ns):
        return self.uwb.generate_tick(timestamp_ns)


def _positive(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def run(args):
    """按IMU高频、测距低频两个独立周期向在线程序发送数据。"""
    if args.port < 1 or args.port > 65535:
        raise ValueError("port must be in [1,65535]")
    simulator = ImuUwbSimulator(args.seed)
    start = time.monotonic()
    next_imu = start
    next_uwb = start
    imu_period = 1.0 / args.imu_rate_hz
    uwb_period = 1.0 / args.uwb_rate_hz
    imu_sent = 0
    uwb_sent = 0
    # 使用单调时钟调度，帧内时间戳仍采用系统纳秒时间轴；两个频率互不整除时，
    # 每次循环只发送各自到期的数据，不用低频测距阻塞高频IMU。
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while args.duration == 0.0 or time.monotonic() - start < args.duration:
            now = time.monotonic()
            timestamp_ns = time.time_ns()
            if now >= next_imu:
                for frame in simulator.generate_imu_tick(timestamp_ns):
                    sender.sendto(frame, (args.host, args.port))
                    imu_sent += 1
                next_imu += imu_period
            if now >= next_uwb:
                for frame in simulator.generate_uwb_tick(timestamp_ns):
                    sender.sendto(frame, (args.host, args.port))
                    uwb_sent += 1
                next_uwb += uwb_period
            delay = min(next_imu, next_uwb) - time.monotonic()
            if delay > 0.0:
                time.sleep(min(delay, 0.005))
    print(f"SUMMARY status=OK imu_sent={imu_sent} uwb_sent={uwb_sent}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=39001)
    parser.add_argument("--imu-rate-hz", type=_positive, default=100.0)
    parser.add_argument("--uwb-rate-hz", type=_positive, default=20.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=2026)
    try:
        return run(parser.parse_args(argv))
    except (OSError, ValueError) as error:
        print(f"ERROR {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

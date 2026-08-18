#!/usr/bin/env python3
"""三车静止IMU加3-4-5平台间测距的临时UDP数据源。

IMU保持标准ROS 2 Imu瞬时量语义：可配置瞬时z角速度、ENU/FLU静止比力+g、不提供温度；
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

    def __init__(self, seed=2026, gyro_z_rad_s=0.0):
        """建立组合数据源；seed控制测距复现，gyro_z_rad_s是各车瞬时z角速度。"""
        # 实例生命周期内独占的UWB子模拟器，由同一发送线程调用。
        self.uwb = uwb_simulator.UwbSimulator(
            seed=seed, noise_std_m=0.01, nlos_probability=0.0,
            drop_probability=0.0, range_std_m=0.1,
        )
        # 三个节点各自持久递增的IMU线缆序号，仅生成线程读写。
        self.imu_sequence = {1: 0, 2: 0, 3: 0}
        # 角速度始终作为ROS 2 Imu语义的瞬时量发送，模拟器不对其积分或伪造orientation。
        self.gyro_z_rad_s = _finite_float("gyro_z_rad_s", gyro_z_rad_s)

    def generate_imu_tick(self, timestamp_ns):
        """为三个节点生成瞬时IMU；timestamp_ns是共享的uint64 Unix纪元纳秒。"""
        # 当前IMU采样批次的三个完整ZJCL/UDP帧。
        frames = []
        # 每个节点维护独立序号；节点号和时间写入ZJCL公共帧头。
        # node_id标识当前车载IMU源，节点间序号互不影响。
        for node_id in (1, 2, 3):
            self.imu_sequence[node_id] += 1
            # 单位姿态仅占位且orientation_valid=false；静止FLU传感器测得+g比力，
            # 让惯导内部R_nb*f_b+[0,0,-g]得到零导航系加速度。
            # 固定332字节IMU payload，表示静止平台的瞬时量。
            payload = zjcl.encode_imu_payload(
                zjcl.ImuPayload(
                    (0.0, 0.0, 0.0, 1.0), (0.0,) * 9,
                    (0.0, 0.0, self.gyro_z_rad_s), (0.0,) * 9,
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
        """生成同批三边测距；timestamp_ns直接作为UWB公共帧头时间。"""
        return self.uwb.generate_tick(timestamp_ns)


def _positive(text):
    """argparse正有限浮点转换器；text是命令行频率词法值。"""
    # 命令行文本转换后的候选频率。
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def _finite_float(name, value):
    """把value转换为有限浮点数；name进入错误文本以区分命令行字段。"""
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a number") from error
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def run(args):
    """按两个独立周期向在线程序发送数据；args为已解析CLI命名空间。"""
    if args.port < 1 or args.port > 65535:
        raise ValueError("port must be in [1,65535]")
    # 本次运行独占的组合模拟器和单调时钟起点。
    simulator = ImuUwbSimulator(args.seed, args.gyro_z_rad_s)
    start = time.monotonic()
    # 两类传感器下一次应发送的单调时刻（s）。
    next_imu = start
    next_uwb = start
    # IMU/UWB各自标称周期（s）。
    imu_period = 1.0 / args.imu_rate_hz
    uwb_period = 1.0 / args.uwb_rate_hz
    # 实际成功送入UDP套接字的IMU与UWB帧累计数。
    imu_sent = 0
    uwb_sent = 0
    # sender由本run调用栈独占，with退出时关闭组合输入的UDP发送套接字。
    # 使用单调时钟调度，帧内时间戳仍采用系统纳秒时间轴；两个频率互不整除时，
    # 每次循环只发送各自到期的数据，不用低频测距阻塞高频IMU。
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while args.duration == 0.0 or time.monotonic() - start < args.duration:
            # 当前调度单调时刻与该轮共享的协议Unix纳秒时间戳。
            now = time.monotonic()
            timestamp_ns = time.time_ns()
            if now >= next_imu:
                # frame为单节点的完整IMU ZJCL/UDP帧。
                for frame in simulator.generate_imu_tick(timestamp_ns):
                    sender.sendto(frame, (args.host, args.port))
                    imu_sent += 1
                next_imu += imu_period
            if now >= next_uwb:
                # frame为单条协同边的完整测距ZJCL/UDP帧。
                for frame in simulator.generate_uwb_tick(timestamp_ns):
                    sender.sendto(frame, (args.host, args.port))
                    uwb_sent += 1
                next_uwb += uwb_period
            # 距离两类流中最早截止时刻的剩余调度时间（s）。
            delay = min(next_imu, next_uwb) - time.monotonic()
            if delay > 0.0:
                time.sleep(min(delay, 0.005))
    print(f"SUMMARY status=OK imu_sent={imu_sent} uwb_sent={uwb_sent}")
    return 0


def main(argv=None):
    """CLI入口；argv为可选参数序列，None表示读取sys.argv。"""
    # 组合模拟器的命令行参数解析器。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=39001)
    parser.add_argument("--imu-rate-hz", type=_positive, default=100.0)
    parser.add_argument("--uwb-rate-hz", type=_positive, default=20.0)
    parser.add_argument(
        "--gyro-z-rad-s", type=float, default=0.0,
        help="三个节点IMU的瞬时FLU z轴角速度；不生成orientation真值",
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=2026)
    try:
        return run(parser.parse_args(argv))
    # error汇总端口/频率校验或UDP发送失败，供CLI输出原始失败原因。
    except (OSError, ValueError) as error:
        print(f"ERROR {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

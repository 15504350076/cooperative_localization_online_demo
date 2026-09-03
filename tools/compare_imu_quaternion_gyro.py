#!/usr/bin/env python3
"""对比四元数姿态变化反推的角速度与ROS Imu消息中的陀螺角速度。

输入是 ``export_imu_bag_csv.py`` 导出的 ``imu_full.csv(.gz)``。程序按时间窗：

1. 用窗口首尾四元数计算相对旋转，再除以时间得到角速度（rad/s）；
2. 对窗口内 ``angular_velocity`` 原始数值求平均；
3. 同时尝试将消息数值按 deg/s 乘 pi/180，便于检查单位是否填错；
4. 输出逐窗口CSV、相关系数、回归比例，按需绘制三轴对比图。

重要限制：暂时假设四元数表示载体系到导航系的旋转。四元数可能由同一陀螺
积分并经滤波得到，因此反推角速度不是独立真值；本程序用于接口内部一致性和
单位排查，不能替代转台或已知转角标定。
"""

import argparse
import csv
import gzip
import math
from pathlib import Path


AXES = "xyz"
DEG_TO_RAD = math.pi / 180.0  # 只用于检验“消息实际为deg/s”的假设。


def quaternion_multiply(a, b):
    """Hamilton四元数乘法；输入和输出顺序均为(x, y, z, w)。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_normalize(q):
    """将四元数单位化，避免数值误差影响相对转角。"""
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("invalid quaternion")
    return tuple(value / norm for value in q)


def quaternion_rate(q_start, q_end, dt):
    """由首尾姿态反推载体系表达的平均角速度，单位rad/s。

    暂按q表示body到navigation：delta_q = inverse(q_start) * q_end。
    如果上交确认实际定义为q_bn，乘法顺序及输出坐标系需要相应调整。
    """
    # 单位四元数的逆等于共轭：向量部取反，标量部不变。
    inverse_start = (-q_start[0], -q_start[1], -q_start[2], q_start[3])
    relative = quaternion_multiply(inverse_start, q_end)
    # q和-q表示同一姿态。统一选择标量部非负的表示，以取得最短旋转弧。
    if relative[3] < 0.0:
        relative = tuple(-value for value in relative)
    vector_norm = math.sqrt(sum(value * value for value in relative[:3]))
    if vector_norm <= 1e-15:
        return (0.0, 0.0, 0.0)
    # 单位四元数q=[axis*sin(theta/2), cos(theta/2)]，故theta如下。
    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, relative[3])))
    # axis*theta是旋转向量；除以首尾时间差得到窗口平均角速度。
    return tuple(value * angle / vector_norm / dt for value in relative[:3])


class PairStats:
    """单遍累计一轴消息值x与四元数反推值y的统计量。"""

    def __init__(self):
        self.count = 0
        self.mean_x = 0.0
        self.mean_y = 0.0
        self.sxx = 0.0
        self.syy = 0.0
        self.sxy = 0.0
        self.error_raw_sq = 0.0
        self.error_deg_sq = 0.0

    def add(self, x, y):
        """使用Welford形式更新均值、方差和协方差，避免保存全部窗口。"""
        self.count += 1
        dx = x - self.mean_x
        self.mean_x += dx / self.count
        dy = y - self.mean_y
        self.mean_y += dy / self.count
        self.sxx += dx * (x - self.mean_x)
        self.syy += dy * (y - self.mean_y)
        self.sxy += dx * (y - self.mean_y)
        # 分别计算“消息已是rad/s”和“消息其实是deg/s”两种假设的误差。
        self.error_raw_sq += (y - x) ** 2
        self.error_deg_sq += (y - x * DEG_TO_RAD) ** 2

    def summary(self):
        """返回相关系数以及y=slope*x+intercept的去均值线性回归。"""
        correlation = self.sxy / math.sqrt(self.sxx * self.syy) \
            if self.sxx > 0.0 and self.syy > 0.0 else float("nan")
        slope = self.sxy / self.sxx if self.sxx > 0.0 else float("nan")
        intercept = self.mean_y - slope * self.mean_x
        return correlation, slope, intercept


def open_text(path):
    """透明读取普通CSV或gzip压缩CSV。"""
    return gzip.open(path, "rt", newline="") if path.suffix == ".gz" \
        else path.open("r", newline="")


def read_record(row):
    """从导出CSV的一行读取时间、xyzw四元数及三轴消息角速度。"""
    timestamp = float(row["header_elapsed_s"])
    quaternion = quaternion_normalize(tuple(float(row[f"q{axis}"]) for axis in "xyzw"))
    gyro = tuple(float(row[f"gyro_{axis}_message_unit"]) for axis in AXES)
    if not math.isfinite(timestamp) or not all(map(math.isfinite, gyro)):
        raise ValueError("non-finite timestamp or gyro")
    return timestamp, quaternion, gyro


def compare(input_path, output_path, window_ms):
    """按真实时间长度分窗对比，并把每个窗口的结果写入CSV。"""
    window_s = window_ms / 1000.0
    stats = [PairStats() for _ in AXES]
    skipped = 0

    # message_unit故意不写死单位，因为当前正要排查其究竟是rad/s还是deg/s。
    columns = ["window_start_s", "window_end_s", "dt_s", "sample_count"]
    columns += [f"quaternion_rate_{axis}_rad_s" for axis in AXES]
    columns += [f"gyro_{axis}_message_unit" for axis in AXES]
    columns += [f"gyro_{axis}_assuming_deg_s_rad_s" for axis in AXES]
    columns += [f"difference_message_as_rad_{axis}_rad_s" for axis in AXES]
    columns += [f"difference_message_as_deg_{axis}_rad_s" for axis in AXES]

    with open_text(input_path) as source, output_path.open("w", newline="") as target:
        reader = csv.DictReader(source)
        writer = csv.writer(target)
        writer.writerow(columns)

        # 每个窗口只需保存首帧姿态、陀螺累加和及样本数。
        start_time = start_q = None
        gyro_sum = [0.0, 0.0, 0.0]
        sample_count = 0

        for row in reader:
            try:
                timestamp, quaternion, gyro = read_record(row)
            except (KeyError, ValueError):
                skipped += 1
                continue

            # 初始化首窗；时间回退时从当前帧重新起窗，避免无效dt。
            if start_time is None or timestamp <= start_time:
                start_time, start_q = timestamp, quaternion
                gyro_sum = list(gyro)
                sample_count = 1
                continue

            dt = timestamp - start_time
            # 尚未达到目标窗长时继续累加消息角速度。
            if dt + 1e-12 < window_s:
                for axis in range(3):
                    gyro_sum[axis] += gyro[axis]
                sample_count += 1
                continue

            # 窗口终点四元数只用于姿态差；gyro_mean不包含该终点样本，避免
            # 相邻窗口重复计入同一条消息。
            q_rate = quaternion_rate(start_q, quaternion, dt)
            gyro_mean = tuple(value / sample_count for value in gyro_sum)
            # 这是待验证的单位假设，不是对原始字段单位的既定结论。
            gyro_as_rad = tuple(value * DEG_TO_RAD for value in gyro_mean)
            difference_raw = tuple(q_rate[i] - gyro_mean[i] for i in range(3))
            difference_deg = tuple(q_rate[i] - gyro_as_rad[i] for i in range(3))
            writer.writerow([
                f"{start_time:.9f}", f"{timestamp:.9f}", f"{dt:.9f}", sample_count,
                *(f"{value:.12g}" for value in q_rate),
                *(f"{value:.12g}" for value in gyro_mean),
                *(f"{value:.12g}" for value in gyro_as_rad),
                *(f"{value:.12g}" for value in difference_raw),
                *(f"{value:.12g}" for value in difference_deg),
            ])
            for axis in range(3):
                stats[axis].add(gyro_mean[axis], q_rate[axis])

            # 当前终点同时成为下一窗口起点，使四元数时间段连续。
            start_time, start_q = timestamp, quaternion
            gyro_sum = list(gyro)
            sample_count = 1

    print(f"input: {input_path}")
    print(f"output: {output_path}")
    print(f"window: {window_ms:g} ms")
    print(f"skipped input rows: {skipped}")
    print("axis,windows,correlation,centered_slope,intercept_rad_s,"
          "mean_message_value,mean_quaternion_rate_rad_s,rmse_as_rad_s,rmse_as_deg_s")
    for axis, item in zip(AXES, stats):
        correlation, slope, intercept = item.summary()
        count = item.count
        print(
            f"{axis},{count},{correlation:.9g},{slope:.9g},{intercept:.9g},"
            f"{item.mean_x:.9g},{item.mean_y:.9g},"
            f"{math.sqrt(item.error_raw_sq / count):.9g},"
            f"{math.sqrt(item.error_deg_sq / count):.9g}"
        )


def plot_comparison(csv_path, output_path):
    """同时绘制消息已是rad/s和消息其实是deg/s两种单位假设。"""
    import matplotlib.pyplot as plt

    time = []
    quaternion_rate = [[] for _ in AXES]
    gyro_raw = [[] for _ in AXES]
    gyro_as_rad = [[] for _ in AXES]
    with csv_path.open(newline="") as source:
        for row in csv.DictReader(source):
            time.append((float(row["window_start_s"]) + float(row["window_end_s"])) / 2.0)
            for index, axis in enumerate(AXES):
                quaternion_rate[index].append(float(row[f"quaternion_rate_{axis}_rad_s"]))
                gyro_raw[index].append(float(row[f"gyro_{axis}_message_unit"]))
                gyro_as_rad[index].append(float(row[f"gyro_{axis}_assuming_deg_s_rad_s"]))

    figure, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for index, (axis, subplot) in enumerate(zip(AXES, axes)):
        subplot.plot(time, quaternion_rate[index], linewidth=0.7,
                     label="quaternion-derived")
        subplot.plot(time, gyro_raw[index], linewidth=0.8,
                     label="message gyro (assuming rad/s)")
        subplot.plot(time, gyro_as_rad[index], linewidth=0.8,
                     label="message gyro × π/180 (assuming deg/s)")
        subplot.axhline(0.0, color="black", linewidth=0.5, alpha=0.5)
        subplot.set_ylabel(f"ω{axis} (rad/s)")
        subplot.grid(True, alpha=0.25)
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("header elapsed time (s)")
    figure.suptitle(
        "Quaternion-derived rate vs message gyro\n"
        "Assumption: quaternion maps body to navigation; both unit hypotheses shown"
    )
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    print(f"plot: {output_path}")


def self_test():
    """用1秒绕Z轴正转90度验证四元数顺序、方向和rad/s输出。"""
    identity = (0.0, 0.0, 0.0, 1.0)
    half_angle = math.pi / 4.0
    quarter_turn_z = (0.0, 0.0, math.sin(half_angle), math.cos(half_angle))
    rate = quaternion_rate(identity, quarter_turn_z, 1.0)
    assert abs(rate[0]) < 1e-12 and abs(rate[1]) < 1e-12
    assert abs(rate[2] - math.pi / 2.0) < 1e-12
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path,
                        help="imu_full.csv or imu_full.csv.gz")
    parser.add_argument("--window-ms", type=float, default=10.0,
                        help="comparison window in milliseconds (default: 10)")
    parser.add_argument("-o", "--output", type=Path,
                        help="output CSV (default: INPUT.parent/四元数反推角速度与消息角速度对比_WINDOWms.csv)")
    parser.add_argument("--plot", action="store_true",
                        help="also write a PNG beside the output CSV")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if args.input is None:
        parser.error("INPUT is required unless --self-test is used")
    if not math.isfinite(args.window_ms) or args.window_ms <= 0.0:
        parser.error("--window-ms must be a positive finite number")
    output = args.output or args.input.parent / (
        f"四元数反推角速度与消息角速度对比_{args.window_ms:g}ms.csv")
    compare(args.input, output, args.window_ms)
    if args.plot:
        plot_output = output.parent / (
            f"四元数反推角速度与消息角速度对比_{args.window_ms:g}ms.png")
        plot_comparison(output, plot_output)


if __name__ == "__main__":
    main()

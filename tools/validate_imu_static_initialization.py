#!/usr/bin/env python3
"""用前段静止窗口估计启动零偏，并在后段冻结该初值积分、对比上交AHRS姿态。

输入为 ``export_imu_bag_csv.py`` 生成的 ``imu_full.csv(.gz)``。本工具按已确认
接口约定把上交 FRD/NED 转为算法 FLU/ENU，四元数和欧拉角仅用于一致性验证，
不是独立姿态真值。欧拉角采用 ``Rz(yaw) Ry(pitch) Rx(roll)``（ZYX）。
"""

import argparse
import csv
import gzip
import math
from pathlib import Path


SQRT_HALF = math.sqrt(0.5)
Q_NED_TO_ENU = (SQRT_HALF, SQRT_HALF, 0.0, 0.0)
Q_FLU_TO_FRD = (1.0, 0.0, 0.0, 0.0)


def q_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def q_normalize(value):
    length = math.sqrt(sum(component * component for component in value))
    if not math.isfinite(length) or length <= 1.0e-15:
        raise ValueError("invalid quaternion")
    return tuple(component / length for component in value)


def q_exp(rotation_vector):
    angle = math.sqrt(sum(component * component for component in rotation_vector))
    if angle <= 1.0e-12:
        return q_normalize((*[0.5 * value for value in rotation_vector], 1.0))
    scale = math.sin(0.5 * angle) / angle
    return tuple(scale * value for value in rotation_vector) + (math.cos(0.5 * angle),)


def q_from_zyx(roll, pitch, yaw):
    return q_normalize(q_multiply(
        q_exp((0.0, 0.0, yaw)),
        q_multiply(q_exp((0.0, pitch, 0.0)), q_exp((roll, 0.0, 0.0))),
    ))


def q_to_zyx(value):
    x, y, z, w = value
    roll = math.atan2(2.0 * (w * x + y * z),
                      1.0 - 2.0 * (x * x + y * y))
    pitch_argument = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_argument)
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def q_error_deg(reference, estimate):
    inverse_reference = (-reference[0], -reference[1], -reference[2], reference[3])
    error = q_normalize(q_multiply(inverse_reference, estimate))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(error[3])))))


def wrap_deg(value):
    return (value + 180.0) % 360.0 - 180.0


def open_csv(path):
    return gzip.open(path, "rt", encoding="utf-8", newline="") \
        if path.suffix == ".gz" else path.open("r", encoding="utf-8", newline="")


def read_row(row, gyro_scale):
    timestamp = float(row["header_elapsed_s"])
    raw_q = q_normalize(tuple(float(row[f"q{axis}"]) for axis in "xyzw"))
    # q_FLU_ENU = q_NED_ENU * q_FRD_NED * q_FLU_FRD。
    ahrs_q = q_normalize(q_multiply(
        Q_NED_TO_ENU, q_multiply(raw_q, Q_FLU_TO_FRD)))
    raw_gyro = tuple(float(row[f"gyro_{axis}_message_unit"]) for axis in "xyz")
    raw_accel = tuple(float(row[f"accel_{axis}_message_unit"]) for axis in "xyz")
    gyro = (raw_gyro[0] * gyro_scale,
            -raw_gyro[1] * gyro_scale,
            -raw_gyro[2] * gyro_scale)
    accel = (raw_accel[0], -raw_accel[1], -raw_accel[2])
    if not all(math.isfinite(value) for value in (timestamp, *gyro, *accel)):
        raise ValueError("non-finite IMU row")
    return timestamp, ahrs_q, gyro, accel


class VectorStats:
    def __init__(self):
        self.count = 0
        self.mean = [0.0, 0.0, 0.0]
        self.m2 = [0.0, 0.0, 0.0]

    def add(self, value):
        self.count += 1
        for axis in range(3):
            delta = value[axis] - self.mean[axis]
            self.mean[axis] += delta / self.count
            self.m2[axis] += delta * (value[axis] - self.mean[axis])

    def standard_deviation(self):
        divisor = max(1, self.count - 1)
        return tuple(math.sqrt(max(0.0, item / divisor)) for item in self.m2)


def calibrate(input_path, seconds, gyro_scale):
    gyro_stats = VectorStats()
    accel_stats = VectorStats()
    start_time = end_time = None
    end_q = end_gyro = None
    with open_csv(input_path) as source:
        for row in csv.DictReader(source):
            timestamp, ahrs_q, gyro, accel = read_row(row, gyro_scale)
            if start_time is None:
                start_time = timestamp
            if timestamp - start_time > seconds and gyro_stats.count >= 2:
                break
            gyro_stats.add(gyro)
            accel_stats.add(accel)
            end_time, end_q, end_gyro = timestamp, ahrs_q, gyro
    if gyro_stats.count < 2 or end_time is None:
        raise ValueError("calibration window contains fewer than two samples")
    return gyro_stats, accel_stats, end_time, end_q, end_gyro


def validate(input_path, output_dir, seconds, gyro_scale, preview_stride):
    gyro_stats, accel_stats, calibration_end, end_q, _ = calibrate(
        input_path, seconds, gyro_scale)
    bias = tuple(gyro_stats.mean)
    mean_accel = tuple(accel_stats.mean)
    roll = math.atan2(mean_accel[1], mean_accel[2])
    pitch = math.atan2(-mean_accel[0], math.hypot(mean_accel[1], mean_accel[2]))
    yaw_seed = q_to_zyx(end_q)[2]
    integrated_q = q_from_zyx(roll, pitch, yaw_seed)
    uncorrected_q = integrated_q
    initialization_error = q_error_deg(end_q, integrated_q)

    output_dir.mkdir(parents=True, exist_ok=True)
    full_path = output_dir / "静止初始化后姿态对比.csv.gz"
    preview_path = output_dir / "静止初始化后姿态对比_10Hz.csv"
    plot_path = output_dir / "静止初始化后姿态对比.png"
    report_path = output_dir / "静止初始化验证.md"
    columns = (
        "post_calibration_s", "dt_s", "q_integrated_x", "q_integrated_y",
        "q_integrated_z", "q_integrated_w", "q_ahrs_x", "q_ahrs_y",
        "q_ahrs_z", "q_ahrs_w", "roll_integrated_deg", "pitch_integrated_deg",
        "yaw_integrated_deg", "roll_ahrs_deg", "pitch_ahrs_deg", "yaw_ahrs_deg",
        "roll_difference_deg", "pitch_difference_deg", "yaw_difference_deg",
        "quaternion_error_deg", "quaternion_error_without_bias_deg",
        "gyro_corrected_x_rad_s",
        "gyro_corrected_y_rad_s", "gyro_corrected_z_rad_s",
    )

    plot_time = []
    plot_integrated, plot_ahrs = [[], [], []], [[], [], []]
    plot_error, plot_uncorrected_error = [], []
    previous_time = None
    previous_gyro = None
    output_count = 0
    invalid_intervals = 0
    error_sq_sum = 0.0
    max_error = 0.0
    final_error = float("nan")
    uncorrected_error_sq_sum = 0.0
    uncorrected_max_error = 0.0
    uncorrected_final_error = float("nan")
    with open_csv(input_path) as source, \
            gzip.open(full_path, "wt", encoding="utf-8", newline="") as full_stream, \
            preview_path.open("w", encoding="utf-8", newline="") as preview_stream:
        full_writer = csv.writer(full_stream)
        preview_writer = csv.writer(preview_stream)
        full_writer.writerow(columns)
        preview_writer.writerow(columns)
        for row in csv.DictReader(source):
            timestamp, ahrs_q, gyro, _ = read_row(row, gyro_scale)
            if timestamp + 1.0e-12 < calibration_end:
                continue
            if previous_time is None:
                previous_time, previous_gyro = timestamp, gyro
                dt = 0.0
            else:
                dt = timestamp - previous_time
                if dt <= 0.0 or dt > 0.1:
                    invalid_intervals += 1
                    previous_time, previous_gyro = timestamp, gyro
                    continue
                corrected_midpoint = tuple(
                    0.5 * (previous_gyro[axis] + gyro[axis]) - bias[axis]
                    for axis in range(3))
                integrated_q = q_normalize(q_multiply(
                    integrated_q,
                    q_exp(tuple(value * dt for value in corrected_midpoint))))
                uncorrected_midpoint = tuple(
                    0.5 * (previous_gyro[axis] + gyro[axis])
                    for axis in range(3))
                uncorrected_q = q_normalize(q_multiply(
                    uncorrected_q,
                    q_exp(tuple(value * dt for value in uncorrected_midpoint))))
                previous_time, previous_gyro = timestamp, gyro

            integrated_rpy = tuple(math.degrees(value) for value in q_to_zyx(integrated_q))
            ahrs_rpy = tuple(math.degrees(value) for value in q_to_zyx(ahrs_q))
            differences = tuple(wrap_deg(integrated_rpy[i] - ahrs_rpy[i])
                                for i in range(3))
            error = q_error_deg(ahrs_q, integrated_q)
            uncorrected_error = q_error_deg(ahrs_q, uncorrected_q)
            corrected = tuple(gyro[i] - bias[i] for i in range(3))
            values = (
                timestamp - calibration_end, dt, *integrated_q, *ahrs_q,
                *integrated_rpy, *ahrs_rpy, *differences, error,
                uncorrected_error, *corrected,
            )
            formatted = [f"{value:.12g}" for value in values]
            full_writer.writerow(formatted)
            if output_count % preview_stride == 0:
                preview_writer.writerow(formatted)
                plot_time.append(timestamp - calibration_end)
                for axis in range(3):
                    plot_integrated[axis].append(integrated_rpy[axis])
                    plot_ahrs[axis].append(ahrs_rpy[axis])
                plot_error.append(error)
                plot_uncorrected_error.append(uncorrected_error)
            output_count += 1
            error_sq_sum += error * error
            max_error = max(max_error, error)
            final_error = error
            uncorrected_error_sq_sum += uncorrected_error * uncorrected_error
            uncorrected_max_error = max(uncorrected_max_error, uncorrected_error)
            uncorrected_final_error = uncorrected_error

    if output_count == 0:
        raise ValueError("no valid samples after calibration")

    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    names = ("roll", "pitch", "yaw")
    for axis, name in enumerate(names):
        axes[axis].plot(plot_time, plot_integrated[axis], label="frozen-bias integration")
        axes[axis].plot(plot_time, plot_ahrs[axis], label="SJTU AHRS", alpha=0.8)
        axes[axis].set_ylabel(f"{name} (deg)")
        axes[axis].grid(True, alpha=0.25)
    axes[0].legend()
    axes[3].plot(plot_time, plot_error, label="with frozen startup bias")
    axes[3].plot(plot_time, plot_uncorrected_error,
                 label="without startup bias", alpha=0.8)
    axes[3].set_ylabel("quaternion error (deg)")
    axes[3].set_xlabel("time after calibration (s)")
    axes[3].grid(True, alpha=0.25)
    axes[3].legend()
    figure.suptitle("Initial static bias frozen, then gyro integration vs SJTU AHRS (ZYX)")
    figure.tight_layout()
    figure.savefig(plot_path, dpi=180)
    plt.close(figure)

    gyro_std = gyro_stats.standard_deviation()
    accel_std = accel_stats.standard_deviation()
    gravity_norm = math.sqrt(sum(value * value for value in mean_accel))
    duration = plot_time[-1] if plot_time else 0.0
    report_path.write_text(
        "\n".join((
            "# 静止 IMU 初始化与后段姿态积分验证",
            "",
            f"- 静止估偏窗口：前 `{calibration_end:.6f} s`，"
            f"`{gyro_stats.count}` 帧（目标 {seconds:g} s）",
            f"- FLU 陀螺启动零偏：`[{bias[0]:.9g}, {bias[1]:.9g}, {bias[2]:.9g}] rad/s`",
            f"- FLU 陀螺三轴标准差：`[{gyro_std[0]:.9g}, {gyro_std[1]:.9g}, {gyro_std[2]:.9g}] rad/s`",
            f"- FLU 平均比力：`[{mean_accel[0]:.9g}, {mean_accel[1]:.9g}, {mean_accel[2]:.9g}] m/s²`；模长 `{gravity_norm:.9g} m/s²`",
            f"- 加速度三轴标准差：`[{accel_std[0]:.9g}, {accel_std[1]:.9g}, {accel_std[2]:.9g}] m/s²`",
            f"- 重力初始化 ZYX 横滚/俯仰：`{math.degrees(roll):.6f}° / {math.degrees(pitch):.6f}°`",
            f"- yaw 仅取窗口末 AHRS 值 `{math.degrees(yaw_seed):.6f}°` 作为比较起点，不代表北向",
            f"- 初始化姿态与窗口末 AHRS 四元数夹角：`{initialization_error:.6f}°`",
            f"- 后段有效帧：`{output_count}`；约 `{duration:.3f} s`；非法积分间隔：`{invalid_intervals}`",
            f"- 后段四元数夹角误差：RMS `{math.sqrt(error_sq_sum / output_count):.6f}°`，"
            f"最大 `{max_error:.6f}°`，末值 `{final_error:.6f}°`",
            f"- 不扣启动零偏的对照误差：RMS "
            f"`{math.sqrt(uncorrected_error_sq_sum / output_count):.6f}°`，"
            f"最大 `{uncorrected_max_error:.6f}°`，末值 `{uncorrected_final_error:.6f}°`",
            "",
            "## 结论边界",
            "",
            "该结果检验坐标、单位、零偏扣除和姿态传播是否与上交 AHRS 内部一致。"
            "AHRS 四元数也由同一组陀螺/加速度计产生，因此不是独立真值；"
            "绝对精度仍需已知转角、转台、视觉或激光里程计等外部参考。",
            "单一静止姿态不能把三轴加速度计零偏与安装倾角、重力分量唯一分离，故这里不估加计零偏。",
            "",
            "## 过程文件",
            "",
            "- `静止初始化后姿态对比.csv.gz`：后段逐帧四元数、ZYX 欧拉角、扣偏/不扣偏误差和扣偏角速度。",
            "- `静止初始化后姿态对比_10Hz.csv`：便于直接查看的抽样文件。",
            "- `静止初始化后姿态对比.png`：三轴欧拉角与四元数夹角误差曲线。",
            "",
        )), encoding="utf-8")
    print(report_path)
    print(plot_path)
    print(full_path)
    print(preview_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="imu_full.csv or imu_full.csv.gz")
    parser.add_argument("--calibration-seconds", type=float, default=30.0)
    parser.add_argument("--gyro-scale-to-rad-s", type=float, default=1.0)
    parser.add_argument("--preview-stride", type=int, default=100)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.calibration_seconds) or args.calibration_seconds <= 0.0:
        parser.error("--calibration-seconds must be positive")
    if not math.isfinite(args.gyro_scale_to_rad_s) or args.gyro_scale_to_rad_s <= 0.0:
        parser.error("--gyro-scale-to-rad-s must be positive")
    if args.preview_stride <= 0:
        parser.error("--preview-stride must be positive")
    validate(args.input, args.output_dir or args.input.parent,
             args.calibration_seconds, args.gyro_scale_to_rad_s,
             args.preview_stride)


if __name__ == "__main__":
    main()

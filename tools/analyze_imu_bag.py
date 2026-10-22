#!/usr/bin/env python3
"""Analyze timing, values, orientation and covariance in a ROS 2 IMU bag."""

# 模块分工：命令行/路径校验 → ROS解码 → numpy全样本统计 → 标准输出报告。
# 本程序不估计姿态、不补偿零偏、不转换单位；只检查上游消息已有数值。
import argparse
import math
import sqlite3
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def _stamp_ns(stamp):
    # sec和nanosec先按整数拼接，保留纳秒精度。
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _format_stamp(ns):
    # divmod分别取整秒与余数，避免将绝对时间转成浮点后再格式化。
    seconds, nanoseconds = divmod(int(ns), 1_000_000_000)
    return f"{seconds}.{nanoseconds:09d}"


def _print_vector_stats(name, values):
    # values形状N×3；std默认ddof=0，即总体标准差，不是均值估计的不确定度。
    print(f"\n{name}（消息单位）")
    print("轴              mean            std            min            max")
    for axis, column in zip("xyz", values.T):
        print(
            f"{axis:<2} {np.mean(column):>16.9f} {np.std(column):>14.9f}"
            f" {np.min(column):>14.9f} {np.max(column):>14.9f}"
        )
    magnitudes = np.linalg.norm(values, axis=1)  # 每帧sqrt(x²+y²+z²)，再统计模长。
    print(
        f"模长 mean={np.mean(magnitudes):.9f}, std={np.std(magnitudes):.9f}, "
        f"min={np.min(magnitudes):.9f}, max={np.max(magnitudes):.9f}"
    )


def _print_time_anomalies(name, indices, sensor_ns, bag_ns, max_details):
    # indices指向异常间隔的后一帧；帧号从0开始，便于与完整CSV对应。
    print(f"{name}: {len(indices)} 次")
    for index in indices[:max_details]:
        delta_ns = int(sensor_ns[index] - sensor_ns[index - 1])
        bag_elapsed_s = (int(bag_ns[index]) - int(bag_ns[0])) / 1e9
        print(
            f"  帧 {index - 1}->{index}, bag相对时间={bag_elapsed_s:.6f}s, "
            f"header={_format_stamp(sensor_ns[index - 1])}"
            f"->{_format_stamp(sensor_ns[index])}, dt={delta_ns / 1e6:+.6f}ms"
        )
    if len(indices) > max_details:
        print(f"  ……另有 {len(indices) - max_details} 条未显示")


def _print_covariance(name, values, *, orientation=False):
    # 每行9元素。全零表示未提供协方差信息，不能解释为传感器无误差。
    # orientation[0]=-1表示未提供姿态估计；这里计数但不据此删原始姿态。
    finite = np.all(np.isfinite(values), axis=1)
    all_zero = np.all(values == 0.0, axis=1)
    unavailable = values[:, 0] == -1.0 if orientation else np.zeros(len(values), dtype=bool)
    print(
        f"{name}: 全零={np.count_nonzero(all_zero)}/{len(values)}, "
        f"非有限={np.count_nonzero(~finite)}/{len(values)}",
        end="",
    )
    if orientation:
        print(f", covariance[0]=-1={np.count_nonzero(unavailable)}/{len(values)}")
    else:
        print()


def analyze(bag_path, topic, max_details, orientation_jump_deg):
    # 主流程：读取单包全部目标消息，打印统计；外部入口用tee保存报告。
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise ValueError(f"不是有效的ROS 2 bag目录: {bag_path}")

    # 从metadata读取存储插件名；reader只读，不向数据库写入。
    metadata = rosbag2_py.Info().read_metadata(str(bag_path), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id=metadata.storage_identifier
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    # 先确认类型，避免把同名但不同结构的消息当作IMU解码。
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types)) or "无"
        raise ValueError(f"bag中没有Topic {topic}；现有Topic: {available}")
    if topic_types[topic] != "sensor_msgs/msg/Imu":
        raise ValueError(
            f"{topic} 类型是 {topic_types[topic]}，不是 sensor_msgs/msg/Imu"
        )
    message_type = get_message(topic_types[topic])

    # 两套时间同时保留：bag为记录端接收时间，sensor为消息header时间。
    bag_ns = []
    sensor_ns = []
    angular_velocity = []
    linear_acceleration = []
    orientation = []
    orientation_covariance = []
    angular_velocity_covariance = []
    linear_acceleration_covariance = []
    frame_ids = {}

    def records():
        # SQLite按写入序号读取，不能让reader按接收时间排序后制造Header假回退。
        if metadata.storage_identifier == "sqlite3":
            for filename in metadata.relative_file_paths:
                with sqlite3.connect((bag_path / filename).resolve().as_uri() + "?mode=ro", uri=True) as db:
                    yield from db.execute("SELECT t.name,m.data,m.timestamp FROM messages m "
                                          "JOIN topics t ON t.id=m.topic_id ORDER BY m.id")
        else:
            while reader.has_next():
                yield reader.read_next()

    for current_topic, serialized, received_ns in records():
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        bag_ns.append(received_ns)
        sensor_ns.append(_stamp_ns(message.header.stamp))
        angular_velocity.append(
            (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z)
        )
        linear_acceleration.append(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
        )
        # ROS字段顺序为xyzw，单位四元数本身无量纲。
        orientation.append(
            (message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w)
        )
        orientation_covariance.append(message.orientation_covariance)
        angular_velocity_covariance.append(message.angular_velocity_covariance)
        linear_acceleration_covariance.append(message.linear_acceleration_covariance)
        frame_ids[message.header.frame_id] = frame_ids.get(message.header.frame_id, 0) + 1

    if not bag_ns:
        raise ValueError(f"{topic} 没有消息")

    # 时间用int64；测量用float64计算，但不会恢复上游float32已损失的精度。
    bag_ns = np.asarray(bag_ns, dtype=np.int64)
    sensor_ns = np.asarray(sensor_ns, dtype=np.int64)
    angular_velocity = np.asarray(angular_velocity, dtype=np.float64)
    linear_acceleration = np.asarray(linear_acceleration, dtype=np.float64)
    orientation = np.asarray(orientation, dtype=np.float64)
    orientation_covariance = np.asarray(orientation_covariance, dtype=np.float64)
    angular_velocity_covariance = np.asarray(angular_velocity_covariance, dtype=np.float64)
    linear_acceleration_covariance = np.asarray(linear_acceleration_covariance, dtype=np.float64)

    count = len(bag_ns)
    bag_duration_ns = int(bag_ns[-1] - bag_ns[0]) if count > 1 else 0
    # N帧对应N-1个时间间隔。接收频率不等同于硬件实际采样频率。
    bag_rate_hz = (count - 1) * 1e9 / bag_duration_ns if bag_duration_ns > 0 else math.nan
    valid_sensor = sensor_ns != 0
    sensor_deltas = np.diff(sensor_ns)
    positive_deltas = sensor_deltas[sensor_deltas > 0]
    # 用正间隔中位数估计典型周期；负数和重复不参与周期估计。
    nominal_ns = int(np.median(positive_deltas)) if len(positive_deltas) else 0
    rollback_indices = np.flatnonzero(sensor_deltas < 0) + 1  # diff索引比后一帧小1。
    duplicate_indices = np.flatnonzero(sensor_deltas == 0) + 1
    # 大跳门限为10倍典型周期与100ms中的较大值；小于门限不代表完全正常。
    jump_threshold_ns = max(10 * nominal_ns, 100_000_000) if nominal_ns else 100_000_000
    forward_jump_indices = np.flatnonzero(sensor_deltas > jump_threshold_ns) + 1

    print(f"bag: {bag_path}")
    print(f"Topic: {topic} [{topic_types[topic]}]")
    print(f"消息数: {count}")
    print(f"bag接收时长: {bag_duration_ns / 1e9:.9f}s")
    print(f"bag平均接收频率: {bag_rate_hz:.6f}Hz")
    print(f"header非零: {np.count_nonzero(valid_sensor)}/{count}")
    print(f"frame_id: {frame_ids}")
    if np.any(valid_sensor):
        valid_values = sensor_ns[valid_sensor]
        print(
            f"header首末时间: {_format_stamp(valid_values[0])}"
            f" -> {_format_stamp(valid_values[-1])}"
        )
        print(f"header首末跨度: {(int(valid_values[-1]) - int(valid_values[0])) / 1e9:.9f}s")
    print(f"header标称间隔（正间隔中位数）: {nominal_ns / 1e6:.6f}ms")
    if np.all(valid_sensor):
        delay_ms = (bag_ns - sensor_ns) / 1e6  # 含时钟偏移，未必是纯传输延迟。
        print(
            "bag接收时间-header时间: "
            f"min={np.min(delay_ms):.6f}ms, median={np.median(delay_ms):.6f}ms, "
            f"max={np.max(delay_ms):.6f}ms"
        )

    print("\n时间戳异常")
    _print_time_anomalies("回退", rollback_indices, sensor_ns, bag_ns, max_details)
    _print_time_anomalies("重复", duplicate_indices, sensor_ns, bag_ns, max_details)
    _print_time_anomalies(
        f"正向大跳（阈值>{jump_threshold_ns / 1e6:.3f}ms）",
        forward_jump_indices,
        sensor_ns,
        bag_ns,
        max_details,
    )

    _print_vector_stats("角速度", angular_velocity)
    _print_vector_stats("线加速度", linear_acceleration)

    # 姿态连续性：只比较原始相邻有效四元数，不跨越无效帧连接。
    norms = np.linalg.norm(orientation, axis=1)
    finite_quaternions = np.all(np.isfinite(orientation), axis=1) & (norms > 1e-12)
    pair_valid = finite_quaternions[:-1] & finite_quaternions[1:]
    steps_deg = np.full(max(count - 1, 0), np.nan)
    if np.any(pair_valid):
        normalized = orientation[finite_quaternions] / norms[finite_quaternions, None]
        normalized_by_index = np.full_like(orientation, np.nan)  # 保持原帧索引。
        normalized_by_index[finite_quaternions] = normalized
        # q与-q表示同一旋转；绝对内积排除等价变号产生的假突变。
        dots = np.abs(
            np.sum(normalized_by_index[:-1] * normalized_by_index[1:], axis=1)
        )
        # 最短相对旋转角=2 acos(|q1·q2|)，clip抑制舍入越界；输出角度0～180°。
        # 这是旋转角幅值，既非ZYX欧拉角差，也非除以dt后的角速度。
        steps_deg[pair_valid] = np.degrees(2.0 * np.arccos(np.clip(dots[pair_valid], 0.0, 1.0)))

    print("\n姿态四元数")
    print(f"有效单位化输入: {np.count_nonzero(finite_quaternions)}/{count}")
    if np.any(np.isfinite(steps_deg)):
        max_step_offset = int(np.nanargmax(steps_deg))
        current = max_step_offset + 1
        print(f"最大相邻旋转步进: {steps_deg[max_step_offset]:.9f}deg")
        print(
            f"  帧 {current - 1}->{current}, bag相对时间="
            f"{(int(bag_ns[current]) - int(bag_ns[0])) / 1e9:.6f}s"
        )
        print(f"  前一帧xyzw: {orientation[current - 1].tolist()}")
        print(f"  当前帧xyzw: {orientation[current].tolist()}")
        jump_indices = np.flatnonzero(steps_deg > orientation_jump_deg) + 1
        print(f"姿态突变（>{orientation_jump_deg:g}deg）: {len(jump_indices)} 次")
        for index in jump_indices[:max_details]:
            print(
                f"  帧 {index - 1}->{index}, bag相对时间="
                f"{(int(bag_ns[index]) - int(bag_ns[0])) / 1e9:.9f}s, "
                f"header={_format_stamp(sensor_ns[index])}, "
                f"步进={steps_deg[index - 1]:.9f}deg"
            )
        if len(jump_indices) > max_details:
            print(f"  ……另有 {len(jump_indices) - max_details} 条未显示")
        # 多次突变分别在接收/header时间下求间隔，便于排查周期性重置。
        if len(jump_indices) > 1:
            bag_intervals = np.diff(bag_ns[jump_indices]).astype(np.float64) * 1.0e-9
            header_intervals = (
                np.diff(sensor_ns[jump_indices]).astype(np.float64) * 1.0e-9
            )
            print(
                "  相邻突变bag间隔(s): "
                + ", ".join(f"{value:.9f}" for value in bag_intervals)
            )
            print(
                "  相邻突变header间隔(s): "
                + ", ".join(f"{value:.9f}" for value in header_intervals)
            )
    else:
        print("最大相邻旋转步进: 无法计算")

    print("\n协方差")
    _print_covariance("orientation_covariance", orientation_covariance, orientation=True)
    _print_covariance("angular_velocity_covariance", angular_velocity_covariance)
    _print_covariance("linear_acceleration_covariance", linear_acceleration_covariance)


def main():
    # 命令行入口：阈值只控制报告列举，不改变原始数据或算法。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag目录")
    parser.add_argument("--topic", default="/imu/raw", help="IMU Topic（默认：/imu/raw）")
    parser.add_argument(
        "--max-details", type=int, default=20, help="每类时间异常最多显示条数（默认：20）"
    )
    parser.add_argument(
        "--orientation-jump-deg", type=float, default=10.0,
        help="列出超过该角度的相邻姿态突变（默认：10deg）",
    )
    args = parser.parse_args()
    if args.max_details < 0:
        parser.error("--max-details不能为负数")
    if not math.isfinite(args.orientation_jump_deg) or args.orientation_jump_deg <= 0.0:
        parser.error("--orientation-jump-deg必须是正的有限数")
    try:
        analyze(
            args.bag.expanduser().resolve(), args.topic, args.max_details,
            args.orientation_jump_deg,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

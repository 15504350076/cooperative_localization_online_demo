#!/usr/bin/env python3
"""分析 Livox ROS 2 雷达短包。

功能：读取 /livox/lidar 和 /livox/imu，比较录包接收时间与消息头时间，
检查消息频率、时间回退、点云尺寸、点字段、零点比例和点内 timestamp。
程序只读 bag，不修改数据库，也不把接收时间差直接当作传输延迟。

运行前加载 ROS 2 Humble：
  source /opt/ros/humble/setup.bash
  python3 tools/analyze_lidar_bag.py data/lidar_20240626_044813
"""

from __future__ import annotations

import argparse
import bisect
import math
import statistics
import sqlite3
from collections import Counter
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


NS = 1_000_000_000


def stamp_ns(message) -> int:
    """把 ROS 时间字段合成为整数纳秒，避免浮点时间损失精度。"""
    return int(message.header.stamp.sec) * NS + int(message.header.stamp.nanosec)


def time_stats(values: list[int]) -> dict[str, float | int]:
    """统计一组按录包顺序排列的时间，包括接收间隔和回退数量。"""
    deltas = [b - a for a, b in zip(values, values[1:])]
    positive = [delta for delta in deltas if delta > 0]
    nominal = statistics.median(positive) if positive else 0
    return {
        "count": len(values),
        "span_s": (values[-1] - values[0]) / NS if len(values) > 1 else 0.0,
        "rate_hz": (len(values) - 1) / ((values[-1] - values[0]) / NS)
        if len(values) > 1 and values[-1] > values[0]
        else math.nan,
        "median_ms": nominal / 1e6 if nominal else math.nan,
        "min_ms": min(deltas) / 1e6 if deltas else math.nan,
        "max_ms": max(deltas) / 1e6 if deltas else math.nan,
        "backward": sum(delta < 0 for delta in deltas),
        "duplicate": sum(delta == 0 for delta in deltas),
        "over_2x": sum(delta > 2 * nominal for delta in positive)
        if nominal
        else 0,
    }


def print_time_stats(label: str, values: list[int]) -> None:
    """打印一组时间的可读统计。"""
    result = time_stats(values)
    print(
        f"{label}: 数量={result['count']}, 跨度={result['span_s']:.9f}s, "
        f"频率={result['rate_hz']:.6f}Hz, "
        f"间隔中位数/最小值/最大值={result['median_ms']:.6f}/"
        f"{result['min_ms']:.6f}/{result['max_ms']:.6f}ms, "
        f"回退次数={result['backward']}, 重复次数={result['duplicate']}, "
        f"超过两倍典型间隔次数={result['over_2x']}"
    )


def read_bag(path: Path) -> dict[str, object]:
    """读取目标 bag，保留统计所需的时间和点云字段信息。"""
    if not path.is_dir() or not (path / "metadata.yaml").is_file():
        raise ValueError(f"不是有效的 ROS 2 bag 目录：{path}")

    metadata = rosbag2_py.Info().read_metadata(str(path), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=metadata.storage_identifier),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        "/livox/lidar": "sensor_msgs/msg/PointCloud2",
        "/livox/imu": "sensor_msgs/msg/Imu",
    }
    for topic, expected in required.items():
        if topic not in topic_types:
            raise ValueError(f"bag 缺少 {topic}")
        if topic_types[topic] != expected:
            raise ValueError(f"{topic} 类型为 {topic_types[topic]}，不是 {expected}")

    message_types = {"/livox/lidar": PointCloud2, "/livox/imu": Imu}
    result: dict[str, object] = {
        topic: {"receive": [], "header": [], "frames": Counter(), "frame_ids": Counter()}
        for topic in required
    }
    lidar = result["/livox/lidar"]
    total_points = 0
    finite_points = 0
    zero_points = 0
    point_timestamp_spans_ms: list[float] = []
    point_timestamp_first_offsets_us: list[float] = []
    point_fields = None
    point_meta = None

    def records():
        # reader按记录时间重排会掩盖系统时间回退，也可能制造Header乱序。
        if metadata.storage_identifier == "sqlite3":
            for filename in metadata.relative_file_paths:
                with sqlite3.connect((path / filename).resolve().as_uri() + "?mode=ro", uri=True) as db:
                    yield from db.execute("SELECT t.name,m.data,m.timestamp FROM messages m "
                                          "JOIN topics t ON t.id=m.topic_id ORDER BY m.id")
        else:
            while reader.has_next():
                yield reader.read_next()

    for topic, serialized, receive_ns in records():
        if topic not in required:
            continue
        message = deserialize_message(serialized, message_types[topic])
        entry = result[topic]
        entry["receive"].append(int(receive_ns))
        entry["header"].append(stamp_ns(message))
        entry["frame_ids"][message.header.frame_id] += 1

        if topic != "/livox/lidar":
            continue

        entry["frames"][(message.width, message.height)] += 1
        fields = {field.name for field in message.fields}
        if point_fields is None:
            point_fields = tuple(
                (field.name, field.offset, field.datatype, field.count)
                for field in message.fields
            )
            point_meta = {
                "point_step": message.point_step,
                "row_step": message.row_step,
                "is_bigendian": message.is_bigendian,
                "is_dense": message.is_dense,
            }
        if not {"x", "y", "z"}.issubset(fields):
            continue

        wanted = ["x", "y", "z"] + (["timestamp"] if "timestamp" in fields else [])
        points = point_cloud2.read_points(message, field_names=wanted, skip_nans=False)
        xyz = np.column_stack((points["x"], points["y"], points["z"]))
        finite = np.isfinite(xyz).all(axis=1)
        zeros = np.all(xyz == 0.0, axis=1)
        total_points += len(points)
        finite_points += int(np.count_nonzero(finite))
        zero_points += int(np.count_nonzero(zeros))
        if "timestamp" in fields and len(points):
            point_timestamp_first_offsets_us.append(
                (float(points["timestamp"][0]) - stamp_ns(message)) / 1e3
            )
            point_timestamp_spans_ms.append(
                (float(points["timestamp"][-1]) - float(points["timestamp"][0])) / 1e6
            )

    lidar["point_fields"] = point_fields
    lidar["point_meta"] = point_meta
    lidar["total_points"] = total_points
    lidar["finite_points"] = finite_points
    lidar["zero_points"] = zero_points
    lidar["point_timestamp_spans_ms"] = point_timestamp_spans_ms
    lidar["point_timestamp_first_offsets_us"] = point_timestamp_first_offsets_us
    for topic in required:
        if not result[topic]["receive"]:
            raise ValueError(f"bag 中 {topic} 没有消息")
    return result


def print_report(path: Path, data: dict[str, object]) -> None:
    """打印离线分析报告。"""
    print(f"数据包: {path}")
    lidar = data["/livox/lidar"]
    imu = data["/livox/imu"]
    for topic, entry in (("/livox/lidar", lidar), ("/livox/imu", imu)):
        print(f"\n[{topic}] 坐标系标识={dict(entry['frame_ids'])}")
        print_time_stats("记录时间", entry["receive"])
        print_time_stats("消息时间", entry["header"])
        delays = [
            (receive - header) / 1e6
            for receive, header in zip(entry["receive"], entry["header"])
        ]
        print(
            "接收时间-消息时间（含时钟偏移，不等于纯传输延迟）: "
            f"最小值/中位数/最大值={min(delays):.6f}/{statistics.median(delays):.6f}/"
            f"{max(delays):.6f}ms"
        )

    print("\n点云字段")
    print(f"尺寸计数: {dict(lidar['frames'])}")
    print(f"字段: {lidar['point_fields']}")
    print(f"元数据: {lidar['point_meta']}")
    print(
        f"点总数={lidar['total_points']}, 有限XYZ={lidar['finite_points']}, "
        f"XYZ全零={lidar['zero_points']} "
        f"({lidar['zero_points'] / max(lidar['total_points'], 1) * 100:.3f}%)"
    )
    spans = lidar["point_timestamp_spans_ms"]
    offsets = lidar["point_timestamp_first_offsets_us"]
    if spans:
        print(
            "逐点timestamp（按纳秒时间解释）: "
            f"帧内跨度 中位数/最小值/最大值={statistics.median(spans):.6f}/"
            f"{min(spans):.6f}/{max(spans):.6f}ms, "
            f"首点相对Header 中位数/最小值/最大值={statistics.median(offsets):.6f}/"
            f"{min(offsets):.6f}/{max(offsets):.6f}us"
        )

    imu_headers = sorted(data["/livox/imu"]["header"])
    lidar_headers = data["/livox/lidar"]["header"]
    nearest_ms = []
    for header in lidar_headers:
        index = bisect.bisect_left(imu_headers, header)
        candidates = [index - 1, index]
        nearest_ms.append(
            min(
                ((imu_headers[i] - header) / 1e6 for i in candidates if 0 <= i < len(imu_headers)),
                key=abs,
            )
        )
    print(
        "\n雷达Header与最近内置IMU Header差: "
        f"全部 中位数/最小值/最大值={statistics.median(nearest_ms):.6f}/"
        f"{min(nearest_ms):.6f}/{max(nearest_ms):.6f}ms; "
        f"去掉首帧后 中位数/最大绝对值={(statistics.median(nearest_ms[1:]) if len(nearest_ms) > 1 else math.nan):.6f}/"
        f"{max((abs(value) for value in nearest_ms[1:]), default=math.nan):.6f}ms"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="分析 Livox ROS 2 雷达短包")
    parser.add_argument("bag", type=Path, help="rosbag2 目录，例如 data/lidar_20240626_044813")
    args = parser.parse_args()
    try:
        print_report(args.bag, read_bag(args.bag))
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

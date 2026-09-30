#!/usr/bin/env python3
"""比较201盒 GNSS 与 IMU 联合录包中的消息时间。"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NS = 1_000_000_000


def stamp_ns(stamp: Any) -> int:
    """把 ROS 时间戳转换为整数纳秒，避免绝对时间浮点化丢精度。"""
    return int(stamp.sec) * NS + int(stamp.nanosec)


def iso_utc(ns: int) -> str:
    """将 Unix 纳秒显示为 UTC，便于核对录包日期。"""
    return datetime.fromtimestamp(ns / NS, tz=timezone.utc).isoformat(timespec="milliseconds")


def read_times(bag: Path, topic: str) -> tuple[np.ndarray, np.ndarray, str]:
    """读取一个子包的消息 header 时间和 rosbag 接收时间。"""
    metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata.storage_identifier),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"{bag} 中没有 {topic}，实际 Topic：{sorted(topic_types)}")
    message_type = get_message(topic_types[topic])
    headers: list[int] = []
    receives: list[int] = []
    while reader.has_next():
        current_topic, serialized, receive_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_type)
        headers.append(stamp_ns(message.header.stamp))
        receives.append(int(receive_ns))
    if not headers:
        raise RuntimeError(f"{bag} 的 {topic} 没有消息")
    return np.asarray(headers, dtype=np.int64), np.asarray(receives, dtype=np.int64), topic_types[topic]


def nearest_signed(reference: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """对每个reference找最近target，返回 target-reference 和绝对误差。"""
    positions = np.searchsorted(target, reference)
    right = np.minimum(positions, len(target) - 1)
    left = np.maximum(positions - 1, 0)
    right_abs = np.abs(target[right] - reference)
    left_abs = np.abs(target[left] - reference)
    use_left = left_abs <= right_abs
    selected = np.where(use_left, left, right)
    signed = target[selected].astype(np.int64) - reference.astype(np.int64)
    return signed, np.abs(signed)


def stats(values_ns: np.ndarray) -> dict[str, float | int | None]:
    """返回纳秒数组的毫秒统计。"""
    if len(values_ns) == 0:
        return {"count": 0, "mean_ms": None, "median_ms": None, "std_ms": None,
                "min_ms": None, "max_ms": None, "p05_ms": None, "p95_ms": None}
    values = values_ns.astype(np.float64) / 1e6
    return {
        "count": int(len(values)),
        "mean_ms": float(np.mean(values)),
        "median_ms": float(np.median(values)),
        "std_ms": float(np.std(values)),
        "min_ms": float(np.min(values)),
        "max_ms": float(np.max(values)),
        "p05_ms": float(np.percentile(values, 5)),
        "p95_ms": float(np.percentile(values, 95)),
    }


def fmt(value: float | int | None, digits: int = 4) -> str:
    """统一显示报告数字。"""
    if value is None:
        return "无"
    return f"{float(value):.{digits}f}"


def monotonic_stats(values: np.ndarray) -> dict[str, Any]:
    """统计零时间、回退、重复及相邻周期。"""
    deltas = np.diff(values)
    positive = deltas[deltas > 0]
    return {
        "count": int(len(values)),
        "zero": int(np.count_nonzero(values == 0)),
        "rollback": int(np.count_nonzero(deltas < 0)),
        "duplicate": int(np.count_nonzero(deltas == 0)),
        "span_s": float((values[-1] - values[0]) / NS),
        "period_median_ms": float(np.median(positive) / 1e6) if len(positive) else None,
        "period_mean_ms": float(np.mean(positive) / 1e6) if len(positive) else None,
        "period_min_ms": float(np.min(positive) / 1e6) if len(positive) else None,
        "period_max_ms": float(np.max(positive) / 1e6) if len(positive) else None,
    }


def read_record_info(root: Path) -> dict[str, str]:
    """读取录包脚本写下的墙钟记录信息；缺失时返回空字典。"""
    path = root / "record_info.txt"
    if not path.is_file():
        return {}
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="比较GNSS和IMU联合包的header与接收时间")
    parser.add_argument("root", type=Path, help="包含 gnss/、imu/ 子包的联合包目录")
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    gnss_header, gnss_receive, gnss_type = read_times(root / "gnss", "/gnss/fix")
    imu_header, imu_receive, imu_type = read_times(root / "imu", "/imu/raw")

    # 录包顺序天然递增；若不是，搜索最近值前不掩盖问题，直接提示。
    gnss_header_stat = monotonic_stats(gnss_header)
    imu_header_stat = monotonic_stats(imu_header)
    gnss_receive_stat = monotonic_stats(gnss_receive)
    imu_receive_stat = monotonic_stats(imu_receive)

    # 以GNSS低频样本为参考，分别找最近的IMU header/接收时间。
    header_signed, header_abs = nearest_signed(gnss_header, imu_header)
    receive_signed, receive_abs = nearest_signed(gnss_receive, imu_receive)
    gnss_receive_minus_header = gnss_receive - gnss_header
    imu_receive_minus_header = imu_receive - imu_header

    rows: list[dict[str, Any]] = []
    for index in range(len(gnss_header)):
        rows.append({
            "gnss_index": index,
            "gnss_header_ns": int(gnss_header[index]),
            "gnss_header_utc": iso_utc(int(gnss_header[index])),
            "gnss_receive_ns": int(gnss_receive[index]),
            "gnss_receive_utc": iso_utc(int(gnss_receive[index])),
            "nearest_imu_header_minus_gnss_header_ms": float(header_signed[index] / 1e6),
            "nearest_imu_header_abs_error_ms": float(header_abs[index] / 1e6),
            "nearest_imu_receive_minus_gnss_receive_ms": float(receive_signed[index] / 1e6),
            "nearest_imu_receive_abs_error_ms": float(receive_abs[index] / 1e6),
        })
    csv_path = root / "GNSS与IMU时间对比.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    info = read_record_info(root)
    report = f"""# GNSS 与 IMU 联合包时间分析

## 1. 数据

- 联合包：`{root}`
- GNSS 子包：`gnss/`，`/gnss/fix`，`{gnss_type}`
- IMU 子包：`imu/`，`/imu/raw`，`{imu_type}`
- 录包墙钟信息：`{info or '未找到 record_info.txt'}`

## 2. 数量、跨度和单流时间质量

| 流 | 帧数 | Header跨度(s) | Header中位周期(ms) | Header回退 | Header重复 | 接收跨度(s) | 接收回退 |
|---|---:|---:|---:|---:|---:|---:|---:|
| GNSS | {gnss_header_stat['count']} | {fmt(gnss_header_stat['span_s'], 6)} | {fmt(gnss_header_stat['period_median_ms'], 6)} | {gnss_header_stat['rollback']} | {gnss_header_stat['duplicate']} | {fmt(gnss_receive_stat['span_s'], 6)} | {gnss_receive_stat['rollback']} |
| IMU | {imu_header_stat['count']} | {fmt(imu_header_stat['span_s'], 6)} | {fmt(imu_header_stat['period_median_ms'], 6)} | {imu_header_stat['rollback']} | {imu_header_stat['duplicate']} | {fmt(imu_receive_stat['span_s'], 6)} | {imu_receive_stat['rollback']} |

GNSS 消息频率约 1 Hz，IMU 约 1000 Hz；这是两种采样频率，不是“GNSS 10 ms、IMU 200 ms”。

## 3. 两种时间的定义

- `header.stamp`：消息内的测量时间（或驱动赋予的传感器时间）。
- rosbag 接收时间：该子包记录器收到并写入消息的时间。
- 同一盒的 `receive - header`：包含时间基准差、驱动处理和记录链路，不能直接等同于纯网络延迟。

| 流 | `receive - header`中位数(ms) | P05(ms) | P95(ms) | 最小(ms) | 最大(ms) |
|---|---:|---:|---:|---:|---:|
| GNSS | {fmt(np.median(gnss_receive_minus_header) / 1e6)} | {fmt(np.percentile(gnss_receive_minus_header, 5) / 1e6)} | {fmt(np.percentile(gnss_receive_minus_header, 95) / 1e6)} | {fmt(np.min(gnss_receive_minus_header) / 1e6)} | {fmt(np.max(gnss_receive_minus_header) / 1e6)} |
| IMU | {fmt(np.median(imu_receive_minus_header) / 1e6)} | {fmt(np.percentile(imu_receive_minus_header, 5) / 1e6)} | {fmt(np.percentile(imu_receive_minus_header, 95) / 1e6)} | {fmt(np.min(imu_receive_minus_header) / 1e6)} | {fmt(np.max(imu_receive_minus_header) / 1e6)} |

## 4. GNSS 与 IMU 的最近时间对应

以每一条 GNSS 消息为参考，在 IMU 消息中寻找最近的时间戳。由于 IMU 约 1 ms 一帧，最近邻误差还包含最多约 0.5 ms 的采样量化误差。

| 对比量 | 中位数(ms) | P05(ms) | P95(ms) | 最小(ms) | 最大(ms) |
|---|---:|---:|---:|---:|---:|
| 最近 IMU Header − GNSS Header | {fmt(np.median(header_signed) / 1e6)} | {fmt(np.percentile(header_signed, 5) / 1e6)} | {fmt(np.percentile(header_signed, 95) / 1e6)} | {fmt(np.min(header_signed) / 1e6)} | {fmt(np.max(header_signed) / 1e6)} |
| 最近 IMU 接收 − GNSS 接收 | {fmt(np.median(receive_signed) / 1e6)} | {fmt(np.percentile(receive_signed, 5) / 1e6)} | {fmt(np.percentile(receive_signed, 95) / 1e6)} | {fmt(np.min(receive_signed) / 1e6)} | {fmt(np.max(receive_signed) / 1e6)} |

“最近”只解决不同频率下如何配对，不能证明 GNSS 和 IMU 硬件已经统一授时。最终还要看最近差值是否稳定、是否随录制时间持续漂移，并让上交确认两路 `header.stamp` 的时间源。

## 5. 初步结论

- 这次联合录包同时包含完整 GNSS 和 IMU 数据，可用于继续做时间对应分析。
- 两路单流 Header 均应检查回退、重复和异常大间隔；本报告列出统计，不修改原始消息。
- GNSS 只有约 1 Hz，不能用 GNSS 帧率评价 IMU 采样率。
- GNSS 静止散布和 IMU 静止偏置应分别查看同目录下的 `gnss/GNSS户外静止分析.md` 与 `imu/imu_analysis.txt`。
- 只有在确认两路 Header 属于同一时间基准后，GNSS 才能作为 IMU/UWB 评价的时间参考；GNSS 位置仍是201的真值，不会自动成为三车相对定位输入。

## 6. 输出文件

- [`GNSS与IMU时间对比.csv`](GNSS与IMU时间对比.csv)：每条GNSS消息对应的最近IMU时间。
- [`gnss/GNSS户外静止分析.md`](gnss/GNSS户外静止分析.md)：GNSS位置、状态和单流时间。
- [`imu/imu_analysis.txt`](imu/imu_analysis.txt)：IMU单流时间、静止统计和姿态突变检查。
    """
    report_path = root / "GNSS与IMU联合时间分析.md"
    report_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"已生成：{report_path}")
    print(f"已生成：{csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

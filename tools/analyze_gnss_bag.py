#!/usr/bin/env python3
"""离线分析 GNSS ROS 2 bag。

这个脚本只读取 bag，不会修改 bag，也不会连接盒子。它同时保留：
1. rosbag 记录时刻（receive_ns）；
2. GNSS 消息 Header 中的测量时刻（message_ns）；
3. 节点号、定位状态、经纬高和协方差。

运行前需要 source ROS 2 和包含 cooperative_interfaces 的工作空间，
否则 rosbag2_py 可以打开 bag，但无法反序列化自定义消息。
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_ns(stamp: Any) -> int:
    """把 ROS 2 builtin_interfaces/Time 转成纳秒整数。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite(value: Any) -> bool:
    """判断一个数是否可用于统计。"""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def percentile(values: list[float], fraction: float) -> float | None:
    """线性插值计算分位数；空数组返回 None。"""
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summary(values: Iterable[float], scale: float = 1.0) -> dict[str, float | int | None]:
    """返回数量、均值、标准差、最小值、最大值和 P95。"""
    raw = [float(x) * scale for x in values if finite(x)]
    if not raw:
        return {"count": 0, "mean": None, "std": None, "min": None, "max": None, "p95": None}
    return {
        "count": len(raw),
        "mean": statistics.fmean(raw),
        "std": statistics.pstdev(raw) if len(raw) > 1 else 0.0,
        "min": min(raw),
        "max": max(raw),
        "p95": percentile(raw, 0.95),
    }


def fmt(value: Any, digits: int = 3) -> str:
    """把报告中的数值统一格式化。"""
    if value is None:
        return "无"
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.{digits}f}"


def iso_utc(ns: int | None) -> str:
    """将纳秒时间戳显示为 UTC；主要用于确认数据日期。"""
    if ns is None:
        return "无"
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat(timespec="milliseconds")


def field(message: Any, name: str, default: Any = None) -> Any:
    """兼容自定义 GnssPosition 和 sensor_msgs/NavSatFix。"""
    return getattr(message, name, default)


def read_messages(bag: Path, topic: str) -> tuple[list[dict[str, Any]], dict[str, int], dict[str, str]]:
    """读取 GNSS 消息，并统计 bag 中每个 Topic 的条数。"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(f"bag 中没有 Topic {topic}，实际 Topic：{sorted(topic_types)}")
    message_class = get_message(topic_types[topic])
    rows: list[dict[str, Any]] = []
    counts: dict[str, int] = {name: 0 for name in topic_types}
    while reader.has_next():
        name, raw, receive_ns = reader.read_next()
        counts[name] = counts.get(name, 0) + 1
        if name != topic:
            continue
        message = deserialize_message(raw, message_class)
        header = field(message, "header")
        message_ns = stamp_to_ns(header.stamp) if header is not None else None
        status_value = field(message, "status")
        if hasattr(status_value, "status"):
            status_value = status_value.status
        node_id = field(message, "node_id")
        if node_id is None:
            node_id = field(message, "src_id")
        row = {
            "receive_ns": int(receive_ns),
            "message_ns": message_ns,
            "node_id": int(node_id) if node_id is not None else None,
            "status": int(status_value) if status_value is not None else None,
            "latitude": float(field(message, "latitude", float("nan"))),
            "longitude": float(field(message, "longitude", float("nan"))),
            "altitude": float(field(message, "altitude", float("nan"))),
            "covariance_type": int(field(message, "position_covariance_type", 0)),
            "covariance": list(field(message, "position_covariance", [])),
        }
        rows.append(row)
    rows.sort(key=lambda row: row["receive_ns"])
    return rows, counts, topic_types


def analyze(rows: list[dict[str, Any]], counts: dict[str, int], topic_types: dict[str, str], bag: Path) -> tuple[str, list[dict[str, Any]]]:
    """生成 CSV 行和中文 Markdown 结论。"""
    if not rows:
        raise RuntimeError("没有读取到 GNSS 消息")
    receive_ns = [row["receive_ns"] for row in rows]
    message_ns = [row["message_ns"] for row in rows if row["message_ns"] is not None]
    receive_dt = [b - a for a, b in zip(receive_ns, receive_ns[1:])]
    message_dt = [b - a for a, b in zip(message_ns, message_ns[1:])]
    offsets = [row["receive_ns"] - row["message_ns"] for row in rows if row["message_ns"] is not None]
    rollbacks = sum(b <= a for a, b in zip(message_ns, message_ns[1:]))
    receive_rollbacks = sum(b <= a for a, b in zip(receive_ns, receive_ns[1:]))
    node_ids = sorted({row["node_id"] for row in rows if row["node_id"] is not None})
    statuses: dict[int, int] = {}
    for row in rows:
        if row["status"] is not None:
            statuses[row["status"]] = statuses.get(row["status"], 0) + 1
    valid = [
        row for row in rows
        if row["status"] is not None and row["status"] >= 0
        and finite(row["latitude"]) and finite(row["longitude"]) and finite(row["altitude"])
        and -90.0 <= row["latitude"] <= 90.0 and -180.0 <= row["longitude"] <= 180.0
        and not (row["latitude"] == 0.0 and row["longitude"] == 0.0)
    ]
    mean_lat = statistics.fmean(row["latitude"] for row in valid) if valid else None
    mean_lon = statistics.fmean(row["longitude"] for row in valid) if valid else None
    east_m: list[float] = []
    north_m: list[float] = []
    if valid and mean_lat is not None and mean_lon is not None:
        lon_scale = 111_320.0 * math.cos(math.radians(mean_lat))
        for row in valid:
            east_m.append((row["longitude"] - mean_lon) * lon_scale)
            north_m.append((row["latitude"] - mean_lat) * 110_540.0)
    horizontal_m = [math.hypot(east, north) for east, north in zip(east_m, north_m)]
    latitudes = [row["latitude"] for row in valid]
    longitudes = [row["longitude"] for row in valid]
    altitude = [row["altitude"] for row in valid]
    csv_rows: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        csv_rows.append({
            "index": index,
            "receive_ns": row["receive_ns"],
            "receive_utc": iso_utc(row["receive_ns"]),
            "message_ns": row["message_ns"] or "",
            "message_utc": iso_utc(row["message_ns"]),
            "receive_minus_message_ms": (row["receive_ns"] - row["message_ns"]) / 1e6 if row["message_ns"] is not None else "",
            "node_id": row["node_id"] if row["node_id"] is not None else "",
            "status": row["status"] if row["status"] is not None else "",
            "latitude_deg": row["latitude"],
            "longitude_deg": row["longitude"],
            "altitude_m": row["altitude"],
            "valid": int(row in valid),
            "covariance_type": row["covariance_type"],
        })

    report = f"""# GNSS 户外静止包离线分析

## 1. 数据来源

- Bag：`{bag}`
- GNSS Topic：`/gnss/fix`
- 消息类型：`{topic_types.get('/gnss/fix', '未知')}`
- `/gnss/fix` 条数：{counts.get('/gnss/fix', 0)}
- 其他 Topic：{', '.join(f'`{key}`={value}' for key, value in counts.items() if key != '/gnss/fix') or '无'}
- 记录接收时段（UTC）：`{iso_utc(receive_ns[0])}` ～ `{iso_utc(receive_ns[-1])}`
- 记录时长：{fmt((receive_ns[-1] - receive_ns[0]) / 1e9, 3)} s
- 识别到的 node_id：{node_ids or '无'}

## 2. 结论摘要

- 本包包含 **{len(rows)} 条 GNSS 消息**，接收时间连续性回退/重复数：**{receive_rollbacks}**。
- 消息 Header 时间回退/重复数：**{rollbacks}**。
- 定位状态分布：`{statuses}`。
- 按 `status >= 0`、经纬高有限且经纬度非零定义的有效帧：**{len(valid)}/{len(rows)}（{100.0 * len(valid) / len(rows):.2f}%）**。
- 有效帧平均位置：纬度 **{fmt(statistics.fmean(latitudes), 9) if latitudes else '无'}°**，经度 **{fmt(statistics.fmean(longitudes), 9) if longitudes else '无'}°**。
- 有效帧经纬范围：纬度 **{fmt(min(latitudes), 9) if latitudes else '无'}～{fmt(max(latitudes), 9) if latitudes else '无'}°**，经度 **{fmt(min(longitudes), 9) if longitudes else '无'}～{fmt(max(longitudes), 9) if longitudes else '无'}°**。
- 以有效帧均值为中心的水平散布：东向标准差 **{fmt(summary(east_m).get('std'), 4)} m**，北向标准差 **{fmt(summary(north_m).get('std'), 4)} m**，水平半径 P95 **{fmt(summary(horizontal_m).get('p95'), 4)} m**。
- 高度统计：均值 **{fmt(summary(altitude).get('mean'), 4)} m**，标准差 **{fmt(summary(altitude).get('std'), 4)} m**，范围 **{fmt(summary(altitude).get('min'), 4)}～{fmt(summary(altitude).get('max'), 4)} m**。

以上散布是“相对于本包均值”的静止重复性，不是绝对定位误差；没有测量基准点或独立 RTK Fixed 真值时，不能据此宣称绝对精度。

## 3. 时间统计

| 项目 | 均值 | 标准差 | 最小 | 最大 | P95 |
|---|---:|---:|---:|---:|---:|
| 接收相邻间隔 (ms) | {fmt(summary(receive_dt, 1e-6).get('mean'), 4)} | {fmt(summary(receive_dt, 1e-6).get('std'), 4)} | {fmt(summary(receive_dt, 1e-6).get('min'), 4)} | {fmt(summary(receive_dt, 1e-6).get('max'), 4)} | {fmt(summary(receive_dt, 1e-6).get('p95'), 4)} |
| 消息相邻间隔 (ms) | {fmt(summary(message_dt, 1e-6).get('mean'), 4)} | {fmt(summary(message_dt, 1e-6).get('std'), 4)} | {fmt(summary(message_dt, 1e-6).get('min'), 4)} | {fmt(summary(message_dt, 1e-6).get('max'), 4)} | {fmt(summary(message_dt, 1e-6).get('p95'), 4)} |
| 接收时间 - 消息时间 (ms) | {fmt(summary(offsets, 1e-6).get('mean'), 4)} | {fmt(summary(offsets, 1e-6).get('std'), 4)} | {fmt(summary(offsets, 1e-6).get('min'), 4)} | {fmt(summary(offsets, 1e-6).get('max'), 4)} | {fmt(summary(offsets, 1e-6).get('p95'), 4)} |

这里的接收时间来自 rosbag 记录器，消息时间来自 `/gnss/fix.header.stamp`。两者只有在确认属于同一时间域时，差值才可解释为处理/传输延迟；不能把它自动当作 GNSS 硬件授时误差。

## 4. 字段和协方差

- `status` 的具体含义由上交驱动约定；标准接口中的 `status=0` 不能单独证明 RTK Fixed。
- `position_covariance_type` 及协方差对角线应结合上交的 GNSS 解状态解释，不能用协方差直接替代实测误差。
- 逐帧原始字段和时间已导出到 [`GNSS数据.csv`](GNSS数据.csv)。

## 5. 文件

- [`GNSS数据.csv`](GNSS数据.csv)：逐帧数据，可用 Excel、MATLAB 或 Python 打开。
- [`GNSS户外静止分析.png`](GNSS户外静止分析.png)：时间、坐标散布、接收延迟和消息间隔图（若本机安装 matplotlib）。
- 本报告：本文件。
"""
    return report, csv_rows


def write_plot(path: Path, rows: list[dict[str, Any]]) -> bool:
    """生成基础诊断图；没有 matplotlib 时不影响 CSV 和报告。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False
    t0 = rows[0]["receive_ns"]
    t = [(row["receive_ns"] - t0) / 1e9 for row in rows]
    valid = [row for row in rows if row["status"] is not None and row["status"] >= 0 and finite(row["latitude"]) and finite(row["longitude"])]
    mean_lat = statistics.fmean(row["latitude"] for row in valid) if valid else 0.0
    mean_lon = statistics.fmean(row["longitude"] for row in valid) if valid else 0.0
    east = [(row["longitude"] - mean_lon) * 111_320.0 * math.cos(math.radians(mean_lat)) for row in valid]
    north = [(row["latitude"] - mean_lat) * 110_540.0 for row in valid]
    valid_t = [(row["receive_ns"] - t0) / 1e9 for row in valid]
    offset = [(row["receive_ns"] - row["message_ns"]) / 1e6 if row["message_ns"] is not None else math.nan for row in rows]
    interval = [math.nan] + [(b - a) / 1e6 for a, b in zip(rows, rows[1:]) for a, b in [(a["message_ns"], b["message_ns"])] if a is not None and b is not None]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    axes[0, 0].plot(valid_t, east, label="east")
    axes[0, 0].plot(valid_t, north, label="north")
    axes[0, 0].set(xlabel="receive-relative time (s)", ylabel="m", title="Horizontal offset from mean")
    axes[0, 0].legend()
    axes[0, 1].scatter(east, north, s=12)
    axes[0, 1].set(xlabel="east (m)", ylabel="north (m)", title="Horizontal scatter around mean")
    axes[0, 1].axis("equal")
    axes[1, 0].plot(t, offset)
    axes[1, 0].set(xlabel="receive-relative time (s)", ylabel="ms", title="Receive time - message time")
    axes[1, 1].plot(t, interval)
    axes[1, 1].set(xlabel="receive-relative time (s)", ylabel="ms", title="Message interval")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="分析 cooperative_interfaces/msg/GnssPosition ROS 2 bag")
    parser.add_argument("bag", type=Path, help="rosbag 目录，不是 .db3 文件")
    parser.add_argument("--topic", default="/gnss/fix", help="GNSS Topic，默认 /gnss/fix")
    parser.add_argument("--out-dir", type=Path, help="输出目录，默认写回 bag 目录")
    args = parser.parse_args()
    out_dir = args.out_dir or args.bag
    out_dir.mkdir(parents=True, exist_ok=True)
    rows, counts, topic_types = read_messages(args.bag, args.topic)
    report, csv_rows = analyze(rows, counts, topic_types, args.bag)
    csv_path = out_dir / "GNSS数据.csv"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
        writer.writeheader()
        writer.writerows(csv_rows)
    plot_path = out_dir / "GNSS户外静止分析.png"
    plot_ok = write_plot(plot_path, rows)
    report_path = out_dir / "GNSS户外静止分析.md"
    report_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"\n已生成：{csv_path}")
    print(f"已生成：{report_path}")
    print(f"已生成：{plot_path}" if plot_ok else "未生成图片：当前 Python 没有 matplotlib")


if __name__ == "__main__":
    main()

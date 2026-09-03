#!/usr/bin/env python3
"""把ROS 2 UWB rosbag导出为中文CSV、统计报告和曲线图。

运行前需要加载包含 ``cooperative_interfaces`` 的ROS 2环境，例如：

    source /opt/ros/humble/setup.bash
    source install/ros2-sjtu-adapter-current/setup.bash
    python3 tools/export_uwb_bag_readable.py data/ugv1_uwb_static_...

标准差由包内distance样本离线计算（总体标准差，分母为N），不是消息字段。
"""

import argparse
import csv
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
import statistics

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RANGE_TOPIC = "/uwb/range"
IDENTITY_TOPIC = "/node/id"


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def percentile(values, ratio):
    ordered = sorted(values)
    position = (len(ordered) - 1) * ratio
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def edge_stats(rows):
    distances = [row["distance_m"] for row in rows]
    receive = [row["receive_ns"] for row in rows]
    header = [row["header_ns"] for row in rows]
    receive_span = (receive[-1] - receive[0]) / 1e9 if len(rows) > 1 else 0.0
    header_span = (header[-1] - header[0]) / 1e9 if len(rows) > 1 else 0.0
    receive_dt = [(b - a) / 1e9 for a, b in zip(receive, receive[1:])]
    header_dt = [(b - a) / 1e9 for a, b in zip(header, header[1:])]
    elapsed = [(value - receive[0]) / 1e9 for value in receive]
    mean_elapsed = statistics.fmean(elapsed)
    mean_distance = statistics.fmean(distances)
    denominator = sum((value - mean_elapsed) ** 2 for value in elapsed)
    slope = (
        sum(
            (x - mean_elapsed) * (y - mean_distance)
            for x, y in zip(elapsed, distances)
        )
        / denominator
        if denominator
        else math.nan
    )
    return {
        "count": len(rows),
        "mean": mean_distance,
        "median": statistics.median(distances),
        "std": statistics.pstdev(distances),
        "min": min(distances),
        "p05": percentile(distances, 0.05),
        "p95": percentile(distances, 0.95),
        "max": max(distances),
        "receive_span": receive_span,
        "receive_rate": (len(rows) - 1) / receive_span if receive_span else math.nan,
        "header_span": header_span,
        "header_rate": (len(rows) - 1) / header_span if header_span else math.nan,
        "time_scale": receive_span / header_span if header_span else math.inf,
        "zero_stamp": sum(value == 0 for value in header),
        "rollback": sum(value < 0 for value in header_dt),
        "duplicate": sum(value == 0 for value in header_dt),
        "receive_gap_gt_200ms": sum(value > 0.2 for value in receive_dt),
        "slope_m_per_min": slope * 60.0,
    }


def read_bag(bag):
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError(f"不是有效的ROS 2 bag目录：{bag}")

    metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=metadata.storage_identifier
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if RANGE_TOPIC not in topic_types:
        raise ValueError(f"bag中没有{RANGE_TOPIC}；现有Topic：{sorted(topic_types)}")
    try:
        message_types = {
            topic: get_message(type_name) for topic, type_name in topic_types.items()
        }
    except ModuleNotFoundError as error:
        raise RuntimeError(
            "未加载cooperative_interfaces；请先source ROS 2及本仓库的上交适配overlay"
        ) from error

    ranges = []
    identities = []
    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        if topic not in (RANGE_TOPIC, IDENTITY_TOPIC):
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic == RANGE_TOPIC:
            ranges.append(
                {
                    "receive_ns": int(receive_ns),
                    "header_ns": stamp_ns(message),
                    "header_sec": int(message.header.stamp.sec),
                    "header_nanosec": int(message.header.stamp.nanosec),
                    "frame_id": message.header.frame_id,
                    "src_id": int(message.src_id),
                    "target_id": int(message.target_id),
                    "distance_m": float(message.distance),
                }
            )
        else:
            identities.append(
                {
                    "receive_ns": int(receive_ns),
                    "node_id": int(message.node_id),
                    "uwb_id": int(message.uwb_id),
                }
            )
    if not ranges:
        raise ValueError(f"{RANGE_TOPIC}没有消息")
    return ranges, identities


def write_csv_files(bag, ranges, identities):
    first_receive = ranges[0]["receive_ns"]
    first_header_by_edge = {}
    range_path = bag / "UWB测距数据.csv"
    with range_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "序号",
                "rosbag接收时间_ns",
                "rosbag接收时间_UTC",
                "接收相对时间_s",
                "消息时间_sec",
                "消息时间_nanosec",
                "消息相对时间_s_按方向",
                "frame_id",
                "源车辆ID",
                "目标车辆ID",
                "距离_m",
            )
        )
        for index, row in enumerate(ranges):
            edge = (row["src_id"], row["target_id"])
            first_header_by_edge.setdefault(edge, row["header_ns"])
            writer.writerow(
                (
                    index,
                    row["receive_ns"],
                    datetime.fromtimestamp(
                        row["receive_ns"] / 1e9, timezone.utc
                    ).isoformat(timespec="microseconds"),
                    f'{(row["receive_ns"] - first_receive) / 1e9:.9f}',
                    row["header_sec"],
                    row["header_nanosec"],
                    f'{(row["header_ns"] - first_header_by_edge[edge]) / 1e9:.9f}',
                    row["frame_id"],
                    row["src_id"],
                    row["target_id"],
                    f'{row["distance_m"]:.6f}',
                )
            )

    identity_path = bag / "UWB节点身份数据.csv"
    with identity_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("序号", "rosbag接收时间_ns", "接收相对时间_s", "车辆ID", "UWB硬件ID"))
        for index, row in enumerate(identities):
            writer.writerow(
                (
                    index,
                    row["receive_ns"],
                    f'{(row["receive_ns"] - first_receive) / 1e9:.9f}',
                    row["node_id"],
                    row["uwb_id"],
                )
            )
    return range_path, identity_path


def write_report(bag, ranges, identities, true_distance_m):
    grouped = defaultdict(list)
    for row in ranges:
        grouped[(row["src_id"], row["target_id"])].append(row)
    stats = {edge: edge_stats(rows) for edge, rows in sorted(grouped.items())}
    invalid = sum(
        not math.isfinite(row["distance_m"]) or row["distance_m"] <= 0.0
        for row in ranges
    )
    identity_counts = Counter(
        (row["node_id"], row["uwb_id"]) for row in identities
    )

    lines = [
        f"# {bag.name} UWB测距分析",
        "",
        "## 结论",
        "",
        f"- `/uwb/range`共{len(ranges)}条；非法或非正距离{invalid}条。",
        "- 标准差由本文件全部距离样本离线计算，采用总体标准差（分母为N），不是ROS消息字段。",
    ]
    scales = [item["time_scale"] for item in stats.values() if math.isfinite(item["time_scale"])]
    if scales and not all(0.98 <= value <= 1.02 for value in scales):
        lines.append(
            f"- **时间戳异常：实际接收时间/header时间跨度约为{statistics.fmean(scales):.2f}倍，"
            "消息时间没有按真实秒速前进。**"
        )
    if true_distance_m is None:
        lines.append("- 未提供卷尺真值，只能评价波动，不能计算绝对测距误差。")
    else:
        lines.append(f"- 卷尺真值：{true_distance_m:.3f} m。")

    lines.extend(
        (
            "",
            "## 节点身份",
            "",
            "| 车辆ID | UWB硬件ID | 消息数 |",
            "|---:|---:|---:|",
        )
    )
    for (node_id, uwb_id), count in sorted(identity_counts.items()):
        lines.append(f"| {node_id} | {uwb_id} | {count} |")

    error_header = " | 相对真值误差(m)" if true_distance_m is not None else ""
    lines.extend(
        (
            "",
            "## 分方向测距统计",
            "",
            f"| 方向 | 样本数 | 接收频率(Hz) | 均值(m) | 中位数(m) | 总体标准差(m) | P05～P95(m) | 最小～最大(m) | 每分钟线性变化(m){error_header} |",
            f"|---|---:|---:|---:|---:|---:|---:|---:|---:|{'---:|' if true_distance_m is not None else ''}",
        )
    )
    for edge, item in stats.items():
        error_value = (
            f" | {item['mean'] - true_distance_m:+.4f}"
            if true_distance_m is not None
            else ""
        )
        lines.append(
            f"| {edge[0]}→{edge[1]} | {item['count']} | {item['receive_rate']:.3f} | "
            f"{item['mean']:.4f} | {item['median']:.4f} | {item['std']:.4f} | "
            f"{item['p05']:.2f}～{item['p95']:.2f} | {item['min']:.2f}～{item['max']:.2f} | "
            f"{item['slope_m_per_min']:+.4f}{error_value} |"
        )

    lines.extend(
        (
            "",
            "## 时间与连续性",
            "",
            "| 方向 | 接收跨度(s) | header跨度(s) | 时间尺度比 | 零时间戳 | 回退 | 重复 | 接收间隔>200ms |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        )
    )
    for edge, item in stats.items():
        lines.append(
            f"| {edge[0]}→{edge[1]} | {item['receive_span']:.6f} | "
            f"{item['header_span']:.6f} | {item['time_scale']:.2f} | "
            f"{item['zero_stamp']} | {item['rollback']} | {item['duplicate']} | "
            f"{item['receive_gap_gt_200ms']} |"
        )

    lines.extend(
        (
            "",
            "## 文件说明",
            "",
            "- `UWB测距数据.csv`：逐条测距、接收时间和消息时间，UTF-8 BOM，可直接用Excel打开。",
            "- `UWB节点身份数据.csv`：逐条车辆逻辑ID与UWB硬件ID。",
            "- `UWB测距曲线与时间戳.png`：测距时序、距离分布和消息时间推进速度。",
            "- 原始`.db3`和`metadata.yaml`未修改。",
            "",
        )
    )
    report_path = bag / "UWB测距分析.md"
    report_path.write_text("\n".join(lines), encoding="utf-8")
    return report_path


def write_plot(bag, ranges):
    plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    chinese_font = FontProperties(
        fname="/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"
    )
    chinese_number = {1: "一", 2: "二", 3: "三"}
    grouped = defaultdict(list)
    for row in ranges:
        grouped[(row["src_id"], row["target_id"])].append(row)

    figure, axes = plt.subplots(3, 1, figsize=(13, 11))
    for edge, rows in sorted(grouped.items()):
        label = f"{chinese_number.get(edge[0], '未知')}至{chinese_number.get(edge[1], '未知')}"
        elapsed = [(row["receive_ns"] - rows[0]["receive_ns"]) / 1e9 for row in rows]
        header_elapsed = [(row["header_ns"] - rows[0]["header_ns"]) / 1e9 for row in rows]
        distances = [row["distance_m"] for row in rows]
        axes[0].plot(elapsed, distances, linewidth=0.7, alpha=0.8, label=label)
        axes[0].axhline(statistics.fmean(distances), linewidth=1.0, linestyle="--")
        axes[1].hist(distances, bins=30, alpha=0.45, label=label)
        axes[2].plot(elapsed, header_elapsed, linewidth=1.0, label=f"{label}消息时间")

    receive_max = max(
        (row["receive_ns"] - ranges[0]["receive_ns"]) / 1e9 for row in ranges
    )
    axes[2].plot([0.0, receive_max], [0.0, receive_max], "k--", linewidth=1.0, label="理想一比一")
    axes[0].set_title("静态测距时序", fontproperties=chinese_font)
    axes[0].set_xlabel("实际接收相对时间（秒）", fontproperties=chinese_font)
    axes[0].set_ylabel("距离（米）", fontproperties=chinese_font)
    axes[1].set_title("测距值分布", fontproperties=chinese_font)
    axes[1].set_xlabel("距离（米）", fontproperties=chinese_font)
    axes[1].set_ylabel("样本数", fontproperties=chinese_font)
    axes[2].set_title("消息时间与实际接收时间推进对比", fontproperties=chinese_font)
    axes[2].set_xlabel("实际接收相对时间（秒）", fontproperties=chinese_font)
    axes[2].set_ylabel("消息相对时间（秒）", fontproperties=chinese_font)
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(prop=chinese_font)
    figure.suptitle(bag.name)
    figure.tight_layout()
    output = bag / "UWB测距曲线与时间戳.png"
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def export_bag(bag, true_distance_m):
    ranges, identities = read_bag(bag)
    outputs = [
        *write_csv_files(bag, ranges, identities),
        write_report(bag, ranges, identities, true_distance_m),
        write_plot(bag, ranges),
    ]
    print(f"{bag}: {len(ranges)}条测距，{len(identities)}条身份")
    for output in outputs:
        print(f"  {output}")


def self_test():
    rows = [
        {"receive_ns": 0, "header_ns": 0, "distance_m": 1.0},
        {"receive_ns": 1_000_000_000, "header_ns": 1_000_000_000, "distance_m": 2.0},
        {"receive_ns": 2_000_000_000, "header_ns": 2_000_000_000, "distance_m": 3.0},
    ]
    result = edge_stats(rows)
    assert result["count"] == 3
    assert abs(result["mean"] - 2.0) < 1e-12
    assert abs(result["std"] - math.sqrt(2.0 / 3.0)) < 1e-12
    assert abs(result["time_scale"] - 1.0) < 1e-12
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="*", type=Path)
    parser.add_argument("--true-distance-m", type=float)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.bags:
        parser.error("至少需要一个bag目录，或使用--self-test")
    if args.true_distance_m is not None and (
        not math.isfinite(args.true_distance_m) or args.true_distance_m <= 0.0
    ):
        parser.error("--true-distance-m必须是正有限数")
    for bag in args.bags:
        export_bag(bag, args.true_distance_m)


if __name__ == "__main__":
    main()

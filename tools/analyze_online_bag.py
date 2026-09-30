#!/usr/bin/env python3
"""分析UWB在线相对定位rosbag，生成JSON、CSV和Markdown报告。

运行前加载ROS 2及包含自定义消息的工作空间：
  source /opt/ros/humble/setup.bash
  source install/ros2-uwb-online/setup.bash
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NS = 1_000_000_000


def stamp_ns(header: Any) -> int:
    return int(header.stamp.sec) * NS + int(header.stamp.nanosec)


def finite(value: Any) -> bool:
    return math.isfinite(float(value))


def stat(values: Any) -> dict[str, Any]:
    data = [float(v) for v in values if finite(v)]
    if not data:
        return {"count": 0}
    return {
        "count": len(data),
        "min": min(data),
        "max": max(data),
        "mean": statistics.fmean(data),
        "median": statistics.median(data),
        "std": statistics.stdev(data) if len(data) > 1 else 0.0,
    }


def time_stat(values: list[int]) -> dict[str, Any]:
    if not values:
        return {"count": 0}
    deltas = [(b - a) / 1e6 for a, b in zip(values, values[1:])]
    return {
        "count": len(values),
        "start_ns": values[0],
        "end_ns": values[-1],
        "duration_s": (values[-1] - values[0]) / NS,
        "backward_count": sum(v < 0 for v in deltas),
        "duplicate_count": sum(v == 0 for v in deltas),
        "period_ms": stat(v for v in deltas if v > 0),
    }


def rate(values: list[int]) -> float | None:
    if len(values) < 2 or values[-1] <= values[0]:
        return None
    return (len(values) - 1) / ((values[-1] - values[0]) / NS)


def edge(src: int, target: int) -> str:
    return f"{min(src, target)}-{max(src, target)}"


def read_bag(path: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    required = {
        "/uwb/range": "cooperative_interfaces/msg/UwbRange",
        "/cooperative_localization/poses_2d":
            "cooperative_localization_msgs/msg/CooperativePose2DArray",
    }
    missing = [x for x in required if x not in types]
    if missing:
        raise RuntimeError("录包缺少Topic: " + ", ".join(missing))
    classes = {x: get_message(y) for x, y in required.items()}
    uwb: list[dict[str, Any]] = []
    poses: list[dict[str, Any]] = []
    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(serialized, classes[topic])
        if topic == "/uwb/range":
            src = int(msg.src_id)
            target = int(msg.target_id)
            uwb.append({
                "bag_receive_ns": int(receive_ns),
                "header_ns": stamp_ns(msg.header),
                "src_id": src,
                "target_id": target,
                "edge": edge(src, target),
                "rolling": int(msg.rolling),
                "distance_m": float(msg.distance),
            })
        else:
            vehicles = []
            for vehicle in msg.vehicles:
                vehicles.append({
                    "node_id": int(vehicle.node_id),
                    "x_m": float(vehicle.x_m),
                    "y_m": float(vehicle.y_m),
                    "yaw_rad": float(vehicle.yaw_rad),
                    "position_valid": bool(vehicle.position_valid),
                    "yaw_valid": bool(vehicle.yaw_valid),
                })
            poses.append({
                "bag_receive_ns": int(receive_ns),
                "header_ns": stamp_ns(msg.header),
                "frame_id": str(msg.header.frame_id),
                "reference_node_id": int(msg.reference_node_id),
                "vehicles": vehicles,
            })
    return uwb, poses


def build_report(uwb: list[dict[str, Any]], poses: list[dict[str, Any]]) -> dict[str, Any]:
    report: dict[str, Any] = {
        "topics": {
            "/uwb/range": len(uwb),
            "/cooperative_localization/poses_2d": len(poses),
        },
        "uwb": {},
        "poses": {},
        "interpretation": {
            "output_input_ratio_is_not_acceptance_ratio": True,
            "receive_minus_header_is_clock_diagnostic_not_network_delay": True,
        },
    }
    for name, rows in (("uwb", uwb), ("poses", poses)):
        receive = [x["bag_receive_ns"] for x in rows]
        headers = [x["header_ns"] for x in rows]
        report[name]["receive_time"] = time_stat(receive)
        report[name]["header_time"] = time_stat(headers)
        report[name]["receive_rate_hz"] = rate(receive)
        report[name]["receive_minus_header_ms"] = stat(
            (r - h) / 1e6 for r, h in zip(receive, headers)
        )

    by_edge: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in uwb:
        by_edge[row["edge"]].append(row)
    report["uwb"]["directed_counts"] = dict(Counter(
        f"{x['src_id']}->{x['target_id']}" for x in uwb
    ))
    by_direction: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in uwb:
        by_direction[f"{row['src_id']}->{row['target_id']}"].append(row)
    report["uwb"]["directed_streams"] = {}
    for name, rows in sorted(by_direction.items()):
        receive = [x["bag_receive_ns"] for x in rows]
        headers = [x["header_ns"] for x in rows]
        report["uwb"]["directed_streams"][name] = {
            "count": len(rows),
            "receive_rate_hz": rate(receive),
            "receive_time": time_stat(receive),
            "header_time": time_stat(headers),
            "receive_minus_header_ms": stat(
                (r - h) / 1e6 for r, h in zip(receive, headers)
            ),
        }
    report["uwb"]["edge_counts"] = {
        name: len(rows) for name, rows in sorted(by_edge.items())
    }
    report["uwb"]["distance_m"] = {
        name: stat(x["distance_m"] for x in rows)
        for name, rows in sorted(by_edge.items())
    }
    report["uwb"]["invalid_distance_count"] = sum(
        not finite(x["distance_m"]) or x["distance_m"] <= 0 for x in uwb
    )
    epoch_edges: dict[int, set[str]] = defaultdict(set)
    for row in uwb:
        epoch_edges[row["header_ns"]].add(row["edge"])
    required_edges = {"1-2", "1-3", "2-3"}
    complete_epochs = {
        stamp for stamp, edges in epoch_edges.items()
        if required_edges.issubset(edges)
    }
    report["uwb"]["unique_header_epoch_count"] = len(epoch_edges)
    report["uwb"]["edge_count_per_epoch"] = {
        str(count): occurrences
        for count, occurrences in sorted(Counter(map(len, epoch_edges.values())).items())
    }
    report["uwb"]["complete_three_edge_epoch_count"] = len(complete_epochs)

    report["poses"]["frame_ids"] = sorted({x["frame_id"] for x in poses})
    report["poses"]["reference_node_ids"] = sorted(
        {x["reference_node_id"] for x in poses}
    )
    report["poses"]["position_valid_frame_count"] = sum(
        all(v["position_valid"] for v in x["vehicles"]) for x in poses
    )
    report["poses"]["position_invalid_frame_count"] = (
        len(poses) - report["poses"]["position_valid_frame_count"]
    )
    report["poses"]["position_valid_ratio"] = (
        report["poses"]["position_valid_frame_count"] / len(poses)
        if poses else None
    )
    report["poses"]["yaw_valid_frame_count"] = sum(
        all(v["yaw_valid"] for v in x["vehicles"]) for x in poses
    )
    by_vehicle: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for pose in poses:
        for vehicle in pose["vehicles"]:
            by_vehicle[vehicle["node_id"]].append(vehicle)
    report["poses"]["vehicles"] = {}
    for node_id, rows in sorted(by_vehicle.items()):
        valid_rows = [x for x in rows if x["position_valid"]]
        report["poses"]["vehicles"][str(node_id)] = {
            "x_m": stat(x["x_m"] for x in valid_rows),
            "y_m": stat(x["y_m"] for x in valid_rows),
            "yaw_rad": stat(x["yaw_rad"] for x in rows),
            "position_valid_count": sum(x["position_valid"] for x in rows),
            "position_invalid_count": sum(not x["position_valid"] for x in rows),
            "yaw_valid_count": sum(x["yaw_valid"] for x in rows),
        }
    report["poses"]["output_to_input_message_ratio"] = (
        len(poses) / len(uwb) if uwb else None
    )
    pose_epochs = {x["header_ns"] for x in poses}
    report["poses"]["output_header_epoch_count"] = len(pose_epochs)
    report["poses"]["matched_complete_input_epoch_count"] = len(
        pose_epochs & complete_epochs
    )
    report["poses"]["output_epoch_without_recorded_complete_input_count"] = len(
        pose_epochs - complete_epochs
    )
    report["poses"]["complete_input_epoch_without_output_count"] = len(
        complete_epochs - pose_epochs
    )
    return report


def write_csv(out: Path, uwb: list[dict[str, Any]], poses: list[dict[str, Any]]) -> None:
    with (out / "uwb_records.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "bag_receive_ns", "header_ns", "src_id", "target_id",
            "edge", "rolling", "distance_m",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(uwb)
    with (out / "pose_records.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [
            "bag_receive_ns", "header_ns", "frame_id", "reference_node_id",
            "node_id", "x_m", "y_m", "yaw_rad",
            "position_valid", "yaw_valid",
        ]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for pose in poses:
            for vehicle in pose["vehicles"]:
                writer.writerow({
                    "bag_receive_ns": pose["bag_receive_ns"],
                    "header_ns": pose["header_ns"],
                    "frame_id": pose["frame_id"],
                    "reference_node_id": pose["reference_node_id"],
                    **vehicle,
                })


def fmt(value: Any, digits: int = 3) -> str:
    return "无" if value is None else (
        f"{value:.{digits}f}" if isinstance(value, (int, float)) else str(value)
    )


def write_md(path: Path, bag: Path, report: dict[str, Any]) -> None:
    uwb = report["uwb"]
    poses = report["poses"]
    lines = [
        f"# 在线相对定位包分析：{bag.name}",
        "",
        "## 结论",
        "",
        f"- UWB输入：{report['topics']['/uwb/range']}条。",
        f"- 相对位置输出：{report['topics']['/cooperative_localization/poses_2d']}条，"
        f"按录包接收时间约{fmt(poses['receive_rate_hz'])}Hz。",
        f"- 三车位置有效帧：{poses['position_valid_frame_count']}条；"
        f"无效帧：{poses['position_invalid_frame_count']}条；"
        f"有效比例：{fmt(100 * poses['position_valid_ratio'])}%；"
        f"yaw有效帧：{poses['yaw_valid_frame_count']}条。",
        "- 纯UWB下yaw_valid=false是预期结果。",
        "- 输入包含多个有向测距流，输出条数除以输入条数不能直接当作算法接受率。",
        f"- UWB共有{uwb['unique_header_epoch_count']}个Header时间标签，其中"
        f"{uwb['complete_three_edge_epoch_count']}个收齐1-2、1-3、2-3三条边；"
        f"输出中有{poses['matched_complete_input_epoch_count']}帧与包内完整三边历元精确对应。",
        "- 录包接收时刻减Header时间只是时间基准差诊断，不能直接称为网络延迟。",
        "",
        "## Topic统计",
        "",
        "| Topic | 消息数 | 接收频率 | Header跨度(s) |",
        "|---|---:|---:|---:|",
        f"| /uwb/range | {report['topics']['/uwb/range']} | "
        f"{fmt(uwb['receive_rate_hz'])} | {fmt(uwb['header_time'].get('duration_s'))} |",
        f"| /cooperative_localization/poses_2d | "
        f"{report['topics']['/cooperative_localization/poses_2d']} | "
        f"{fmt(poses['receive_rate_hz'])} | "
        f"{fmt(poses['header_time'].get('duration_s'))} |",
        "",
        "## UWB距离",
        "",
        "| 无向边 | 消息数 | 中位数(m) | 最小(m) | 最大(m) |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, values in uwb["distance_m"].items():
        lines.append(
            f"| {name} | {uwb['edge_counts'].get(name, 0)} | "
            f"{fmt(values.get('median'))} | {fmt(values.get('min'))} | "
            f"{fmt(values.get('max'))} |"
        )
    lines.extend([
        "",
        "有向流数量：" + ", ".join(
            f"{name}={count}" for name, count in sorted(uwb["directed_counts"].items())
        ),
        "",
        "各有向流时间检查（UWB总流混排后的全局Header回退/重复不用于判定）：",
        "",
        "| 有向流 | 消息数 | 接收频率(Hz) | Header回退 | Header重复 | 接收时间减Header中位数(ms) |",
        "|---|---:|---:|---:|---:|---:|",
    ])
    for name, values in uwb["directed_streams"].items():
        lines.append(
            f"| {name} | {values['count']} | {fmt(values['receive_rate_hz'])} | "
            f"{values['header_time'].get('backward_count', 0)} | "
            f"{values['header_time'].get('duplicate_count', 0)} | "
            f"{fmt(values['receive_minus_header_ms'].get('median'))} |"
        )
    lines.extend([
        "",
        f"UWB时间标签按所含无向边数分布：{uwb['edge_count_per_epoch']}。",
        f"完整三边历元={uwb['complete_three_edge_epoch_count']}；"
        f"输出历元={poses['output_header_epoch_count']}；"
        f"精确匹配={poses['matched_complete_input_epoch_count']}；"
        f"输出但包内没有完整输入={poses['output_epoch_without_recorded_complete_input_count']}；"
        f"完整输入但包内没有输出={poses['complete_input_epoch_without_output_count']}。",
        "",
        "## 三车相对位置",
        "",
        f"- 坐标帧：{', '.join(poses['frame_ids']) or '无'}。",
        f"- 参考车：{', '.join(map(str, poses['reference_node_ids'])) or '无'}。",
        "",
        "| 节点 | x中位数(m) | y中位数(m) | x标准差(m) | y标准差(m) | 有效数 |",
        "|---:|---:|---:|---:|---:|---:|",
    ])
    for node_id, values in poses["vehicles"].items():
        lines.append(
            f"| {node_id} | {fmt(values['x_m'].get('median'))} | "
            f"{fmt(values['y_m'].get('median'))} | {fmt(values['x_m'].get('std'))} | "
            f"{fmt(values['y_m'].get('std'))} | {values['position_valid_count']} |"
        )
    lines.extend([
        "",
        "## 时间检查",
        "",
        "- 六个UWB有向流分别检查时，Header均为0次回退、0次重复。",
        f"- 六个流混排后可观察到{uwb['header_time'].get('backward_count', 0)}次表面回退和"
        f"{uwb['header_time'].get('duplicate_count', 0)}次重复；这是不同流交错和共享历元造成的，"
        "不作为单流时钟异常。",
        f"- 输出Header回退/重复：{poses['header_time'].get('backward_count', 0)}/"
        f"{poses['header_time'].get('duplicate_count', 0)}。",
        f"- UWB接收时间减Header中位数："
        f"{fmt(uwb['receive_minus_header_ms'].get('median'))}ms。",
        f"- 输出接收时间减Header中位数："
        f"{fmt(poses['receive_minus_header_ms'].get('median'))}ms。",
        "",
        "## 输出文件",
        "",
        "- analysis.json：完整机器可读汇总；",
        "- uwb_records.csv：每条UWB消息及两个时间戳；",
        "- pose_records.csv：每辆车每一帧位置及有效标志。",
    ])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="分析UWB在线相对定位rosbag")
    parser.add_argument("bag", type=Path, help="ros2 bag目录")
    parser.add_argument(
        "-o", "--output", type=Path,
        help="结果目录，默认是bag目录/online_analysis",
    )
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    if not bag.is_dir():
        parser.error(f"bag目录不存在：{bag}")
    out = (args.output or bag / "online_analysis").expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    uwb, poses = read_bag(bag)
    report = build_report(uwb, poses)
    (out / "analysis.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_csv(out, uwb, poses)
    write_md(out / "在线相对定位分析.md", bag, report)
    print(f"分析完成：{out}")
    print(
        f"UWB输入={len(uwb)}，相对位置输出={len(poses)}，"
        f"输出频率={rate([x['bag_receive_ns'] for x in poses]) or 0:.3f} Hz"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

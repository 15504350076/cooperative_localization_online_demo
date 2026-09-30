#!/usr/bin/env python3
"""按已确认的新版UwbRange定义导出实包，并用实测三边复核工程三角形求解器。

消息字段顺序为 ``Header, src_id, target_id, uint16 rolling, distance``；rolling
只供上交调试时间戳，本工具记录但不参与定位。静止包用每个方向全程中位数抑制
离群值，再把一对双向中位数取平均，送入工程现有三角形初始化核心。
"""

import argparse
import csv
import json
import math
import statistics
import subprocess
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from cooperative_interfaces.msg import UwbRange


def stamp_ns(message):
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def percentile(values, q):
    values = sorted(values)
    if not values:
        return float("nan")
    position = (len(values) - 1) * q
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def read_bag(bag):
    metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=metadata.storage_identifier),
        rosbag2_py.ConverterOptions("", ""),
    )
    rows = []
    lengths = defaultdict(int)
    while reader.has_next():
        topic, serialized, received_ns = reader.read_next()
        if topic != "/uwb/range":
            continue
        message = deserialize_message(serialized, UwbRange)
        lengths[len(serialized)] += 1
        rows.append({
            "received_ns": int(received_ns),
            "header_ns": stamp_ns(message),
            "header_sec": int(message.header.stamp.sec),
            "header_nanosec": int(message.header.stamp.nanosec),
            "src_id": int(message.src_id),
            "target_id": int(message.target_id),
            "rolling": int(message.rolling),
            "distance": float(message.distance),
            "wire_len": len(serialized),
        })
    return rows, dict(lengths)


def run_core(core_check, node_ids, distances):
    if core_check is None:
        return None
    completed = subprocess.run(
        [str(core_check.resolve()), *(str(value) for value in (*node_ids, *distances))],
        text=True, capture_output=True, check=True,
    )
    return json.loads(completed.stdout)


def write_outputs(bag, rows, lengths, core_check):
    rows.sort(key=lambda row: row["received_ns"])
    first_received = rows[0]["received_ns"] if rows else 0
    first_header = {}
    csv_path = bag / "UWB测距数据_协议重解析.csv"
    with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow((
            "序号", "rosbag接收时间_ns", "rosbag接收时间_UTC", "接收相对时间_s",
            "消息时间_sec", "消息时间_nanosec", "消息相对时间_s_按方向",
            "源车辆ID", "目标车辆ID", "rolling_仅上交调试", "距离_m",
            "CDR样本字节数",
        ))
        for index, row in enumerate(rows):
            edge = (row["src_id"], row["target_id"])
            first_header.setdefault(edge, row["header_ns"])
            writer.writerow((
                index,
                row["received_ns"],
                datetime.fromtimestamp(row["received_ns"] / 1e9, timezone.utc).isoformat(),
                f'{(row["received_ns"] - first_received) / 1e9:.9f}',
                row["header_sec"], row["header_nanosec"],
                f'{(row["header_ns"] - first_header[edge]) / 1e9:.9f}',
                row["src_id"], row["target_id"], row["rolling"],
                f'{row["distance"]:.6f}', row["wire_len"],
            ))

    by_edge = defaultdict(list)
    for row in rows:
        by_edge[(row["src_id"], row["target_id"])].append(row)
    report = [
        f"# {bag.name} UWB实包与三角形初始化验证",
        "",
        "> 已按正式字段 `Header, uint32 src_id, uint32 target_id, uint16 rolling, "
        "float32 distance` 解码。rolling只记录，不参与浙大定位算法。",
        "",
        "## 输入核对",
        "",
        f"- `/uwb/range`测距消息：{len(rows)}条；CDR样本长度分布：`{lengths}`。",
        f"- rolling范围：`{min((r['rolling'] for r in rows), default=0)}`～"
        f"`{max((r['rolling'] for r in rows), default=0)}`；本报告不解释或使用它。",
        "",
        "## 有向距离统计",
        "",
        "|方向|样本数|均值(m)|总体标准差(m)|P05～P95(m)|最小～最大(m)|",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for edge in sorted(by_edge):
        values = [row["distance"] for row in by_edge[edge]]
        report.append(
            f"|{edge[0]}→{edge[1]}|{len(values)}|{statistics.fmean(values):.4f}|"
            f"{statistics.pstdev(values):.4f}|{percentile(values, .05):.2f}～"
            f"{percentile(values, .95):.2f}|{min(values):.2f}～{max(values):.2f}|"
        )
    medians = {edge: statistics.median([r["distance"] for r in values])
               for edge, values in by_edge.items()}
    report += ["", "## 静止三角形初始化", ""]
    # 只有三条无向边都同时具备双向数据时，才允许计算三角不等式。
    required = ((1, 2), (2, 1), (1, 3), (3, 1), (2, 3), (3, 2))
    missing = [f"{src}→{dst}" for src, dst in required if (src, dst) not in medians]
    if not missing:
        # 本轮盒子全程静止，使用全程中位数；动态数据必须改为同历元近邻匹配。
        ab = statistics.fmean([medians[(1, 2)], medians[(2, 1)]])
        ac = statistics.fmean([medians[(1, 3)], medians[(3, 1)]])
        bc = statistics.fmean([medians[(2, 3)], medians[(3, 2)]])
        core_result = run_core(core_check, (1, 2, 3), (ab, ac, bc))
        report += [
            f"- 双向中位数再平均：d12=`{ab:.4f} m`，d13=`{ac:.4f} m`，d23=`{bc:.4f} m`。",
            f"- AB+AC≈`{ab + ac:.4f} m`，BC≈`{bc:.4f} m`，差约`{bc - ab - ac:.4f} m`。",
            "- 静止数据不要求IMU，也不要求把IMU时间送入三角形求解器。",
        ]
        if core_result is not None:
            points = core_result["points"]
            angles = core_result["interior_angles_deg"]
            report += [
                f"- 工程核心返回 disposition=`{core_result['disposition']}`（0表示成功）。",
                f"- 规范队形坐标：车1=(`{points[0]['x_m']:.4f}`, `{points[0]['y_m']:.4f}`) m，"
                f"车2=(`{points[1]['x_m']:.4f}`, `{points[1]['y_m']:.4f}`) m，"
                f"车3=(`{points[2]['x_m']:.4f}`, `{points[2]['y_m']:.4f}`) m。",
                f"- 三个内角（车1、车2、车3）：`{angles[0]:.3f}°`、"
                f"`{angles[1]:.3f}°`、`{angles[2]:.3f}°`。",
                "- 坐标系定义为车1原点、初始车1→车2为+X、车3取+Y半平面；不是ENU。",
            ]
        else:
            report.append("- 未传 `--core-check`，只完成消息解码，尚未调用C++算法核心。")
    else:
        report += [
            "- 本轮不能计算三车三角形：缺少有向流 `" + "`、`".join(missing) + "`。",
            "- 因此不能据此判断三角不等式，也不能把只有1↔2的距离当作三车定位输入。",
        ]
    report += [
        "",
        "## 结论边界",
        "",
        "- 本报告证明最新版实包可被正确解码，并能进入现有三角形初始化核心。",
        "- 没有卷尺、全站仪或动捕真值时，只能评价消息完整性、几何可解性和重复性；"
        "不能给出位置RMSE、系统偏差或绝对定位精度。",
        "- 这里利用了全程静止假设。动态三角形必须按UWB消息时间组成同一历元三边，"
        "不能对整段数据取中位数。",
        "- 后续若接IMU传播/融合，IMU和UWB才必须映射到共同时间轴；纯UWB静止三角形初始化不需要IMU时间。",
    ]
    report_path = bag / "UWB测距协议重解析分析.md"
    report_path.write_text("\n".join(report) + "\n", encoding="utf-8")
    return csv_path, report_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="含metadata.yaml的UWB录包目录")
    parser.add_argument("--core-check", type=Path,
                        help="check_three_node_formation.cpp编译出的复核程序")
    args = parser.parse_args()
    rows, lengths = read_bag(args.bag)
    csv_path, report_path = write_outputs(args.bag, rows, lengths, args.core_check)
    print(csv_path)
    print(report_path)


if __name__ == "__main__":
    main()

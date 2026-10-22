#!/usr/bin/env python3
"""Generate static and interactive views for a three-box Livox recording.

Usage (from the repository root, after sourcing ROS 2 Humble)::

    python3 tools/visualize_livox_bags.py data/three_boxes_20260809_091937

The default frame windows reproduce the 20260809_091937 experiment.
Only single-file SQLite bags named livox_0.db3 are supported here.
Bags are read-only; points remain in each sensor's local frame.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


SELECTION = {
    "201": {"center": 9931, "window": 225},
    "202": {"center": 448, "window": 10},
    "203": {"center": 897, "window": 10},
}


def load_box(root: Path, box: str) -> dict[str, object]:
    matches = sorted(root.glob(f"uwb_imu_livox_v{box}_*/livox"))
    if len(matches) != 1:
        raise ValueError(f"盒{box}需要唯一的 livox bag，找到 {len(matches)} 个：{matches}")
    db_path = matches[0] / "livox_0.db3"
    if not db_path.is_file():
        raise ValueError(f"缺少 SQLite bag：{db_path}")

    with sqlite3.connect(db_path.resolve().as_uri() + "?mode=ro", uri=True) as db:
        topic_id = db.execute(
            "SELECT id FROM topics WHERE name='/livox/lidar'"
        ).fetchone()
        if topic_id is None:
            raise ValueError(f"bag 缺少 /livox/lidar：{matches[0]}")
        topic_id = topic_id[0]
        count = db.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?", (topic_id,)
        ).fetchone()[0]
        selection = SELECTION[box]
        lo = max(0, selection["center"] - selection["window"] // 2)
        if count <= lo:
            raise ValueError(f"盒{box}仅有{count}帧，无法读取从{lo}开始的实验窗口")
        hi = min(count, lo + selection["window"])
        rows = db.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT ? OFFSET ?",
            (topic_id, hi - lo, lo),
        )
        chunks = []
        zero_points = 0
        raw_points = 0
        widths = []
        for (blob,) in rows:
            message = deserialize_message(blob, PointCloud2)
            values = point_cloud2.read_points(
                message, field_names=["x", "y", "z", "intensity"], skip_nans=False
            )
            xyz = np.column_stack((values["x"], values["y"], values["z"])).astype(
                np.float32, copy=False
            )
            finite = np.isfinite(xyz).all(axis=1)
            nonzero = np.any(np.abs(xyz) > 1e-8, axis=1)
            keep = finite & nonzero
            if np.any(keep):
                chunks.append(
                    np.column_stack((xyz[keep], values["intensity"][keep]))
                )
            raw_points += len(xyz)
            zero_points += int(np.count_nonzero(finite & ~nonzero))
            widths.append(message.width)

    if not chunks:
        raise ValueError(f"盒{box}选定窗口没有非零点：{matches[0]}")
    points = np.vstack(chunks)
    if len(points) > 120_000:
        rng = np.random.default_rng(205)
        points = points[rng.choice(len(points), 120_000, replace=False)]
    return {
        "points": points,
        "bag_frames": count,
        "selected_frames": hi - lo,
        "raw_points": raw_points,
        "zero_points": zero_points,
        "widths": sorted(set(widths)),
        "lo": lo,
        "hi": hi,
    }


def write_overview(output: Path, data: dict[str, dict[str, object]]) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(15, 17), constrained_layout=True)
    for row, box in enumerate(("201", "202", "203")):
        points = data[box]["points"]
        xyz = points[:, :3]
        for col, (yidx, title) in enumerate(((1, "Top view XY"), (2, "Side view XZ"))):
            scatter = axes[row, col].scatter(
                xyz[:, 0], xyz[:, yidx], c=xyz[:, 2 if col == 0 else 1],
                s=1.0, alpha=0.42, cmap="turbo", linewidths=0,
                norm=Normalize(
                    vmin=np.percentile(xyz[:, 2 if col == 0 else 1], 1),
                    vmax=np.percentile(xyz[:, 2 if col == 0 else 1], 99),
                ),
            )
            axes[row, col].set(
                xlabel="X (m)", ylabel="Y (m)" if col == 0 else "Z (m)",
                title=(f"Box {box} · {title}\n"
                       f"frames {data[box]['lo']}–{data[box]['hi'] - 1}, "
                       f"valid points {len(points):,}"),
            )
            axes[row, col].set_aspect("equal", adjustable="box")
            axes[row, col].grid(alpha=0.2)
            figure.colorbar(scatter, ax=axes[row, col], fraction=0.046, pad=0.03,
                            label="Z (m)" if col == 0 else "Y (m)")
    figure.suptitle("Latest three-box Livox representative windows (XYZ zeros removed)")
    figure.savefig(output / "livox_overview.png", dpi=180)
    plt.close(figure)


def write_html(output: Path, box: str, data: dict[str, object]) -> bool:
    try:
        import plotly.graph_objects as go
    except ImportError:
        return False
    points = data["points"]
    figure = go.Figure(data=[go.Scatter3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2], mode="markers",
        marker={"size": 1.5, "color": points[:, 2], "colorscale": "Turbo", "opacity": 0.65,
                "colorbar": {"title": "Z (m)"}},
        hoverinfo="skip",
    )])
    figure.update_layout(
        title=(f"Box {box} Livox · frames {data['lo']}–{data['hi'] - 1} · "
               f"valid points {len(points):,}"),
        scene={"xaxis_title": "X (m)", "yaxis_title": "Y (m)", "zaxis_title": "Z (m)",
               "aspectmode": "data"}, margin={"l": 0, "r": 0, "t": 45, "b": 0},
    )
    figure.write_html(output / f"box{box}_livox_3d.html", include_plotlyjs="inline")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="三盒联合包解压后的目录")
    parser.add_argument("--output", type=Path, help="输出目录，默认 <input>/livox_visualization")
    args = parser.parse_args()
    output = args.output or args.input / "livox_visualization"
    output.mkdir(parents=True, exist_ok=True)
    data = {box: load_box(args.input, box) for box in ("201", "202", "203")}
    write_overview(output, data)
    html = {box: write_html(output, box, data[box]) for box in data}
    summary = {}
    for box, value in data.items():
        xyz = value["points"][:, :3]
        summary[box] = {
            "bag_frames": value["bag_frames"],
            "selected_frame_range": [value["lo"], value["hi"] - 1],
            "selected_frames": value["selected_frames"],
            "raw_points_in_window": value["raw_points"],
            "nonzero_finite_points_plotted": len(value["points"]),
            "zero_points_in_window": value["zero_points"],
            "point_widths": value["widths"],
            "xyz_min": xyz.min(0).round(4).tolist(),
            "xyz_max": xyz.max(0).round(4).tolist(),
            "html_written": html[box],
        }
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"已生成：{output / 'livox_overview.png'}")
    for box in data:
        if html[box]:
            print(f"已生成：{output / f'box{box}_livox_3d.html'}")
        else:
            print(f"盒{box}跳过HTML：当前Python环境未安装plotly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

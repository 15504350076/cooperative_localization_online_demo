#!/usr/bin/env python3
"""联合静止包：导出逐流时间统计和共用原点的消息时间曲线（需ROS环境）。"""
import csv
from pathlib import Path
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main(directory):
    streams = {}
    for bag in sorted(directory.iterdir()):
        if not (bag / 'metadata.yaml').is_file():
            continue
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, raw, received = reader.read_next()
            if topic not in ('/imu/raw', '/uwb/range'):
                continue
            msg = deserialize_message(raw, types[topic])
            label = 'IMU v' + bag.name.rsplit('v', 1)[1] if topic == '/imu/raw' else f'UWB {msg.src_id}->{msg.target_id}'
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            streams.setdefault(label, []).append((stamp, received))
    fields = ['stream', 'count', 'receive_span_s', 'receive_hz', 'header_span_s', 'zero', 'rollback', 'duplicate',
              'header_dt_median_ms', 'header_dt_max_ms', 'header_gap_gt_100ms', 'receive_minus_header_median_s', 'offset_p95_minus_p05_ms']
    origin = min(v[0][0] for v in streams.values())
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    with (directory / '联合时间详细统计.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for label, values in streams.items():
            a = np.asarray(values, dtype=np.int64)
            dt = np.diff(a[:, 0])
            recv_span = (int(a[-1, 1]) - int(a[0, 1])) / 1e9
            offset = (a[:, 1] - a[:, 0]) / 1e9
            row = [label, len(a), recv_span, (len(a)-1)/recv_span if recv_span else 0,
                   (int(a[-1, 0])-int(a[0, 0]))/1e9, int(np.sum(a[:, 0] == 0)), int(np.sum(dt < 0)), int(np.sum(dt == 0)),
                   float(np.median(dt)/1e6) if len(dt) else 0, float(np.max(dt)/1e6) if len(dt) else 0,
                   int(np.sum(dt > 100_000_000)), float(np.median(offset)), float(np.ptp(np.percentile(offset, [5, 95])) * 1000)]
            writer.writerow(row)
            print(row)
            stride = max(1, len(a)//2000)
            elapsed = (a[:, 1] - a[0, 1])/1e9
            axes[0].plot(elapsed[::stride], ((a[:, 0]-origin)/1e9)[::stride], label=label)
            axes[1].plot(elapsed[::stride], ((offset-offset[0])*1000)[::stride], label=label)
    axes[0].set_ylabel('Header time - ONE shared origin (s)')
    axes[0].set_title('Static recording: overlapping curves do NOT prove synchronization')
    axes[1].set_ylabel('Change in (receive - header) since first sample (ms)')
    for ax in axes:
        ax.set_xlabel('Receive elapsed time WITHIN each bag (s; not common event time)')
        ax.grid(True)
        ax.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(directory / '联合时间轴对比.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main(Path(sys.argv[1]).resolve())

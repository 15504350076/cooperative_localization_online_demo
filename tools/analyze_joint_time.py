#!/usr/bin/env python3
"""联合静止包：导出逐流时间统计和共用原点的消息时间曲线（需ROS环境）。"""
# 文件输出用标准库；numpy负责逐帧数组统计，ROS负责按消息定义解码CDR。
import csv
from pathlib import Path
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')  # 离线保存PNG，不打开窗口；必须在导入pyplot前设置。
import matplotlib.pyplot as plt


def plot_timestamp_comparison(directory, streams):
    """直接比较完整header时间：横纵轴使用同一个整数秒起点，不校正偏差。"""
    # R是各录包机写入bag的记录时间，H是消息自带时间；R不是独立同步真值。
    origin = min(v[0][0] for v in streams.values()) // 10**9 * 10**9
    with plt.rc_context({'font.family': ['Noto Sans CJK JP', 'Noto Sans CJK SC', 'sans-serif'], 'axes.unicode_minus': False}):
        fig, axes = plt.subplots(2, 1, figsize=(13, 10))
        colors = plt.get_cmap('tab10').colors
        for i, (label, values) in enumerate(sorted(streams.items())):
            a = np.asarray(values, dtype=np.int64)
            # 先做整数纳秒减法，再转浮点秒，避免大Unix秒相减损失精度。
            x, y = (a[:, 1] - origin) / 1e9, (a[:, 0] - origin) / 1e9
            stride = max(1, len(a) // 3000)
            name = label.replace('IMU v', 'IMU 盒')
            style = '-' if label.startswith('IMU') else '--'
            axes[0].plot(x[::stride], y[::stride], style, color=colors[i % 10], label=name, alpha=.8)
            # 局部图不抽样；保留UWB约100ms采样间隔，不插造同时采样的数据。
            mask = (x >= 10) & (x <= 12)
            axes[1].plot(x[mask], y[mask], style, color=colors[i % 10],
                         marker='.' if label.startswith('UWB') else None, alpha=.8)
        axes[0].set_title('全程：三路 IMU 与六路 UWB 的消息时间戳（曲线重叠不代表精确同步）')
        axes[0].legend(ncol=3, fontsize=10)
        axes[1].set_title('局部放大：相同记录时刻附近，IMU 与 UWB 的消息时间仍有差距')
        axes[1].set_xlim(10, 12)
        for ax in axes:
            ax.set_xlabel('包内记录时间 R − T0（秒）；跨盒的记录时钟尚未独立验证同步')
            ax.set_ylabel('消息时间 H − T0（秒）')
            ax.grid(True, alpha=.3)
        fig.suptitle(f'IMU / UWB 原始时间戳对比 · {directory.name}\n'
                     f'H = header.stamp.sec + nanosec / 1e9；共同起点 T0 = {origin // 10**9} Unix秒', fontsize=12)
        fig.text(.5, .015, '未平移对齐、未校正时间。曲线差距包含时钟偏差与处理/传输延迟，不等于采样同步误差。',
                 ha='center', fontsize=10)
        fig.tight_layout(rect=(0, .045, 1, .93))
        fig.savefig(directory / 'IMU与UWB原始时间戳对比.png', dpi=160)
        plt.close(fig)


def main(directory):
    # 输入：实验根目录，直接子目录为各录包。要求至少有一条IMU或UWB消息。
    # 同盒同类型若有多个包会合并到同一label，因此每轮应只放每路一个包。
    streams = {}
    for bag in sorted(directory.iterdir()):
        if not (bag / 'metadata.yaml').is_file():
            continue
        # 顺序读取保留接收顺序，不能按header排序，否则回退异常会被掩盖。
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        # get_message获取已安装的Python消息类；需要source自定义消息工作空间。
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, raw, received = reader.read_next()
            if topic not in ('/imu/raw', '/uwb/range'):
                continue
            msg = deserialize_message(raw, types[topic])
            # IMU按录包盒号区分；UWB按消息中的源/目标区分，录包盒不是测距源。
            label = 'IMU v' + bag.name.rsplit('v', 1)[1] if topic == '/imu/raw' else f'UWB {msg.src_id}->{msg.target_id}'
            # 先用整数纳秒保存绝对时间，避免浮点Unix秒损失细小时间差。
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            streams.setdefault(label, []).append((stamp, received))
    # 表头单位：跨度/偏差为s，间隔/分位宽度为ms；统计使用全部样本。
    fields = ['stream', 'count', 'receive_span_s', 'receive_hz', 'header_span_s', 'zero', 'rollback', 'duplicate',
              'header_dt_median_ms', 'header_dt_max_ms', 'header_gap_gt_100ms', 'receive_minus_header_median_s', 'offset_p95_minus_p05_ms']
    # 所有流共用一个纵轴原点，保留IMU与UWB之间的epoch差异。
    # 这是各流首帧的最小值，不保证是发生回退后全部样本的最小值。
    origin = min(v[0][0] for v in streams.values())
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    # BOM便于Excel识别UTF-8；同名分析文件会覆盖，原始bag不改。
    with (directory / '联合时间详细统计.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for label, values in streams.items():
            a = np.asarray(values, dtype=np.int64)  # 第0列header，第1列bag接收时间。
            dt = np.diff(a[:, 0])
            recv_span = (int(a[-1, 1]) - int(a[0, 1])) / 1e9
            # 包含时钟偏移与传输/处理延迟；未验证同源前不能称为网络延迟。
            offset = (a[:, 1] - a[:, 0]) / 1e9
            # N帧有N-1个间隔；dt<0回退、dt==0重复，零header另行计数。
            # P95-P05描述偏差波动宽度，不是同步精度。100ms仅为统计阈值。
            row = [label, len(a), recv_span, (len(a)-1)/recv_span if recv_span else 0,
                   (int(a[-1, 0])-int(a[0, 0]))/1e9, int(np.sum(a[:, 0] == 0)), int(np.sum(dt < 0)), int(np.sum(dt == 0)),
                   float(np.median(dt)/1e6) if len(dt) else 0, float(np.max(dt)/1e6) if len(dt) else 0,
                   int(np.sum(dt > 100_000_000)), float(np.median(offset)), float(np.ptp(np.percentile(offset, [5, 95])) * 1000)]
            writer.writerow(row)
            print(row)
            # 仅绘图抽样到约2000点；短暂尖峰可能没画到，CSV统计仍保留它。
            stride = max(1, len(a)//2000)
            # 横轴每流各自置零，不能据曲线重合证明跨盒同一物理事件同步。
            elapsed = (a[:, 1] - a[0, 1])/1e9
            axes[0].plot(elapsed[::stride], ((a[:, 0]-origin)/1e9)[::stride], label=label)
            # 下图去除首帧偏差，只展示偏差变化；绝对偏差看CSV及上图。
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
    plot_timestamp_comparison(directory, streams)


if __name__ == '__main__':
    # 唯一位置参数是实验目录，路径可为相对路径。
    main(Path(sys.argv[1]).resolve())

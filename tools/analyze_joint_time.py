#!/usr/bin/env python3
"""离线汇总 IMU、UWB、Livox、GNSS及标准图像包的时间统计（需 ROS 环境）。

用法：
  python3 tools/analyze_joint_time.py <联合包目录> [其他传感器bag目录 ...]

其他bag可不传，或传入多盒雷达、GNSS、图像包。按SQLite写入顺序读取，
保留接收时间回退；只统计Header和录包时间，不证明硬件同步或传输延迟。
"""
# 文件输出用标准库；numpy负责逐帧数组统计，ROS负责按消息定义解码CDR。
import argparse
import csv
from pathlib import Path
import sqlite3
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')  # 离线保存PNG，不打开窗口；必须在导入pyplot前设置。
import matplotlib.pyplot as plt


HEADER_TYPES = {
    'sensor_msgs/msg/Imu', 'sensor_msgs/msg/PointCloud2',
    'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
    'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/NavSatFix',
    'cooperative_interfaces/msg/UwbRange',
    'cooperative_interfaces/msg/GnssPosition',
}


def append_bag_streams(streams, bag, topics=None):
    """读取原始写入顺序；ROS按接收时间排序的reader会掩盖接收回退。"""
    metadata = rosbag2_py.Info().read_metadata(str(bag), '')
    if metadata.storage_identifier != 'sqlite3':
        raise ValueError(f'当前工具仅支持sqlite3 bag：{bag}')
    types = {}
    # 分片顺序使用metadata，片内使用id；原始数据库仅以只读方式打开。
    for filename in metadata.relative_file_paths:
        with sqlite3.connect((bag / filename).resolve().as_uri() + '?mode=ro', uri=True) as db:
            rows = db.execute('SELECT t.name, t.type, m.data, m.timestamp '
                              'FROM messages m JOIN topics t ON m.topic_id=t.id ORDER BY m.id')
            for topic, type_name, raw, received in rows:
                if type_name not in HEADER_TYPES or (topics is not None and topic not in topics):
                    continue
                if type_name not in types:
                    types[type_name] = get_message(type_name)
                msg = deserialize_message(raw, types[type_name])
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if type_name == 'cooperative_interfaces/msg/UwbRange':
                    label = f'UWB {msg.src_id}->{msg.target_id}'
                elif topic == '/imu/raw':
                    label = ('IMU v' + bag.name.rsplit('_imu_rec_v', 1)[1]
                             if '_imu_rec_v' in bag.name else f'IMU {bag.name}')
                elif topic == '/livox/lidar':
                    label = f'LiDAR {bag.name}'
                elif topic == '/livox/imu':
                    label = f'Livox IMU {bag.name}'
                else:
                    label = f'{topic} {bag.name}'
                streams.setdefault(label, []).append((stamp, int(received)))


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
        axes[0].set_title('全程：已录各传感器的消息时间戳（曲线重叠不代表精确同步）')
        axes[0].legend(ncol=3, fontsize=10)
        axes[1].set_title('局部放大：相同记录时间附近的原始消息时间')
        axes[1].set_xlim(10, 12)
        for ax in axes:
            ax.set_xlabel('包内记录时间 R − T0（秒）；跨盒的记录时钟尚未独立验证同步')
            ax.set_ylabel('消息时间 H − T0（秒）')
            ax.grid(True, alpha=.3)
        fig.suptitle(f'传感器原始时间戳对比 · {directory.name}\n'
                     f'H = header.stamp.sec + nanosec / 1e9；共同起点 T0 = {origin // 10**9} Unix秒', fontsize=12)
        fig.text(.5, .015, '未平移对齐、未校正时间。曲线差距包含时钟偏差与处理/传输延迟，不等于采样同步误差。',
                 ha='center', fontsize=10)
        fig.tight_layout(rect=(0, .045, 1, .93))
        fig.savefig(directory / 'IMU与UWB原始时间戳对比.png', dpi=160)
        plt.close(fig)


def main(directory, lidar_bag=None):
    # 输入：实验根目录或单个bag；至少有一条受支持的传感器Header消息。
    # 同盒同类型若有多个包会合并到同一label，因此每轮应只放每路一个包。
    streams = {}
    bags = []
    if (directory / 'metadata.yaml').is_file():
        # 同时录制得到的单个 uwb_imu bag 也可直接作为输入。
        bags.append(directory)
    else:
        for bag in sorted(directory.iterdir()):
            if not (bag / 'metadata.yaml').is_file():
                continue
            bags.append(bag)
    if lidar_bag is not None:
        # 保留main(directory, 单个Path)的原调用，同时支持多个附加bag。
        bags.extend([lidar_bag] if isinstance(lidar_bag, Path) else lidar_bag)
    for bag in dict.fromkeys(b.resolve() for b in bags):
        if not (bag / 'metadata.yaml').is_file():
            raise ValueError(f'不是有效的传感器 bag：{bag}')
        append_bag_streams(streams, bag)
    if not streams:
        raise ValueError('没有找到可分析的传感器Header消息；先检查bag消息数和类型')
    # 表头单位：跨度/偏差为s，间隔/分位宽度为ms；统计使用全部样本。
    fields = ['stream', 'count', 'receive_span_s', 'receive_hz', 'header_span_s', 'zero', 'rollback', 'duplicate',
              'header_dt_median_ms', 'header_dt_max_ms', 'header_gap_gt_100ms', 'receive_minus_header_median_s', 'offset_p95_minus_p05_ms',
              'receive_rollback', 'receive_duplicate', 'first_header_ns', 'last_header_ns', 'first_receive_ns', 'last_receive_ns']
    # 所有流共用一个纵轴原点，保留IMU与UWB之间的epoch差异。
    # 这是各流首帧的最小值，不保证是发生回退后全部样本的最小值。
    origin = min(v[0][0] for v in streams.values())
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    report = ['# 联合时间验收', '', '原始bag只读；按数据库写入顺序逐流统计，UWB按src_id→target_id拆流。',
              '缺失话题不会出现在表中，必须先结合采集清单和ros2 bag info核对各盒各传感器。', '',
              '| 数据流 | 数量 | 接收频率Hz | Header首时间s | Header末时间s | Header零/回退/重复 | 接收回退/重复 | R−H中位数ms |',
              '|---|---:|---:|---:|---:|---:|---:|---:|']
    # BOM便于Excel识别UTF-8；同名分析文件会覆盖，原始bag不改。
    with (directory / '联合时间详细统计.csv').open('w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        for label, values in streams.items():
            a = np.asarray(values, dtype=np.int64)  # 第0列header，第1列bag接收时间。
            dt = np.diff(a[:, 0])
            receive_dt = np.diff(a[:, 1])
            recv_span = (int(a[-1, 1]) - int(a[0, 1])) / 1e9
            recv_rate = ((len(a) - 1) / recv_span
                         if recv_span > 0 and not np.any(receive_dt < 0) else float('nan'))
            # 包含时钟偏移与传输/处理延迟；未验证同源前不能称为网络延迟。
            offset = (a[:, 1] - a[:, 0]) / 1e9
            # N帧有N-1个间隔；dt<0回退、dt==0重复，零header另行计数。
            # P95-P05描述偏差波动宽度，不是同步精度。100ms仅为统计阈值。
            row = [label, len(a), recv_span, recv_rate,
                   (int(a[-1, 0])-int(a[0, 0]))/1e9, int(np.sum(a[:, 0] == 0)), int(np.sum(dt < 0)), int(np.sum(dt == 0)),
                   float(np.median(dt)/1e6) if len(dt) else 0, float(np.max(dt)/1e6) if len(dt) else 0,
                   int(np.sum(dt > 100_000_000)), float(np.median(offset)), float(np.ptp(np.percentile(offset, [5, 95])) * 1000),
                   int(np.sum(receive_dt < 0)), int(np.sum(receive_dt == 0)),
                   int(a[0, 0]), int(a[-1, 0]), int(a[0, 1]), int(a[-1, 1])]
            writer.writerow(row)
            print(row)
            first = f'{int(a[0, 0]) // 10**9}.{int(a[0, 0]) % 10**9:09d}'
            last = f'{int(a[-1, 0]) // 10**9}.{int(a[-1, 0]) % 10**9:09d}'
            report.append(f'| {label} | {len(a)} | {row[3]:.3f} | {first} | {last} | '
                          f'{row[5]}/{row[6]}/{row[7]} | {row[13]}/{row[14]} | {row[11] * 1000:.3f} |')
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
    report.extend(['', '接收时钟回退或跳变时，接收跨度/频率不能作为可靠采样率；先看异常和Header间隔。',
                   'R−H同时包含时钟偏差和处理/传输延迟，不能直接解释为网络延迟。',
                   '最近采样时间接近、时间区间重叠或曲线重合，都不能证明硬件同步；静态包无法提供独立同步真值。',
                   '静态CameraInfo可能没有采样时间，图像同步应检查图像本身的Header。',
                   '联合时间详细统计.csv保留整数纳秒首末值；100ms只是长间隔统计阈值，不是所有传感器的合格门限。'])
    (directory / '联合时间验收.md').write_text('\n'.join(report) + '\n', encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path, help='联合实验目录，或单个传感器bag目录')
    parser.add_argument('extra_bags', nargs='*', type=Path, help='可选：同轮其他盒的雷达、GNSS或图像bag')
    args = parser.parse_args()
    main(args.directory.resolve(), args.extra_bags)

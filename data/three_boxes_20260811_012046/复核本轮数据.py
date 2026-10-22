#!/usr/bin/env python3
"""复核本轮原始数据库计数和201 IMU异常，只写中文CSV与图片。先加载ROS环境。"""
from collections import Counter, defaultdict
import csv
from pathlib import Path
import sqlite3
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1] / 'tools'))
from compare_uwb_timestamps import exact_three_edge_epochs

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def write_csv(name, fields, rows):
    with (ROOT / name).open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def main():
    counts, edge_rows, completeness, imu_arrays = [], [], [], None
    for box in ('201', '202', '203'):
        matches = [p for p in ROOT.glob(f'uwb_imu_livox_v{box}_*') if p.is_dir()]
        if len(matches) != 1:
            raise ValueError(f'盒{box}目录不唯一：{matches}')
        run = matches[0]
        totals = Counter()
        for sensor in ('imu', 'livox', 'uwb'):
            bag = run / sensor
            metadata = rosbag2_py.Info().read_metadata(str(bag), '')
            declared = {item.topic_metadata.name: item.message_count
                        for item in metadata.topics_with_message_count}
            actual = Counter()
            for filename in metadata.relative_file_paths:
                with sqlite3.connect((bag / filename).as_uri() + '?mode=ro', uri=True) as db:
                    actual.update(dict(db.execute('SELECT t.name, COUNT(*) FROM messages m '
                                                 'JOIN topics t ON m.topic_id=t.id GROUP BY t.name')))
                    if box == '201' and sensor == 'imu':
                        h, r, q, gyro = [], [], [], []
                        for raw, received in db.execute('SELECT m.data,m.timestamp FROM messages m '
                                                       'JOIN topics t ON m.topic_id=t.id '
                                                       "WHERE t.name='/imu/raw' ORDER BY m.id"):
                            message = deserialize_message(raw, Imu)
                            h.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
                            r.append(received)
                            q.append((message.orientation.x, message.orientation.y,
                                      message.orientation.z, message.orientation.w))
                            gyro.append((message.angular_velocity.x, message.angular_velocity.y,
                                         message.angular_velocity.z))
                        if imu_arrays is None:
                            imu_arrays = [h, r, q, gyro]
                        else:
                            for target, values in zip(imu_arrays, (h, r, q, gyro)):
                                target.extend(values)
            expected = {'imu': ['/imu/raw'], 'livox': ['/livox/lidar', '/livox/imu'],
                        'uwb': ['/uwb/range']}[sensor]
            for topic in sorted(set(expected) | set(actual)):
                counts.append([box, topic, actual[topic], declared.get(topic, 0),
                               '一致' if actual[topic] == declared.get(topic, 0) else '不一致'])
            totals.update(actual)
        completeness.append([box, totals['/imu/raw'], totals['/livox/lidar'],
                             totals['/livox/imu'], totals['/uwb/range']])
        streams = defaultdict(list)
        with (run / 'uwb/UWB测距数据.csv').open(encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                edge = (int(row['源车辆ID']), int(row['目标车辆ID']))
                h = int(row['消息时间_sec']) * 10**9 + int(row['消息时间_nanosec'])
                streams[edge].append((int(row['序号']), int(row['rosbag接收时间_ns']), h))
        edges = [sum(len(values) for edge, values in streams.items() if set(edge) == set(pair))
                 for pair in ((1, 2), (1, 3), (2, 3))]
        complete_epochs, epochs, _ = exact_three_edge_epochs(streams)
        edge_rows.append([box, *edges, complete_epochs, epochs, ','.join(str(i) for i in sorted({e[0] for e in streams}))])
    write_csv('采集完整性复核.csv', ['录包盒号', '话题', '数据库实际消息数', '清单消息数', '计数是否一致'], counts)
    write_csv('UWB边覆盖复核.csv', ['录包盒号', '1—2边消息数', '1—3边消息数', '2—3边消息数',
                                  '三边完整同消息时间历元数', '不同消息时间数量', '实际源编号'], edge_rows)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    for ax, title, rows, labels in [(axes[0], '上：原始数据库消息计数（红格表示未录到）', completeness,
                                    ['录包盒号', '车载IMU', 'Livox点云', 'Livox内置IMU', 'UWB测距']),
                                   (axes[1], '下：UWB无向边覆盖（不是六方向重复计算）',
                                    [row[:5] for row in edge_rows],
                                    ['录包盒号', '1—2边', '1—3边', '2—3边', '三边完整历元'])]:
        ax.axis('off')
        ax.set_title(title, pad=12)
        table = ax.table(cellText=rows, colLabels=labels, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(12)
        table.scale(1, 2.4)
        for i, row in enumerate(rows):
            for j, value in enumerate(row):
                if j:
                    table[(i+1, j)].set_facecolor('#f8cccc' if value == 0 else '#ddf2df')
    fig.suptitle('本轮关键问题：202/203车载IMU为空，三个录包均缺2—3测距边', fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(ROOT / '采集完整性与UWB边覆盖.png', dpi=160)
    plt.close(fig)

    h, r, q, gyro = (np.asarray(x) for x in imu_arrays)
    h, r = h.astype(np.int64), r.astype(np.int64)
    dt = np.diff(h) / 1e6
    elapsed = (r - r[0]) / 1e9
    q = q / np.linalg.norm(q, axis=1)[:, None]
    angles = np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(q[:-1] * q[1:], axis=1)), 0, 1)))
    issues = []
    for i in np.flatnonzero((dt < 0) | (dt > 100) | (angles > 10)):
        reason = '消息时间回退' if dt[i] < 0 else ('消息时间正向大跳' if dt[i] > 100 else '姿态突变')
        issues.append([int(i), int(i+1), reason, float(elapsed[i+1]), int(h[i]), int(h[i+1]),
                       float(dt[i]), float(angles[i]), float(np.linalg.norm(gyro[i+1]))])
    write_csv('201_IMU异常明细.csv', ['前帧序号（从0起）', '后帧序号（从0起）', '异常类型',
                                   '记录相对时间（秒）', '前帧消息时间（纳秒）', '后帧消息时间（纳秒）',
                                   '消息间隔（毫秒）', '相邻姿态旋转角（度）', '后一帧角速度模长（弧度每秒）'], issues)
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    window = elapsed[1:] <= 14
    axes[0].plot(elapsed[1:][window], dt[window], lw=.7)
    for row in issues:
        if row[3] <= 14:
            axes[0].annotate(f'{row[3]:.3f}秒：{row[6]:+.3f}毫秒', (row[3], row[6]),
                             xytext=(12, -20 if row[6] > 0 else 15), textcoords='offset points')
    axes[0].set(title='上：前14秒消息间隔——正常约1毫秒，出现约−1999和+1001毫秒跳变', ylabel='消息相邻间隔（毫秒）')
    mask = elapsed <= 14
    axes[1].plot(elapsed[mask], ((r-h) / 1e6)[mask], lw=.7)
    axes[1].set(title='中：记录减消息时间——秒级阶跃（包含时钟偏差，非纯传输延迟）', ylabel='记录减消息时间（毫秒）')
    axes[2].plot(elapsed[1:], angles, lw=.7)
    peak = int(np.argmax(angles))
    axes[2].annotate(f'{elapsed[peak+1]:.3f}秒：{angles[peak]:.6f}度',
                     (elapsed[peak+1], angles[peak]), xytext=(15, -20), textcoords='offset points')
    axes[2].set(title='下：全程相邻姿态旋转——约64.953秒处接近180度突变', ylabel='相邻姿态旋转角（度）')
    for ax in axes:
        ax.set_xlabel('本盒记录时间相对首帧（秒）')
        ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(ROOT / '201_IMU时间异常与姿态突变.png', dpi=160)
    plt.close(fig)
    print('完整性与UWB覆盖：', completeness, edge_rows)
    print('201 IMU异常：', issues)
    print('已写入本轮根目录：两张问题图和三份复核CSV。原始数据库未修改。')


if __name__ == '__main__':
    main()

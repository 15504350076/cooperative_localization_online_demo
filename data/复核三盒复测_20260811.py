#!/usr/bin/env python3
"""复核8月11日两次三盒复测；先运行总分析生成全量UWB CSV，再加载ROS环境执行。

仅写派生CSV、中文图片和统计摘要，数据库以只读方式按写入顺序读取。
无参数复核054640与060745；也可传一个或多个解压目录。
"""
import argparse
from collections import Counter, defaultdict
import csv
from pathlib import Path
import sqlite3
import struct
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2

DATA = Path(__file__).resolve().parent
sys.path.insert(0, str(DATA.parent / 'tools'))
from compare_uwb_timestamps import exact_three_edge_epochs

plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'Noto Sans CJK JP', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def write_csv(root, name, fields, rows):
    with (root / name).open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


def review(root):
    root = root.resolve()
    counts, edges, directions, timing, issues, points, cars = [], [], [], [], [], [], {}
    for box in ('201', '202', '203'):
        matches = [p for p in root.glob(f'uwb_imu_livox_v{box}_*') if p.is_dir()]
        if len(matches) != 1:
            raise ValueError(f'{root.name}盒{box}目录不唯一：{matches}')
        run = matches[0]
        streams = defaultdict(list)
        q, gyro = [], []
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
                    if sensor == 'uwb':
                        continue  # 使用总分析输出的逐条全量CSV，不重复依赖自定义消息反序列化。
                    for topic, raw, received in db.execute(
                            'SELECT t.name,m.data,m.timestamp FROM messages m '
                            'JOIN topics t ON m.topic_id=t.id ORDER BY m.id'):
                        msg = deserialize_message(raw, PointCloud2 if topic == '/livox/lidar' else Imu)
                        h = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                        index = len(streams[topic])
                        previous = streams[topic][-1] if index else None
                        streams[topic].append((index, received, h))
                        if topic == '/imu/raw':
                            q.append((msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
                            gyro.append((msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))
                        if topic == '/livox/lidar':
                            field = next((f for f in msg.fields if f.name == 'timestamp'), None)
                            if field is None or field.datatype != 8 or not msg.width or not msg.height:
                                raise ValueError('复核脚本要求本批PointCloud2的双精度逐点timestamp字段')
                            fmt = '>d' if msg.is_bigendian else '<d'
                            first = struct.unpack_from(fmt, msg.data, field.offset)[0]
                            last_offset = (msg.height-1)*msg.row_step + (msg.width-1)*msg.point_step + field.offset
                            last = struct.unpack_from(fmt, msg.data, last_offset)[0]
                            span = (last-first)/1e6  # 与雷达分析一致，按纳秒解释该驱动字段。
                            if span > 150:
                                points.append([box, index, h, received, msg.width*msg.height, span,
                                               (h-previous[2])/1e6 if previous else '',
                                               (received-previous[1])/1e6 if previous else ''])
            expected = {'imu': ['/imu/raw'], 'livox': ['/livox/lidar', '/livox/imu'],
                        'uwb': ['/uwb/range']}[sensor]
            for topic in sorted(set(expected) | set(actual)):
                counts.append([box, topic, actual[topic], declared.get(topic, 0),
                               '一致' if actual[topic] == declared.get(topic, 0) else '不一致'])
        uwb = defaultdict(list)
        with (run / 'uwb/UWB测距数据.csv').open(encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                edge = (int(row['源车辆ID']), int(row['目标车辆ID']))
                h = int(row['消息时间_sec'])*10**9 + int(row['消息时间_nanosec'])
                uwb[edge].append((int(row['序号']), int(row['rosbag接收时间_ns']), h))
        complete, epochs, _ = exact_three_edge_epochs(uwb)
        edge_counts = [sum(len(v) for e, v in uwb.items() if set(e) == set(pair))
                       for pair in ((1, 2), (1, 3), (2, 3))]
        edges.append([box, *edge_counts, complete, epochs])
        for edge, rows in sorted(uwb.items()):
            directions.append([box, f'{edge[0]}→{edge[1]}', len(rows)])
            streams[f'UWB {edge[0]}→{edge[1]}'] = rows
        for topic, rows in streams.items():
            arr = np.asarray(rows, dtype=np.int64)
            dh, dr = np.diff(arr[:, 2])/1e6, np.diff(arr[:, 1])/1e6
            timing.append([box, topic, len(rows), int(sum(dh < 0)), int(sum(dr < 0)),
                           float(np.min(dh)), float(np.max(dh)), float(np.min(dr)), float(np.max(dr)),
                           float(np.median((arr[:, 1]-arr[:, 2])/1e6))])
        arr = np.asarray(streams['/imu/raw'], dtype=np.int64)
        h, r = arr[:, 2], arr[:, 1]
        dh, dr = np.diff(h)/1e6, np.diff(r)/1e6
        quat = np.asarray(q)
        quat /= np.linalg.norm(quat, axis=1)[:, None]
        angles = np.degrees(2*np.arccos(np.clip(np.abs(np.sum(quat[:-1]*quat[1:], axis=1)), 0, 1)))
        for i in np.flatnonzero((dh < 0) | (dh > 100) | (dr < 0) | (angles > 10)):
            why = '；'.join(name for flag, name in [(dh[i] < 0, '消息时间回退'), (dh[i] > 100, '消息正向大跳'),
                                                   (dr[i] < 0, '记录时间回退'), (angles[i] > 10, '姿态突变')] if flag)
            issues.append([box, int(i), int(i+1), why, int(h[i]), int(h[i+1]), int(r[i]), int(r[i+1]),
                           float(dh[i]), float(dr[i]), float(angles[i]), float(np.linalg.norm(gyro[i+1]))])
        cars[box] = (dh, dr, angles)
        print(f'{root.name}盒{box}复核完成', flush=True)

    write_csv(root, '采集完整性复核.csv', ['录包盒号', '话题', '数据库实际消息数', '清单消息数', '计数是否一致'], counts)
    write_csv(root, 'UWB边覆盖复核.csv', ['录包盒号', '1—2边消息数', '1—3边消息数', '2—3边消息数',
                                      '三边完整同消息时间历元数', '不同消息时间数量'], edges)
    write_csv(root, 'UWB六方向计数.csv', ['录包盒号', '方向', '消息数'], directions)
    write_csv(root, '时间连续性复核.csv', ['录包盒号', '数据流', '消息数', '消息时间回退数', '记录时间回退数',
                                       '最小消息间隔（毫秒）', '最大消息间隔（毫秒）', '最小记录间隔（毫秒）',
                                       '最大记录间隔（毫秒）', '记录减消息时间中位数（毫秒）'], timing)
    write_csv(root, '三盒IMU异常明细.csv', ['录包盒号', '前帧序号（从0起）', '后帧序号（从0起）', '异常类型',
                                         '前帧消息时间（纳秒）', '后帧消息时间（纳秒）', '前帧记录时间（纳秒）',
                                         '后帧记录时间（纳秒）', '消息间隔（毫秒）', '记录间隔（毫秒）',
                                         '相邻姿态旋转角（度）', '后一帧角速度模长（弧度每秒）'], issues)
    write_csv(root, '点云帧内时间跨度异常.csv', ['录包盒号', '帧序号（从0起）', '消息时间（纳秒）',
                                             '记录时间（纳秒）', '点数', '帧内首末点跨度（毫秒）',
                                             '与前帧消息间隔（毫秒）', '与前帧记录间隔（毫秒）'], points)
    fig, axes = plt.subplots(3, 3, figsize=(16, 10))
    car_rows = []
    for row, (box, arrays) in enumerate(cars.items()):
        dh, dr, angles = arrays
        car_rows.append([box, len(dh)+1, int(sum(dh < 0)), int(sum(dh > 100)), int(sum(dr < 0)),
                         int(sum(angles > 10)), float(np.max(angles))])
        for col, (values, label, detail) in enumerate(zip(arrays,
                ['消息相邻间隔（毫秒）', '记录相邻间隔（毫秒）', '相邻姿态旋转角（度）'],
                [f'消息回退{sum(dh < 0)}次', f'记录回退{sum(dr < 0)}次', f'大于10度：{sum(angles > 10)}次'])):
            ax = axes[row, col]
            ax.plot(np.arange(1, len(values)+1), values, lw=.5)
            ax.set(title=f'盒{box}：{detail}', xlabel='后帧样本序号（从0起，按数据库写入顺序）', ylabel=label)
            ax.grid(alpha=.25)
    fig.suptitle(f'{root.name}：三盒车载IMU时间与姿态连续性', fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(root / '三盒IMU时间与姿态连续性.png', dpi=150)
    plt.close(fig)
    write_csv(root, '车载IMU连续性汇总.csv', ['录包盒号', '消息数', '消息时间回退数', '消息正向间隔超过100毫秒次数',
                                           '记录时间回退数', '相邻姿态超过10度次数', '最大相邻姿态角（度）'], car_rows)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7))
    totals = [[box, *[next(r[2] for r in counts if r[:2] == [box, topic])
                     for topic in ('/imu/raw', '/livox/lidar', '/livox/imu', '/uwb/range')]]
              for box in cars]
    for ax, rows, labels, title in [(axes[0], totals, ['录包盒号', '车载IMU', '点云', '雷达内置IMU', 'UWB测距'],
                                    '上：数据库实际消息数（绿色只代表非空）'),
                                   (axes[1], edges, ['录包盒号', '1—2边', '1—3边', '2—3边', '严格完整历元', '不同消息时间'],
                                    '下：三边覆盖及完全相同消息时间的完整历元')]:
        ax.axis('off')
        ax.set_title(title)
        table = ax.table(cellText=rows, colLabels=labels, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2.5)
        for i, row in enumerate(rows):
            for j, v in enumerate(row):
                if j:
                    table[i+1, j].set_facecolor('#ddf2df' if v else '#f8cccc')
    fig.suptitle(f'{root.name}：采集完整性与UWB覆盖', fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .93))
    fig.savefig(root / '采集完整性与UWB边覆盖.png', dpi=150)
    plt.close(fig)
    print('车载IMU连续性：', car_rows, '\nUWB边与严格历元：', edges, '\n长跨度点云帧：', points, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('roots', nargs='*', type=Path)
    args = parser.parse_args()
    for directory in args.roots or [DATA / f'three_boxes_20260811_{s}' for s in ('054640', '060745')]:
        review(directory)

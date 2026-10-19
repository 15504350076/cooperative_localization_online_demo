#!/usr/bin/env python3
"""验证离线时间检查不会因接收时间排序而漏报回退，并支持图像包且不重复读包。"""

import csv
from pathlib import Path
import tempfile

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, Imu

from analyze_joint_time import append_bag_streams, main


def write_bag(path, topic, type_name, messages):
    """生成带故意接收回退的测试bag；只写临时目录。"""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name,
                                              serialization_format='cdr'))
    for message, received in messages:
        writer.write(topic, serialize_message(message), received)
    del writer


def check():
    with tempfile.TemporaryDirectory(prefix='joint-time-check-') as temporary:
        root = Path(temporary)
        imu_bag = root / 'sample_imu_rec_v1'
        images = root / 'camera_201'
        first, second = Imu(), Imu()
        first.header.stamp.sec = 5
        second.header.stamp.sec = 6
        write_bag(imu_bag, '/imu/raw', 'sensor_msgs/msg/Imu',
                  [(first, 2_000_000_000), (second, 1_000_000_000)])
        image = Image()
        image.header.stamp.sec = 5
        write_bag(images, '/camera/image_raw', 'sensor_msgs/msg/Image',
                  [(image, 2_000_000_000)])

        streams = {}
        append_bag_streams(streams, imu_bag)
        assert streams['IMU v1'] == [(5_000_000_000, 2_000_000_000),
                                     (6_000_000_000, 1_000_000_000)]
        # 附加目录同时已在根目录内时只读一次，不能把消息数量翻倍。
        main(root, [images])
        with (root / '联合时间详细统计.csv').open(encoding='utf-8-sig', newline='') as source:
            rows = list(csv.DictReader(source))
        imu = next(row for row in rows if row['stream'] == 'IMU v1')
        camera = next(row for row in rows if row['stream'].startswith('/camera/'))
        assert imu['receive_rollback'] == '1'
        assert imu['rollback'] == '0'
        assert camera['count'] == '1'
        assert 'R−H' in (root / '联合时间验收.md').read_text(encoding='utf-8')
    print('联合时间离线检查通过')


if __name__ == '__main__':
    check()

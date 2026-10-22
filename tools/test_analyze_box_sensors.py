#!/usr/bin/env python3
"""小型ROS录包回归：空IMU明确失败、后续分析继续、旧报告不被错误覆盖。"""
from pathlib import Path
import subprocess
import tempfile

import rosbag2_py
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from cooperative_interfaces.msg import UwbRange

from test_analyze_joint_time import write_bag


def check():
    script = Path(__file__).with_name('analyze_box_sensors.sh')
    with tempfile.TemporaryDirectory(prefix='box-analysis-') as temporary:
        for empty_imu in (True, False):
            root = Path(temporary) / ('缺少IMU' if empty_imu else '数据完整')
            root.mkdir()
            imu = Imu()
            imu.header.stamp.sec = 5
            imu.orientation.w = 1.0
            imu.linear_acceleration.z = 9.80665
            write_bag(root / 'imu', '/imu/raw', 'sensor_msgs/msg/Imu',
                      [] if empty_imu else [(imu, 5_000_000_000), (imu, 5_010_000_000)])
            uwb = UwbRange()
            uwb.header.stamp.sec = 5
            uwb.src_id, uwb.target_id, uwb.distance = 1, 2, 4.0
            write_bag(root / 'uwb', '/uwb/range', 'cooperative_interfaces/msg/UwbRange',
                      [(uwb, 5_000_000_000), (uwb, 5_100_000_000)])
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(root / 'livox'), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            for topic, typename in [('/livox/lidar', 'sensor_msgs/msg/PointCloud2'),
                                     ('/livox/imu', 'sensor_msgs/msg/Imu')]:
                writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename,
                                                            serialization_format='cdr'))
            cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12,
                                data=bytes(12), fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                                       for i, n in enumerate(('x', 'y', 'z'))])
            cloud.header.stamp.sec = 5
            for time in (5_000_000_000, 5_100_000_000):
                writer.write('/livox/lidar', serialize_message(cloud), time)
                writer.write('/livox/imu', serialize_message(imu), time)
            del writer
            old_report = root / 'IMU分析.txt'
            old_report.write_text('历史报告', encoding='utf-8')
            result = subprocess.run(['bash', str(script), str(root)], text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
            assert result.returncode == (1 if empty_imu else 0), result.stdout
            assert '[成功] 4/4' in result.stdout, result.stdout
            assert '本次所有分析进程均已结束' in result.stdout
            summary = (root / '分析执行结果.md').read_text()
            assert ('最终状态：部分失败（1项）' if empty_imu else '最终状态：四项程序全部执行成功') in summary
            assert not list(root.glob('*.tmp.*'))
            if empty_imu:
                assert old_report.read_text() == '历史报告'
                assert '/imu/raw' in (root / 'IMU分析.txt.失败.log').read_text()
                assert '[部分失败]' in result.stdout
            else:
                assert '消息数: 2' in old_report.read_text()
                assert '[全部完成]' in result.stdout
            assert (root / '雷达分析.txt').is_file()
            assert (root / 'uwb/UWB测距分析.md').is_file()
    print('多传感器分析：失败可见、后续继续、报告保护和成功退出检查通过')


if __name__ == '__main__':
    check()

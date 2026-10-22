#!/usr/bin/env python3
"""回归检查：接收时间回退时，IMU、点云、UWB分析仍按SQLite写入顺序读取。"""
from contextlib import redirect_stdout
import io
from pathlib import Path
import tempfile

from sensor_msgs.msg import Imu, PointCloud2, PointField
from cooperative_interfaces.msg import UwbRange
from test_analyze_joint_time import write_bag
from analyze_imu_bag import analyze
from analyze_lidar_bag import read_bag
from export_uwb_bag_readable import read_bag as read_uwb


def check():
    with tempfile.TemporaryDirectory(prefix='sensor-order-') as directory:
        root = Path(directory)
        imu1, imu2 = Imu(), Imu()
        imu1.header.stamp.sec, imu2.header.stamp.sec = 5, 6
        write_bag(root / 'imu', '/imu/raw', 'sensor_msgs/msg/Imu',
                  [(imu1, 2_000_000_000), (imu2, 1_000_000_000)])
        output = io.StringIO()
        with redirect_stdout(output):
            analyze(root / 'imu', '/imu/raw', 2, 10)
        assert '回退: 0 次' in output.getvalue()
        # 写入包含两种雷达话题的临时bag。
        import rosbag2_py
        from rclpy.serialization import serialize_message
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(root / 'livox'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for topic, type_name in [('/livox/lidar', 'sensor_msgs/msg/PointCloud2'),
                                 ('/livox/imu', 'sensor_msgs/msg/Imu')]:
            writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=type_name, serialization_format='cdr'))
        for second, received in [(5, 2_000_000_000), (6, 1_000_000_000)]:
            cloud = PointCloud2(height=1, width=1, point_step=12, row_step=12,
                                data=bytes(12), fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                                       for i, n in enumerate(('x', 'y', 'z'))])
            cloud.header.stamp.sec = second
            imu = Imu()
            imu.header.stamp.sec = second
            writer.write('/livox/lidar', serialize_message(cloud), received)
            writer.write('/livox/imu', serialize_message(imu), received)
        del writer
        lidar = read_bag(root / 'livox')
        assert lidar['/livox/lidar']['header'] == [5_000_000_000, 6_000_000_000]
        assert lidar['/livox/lidar']['receive'] == [2_000_000_000, 1_000_000_000]
        u1, u2 = UwbRange(), UwbRange()
        u1.header.stamp.sec, u2.header.stamp.sec = 5, 6
        write_bag(root / 'uwb', '/uwb/range', 'cooperative_interfaces/msg/UwbRange',
                  [(u1, 2_000_000_000), (u2, 1_000_000_000)])
        ranges, _ = read_uwb(root / 'uwb')
        assert [row['header_ns'] for row in ranges] == [5_000_000_000, 6_000_000_000]
    print('传感器包写入顺序检查通过')


if __name__ == '__main__':
    check()

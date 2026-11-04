"""坐标变换及失效保护回归；python3 -m unittest discover -s tools -p test_rte_coordinate_viewer.py。"""
import copy
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
import urllib.request

from rte_coordinate_viewer import Coordinates, rotate, serve_page


def header(t, frame):
    sec, nano = divmod(t, 1_000_000_000)
    return NS(stamp=NS(sec=sec, nanosec=nano), frame_id=frame)


def reference(t, p=(10., 20.)):
    return NS(header=header(t, 'rte_odom_1'), node_id=1, valid=True, position_enu_m=list(p)+[0.])


def pose(t, points=((0., 0.), (0., 5.), (-3., 1.))):
    return NS(header=header(t, 'coop_rte_1_local'), reference_node_id=1,
              vehicles=[NS(node_id=i+1, x_m=p[0], y_m=p[1], yaw_rad=math.pi/2,
                           position_valid=True, yaw_valid=True) for i, p in enumerate(points)])


class CoordinatesTests(unittest.TestCase):
    def ready(self):
        c = Coordinates()
        c.push_reference(reference(1_000_000_000))
        self.assertTrue(c.ingest(pose(1_000_000_000), now=1.))
        return c

    def test_initial_axes_distances_yaw_and_later_translation(self):
        c = self.ready()
        first = c.snapshot(now=1.)
        self.assertAlmostEqual(first['relative'][1]['x'], 5.)
        self.assertAlmostEqual(first['relative'][1]['y'], 0.)
        self.assertAlmostEqual(first['relative'][2]['y'], 3.)
        self.assertAlmostEqual(first['relative'][0]['yaw'], 0.)
        # 参考车向L1正Y走2m，而相对队形不变：任务系所有车向X走2m。
        c.push_reference(reference(2_000_000_000, (10., 22.)))
        c.ingest(pose(2_000_000_000), now=2.)
        latest = c.snapshot(now=2.)
        self.assertEqual(latest['anchor'], first['anchor'])
        for before, after, relative_pose in zip(first['task'], latest['task'], latest['relative']):
            self.assertAlmostEqual(after['x'], before['x']+2.)
            self.assertAlmostEqual(after['y'], before['y'])
            self.assertAlmostEqual(math.hypot(relative_pose['x'], relative_pose['y']),
                                   math.hypot(before['x'], before['y']))
        self.assertAlmostEqual(latest['relative'][0]['x'], 0.)
        # 第二车绕行不改变已锁定轴；第三车穿线允许负Y，不进行镜像翻转。
        c.push_reference(reference(3_000_000_000, (10., 22.)))
        c.ingest(pose(3_000_000_000, ((0., 0.), (5., 0.), (3., 1.))), now=3.)
        self.assertAlmostEqual(c.latest['relative'][2]['y'], -3.)
        self.assertAlmostEqual(c.latest['triangle'][2]['y'], 1.)
        self.assertFalse(c.latest['triangle'][0]['yaw_valid'])

    def test_interpolation_no_extrapolation_or_receive_timestamp(self):
        c = Coordinates()
        c.push_reference(reference(1_000_000_000, (0., 0.)))
        c.push_reference(reference(1_020_000_000, (2., 4.)))
        self.assertEqual(c.reference_at(1_010_000_000), [1., 2.])
        for t in [990_000_000, 1_030_000_000]:
            with self.assertRaises(ValueError):
                c.reference_at(t)
        c.push_reference(reference(1_200_000_000))
        with self.assertRaises(ValueError):
            c.reference_at(1_100_000_000)

    def test_pose_can_arrive_before_matching_reference(self):
        c = Coordinates()
        self.assertFalse(c.ingest(pose(1_000_000_000)))
        self.assertIsNotNone(c.pending)
        self.assertTrue(c.push_reference(reference(1_000_000_000)))
        self.assertTrue(c.snapshot()['valid'])
        self.assertIsNone(c.pending)
        self.assertAlmostEqual(c.latest['relative'][1]['x'], 5.)
        # 新的无效位置必须清掉待配对数据，不能被较晚参考状态复活。
        self.assertFalse(c.ingest(pose(2_000_000_000)))
        invalid = pose(2_100_000_000)
        invalid.vehicles[1].position_valid = False
        c.ingest(invalid)
        c.push_reference(reference(2_000_000_000))
        self.assertFalse(c.snapshot()['valid'])

    def test_invalid_stale_frozen_and_recovery(self):
        c = self.ready()
        no_heading = pose(1_000_000_000)
        no_heading.vehicles[1].yaw_valid = False
        c.ingest(no_heading, now=1.1)
        self.assertFalse(c.latest['relative'][1]['yaw_valid'])
        c.ingest(pose(1_000_000_000), now=1.4)
        self.assertFalse(c.snapshot(now=1.51)['valid'])
        c.push_reference(reference(2_000_000_000))
        good = pose(2_000_000_000)
        c.ingest(good, now=2.)
        invalid = copy.deepcopy(good)
        invalid.vehicles[1].position_valid = False
        c.ingest(invalid, now=2.1)
        self.assertFalse(c.snapshot(now=2.1)['valid'])
        c.push_reference(reference(3_000_000_000))
        recovered = pose(3_000_000_000)
        recovered.vehicles[1].yaw_valid = False
        self.assertTrue(c.ingest(recovered, now=3.))
        self.assertTrue(c.snapshot(now=3.)['valid'])
        self.assertFalse(c.latest['relative'][1]['yaw_valid'])

    def test_rollback_requires_session_restart(self):
        for stream in ['pose', 'reference']:
            c = self.ready()
            if stream == 'pose':
                c.ingest(pose(999_000_000), now=1.1)
            else:
                c.push_reference(reference(999_000_000))
            self.assertTrue(c.blocked)
            c.push_reference(reference(2_000_000_000))
            self.assertFalse(c.ingest(pose(2_000_000_000), now=2.))
            self.assertFalse(c.snapshot(now=2.)['valid'])

    def test_reject_mixed_frames_invalid_ids_and_nonfinite(self):
        for mutate in [lambda m: setattr(m.header, 'frame_id', 'coop_ref_1_relative'),
                       lambda m: setattr(m.vehicles[1], 'node_id', 1),
                       lambda m: setattr(m.vehicles[1], 'x_m', math.nan),
                       lambda m: setattr(m.vehicles[0], 'x_m', 1.),
                       lambda m: setattr(m.header.stamp, 'sec', 0),
                       lambda m: setattr(m.vehicles[1], 'position_valid', False)]:
            c = Coordinates()
            c.push_reference(reference(1_000_000_000))
            msg = pose(1_000_000_000)
            mutate(msg)
            self.assertFalse(c.ingest(msg, now=1.))
            self.assertIsNone(c.anchor)
        c = Coordinates()
        c.push_reference(reference(1_000_000_000))
        self.assertFalse(c.ingest(pose(1_000_000_000, ((0., 0.), (.01, 0.), (1., 2.)))))

    def test_http_read_only_snapshot_and_standalone_html(self):
        c = self.ready()
        server = serve_page('127.0.0.1', 0, lambda: c.snapshot(now=1.))
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        try:
            base = f'http://127.0.0.1:{server.server_port}'
            with urllib.request.urlopen(base+'/api/state') as response:
                payload = json.load(response)
                self.assertTrue(payload['valid'])
                self.assertEqual(payload['relative_frame'], 'coop_rte_1_relative_fixed')
            with urllib.request.urlopen(base) as response:
                self.assertIn('同一组车辆，三种坐标表达', response.read().decode())
            with self.assertRaises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(urllib.request.Request(base+'/api/state', data=b'{}'))
            self.assertEqual(error.exception.code, 501)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()

    @unittest.skipUnless(os.environ.get('RTE_COORDINATE_ROS_TEST') == '1', '可选：隔离域实际ROS收发测试')
    def test_actual_ros_topics_and_watchdog(self):
        self.assertNotEqual(os.environ.get('ROS_DOMAIN_ID', '0'), '0', '请使用独立ROS_DOMAIN_ID')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState, VehiclePose2D
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        root = Path(__file__).resolve().parents[1]
        output = '/cooperative_localization/rte/coordinates/smoke/poses_2d'
        rclpy.init()
        node = rclpy.create_node('rte_coordinates_isolated_test')
        received = []
        node.create_subscription(CooperativePose2DArray, output, received.append, 10)
        poses = node.create_publisher(CooperativePose2DArray, '/coordinate_test/input_poses', 10)
        states = node.create_publisher(NodeState, '/coordinate_test/input_states', qos_profile_sensor_data)
        with tempfile.TemporaryFile() as log:
            process = subprocess.Popen([sys.executable, str(root/'tools/rte_coordinate_viewer.py'),
                '--port', str(port), '--poses-topic', '/coordinate_test/input_poses',
                '--states-topic', '/coordinate_test/input_states', '--output-topic', output],
                stdout=log, stderr=log)
            try:
                deadline = time.monotonic()+12
                while time.monotonic() < deadline and poses.get_subscription_count() == 0:
                    rclpy.spin_once(node, timeout_sec=.05)
                self.assertGreater(poses.get_subscription_count(), 0)
                def publish(t):
                    state = NodeState()
                    state.node_id, state.valid = 1, True
                    state.header.frame_id = 'rte_odom_1'
                    state.header.stamp.sec, state.header.stamp.nanosec = divmod(t, 1_000_000_000)
                    state.position_enu_m = [10., 20., 0.]
                    states.publish(state)
                    # RTE共同历元应落在已缓存参考流中；分开spin模拟实际异步话题。
                    rclpy.spin_once(node, timeout_sec=.02)
                    msg = CooperativePose2DArray()
                    msg.header = copy.deepcopy(state.header)
                    msg.header.frame_id = 'coop_rte_1_local'
                    msg.reference_node_id = 1
                    for v in pose(t).vehicles:
                        item = VehiclePose2D()
                        for field in ['node_id', 'x_m', 'y_m', 'yaw_rad', 'position_valid', 'yaw_valid']:
                            setattr(item, field, getattr(v, field))
                        msg.vehicles.append(item)
                    poses.publish(msg)
                deadline = time.monotonic()+8
                tick = 0
                while time.monotonic() < deadline:
                    publish(10_000_000_000+tick*100_000_000)
                    tick += 1
                    rclpy.spin_once(node, timeout_sec=.05)
                    if received and all(v.position_valid for v in received[-1].vehicles):
                        break
                self.assertTrue(received and all(v.position_valid for v in received[-1].vehicles))
                good = received[-1]
                self.assertEqual(good.header.frame_id, 'coop_rte_1_relative_fixed')
                self.assertAlmostEqual(good.vehicles[1].x_m, 5.)
                self.assertAlmostEqual(good.vehicles[1].y_m, 0., places=8)
                for topic in ['/cooperative_localization/poses_2d', '/cooperative_localization/rte/poses_2d']:
                    self.assertEqual(node.count_publishers(topic), 0)
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state') as response:
                    self.assertTrue(json.load(response)['valid'])
                # 停止输入；新话题应继续发无效状态，不能保留true或伪造新Header。
                deadline = time.monotonic()+1.2
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.05)
                self.assertFalse(any(v.position_valid for v in received[-1].vehicles))
                self.assertEqual(received[-1].header.stamp, good.header.stamp)
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state') as response:
                    self.assertFalse(json.load(response)['valid'])
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                node.destroy_node()
                rclpy.shutdown()
                log.seek(0)
                print(log.read().decode())


if __name__ == '__main__':
    unittest.main()

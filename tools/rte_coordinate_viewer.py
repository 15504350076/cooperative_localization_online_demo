#!/usr/bin/env python3
"""独立RTE坐标适配器与只读网页；--demo不连接ROS，不改变旧节点或配置。"""
import argparse
from collections import deque
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import unquote

PAGE = Path(__file__).resolve().parents[1] / 'docs/visualization/rte_coordinates.html'
POSE_TOPIC = '/cooperative_localization/rte/poses_2d'
STATE_TOPIC = '/cooperative_localization/rte/node_state'
OUTPUT_TOPIC = '/cooperative_localization/rte/coordinates/poses_2d'


def rotate(point, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [c * point[0] - s * point[1], s * point[0] + c * point[1]]


def stamp_ns(header):
    sec, nano = header.stamp.sec, header.stamp.nanosec
    if sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError('Header时间戳不合法')
    value = sec * 1_000_000_000 + nano
    if value <= 0:
        raise ValueError('Header时间戳为零')
    return value


class Coordinates:
    """所有计算只做二维刚体坐标变换，不求解/修正RTE；调用方负责串行访问。"""
    def __init__(self, ids=(1, 2, 3), timeout=.5):
        self.ids = tuple(ids)
        self.timeout = timeout
        self.reference = deque(maxlen=500)
        self.anchor = None
        self.last_stamp = 0
        self.last_progress = None
        self.blocked = False
        self.reason = '等待RTE有效位置和同历元参考车NodeState'
        self.latest = None
        self.pending = None

    def reject(self, reason, blocked=False):
        self.reason = reason
        self.blocked = self.blocked or blocked
        if self.latest:
            self.latest['valid'] = False
        return False

    def push_reference(self, message):
        if message.node_id != self.ids[0]:
            return False
        try:
            if message.header.frame_id != f'rte_odom_{self.ids[0]}' or not message.valid:
                raise ValueError('参考车NodeState无效或不是预期rte_odom局部系')
            t = stamp_ns(message.header)
            p = list(message.position_enu_m[:2])
            if not all(math.isfinite(x) for x in p):
                raise ValueError('参考车位置不是有限数')
            if self.reference and t < self.reference[-1][0]:
                return self.reject('参考车Header回退：请协调重启RTE及本适配器', True)
            if self.reference and t == self.reference[-1][0]:
                if p != self.reference[-1][1]:
                    return self.reject('相同参考车Header对应不同位置：请重启本次会话', True)
                return False
            self.reference.append((t, p))
            if self.pending is not None and stamp_ns(self.pending.header) <= t:
                self.ingest(self.pending)
            return True
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            return self.reject(str(error))

    def reference_at(self, t):
        # 只接受同Header或两侧包围的线性插值，不用录包/接收时间配对，不外推。
        for index, (right, p) in enumerate(self.reference):
            if right == t:
                return p
            if right > t and index:
                left, q = self.reference[index - 1]
                if left <= t and right - left <= 100_000_000:
                    ratio = (t - left) / (right - left)
                    return [a + ratio * (b - a) for a, b in zip(q, p)]
                break
        raise ValueError('找不到同历元参考位置：需同Header或间隔≤100毫秒的包围样本')

    def ingest(self, message, now=None):
        now = time.monotonic() if now is None else now
        if self.blocked:
            return False
        self.pending = None
        try:
            if (message.reference_node_id != self.ids[0] or
                    message.header.frame_id != f'coop_rte_{self.ids[0]}_local'):
                raise ValueError('仅接收本参考车的RTE局部坐标，拒绝旧UWB/ENU/map混入')
            t = stamp_ns(message.header)
            if t < self.last_stamp:
                return self.reject('RTE Header回退：请协调重启RTE及本适配器', True)
            if t == self.last_stamp and self.latest and int(self.latest['stamp_ns']) == t:
                # 重复Header不刷新新鲜度；仍立即传递上游失效标志。
                if (len(message.vehicles) != 3 or
                        {v.node_id for v in message.vehicles} != set(self.ids) or
                        not all(v.position_valid for v in message.vehicles)):
                    return self.reject('上游RTE位置无效')
                if self.latest:
                    invalid_yaws = {v.node_id for v in message.vehicles if not v.yaw_valid}
                    for key in ('source', 'relative', 'task'):
                        for v in self.latest[key]:
                            if v['id'] in invalid_yaws:
                                v['yaw_valid'] = False
                return False
            self.last_stamp = t
            items = {v.node_id: v for v in message.vehicles}
            if len(items) != 3 or len(message.vehicles) != 3 or set(items) != set(self.ids):
                raise ValueError('需要配置的三个唯一逻辑车辆编号')
            vehicles = []
            for i in self.ids:
                v = items[i]
                if not v.position_valid:
                    raise ValueError('上游RTE位置无效；不把未初始化结果显示成定位成功')
                values = [v.x_m, v.y_m, v.yaw_rad]
                if not all(math.isfinite(x) for x in values):
                    raise ValueError('RTE位姿包含非有限数')
                vehicles.append(dict(id=i, x=v.x_m, y=v.y_m, yaw=v.yaw_rad,
                                     position_valid=True, yaw_valid=bool(v.yaw_valid)))
            if math.hypot(vehicles[0]['x'], vehicles[0]['y']) > 1e-4:
                raise ValueError('上游参考车不在相对原点')
            # 最多暂存一条已校验的最新结果；参考话题晚到时重试，不依赖到达顺序。
            self.pending = copy.deepcopy(message)
            host = self.reference_at(t)
            self.pending = None
            if self.anchor is None:
                second = vehicles[1]
                if math.hypot(second['x'], second['y']) < .2:
                    raise ValueError('初始参考车到第二车的基线不足0.2米，无法稳定定义X轴')
                self.anchor = dict(stamp_ns=str(t), host=list(host),
                                   beta=math.atan2(second['y'], second['x']))
            beta = self.anchor['beta']
            relative = []
            task = []
            for v in vehicles:
                x, y = rotate([v['x'], v['y']], -beta)
                yaw = math.atan2(math.sin(v['yaw'] - beta), math.cos(v['yaw'] - beta))
                relative.append(dict(v, x=x, y=y, yaw=yaw))
                xy = rotate([v['x'] + host[0] - self.anchor['host'][0],
                             v['y'] + host[1] - self.anchor['host'][1]], -beta)
                task.append(dict(v, x=xy[0], y=xy[1], yaw=yaw))
            # 仅比较几何规范化效果；此数组不是旧UWB节点实际输出。
            second = vehicles[1]
            angle = math.atan2(second['y'], second['x'])
            triangle = []
            third_y = rotate([vehicles[2]['x'], vehicles[2]['y']], -angle)[1]
            for v in vehicles:
                x, y = rotate([v['x'], v['y']], -angle)
                triangle.append(dict(v, x=x, y=y * (-1 if third_y < 0 else 1), yaw_valid=False))
            self.latest = dict(valid=True, stamp_ns=str(t), anchor=copy.deepcopy(self.anchor),
                               relative_frame=f'coop_rte_{self.ids[0]}_relative_fixed',
                               task_frame=f'coop_rte_{self.ids[0]}_task',
                               source=vehicles, relative=relative, task=task, triangle=triangle)
            self.last_progress = now
            self.reason = '坐标变换有效；精度依据独立测试结果读取'
            return True
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            return self.reject(str(error))

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        result = copy.deepcopy(self.latest) if self.latest else dict(valid=False)
        age = None if self.last_progress is None else max(0, now - self.last_progress)
        if age is not None and age > self.timeout:
            result['valid'] = False
        result.update(live=True, reason=self.reason, age_s=age, blocked=self.blocked)
        if age is not None and age > self.timeout and not self.blocked:
            result['reason'] = '有效Header未推进或输入断流，已撤销新坐标有效位'
        return result


def serve_page(bind, port, snapshot):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/api/state':
                body = json.dumps(snapshot(), ensure_ascii=False, allow_nan=False).encode()
                mime = 'application/json; charset=utf-8'
            elif self.path in ('/', '/rte_coordinates.html'):
                body, mime = PAGE.read_bytes(), 'text/html; charset=utf-8'
            elif unquote(self.path) == '/参考/46_RTE坐标系与独立可视化.md':
                body = (PAGE.parent.parent/'参考/46_RTE坐标系与独立可视化.md').read_bytes()
                mime = 'text/plain; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass
    return ThreadingHTTPServer((bind, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='纯几何动画示意；不运行RTE或发布ROS消息')
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', default=8766, type=int)
    parser.add_argument('--ids', default='1,2,3', help='参考车、第二车、第三车的逻辑编号')
    parser.add_argument('--poses-topic', default=POSE_TOPIC)
    parser.add_argument('--states-topic', default=STATE_TOPIC)
    parser.add_argument('--output-topic', default=OUTPUT_TOPIC)
    args = parser.parse_args()
    try:
        ids = tuple(int(i) for i in args.ids.split(','))
        if len(ids) != 3 or len(set(ids)) != 3 or min(ids) <= 0 or max(ids) > 65535:
            raise ValueError('需要三个唯一的1..65535逻辑编号')
        if not 0 <= args.port <= 65535:
            raise ValueError('端口必须在0..65535')
        if args.output_topic in (args.poses_topic, POSE_TOPIC,
                                 '/cooperative_localization/poses_2d'):
            raise ValueError('输出禁止覆盖原UWB或原RTE话题')
        if not args.output_topic.startswith('/cooperative_localization/rte/coordinates/'):
            raise ValueError('新输出必须位于/cooperative_localization/rte/coordinates/命名空间')
    except ValueError as error:
        parser.error(str(error))
    state = Coordinates(ids)
    lock = threading.Lock()
    node = None
    def snapshot():
        with lock:
            return state.snapshot() if not args.demo else dict(live=False)
    if not args.demo:
        try:
            import rclpy
            from cooperative_localization_msgs.msg import CooperativePose2DArray, NodeState, VehiclePose2D
            from rclpy.qos import qos_profile_sensor_data
        except ImportError as error:
            parser.error(f'请先source ROS 2和本工程消息工作空间：{error}')
        rclpy.init()
        node = rclpy.create_node('zju_rte_coordinates_viewer')
        publisher = node.create_publisher(CooperativePose2DArray, args.output_topic, 10)
        def reference(message):
            with lock:
                state.push_reference(message)
        def pose(message):
            with lock:
                state.ingest(message)
        def publish():
            view = snapshot()
            out = CooperativePose2DArray()
            ns = int(view.get('stamp_ns', '0'))
            out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            out.header.frame_id = f'coop_rte_{ids[0]}_relative_fixed'
            out.reference_node_id = ids[0]
            for v in view.get('relative', [dict(id=i, x=0., y=0., yaw=0., yaw_valid=False) for i in ids]):
                item = VehiclePose2D()
                item.node_id, item.x_m, item.y_m, item.yaw_rad = v['id'], v['x'], v['y'], v['yaw']
                item.position_valid = bool(view['valid'])
                item.yaw_valid = bool(view['valid'] and v['yaw_valid'])
                out.vehicles.append(item)
            publisher.publish(out)
        node.create_subscription(NodeState, args.states_topic, reference, qos_profile_sensor_data)
        node.create_subscription(CooperativePose2DArray, args.poses_topic, pose, 10)
        node.create_timer(.05, publish)
    server = serve_page(args.bind, args.port, snapshot)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print(f'RTE坐标展示：http://{args.bind}:{server.server_port}；模式：' +
          ('几何示意（无ROS）' if args.demo else f'ROS订阅；新输出{args.output_topic}'), flush=True)
    try:
        if node is None:
            while True:
                time.sleep(.2)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
        if node:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()

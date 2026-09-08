#!/usr/bin/env python3
"""电脑端一键录包（Python 3.9+）；只连接已启动传感器的盒子，不启动或停止驱动。"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile
import time


ROOT = Path(__file__).resolve().parents[1]
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8']
ENV = 'source /opt/ros/humble/setup.bash\nsource /home/firefly/imu_ws/install/setup.bash\nexport ROS_DOMAIN_ID=107 RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n'


def seconds_text(ns):
    seconds, nanos = divmod(ns, 10**9)
    return f'{seconds}.{nanos:09d}'


def remote_command(bag, topics, duration):
    """独立进程组内有限时录制；SIGINT落盘后才压缩，断线仍有时间上限。"""
    local = 1 if topics == ['/imu/raw'] else 0
    return ENV + f'''export ROS_LOCALHOST_ONLY={local}
test ! -e {shlex.quote(bag)} || exit 2
mkdir -p /home/firefly/time_test
timeout --foreground --signal=INT --kill-after=10s {duration}s ros2 bag record -o {shlex.quote(bag)} {' '.join(topics)}
rc=$?
if [ "$rc" != 0 ] && [ "$rc" != 124 ]; then exit "$rc"; fi
test -f {shlex.quote(bag + '/metadata.yaml')} || exit 3
ros2 bag info {shlex.quote(bag)}
tar -C /home/firefly/time_test -czf {shlex.quote(bag + '.tar.gz')} {shlex.quote(Path(bag).name)}
'''


def unpack(archive, destination):
    with tarfile.open(archive) as tar:
        for item in tar.getmembers():
            target = (destination / item.name).resolve()
            if not target.is_relative_to(destination.resolve()) or not (item.isfile() or item.isdir()):
                raise ValueError(f'非法压缩包路径或类型：{item.name}')
        tar.extractall(destination)


def analyze(directory):
    # 复用ROS官方反序列化，电脑需source含cooperative_interfaces的工作空间。
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows = []
    for bag in sorted(directory.iterdir()):
        if not (bag / 'metadata.yaml').exists():
            continue
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        streams = {}
        while reader.has_next():
            topic, data, received = reader.read_next()
            if topic not in ('/imu/raw', '/uwb/range'):
                continue
            msg = deserialize_message(data, types[topic])
            key = topic if topic == '/imu/raw' else f'/uwb/range {msg.src_id}→{msg.target_id}'
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            s = streams.setdefault(key, dict(count=0, first=stamp, last=stamp,
                                            zero=0, rollback=0, duplicate=0))
            if s['count']:
                s['rollback'] += stamp < s['last']
                s['duplicate'] += stamp == s['last']
            s['zero'] += stamp == 0
            s['last'] = stamp
            s['count'] += 1
        for key, s in streams.items():
            rows.append(dict(bag=bag.name, stream=key, **s))
    (directory / '时间戳统计.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# 联合时间初查', '', '本报告只检查消息时间范围及连续性，不证明传感器同步精度。', '',
             '| 包 / 数据流 | 数量 | 首消息时间(s) | 末消息时间(s) | 零值 / 回退 / 重复 |',
             '|---|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['bag']} / {r['stream']} | {r['count']} | {seconds_text(r['first'])} | {seconds_text(r['last'])} | {r['zero']} / {r['rollback']} / {r['duplicate']} |")
    overlap = (min(r['last'] for r in rows) - max(r['first'] for r in rows)) / 1e9 if rows else 0
    lines += ['', f'各已收到数据流的首末消息区间交集长度：{max(0, overlap):.6f} 秒。', '',
              '区间无交集：检查录制是否覆盖共同过程、各盒epoch及授时源。区间有交集：仍需核对动作事件及PPS诊断。',
              '零值、回退、重复按每条有向流统计；首末区间在发生回退时不能作为连续覆盖证明。',
              '必须结合manifest.json检查包是否齐全、UWB三边是否齐全。缺失数据流不会出现在表中。',
              '不能把各包首帧分别置零后判断同步；不同盒子的rosbag接收时间也未必同源。']
    (directory / '联合时间初查.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f"时间报告：{directory / '联合时间初查.md'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--boxes', nargs='+', default=['1=192.168.1.201', '2=192.168.1.202', '3=192.168.1.203'])
    parser.add_argument('--uwb-recorder', default='3', help='共享UWB录包盒号')
    parser.add_argument('--user', default='firefly')
    parser.add_argument('--run-id', default=datetime.now().strftime('joint_time_%Y%m%d_%H%M%S'))
    parser.add_argument('--duration', type=int, default=180, help='每路录包时长，秒，至少150')
    parser.add_argument('--analyze', type=Path, help='仅分析已下载的实验目录，需要电脑ROS环境')
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze.resolve())
        return
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id) or args.duration < 150:
        parser.error('实验编号只能用字母数字下划线横线；录制至少150秒')
    boxes = {}
    for spec in args.boxes:
        if not re.fullmatch(r'[1-9][0-9]*=[A-Za-z0-9.]+', spec):
            parser.error('盒子格式：1=192.168.1.201')
        vehicle, host = spec.split('=')
        if vehicle in boxes:
            parser.error('盒号重复')
        boxes[vehicle] = host
    if args.uwb_recorder not in boxes or not re.fullmatch(r'[A-Za-z0-9_-]+', args.user):
        parser.error('UWB录包盒须在boxes中；检查用户名')
    if args.duration < 90 + 20 * len(boxes):
        parser.error(f'本轮至少录制{90 + 20 * len(boxes)}秒，给所有盒子的动作留出时间')
    directory = ROOT / 'data' / args.run_id
    directory.mkdir(parents=True, exist_ok=False)
    jobs = [(v, ['/imu/raw'], f'{args.run_id}_imu_rec_v{v}') for v in boxes]
    jobs.append((args.uwb_recorder, ['/uwb/range', '/node/id'],
                 f'{args.run_id}_uwb_rec_v{args.uwb_recorder}'))
    (directory / 'manifest.json').write_text(json.dumps(vars(args) | {'analyze': None}, ensure_ascii=False, indent=2), encoding='utf-8')
    (directory / '实验备注.txt').write_text('请填写实际动作、初始摆放、是否重启、异常情况。\n电脑动作口令采用单调计时，不是传感器时钟真值。\n', encoding='utf-8')

    def preflight(item):
        vehicle, host = item
        command = ENV + "command -v timeout >/dev/null\npgrep -f '^/home/firefly/imu_ws/install/navcore/lib/navcore/navcore( |$)' >/dev/null && pgrep -f '^/home/firefly/imu_ws/install/gnss_uwb/lib/gnss_uwb/gnss_uwb( |$)' >/dev/null"
        subprocess.run(SSH + ['-T', f'{args.user}@{host}', 'bash -c ' + shlex.quote(command)],
                       check=True, timeout=20, stdin=subprocess.DEVNULL)
        print(f'盒{vehicle}：SSH和驱动进程检查完成', flush=True)

    with ThreadPoolExecutor(max_workers=len(boxes)) as pool:
        list(pool.map(preflight, boxes.items()))
    processes = []
    for vehicle, topics, name in jobs:
        logfile = directory / f'{name}.log'
        remote = f'/home/firefly/time_test/{name}'
        with logfile.open('wb') as output:
            proc = subprocess.Popen(SSH + ['-tt', f'{args.user}@{boxes[vehicle]}',
                                          'bash -c ' + shlex.quote(remote_command(remote, topics, args.duration))],
                                    stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT)
        processes.append((proc, logfile, vehicle, remote, topics))
    print('等待所有录包订阅就绪，暂时保持静止……', flush=True)
    deadline = time.monotonic() + 40
    ready = False
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p, *_ in processes):
            break
        ready = all(all(f"Subscribed to topic '{t}'" in log.read_text(encoding='utf-8', errors='replace') for t in topics)
                    for _, log, _, _, topics in processes)
        if ready:
            break
        time.sleep(0.5)
    if ready:
        print('全部就绪，现在开始动作计时。', flush=True)
        start = time.monotonic()
        events = [(0, '全部静止')]
        for index, vehicle in enumerate(boxes):
            events += [(15 + 20 * index, f'只平移盒{vehicle}约0.5～1米'),
                       (25 + 20 * index, '全部静止')]
        events.append((30 + 20 * len(boxes), '动作结束，继续静止等待自动收包'))
        for at, instruction in events:
            time.sleep(max(0, start + at - time.monotonic()))
            print(f'{at:3d}s：{instruction}', flush=True)
    else:
        print('40秒内未全部就绪，本轮只作诊断，不执行动作；等待有限时录包结束。', flush=True)
    failures = []
    for proc, logfile, vehicle, remote, topics in processes:
        try:
            rc = proc.wait(timeout=args.duration + 60)
            if rc:
                raise RuntimeError(f'SSH/录包退出码{rc}')
            archive = directory / (Path(remote).name + '.tar.gz')
            subprocess.run(['scp', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                            f'{args.user}@{boxes[vehicle]}:{remote}.tar.gz', str(archive)], check=True, timeout=180)
            unpack(archive, directory)
            print(f'已收回：{archive.name}', flush=True)
        except (subprocess.SubprocessError, RuntimeError, OSError, ValueError) as exc:
            failures.append(f'{logfile.name}: {exc}；远端保留在{remote}')
    (directory / '采集状态.json').write_text(json.dumps({'all_subscribed': ready, 'errors': failures}, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'数据和日志：{directory}')
    print('同步尚未判定；加载电脑ROS环境后执行：')
    print(f'python3 tools/record_joint_time.py --analyze {shlex.quote(str(directory))}')
    if failures or not ready:
        raise SystemExit('\n'.join(failures) or '本轮就绪检查失败，请查日志')


if __name__ == '__main__':
    main()

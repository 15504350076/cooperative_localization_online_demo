#!/usr/bin/env python3
"""电脑端一键录包（Python 3.9+）；只连接已启动传感器的盒子，不启动或停止驱动。"""
# 本文件有采集和离线分析两条路径；ROS依赖放在analyze内部，Windows录包不需ROS。
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


ROOT = Path(__file__).resolve().parents[1]  # tools上一级；不依赖启动命令所在目录。
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8']


def ros_env(domain_id, rmw):
    """生成录包终端环境；域号可与上交当前运行进程保持一致。"""
    return ('source /opt/ros/humble/setup.bash\n'
            'source /home/firefly/imu_ws/install/setup.bash\n'
            f'export ROS_DOMAIN_ID={domain_id} RMW_IMPLEMENTATION={rmw}\n')


def seconds_text(ns):
    # 整数纳秒格式化为sec.nanosec，固定9位小数，保留原始精度。
    seconds, nanos = divmod(ns, 10**9)
    return f'{seconds}.{nanos:09d}'


def remote_command(bag, topics, duration, environment):
    """独立进程组内有限时录制；SIGINT落盘后才压缩，断线仍有时间上限。"""
    # IMU仅收本机，UWB允许共享DDS；这些设置不改变已启动驱动的环境。
    local = 1 if topics == ['/imu/raw'] else 0
    # 引号保护远程路径；timeout的124表示到时结束，其他非零状态不压缩。
    return environment + f'''export ROS_LOCALHOST_ONLY={local}
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
    # 解压前逐项检查目录边界及类型，拒绝路径穿越与符号链接。
    with tarfile.open(archive) as tar:
        for item in tar.getmembers():
            target = (destination / item.name).resolve()
            if not target.is_relative_to(destination.resolve()) or not (item.isfile() or item.isdir()):
                raise ValueError(f'非法压缩包路径或类型：{item.name}')
        tar.extractall(destination)


def analyze(directory):
    # 离线入口：逐包、逐有向流统计header，输出JSON及Markdown，不连接SSH。
    # 复用ROS官方反序列化，电脑需source含cooperative_interfaces的工作空间。
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    rows = []
    for bag in sorted(directory.iterdir()):
        if not (bag / 'metadata.yaml').exists():
            continue
        # 只处理直接子目录中的sqlite3包；保持读取顺序，不按header排序。
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        # 从bag记录的类型名加载消息类，自定义接口必须已经构建并source。
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        streams = {}
        while reader.has_next():
            topic, data, received = reader.read_next()
            if topic not in ('/imu/raw', '/uwb/range'):
                continue
            msg = deserialize_message(data, types[topic])
            # 一个包内的IMU为一条流；UWB拆成有向流，避免交错时间造成假回退。
            key = topic if topic == '/imu/raw' else f'/uwb/range {msg.src_id}→{msg.target_id}'
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            # first只在首次出现时赋值；last随读取更新，均不是全局min/max。
            s = streams.setdefault(key, dict(count=0, first=stamp, last=stamp,
                                            zero=0, rollback=0, duplicate=0))
            if s['count']:  # 第一帧无前帧，不作回退/重复比较。
                s['rollback'] += stamp < s['last']
                s['duplicate'] += stamp == s['last']
            s['zero'] += stamp == 0
            s['last'] = stamp
            s['count'] += 1
        for key, s in streams.items():
            rows.append(dict(bag=bag.name, stream=key, **s))
    # 数值JSON供后续处理，中文Markdown供人查看；同名派生报告会更新。
    (directory / '时间戳统计.json').write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding='utf-8')
    lines = ['# 联合时间初查', '', '本报告只检查消息时间范围及连续性，不证明传感器同步精度。', '',
             '| 包 / 数据流 | 数量 | 首消息时间(s) | 末消息时间(s) | 零值 / 回退 / 重复 |',
             '|---|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['bag']} / {r['stream']} | {r['count']} | {seconds_text(r['first'])} | {seconds_text(r['last'])} | {r['zero']} / {r['rollback']} / {r['duplicate']} |")
    # 区间交集=最早结束-最晚开始，负值输出0；缺失流未参与，回退会削弱意义。
    # 非零交集仅是必要初查，不是采样同步证明。
    overlap = (min(r['last'] for r in rows) - max(r['first'] for r in rows)) / 1e9 if rows else 0
    lines += ['', f'各已收到数据流的首末消息区间交集长度：{max(0, overlap):.6f} 秒。', '',
              '区间无交集：检查录制是否覆盖共同过程、各盒epoch及授时源。区间有交集：仍需核对动作事件及PPS诊断。',
              '零值、回退、重复按每条有向流统计；首末区间在发生回退时不能作为连续覆盖证明。',
              '必须结合manifest.json检查包是否齐全、UWB三边是否齐全。缺失数据流不会出现在表中。',
              '不能把各包首帧分别置零后判断同步；不同盒子的rosbag接收时间也未必同源。']
    (directory / '联合时间初查.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(f"时间报告：{directory / '联合时间初查.md'}")


def main():
    # 参数调度：--analyze提前返回，下面的SSH、录包、下载完全不会执行。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--boxes', nargs='+', default=['1=192.168.1.201', '2=192.168.1.202', '3=192.168.1.203'])
    parser.add_argument('--uwb-recorder', default='1', help='共享UWB录包盒号，默认1（201参考车）')
    parser.add_argument('--user', default='firefly')
    parser.add_argument('--domain-id', type=int, default=107,
                        help='ROS通信域，必须与已有驱动一致，默认107')
    parser.add_argument('--rmw', default='rmw_fastrtps_cpp',
                        help='ROS中间件实现，必须与已有驱动一致')
    parser.add_argument('--run-id', default=datetime.now().strftime('joint_time_%Y%m%d_%H%M%S'))
    parser.add_argument('--duration', type=int, default=180, help='每路录包时长，秒，至少150')
    parser.add_argument('--analyze', type=Path, help='仅分析已下载的实验目录，需要电脑ROS环境')
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze.resolve())
        return
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id) or args.duration < 150:
        parser.error('实验编号只能用字母数字下划线横线；录制至少150秒')
    if not 0 <= args.domain_id <= 232 or not re.fullmatch(r'[A-Za-z0-9_]+', args.rmw):
        parser.error('ROS domain须为0～232；RMW名称只能含字母、数字和下划线')
    environment = ros_env(args.domain_id, args.rmw)
    # 采集路径：解析盒号与地址，校验重复及字符，避免远程命令歧义。
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
    # N个本机IMU包+1个共享UWB包；rec_v标记记录盒，不过滤UWB测距来源。
    jobs = [(v, ['/imu/raw'], f'{args.run_id}_imu_rec_v{v}') for v in boxes]
    jobs.append((args.uwb_recorder, ['/uwb/range', '/node/id'],
                 f'{args.run_id}_uwb_rec_v{args.uwb_recorder}'))
    (directory / 'manifest.json').write_text(json.dumps(vars(args) | {'analyze': None}, ensure_ascii=False, indent=2), encoding='utf-8')
    (directory / '实验备注.txt').write_text('请填写实际动作、初始摆放、是否重启、异常情况。\n电脑动作口令采用单调计时，不是传感器时钟真值。\n', encoding='utf-8')

    def preflight(item):
        # 只查SSH、timeout与驱动进程存在，不保证消息质量及驱动域号正确。
        vehicle, host = item
        command = environment + "command -v timeout >/dev/null\npgrep -f '^/home/firefly/imu_ws/install/navcore/lib/navcore/navcore( |$)' >/dev/null && pgrep -f '^/home/firefly/imu_ws/install/gnss_uwb/lib/gnss_uwb/gnss_uwb( |$)' >/dev/null"
        subprocess.run(SSH + ['-T', f'{args.user}@{host}', 'bash -c ' + shlex.quote(command)],
                       check=True, timeout=20, stdin=subprocess.DEVNULL)
        print(f'盒{vehicle}：SSH和驱动进程检查完成', flush=True)

    # 并行预检减少等待；每路采集独立启动，不提供硬件同时触发。
    with ThreadPoolExecutor(max_workers=len(boxes)) as pool:
        list(pool.map(preflight, boxes.items()))
    processes = []
    for vehicle, topics, name in jobs:
        logfile = directory / f'{name}.log'
        remote = f'/home/firefly/time_test/{name}'
        with logfile.open('wb') as output:
            proc = subprocess.Popen(SSH + ['-tt', f'{args.user}@{boxes[vehicle]}',
                                          'bash -c ' + shlex.quote(remote_command(
                                              remote, topics, args.duration, environment))],
                                    stdin=subprocess.PIPE, stdout=output, stderr=subprocess.STDOUT)
        processes.append((proc, logfile, vehicle, remote, topics))
    print('等待所有录包订阅就绪，暂时保持静止……', flush=True)
    # 用单调钟控制等待和口令时间，不受电脑日历校时影响，也非传感器真值。
    deadline = time.monotonic() + 40
    ready = False
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p, *_ in processes):
            break
        # 订阅日志仅表示已建立订阅，不保证持续收到数据、三边齐全或时钟同步。
        ready = all(all(f"Subscribed to topic '{t}'" in log.read_text(encoding='utf-8', errors='replace') for t in topics)
                    for _, log, _, _, topics in processes)
        if ready:
            break
        time.sleep(0.5)
    if ready:
        print(f'全部就绪，正在录制{args.duration}秒。请在录制期间只移动一个指定盒子，'
              '完成后保持静止，并把盒号和大致动作时间写入实验备注。', flush=True)
    else:
        print('40秒内未全部就绪，本轮只作诊断，不执行动作；等待有限时录包结束。', flush=True)
    # 逐路等结束、下载并解压；失败记录到采集状态，远端数据仍保留。
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

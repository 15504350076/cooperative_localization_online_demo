"""本地检查录包和21号雷达按需启动命令，不连接实盒：python3 tools/test_record_joint_time.py。"""
import io
import argparse
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
from unittest.mock import patch
from record_joint_time import main, remote_command, ros_env, unpack, seconds_text
from record_201_gnss_imu import ros_env as gnss_imu_env
from record_box_uwb_imu_livox import env_for as uwb_imu_livox_env

# 走离线分支验证真实CLI默认值，不连接SSH，也不生成分析文件。
parse_args = argparse.ArgumentParser.parse_args

def parse_offline(parser):
    args = parse_args(parser, ['--analyze', '.'])
    assert args.domain_id == 0
    return args

with patch.dict(os.environ, {'ROS_DOMAIN_ID': '107'}), \
        patch('argparse.ArgumentParser.parse_args', parse_offline), \
        patch('record_joint_time.analyze'):
    main()

for stale_domain in (None, '107'):
    with patch.dict(os.environ, {} if stale_domain is None else {'ROS_DOMAIN_ID': stale_domain}, clear=True):
        for local in ('0', '1'):
            assert gnss_imu_env(local)['ROS_DOMAIN_ID'] == '0'
            assert uwb_imu_livox_env(local)['ROS_DOMAIN_ID'] == '0'

assert seconds_text(1785547399180000000) == '1785547399.180000000'

# 最小在线入口必须明确设域0，不能只依赖父终端环境。
root = Path(__file__).resolve().parents[1]
launch_source = (root / 'ros2/zju_coop_bringup/launch/uwb_only.launch.py').read_text(encoding='utf-8')
assert 'DeclareLaunchArgument("domain_id", default_value="0")' in launch_source
assert 'SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("domain_id"))' in launch_source

# 直接验证文档中的启动判断；替换实际启动命令，绝不启动真实Livox驱动。
guide = Path(__file__).resolve().parents[1] / 'docs/21_上交盒端传感器节点启动与可用性检查_精简执行版.md'
lidar_step = guide.read_text(encoding='utf-8').split('### 6.1 雷达', 1)[1].split('```bash\n', 1)[1].split('\n```', 1)[0]
assert '/bin/bash /home/firefly/livox_ws/start_livox.sh' in lidar_step
pattern = lidar_step.split("pgrep --full '", 1)[1].split("'", 1)[0]
for command, expected in (
    ('/home/firefly/livox_ws/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node --ros-args', True),
    ('/usr/bin/python3 /opt/ros/humble/bin/ros2 launch livox_ros_driver2 msg_MID360s_launch.py', True),
    ('/bin/bash /home/firefly/livox_ws/start_livox.sh', True),
    (f'pgrep --full {pattern}', False),
    ('/home/firefly/imu_ws/install/navcore/lib/navcore/navcore', False),
):
    result = subprocess.run(['bash', '-c', '[[ "$1" =~ $2 ]]', 'test', command, pattern])
    assert (result.returncode == 0) == expected

fake_step = lidar_step.replace('/bin/bash /home/firefly/livox_ws/start_livox.sh', 'fake_start')
for present in (0, 1):  # pgrep返回0表示已有启动链，1表示未运行。
    shell = f'pgrep() {{ return {present}; }}\n' + 'fake_start() { printf "TEST_STARTED:%s\\n" "$ROS_DOMAIN_ID"; }\n'
    result = subprocess.run(['bash', '-c', shell + fake_step], text=True, capture_output=True, check=True)
    assert ('TEST_STARTED:' in result.stdout) == bool(present)
    assert ('跳过' in result.stdout) == (present == 0)

for domain, topics in ((0, ['/imu/raw']), (0, ['/uwb/range', '/node/id']),
                      (42, ['/uwb/range', '/node/id']),
                      (0, ['/uwb/range', '/node/id', '/cooperative_localization/poses_2d'])):
    command = remote_command(
        '/home/firefly/time_test/test_imu_rec_v1', topics, 180,
        ros_env(domain, 'rmw_fastrtps_cpp'),
    )
    subprocess.run(['bash', '-n'], input=command, text=True, check=True)
    assert 'timeout --foreground --signal=INT --kill-after=10s 180s' in command
    assert 'ros2 bag record --output ' in command
    assert 'tar --directory /home/firefly/time_test --create --gzip --file ' in command
    assert 'metadata.yaml' in command
    assert f'ROS_DOMAIN_ID={domain}' in command
    assert f'ROS_LOCALHOST_ONLY={1 if topics == ["/imu/raw"] else 0}' in command
    if '/cooperative_localization/poses_2d' in topics:
        assert '/cooperative_localization/poses_2d' in command
        assert 'source /home/firefly/zju_coop_ws/install/ros2-box/setup.bash' in command

with tempfile.TemporaryDirectory() as tmp:
    directory = Path(tmp)
    archive = directory / 'test.tar.gz'
    for name in ('bag/metadata.yaml', '../escape'):
        with tarfile.open(archive, 'w:gz') as tar:
            member = tarfile.TarInfo(name)
            member.size = 2
            tar.addfile(member, io.BytesIO(b'{}'))
        if name.startswith('bag/'):
            unpack(archive, directory)
            assert (directory / name).read_text() == '{}'
        else:
            try:
                unpack(archive, directory)
                raise AssertionError('路径穿越未被拒绝')
            except ValueError:
                pass
print('雷达按需启动、远端shell语法、录包隔离配置和解包检查通过；未连接实盒。')

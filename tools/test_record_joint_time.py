"""本地可运行检查，不连接实盒：python3 tools/test_record_joint_time.py。"""
import io
from pathlib import Path
import subprocess
import tarfile
import tempfile
from record_joint_time import remote_command, ros_env, unpack, seconds_text

assert seconds_text(1785547399180000000) == '1785547399.180000000'

for topics in (['/imu/raw'], ['/uwb/range', '/node/id']):
    command = remote_command(
        '/home/firefly/time_test/test_imu_rec_v1', topics, 180,
        ros_env(107, 'rmw_fastrtps_cpp'),
    )
    subprocess.run(['bash', '-n'], input=command, text=True, check=True)
    assert 'timeout --foreground --signal=INT --kill-after=10s 180s' in command
    assert 'metadata.yaml' in command
    assert f'ROS_LOCALHOST_ONLY={1 if topics == ["/imu/raw"] else 0}' in command

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
print('远端shell语法、录包隔离配置和解包检查通过；未连接实盒。')

#!/usr/bin/env python3
"""离线验证三盒录包放行协议和总目录压缩：不需要盒子或ROS。"""

from pathlib import Path
import subprocess
import tarfile
import tempfile
import time

from record_three_boxes import READY, command_for, pack_results


def main():
    with tempfile.TemporaryDirectory() as directory:
        worker = Path(directory) / "fake_recorder.py"
        worker.write_text("import sys\nprint('STARTED', *sys.argv[1:], flush=True)\n")
        processes = []
        try:
            for box in ("201", "202", "203"):
                command = command_for(box, worker, "common_run", 180)
                # 本测试只替换ROS环境加载，不替换真实bash等待/exec协议。
                command = "\n".join(line for line in command.splitlines()
                                    if not line.startswith("source "))
                process = subprocess.Popen(["bash", "-c", command], stdin=subprocess.PIPE,
                                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                           text=True)
                processes.append((box, process))
            for _, process in processes:
                assert process.stdout.readline().strip() == READY
            time.sleep(0.1)
            assert all(process.poll() is None for _, process in processes)
            for _, process in processes:
                process.stdin.write("GO\n")
                process.stdin.flush()
            for box, process in processes:
                output, errors = process.communicate(timeout=5)
                assert process.returncode == 0, errors
                assert f"--box-id {box} --run-id common_run --duration 180" in output, output
        finally:
            for _, process in processes:
                if process.poll() is None:
                    process.kill()
                process.wait()
        root = Path(directory) / "three_boxes_test"
        root.mkdir()
        for box in ("201", "202", "203"):
            (root / f"uwb_imu_livox_v{box}_test.tar.gz").write_bytes(b"sample bag")
            (root / f"box{box}.log").write_text(f"box{box} finished\n")
        archive = pack_results(root)
        assert archive == root.with_suffix(".tar.gz")
        assert len(list(root.iterdir())) == 6, "打包不应删除原文件"
        with tarfile.open(archive, "r:gz") as tar:
            names = set(tar.getnames())
            assert names == {root.name} | {f"{root.name}/{p.name}" for p in root.iterdir()}
            for path in root.iterdir():
                assert tar.extractfile(f"{root.name}/{path.name}").read() == path.read_bytes()
    print("三盒等待放行、统一传参及总文件夹压缩测试通过")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""在201盒一键并行录制三盒UWB、车载IMU、Livox，收回并打成一个总压缩包。

复用同目录的record_box_uwb_imu_livox.py，自动上传到202、203。
三盒加载环境并就绪后由201统一放行，不依赖各盒日历相等；这不是硬件同步触发。
要求201已能免交互SSH到192.168.2.202、192.168.2.203，且各盒驱动正在出数。
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
import re
import shlex
import subprocess
import tarfile
import time


HOSTS = {"201": None, "202": "firefly@192.168.2.202", "203": "firefly@192.168.2.203"}
SSH = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
REMOTE_SCRIPT = "/home/firefly/sensor_test/record_box_uwb_imu_livox.py"
READY = "THREE_BOX_READY"


def pack_results(root):
    """三盒文件收齐后打包整个实验目录，保留原目录和各盒原始数据。"""
    archive = root.with_suffix(".tar.gz")
    # 内层数据已压缩；外层用低压缩级别，避免重复压缩耗时过长。
    with tarfile.open(archive, "w:gz", compresslevel=1) as tar:
        tar.add(root, arcname=root.name)
    return archive


def command_for(box, worker, run_id, duration):
    """加载盒端环境，等待201放行，再启动原单盒录包程序。"""
    launch = shlex.join(["python3", "-u", str(worker), "--box-id", box,
                         "--run-id", run_id, "--duration", str(duration)])
    return ("set -e\nsource /opt/ros/humble/setup.bash\n"
            "source /home/firefly/imu_ws/install/setup.bash\n"
            "source /home/firefly/livox_ws/install/setup.bash\n"
            f"printf '{READY}\\n'\n"
            'read -r -t 60 trigger\n[ "$trigger" = GO ]\nexec ' + launch)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=180, help="录包秒数，默认180")
    parser.add_argument("--run-id", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    args = parser.parse_args()
    if args.duration <= 0 or not re.fullmatch(r"[A-Za-z0-9_-]+", args.run_id):
        parser.error("时长须大于0，实验编号只能含字母数字、下划线和横线")
    worker = Path(__file__).resolve().with_name("record_box_uwb_imu_livox.py")
    if not worker.is_file():
        parser.error("请将两个录包Python文件放在同一目录")
    root = Path("/home/firefly/sensor_test") / f"three_boxes_{args.run_id}"
    root.mkdir(parents=True, exist_ok=False)

    def prepare(host):
        subprocess.run(SSH + [host, "mkdir --parents /home/firefly/sensor_test"], check=True)
        subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                        str(worker), f"{host}:{REMOTE_SCRIPT}"], check=True)

    # 上传全部完成后再启动，避免传文件耗时挤占某盒的录制窗口。
    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(prepare, [HOSTS["202"], HOSTS["203"]]))

    processes = {}
    logs = {}
    try:
        for box, host in HOSTS.items():
            logs[box] = root / f"box{box}.log"
            command = command_for(box, REMOTE_SCRIPT if host else worker, args.run_id, args.duration)
            argv = SSH + [host, "bash -c " + shlex.quote(command)] if host else ["bash", "-c", command]
            with logs[box].open("wb") as output:
                processes[box] = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=output,
                                                   stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 40
        while not all(READY in log.read_text(errors="replace") for log in logs.values()):
            if any(p.poll() is not None for p in processes.values()) or time.monotonic() > deadline:
                raise RuntimeError(f"三盒未全部就绪，检查日志：{root}")
            time.sleep(0.1)
        for process in processes.values():
            process.stdin.write(b"GO\n")
            process.stdin.flush()
        print(f"三盒已统一放行，录制{args.duration:g}秒；前20秒保持静止。", flush=True)
        print(f"日志：{root}；录完后还需压缩和传输，请等待最终结果。", flush=True)

        def collect(item):
            box, process = item
            rc = process.wait()
            if rc:
                return f"盒{box}失败（退出码{rc}），查看{logs[box]}"
            archive = f"/home/firefly/sensor_test/uwb_imu_livox_v{box}_{args.run_id}.tar.gz"
            if HOSTS[box]:
                result = subprocess.run(["scp", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
                                         f"{HOSTS[box]}:{archive}", str(root)])
                if result.returncode:
                    return f"盒{box}下载失败，原包保留在该盒{archive}"
            else:
                Path(archive).rename(root / Path(archive).name)
            print(f"盒{box}压缩包已收齐", flush=True)
            return None

        with ThreadPoolExecutor(max_workers=3) as pool:
            failures = [result for result in pool.map(collect, processes.items()) if result]
        if failures:
            raise RuntimeError("\n".join(failures))
        print(f"三盒文件已收齐，正在打包总文件夹：{root}", flush=True)
        archive = pack_results(root)
        print(f"最终压缩包（只需下载这个文件）：{archive}", flush=True)
        print(f"原文件夹保留：{root}")
        print("录制完成不代表同步通过；请按31号文档分析Header及数据完整性。")
    finally:
        # 放行前EOF使等待中的shell退出；放行后不强杀SSH，有限时录包自行保存。
        for process in processes.values():
            if process.stdin:
                process.stdin.close()


if __name__ == "__main__":
    main()

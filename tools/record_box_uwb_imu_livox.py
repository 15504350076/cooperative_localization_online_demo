#!/usr/bin/env python3
"""在任一盒同时录制本盒 UWB、IMU 和 Livox。

三类数据使用不同的ROS 2发现范围，因此由三个rosbag2进程在同一时间窗口录制：
UWB使用跨盒环境，IMU和Livox使用本盒环境，分别写入uwb、imu、livox子目录。
脚本不会启动或停止驱动；结束后检查metadata并自动压缩。
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import os
import re
from pathlib import Path
import signal
import subprocess
import tarfile
import threading
import time


UWB_TOPICS = ["/uwb/range"]
IMU_TOPICS = ["/imu/raw"]
LIVOX_TOPICS = ["/livox/lidar", "/livox/imu"]


def livox_rmw() -> str:
    """读取已运行 Livox 驱动的 RMW，避免脚本替驱动猜配置。"""
    result = subprocess.run(
        ["pgrep", "--newest", "--full", "/livox_ros_driver2_node( |$)"],
        text=True,
        capture_output=True,
        check=False,
    )
    pid = result.stdout.strip()
    if not pid:
        return ""
    try:
        values = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
    except OSError:
        return ""
    prefix = b"RMW_IMPLEMENTATION="
    for value in values:
        if value.startswith(prefix):
            return value[len(prefix):].decode()
    return ""


def env_for(localhost_only: str, rmw: str | None = None) -> dict[str, str]:
    """复制当前终端环境，统一使用域0并设置本次录包所需的通信范围和RMW。"""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "0"
    env["ROS_LOCALHOST_ONLY"] = localhost_only
    if rmw:
        env["RMW_IMPLEMENTATION"] = rmw
    else:
        env.pop("RMW_IMPLEMENTATION", None)
    return env


def stop_process(process: subprocess.Popen[bytes] | None) -> None:
    """按 Ctrl+C 语义停止一个独立进程组，必要时再强制结束。"""
    if process is None or process.poll() is not None:
        return

    def wait_quietly(seconds: float) -> bool:
        """等待退出，但不要让再次到达的 Ctrl+C 打断清理流程。"""
        deadline = time.monotonic() + seconds
        while process.poll() is None and time.monotonic() < deadline:
            try:
                process.wait(timeout=min(0.2, max(0.0, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                continue
            except KeyboardInterrupt:
                # 终端的 Ctrl+C 可能同时打断 wait；继续清理，不再向上抛出。
                continue
        return process.poll() is not None

    try:
        os.killpg(process.pid, signal.SIGINT)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        return
    if wait_quietly(8):
        return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        if wait_quietly(3):
            return
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description="同时录制本盒 UWB、IMU、Livox")
    parser.add_argument("--box-id", default="201", choices=("201", "202", "203"),
                        help="本盒编号，默认201")
    parser.add_argument("--duration", type=float, default=10.0, help="录制秒数，默认10秒")
    parser.add_argument("--run-id", help="三盒共用的实验编号；省略则使用本盒日期")
    parser.add_argument("--output-dir", type=Path, default=Path("/home/firefly/sensor_test"))
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration 必须大于0")
    if args.run_id and not re.fullmatch(r"[A-Za-z0-9_-]+", args.run_id):
        parser.error("--run-id 只能包含字母、数字、下划线和横线")

    radar_rmw = livox_rmw()
    if not radar_rmw:
        print("错误：未找到正在运行的 Livox 驱动或其 RMW。先完成21.md的雷达启动和检查。")
        return 2

    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id or datetime.now().strftime("%Y%m%d_%H%M%S")
    root = args.output_dir / f"uwb_imu_livox_v{args.box_id}_{run_id}"
    uwb_bag = root / "uwb"
    imu_bag = root / "imu"
    livox_bag = root / "livox"
    root.mkdir()
    uwb_log = (root / "uwb_record.log").open("wb")
    imu_log = (root / "imu_record.log").open("wb")
    livox_log = (root / "livox_record.log").open("wb")

    commands = [
        (["ros2", "bag", "record", "--output", str(uwb_bag), *UWB_TOPICS],
         env_for("0", "rmw_fastrtps_cpp"), uwb_log),
        (["ros2", "bag", "record", "--output", str(imu_bag), *IMU_TOPICS],
         env_for("1", "rmw_fastrtps_cpp"), imu_log),
        (["ros2", "bag", "record", "--output", str(livox_bag), *LIVOX_TOPICS],
         env_for("1", radar_rmw), livox_log),
    ]
    processes: list[subprocess.Popen[bytes] | None] = [None, None, None]
    barrier = threading.Barrier(3)

    def launch(index: int) -> None:
        barrier.wait()
        command, environment, log = commands[index]
        processes[index] = subprocess.Popen(
            command,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    threads = [threading.Thread(target=launch, args=(index,)) for index in range(3)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    print(f"三路录包已启动（盒{args.box_id}），持续 {args.duration:g} 秒；Ctrl+C 可安全提前结束。")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("收到 Ctrl+C，正在停止并保存三个 bag……")
    finally:
        # 三路并行停止，避免等待前一路退出时后一路继续录十多秒。
        with ThreadPoolExecutor(max_workers=3) as pool:
            list(pool.map(stop_process, processes))
        uwb_log.close()
        imu_log.close()
        livox_log.close()

    missing = [
        str(path / "metadata.yaml")
        for path in (uwb_bag, imu_bag, livox_bag)
        if not (path / "metadata.yaml").is_file()
    ]
    if missing:
        print("录包失败，缺少：" + ", ".join(missing))
        print(f"日志目录：{root}")
        return 1

    for bag in (uwb_bag, imu_bag, livox_bag):
        subprocess.run(["ros2", "bag", "info", str(bag)], check=False)
    archive = root.with_suffix(".tar.gz")
    print(f"正在压缩：{archive}（点云较大，请等待）", flush=True)
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(root, arcname=root.name)
    print(f"录包目录：{root}")
    print(f"压缩包：{archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

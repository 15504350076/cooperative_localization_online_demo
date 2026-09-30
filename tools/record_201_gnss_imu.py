#!/usr/bin/env python3
"""在 201 盒上同时录制 GNSS 和本机 IMU。

GNSS 与 IMU 当前使用不同的 ROS_LOCALHOST_ONLY 设置：
GNSS=0（共享 DDS），IMU=1（本机 DDS）。因此使用两个 rosbag2 进程，
由本脚本在同一个屏障后同时启动，并在同一时长后同时停止。
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import tarfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path


GNSS_TOPIC = "/gnss/fix"
GNSS_ID_TOPIC = "/node/id"
IMU_TOPIC = "/imu/raw"


def ros_env(localhost_only: str) -> dict[str, str]:
    """为单个 ROS 子进程复制当前环境并覆盖 DDS 相关变量。"""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "107"
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["ROS_LOCALHOST_ONLY"] = localhost_only
    return env


def topic_type(topic: str, env: dict[str, str]) -> str:
    """在指定 DDS 环境下查询 Topic 类型。"""
    result = subprocess.run(
        ["ros2", "topic", "type", "--no-daemon", topic],
        env=env,
        text=True,
        capture_output=True,
        timeout=10,
        check=False,
    )
    return result.stdout.strip().splitlines()[-1] if result.stdout.strip() else ""


def stop_process(process: subprocess.Popen[bytes] | None) -> None:
    """按 Ctrl+C 的语义停止 rosbag，超时后再使用 TERM/KILL。"""
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def main() -> int:
    parser = argparse.ArgumentParser(description="同时录制201盒 GNSS 和 IMU")
    parser.add_argument("--duration", type=float, default=180.0, help="正式录制时长，默认180秒")
    parser.add_argument("--settle", type=float, default=3.0, help="启动后等待录包订阅稳定的秒数，默认3秒")
    parser.add_argument("--output-dir", type=Path, default=Path("/home/firefly/gnss_test"))
    args = parser.parse_args()
    if args.duration <= 0 or args.settle < 0:
        parser.error("--duration 必须大于0，--settle不能为负数")

    gnss_env = ros_env("0")
    imu_env = ros_env("1")
    gnss_type = topic_type(GNSS_TOPIC, gnss_env)
    imu_type = topic_type(IMU_TOPIC, imu_env)
    if not gnss_type:
        print("错误：共享DDS环境中看不到 /gnss/fix。请先确认 gnss_uwb 正在运行。")
        return 2
    if not imu_type:
        print("错误：本机DDS环境中看不到 /imu/raw。请先确认 navcore 正在运行。")
        return 2
    print(f"GNSS: {GNSS_TOPIC} -> {gnss_type} (ROS_LOCALHOST_ONLY=0)")
    print(f"IMU:  {IMU_TOPIC} -> {imu_type} (ROS_LOCALHOST_ONLY=1)")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    root = args.output_dir / f"gnss_imu_v1_static_{stamp}"
    gnss_bag = root / "gnss"
    imu_bag = root / "imu"
    root.mkdir(parents=True)
    gnss_log = (root / "gnss_record.log").open("wb")
    imu_log = (root / "imu_record.log").open("wb")
    commands = [
        ["ros2", "bag", "record", "-o", str(gnss_bag), GNSS_TOPIC, GNSS_ID_TOPIC],
        ["ros2", "bag", "record", "-o", str(imu_bag), IMU_TOPIC],
    ]
    processes: list[subprocess.Popen[bytes] | None] = [None, None]
    barrier = threading.Barrier(2)

    def launch(index: int, env: dict[str, str], log) -> None:
        barrier.wait()
        processes[index] = subprocess.Popen(
            commands[index],
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    threads = [
        threading.Thread(target=launch, args=(0, gnss_env, gnss_log)),
        threading.Thread(target=launch, args=(1, imu_env, imu_log)),
    ]
    start_wall = utc_now()
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    print(f"两路录包进程已同时发起：{start_wall}")
    print(f"等待 {args.settle:g} 秒后开始正式计时，请保持201静止……")
    time.sleep(args.settle)
    record_start = utc_now()
    print(f"正式录制开始：{record_start}，持续 {args.duration:g} 秒")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("收到 Ctrl+C，提前停止两路录包")
    record_end = utc_now()
    for process in processes:
        stop_process(process)
    gnss_log.close()
    imu_log.close()

    info = root / "record_info.txt"
    info.write_text(
        "".join([
            f"launch_wall_utc={start_wall}\n",
            f"record_start_wall_utc={record_start}\n",
            f"record_end_wall_utc={record_end}\n",
            f"duration_requested_s={args.duration}\n",
            f"gnss_topic={GNSS_TOPIC}\n",
            f"gnss_type={gnss_type}\n",
            f"gnss_ros_localhost_only=0\n",
            f"imu_topic={IMU_TOPIC}\n",
            f"imu_type={imu_type}\n",
            f"imu_ros_localhost_only=1\n",
        ]),
        encoding="utf-8",
    )
    missing = [str(path) for path in (gnss_bag / "metadata.yaml", imu_bag / "metadata.yaml") if not path.is_file()]
    if missing:
        print("录包失败，缺少：" + ", ".join(missing))
        print(f"日志目录：{root}")
        return 1

    archive = root.with_suffix(".tar.gz")
    with tarfile.open(archive, "w:gz") as tar:
        tar.add(root, arcname=root.name)
    print(f"录包目录：{root}")
    print(f"压缩包：{archive}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

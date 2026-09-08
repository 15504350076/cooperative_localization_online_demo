#!/usr/bin/env python3
"""直接测量仿真定位输出相对于同历元真值的误差。"""

import argparse
import math
import time

import rclpy
from cooperative_localization_msgs.msg import CooperativePose2DArray


POSE_TOPIC = "/cooperative_localization/poses_2d"
TRUTH_TOPIC = "/simulation/ground_truth/poses_2d"
NODE_IDS = (1, 2, 3)


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def angle_error_rad(left, right):
    return math.atan2(math.sin(left - right), math.cos(left - right))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0,
                        help="采集时长（秒），默认10")
    parser.add_argument("--max-match-ms", type=float, default=100.0,
                        help="输出与真值最大允许时间差（毫秒），默认100")
    args = parser.parse_args()
    if args.duration <= 0.0 or args.max_match_ms <= 0.0:
        parser.error("--duration 和 --max-match-ms 必须为正数")

    rclpy.init()
    node = rclpy.create_node("simulation_accuracy_measurement")
    poses = []
    truth = []
    pose_sub = node.create_subscription(
        CooperativePose2DArray, POSE_TOPIC, poses.append, 10
    )
    truth_sub = node.create_subscription(
        CooperativePose2DArray, TRUTH_TOPIC, truth.append, 10
    )
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(pose_sub)
        node.destroy_subscription(truth_sub)
        node.destroy_node()
        rclpy.shutdown()

    if not poses or not truth:
        print("没有同时收到算法输出和仿真真值，请确认仿真正在运行。")
        return 1

    max_delta_ns = int(args.max_match_ms * 1_000_000.0)
    truth_by_time = [(stamp_ns(message), message) for message in truth]
    samples = {node_id: [] for node_id in NODE_IDS}
    for pose_message in poses:
        pose_time = stamp_ns(pose_message)
        truth_time, truth_message = min(
            truth_by_time, key=lambda item: abs(item[0] - pose_time)
        )
        if abs(truth_time - pose_time) > max_delta_ns:
            continue
        pose_by_id = {item.node_id: item for item in pose_message.vehicles}
        truth_by_id = {item.node_id: item for item in truth_message.vehicles}
        for node_id in NODE_IDS:
            pose = pose_by_id.get(node_id)
            reference = truth_by_id.get(node_id)
            if pose is None or reference is None:
                continue
            if not (pose.position_valid and pose.yaw_valid):
                continue
            position_error = math.hypot(
                pose.x_m - reference.x_m, pose.y_m - reference.y_m
            )
            yaw_error = abs(angle_error_rad(pose.yaw_rad, reference.yaw_rad))
            samples[node_id].append((position_error, yaw_error))

    print(f"算法样本={len(poses)} 真值样本={len(truth)}")
    print(f"时间配对阈值={args.max_match_ms:.1f} ms")
    if not any(samples.values()):
        print("没有成功配对的有效位姿，请检查时间戳或仿真是否仍在运行。")
        return 1
    for node_id in NODE_IDS:
        values = samples[node_id]
        if not values:
            print(f"车辆{node_id}: 无有效配对样本")
            continue
        position = [item[0] for item in values]
        yaw_rad = [item[1] for item in values]
        position_rms = math.sqrt(sum(value * value for value in position) / len(position))
        yaw_rms = math.sqrt(sum(value * value for value in yaw_rad) / len(yaw_rad))
        print(
            f"车辆{node_id}: 样本={len(values)} "
            f"位置误差 RMSE={position_rms:.4f} m "
            f"平均={sum(position) / len(position):.4f} m "
            f"最大={max(position):.4f} m; "
            f"航向误差 RMSE={math.degrees(yaw_rms):.4f} deg "
            f"平均={math.degrees(sum(yaw_rad) / len(yaw_rad)):.4f} deg "
            f"最大={math.degrees(max(yaw_rad)):.4f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

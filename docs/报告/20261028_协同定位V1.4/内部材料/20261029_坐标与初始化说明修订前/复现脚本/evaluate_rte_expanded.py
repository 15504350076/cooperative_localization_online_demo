#!/usr/bin/env python3
"""生成并评估大运动范围三车RTE带真值仿真。

默认采用300秒低速编队运动：前30秒静止标定，之后编队沿大范围平面轨迹运动，
车辆间距保持在20～30米以内，最大速度控制在3米/秒以内。
该脚本写入独立目录，不覆盖data/rte_accuracy_20261023中的18组基线结果。
"""
import argparse
import csv
import json
import subprocess
from pathlib import Path

import numpy as np

from evaluate_rte import EDGES, HEADER, HEADINGS, ROOT, rotate, save_json, summarize

OUTPUT_DEFAULT = ROOT / 'data/rte_expanded_20261025'
DURATION_S = 300.0
MOTION_START_S = 30.0
DROPOUT = None
# 合成里程计慢漂移模型，标准差为各车每轴恒定漂移速率；不是雷达或视觉设备指标。
ODOMETRY_DRIFT_MPS = 0.0005
ODOMETRY_YAW_DRIFT_RAD_S = 0.00002
START_RAMP_S = 5.0
FORMATION_OFFSETS = np.array(((0.0, 0.0), (15.0, 0.0), (5.0, 13.0)))
FORMATION_MOTION_SCALE = 0.6
CENTER_AX, CENTER_AY = 35.0, 50.0
CENTER_WX, CENTER_WY = 0.04, 0.035


def _center_state(u):
    p = np.array((CENTER_AX * (1.0 - np.cos(CENTER_WX * u)),
                  CENTER_AY * np.sin(CENTER_WY * u)))
    v = np.array((CENTER_AX * CENTER_WX * np.sin(CENTER_WX * u),
                  CENTER_AY * CENTER_WY * np.cos(CENTER_WY * u)))
    a = np.array((CENTER_AX * CENTER_WX ** 2 * np.cos(CENTER_WX * u),
                  -CENTER_AY * CENTER_WY ** 2 * np.sin(CENTER_WY * u)))
    return p, v, a


def _offset_state(n, u):
    if n == 0:
        x, y = 6.0 * np.sin(0.10 * u + 0.1), 5.0 * np.cos(0.08 * u + 0.6)
        vx, vy = 6.0 * 0.10 * np.cos(0.10 * u + 0.1), -5.0 * 0.08 * np.sin(0.08 * u + 0.6)
        ax, ay = -6.0 * 0.10 ** 2 * np.sin(0.10 * u + 0.1), -5.0 * 0.08 ** 2 * np.cos(0.08 * u + 0.6)
    elif n == 1:
        x, y = 6.0 * np.sin(0.12 * u + 0.4), 6.0 * np.cos(0.09 * u + 0.2)
        vx, vy = 6.0 * 0.12 * np.cos(0.12 * u + 0.4), -6.0 * 0.09 * np.sin(0.09 * u + 0.2)
        ax, ay = -6.0 * 0.12 ** 2 * np.sin(0.12 * u + 0.4), -6.0 * 0.09 ** 2 * np.cos(0.09 * u + 0.2)
    else:
        x, y = 6.0 * np.cos(0.11 * u + 0.8), 5.0 * np.sin(0.13 * u + 0.3)
        vx, vy = -6.0 * 0.11 * np.sin(0.11 * u + 0.8), 5.0 * 0.13 * np.cos(0.13 * u + 0.3)
        ax, ay = -6.0 * 0.11 ** 2 * np.cos(0.11 * u + 0.8), -5.0 * 0.13 ** 2 * np.sin(0.13 * u + 0.3)
    return (FORMATION_MOTION_SCALE * np.array((x, y)),
            FORMATION_MOTION_SCALE * np.array((vx, vy)),
            FORMATION_MOTION_SCALE * np.array((ax, ay)))


def world_position(n, t):
    u, _, _ = motion_clock(t)
    center, _, _ = _center_state(u)
    offset, _, _ = _offset_state(n, u)
    offset0, _, _ = _offset_state(n, 0.0)
    return FORMATION_OFFSETS[n] + center + offset - offset0


def motion_clock(t):
    elapsed = max(0.0, t - MOTION_START_S)
    decay = np.exp(-elapsed / START_RAMP_S)
    return elapsed - START_RAMP_S * (1.0 - decay), 1.0 - decay, decay / START_RAMP_S if elapsed > 0 else 0.0


def motion(n, t):
    """返回局部位置、速度、加速度、航向和角速度。单位分别为m、m/s、m/s²、rad、rad/s。"""
    u, du, ddu = motion_clock(t)
    center, center_v, center_a = _center_state(u)
    offset, offset_v, offset_a = _offset_state(n, u)
    offset0, _, _ = _offset_state(n, 0.0)
    world_delta = center + offset - offset0
    world_v = (center_v + offset_v) * du
    world_a = (center_a + offset_a) * du * du + (center_v + offset_v) * ddu
    p = rotate(world_delta, -HEADINGS[n])
    v = rotate(world_v, -HEADINGS[n])
    a = rotate(world_a, -HEADINGS[n])
    yaw = 0.04 * (n + 1) * u + 0.08 * (np.sin(0.04 * u + 0.2 * n) - np.sin(0.2 * n))
    rate = (0.04 * (n + 1) + 0.0032 * np.cos(0.04 * u + 0.2 * n)) * du
    return p, v, a, yaw, rate


def generate(path, truth_path, mode, sigma, seed):
    range_rng = np.random.default_rng(seed)
    sensor_rng = np.random.default_rng(seed + 1_000_000)
    odometry_drift_v = sensor_rng.normal(0.0, ODOMETRY_DRIFT_MPS, (3, 2))
    odometry_drift_yaw = sensor_rng.normal(0.0, ODOMETRY_YAW_DRIFT_RAD_S, 3)
    rows, truth, world_track = [], [], []
    speed_track, accel_track, yaw_rate_track = [], [], []
    ticks = int(DURATION_S * 100)
    for tick in range(ticks + 1):
        t = tick * 0.01
        stamp = 10_000_000_000 + tick * 10_000_000
        world, yaws = [], []
        for n in range(3):
            p, v, acc, yaw, rate = motion(n, t)
            world.append(world_position(n, t))
            yaws.append(HEADINGS[n] + yaw)
            speed_track.append(float(np.linalg.norm(v)))
            accel_track.append(float(np.linalg.norm(acc)))
            yaw_rate_track.append(abs(float(rate)))
            if mode == 'odometry':
                if tick % 2 == 0:
                    values = [*p, 0.0, *v, 0.0, 0.0, 0.0,
                              np.sin(yaw / 2.0), np.cos(yaw / 2.0)]
                    rows.append(('O', stamp, n + 1, 0, *values))
            elif mode == 'odometry_small_error':
                if tick % 2 == 0:
                    elapsed = max(0.0, t - MOTION_START_S)
                    noisy_p = p + odometry_drift_v[n] * elapsed
                    noisy_v = v + (odometry_drift_v[n] if elapsed > 0 else 0)
                    noisy_yaw = yaw + odometry_drift_yaw[n] * elapsed
                    values = [*noisy_p, 0.0, *noisy_v, 0.0, 0.0, 0.0,
                              np.sin(noisy_yaw / 2.0), np.cos(noisy_yaw / 2.0)]
                    rows.append(('O', stamp, n + 1, 0, *values))
            else:
                gyro = np.array((0.0, 0.0, rate))
                accel = np.array((*rotate(acc, -yaw), 9.80665))
                if mode == 'imu_noisy':
                    gyro += sensor_rng.normal(0.0, 0.0005, 3)
                    accel += sensor_rng.normal(0.0, 0.02, 3)
                    if n == 1:
                        accel[0] += 0.002
                rows.append(('I', stamp, n + 1, 0, *gyro, *accel, 0.0, 0.0, 0.0, 0.0))
        world_track.append(world)
        truth.append([stamp, *[x for n in range(3) for x in (*((world[n] - world[0]).tolist()), yaws[n])]])
        if tick % 10 == 0:
            for i, j in EDGES:
                distance = np.linalg.norm(world[i] - world[j]) + range_rng.normal(0.0, sigma)
                rows.append(('R', stamp, i + 1, j + 1, distance, max(sigma, 0.001), *([0.0] * 8)))
    rows.sort(key=lambda row: (row[1], row[0] == 'R', row[2], row[3]))
    with path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(HEADER)
        writer.writerows(rows)
    np.savetxt(
        truth_path, truth, delimiter=',',
        header='timestamp_ns,' + ','.join(f'{axis}_{n}' for n in range(1, 4) for axis in ['x', 'y', 'yaw']),
        comments='', fmt='%.17g')
    world_xy = np.asarray(world_track)
    max_range = max(
        np.linalg.norm(world_xy[:, i, :] - world_xy[:, j, :], axis=1).max()
        for i, j in EDGES)
    min_range = min(
        np.linalg.norm(world_xy[:, i, :] - world_xy[:, j, :], axis=1).min()
        for i, j in EDGES)
    return {
        'duration_s': DURATION_S,
        'motion_start_s': MOTION_START_S,
        'range_dropout_s': None,
        'world_extent_x_m': float(np.ptp(world_xy[:, :, 0])),
        'world_extent_y_m': float(np.ptp(world_xy[:, :, 1])),
        'max_true_range_m': float(max_range),
        'min_true_range_m': float(min_range),
        'max_speed_mps': max(speed_track),
        'max_horizontal_accel_mps2': max(accel_track),
        'max_yaw_rate_rps': max(yaw_rate_track),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runner', type=Path, default=ROOT / 'build_rte_verify/evaluate_rte')
    parser.add_argument('--output', type=Path, default=OUTPUT_DEFAULT)
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--include-raw-imu', action='store_true',
                        help='同时生成原始含噪声IMU积分诊断组；默认大运动统计采用局部里程计输入。')
    parser.add_argument('--include-small-odometry-error', action='store_true',
                        help='增加小误差局部里程计补充组；默认目标统计使用无误差局部里程计。')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    summary = {
        'date': '2026-10-27',
        'profile': 'large_motion_range_300s',
        'scope': '开发机带真值仿真；不覆盖18组基线结果。',
        'configuration': {'duration_s': DURATION_S, 'motion_start_s': MOTION_START_S,
        'range_dropout_s': None, 'range_rate_hz': 10,
                          'odometry_rate_hz': 50, 'imu_rate_hz': 100,
                          'minimum_initialization_motion_span_m': 10.0,
                          'start_ramp_s': START_RAMP_S,
                          'small_odometry_drift': {
                              'position_drift_rate_std_mps': ODOMETRY_DRIFT_MPS,
                              'yaw_drift_rate_std_rad_s': ODOMETRY_YAW_DRIFT_RAD_S,
                              'description': '每车独立的恒定慢漂移；合成里程计输出，非雷达或视觉传感器仿真'},
                          'target_metric_input': '局部里程计输入'},
        'simulation': [],
    }
    experiments = [('odometry', 0.02), ('odometry', 0.05), ('odometry', 0.10)]
    if args.include_small_odometry_error:
        experiments += [('odometry_small_error', 0.02), ('odometry_small_error', 0.05), ('odometry_small_error', 0.10)]
    if args.include_raw_imu:
        experiments += [('imu_noisy', 0.02), ('imu_noisy', 0.05)]
    for mode, sigma in experiments:
        for seed in range(args.seeds):
            name = f'exp_{mode}_{sigma:.2f}_seed{seed}'
            source = out / f'{name}.csv'
            truth = out / f'{name}.truth.csv'
            geometry = generate(source, truth, mode, sigma, seed)
            if not summary['simulation']:
                summary['configuration'].update({
                    key: geometry[key] for key in (
                        'max_speed_mps', 'max_horizontal_accel_mps2', 'max_yaw_rate_rps')})
            output = source.with_name(source.stem + '_output.csv')
            subprocess.run([str(args.runner.resolve()), str(source), str(output)], check=True)
            result = summarize(output, truth, duration_s=DURATION_S,
                               motion_start_s=MOTION_START_S, dropout_window=None)
            result.update(name=name, mode=mode, range_sigma_m=sigma, seed=seed, **geometry)
            summary['simulation'].append(result)
            save_json(out / 'summary.json', summary)
            rmse = '无有效输出' if result['position_rmse_m'] is None else f'{result["position_rmse_m"]:.3f}米'
            print(f'[完成] {name} READY={result["ready_rows"]} RMSE={rmse} '
                  f'最大真值间距={result["max_true_range_m"]:.1f}米', flush=True)
    save_json(out / 'summary.json', summary)
    print(f'[全部完成] {out / "summary.json"}')


if __name__ == '__main__':
    main()

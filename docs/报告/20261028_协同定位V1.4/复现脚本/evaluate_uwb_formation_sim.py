#!/usr/bin/env python3
"""对大运动仿真的同步UWB测距调用现有三角形库并与规范坐标真值比较。"""
import argparse
import csv
import ctypes as ct
import json
from pathlib import Path

import numpy as np

from evaluate_rte import ROOT

DATA = ROOT / 'data/rte_expanded_20261025'
OUT = DATA / 'uwb_formation'


class Ranges(ct.Structure):
    _fields_ = [('struct_size', ct.c_uint32), ('abi_version', ct.c_uint32),
                ('node_ids', ct.c_uint32 * 3), ('reserved0', ct.c_uint32),
                ('range_01_m', ct.c_double), ('range_02_m', ct.c_double),
                ('range_12_m', ct.c_double), ('minimum_height_m', ct.c_double)]


class Formation(ct.Structure):
    _fields_ = [('struct_size', ct.c_uint32), ('abi_version', ct.c_uint32),
                ('disposition', ct.c_int32), ('reserved0', ct.c_uint32),
                ('node_ids', ct.c_uint32 * 3), ('reserved1', ct.c_uint32),
                ('x_m', ct.c_double * 3), ('y_m', ct.c_double * 3),
                ('twice_area_m2', ct.c_double)]


def library(path):
    lib = ct.CDLL(str(path.resolve()))
    lib.zju_coop_three_node_ranges_init.argtypes = [ct.POINTER(Ranges)]
    lib.zju_coop_three_node_formation_init.argtypes = [ct.POINTER(Formation)]
    lib.zju_coop_solve_three_node_formation.argtypes = [ct.POINTER(Ranges), ct.POINTER(Formation)]
    for name in ('zju_coop_three_node_ranges_init', 'zju_coop_three_node_formation_init',
                 'zju_coop_solve_three_node_formation'):
        getattr(lib, name).restype = ct.c_int32
    return lib


def solve(lib, distances):
    ranges, result = Ranges(), Formation()
    if lib.zju_coop_three_node_ranges_init(ct.byref(ranges)) or lib.zju_coop_three_node_formation_init(ct.byref(result)):
        raise RuntimeError('三角形结构初始化失败')
    ranges.node_ids[:] = (1, 2, 3)
    ranges.range_01_m, ranges.range_02_m, ranges.range_12_m = distances
    ranges.minimum_height_m = 0.001
    if lib.zju_coop_solve_three_node_formation(ct.byref(ranges), ct.byref(result)):
        raise RuntimeError('三角形库调用失败')
    return result


def evaluate(lib, name, output_dir):
    grouped = {}
    with (DATA / f'{name}.csv').open(newline='') as stream:
        for row in csv.DictReader(stream):
            if row['kind'] == 'R':
                stamp = int(row['timestamp_ns'])
                edge = (int(row['node_id']), int(row['target_id']))
                if edge in grouped.setdefault(stamp, {}):
                    raise ValueError(f'重复测距: {stamp} {edge}')
                grouped[stamp][edge] = float(row['v0'])
    truth = np.genfromtxt(DATA / f'{name}.truth.csv', delimiter=',', names=True)
    start = min(grouped)
    rows, errors, errors_by_car = [], [], [[], []]
    dispositions = {}
    for stamp, edges in sorted(grouped.items()):
        if set(edges) != {(1, 2), (1, 3), (2, 3)}:
            raise ValueError(f'该仿真历元缺少三边: {stamp}')
        formed = solve(lib, [edges[(1, 2)], edges[(1, 3)], edges[(2, 3)]])
        dispositions[str(formed.disposition)] = dispositions.get(str(formed.disposition), 0) + 1
        t = (stamp - start) / 1e9
        if formed.disposition != 0:
            rows.append([stamp, t, 0, formed.disposition] + [''] * 8)
            continue
        p2 = np.array([np.interp(stamp, truth['timestamp_ns'], truth[f'{axis}_2']) for axis in ('x', 'y')])
        p3 = np.array([np.interp(stamp, truth['timestamp_ns'], truth[f'{axis}_3']) for axis in ('x', 'y')])
        norm = np.linalg.norm(p2)
        if norm <= 0:
            raise ValueError('真值参考边长度为零')
        truth2 = np.array([norm, 0.0])
        truth3 = np.array([np.dot(p2, p3) / norm, abs(p2[0] * p3[1] - p2[1] * p3[0]) / norm])
        e2 = np.linalg.norm(np.array([formed.x_m[1], formed.y_m[1]]) - truth2)
        e3 = np.linalg.norm(np.array([formed.x_m[2], formed.y_m[2]]) - truth3)
        errors.extend((float(e2), float(e3)))
        errors_by_car[0].append(float(e2)); errors_by_car[1].append(float(e3))
        rows.append([stamp, t, 1, formed.disposition, formed.x_m[1], formed.y_m[1],
                     formed.x_m[2], formed.y_m[2], *truth3, e2, e3])
    with (output_dir / f'{name}_uwb_output.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('timestamp_ns', 'elapsed_s', 'valid', 'disposition', 'x_2', 'y_2',
                         'x_3', 'y_3', 'truth_x_3', 'truth_y_3', 'error_2_m', 'error_3_m'))
        writer.writerows(rows)
    values = np.asarray(errors)
    return {'name': name, 'evaluated_epochs': len(grouped), 'valid_epochs': len(errors) // 2,
            'dispositions': dispositions,
            'first_valid_s': next((r[1] for r in rows if r[2]), None),
            'position_rmse_m': float(np.sqrt(np.mean(values ** 2))) if len(values) else None,
            'position_p95_m': float(np.percentile(values, 95)) if len(values) else None,
            'position_max_m': float(np.max(values)) if len(values) else None,
            'under_1m_fraction': float(np.mean(values < 1.0)) if len(values) else None,
            'vehicle_rmse_m': [float(np.sqrt(np.mean(np.square(e)))) if e else None for e in errors_by_car]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library', type=Path, default=ROOT / 'build_rte_verify/libzju_coop.so')
    parser.add_argument('--output', type=Path, default=OUT)
    args = parser.parse_args()
    lib = library(args.library)
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for sigma in (0.02, 0.05, 0.10):
        for seed in range(3):
            name = f'exp_odometry_{sigma:.2f}_seed{seed}'
            result = evaluate(lib, name, args.output)
            result.update(range_sigma_m=sigma, seed=seed)
            results.append(result)
            print(f"{name}: RMSE={result['position_rmse_m']:.3f}米 有效={result['valid_epochs']}/{result['evaluated_epochs']}", flush=True)
    summary = {'method': '现有zju_coop_solve_three_node_formation；同一历元三边',
               'frame': '车1原点、车1到车2为正X、车3取正Y的规范相对三角形坐标',
               'sensor_input': '已有大运动场景同步UWB仿真测距；无IMU/里程计',
               'position_rmse': '车2和车3规范相对位置向量误差在有效历元上的合并均方根',
               'simulation': results}
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(f"结果：{args.output / 'summary.json'}")


if __name__ == '__main__':
    main()

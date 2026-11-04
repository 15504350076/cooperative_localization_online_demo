#!/usr/bin/env python3
"""为大运动范围三车仿真生成报告图表。"""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np
from matplotlib.patches import Rectangle

from evaluate_rte import EDGES
from evaluate_rte_expanded import DURATION_S, MOTION_START_S, OUTPUT_DEFAULT, world_position

ROOT = Path(__file__).resolve().parents[1]
DATA = OUTPUT_DEFAULT
DEST = ROOT / 'docs/报告/20261023_协同定位阶段报告/figures'
COLORS = ['#24557b', '#ad653a', '#4f806b']
_font_candidates = [
    ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 'Noto Sans CJK JP'),
    ('/usr/share/fonts/truetype/wqy/wqy-microhei.ttc', 'WenQuanYi Micro Hei'),
    ('/usr/share/fonts/truetype/arphic/uming.ttc', 'AR PL UMing CN'),
]
FONT_FAMILY = 'DejaVu Sans'
for _font_path, _font_name in _font_candidates:
    if Path(_font_path).exists():
        fm.fontManager.addfont(_font_path)
        FONT_FAMILY = _font_name
        break
plt.rcParams.update({'font.family': FONT_FAMILY, 'axes.unicode_minus': False,
                     'font.size': 10, 'axes.spines.top': False,
                     'axes.spines.right': False, 'figure.facecolor': 'white'})


def save(fig, name):
    DEST.mkdir(parents=True, exist_ok=True)
    fig.savefig(DEST / f'{name}.png', dpi=190, bbox_inches='tight')
    fig.savefig(DEST / f'{name}.svg', bbox_inches='tight')
    plt.close(fig)


def build_world():
    t = np.arange(0.0, DURATION_S + 0.0001, 0.01)
    world = np.empty((len(t), 3, 2))
    for k, sec in enumerate(t):
        for n in range(3):
            world[k, n] = world_position(n, sec)
    return t, world


def trajectory(t, world):
    fig, ax = plt.subplots(figsize=(10.5, 6.4))
    xmin, xmax = world[:, :, 0].min(), world[:, :, 0].max()
    ymin, ymax = world[:, :, 1].min(), world[:, :, 1].max()
    pad = 6
    ax.add_patch(Rectangle((xmin - pad, ymin - pad), xmax - xmin + 2 * pad,
                           ymax - ymin + 2 * pad, facecolor='#edf3ed',
                           edgecolor='#91aa96', lw=1.2, alpha=.75,
                           label='大运动范围仿真'))
    for n, color in enumerate(COLORS):
        ax.plot(world[:, n, 0], world[:, n, 1], color=color, lw=1.6,
                label=f'车{n + 1}轨迹')
        ax.scatter(world[0, n, 0], world[0, n, 1], color=color, marker='o', s=35)
        ax.scatter(world[-1, n, 0], world[-1, n, 1], color=color, marker='s', s=35)
        ax.text(world[0, n, 0] + .8, world[0, n, 1] + .8, f'车{n + 1}起点', color=color, fontsize=9)
        ax.text(world[-1, n, 0] + .8, world[-1, n, 1] + .8, f'车{n + 1}终点', color=color, fontsize=9)
    ax.set(xlabel='场景X（米）', ylabel='场景Y（米）',
           title=f'大运动范围三车仿真轨迹（{DURATION_S:.0f}秒）', aspect='equal')
    ax.grid(alpha=.22)
    ax.legend(ncol=4, loc='upper center', bbox_to_anchor=(.5, 1.12))
    max_range = max(np.linalg.norm(world[:, i] - world[:, j], axis=1).max() for i, j in EDGES)
    ax.text(.01, .01,
            f'静止标定{MOTION_START_S:.0f}秒；场景包络约{np.ptp(world[:, :, 0]):.1f}×{np.ptp(world[:, :, 1]):.1f}米；最大真值车间距约{max_range:.1f}米',
            transform=ax.transAxes, fontsize=9, color='#485c50', va='bottom')
    fig.tight_layout()
    save(fig, '11_仿真大范围三车轨迹')


def range_and_displacement(t, world):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for color, (i, j) in zip(COLORS, EDGES):
        axes[0].plot(t, np.linalg.norm(world[:, i] - world[:, j], axis=1),
                     color=color, lw=1.0, label=f'车{i + 1}—车{j + 1}')
    for n, color in enumerate(COLORS):
        axes[1].plot(t, np.linalg.norm(world[:, n] - world[0, n], axis=1),
                     color=color, lw=1.0, label=f'车{n + 1}')
    for ax in axes:
        ax.grid(alpha=.2)
    axes[0].set(ylabel='真值车间距（米）', title='大运动范围中的三条UWB真值测距')
    axes[1].set(xlabel='仿真时间（秒）', ylabel='相对初始位置位移（米）',
                title='三车相对初始位置的运动幅度')
    axes[0].legend(ncol=4, fontsize=9)
    axes[1].legend(ncol=4, fontsize=9)
    fig.tight_layout()
    save(fig, '12_仿真测距与运动幅度')


def load_output(name):
    output = np.genfromtxt(DATA / f'{name}_output.csv', delimiter=',', names=True)
    return output[output['elapsed_s'] <= DURATION_S + 1e-6]


def localization_error_curves():
    """同一条噪声样本的逐车误差；RTE与纯UWB分别采用各自的坐标真值。"""
    uwb_dir = DATA / 'uwb_formation'
    if not (uwb_dir / 'summary.json').exists():
        raise SystemExit('缺少纯UWB仿真结果，请先运行evaluate_uwb_formation_sim.py')
    for kind in ('uwb', 'rte'):
        fig, axes = plt.subplots(3, 1, figsize=(10, 7.8), sharex=True)
        for ax, sigma in zip(axes, (0.02, 0.05, 0.10)):
            name = f'exp_odometry_{sigma:.2f}_seed0'
            if kind == 'uwb':
                a = np.genfromtxt(uwb_dir / f'{name}_uwb_output.csv', delimiter=',', names=True)
                good = a['valid'] == 1
                errors = [a['error_2_m'], a['error_3_m']]
            else:
                a = load_output(name)
                truth = np.genfromtxt(DATA / f'{name}.truth.csv', delimiter=',', names=True)
                good = a['status'] == 1
                stamp = a['state_timestamp_ns']
                errors = [np.hypot(a[f'x_{n}'] - np.interp(stamp, truth['timestamp_ns'], truth[f'x_{n}']),
                                   a[f'y_{n}'] - np.interp(stamp, truth['timestamp_ns'], truth[f'y_{n}']))
                          for n in (2, 3)]
            for car, error, color in zip((2, 3), errors, COLORS):
                ax.plot(a['elapsed_s'], np.where(good, error, np.nan), color=color,
                        lw=0.8, label=f'车{car}相对位置误差')
            ax.axhline(1.0, color='#a3483c', ls='--', lw=1.0, label='1米参考线')
            ax.set_ylabel('误差（米）')
            ax.set_title(f'测距误差标准差 {sigma * 100:.0f} 厘米', fontsize=10)
            ax.grid(alpha=.2)
            ax.legend(ncol=3, fontsize=8, loc='upper right')
        axes[-1].set_xlabel('仿真时间（秒）')
        axes[-1].set_xlim(0, DURATION_S)
        fig.suptitle(('纯UWB相对三角形' if kind == 'uwb' else 'RTE（无误差局部里程计）') +
                     '逐车位置误差：重复试验1', fontsize=13)
        fig.tight_layout()
        save(fig, '15_纯UWB逐车定位误差' if kind == 'uwb' else '16_RTE逐车定位误差')


def errors_and_status(summary):
    candidates = [r for r in summary['simulation']
                  if r['mode'] == 'odometry' and r['range_sigma_m'] == 0.02]
    candidates.sort(key=lambda r: float('inf') if r['position_rmse_m'] is None else r['position_rmse_m'])
    names = [candidates[0]['name'], candidates[-1]['name']]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.2))
    for name, color in zip(names, COLORS[:2]):
        a = load_output(name)
        truth = np.genfromtxt(DATA / f'{name}.truth.csv', delimiter=',', names=True)
        good = a['status'] == 1
        stamp = a['state_timestamp_ns']
        err2 = np.hypot(a['x_2'] - np.interp(stamp, truth['timestamp_ns'], truth['x_2']),
                        a['y_2'] - np.interp(stamp, truth['timestamp_ns'], truth['y_2']))
        err3 = np.hypot(a['x_3'] - np.interp(stamp, truth['timestamp_ns'], truth['x_3']),
                        a['y_3'] - np.interp(stamp, truth['timestamp_ns'], truth['y_3']))
        label = f'重复试验{int(name.rsplit("_", 1)[-1].replace("seed", "")) + 1}'
        axes[0, 0].plot(a['elapsed_s'], np.where(good, err2, np.nan),
                        color=color, lw=1.0, label=f'{label}·车2')
        axes[0, 0].plot(a['elapsed_s'], np.where(good, err3, np.nan),
                        color=color, lw=1.0, ls='--', label=f'{label}·车3')
        axes[0, 1].plot(a['elapsed_s'], a['status'], color=color, lw=1.0, label=label)
        axes[1, 1].plot(a['elapsed_s'], a['rms_m'], color=color, lw=1.0, label=label)
    axes[0, 0].set(title='无误差里程计、2厘米测距：位置误差', ylabel='位置误差（米）')
    axes[0, 1].set(title='相同条件下的相对变换估计状态码', ylabel='状态码')
    axes[1, 1].set(title='相同条件下的测距拟合残差', ylabel='残差（米）', xlabel='仿真时间（秒）')
    groups = {}
    for record in summary['simulation']:
        if record['mode'] in ('odometry', 'odometry_small_error') and record['position_rmse_m'] is not None:
            groups.setdefault((record['mode'], record['range_sigma_m']), []).append(record['position_rmse_m'])
    keys = sorted(groups)
    labels = [f"{'无误差' if mode == 'odometry' else '小漂移'}\n{int(sigma * 100)}厘米"
              for mode, sigma in keys]
    means = np.array([np.mean(groups[k]) for k in keys])
    lower = means - np.array([min(groups[k]) for k in keys])
    upper = np.array([max(groups[k]) for k in keys]) - means
    axes[1, 0].bar(np.arange(len(keys)), means, yerr=np.array([lower, upper]), capsize=3,
                   color=['#24557b' if mode == 'odometry' else '#ad653a' for mode, _ in keys])
    axes[1, 0].set_xticks(np.arange(len(keys)), labels, fontsize=8)
    axes[1, 0].set_title('各组位置均方根误差：均值及范围')
    axes[1, 0].set_ylabel('位置均方根误差（米）')
    for ax in axes.flat:
        if ax.has_data():
            ax.grid(alpha=.18)
            handles, labels = ax.get_legend_handles_labels()
            if labels:
                ax.legend(handles, labels, fontsize=8)
    fig.tight_layout()
    save(fig, '13_扩展仿真误差与状态')


def algorithm_trajectories():
    """对照展示RTE实际输出轨迹，区分低误差和较大误差示例。"""
    summary = json.loads((DATA / 'summary.json').read_text())
    candidates = [r for r in summary['simulation']
                  if r['mode'] == 'odometry' and r['range_sigma_m'] == 0.02]
    candidates.sort(key=lambda r: float('inf') if r['position_rmse_m'] is None else r['position_rmse_m'])
    low, high = candidates[0], candidates[-1]
    cases = [
        (low['name'], f"2厘米测距噪声，重复试验{low['seed'] + 1}（较低误差示例）"),
        (high['name'], f"2厘米测距噪声，重复试验{high['seed'] + 1}（较高误差示例）"),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), sharex=True, sharey=True)
    for ax, (name, title) in zip(axes, cases):
        output = load_output(name)
        truth = np.genfromtxt(DATA / f'{name}.truth.csv', delimiter=',', names=True)
        truth_t = truth['timestamp_ns']
        valid = output['status'] == 1
        for vehicle, color in zip((2, 3), COLORS[:2]):
            true_x = np.interp(output['state_timestamp_ns'], truth_t, truth[f'x_{vehicle}'])
            true_y = np.interp(output['state_timestamp_ns'], truth_t, truth[f'y_{vehicle}'])
            ax.plot(true_x, true_y, color=color, lw=1.7, label=f'车{vehicle}真值')
            estimate_x = np.where(valid, output[f'x_{vehicle}'], np.nan)
            estimate_y = np.where(valid, output[f'y_{vehicle}'], np.nan)
            ax.plot(estimate_x, estimate_y, color=color, lw=1.1, ls='--',
                    label=f'车{vehicle} RTE输出')
        ax.scatter([0], [0], color=COLORS[2], s=28, marker='o', label='参考车车1')
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=.2)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(fontsize=8, loc='best')
    axes[0].set_ylabel('参考车局部Y（米）')
    for ax in axes:
        ax.set_xlabel('参考车局部X（米）')
    fig.suptitle('大运动范围场景中RTE输出轨迹与真值对照', fontsize=14)
    fig.tight_layout()
    save(fig, '14_大范围仿真RTE输出轨迹')


def main():
    summary_path = DATA / 'summary.json'
    if not summary_path.exists():
        raise SystemExit(f'缺少{summary_path}，请先运行evaluate_rte_expanded.py')
    summary = json.loads(summary_path.read_text())
    t, world = build_world()
    trajectory(t, world)
    range_and_displacement(t, world)
    errors_and_status(summary)
    algorithm_trajectories()
    localization_error_curves()
    print('已生成大运动范围轨迹和逐车定位误差图')


if __name__ == '__main__':
    main()

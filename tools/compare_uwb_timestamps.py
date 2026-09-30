#!/usr/bin/env python3
"""比较同一录包盒收到的三盒 UWB 消息时间；输入既有 UWB测距数据.csv。

python3 tools/compare_uwb_timestamps.py data/某联合实验/某_uwb_rec_v3
仅写派生 CSV、MD、PNG；不校时，不使用 rolling，不把接收相近冒充硬件同步。
"""
import argparse
from bisect import bisect_left
from collections import defaultdict
import csv
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from statistics import median


def percentile(values, p):
    """线性插值分位数；空配对返回 NaN，而不是捏造 0。"""
    if not values:
        return float("nan")
    values = sorted(values)
    pos = (len(values) - 1) * p
    lo = int(pos)
    return values[lo] + (values[min(lo + 1, len(values) - 1)] - values[lo]) * (pos - lo)


def nearest(times, value):
    pos = bisect_left(times, value)
    return min((i for i in (pos - 1, pos) if 0 <= i < len(times)),
               key=lambda i: (abs(times[i] - value), i))


def pairs(left, right, window_ns=20_000_000):
    """两个源盒事件按接收时刻互为最近邻，最多20ms，一对一且不跨录包盒。

    配对不依赖待检查的 header，保留坏时间戳；两个源盒接收相近仍不保证是同一硬件测距事件。
    """
    if not left or not right:
        return
    lt, rt = [r[1] for r in left], [r[1] for r in right]
    for i, a in enumerate(left):
        j = nearest(rt, a[1])
        if abs(rt[j] - a[1]) <= window_ns and nearest(lt, rt[j]) == i:
            yield a, right[j]


def integer_median(values):
    """整数纳秒中位数，避免先转float导致Unix纳秒丢失低位。"""
    values = sorted(values)
    middle = len(values) // 2
    return values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) // 2


def source_events(streams):
    """把同一源盒、同一header发给不同目标的消息合并成一个源盒事件。"""
    grouped = defaultdict(lambda: defaultdict(list))
    for (src, _dst), rows in streams.items():
        for sequence, received, header in rows:
            grouped[src][header].append((sequence, received, header))
    result = {}
    for src, by_header in grouped.items():
        # 元组仍为（可追溯首序号、代表接收时刻、header）；按接收顺序配对。
        events = [(min(r[0] for r in rows), integer_median(r[1] for r in rows), header)
                  for header, rows in by_header.items()]
        result[src] = sorted(events, key=lambda row: row[1])
    return result


def exact_three_edge_epochs(streams):
    """统计同一header中是否同时具备三条无向边。

    这只回答“算法能否按消息时间组成三边历元”，不把相同header
    解释成三台无线电在物理上同一瞬间完成了测距。
    """
    node_ids = sorted({node for edge in streams for node in edge})
    if len(node_ids) != 3:
        return 0, 0, ()
    required = {tuple(edge) for edge in combinations(node_ids, 2)}
    by_header = defaultdict(set)
    for (src, dst), rows in streams.items():
        edge = tuple(sorted((src, dst)))
        for _sequence, _received, header in rows:
            by_header[header].add(edge)
    return sum(required <= edges for edges in by_header.values()), len(by_header), tuple(node_ids)


def analyze(directory):
    streams = defaultdict(list)
    # 数据格式为（原测距序号，接收纳秒，header纳秒），所有差值先用整数计算。
    with (directory / "UWB测距数据.csv").open(encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            key = (int(r["源车辆ID"]), int(r["目标车辆ID"]))
            streams[key].append((int(r["序号"]), int(r["rosbag接收时间_ns"]),
                                 int(r["消息时间_sec"]) * 10**9 + int(r["消息时间_nanosec"])))
    if not streams:
        raise ValueError("CSV 没有测距消息")
    for rows in streams.values():
        if any(b[1] < a[1] for a, b in zip(rows, rows[1:])):
            raise ValueError("接收时间自身回退，不能用二分最近邻；需先分析录包系统时钟")
    origin = min(r[1] for rows in streams.values() for r in rows)
    output = directory / "UWB时间对比"
    output.mkdir(exist_ok=True)
    lines = ["# 三盒 UWB 时间戳对比", "", f"输入：`{directory.name}/UWB测距数据.csv`。",
             "", "## 方法和限制", "",
             "- 本文件按消息的 src_id/target_id 区分来源，不把录包盒当作测距源。",
             "- 六方向分别按原接收顺序差分，不对交织的整条 Topic 直接差分。",
             "- 同一src_id在一轮内会分别产生发给两个目标的消息；先按相同header合并成一个源盒事件，避免把它重复计算两次。",
             "- 再比较源盒1/2/3：按同一录包盒的接收时刻互为最近邻，间隔≤20ms，一对一配对。",
             "- 定义 ΔH=H_B−H_A，ΔR=R_B−R_A，E=ΔH−ΔR（单位ms）。E仍含UWB时隙、处理及传输延迟差，不等于纯时钟偏移或同步精度。",
             "- 20ms只是配对窗口；100ms只是统计大差值的描述阈值，右侧图放大到±10ms，均不是同步合格阈值。全量配对和异常均保存。",
             "- rolling不参与分组、配对、统计或结论；它是上交调试字段。",
             "- 静止数据没有独立共同事件，不能由这些比较证明PPS/硬件采样同步，也不能据此校准时钟。",
             "", "## 六方向全部消息", "",
             "|方向|条数|接收−消息中位数(s)|消息间隔P50(ms)|回退数|最小消息间隔(ms)|最大消息间隔(ms)|",
             "|---|---:|---:|---:|---:|---:|---:|"]
    anomalies = []
    for (src, dst), rows in sorted(streams.items()):
        dt = [(b[2] - a[2]) / 1e6 for a, b in zip(rows, rows[1:])]
        lines.append(f"|{src}→{dst}|{len(rows)}|{median((r[1]-r[2])/1e9 for r in rows):.6f}|"
                     f"{percentile(dt, .5):.3f}|{sum(v < 0 for v in dt)}|"
                     f"{min(dt) if dt else float('nan'):.3f}|{max(dt) if dt else float('nan'):.3f}|")
        for a, b in zip(rows, rows[1:]):
            if b[2] < a[2]:
                anomalies.append(f"- {src}→{dst}，原CSV序号{a[0]}→{b[0]}："
                                 f"header `{a[2]}`→`{b[2]}` ns，"
                                 f"消息差{(b[2]-a[2])/1e6:.3f}ms，接收差{(b[1]-a[1])/1e6:.3f}ms。")
    first = min((r for rows in streams.values() for r in rows), key=lambda r: r[1])
    utc = lambda ns: datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()
    exact_count, unique_headers, node_ids = exact_three_edge_epochs(streams)
    node_text = "-".join(str(value) for value in node_ids) if node_ids else "非三节点"
    lines += ["", f"第一条接收时间UTC：{utc(first[1])}；消息时间UTC：{utc(first[2])}。",
              "", "## 三边消息同历元", "",
              f"- 节点 `{node_text}` 共有 **{exact_count}** 个完全相同 `header.stamp` 且三条无向边齐全的历元；全包共有 {unique_headers} 个不同header值。",
              "- 这些历元可供三角形解算从正反向报告中挑选三条无向边；缺边的header不得强行补成三角形。",
              "- header完全相同只说明应用层分组条件满足，不证明物理测距同时，也不消除后文列出的header回退。",
              "", "## 三个源盒事件的近邻配对", "",
              "|源盒A / B|配对数|A/B未配对事件|ΔR中位数(ms)|ΔH P05 / 中位数 / P95(ms)|E中位数(ms)|全量|E| P95(ms)|全量最大|E|(ms)||E|>100ms条数|",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    # Markdown 表头避免绝对值竖线被当成列分隔符。
    lines[-2] = lines[-2].replace("|E|", "abs(E)")
    comparisons = {}
    csv_rows = []
    events = source_events(streams)
    for src, dst in combinations(sorted(events), 2):
        left, right = events[src], events[dst]
        results = []
        for a, b in pairs(left, right):
            dr, dh = (b[1] - a[1]) / 1e6, (b[2] - a[2]) / 1e6
            e = ((b[2] - a[2]) - (b[1] - a[1])) / 1e6
            t = (a[1] - origin) / 1e9
            results.append((t, dr, dh, e))
            csv_rows.append([f"盒{src}", f"盒{dst}", a[0], b[0],
                             a[1], b[1], a[2], b[2], t, dr, dh, e, abs(e) > 100])
        comparisons[f"盒{src}↔盒{dst}"] = results
        headers = [r[2] for r in results]
        errors = [r[3] for r in results]
        absolute = [abs(e) for e in errors]
        lines.append(f"|盒{src} / 盒{dst}|{len(results)}|{len(left)-len(results)}/{len(right)-len(results)}|"
                     f"{percentile([r[1] for r in results], .5):.3f}|"
                     f"{percentile(headers, .05):.3f} / {percentile(headers, .5):.3f} / {percentile(headers, .95):.3f}|"
                     f"{percentile(errors, .5):.3f}|"
                     f"{percentile(absolute, .95):.3f}|{max(absolute) if absolute else float('nan'):.3f}|"
                     f"{sum(e > 100 for e in absolute)}|")
    lines += ["", "## 异常定位", ""] + (anomalies or ["未检出同方向 header 回退；不代表同步已验证。"])
    lines += ["", "## 文件", "", "- [三盒UWB时间近邻配对.csv](三盒UWB时间近邻配对.csv)：全量配对和原消息序号，可追溯。",
              "- [三盒UWB时间对比.png](三盒UWB时间对比.png)：同一接收时间横轴；左侧全量纵轴，右侧放大到±10ms并注明图外点数。",
              "- 无法配对的消息不计入配对统计，但仍全部计入上方六方向统计和原始CSV。", ""]
    with (output / "三盒UWB时间近邻配对.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["A源盒", "B源盒", "A首原序号", "B首原序号", "A代表接收_ns", "B代表接收_ns",
                         "A消息_ns", "B消息_ns", "接收相对时间_s", "接收差B减A_ms",
                         "消息差B减A_ms", "消息差减接收差_ms", "绝对差大于100ms"])
        writer.writerows(csv_rows)
    (output / "三盒UWB时间对比.md").write_text("\n".join(lines), encoding="utf-8")
    plot(comparisons, output)
    print("\n".join(lines))


def plot(comparisons, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(len(comparisons), 2, figsize=(14, 3 * len(comparisons)), squeeze=False)
    for (label, rows), (full, zoom) in zip(comparisons.items(), axes):
        full.scatter([r[0] for r in rows], [r[3] for r in rows], s=3)
        full.set_title(f"{label}：全量 E=消息差−接收差")
        zoom.scatter([r[0] for r in rows], [r[3] for r in rows], s=3)
        zoom.set_ylim(-10, 10)
        zoom.set_title(f"{label}：±10ms局部放大（{sum(abs(r[3]) > 10 for r in rows)} 点在图外）")
        for ax in (full, zoom):
            ax.set_xlabel("相对本包首条UWB接收时间（s）")
            ax.set_ylabel("E（ms，非纯时钟偏移）")
            ax.grid(alpha=.3)
    fig.suptitle("三盒UWB源时钟比较：同源重复消息已合并；静止近邻配对 ≠ 硬件同步", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, .96))
    fig.savefig(output / "三盒UWB时间对比.png", dpi=160)
    plt.close(fig)


def self_check():
    a = [(0, 1_000_000_000, 0), (1, 1_100_000_000, 100_000_000)]
    b = [(2, 1_001_000_000, 5_000_000), (3, 1_101_000_000, -1_000_000_000)]
    result = list(pairs(a, b))
    assert len(result) == 2  # 坏 header 不能被配对逻辑隐藏。
    x, y = result[0]
    assert ((y[2]-x[2])-(y[1]-x[1])) / 1e6 == 4
    assert list(pairs(a, [(4, 2_000_000_000, 0)])) == []
    assert len(list(pairs(a, b[:1]))) == 1  # 不复用对方样本。
    assert list(pairs([], b)) == [] and percentile([0, 10], .95) == 9.5
    streams = {(1, 2): [(0, 10, 100)], (1, 3): [(1, 12, 100)],
               (2, 1): [(2, 20, 101)], (2, 3): [(3, 22, 101)]}
    events = source_events(streams)
    assert events[1] == [(0, 11, 100)] and events[2] == [(2, 21, 101)]
    count, total, nodes = exact_three_edge_epochs(streams)
    assert (count, total, nodes) == (0, 2, (1, 2, 3))
    streams[(2, 3)].append((4, 23, 100))
    assert exact_three_edge_epochs(streams)[0] == 1
    print("自检通过：同源合并/近邻一对一/窗口/坏时间保留/整数时间差/分位数")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_check()
    elif args.directory:
        analyze(args.directory)
    else:
        parser.error("请指定含 UWB测距数据.csv 的单个录包目录")

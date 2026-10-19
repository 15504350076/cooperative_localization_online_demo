#!/usr/bin/env python3
"""分析 v4l2-ctl --verbose 生成的三路相机时间日志。"""

import argparse
import re
import statistics
import tarfile
from pathlib import Path


LINE = re.compile(
    r"cap dqbuf:\s*\d+ seq:\s*(\d+) bytesused:\s*(\d+) "
    r"ts:\s*(\d+)\.(\d+)"
)


def read_logs(archive: Path) -> dict[str, list[tuple[int, int, int]]]:
    result = {}
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle.getmembers():
            match = re.search(r"video(\d+)\.log$", member.name)
            if not match:
                continue
            text = bundle.extractfile(member).read().decode("utf-8", errors="replace")
            rows = [
                (int(m.group(1)), int(m.group(3)) * 1_000_000 + int(m.group(4)), int(m.group(2)))
                for m in LINE.finditer(text)
            ]
            if rows:
                result[match.group(1)] = rows
    if not result:
        raise ValueError("压缩包中没有可解析的 video*.log")
    return dict(sorted(result.items()))


def summarize(rows: list[tuple[int, int, int]]) -> str:
    deltas = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    jumps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    return (
        f"帧数 {len(rows)}，序号 {rows[0][0]}..{rows[-1][0]}，"
        f"序号跳变 {sum(x != 1 for x in jumps)}，回退 {sum(x < 0 for x in deltas)}，"
        f"重复时间戳 {sum(x == 0 for x in deltas)}，"
        f"时长 {(rows[-1][1] - rows[0][1]) / 1e6:.6f} s，"
        f"频率 {(len(rows) - 1) / ((rows[-1][1] - rows[0][1]) / 1e6):.6f} Hz，"
        f"帧间隔中位数/最小/最大 "
        f"{statistics.median(deltas) / 1000:.3f}/"
        f"{min(deltas) / 1000:.3f}/{max(deltas) / 1000:.3f} ms，"
        f"bytesused={sorted({row[2] for row in rows})}"
    )


def analyze(archive: Path, output: Path) -> None:
    logs = read_logs(archive)
    lines = [f"# 相机时间日志分析：{archive.name}", ""]
    for number, rows in logs.items():
        lines += [f"## /dev/video{number}", f"- {summarize(rows)}", ""]

    numbers = list(logs)
    lines += ["## 同序号时间偏差", ""]
    for index, left in enumerate(numbers):
        for right in numbers[index + 1:]:
            a = {seq: ts for seq, ts, _ in logs[left]}
            b = {seq: ts for seq, ts, _ in logs[right]}
            common = sorted(set(a) & set(b))
            offsets = [b[seq] - a[seq] for seq in common]
            absolute = sorted(abs(value) for value in offsets)
            p95 = absolute[max(0, int(len(absolute) * 0.95) - 1)] / 1000
            lines.append(
                f"- video{left} → video{right}：共同序号 {len(common)}，"
                f"偏差中位数/最小/最大 {statistics.median(offsets) / 1000:.3f}/"
                f"{min(offsets) / 1000:.3f}/{max(offsets) / 1000:.3f} ms，"
                f"绝对偏差P95 {p95:.3f} ms，首尾漂移 {(offsets[-1] - offsets[0]) / 1000:.3f} ms"
            )
    lines += [
        "",
        "结论只针对 V4L2 buffer 的 ts-monotonic；本日志没有图像文件，也没有 IMU/UWB 时间，不能单独证明跨传感器同步。",
        "",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")
    print(f"分析完成：{output}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or args.archive.with_name(args.archive.name.removesuffix(".tar.gz") + "_analysis.md")
    analyze(args.archive, output)


if __name__ == "__main__":
    main()

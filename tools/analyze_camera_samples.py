#!/usr/bin/env python3
"""分析三路 V4L2 NV12 单帧样本，输出画面预览和简单质量统计。

用法：python3 tools/analyze_camera_samples.py data/camera_samples.tar.gz
依赖：numpy、opencv-python（或系统提供的 python3-numpy、python3-opencv）。
本程序不分析帧率或同步：v4l2-ctl --stream-to 保存的单帧 raw 没有时间戳。
"""

import argparse
import hashlib
import re
import tarfile
from pathlib import Path

import cv2
import numpy as np


def analyze(archive: Path, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    lines = [f"# 相机单帧分析：{archive.name}", ""]
    luma = {}
    hashes = {}

    with tarfile.open(archive, "r:gz") as bundle:
        members = {Path(item.name).name: item for item in bundle if item.isfile()}
        ids = sorted(
            name[5:-4] for name in members
            if re.fullmatch(r"video\d+\.raw", name)
        )
        if not ids:
            raise ValueError("压缩包中没有 video*.raw")

        for number in ids:
            info_name = f"video{number}.txt"
            raw_name = f"video{number}.raw"
            if info_name not in members:
                raise ValueError(f"缺少 {info_name}")
            info = bundle.extractfile(members[info_name]).read().decode("utf-8")
            size = re.search(r"Width/Height\s*:\s*(\d+)/(\d+)", info)
            fmt = re.search(r"Pixel Format\s*:\s*'([^']+)'", info)
            bus = re.search(r"Bus info\s*:\s*(.+)", info)
            if not size or not fmt or fmt.group(1) != "NV12":
                raise ValueError(f"{info_name} 未给出可处理的 NV12 尺寸")
            width, height = map(int, size.groups())
            raw = bundle.extractfile(members[raw_name]).read()
            expected = width * height * 3 // 2
            if len(raw) != expected:
                raise ValueError(f"{raw_name} 大小 {len(raw)}，预期 {expected} 字节")

            nv12 = np.frombuffer(raw, dtype=np.uint8).reshape(height * 3 // 2, width)
            image = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
            preview = cv2.resize(image, (960, round(height * 960 / width)), interpolation=cv2.INTER_AREA)
            preview_path = output / f"video{number}_preview.png"
            if not cv2.imwrite(str(preview_path), preview):
                raise OSError(f"无法写入 {preview_path}")
            y = nv12[:height]
            luma[number] = cv2.resize(y, (240, round(height * 240 / width)), interpolation=cv2.INTER_AREA)
            hashes[number] = hashlib.sha256(raw).hexdigest()
            lines += [
                f"## /dev/video{number}",
                f"- 设备：{bus.group(1) if bus else '未记录'}；格式：NV12；尺寸：{width}×{height}；文件：{len(raw)} 字节",
                f"- 亮度 Y：均值 {y.mean():.1f}，标准差 {y.std():.1f}，近黑（Y<16）{np.mean(y < 16):.2%}，近白（Y>240）{np.mean(y > 240):.2%}",
                f"- SHA-256：`{hashes[number]}`",
                f"- [预览图]({preview_path.name})",
                "",
            ]

    lines += ["## 三路比较", ""]
    for index, left in enumerate(ids):
        for right in ids[index + 1:]:
            a, b = luma[left].astype(np.int16), luma[right].astype(np.int16)
            lines.append(
                f"- video{left} / video{right}："
                f"{'完全相同' if hashes[left] == hashes[right] else '文件不同'}；"
                f"缩略亮度平均绝对差 {np.abs(a - b).mean():.1f}"
            )
    lines += ["", "本包只有每路一帧；无法判定帧率、三路同步、与 IMU/UWB 的时间关系或相机标定。", ""]
    report = output / "相机单帧分析.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    print(f"分析完成：{report}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("archive", type=Path, help="camera_samples.tar.gz")
    parser.add_argument("--output", type=Path, help="输出目录；默认与压缩包同级的 camera_samples_analysis")
    args = parser.parse_args()
    output = args.output or args.archive.with_name(args.archive.name.removesuffix(".tar.gz") + "_analysis")
    analyze(args.archive, output)


if __name__ == "__main__":
    main()

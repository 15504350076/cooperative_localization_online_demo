#!/usr/bin/env python3
"""Export a ROS 2 sensor_msgs/Imu bag to complete and human-sized CSV files."""

# 流式解码并导出，不把整包装进内存；不估姿态、不补零偏、不转换消息单位。
import argparse
import csv
import gzip
from pathlib import Path
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


# 主表顺序必须与main_row一致；四元数采用ROS的xyzw顺序。
MAIN_FIELDS = (
    "index", "bag_receive_ns", "bag_elapsed_s", "header_sec", "header_nanosec",
    "header_elapsed_s", "header_dt_ms", "frame_id", "qx", "qy", "qz", "qw",
    "gyro_x_message_unit", "gyro_y_message_unit", "gyro_z_message_unit",
    "accel_x_message_unit", "accel_y_message_unit", "accel_z_message_unit",
)
# 三组3×3协方差各展开9列，完整CSV保留；抽样表省略它们以便阅读。
COVARIANCE_FIELDS = tuple(
    f"{name}_{index}"
    for name in ("orientation_covariance", "gyro_covariance", "accel_covariance")
    for index in range(9)
)
FULL_FIELDS = MAIN_FIELDS + COVARIANCE_FIELDS
TIMING_FIELDS = (
    "index", "event", "bag_receive_ns", "bag_elapsed_s", "previous_header_sec",
    "previous_header_nanosec", "header_sec", "header_nanosec", "header_dt_ms",
)


def stamp_ns(message):
    # 整数时间用于精确差分，避免浮点绝对秒吞掉纳秒低位。
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def timing_event(delta_ns):
    # 以1000Hz/1ms为基准分级，优先判回退和重复；每间隔只归一个类别。
    # 不是1ms的间隔都会被列出，并非都属于故障（如时钟微调或其他采样率）。
    if delta_ns < 0:
        return "rollback"
    if delta_ns == 0:
        return "duplicate"
    if delta_ns > 100_000_000:
        return "forward_jump_gt_100ms"
    if delta_ns >= 2_000_000:
        return "gap_ge_2ms"
    if delta_ns != 1_000_000:
        return "sub_2ms_adjustment"
    return ""


def main_row(index, received_ns, first_received_ns, message, first_stamp_ns, previous_stamp_ns):
    # 返回一条完整CSV行；首帧没有前帧，header_dt_ms留空。
    # 两种elapsed各自减本包首值，仅用于看本包变化，不是跨盒同步时间。
    current_stamp_ns = stamp_ns(message)
    return (
        index,
        received_ns,
        f"{(received_ns - first_received_ns) / 1e9:.9f}",
        message.header.stamp.sec,
        message.header.stamp.nanosec,
        f"{(current_stamp_ns - first_stamp_ns) / 1e9:.9f}",
        "" if previous_stamp_ns is None else f"{(current_stamp_ns - previous_stamp_ns) / 1e6:.6f}",
        message.header.frame_id,
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
        *message.orientation_covariance,
        *message.angular_velocity_covariance,
        *message.linear_acceleration_covariance,
    )


def export_bag(bag, topic, stride):
    # 单包导出：完整gzip CSV、抽样CSV、时间事件CSV、阅读说明。
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a ROS 2 bag directory: {bag}")

    metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=metadata.storage_identifier
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topics.get(topic) != "sensor_msgs/msg/Imu":
        raise ValueError(
            f"{topic} must be sensor_msgs/msg/Imu; available: {topics or 'none'}"
        )

    full_path = bag / "imu_full.csv.gz"
    preview_path = bag / "imu_10hz.csv"
    timing_path = bag / "imu_timing_events.csv"
    readme_path = bag / "EXTRACTED_README.md"
    # 先写临时文件，读完并关闭后逐个替换，避免读取失败时覆盖旧报告。
    # 三次替换不是跨文件事务；不要并发导出同一个bag目录。
    temporary_paths = [Path(f"{path}.tmp") for path in (full_path, preview_path, timing_path)]

    event_counts = {}
    count = 0
    first_received_ns = None  # 用None区分尚未初始化与合法的零时间戳。
    first_stamp_ns = None
    previous_stamp_ns = None
    previous_stamp_parts = None
    last_preview_index = -1
    last_row = None

    try:
        with gzip.open(temporary_paths[0], "wt", encoding="utf-8", newline="") as full_stream, \
                temporary_paths[1].open("w", encoding="utf-8", newline="") as preview_stream, \
                temporary_paths[2].open("w", encoding="utf-8", newline="") as timing_stream:
            full_writer = csv.writer(full_stream)
            preview_writer = csv.writer(preview_stream)
            timing_writer = csv.writer(timing_stream)
            full_writer.writerow(FULL_FIELDS)
            preview_writer.writerow(MAIN_FIELDS)
            timing_writer.writerow(TIMING_FIELDS)

            # 保留reader读取顺序，不按header排序；全部消息写入full_writer。
            while reader.has_next():
                current_topic, serialized, received_ns = reader.read_next()
                if current_topic != topic:
                    continue
                message = deserialize_message(serialized, Imu)
                current_stamp_ns = stamp_ns(message)
                if first_received_ns is None:
                    first_received_ns = received_ns
                    first_stamp_ns = current_stamp_ns

                row = main_row(
                    count, received_ns, first_received_ns, message, first_stamp_ns,
                    previous_stamp_ns,
                )
                full_writer.writerow(row)
                # 只有预览表抽样；stride=100且原频率1000Hz时才约10Hz。
                if count % stride == 0:
                    preview_writer.writerow(row[: len(MAIN_FIELDS)])
                    last_preview_index = count

                # 异常表从全量帧计算，不从预览表计算，避免漏掉瞬间跳变。
                if previous_stamp_ns is not None:
                    delta_ns = current_stamp_ns - previous_stamp_ns
                    event = timing_event(delta_ns)
                    if event:
                        event_counts[event] = event_counts.get(event, 0) + 1
                        timing_writer.writerow(
                            (
                                count,
                                event,
                                received_ns,
                                f"{(received_ns - first_received_ns) / 1e9:.9f}",
                                previous_stamp_parts[0],
                                previous_stamp_parts[1],
                                message.header.stamp.sec,
                                message.header.stamp.nanosec,
                                f"{delta_ns / 1e6:.6f}",
                            )
                        )

                last_row = row
                previous_stamp_ns = current_stamp_ns
                previous_stamp_parts = (
                    message.header.stamp.sec,
                    message.header.stamp.nanosec,
                )
                count += 1

            if count == 0:
                raise ValueError(f"no messages on {topic}")
            # 额外保存末帧，便于直观看首末跨度；避免末帧重复。
            if last_preview_index != count - 1:
                preview_writer.writerow(last_row[: len(MAIN_FIELDS)])

        for temporary, final in zip(temporary_paths, (full_path, preview_path, timing_path)):
            temporary.replace(final)
    except Exception:
        # 清理本次临时文件并向调用方抛错；不删除原始bag。
        for path in temporary_paths:
            path.unlink(missing_ok=True)
        raise

    event_summary = ", ".join(
        f"`{name}`={value}" for name, value in sorted(event_counts.items())
    ) or "无"
    # 此说明还列举其他工具可生成的imu_analysis.txt/PNG，本脚本自身不生成它们。
    readme_path.write_text(
        "\n".join(
            (
                "# IMU 可读数据说明",
                "",
                f"- 源 rosbag：`{bag.name}`",
                f"- Topic：`{topic}`",
                f"- 完整消息数：{count}",
                f"- 可读抽样：每 {stride} 帧保留一帧，并额外保留末帧",
                f"- 时间事件：{event_summary}",
                "",
                "## 文件",
                "",
                "- `imu_full.csv.gz`：完整逐帧CSV，包含三组9元素协方差；使用电子表格前先解压。",
                "- `imu_10hz.csv`：约10 Hz降采样，适合直接用编辑器或电子表格查看。",
                "- `imu_timing_events.csv`：所有非1.000 ms间隔、回退、重复和大跳。",
                "- `imu_analysis.txt`：频率、均值、标准差、姿态跳变和协方差摘要。",
                "- `IMU姿态连续性与跳变分析.png`：姿态连续性图和最大跳变附近数据。",
                "",
                "## 注意",
                "",
                "CSV中的陀螺列保留消息原始数值，名称明确标为`message_unit`；本工具不替上游猜测deg/s或rad/s。",
                "`header_sec/header_nanosec`保留精确整数，避免用浮点秒损失纳秒精度。",
                "",
            )
        ),
        encoding="utf-8",
    )
    return count, full_path, preview_path, timing_path, readme_path


def main():
    # 命令行支持多包与自定义抽样步长；抽样不影响原始包和完整CSV。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--topic", default="/imu/raw")
    parser.add_argument("--stride", type=int, default=100)
    args = parser.parse_args()
    if args.stride <= 0:
        parser.error("--stride must be positive")

    # 四个轻量断言验证关键分类边界。
    assert timing_event(-1) == "rollback"
    assert timing_event(0) == "duplicate"
    assert timing_event(1_000_000) == ""
    assert timing_event(2_000_000) == "gap_ge_2ms"

    try:
        for bag in args.bags:
            count, *paths = export_bag(bag.expanduser().resolve(), args.topic, args.stride)
            print(f"{bag}: {count} messages")
            for path in paths:
                print(f"  {path}")
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Plot a ROS 2 sensor_msgs/Imu bag and mark all significant orientation jumps."""

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def positive_float(value):
    value = float(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


def read_imu(bag, topic):
    metadata = rosbag2_py.Info().read_metadata(str(bag), "")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag), storage_id=metadata.storage_identifier
        ),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in types:
        raise ValueError(f"topic not found: {topic}; available: {', '.join(sorted(types))}")
    if types[topic] != "sensor_msgs/msg/Imu":
        raise ValueError(f"{topic} is {types[topic]}, expected sensor_msgs/msg/Imu")

    received_times = []
    header_seconds = []
    header_nanoseconds = []
    quaternions = []
    gyro = []
    acceleration = []
    while reader.has_next():
        name, serialized, received_ns = reader.read_next()
        if name != topic:
            continue
        message = deserialize_message(serialized, Imu)
        stamp = message.header.stamp
        received_times.append(received_ns)
        header_seconds.append(stamp.sec)
        header_nanoseconds.append(stamp.nanosec)
        quaternions.append(
            (message.orientation.x, message.orientation.y,
             message.orientation.z, message.orientation.w)
        )
        gyro.append(
            (message.angular_velocity.x, message.angular_velocity.y,
             message.angular_velocity.z)
        )
        acceleration.append(
            (message.linear_acceleration.x, message.linear_acceleration.y,
             message.linear_acceleration.z)
        )
    if len(received_times) < 2:
        raise ValueError(
            f"need at least two messages on {topic}, found {len(received_times)}"
        )
    return (
        np.asarray(received_times, dtype=np.int64),
        np.asarray(header_seconds, dtype=np.int64),
        np.asarray(header_nanoseconds, dtype=np.int64),
        np.asarray(quaternions, dtype=np.float64),
        np.asarray(gyro, dtype=np.float64),
        np.asarray(acceleration, dtype=np.float64),
    )


def largest_quaternion_jump(quaternions):
    norms = np.linalg.norm(quaternions, axis=1)
    valid = np.isfinite(norms) & (norms > 1e-12)
    normalized = np.zeros_like(quaternions)
    normalized[valid] = quaternions[valid] / norms[valid, None]
    pair_valid = valid[:-1] & valid[1:]
    if not pair_valid.any():
        raise ValueError("bag contains no adjacent valid orientation quaternions")
    dots = np.abs(np.sum(normalized[:-1] * normalized[1:], axis=1))
    angles = np.full(len(dots), np.nan)
    angles[pair_valid] = np.degrees(2.0 * np.arccos(np.clip(dots[pair_valid], 0.0, 1.0)))
    pair_index = int(np.nanargmax(angles))
    return pair_index + 1, angles


def write_window_csv(
    path, received_ns, header_seconds, header_nanoseconds,
    quaternions, gyro, acceleration, elapsed_s, jump_angles, indices,
):
    header = (
        "index", "bag_receive_ns", "bag_elapsed_s", "header_sec", "header_nanosec",
        "qx", "qy", "qz", "qw", "gx_rad_s", "gy_rad_s", "gz_rad_s",
        "ax_m_s2", "ay_m_s2", "az_m_s2", "jump_from_previous_deg",
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index in indices:
            jump = "" if index == 0 else jump_angles[index - 1]
            writer.writerow(
                (index, received_ns[index], elapsed_s[index],
                 header_seconds[index], header_nanoseconds[index],
                 *quaternions[index], *gyro[index], *acceleration[index], jump)
            )


def plot(quaternions, gyro, acceleration, elapsed_s,
         jump_index, jump_angle, jump_indices, jump_threshold, indices, output):
    displayed_quaternions = quaternions.copy()
    norms = np.linalg.norm(displayed_quaternions, axis=1)
    valid = np.isfinite(norms) & (norms > 1e-12)
    displayed_quaternions[valid] /= norms[valid, None]
    for index in range(1, len(displayed_quaternions)):
        if valid[index - 1] and valid[index] and np.dot(
            displayed_quaternions[index - 1], displayed_quaternions[index]
        ) < 0.0:
            displayed_quaternions[index] *= -1.0
    jump_time = elapsed_s[jump_index]
    window_time = elapsed_s[indices]

    figure, axes = plt.subplots(4, 1, figsize=(13, 11), constrained_layout=True)
    labels = (("qx", "qy", "qz", "qw"), ("gx", "gy", "gz"), ("ax", "ay", "az"))
    for column, label in enumerate(labels[0]):
        axes[0].plot(
            elapsed_s, displayed_quaternions[:, column], linewidth=0.8,
            label=label,
        )
        axes[1].plot(
            window_time, displayed_quaternions[indices, column], marker=".",
            label=label,
        )
    for column, label in enumerate(labels[1]):
        axes[2].plot(window_time, gyro[indices, column], label=label)
    for column, label in enumerate(labels[2]):
        axes[3].plot(window_time, acceleration[indices, column], label=label)

    axes[0].set_title(
        f"Orientation jumps > {jump_threshold:g} deg: {len(jump_indices)}; "
        f"largest {jump_angle:.6f} deg at {jump_time:.6f} s"
    )
    axes[0].set_ylabel("quaternion")
    axes[1].set_ylabel("quaternion")
    axes[2].set_ylabel("angular velocity")
    axes[3].set_ylabel("acceleration")
    axes[3].set_xlabel("rosbag receive time since first IMU message (s)")
    for index in jump_indices:
        axes[0].axvline(
            elapsed_s[index], color="red", linestyle="--", linewidth=1.0
        )
    for axis in axes[1:]:
        axis.axvline(jump_time, color="red", linestyle="--", linewidth=1.0)
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best", ncol=4)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def run(args):
    bag = Path(args.bag).expanduser().resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a ROS 2 bag directory: {bag}")

    if args.output:
        output = Path(args.output).expanduser().resolve()
        csv_output = output.with_name(f"{output.stem}_window.csv")
    else:
        output = bag / "imu_orientation_jump_analysis.png"
        csv_output = bag / "imu_orientation_jump_window.csv"
    if output.suffix.lower() != ".png":
        raise ValueError("--output must end with .png")
    output.parent.mkdir(parents=True, exist_ok=True)

    (received_ns, header_seconds, header_nanoseconds,
     quaternions, gyro, acceleration) = read_imu(bag, args.topic)
    elapsed_s = (received_ns - received_ns[0]).astype(np.float64) * 1e-9
    jump_index, jump_angles = largest_quaternion_jump(quaternions)
    jump_indices = np.flatnonzero(jump_angles > args.orientation_jump_deg) + 1
    jump_time = elapsed_s[jump_index]
    indices = np.flatnonzero(np.abs(elapsed_s - jump_time) <= args.window)
    plot(
        quaternions, gyro, acceleration, elapsed_s, jump_index,
        jump_angles[jump_index - 1], jump_indices, args.orientation_jump_deg,
        indices, output,
    )
    write_window_csv(
        csv_output, received_ns, header_seconds, header_nanoseconds,
        quaternions, gyro, acceleration, elapsed_s, jump_angles, indices,
    )

    before = quaternions[jump_index - 1]
    after = quaternions[jump_index]
    print(f"messages: {len(received_ns)}")
    print(f"jumps > {args.orientation_jump_deg:g} deg: {len(jump_indices)}")
    if len(jump_indices) > 1:
        bag_intervals = (
            np.diff(received_ns[jump_indices]).astype(np.float64) * 1.0e-9
        )
        header_ns = header_seconds * 1_000_000_000 + header_nanoseconds
        header_intervals = (
            np.diff(header_ns[jump_indices]).astype(np.float64) * 1.0e-9
        )
        print(
            "jump bag intervals: "
            + ", ".join(f"{value:.9f}" for value in bag_intervals)
            + " s"
        )
        print(
            "jump header intervals: "
            + ", ".join(f"{value:.9f}" for value in header_intervals)
            + " s"
        )
    print(f"largest jump: {jump_angles[jump_index - 1]:.9f} deg")
    print(f"message pair: {jump_index - 1} -> {jump_index}")
    print(f"bag elapsed: {elapsed_s[jump_index - 1]:.9f} -> {jump_time:.9f} s")
    print(
        f"header stamp: {header_seconds[jump_index - 1]}."
        f"{header_nanoseconds[jump_index - 1]:09d} -> "
        f"{header_seconds[jump_index]}.{header_nanoseconds[jump_index]:09d}"
    )
    print(f"quaternion: {before.tolist()} -> {after.tolist()}")
    print(f"plot: {output}")
    print(f"csv: {csv_output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS 2 bag directory")
    parser.add_argument("--topic", default="/imu/raw", help="IMU topic (default: /imu/raw)")
    parser.add_argument("--output", help="output PNG path (default: BAG/imu_orientation_jump_analysis.png)")
    parser.add_argument(
        "--window", type=positive_float, default=0.01,
        help="seconds shown/exported on each side of the jump (default: 0.01)",
    )
    parser.add_argument(
        "--orientation-jump-deg", type=positive_float, default=10.0,
        help="mark adjacent orientation jumps above this angle (default: 10)",
    )
    args = parser.parse_args()
    try:
        run(args)
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

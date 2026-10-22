#!/usr/bin/env python3
"""Check zero/NaN filtering and SQLite insertion order with reversed record times.

Run: source /opt/ros/humble/setup.bash
     python3 -B tools/test_visualize_livox_bags.py
"""
from pathlib import Path
import sqlite3
import tempfile
from unittest.mock import patch

import numpy as np
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from visualize_livox_bags import SELECTION, load_box


def main():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        bag = root / "uwb_imu_livox_v201_test" / "livox"
        bag.mkdir(parents=True)
        fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(("x", "y", "z", "intensity"))]
        with sqlite3.connect(bag / "livox_0.db3") as db:
            db.execute("CREATE TABLE topics (id INTEGER, name TEXT)")
            db.execute("INSERT INTO topics VALUES (1, '/livox/lidar')")
            db.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, "
                       "data BLOB, timestamp INTEGER)")
            for index in range(3):
                message = point_cloud2.create_cloud(
                    Header(frame_id="livox_frame"), fields,
                    [(0, 0, 0, 0), (float('nan'), 1, 2, 3), (index + 1, 2, 3, 4)],
                )
                db.execute("INSERT INTO messages VALUES (?, 1, ?, ?)",
                           (index + 1, serialize_message(message), 30 - index))
        with patch.dict(SELECTION, {"201": {"center": 1, "window": 3}}):
            result = load_box(root, "201")
        assert result["raw_points"] == 9
        assert result["zero_points"] == 3
        assert result["selected_frames"] == 3
        np.testing.assert_array_equal(result["points"][:, 0], [1, 2, 3])
        assert np.isfinite(result["points"]).all()
        with patch.dict(SELECTION, {"201": {"center": 100, "window": 3}}):
            try:
                load_box(root, "201")
            except ValueError as error:
                assert "实验窗口" in str(error)
            else:
                raise AssertionError("Out-of-range windows must fail clearly")
            adaptive = load_box(root, "201", allow_adaptive_window=True)
            assert adaptive["adaptive_window"] is True
            assert adaptive["selected_frames"] == 3
    print("Livox过滤、写入顺序及窗口边界检查通过")


if __name__ == "__main__":
    main()

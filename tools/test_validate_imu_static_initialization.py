"""小型回归：已知静止偏置、独立后段、异常间隔和可选C++核心复核。"""
import csv
import json
import math
from pathlib import Path
import sys
import tempfile

from validate_imu_static_initialization import calibrate, validate


def main():
    core = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    with tempfile.TemporaryDirectory() as folder:
        root = Path(folder)
        source = root / "imu.csv"
        # 100Hz；恒定FRD gyro=[0,0,.01]。两秒估偏，两秒独立检验。
        fields = ["index", "header_elapsed_s", "qx", "qy", "qz", "qw"]
        fields += [f"{name}_{axis}_message_unit" for name in ("gyro", "accel") for axis in "xyz"]
        with source.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            for i in range(501):
                writer.writerow([i, i*.01, 0, 0, 0, 1, 0, 0, .01, 0, 0, -9.80665])
        validate(source, root / "normal", 2., 1., 10, 4., core)
        result = json.loads((root / "normal/静止初始化统计.json").read_text())
        assert result["calibration_count"] == 201
        assert abs(result["integrated_seconds"] - 2.) < 1e-9
        assert result["final_drift_deg"] < 1e-6
        assert abs(result["final_drift_without_bias_deg"] - math.degrees(.02)) < 1e-6
        assert max(abs(v) for v in result["post_corrected_mean_rad_s"]) < 1e-12
        assert not result["evaluation_timing_events"]
        try:
            calibrate(source, 30., 1.)
        except ValueError:
            pass
        else:
            raise AssertionError("short input accepted as a full calibration window")
        # 后段真实偏置变化时，不能偷偷重新估偏而给出零漂移。
        with source.open() as stream:
            rows = list(csv.DictReader(stream))
        with source.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in rows:
                if int(row["index"]) > 200:
                    row["gyro_z_message_unit"] = ".012"
                writer.writerow(row)
        validate(source, root / "changed", 2., 1., 10, 4.)
        result = json.loads((root / "changed/静止初始化统计.json").read_text())
        assert abs(result["bias_flu_rad_s"][2] + .01) < 1e-12
        assert abs(result["post_corrected_mean_rad_s"][2] + .002) < 1e-12
        assert abs(result["final_drift_deg"] - math.degrees(.00399)) < 1e-6
        # 缺0.5秒不能被补作有效积分，但之后要能继续；保存断点原序号。
        with source.open() as stream:
            rows = list(csv.DictReader(stream))
        with source.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(row for row in rows if not 250 <= int(row["index"]) < 300)
        validate(source, root / "gap", 2., 1., 10, 4.)
        result = json.loads((root / "gap/静止初始化统计.json").read_text())
        assert len(result["evaluation_timing_events"]) == 1
        assert result["evaluation_timing_events"][0][0] == "300"
        assert abs(result["integrated_seconds"] - 1.49) < 1e-9
    print("静止偏置扣除、独立后段、窗口截断、异常间隔和C++可选复核：完成")


if __name__ == "__main__":
    main()

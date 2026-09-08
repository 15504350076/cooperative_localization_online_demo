#!/usr/bin/env bash
# 已加载ROS和cooperative_interfaces环境后运行；只分析本地已解压录包。
set -euo pipefail
if [[ $# != 1 || ! -d "$1" ]]; then
  echo '用法：bash tools/analyze_joint_recording.sh data/joint_time_实验编号' >&2
  exit 2
fi
TOOLS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$(cd -- "$1" && pwd)
python3 -c 'import rosbag2_py, numpy, matplotlib; from cooperative_interfaces.msg import UwbRange, Identity'
shopt -s nullglob
IMU_BAGS=("$RUN_DIR"/*_imu_rec_v*)
UWB_BAGS=("$RUN_DIR"/*_uwb_rec_v*)
count=0
for bag in "${IMU_BAGS[@]}" "${UWB_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] && count=$((count + 1))
done
if [[ $count == 0 ]]; then
  echo '未找到已解压的 *_imu_rec_v* 或 *_uwb_rec_v* 录包目录。' >&2
  exit 1
fi
python3 "$TOOLS_DIR/record_joint_time.py" --analyze "$RUN_DIR"
python3 "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR"
for bag in "${IMU_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/analyze_imu_bag.py" "$bag" | tee "$bag/imu_analysis.txt"
done
for bag in "${UWB_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/export_uwb_bag_readable.py" "$bag"
done
printf '\n分析完成，共处理%s个包：%s\n' "$count" "$RUN_DIR"
printf '先看联合时间初查.md，再看各IMU的imu_analysis.txt及UWB测距分析.md。\n'

#!/usr/bin/env bash
# 功能：离线验收一个或多个盒端UWB/IMU/Livox录包目录；复用现有分析程序。
# 输入是已解压的uwb_imu_livox_v<盒号>_<时间>/，不连接盒子、不修改原始bag。
# 输出：每盒时间报告、IMU分析.txt、雷达分析.txt及uwb/中的测距报告和CSV。
set -euo pipefail
TOOLS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
shopt -s nullglob
bag_has_data() {
  local metadata=$1 topic=$2
  [[ -f "$metadata" ]] || return 1
  grep --quiet "name: $topic" "$metadata" || return 1
  grep --quiet --extended-regexp '^[[:space:]]+message_count:[[:space:]]+[1-9][0-9]*' "$metadata"
}
if [[ $# == 0 ]]; then
  # 不传参数时自动选择data/下最新且三盒目录完整的实验，避免每轮手填编号。
  DATA_DIR="$TOOLS_DIR/../data"
  while IFS= read -r candidate; do
    box_dirs=()
    for box in 201 202 203; do
      matches=()
      for run_dir in "$candidate"/uwb_imu_livox_v${box}_*/; do
        [[ -d "$run_dir" ]] && matches+=("${run_dir%/}")
      done
      [[ ${#matches[@]} == 1 ]] || { box_dirs=(); break; }
      box_dirs+=("${matches[0]}")
    done
    [[ ${#box_dirs[@]} == 3 ]] || continue
    complete=true
    for run_dir in "${box_dirs[@]}"; do
      bag_has_data "$run_dir/uwb/metadata.yaml" '/uwb/range' || complete=false
      bag_has_data "$run_dir/imu/metadata.yaml" '/imu/raw' || complete=false
      bag_has_data "$run_dir/livox/metadata.yaml" '/livox/lidar' || complete=false
    done
    if [[ "$complete" == true ]]; then
      set -- "${box_dirs[@]}"
      printf '自动选择最新三盒联合包：%s\n' "$candidate"
      break
    fi
  done < <(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -name 'three_boxes_*' -print | sort --reverse)
  if [[ $# == 0 ]]; then
    echo 'data/下没有找到三盒目录完整的three_boxes_*实验。请先解压总包。' >&2
    exit 1
  fi
fi
if [[ $# -lt 1 || $# -gt 3 ]]; then
  echo '用法：bash tools/analyze_box_sensors.sh [盒201目录 盒202目录 盒203目录]' >&2
  echo '不填目录时，自动选择data/下最新的三盒联合包。' >&2
  exit 2
fi
python3 - <<'PY'
from cooperative_interfaces.msg import UwbRange
expected = ['header', 'src_id', 'target_id', 'rolling', 'distance']
if list(UwbRange.get_fields_and_field_types()) != expected:
    raise SystemExit('UWB消息安装版本过旧：请重新构建cooperative_interfaces并加载新环境；不能用旧定义解析新包。')
PY
for RUN_DIR in "$@"; do
  for BAG in uwb imu livox; do
    [[ -f "$RUN_DIR/$BAG/metadata.yaml" ]] || { echo "缺少$RUN_DIR/$BAG/metadata.yaml" >&2; exit 1; }
  done
  printf '\n正在分析：%s\n' "$RUN_DIR"
  python3 "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR" > "$RUN_DIR/time_analysis.log" 2>&1
  python3 "$TOOLS_DIR/analyze_imu_bag.py" "$RUN_DIR/imu" > "$RUN_DIR/IMU分析.txt" 2>&1
  python3 "$TOOLS_DIR/analyze_lidar_bag.py" "$RUN_DIR/livox" > "$RUN_DIR/雷达分析.txt" 2>&1
  python3 "$TOOLS_DIR/export_uwb_bag_readable.py" "$RUN_DIR/uwb" > "$RUN_DIR/uwb_analysis.log" 2>&1
  printf '完成：%s（完成分析不等于验收通过）\n' "$RUN_DIR"
done

#!/usr/bin/env bash
# 功能：离线验收一个或多个盒端UWB/IMU/Livox录包目录；复用现有分析程序。
# 输入是已解压的uwb_imu_livox_v<盒号>_<时间>/，不连接盒子、不修改原始bag。
# 输出：每盒时间报告、IMU分析.txt、雷达分析.txt及uwb/中的测距报告和CSV。
set -euo pipefail
TOOLS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# == 0 ]]; then
  echo '用法：bash tools/analyze_box_sensors.sh 已解压盒201目录 [盒202目录 盒203目录]' >&2
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

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
# 所有子程序都等待退出；报告先写临时文件，失败时保留旧报告并另存错误日志。
active_pid=''
active_tmp=''
active_output=''
summary=''
stop_analysis() {
  trap '' INT TERM
  if [[ -n "$active_pid" ]]; then
    kill -TERM "$active_pid" 2>/dev/null || true
    wait "$active_pid" 2>/dev/null || true
  fi
  if [[ -n "$active_tmp" && -f "$active_tmp" ]]; then
    mv -- "$active_tmp" "$active_output.失败.log"
  fi
  [[ -z "$summary" ]] || sed --in-place 's/^状态：运行中。/状态：已中断。/' "$summary"
  [[ -z "$summary" ]] || printf '\n本次分析被中断，尚未完成的项目不可视为成功。\n' >> "$summary"
  printf '\n[已中断] 本次分析没有全部完成。\n' >&2
  exit 130
}
trap stop_analysis INT TERM

run_step() {
  local name=$1 output=$2 started=$SECONDS next_notice=$((SECONDS + 10)) rc
  shift 2
  active_output=$output
  active_tmp=$(mktemp "$output.tmp.XXXXXX")
  printf '[开始] %s；详细输出：%s\n' "$name" "$active_tmp"
  "$@" > "$active_tmp" 2>&1 &
  active_pid=$!
  while kill -0 "$active_pid" 2>/dev/null; do
    sleep 2
    if (( SECONDS >= next_notice )); then
      printf '[运行中] %s，已用时%s秒，请等待。\n' "$name" "$((SECONDS - started))"
      next_notice=$((SECONDS + 10))
    fi
  done
  if wait "$active_pid"; then rc=0; else rc=$?; fi
  active_pid=''
  if (( rc == 0 )); then
    mv -- "$active_tmp" "$output"
    printf '[成功] %s，用时%s秒；输出：%s\n' "$name" "$((SECONDS - started))" "$output"
    printf '| %s | 成功 | %s | [%s](%s) |\n' "$name" "$((SECONDS - started))" "$(basename "$output")" "$(basename "$output")" >> "$summary"
  else
    mv -- "$active_tmp" "$output.失败.log"
    printf '[失败] %s（退出码%s）；错误日志：%s.失败.log\n' "$name" "$rc" "$output" >&2
    tail -n 8 "$output.失败.log" >&2
    printf '| %s | 失败（退出码%s） | %s | [%s.失败.log](%s.失败.log) |\n' "$name" "$rc" "$((SECONDS - started))" "$(basename "$output")" "$(basename "$output")" >> "$summary"
    failures=$((failures + 1))
  fi
  active_tmp=''
}

failures=0
for RUN_DIR in "$@"; do
  [[ -d "$RUN_DIR" ]] || { printf '盒端目录不存在：%s\n' "$RUN_DIR" >&2; exit 1; }
  summary="$RUN_DIR/分析执行结果.md"
  printf '# 分析执行结果\n\n执行时间：%s\n\n状态：运行中。程序成功仅表示完成统计，数据是否合格须查看各报告。\n\n失败项目的旧报告可能仍存在，不能当成本次成功结果。联合时间报告只包含实际存在的流，仍须核对采集清单。\n\n| 分析项目 | 执行结果 | 用时（秒） | 日志或报告 |\n|---|---|---:|---|\n' "$(date --iso-8601=seconds)" > "$summary"
  before=$failures
  printf '\n正在分析：%s\n' "$RUN_DIR"
  run_step '1/4 联合时间统计及绘图' "$RUN_DIR/time_analysis.log" python3 -u "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR"
  run_step '2/4 车载IMU分析' "$RUN_DIR/IMU分析.txt" python3 -u "$TOOLS_DIR/analyze_imu_bag.py" "$RUN_DIR/imu"
  run_step '3/4 Livox点云与内置IMU分析' "$RUN_DIR/雷达分析.txt" python3 -u "$TOOLS_DIR/analyze_lidar_bag.py" "$RUN_DIR/livox"
  run_step '4/4 UWB测距统计及绘图' "$RUN_DIR/uwb_analysis.log" python3 -u "$TOOLS_DIR/export_uwb_bag_readable.py" "$RUN_DIR/uwb"
  sed --in-place 's/^状态：运行中。/状态：已结束。/' "$summary"
  if (( failures == before )); then
    printf '\n最终状态：四项程序全部执行成功，数据质量请继续查看专项报告。\n' >> "$summary"
    printf '[全部完成] %s；先看：%s\n' "$RUN_DIR" "$summary"
  else
    printf '\n最终状态：部分失败（%s项）。其余项目已执行完毕；失败原因和缺失数据需处理。\n' "$((failures - before))" >> "$summary"
    printf '[部分失败] %s，失败%s项；先看：%s\n' "$RUN_DIR" "$((failures - before))" "$summary" >&2
  fi
done
printf '\n本次所有分析进程均已结束；失败项目总数：%s。\n' "$failures"
(( failures == 0 ))

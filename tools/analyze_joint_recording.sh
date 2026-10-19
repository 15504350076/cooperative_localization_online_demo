#!/usr/bin/env bash
# 已加载ROS和cooperative_interfaces环境后运行；只分析本地已解压录包。
# 任一命令失败停止；pipefail确保python失败不会被tee成功掩盖。
set -euo pipefail
# 工具位置按脚本所在目录解析，数据位置按用户参数解析；引用支持路径空格。
TOOLS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $# == 0 ]]; then
  # 不传目录时按实验编号倒序选择最新的、已解压且含metadata.yaml的联合包。
  DATA_DIR="$TOOLS_DIR/../data"
  while IFS= read -r candidate; do
    if compgen -G "$candidate/*_imu_rec_v*/metadata.yaml" >/dev/null &&
       compgen -G "$candidate/*_uwb_rec_v*/metadata.yaml" >/dev/null; then
      set -- "$candidate"
      printf '自动选择最新联合包：%s\n' "$candidate"
      break
    fi
  done < <(find "$DATA_DIR" -mindepth 1 -maxdepth 1 -type d -name 'joint_time_*' -print | sort --reverse)
  if [[ $# == 0 ]]; then
    echo 'data/下没有找到已解压且同时含IMU、UWB metadata.yaml的joint_time_*目录。先解压新包。' >&2
    exit 1
  fi
fi
if [[ ( $# != 1 && $# != 2 ) || ! -d "$1" ]]; then
  echo '用法：bash tools/analyze_joint_recording.sh [联合包目录] [雷达短包目录]' >&2
  echo '不填目录时，自动选择data/下最新的已解压joint_time_*目录。' >&2
  exit 2
fi
RUN_DIR=$(cd -- "$1" && pwd)
LIDAR_BAG=''
if [[ $# == 2 ]]; then
  [[ -f "$2/metadata.yaml" ]] || { echo "雷达包无效：$2" >&2; exit 2; }
  LIDAR_BAG=$(cd -- "$2" && pwd)
fi
# 开始处理前验证环境，缺少ROS或自定义接口时立即报错。
python3 -c 'import rosbag2_py, numpy, matplotlib; from cooperative_interfaces.msg import UwbRange, Identity'
shopt -s nullglob  # 没匹配的通配符展开为空数组，避免把星号当文件名。
IMU_BAGS=("$RUN_DIR"/*_imu_rec_v*)
UWB_BAGS=("$RUN_DIR"/*_uwb_rec_v*)
count=0
# glob也可能命中.log/.tar.gz，只有带metadata.yaml的目录才计为bag。
for bag in "${IMU_BAGS[@]}" "${UWB_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] && count=$((count + 1))
done
if [[ $count == 0 ]]; then
  echo '未找到已解压的 *_imu_rec_v* 或 *_uwb_rec_v* 录包目录。' >&2
  exit 1
fi
# 第一步：原始首末header及连续性；第二步：全样本详细时间统计与绘图。
python3 "$TOOLS_DIR/record_joint_time.py" --analyze "$RUN_DIR"
if [[ -n "$LIDAR_BAG" ]]; then
  python3 "$TOOLS_DIR/analyze_lidar_bag.py" "$LIDAR_BAG"
  python3 "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR" "$LIDAR_BAG"
else
  python3 "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR"
fi
# 第三步：各盒IMU独立分析；tee同时打印和保存，重复运行覆盖派生报告。
for bag in "${IMU_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/analyze_imu_bag.py" "$bag" | tee "$bag/imu_analysis.txt"
done
# 第四步：共享UWB按消息源/目标ID拆分统计，并保存全量CSV和PNG。
for bag in "${UWB_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/export_uwb_bag_readable.py" "$bag"
  # 同一录包盒内按src_id合并同轮重复消息，再比较三个UWB源盒时间。
  python3 "$TOOLS_DIR/compare_uwb_timestamps.py" "$bag"
  # 录包包含在线位置时自动继续分析，不再要求用户手填该轮目录名。
  if grep --quiet 'name: /cooperative_localization/poses_2d' "$bag/metadata.yaml"; then
    python3 -c 'from cooperative_localization_msgs.msg import CooperativePose2DArray'
    python3 "$TOOLS_DIR/analyze_online_bag.py" "$bag"
  fi
done
printf '\n分析完成，共处理%s个包：%s\n' "$count" "$RUN_DIR"
printf '先看联合时间验收.md（含接收回退）和联合时间详细统计.csv，再看各IMU的imu_analysis.txt、UWB测距分析.md及UWB时间对比/三盒UWB时间对比.md。\n'

#!/usr/bin/env bash
# 已加载ROS和cooperative_interfaces环境后运行；只分析本地已解压录包。
# 任一命令失败停止；pipefail确保python失败不会被tee成功掩盖。
set -euo pipefail
if [[ $# != 1 || ! -d "$1" ]]; then
  echo '用法：bash tools/analyze_joint_recording.sh data/joint_time_实验编号' >&2
  exit 2
fi
# 工具位置按脚本所在目录解析，数据位置按用户参数解析；引用支持路径空格。
TOOLS_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$(cd -- "$1" && pwd)
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
python3 "$TOOLS_DIR/analyze_joint_time.py" "$RUN_DIR"
# 第三步：各盒IMU独立分析；tee同时打印和保存，重复运行覆盖派生报告。
for bag in "${IMU_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/analyze_imu_bag.py" "$bag" | tee "$bag/imu_analysis.txt"
done
# 第四步：共享UWB按消息源/目标ID拆分统计，并保存全量CSV和PNG。
for bag in "${UWB_BAGS[@]}"; do
  [[ -f "$bag/metadata.yaml" ]] || continue
  python3 "$TOOLS_DIR/export_uwb_bag_readable.py" "$bag"
done
printf '\n分析完成，共处理%s个包：%s\n' "$count" "$RUN_DIR"
printf '先看联合时间初查.md，再看各IMU的imu_analysis.txt及UWB测距分析.md。\n'

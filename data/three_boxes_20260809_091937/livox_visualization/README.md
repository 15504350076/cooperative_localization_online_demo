# 最新三盒 Livox 点云可视化

数据：`data/three_boxes_20260809_091937`。原始 bag 未修改。

以下命令在本地Ubuntu开发电脑的工程根目录执行：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
```

## 复现

在工程根目录执行；需要 ROS 2 Humble、`sensor_msgs`、`numpy`、`matplotlib`，若要生成交互式 3D HTML 还需要 `plotly`：

```bash
source /opt/ros/humble/setup.bash
python3 tools/visualize_livox_bags.py
```

工具会自动选择`data/`下最新且三盒Livox目录完整的实验。当前数据已经解压，可以直接执行上面的复现命令。只有ZIP时才执行以下解包；已完整解压的目录跳过，避免覆盖已有文件：

```bash
ARCHIVE=$(find data -maxdepth 1 -type f \( -name 'three_boxes_*.zip' -o -name 'three_boxes_*.tar.gz' \) -print | sort --reverse | head -n1)
case "$ARCHIVE" in
  *.zip) RUN=${ARCHIVE%.zip}; [ -d "$RUN" ] || unzip -n -q "$ARCHIVE" -d data ;;
  *.tar.gz) RUN=${ARCHIVE%.tar.gz}; [ -d "$RUN" ] || tar --extract --gzip --keep-old-files --file "$ARCHIVE" --directory data ;;
esac
for archive in "$RUN"/uwb_imu_livox_v*.tar.gz; do
  BOX_DIR=${archive%.tar.gz}
  [ -d "$BOX_DIR" ] || tar --extract --gzip --keep-old-files --file "$archive" --directory "$RUN"
done
source /opt/ros/humble/setup.bash
python3 tools/visualize_livox_bags.py
```

输出仍写入本目录，覆盖PNG/HTML/summary.json；本README保留。可通过`--output /tmp/livox-preview`写到其他目录。HTML内嵌所需JavaScript，浏览器打开后可以拖动旋转、滚轮缩放，无需互联网。IDE文本编辑器显示HTML源码时，使用系统浏览器打开文件。

工具按SQLite `messages.id`写入顺序读取窗口，过滤非有限XYZ与全零点；只支持单文件`livox_0.db3`格式。无参数运行时会自动选择最新且三盒Livox包完整的实验；若该实验帧数不足本轮固定窗口，会自动取该盒最后一段可用帧。显式传入本目录时仍复现本轮固定窗口。点云累积没有运动补偿，窗口内移动可能产生重影。

## 在RViz2中回放整段点云

先开RViz，再播放bag，两个终端使用相同通信环境。这里只查看本机回放，不需要盒子在线，也不需要启动定位算法。

终端1：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
rviz2
```

在RViz2中设置：

1. `Global Options → Fixed Frame`：`livox_frame`。
2. `Add → PointCloud2`，Topic选择`/livox/lidar`。
3. Topic下的`Reliability Policy`选择`Best Effort`，`Durability Policy`选择`Volatile`。
4. `Position Transformer`选择`XYZ`；`Color Transformer`选择`AxisColor`，`Axis`选择`Z`；`Style`选择`Points`，`Size (Pixels)`设为2或3。
5. `Decay Time`先设为`1`秒，让201的96点小消息累积成可见场景；202/203也可设为`0`只看当前消息。累积时间越长，移动物体越容易产生重影。

终端2：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
BOX=201
RUN=$(find data -maxdepth 1 -type d -name 'three_boxes_*' -print | sort --reverse | head -n1)
BAG=$(find "$RUN" -maxdepth 1 -type d -name "uwb_imu_livox_v${BOX}_*" -print -quit)/livox
ros2 bag play "$BAG" \
  --topics /livox/lidar --rate 1.0 --loop
```

终端2按`Ctrl+C`停止，再把`BOX`改为202或203，重新执行播放命令即可切换盒子。`--rate 0.5`可半速，去掉`--loop`则只播放一次；看完在两个终端分别按`Ctrl+C`。保存RViz配置可用`File → Save Config As`，下次用`rviz2 -d <保存的.rviz文件>`打开。

本轮bag记录时间存在回退，ROS播放器按bag记录时间安排播放，顺序和时长可能与采集写入顺序不同。原始Header仍保留；回放画面用来查看空间结构，Header回退和同步判断仍以原有分析报告为准。RViz直接显示原始消息，原点处可能聚集全零点；PNG/HTML会过滤这些点。

三盒的`frame_id`都叫`livox_frame`，各自代表本盒雷达，且没有跨盒外参配准。建议一次播放一盒；同时播放时需用`--remap /livox/lidar:=/box201/livox/lidar`等方式区分话题，并逐个启用显示。不能将叠加画面当成公共坐标地图。

## 结果与验证

- `livox_overview.png`：每盒 XY 俯视图和 XZ 侧视图。
- `box201_livox_3d.html`、`box202_livox_3d.html`、`box203_livox_3d.html`：浏览器交互式 3D 点云。
- `summary.json`：抽样窗口和绘图统计。

选取规则：201 因每帧只有 96 点，取第 9931 帧附近约 225 帧；202/203 各取第 448/897 帧附近 10 帧。
绘图过滤 XYZ 全零点，并对超过 120000 点的数据做固定随机抽样；图像用于观察形状，不代表完整扫描或跨盒坐标配准。

重新生成后，`summary.json`中三盒`nonzero_finite_points_plotted`应分别为10417、84757、100081。过滤与读取顺序的回归检查：

```bash
source /opt/ros/humble/setup.bash
python3 -B tools/test_visualize_livox_bags.py
```

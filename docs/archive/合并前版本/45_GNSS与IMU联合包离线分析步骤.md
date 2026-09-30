# GNSS、IMU 单独与联合包离线分析步骤（已合并）

本文件已合并到 `docs/30_record_joint_time_使用说明.md` 第10节，仅保留用于追溯。

本文件统一处理 201 盒的 GNSS 单独包、IMU 单独包，以及 GNSS+IMU 联合包。它不会修改 `.db3`，只在数据目录中生成报告和 CSV。原来的单独 GNSS 步骤已合并到本文件；历史原文保存在 `archive/历史记录/44_GNSS户外静止包离线分析步骤.md`。

## 1. 准备环境

在工程根目录执行：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/real-data-ros-adapter/setup.bash
```

如果 `install/real-data-ros-adapter/setup.bash` 不存在，换成已构建出 `cooperative_interfaces` 的工作空间环境。

## 2. 分析联合包

联合包应包含：

```text
gnss_imu_v1_static_20260927_115141/
├── gnss/       # /gnss/fix
├── imu/        # /imu/raw
└── record_info.txt
```

运行：

```bash
python3 tools/analyze_gnss_imu_bag.py \
  data/gnss_imu_v1_static_20260927_115141 \
  | tee data/gnss_imu_v1_static_20260927_115141/分析过程.txt
```

脚本会生成：

```text
GNSS与IMU联合时间分析.md   # 总结结论
GNSS与IMU时间对比.csv      # 每条GNSS对应的最近IMU时间
```

CSV 中最重要的字段：

- `nearest_imu_header_minus_gnss_header_ms`：最近 IMU 测量时间减 GNSS 测量时间；
- `nearest_imu_receive_minus_gnss_receive_ms`：最近 IMU 录包接收时间减 GNSS 录包接收时间。

## 3. 分别分析两路数据

### GNSS

```bash
python3 tools/analyze_gnss_bag.py \
  data/gnss_imu_v1_static_20260927_115141/gnss
```

查看：

```text
data/gnss_imu_v1_static_20260927_115141/gnss/GNSS户外静止分析.md
data/gnss_imu_v1_static_20260927_115141/gnss/GNSS数据.csv
```

### IMU

```bash
python3 tools/analyze_imu_bag.py \
  data/gnss_imu_v1_static_20260927_115141/imu \
  | tee data/gnss_imu_v1_static_20260927_115141/imu/imu_analysis.txt
```

## 4. 怎么判断

先看单流：

1. Header 时间是否非零；
2. 是否回退、重复或出现长间隔；
3. GNSS 状态是否有效、位置是否稳定；
4. IMU 是否约 1000 Hz；
5. IMU 静止角速度和加速度是否有限。

再看联合时间表：

- 最近 Header 时间差长期接近 0，说明两路消息时间可以直接配对；
- 差值在约 ±0.5 ms 内变化，通常主要是 GNSS 1 Hz 样本落在 IMU 1 ms 采样网格之间造成的最近邻量化；
- 若差值随时间持续增大，说明两路时钟可能存在频率偏差；
- 若出现秒级、天级偏移，说明时间基准没有统一；
- `receive - header` 不是纯网络延迟，不能单独作为硬件同步精度。

GNSS 只有约 1 Hz，IMU 约 1000 Hz，因此不能要求一条 GNSS 消息和一条 IMU 消息“同一序号”。脚本使用最近时间配对，并把原始时间完整保留在 CSV 中。

## 5. 当前这组数据的文件

```text
data/gnss_imu_v1_static_20260927_115141/
├── GNSS与IMU联合时间分析.md
├── GNSS与IMU时间对比.csv
├── 分析过程.txt
├── gnss/GNSS户外静止分析.md
└── imu/imu_analysis.txt
```

单独的 `gnss_v1_outdoor_static_20260927_105736` 使用同一个 `analyze_gnss_bag.py` 分析即可。

## 6. 单独 GNSS 包

如果只有 `/gnss/fix`，在工程根目录加载 ROS 2 和消息工作空间后运行：

```bash
python3 tools/analyze_gnss_bag.py \
  data/gnss_v1_outdoor_static_20260923_054648
```

脚本默认读取 `/gnss/fix`，并在原目录生成：

```text
GNSS户外静止分析.md   # 统计和结论
GNSS数据.csv          # 接收时间、消息时间、位置和状态
GNSS户外静止分析.png  # 时间、散布、延迟和间隔图
```

重点检查：消息数量和频率、`node_id`、有效状态、`header.stamp` 是否回退、接收间隔、位置散布，以及接收时间与消息时间的变化。没有独立真值时，这些结果只能说明静止重复性，不能称为绝对定位精度。

如果 GNSS Topic 被改名，显式指定：

```bash
python3 tools/analyze_gnss_bag.py data/某个bag --topic /新的gnss_topic
```

若还要同时分析本机 IMU，不要手工拼接两个独立包；使用本文件第 2 节的联合包目录和 `analyze_gnss_imu_bag.py`。

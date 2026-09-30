# GNSS 户外静止包离线分析步骤

本文件用于没有 AI 辅助时，自己分析 201 户外 GNSS 录包。脚本只读 bag，不会修改原始 `.db3`。

本文件内容已合并到[30号录包与离线分析说明](../../30_record_joint_time_使用说明.md)第10节。如果同时录了 GNSS 和 IMU，不要把两个独立包的报告手工拼接。

## 1. 分析前准备

在工程根目录执行。先加载 ROS 2 和包含 `cooperative_interfaces` 的工作空间：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/real-data-ros-adapter/setup.bash
```

如果这个快照目录不存在，可换成其他已经构建出 `cooperative_interfaces` 的 `install/*` 工作空间。

## 2. 分析指定 bag

例如分析本次数据：

```bash
python3 tools/analyze_gnss_bag.py \
  data/gnss_v1_outdoor_static_20260923_054648
```

脚本默认读取 `/gnss/fix`，并在原 bag 目录生成：

```text
GNSS户外静止分析.md   # 自动生成的结论和统计表
GNSS数据.csv          # 每一帧的接收时间、消息时间、位置和状态
GNSS户外静止分析.png  # 时间、散布、延迟和消息间隔图
```

如果 GNSS Topic 被上交改名，可以显式指定：

```bash
python3 tools/analyze_gnss_bag.py data/某个bag --topic /新的gnss_topic
```

## 3. 重点看哪些结果

打开生成的 `GNSS户外静止分析.md`，按以下顺序判断：

1. `/gnss/fix` 条数是否接近录制时长乘以发布频率；
2. `node_id` 是否为 1；
3. `status` 是否长期为 `-1`；
4. 有效帧比例是否足够高；
5. `header.stamp` 是否有回退或重复；
6. 接收相邻间隔和消息相邻间隔是否稳定；
7. `接收时间 - 消息时间` 是否有突跳；
8. 东向、北向散布和高度散布是否稳定。

脚本同时输出两种时间：

- `receive_ns`：rosbag 记录器写入该消息时的接收时间；
- `message_ns`：`/gnss/fix.header.stamp` 中的消息时间。

只有上交确认二者属于同一时间域时，`receive_minus_message_ms` 才能解释为接收/处理延迟。它不能单独证明 GNSS PPS 或 UWB 授时精度。

## 4. 本次包的分析对象

本次目录为：

```text
data/gnss_v1_outdoor_static_20260923_054648
```

原始 bag 只包含：

- `/gnss/fix`：179 条 `cooperative_interfaces/msg/GnssPosition`；
- `/node/id`：179 条 `cooperative_interfaces/msg/Identity`。

没有 `/imu/raw`，所以本包不能评价 GNSS 与 IMU 的时间对应关系；也没有三车 UWB 测距，因此不能评价三车相对定位。

## 5. 注意事项

- 没有测量基准点或独立 RTK Fixed 真值时，脚本计算的是静止重复性，不是绝对定位误差。
- `status=0` 不足以证明 RTK Fixed，必须让上交提供明确的解类型或质量字段。
- 201 单独在户外时没有 `/uwb/range` 不等于 UWB 驱动故障，UWB 测距需要其他标签在通信范围内。
- 原始目录和压缩包都要保留，分析结果可以重新生成。

## 6. 同时录制 GNSS 和 IMU

只有201有GNSS时，仍然可以在201上同步录制本机GNSS和本机IMU。把脚本上传到201：

```bash
scp tools/record_201_gnss_imu.py \
  firefly@192.168.1.201:/home/firefly/gnss_test/
```

也可以用MobaXterm的SFTP窗口把该文件放到同一目录。登录201后执行：

```bash
source /opt/ros/humble/setup.bash
source /home/firefly/imu_ws/install/setup.bash
python3 /home/firefly/gnss_test/record_201_gnss_imu.py --duration 180
```

脚本会先分别检查共享DDS下的`/gnss/fix`和本机DDS下的`/imu/raw`，然后同时发起两个`ros2 bag record`进程。两路包分开保存，结束后自动生成一个压缩包：

```text
/home/firefly/gnss_test/gnss_imu_v1_static_<时间>.tar.gz
```

录制时保持201静止。两个录包进程由同一个脚本在同一启动屏障后发起；最终是否同一时间轴，仍以两个 bag 中各自的`header.stamp`比较为准。

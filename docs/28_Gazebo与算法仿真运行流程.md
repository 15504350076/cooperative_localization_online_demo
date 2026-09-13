# 28 Gazebo 主流程与算法精度测量

在开发电脑的 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 环境执行，不需要连接实物盒子。

**日常只执行步骤 1～4。** 步骤 5 仅在需要检查静止 IMU 初始化时执行；不再把另一套无 Gazebo 仿真混入日常流程。打开两个终端，每个代码块从 `cd` 开始完整复制。环境加载只对当前终端生效。

两套仿真的关系：Gazebo 是主流程，负责物理场景和传感器链路；`simulation.launch.py` 只是排查算法时可用的替代输入，和 Gazebo 二选一，日常不需要运行。

## 1. 终端 1：编译

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash

cmake -S . -B build-stage-current \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DCMAKE_INSTALL_PREFIX="$PWD/stage/zju_coop" &&
cmake --build build-stage-current --parallel &&
cmake --install build-stage-current &&
colcon --log-base log/ros2-sim-check-current-build build \
  --base-paths ros2 \
  --build-base build/ros2-sim-check-current \
  --install-base install/ros2-sim-check-current \
  --packages-up-to zju_coop_gazebo zju_coop_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DZJU_COOP_SDK_ROOT="$PWD/stage/zju_coop"
```

看到 colcon 的 `Summary: ... packages finished` 且没有失败包，再继续。

## 2. 终端 1：启动 Gazebo 和算法

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
export ROS_DOMAIN_ID=169
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

ros2 launch zju_coop_gazebo three_vehicle_gazebo.launch.py
```

这条命令已包含 Gazebo、三车 IMU/UWB 和定位算法，不要再同时启动 `simulation.launch.py`。窗口打开后等待约 5 秒，三车自动开始运动。**保持终端 1 运行，不要按 `Ctrl+C`。**

## 3. 终端 2：直接输出定位精度

Gazebo 运行约 5 秒后，在终端 2 完整执行下面命令。程序采集 10 秒，将算法输出和仿真真值按时间戳配对，直接打印三车的数值精度。

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
export ROS_DOMAIN_ID=169
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

python3 tools/measure_simulation_accuracy.py --duration 10
```

输出包括每辆车的位置误差 RMSE、平均值、最大值（米），以及航向误差 RMSE、平均值、最大值（度）。如果没有收到算法输出或真值，程序会直接提示原因。

## 4. 终端 1：停止 Gazebo 和算法

在终端 1 按 `Ctrl+C`，等待启动的节点退出并返回命令提示符。本次 Gazebo 最简测试结束。

## 5. 可选：验证静止 IMU 初始化

停止仿真后，在终端 1 完整执行。该测试自行提供静止 IMU 数据，不需要启动 Gazebo：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
export ROS_DOMAIN_ID=169
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1

ctest --test-dir build/ros2-sim-check-current/zju_coop_ros2 \
  -R '^test_test_static_imu_initialization.launch.py$' --output-on-failure
```

看到 `100% tests passed` 表示该专项测试完成。这项测试检查启动静止估计和恒定陀螺零偏扣除，不代表实测精度。

## 仿真消息与上交消息的对应关系

**消息类型和字段与上交已提供的接口一致，但坐标和时间来源不同。**

| 项目 | 当前 Gazebo 仿真 | 上交盒端 |
| --- | --- | --- |
| IMU 消息类型 | `sensor_msgs/msg/Imu` | 相同 |
| IMU Topic | `/vehicle_1/imu/data`、`/vehicle_2/imu/data`、`/vehicle_3/imu/data` | 每盒本地 `/imu/raw` |
| IMU 坐标 | 车体前左上 FLU、导航 ENU | 已确认车体前右下 FRD、AHRS 导航 NED，需转换 |
| UWB 类型与 Topic | `cooperative_interfaces/msg/UwbRange`，`/uwb/range` | 相同 |
| UWB 字段 | `header、uint32 src_id、uint32 target_id、float32 distance` | 相同；不含标准差或 NLOS 标志 |
| 时间来源 | 统一仿真时间 `/clock` | 设备授时，目标是三盒统一的 `UWB_SYSTEM_TIME` |

角速度按 `rad/s`、加速度按 `m/s²`、距离按米使用。Gazebo 的 UWB 距离由三车真值位置计算。历史实包 UWB 的 `frame_id` 是 `uwb_link`，仿真是 `common_enu`；仿真 UWB 使用 Best Effort，已采上交实包使用 Reliable。

因此当前能检查 IMU＋UWB 算法是否收到数据并输出结果；上交原始 FRD/NED 转换和三盒硬件授时还要另外验证。仿真给定了共同初始航向，不能据此认为三条 UWB 距离能测出车头朝向。

## 本次最简运行记录

2026-09-06 按上述默认命令启动带窗口的 Gazebo，三路 IMU 均出数。同一仿真时刻 `21.580000000 s` 收到：

| 测距边 | 距离（米） |
| --- | --- |
| 1—2 | 3.947398 |
| 1—3 | 9.249624 |
| 2—3 | 11.973040 |

随后收到三车定位结果，各车 `position_valid=true、yaw_valid=true`。这是出数检查，不是定位精度评估。

停止时算法节点正常退出；Gazebo GUI 在图形库释放过程中报告段错误，退出问题尚未处理。

# 28 Gazebo 主流程与算法精度测量

在开发电脑的 Ubuntu 22.04、ROS 2 Humble、Gazebo Fortress 环境执行，不需要连接实物盒子。

**日常只执行步骤 1～4。** 步骤 5 仅在需要检查静止 IMU 初始化时执行；不再把另一套无 Gazebo 仿真混入日常流程。打开两个终端，每个代码块从 `cd` 开始完整复制。环境加载只对当前终端生效。

两套仿真的关系：Gazebo 是主流程，负责物理场景和传感器链路；`simulation.launch.py` 只是排查算法时可用的替代输入，和 Gazebo 二选一，日常不需要运行。

当前三车场景是仓库式测试环境：包含仓库外壳和门洞、货架货箱、托盘、输送线、安全护栏、地面导向线、顶梁灯具，以及三辆带 IMU/UWB/LiDAR/相机外观的车辆。场景仍使用本地 SDF，不依赖下载模型；UWB 仍是算法必需输入。

三车不是在空地画圆：场景节点会按闭环航点控制车辆。1 号车进入中部货架巷道，2 号车经过东侧输送线和装卸区，3 号车穿过两排货架之间的通道。启动后约 30～50 秒即可看到车辆进入障碍物群；长时间运行会重复巡航。

场景节点还根据已知的静态障碍物位置和三车 Gazebo 位姿做安全层：接近货架、托盘、输送线时减速绕行，车辆互相接近时按车号让行并转向避让。它用于仿真测试安全性，不等同于真实车上的 LiDAR 自主避障规划器。

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
export ROS_LOCALHOST_ONLY=1

ros2 launch zju_coop_gazebo three_vehicle_gazebo.launch.py
```

这条命令已包含 Gazebo、三车 IMU/UWB 和定位算法，不要再同时启动 `simulation.launch.py`。窗口打开后首次渲染可能需要 20～30 秒（软件渲染时更明显），看到仓库和三车后再开始操作。**保持终端 1 运行，不要按 `Ctrl+C`。**如果仍只看到实体树而看不到场景，确认没有复用旧的 Gazebo 进程，再重新执行本节启动命令；当前默认使用软件渲染以避免空白视口。

需要同时显示三车的激光雷达和相机数据时，把启动命令改为：

```bash
ros2 launch zju_coop_gazebo three_vehicle_gazebo.launch.py \
  enable_lidar:=true enable_camera:=true
```

默认启动即可得到三车 IMU、仿真 UWB 和 `/cooperative_localization/poses_2d`；加上上面的参数后再得到三车 LiDAR 点云和相机 `Image/CameraInfo`。定位真值同时发布在 `/simulation/ground_truth/poses_2d`，用于验收算法误差。

当前版本没有发布 GNSS 话题；`/simulation/ground_truth/poses_2d` 是真值，不是 GNSS 传感器。若要验证 GNSS 接入，需要另加 `NavSatFix` 仿真节点，不能把真值话题当作硬件 GNSS。

## 外部高保真环境候选

- 公路/城市：优先 CARLA。官方 ROS bridge 支持多车控制，以及 LiDAR、相机、GNSS、IMU 等传感器；UWB 需要由本工程根据车辆真值自行生成。它依赖 Unreal Engine 和较强 GPU，不建议替换当前离线 Gazebo 流程。
- ROS 2 原生道路数字孪生：可评估 Autoware Foundation 的 AWSIM。它使用 Unity、ROS 2 原生通信和道路数字孪生，适合道路交通场景，但需要重新接入本工程的 UWB 和三车模型。
- 仓库/平地/森林：继续使用 Gazebo，并从 Gazebo Fuel 引入网格、植被、地形或仓库模型后固定到本地资源路径。这样保留现有 UWB、算法和离线运行能力，但照片级效果取决于所选模型材质。
- Webots 更轻量，适合快速多机器人传感器验证，但视觉真实度低于 CARLA/AWSIM。

建议：Gazebo 作为算法回归基线，CARLA/AWSIM 作为高保真视觉与道路数据源，二者通过 ROS 2 适配层复用本工程算法；不要直接把算法核心绑死在某个仿真器上。

## 3. 终端 2：直接输出定位精度

Gazebo 运行约 5 秒后，在终端 2 完整执行下面命令。程序采集 10 秒，将算法输出和仿真真值按时间戳配对，直接打印三车的数值精度。

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
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
  --tests-regex '^test_test_static_imu_initialization.launch.py$' --output-on-failure
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
| UWB 字段 | `header、uint32 src_id、uint32 target_id、uint16 rolling、float32 distance` | 相同；`rolling`置0且算法忽略，不含标准差或 NLOS 标志 |
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

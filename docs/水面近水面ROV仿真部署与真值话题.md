# 水面/近水面 ROV 仿真

本工程新增了一个与 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress 兼容的近水面 ROV 场景，位置在 `ros2/zju_coop_gazebo`。它使用 Gazebo Fortress 自带的分层浮力和水动力系统，避免引入当前 ROS 发行版无法直接编译的 Gazebo Jetty/Lyrical 项目。

## 启动

在工作区完成 ROS 2 包构建后运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch zju_coop_gazebo surface_rov_gazebo.launch.py
```

无图形界面运行：

```bash
ros2 launch zju_coop_gazebo surface_rov_gazebo.launch.py \
  headless:=true software_rendering:=true
```

场景位于 `worlds/surface_rov.sdf`，ROV 模型位于 `models/zju_surface_rov`，俯视高速相机位于 `models/zju_truth_camera`。

## 主要话题

| 用途 | ROS 2 话题 | 频率/类型 |
|---|---|---|
| ROV IMU | `/surface_rov/imu/data` | 100 Hz，`sensor_msgs/msg/Imu` |
| ROV 前视相机 | `/surface_rov/camera/front/image_raw` | 30 Hz，按需启用 |
| 外部高速相机 | `/surface_rov/camera/overhead/image_raw` | 120 Hz，按需启用 |
| 外部相机内参 | `/surface_rov/camera/overhead/camera_info` | `sensor_msgs/msg/CameraInfo` |
| ROV 姿态真值 | `/simulation/ground_truth/poses` | `tf2_msgs/msg/TFMessage` |
| 左推进器 | `/surface_rov/thrusters/left/cmd_thrust` | `std_msgs/msg/Float64`，N |
| 右推进器 | `/surface_rov/thrusters/right/cmd_thrust` | `std_msgs/msg/Float64`，N |

例如向两个推进器发送相同推力：

```bash
ros2 topic pub --once /surface_rov/thrusters/left/cmd_thrust \
  std_msgs/msg/Float64 '{data: 20.0}'
ros2 topic pub --once /surface_rov/thrusters/right/cmd_thrust \
  std_msgs/msg/Float64 '{data: 20.0}'
```

## 真值与相机观测的边界

`/simulation/ground_truth/poses` 来自 Gazebo 的物理模型状态，作为评估姿态误差的真值。高速相机只提供图像观测；相机图像应经过 AprilTag、PnP、特征跟踪或视觉惯导算法后形成估计姿态，不能直接把图像姿态当作真值。

俯视相机安装在 z=8 m，ROV 甲板上有红、绿、蓝、黄四个视觉标记，便于后续编写外部视觉定位节点。相机设置为 `always_on=false`，没有订阅者时不会消耗 120 FPS 的渲染开销。

## 当前模型边界

- 已包含平面水面、空气/水分层浮力、6 自由度水动力阻尼、双推进器、IMU、机载相机、外部高速相机和物理位姿真值。
- 当前水面是平面，波浪只作为后续扩展点；它适合先验收姿态、时间戳、视觉观测和算法接口，不应直接作为复杂海况或波浪耦合精度结论。
- 水动力系数是可运行的初始值，后续应使用实艇参数、拖曳试验或 CFD/实验数据标定。

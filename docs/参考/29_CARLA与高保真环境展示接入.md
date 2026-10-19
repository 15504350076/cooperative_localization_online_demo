# CARLA/AWSIM 高保真展示接入

## 目标

CARLA 或 AWSIM 负责道路、车辆、相机、LiDAR、IMU 和可视化；本工程负责 UWB 测距模拟、协同定位和结果显示。UWB 不要求仿真器原生实现。

## 接口

高保真仿真器需向 ROS 2 提供三路 `nav_msgs/msg/Odometry`，默认话题：

```text
/carla/vehicle_1/odometry
/carla/vehicle_2/odometry
/carla/vehicle_3/odometry
```

启动本工程的 UWB 适配节点：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch zju_coop_gazebo carla_uwb_bridge.launch.py
```

输出：

```text
/uwb/range
/simulation/ground_truth/poses_2d
```

默认 UWB 距离标准差为 0.02 m。演示丢包时可改为：

```bash
ros2 launch zju_coop_gazebo carla_uwb_bridge.launch.py \
  range_noise_std_m:=0.08 dropout_probability:=0.05
```

如果仿真器话题名称不同，只改启动参数，不改节点：

```bash
ros2 launch zju_coop_gazebo carla_uwb_bridge.launch.py \
  odom_topic_1:=/carla/ugv_1/odometry \
  odom_topic_2:=/carla/ugv_2/odometry \
  odom_topic_3:=/carla/ugv_3/odometry
```

## 当前边界

- 适配节点不依赖 CARLA Python 包，当前开发机没有独显或 CARLA 时仍可编译检查。
- 里程计坐标必须已经统一到同一平面坐标系；若仿真器使用不同轴向，应在适配节点前转换。
- 当前输出的是“真值位置计算的 UWB 距离”，后续可加入遮挡、NLOS、范围限制和时间延迟。
- 这不是 GNSS 输入；`/simulation/ground_truth/poses_2d` 只用于误差评估。

## 展示建议

正式演示使用 CARLA/AWSIM 的道路场景；本地 Gazebo 继续作为无网、可重复的算法回归环境。展示界面至少同时显示三车轨迹、`position_valid`、UWB 状态和真值误差。

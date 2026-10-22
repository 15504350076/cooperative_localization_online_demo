# 三车 RTE 初始化与 IMU-UWB 连续修正链路

本链路是独立的可选算法路径，用于把三辆车各自的本地 IMU 航位推算与车间 UWB 测距组合起来。原有 `uwb_only` 三边测距恢复相对三角形的程序和默认参数没有改动；只有显式启用 RTE 构建和节点时才会使用本文功能。

## 当前实现的含义

每辆车仍由现有 `zju_local_inertial_node` 运行本地 15 维 ESKF，发布带时间戳的 `NodeState`。新的 `zju_three_vehicle_rte_node` 收集三辆车的本地位置、速度和车体姿态，同时收集三条 UWB 边：1-2、1-3、2-3。

初始化阶段先分别求每条边的相对变换，再用三条边联合优化三车变换。默认平面模式以 1 号车为参考车，使用已知的本地原点高度，求出 2、3 号车相对于 1 号车的平移和航向；三条边共同参与残差、条件数和不确定度检查。

初始化通过后，节点持续用滑动窗口中的本地 IMU 航位推算和新 UWB 测距重复优化三车变换，并发布当前相对位置、相对速度和车体航向。这个结果会修正相对变换输出，但不会把协同结果回灌到每辆车的本地 ESKF；因此可以随时停用新节点而不改变原本地惯导和 `uwb_only` 程序。

启用 `planar: false` 时使用空间变换和三维测距模型。真实三维场景必须有足够的高度变化或其他垂向约束；平面静止数据不能可靠估计完整空间姿态。

## 构建

```bash
cmake -S . -B build-rte -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DZJU_COOP_BUILD_RTE=ON
cmake --build build-rte -j2
./build-rte/zju_coop_three_rte_self_check
cmake --install build-rte --prefix install-rte-three
```

ROS 2 Humble 环境中，把安装目录交给适配包：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-uwb-online/setup.bash
colcon --log-base log/ros2-rte-three build \
  --base-paths ros2/zju_coop_ros2 ros2/zju_coop_bringup \
  --packages-select zju_coop_ros2 zju_coop_bringup \
  --build-base build/ros2-rte-three --install-base install/ros2-rte-three \
  --cmake-args -DZJU_COOP_SDK_ROOT="$PWD/install-rte-three" \
  -DZJU_COOP_ROS2_BUILD_RTE=ON -DBUILD_TESTING=ON
```

## 合成三车回放

不接硬件时可直接运行内置合成发布器：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-rte-three/setup.bash
ros2 launch zju_coop_bringup rte_three_vehicle_demo.launch.py
```

它发布三路合成 `sensor_msgs/Imu` 和三条 `UwbRange`，每辆本地节点发布 `NodeState`，RTE 节点发布：

```text
/rte_demo/poses_2d     cooperative_localization_msgs/CooperativePose2DArray
/rte_demo/status       std_msgs/String
```

状态 JSON 中的 `initialized`、`active_edges`、`sample_counts`、`rms_residual_m`、`translation_std_m` 和 `yaw_std_rad` 用于判断是否已经完成三车初始化及当前质量。停止测距或冻结某辆车后，状态会进入 stale；恢复输入并重新满足窗口条件后会自动恢复。

## 接入真实三盒数据

真实接入时必须保证：

1. 三辆车的 `NodeState.header.stamp` 与 UWB 使用同一 `UWB_SYSTEM_TIME` 纳秒时间轴，且每辆车的时间戳严格递增。
2. 新 RTE 节点收到的 `NodeState.header.frame_id` 与参数 `local_frame_prefix + node_id` 一致，例如 `rte_odom_1`、`rte_odom_2`、`rte_odom_3`。可用 `rte_vehicle.launch.py` 为每辆本地节点设置该帧名。
3. `node_ids[0]` 是参考车。`local_origin_heights_m` 填各本地里程计原点的高度差；平面模式下高度差必须来自外部标定，不能用测距自动猜测。
4. `antenna_lever_arms_body_m` 按每辆车的 FLU 车体系填写，未知时可置零但会引入系统误差。
5. 先确认 `status` 为 `initialized=true` 且误差门限通过，再把 `poses_2d` 交给 GCS 或规划模块。

单车本地 IMU 话题仍是 `sensor_msgs/Imu`，RTE 节点不读取原始 IMU，也不改变原来的 IMU 轴系转换和本地滤波。新路径使用的是本地节点输出的航位推算状态，因此可以在保留原链路的同时逐步接入三车协同修正。

## 与原有路径的关系

| 路径 | 启动方式 | 作用 |
| --- | --- | --- |
| 原有 `uwb_only` | `uwb_only.launch.py` 或旧在线程序 | 仅用三边 UWB 恢复规范相对三角形，默认行为 |
| 现有本地惯导 | `zju_local_inertial_node` | 每车独立 IMU 航位推算，原接口不变 |
| 新三车 RTE | `rte_three_vehicle_demo.launch.py` 或单独启动 `zju_three_vehicle_rte_node` | 三边 UWB 对三车本地航位推算做相对变换初始化和连续修正 |

新库的 C 接口集中在 `include/zju_coop/three_vehicle_rte_api.h`，方法名带 `_rte` 后缀；库名为 `zju_coop_rte`。这条路径没有替换或重命名旧 `zju_coop` 接口。

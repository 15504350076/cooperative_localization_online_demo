# 26 静止 IMU 初始化与三车 UWB 三角形

## 1. 本版结论

本版新增两项彼此独立的核心算法：

1. 静止 IMU 窗口解算 roll、pitch 和启动陀螺零偏；
2. 三个普通单标签 UWB 在同一历元给出三条边长后，恢复标签点构成的二维三角形。

两项功能都不产生绝对 yaw。重力没有水平指向信息；三条距离也没有北向、地图或车体朝向信息。

## 2. 静止 IMU 初始化

实现位置：

- `src/core/inertial_eskf15.hpp`：输入、配置、返回值和处置枚举；
- `src/core/inertial_eskf15.cpp`：`initialize_static_imu()`；
- `include/zju_coop/c_api.h`、`src/api/c_api.cpp`：稳定 C ABI；
- `ros2/zju_coop_ros2/src/local_inertial_node.cpp`：启动窗口缓存、静止失败重试和成功后惯导配置；
- `tests/test_inertial_eskf15.cpp`：水平化、零偏均值、样本不足和非静止拒绝测试。

输入 `StaticImuSample` 必须已经转换为本工程内部 FLU/SI 口径：

```text
angular_velocity_rad_s : rad/s
specific_force_m_s2    : m/s²，静止水平时 +Z 约为 +g
```

调用示意：

```cpp
zju::coop::StaticImuInitializationConfig config;
config.minimum_sample_count = 200;
config.initial_yaw_rad = 0.0;  // 仅固定水平自由度，不代表北向

const auto result = zju::coop::initialize_static_imu(samples, config);
if (result.disposition ==
    zju::coop::StaticImuInitializationDisposition::kReady) {
  node_initialization.orientation_b_to_n = result.orientation_b_to_n;
  node_initialization.gyro_bias_rad_s = result.gyro_bias_rad_s;
}
```

处理顺序：

1. 用 Welford 算法计算角速度、比力的三轴均值和样本标准差；
2. 检查样本数、重力模长误差、平均角速度模长和逐轴标准差；
3. 由平均比力计算 roll/pitch；
4. 保留调用方给定的任意初始 yaw；
5. 以平均角速度作为本次启动的陀螺零偏初值。

不能从一个静止姿态可靠分离三轴加计零偏和倾斜误差，所以结果没有输出加计零偏。默认门限是保守工程起点，不是当前盒子已经标定完成的参数；实测后应按采样率、噪声和振动调整。

## 3. 三条距离恢复三角形

实现位置：

- `src/core/range_ekf.hpp`：`ThreeNodeRanges`、`ThreeNodeFormation`；
- `src/core/range_ekf.cpp`：`solve_three_node_formation()`；
- `tests/test_range_ekf.cpp`：3-4-5 三角形、三角不等式、重复节点和近共线拒绝测试。

输入顺序定义坐标系：

```text
node_ids = [A, B, C]
A = (0, 0)
B = (dAB, 0)
C 位于 +Y 半平面
frame = formation_<A>_<B>
```

闭式解为：

```text
xC = (dAB² + dAC² - dBC²) / (2 dAB)
yC = +sqrt(dAC² - xC²)
```

代码先按最长边归一化再计算平方，降低极端大数的中间上溢风险；随后检查三角不等式和最小高度，拒绝不可能或近共线的几何。

调用示意：

```cpp
const zju::coop::ThreeNodeRanges ranges{{1, 2, 3}, d12, d13, d23};
const auto formation = zju::coop::solve_three_node_formation(ranges);
if (formation.disposition == zju::coop::FormationDisposition::kSolved) {
  // formation.points 可作为纯测距 RangeEkf 的规范初始位置。
  // interior_angles_rad 是 A/B/C 三个内角。
  // bearing_01/02/12_rad 是三条有向连线相对人为 +X 的方向。
}
```

`C` 放在 `+Y` 只是固定镜像分支。真实场景在相同三条距离下还存在关于 AB 的镜像解，算法没有传感信息可以判断哪一侧才是现实中的左/右。

## 4. 能输出和不能输出的量

| 量 | 纯三边 UWB 是否可得 | 说明 |
|---|---|---|
| 三个标签点的相对二维位置 | 可以 | 在人为 `formation_A_B` 坐标系内 |
| 三角形三个内角 | 可以 | 由三条边唯一确定，退化三角形除外 |
| 连线相对人为 +X 的方向角 | 可以 | 是标签间连线角，不是车辆 yaw |
| 三角形相对北向/地图的旋转 | 不可以 | 缺少外部方向锚定 |
| 任一车辆自身朝向 | 不可以 | 每车只有一个标签，转动车身不改变标签间瞬时距离 |
| UWB 标签点到车体中心 | 不可以直接得到 | 需要测量安装杆臂并结合有效车体姿态 |
| 绝对位置 | 不可以 | 缺少已知坐标锚点 |

如果只要求“看三车相互队形”，这个规范解已经是最小闭环；如果要叠加到地图或使用车头箭头，必须增加外部位置/方向来源或每车多标签几何。

## 5. 与在线工程的衔接状态

静止 IMU 初始化已经接入本地 ROS 2 惯导节点。三车盒端配置默认启用 10 秒启动窗口；窗口未通过时不配置惯导、不发布 `NodeState`，窗口出现运动或超过 0.1 秒断流时清空后重采。通过后将重力 roll/pitch 和本次启动陀螺零偏写入惯导初值，yaw 仍使用配置的公共相对航向种子。

```text
每车缓存静止IMU窗口
  → FRD/NED转FLU/ENU、单位转SI
  → initialize_static_imu
  → 写入该车惯性初值的roll/pitch和gyro bias
  → 配置15维ESKF并开始连续传播

同一历元收齐三边UWB
  → solve_three_node_formation
  → 将三个points写入纯测距/融合初始位置
  → 配置滤波器并开始在线处理
```

静止初始化已在线接通；三边规范解目前仍是独立的核心几何能力，尚未自动替换现有车辆配置中的初始位置。只有在静止判定和三边解都成功后才允许进入对应模式；失败时应继续收集新窗口或报告初始化未完成，不能偷偷使用全零初值。

历史实测回放的过程和结果见 `data/imu_static_20260805_053052/静止初始化验证.md`、`静止初始化后姿态对比.csv.gz` 与 `静止初始化后姿态对比.png`。

## 6. 参考实现

- FAST-LIO 的 IMU 初始化采用启动窗口的加速度/角速度均值估计重力方向和陀螺零偏：<https://github.com/hku-mars/FAST_LIO/blob/main/src/IMU_Processing.hpp>
- VINS-Mono 的视觉惯性初始化进一步在有运动和视觉约束时求解陀螺零偏、速度、尺度和重力：<https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/initial/initial_aligment.cpp>
- TIERS 的多机器人 UWB 工程也以一节点为原点、另一节点为基准轴，用余弦定理由边长构造三角形：<https://github.com/TIERS/uwb-cooperative-mrs-localization>

本工程只复用了这些公开实现体现的初始化原则和标准三角几何，没有引入它们的依赖或复制其系统框架。

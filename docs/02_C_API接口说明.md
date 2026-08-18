# C ABI 接口说明

> 本文覆盖兼容 C ABI v1 和独立 Pose2D ABI v2。完整声明以 `include/zju_coop/c_api.h` 为准。

公开头文件为纯 C，不含 ROS 2、STL、C++ `bool`、引用或异常类型。上海交大可在 AIBrainBox 的 ROS 2 C++ 节点中调用动态库，再把输出映射到 ROS 2 消息。当前 Windows 构建生成 `zju_coop.dll`，Linux/AArch64 目标产物应为 `libzju_coop.so`。

## 1. 版本与兼容原则

| 接口域 | 版本宏 | 查询函数 | 用途 |
|---|---:|---|---|
| 既有 C ABI v1 | `ZJU_COOP_ABI_VERSION_V1 = 0x00010000` | `zju_coop_abi_version()` | 配置、IMU、测距、原始数据预留、旧定位/拓扑/质量输出 |
| 独立 Pose2D ABI v2 | `ZJU_COOP_POSE2D_ABI_VERSION_V2 = 0x00020000` | `zju_coop_pose2d_abi_version()` | 只读查询每辆车的二维相对位置和 ENU 航向 |

Pose2D v2 是一个独立扩展域，没有改变 v1 结构尺寸和函数语义。尤其要遵守以下兼容口径：

- `zju_coop_localization_t` 仍按 v1 输出，`yaw_valid` 固定为 `ZJU_COOP_FALSE`；
- 真实 `yaw_rad` 只从 `zju_coop_get_pose2d_v2()` 获取；
- 旧调用方可以继续只使用 v1；需要 GCS 二维朝向的新调用方同时检查两个版本函数；
- 所有公开结构必须先调用对应 `*_init()`，不得自行猜测结构布局、packing 或默认值；
- 同一个 `zju_coop_handle_t` 不支持并发调用，ROS 2 wrapper 必须串行化。

## 2. 总体生命周期

默认 IMU＋测距路径的调用顺序为：

```text
zju_coop_create
  → zju_coop_configure_inertial（首个输入和有效step之前，仅一次）
  → zju_coop_push_imu（每个节点、每帧瞬时IMU）
  → zju_coop_push_range（每条平台间测距）
  → zju_coop_step（每个输出周期只实际推进一次）
  → zju_coop_get_pose2d_v2（只读，两阶段容量查询）
  → 输出映射/发布
  → zju_coop_destroy
```

一次输出周期的关键顺序如下：

```c
/* A. v1 step 第一次调用只查询容量，不推进算法。 */
zju_coop_step(handle, now_ns,
              NULL, 0, 0, &localization_count,
              NULL, 0, 0, &observation_count,
              NULL);  /* 预期返回 ZJU_COOP_BUFFER_TOO_SMALL */

/* B. 分配并逐元素 init 后，第二次调用真正推进且提交一次原子快照。 */
zju_coop_step(handle, now_ns,
              localizations, localization_count,
              sizeof(zju_coop_localization_t), &localization_count,
              observations, observation_count,
              sizeof(zju_coop_observation_t), &observation_count,
              &network);  /* 预期返回 ZJU_COOP_OK */

/* C. Pose2D 两次调用均为只读；第一次只查询车辆数。 */
zju_coop_pose2d_snapshot_v2_init(&pose_snapshot);
zju_coop_get_pose2d_v2(handle, &pose_snapshot,
                       NULL, 0, 0, &vehicle_count);

/* D. 分配并逐元素 init 后，第二次只读取得同一已提交状态的位姿。 */
zju_coop_get_pose2d_v2(handle, &pose_snapshot,
                       vehicles, vehicle_count,
                       sizeof(zju_coop_vehicle_pose2d_v2_t),
                       &vehicle_count);
```

这里的“只推进一次”指：容量查询不推进，第二次 `zju_coop_step()` 才推进一次；两个 `zju_coop_get_pose2d_v2()` 都不调用预测、滤波更新、质量窗口推进或时间推进。

## 3. 配置、内存与两阶段查询

`zju_coop_create()` 在成功返回前深拷贝节点配置。所有输入、输出数组及其生命周期由调用方管理。

数组采用显式字节步长。正确流程为：

1. 用 `NULL, 0, 0` 查询所需数量；
2. 按返回数量分配数组；
3. 对每个数组元素调用对应 init；
4. 以真实 `sizeof(element)` 作为 stride 再次调用；
5. 只在返回 `ZJU_COOP_OK` 后消费输出。

容量不足只回填所需数量并返回 `ZJU_COOP_BUFFER_TOO_SMALL`。结构大小、ABI、stride、保留字段或内部转换失败时，不部分覆盖快照和数组。

## 4. 输入接口

### 4.1 IMU

`zju_coop_imu_packet_t` 对应标准 ROS 2 `sensor_msgs/msg/Imu` 的核心字段，算法核心不依赖 ROS 2 类型：

| 字段 | 语义 |
|---|---|
| `timestamp_ns` | 上交统一时间轴中的测量时刻 |
| `receive_timestamp_ns` | wrapper 的本地接收时刻，必须与测量时间可比较 |
| `orientation_xyzw[4]` | FLU 车体系到公共 ENU 的四元数，顺序 x/y/z/w |
| `angular_velocity_rad_s[3]` | 瞬时角速度，单位 rad/s，未预积分 |
| `linear_acceleration_m_s2[3]` | 瞬时比力，单位 m/s²；静止水平时 FLU +Z 约为 +g |
| 三组 covariance | 3×3 行主序；是否使用由配置决定 |
| `frame_id[32]` | NUL 结尾，必须与配置一致 |
| `orientation_valid` | 仅表示消息姿态能否用于首帧初始化 |
| `valid/status` | 整帧和设备质量状态 |

当前最小闭环采用“配置中的初始公共 ENU 姿态＋后续瞬时陀螺递推”的理想前提。第一帧只建立时间基准；第二帧起使用相邻瞬时量中值积分。实车时，公共 ENU 初始航向和跨车统一时间必须由上海交大上游对准/同步链或双方冻结的初始化流程提供。

### 4.2 平台间测距

| 字段 | 要求 |
|---|---|
| `from_node/to_node` | 已配置且不同的两个节点 |
| `sequence` | 有向链路内递增，用于重复检测 |
| `timestamp_ns` | 上交统一时间轴中的测量时刻 |
| `receive_timestamp_ns` | wrapper 接收时刻，非零且与采样时刻可比较 |
| `range_m` | 有限且 >0，单位 m |
| `range_std_m` | 有限且 >0，单位 m，表示 1σ 标准差 |
| `nlos_probability` | `[0,1]`；缺失时令 `has_nlos_probability=false` |
| `nlos_flag` | 上游的 NLOS 硬判定 |
| `valid/status` | 字段、硬件和同步质量状态 |

`zju_coop_push_range()` 的函数返回值表示 ABI 调用是否成功；`zju_coop_range_processing_result_t` 进一步给出包处理、融合动作、滤波更新、创新、NIS 和协方差缩放结果。

### 4.3 点云预留

`zju_coop_point_cloud_packet_t`/`zju_coop_push_point_cloud()` 只校验与 ROS 2 `PointCloud2` 相容的字段布局、长度、时间、帧名和有效性，并返回“已验证、未使用”的回执。当前算法没有解析点、提取相对观测或融合激光雷达；点云接口只用于后续扩展，不能列入当前定位闭环能力。

## 5. v1 输出

### `zju_coop_localization_t`

每节点一条，包含节点/参考节点、时间、`x/y/vx/vy`、二维位置协方差、`valid` 和 `LocalizationState`。为保持 ABI v1 业务兼容，`yaw_valid=false`、`z_valid=false`。

### `zju_coop_network_t`

包含节点数、参考节点可达数、活动边数、连通/可观标志、`reason_mask` 和聚合 `LocalizationState`。

### `zju_coop_observation_t`

每候选边一条，包含滑动窗口计数、NLOS 比例、有效率、实际频率、`ObservationState`、`FusionAction`、`reason_mask`、输入溢出和协方差缩放。

## 6. Pose2D ABI v2 输出

### 6.1 公共快照头

`zju_coop_pose2d_snapshot_v2_t`：

| 字段 | 语义 |
|---|---|
| `timestamp_ns` | 三车同一 IMU 测量历元；无法形成共同历元时为 0 |
| `reference_node_id` | 当前参考车辆编号 |
| `frame_id[32]` | `coop_ref_<reference_node_id>_enu` |

### 6.2 单车元素

`zju_coop_vehicle_pose2d_v2_t`：

| 字段 | 语义 |
|---|---|
| `node_id` | 本元素所属车辆 |
| `x_m/y_m` | `node-reference` 的 ENU 东/北相对位置，单位 m |
| `yaw_rad` | 本车 FLU 前向 +X 在公共 ENU 平面内的航向，范围 `[-π, π)` |
| `position_valid` | `x_m/y_m` 当前是否可以使用 |
| `yaw_valid` | `yaw_rad` 当前是否可以使用 |

坐标原点随参考车当前位置动态移动，坐标轴始终与固定公共 ENU 平行，不跟随参考车航向旋转。参考车自身 `x_m=0、y_m=0`，其 `yaw_rad` 仍保留本车相对公共 ENU 的航向。其他车辆的航向同样是各自的 ENU 航向，不做 `yaw_node-yaw_reference`。

惯性模式要求所有配置节点具有相同、非零的姿态历元。任一节点落后时，快照 `timestamp_ns=0`，本批 `position_valid/yaw_valid=false`，从而避免把异步车辆拼成伪同步快照。仅测距模式无法观测航向，`yaw_valid=false`。

## 7. 错误处理

公开函数不让 C++ 异常穿过 ABI：

| 错误码 | 含义 |
|---:|---|
| 0 | `ZJU_COOP_OK` |
| 1 | `ZJU_COOP_INVALID_ARGUMENT` |
| 2 | `ZJU_COOP_ABI_MISMATCH` |
| 3 | `ZJU_COOP_STRUCT_SIZE_MISMATCH` |
| 4 | `ZJU_COOP_BUFFER_TOO_SMALL` |
| 5 | `ZJU_COOP_OUT_OF_MEMORY` |
| 6 | `ZJU_COOP_INTERNAL_ERROR` |
| 7 | `ZJU_COOP_NOT_READY` |

wrapper 必须记录函数名、错误码、节点号和输入序列号。非零返回值不得忽略。

## 8. ABI 与部署注意事项

- C 结构是主机内存 ABI，禁止用 `send(sizeof(struct))` 当作网络协议；
- Windows DLL 与 Linux/AArch64 `.so` 需要分别在目标平台用对应工具链生成；
- ROS 2 节点负责订阅标准消息、自定义 UWB 消息、字段转换、串行调用和结果发布；
- C ABI 不负责 DDS、ROS 2 QoS、无线路由、车辆控制或 GCS 网络传输；
- 当前 Windows 已有构建、单元测试、C 头文件、SDK、自检和本机 UDP smoke 验证链；
- Ubuntu 22.04 x86-64 上的 ROS 2 适配和 Gazebo 闭环已验证；RK3588/AArch64 动态库加载、跨盒 DDS、真实传感器、实车时钟同步和精度尚未完成验证。

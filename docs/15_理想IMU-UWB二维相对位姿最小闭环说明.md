# 理想 IMU＋UWB 二维相对位姿最小闭环说明

> 本文保留的是分布式/ROS 2 改造前的集中式 UDP 最小闭环历史快照。当前每车本地惯导、NodeState、参考车二维修正和 ROS 2/Gazebo 实现见 `docs/17_分布式本地惯导与参考车协同融合设计.md` 与 `docs/18_分布式改造实施计划.md`。本文中“当前”均指该历史版本。

## 1. 这版完成了什么

当前工程已经形成一个面向三车 GCS 展示的最小软件闭环：

```text
三车瞬时IMU + 三车平台间UWB测距
              ↓
      输入校验、统一时间和去重
              ↓
   每车15维ESKF惯性递推 + 联合测距更新
              ↓
  参考车动态原点下的三车x/y + 各车ENU yaw
              ↓
       C ABI Pose2D v2只读快照
              ↓
  临时UDP 105号消息 → 本机GCS二维显示
```

这条链用于验证接口、调用顺序、状态递推、三车快照和展示。它采用理想初始对准与模拟传感器，当前不能证明实车精度。

## 2. 当前能力与边界

| 项目 | 当前状态 | 说明 |
|---|---|---|
| 三车瞬时 IMU 输入 | 已实现 | 字段语义与 ROS 2 `sensor_msgs/Imu` 核心字段相容 |
| UWB 平台间测距 | 已实现 | 临时协议含距离、标准差、NLOS 和有效状态；ROS 2 自定义消息仍需双方映射 |
| 每车 15 维惯性递推 | 已实现 | 名义位置、速度、姿态、陀螺零偏、加速度计零偏及 15 维误差协方差 |
| 测距联合更新 | 已实现 | UWB 约束更新多车联合状态，并保留跨节点协方差 |
| 三车二维相对位置 | 已实现 | 输出相对参考车的 x/y |
| 三车各自 ENU 航向 | 已实现 | 从各车 FLU→ENU 姿态提取 yaw；UWB 本身不提供航向观测 |
| 同一历元快照保护 | 已实现 | 三车姿态历元不一致时，时间和有效位明确无效 |
| C ABI v1 | 保持兼容 | 旧结构、调用顺序和 `yaw_valid=false` 保持原语义 |
| Pose2D ABI v2 | 已实现 | 独立只读查询 x/y/yaw 和两个有效位 |
| 临时 UDP 105 | 已实现 | 每车 32 B payload，三帧组成一个三车快照 |
| Windows GCS 看板 | 已实现 | 按时间与参考节点收齐三车后原子更新，处理乱序、残缺批次和旧帧，并显示参考节点/快照时间/frame |
| 点云接口 | 仅预留 | 只做 PointCloud2 相容布局校验，未解析、未形成相对观测、未融合 |
| ROS 2/DDS | 尚未验证 | 需要盒端 wrapper/节点和正式消息/QoS |
| RK3588/AArch64 | 尚未验证 | 需要 Ubuntu 22.04 真机原生构建与动态库加载测试 |
| 实车 | 尚未验证 | 需要真实同步、对准、外参、UWB质量和RTK真值 |

## 3. 理想前提

当前能输出有效 yaw，依赖以下前提同时成立：

1. 三车的初始四元数或 `initial_yaw_rad` 已处于同一个公共 ENU；
2. 车体系采用 FLU：x 前、y 左、z 上；导航系采用 ENU：x 东、y 北、z 上；
3. IMU 给出瞬时角速度和瞬时比力，单位分别为 rad/s、m/s²；
4. 三车测量时间戳已由上海交大同步链映射到同一时间轴；
5. 第一帧只建立各车时间基准，后续帧时间间隔合法；
6. 初始 yaw 之后主要由陀螺递推；当前 UWB 距离不会独立校正绝对 yaw；
7. 模拟器未覆盖真实温漂、安装误差、振动、时钟漂移和复杂 NLOS。

因此，当前 `yaw_valid=true` 表示“在上述初始化和同步口径下，算法持有可递推姿态”。它不等价于“UWB 已观测并验证航向”，也不构成长期无漂移保证。

## 4. 坐标系定义

默认参考车为节点 1，可由 `reference_node_id` 配置。设公共 ENU 中车辆 `i` 和参考车 `r` 的位置为：

```text
p_i^ENU = [E_i, N_i, U_i]^T
p_r^ENU = [E_r, N_r, U_r]^T
```

二维相对输出为：

```text
x_i = E_i - E_r
y_i = N_i - N_r
```

该坐标系有两个同时成立的特征：

- 动态原点：原点随参考车当前位置移动；
- 固定轴向：x/y 始终平行公共 ENU 的东/北轴，不随参考车航向转动。

参考车输出 `x=0、y=0`。参考车的 yaw 仍是本车前向相对公共 ENU 的航向，不强制为 0。

每辆车 yaw 的计算口径为：

```text
f_i^ENU = R_body_to_enu · [1,0,0]^T
yaw_i = atan2(f_i.N, f_i.E),  yaw_i ∈ [-π,π)
```

其他车辆输出自己的 `yaw_i`，不输出 `yaw_i-yaw_r`。这一设计让 GCS 地图的东/北方向固定，参考车转向时整张图不会跟着旋转。

对应 frame 名为：

```text
coop_ref_<reference_node_id>_enu
```

例如参考车为 1 时是 `coop_ref_1_enu`。

## 5. 输入数据

### 5.1 IMU

每车至少需要：

- `node_id`、`sequence`；
- 测量 `timestamp_ns` 和本地 `receive_timestamp_ns`；
- FLU 三轴瞬时角速度；
- FLU 三轴瞬时比力；
- 三组 3×3 行主序协方差；
- 可选初始姿态四元数及独立有效位；
- `frame_id`、整帧 `valid` 和设备 `status`。

默认 `config/demo.ini` 使用配置中的初始 yaw，`use_orientation_for_initialization=false`。后续如由上海交大 IMU/组合导航模块提供可靠首帧公共 ENU 四元数，可经双方验证后启用消息姿态初始化。

### 5.2 UWB

现有算法结构需要：

- 起点节点、终点节点；
- 有向链路序列号；
- 统一测量时间和接收时间；
- 距离与 1σ 标准差；
- 可选 NLOS 概率、NLOS 标志；
- 有效位与设备状态。

上海交大当前提出的 ROS 2 UWB 消息 `header/src_id/target_id/distance` 能提供最基础测距，但还缺少序列、测距不确定度、NLOS/质量和接收时间的明确映射。最小联调可由 wrapper 填保守默认值；正式接口应书面冻结这些字段来源，避免把临时默认值误当设备真实质量。

## 6. 算法如何运行

每辆车拥有一套名义惯性状态，联合滤波器保存全部车辆的误差协方差：

```text
δx_i = [δp_i, δv_i, δθ_i, δb_gi, δb_ai]  共15维
```

一次典型处理过程为：

1. IMU 首帧建立该节点时间基准；
2. 后续 IMU 用前后瞬时量中值积分位置、速度和四元数；
3. 同步传播 15 维误差状态转移和过程噪声；
4. UWB 距离形成两车位置差的标量量测；
5. 通过质量状态、NIS 和协方差缩放决定正常更新、降权、暂缓或拒绝；
6. 联合更新允许一个测距约束同时修正两端节点，并建立跨节点协方差；
7. 输出时减去参考车位置，提取各车 ENU yaw；
8. 只有三车姿态都落在同一非零 IMU 历元，Pose2D 批次才有效。

## 7. C ABI 调用口径

### 7.1 v1 保持兼容

`ZJU_COOP_ABI_VERSION_V1=0x00010000`。既有的配置、输入和 `zju_coop_step()` 保持原行为；旧 `zju_coop_localization_t.yaw_valid` 始终为 false。

### 7.2 独立 Pose2D v2

`ZJU_COOP_POSE2D_ABI_VERSION_V2=0x00020000`。新增：

```c
uint32_t zju_coop_pose2d_abi_version(void);

zju_coop_error_code_t
zju_coop_pose2d_snapshot_v2_init(zju_coop_pose2d_snapshot_v2_t* value);

zju_coop_error_code_t
zju_coop_vehicle_pose2d_v2_init(zju_coop_vehicle_pose2d_v2_t* value);

zju_coop_error_code_t
zju_coop_get_pose2d_v2(zju_coop_handle_t* handle,
                       zju_coop_pose2d_snapshot_v2_t* snapshot,
                       zju_coop_vehicle_pose2d_v2_t* vehicles,
                       uint32_t vehicle_capacity,
                       uint32_t vehicle_stride,
                       uint32_t* vehicle_count);
```

一次输出周期遵循：

```text
v1 step容量查询（不推进）
  → v1正式step（只推进并提交一次）
  → Pose2D容量查询（只读）
  → Pose2D正式查询（只读）
```

Pose2D 第一次查询必须使用 `vehicles=NULL, capacity=0, stride=0`。第二次查询前，每个元素都要调用 v2 init。查询不推进滤波，不修改质量窗口，也不改变已提交时间。

## 8. 三车同一快照

`zju_coop_pose2d_snapshot_v2_t` 提供：

- `timestamp_ns`：三车共同 IMU 测量历元；
- `reference_node_id`：参考车；
- `frame_id`：动态原点、固定 ENU 轴坐标系名称。

每个 `zju_coop_vehicle_pose2d_v2_t` 提供：

- `node_id`；
- `x_m/y_m/yaw_rad`；
- `position_valid/yaw_valid`。

如果任一车没有时基或时间不同，公共时间置 0，整批位置与航向有效位均置 false。调用方应保留上一合法结果或显示无效状态，不能用数值字段本身推断有效性。

## 9. 105 号临时 UDP 映射

临时在线进程把每辆车编码成一帧 105：

| 来源 | 105 公共头/载荷 |
|---|---|
| `snapshot.timestamp_ns` | header.timestamp_ns |
| `vehicle.node_id` | header.source_node |
| `snapshot.reference_node_id` | header.target_node |
| `vehicle.x_m/y_m/yaw_rad` | payload 偏移 0/8/16 的三个 f64LE |
| `vehicle.position_valid/yaw_valid` | payload 偏移 24/25 的两个 u8 |
| 当前能力 | payload `capability_mask=0x0000000A` |

payload 固定 32 B：

```text
<dddBBHI
x_m, y_m, yaw_rad, position_valid, yaw_valid, reserved=0, capability_mask
```

加上 40 B ZJCL 公共头后，每帧共 72 B。一个三车快照由三个相同 `timestamp_ns`、相同 `target_node`、不同 `source_node` 的 105 帧组成。

该 UDP 只为当前 demo 服务。正式 GCS 通过 ROS 2/DDS 时，更适合一次发布一个含三车数组的快照消息，从消息层保证同批语义。

## 10. Windows 如何运行

所有命令从工程根目录执行。

### 10.1 构建和全套自动测试

```powershell
cmake -S . -B build-vscode -DBUILD_TESTING=ON
cmake --build build-vscode --config Release --parallel
ctest --test-dir build-vscode -C Release --output-on-failure
```

### 10.2 独立自检

```powershell
.\build-vscode\Release\zju_coop_self_check.exe
```

预期末行：

```text
SELF_CHECK PASS (passed=14, failed=0)
```

其中 `[PASS] pose2d_snapshot` 检查三车 0°/+90°/-135° 初始 ENU 航向、3-4-5 相对位置、共同时间戳和 `coop_ref_1_enu` 帧名。

### 10.3 三终端在线演示

终端 1：

```powershell
py -3 -B tools\gcs_dashboard.py --udp-port 39002 --http-port 8080
```

终端 2：

```powershell
.\build-vscode\Release\zju_coop_online.exe --config config\demo.ini
```

终端 3：

```powershell
py -3 -B tools\imu_uwb_simulator.py --host 127.0.0.1 --port 39001 --duration 10
```

浏览器打开 `http://127.0.0.1:8080`。

### 10.4 单独执行默认 smoke

```powershell
py -3 -B tools\imu_online_smoke_test.py `
  --online-exe build-vscode\Release\zju_coop_online.exe `
  --replay-exe build-vscode\Release\zju_coop_replay.exe `
  --config build-vscode\config\demo.ini
```

该 smoke 会检查三车同时间戳 105、三车有效位、配置 yaw、模式 2、日志和回放。测试失败时应以脚本异常和 online `SUMMARY` 定位，不能只观察页面“看起来在动”。

## 11. Windows 当前可验证的结论

通过上述测试可以形成以下有限结论：

- 源码能生成 Windows `.dll/.lib/.exe`；
- 纯 C 头文件与 v1/v2 初始化器可由 C 编译单元使用；
- v1 step 容量查询不推进，正式 step 原子提交一次；
- Pose2D 两阶段查询只读；
- 错版本、错结构长度、错 stride、非零 reserved 和容量不足不会部分写输出；
- 三车配置 yaw 可经瞬时 IMU 路径输出到 Pose2D v2 和 105；
- 参考车 x/y 归零且 yaw 保留；
- `yaw=+π` 规范化为 `-π`；
- 三车不同历元时有效位关闭；
- Python 与 C++ 对 32 B 105 payload、CRC 和字段约束具有交叉测试；
- online、GCS、日志和 replay 能组成进程级闭环。

验收结论以实际执行时的完整输出和退出码为准。任何失败项都必须保留，不能只摘录通过行。

## 12. 仍需上海交大和实车联调完成的工作

1. 冻结 UWB 自定义消息的序列、距离标准差、NLOS、状态与接收时间来源；
2. 冻结单车和跨车同步状态、统一 epoch 及异常上报方式；
3. 冻结三车公共 ENU 建立与初始 yaw/四元数来源；
4. 编写或集成 ROS 2 节点：订阅 IMU/UWB、转换 C ABI、串行调用、发布数组快照；
5. 冻结 ROS 2 结果消息、topic、QoS、坐标系和 GCS 超时策略；
6. 在 Ubuntu 22.04/RK3588 原生编译 `.so` 并验证导出符号、加载、运行与资源占用；
7. 使用真实车辆、UWB、IMU 和独立 RTK 真值测试同步、误差、NLOS、断链与恢复；
8. 后续需要激光雷达时，再实现点云前端、相对观测、质量字段和融合更新。当前预留接口没有这些算法。

## 13. 正式联调时必须避免的误判

- 页面出现三辆车，只能说明数据链路可见；还需检查共同时间戳、参考节点和有效位；
- `yaw_valid=true` 依赖共同 ENU 初始化与陀螺递推，不能证明 UWB 已观测航向；
- 模拟数据通过不能证明真实 NLOS、温漂、振动和时钟漂移可控；
- Windows DLL 通过不能证明 RK3588 `.so` 可加载；
- 临时 UDP 105 通过不能证明 ROS 2/DDS 消息、QoS 和 GCS 跨盒链路已完成；
- 点云结构能进入预留函数不能证明激光雷达协同定位已经实现。

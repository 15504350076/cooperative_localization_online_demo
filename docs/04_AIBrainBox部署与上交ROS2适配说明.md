# AIBrainBox 部署与浙大/上交 ROS 2 适配说明

## 1. 目标与验证边界

首要目标是 AiBrain BOX-UGV（V3）的 RK3588/AArch64、Ubuntu 22.04、ROS 2 环境。AiBrain BOX-V5 的 Ubuntu 20.04 规格只作参考，不能据此推断两种盒子的二进制兼容性。

当前工程已在 Windows x64 和 Ubuntu 22.04 x86-64 完成算法构建；ROS 2 Humble/Fast DDS 节点、解析仿真和 Gazebo Fortress 同机闭环已在 Ubuntu x86-64 验证。以下 ARM64 命令仍只是盒端执行步骤，不是 RK3588 已通过的证据。

### 1.1 同一份代码与不同二进制

Windows 与 RK3588 使用同一份算法源码、`c_api.h`、配置语义和调用顺序；不能使用同一个编译产物。Windows x64 由 MSVC 生成 `zju_coop.dll`，RK3588/AArch64 由 Ubuntu GCC/Clang 生成 `libzju_coop.so`。两者的处理器指令集、目标文件格式和系统 ABI 不同，禁止直接复制替换。

浙大 ROS 2 适配节点已作为独立包实现，并保持算法库本体不包含 `rclcpp` 或 ROS 2 消息头；ROS 2 依赖只进入适配层。节点必须在目标 ROS 2 发行版、消息包和编译器环境中重新构建。当前首版节点、消息、topic 和 QoS 已在 x86-64 同机验证，但上交正式 UWB 包替换、跨盒 DDS 和 RK3588 仍未验证。

## 2. 首版生产链路与双方分工

```text
IMU驱动 + UWB/MCU UART TLV
  → 上交驱动和字段解析
  → 上交单平台及跨平台统一时间轴
  → 上交 ROS 2标准Imu消息和自定义Range消息
  → 浙大 ROS 2/C ABI适配节点（首版已实现）
  → 浙大 libzju_coop.so
  → 浙大 CooperativePose2DArray 发布（首版已实现）
  → 上交DDS/盒端运行环境
  → 交大 GCS
```

- 浙大已实现并计划交付算法动态库、ROS 2 适配节点、最小结果消息定义和发布逻辑。算法库保持纯 C/C++，适配节点负责 ROS 2 输入到 C ABI 的逐字段转换，以及 C ABI 输出到结果消息的转换。
- 上交负责驱动、TLV/厂商协议解析、单平台与多平台统一时间、输入 ROS 2 消息与 topic、DDS 和盒端构建运行环境、无线转发/路由，并向浙大提供可编译的消息包和接口约定。
- 交大 GCS 在另一盒子上通过 DDS 消费结果消息，不直接调用浙大算法库。
- 首版节点、结果消息、topic 和 QoS 已在 Ubuntu x86-64 落地；上交正式 UWB 类型、跨盒 DDS 和 RK3588 部署尚未落地，不能由同机测试替代。

## 3. ARM64 原生构建步骤

在盒端安装 CMake、C/C++ 编译器和 Python 3 后：

```bash
cmake -S . -B build-arm64 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON
cmake --build build-arm64 -j"$(nproc)"
ctest --test-dir build-arm64 --output-on-failure
cmake --install build-arm64 --prefix install-arm64
```

在启动 ROS 2 wrapper 或连接真实传感器之前，先运行不占用端口和硬件的独立自检：

```bash
./build-arm64/zju_coop_self_check
```

只有看到 `SELF_CHECK PASS` 且退出码为 0，才进入 wrapper 字段映射和在线联调；失败时应保存完整输出、编译器版本和盒端架构信息。该结果验证软件最小闭环，不代表实车精度和时间同步已经验收。

安装后从任意工作目录启动时必须显式传入配置；默认 `config/demo.ini` 只适用于工程根目录运行：

```bash
./install-arm64/bin/zju_coop_online \
  --config ./install-arm64/share/zju_coop/config/demo.ini
```

相对日志路径以进程当前工作目录为基准，盒端 systemd 服务应使用明确的 `WorkingDirectory` 或把日志路径改成部署约定的绝对路径。

预期安装内容：

```text
install-arm64/
├── bin/zju_coop_online
├── bin/zju_coop_replay
├── bin/zju_coop_self_check
├── include/zju_coop/c_api.h
├── include/zju_coop/export.h
├── lib/libzju_coop.so
└── share/zju_coop/{config,tools}/...
```

Linux 构建包含导出符号检查：动态库必须导出公开 C API，且不得泄露 `zju::coop` C++ 符号。该检查仍需在 ARM64 实际执行。

## 4. ROS 2 输入消息建议

### 4.1 IMU

当前首版浙大适配节点订阅上交发布的 `sensor_msgs/msg/Imu`，并把以下字段逐项映射到 `zju_coop_imu_packet_t`：

- `header.stamp` → 统一时间轴纳秒 `timestamp_ns`；回调开始时记录同时间域的 `receive_timestamp_ns`；
- `header.frame_id` → `frame_id`，当前惯性配置要求 `imu_link`；
- `orientation` 顺序为 x/y/z/w，只在配置允许且首帧协方差有效时用于初始对准，后续不连续覆盖惯导姿态；若采用理想配置初值，则每辆车还必须提供公共 ENU 下的初始航向；
- `angular_velocity` 单位 rad/s；
- `linear_acceleration` 单位 m/s²，按比力解释，静止水平FLU传感器约为 `(0,0,+9.80665)`；
- 三组3×3协方差均按行主序；首元素 `-1` 表示不可用，全零表示未知；
- 标准消息没有温度，温度属于模组私有消息，可由上交驱动单独记录，不是当前融合必需输入。

上交必须给出 IMU 到车体 FLU 的安装外参。当前 Demo 假设消息已在 `imu_link`/FLU 轴系；若原始模组轴系不同，转换责任和执行位置需在联调接口中冻结，浙大适配节点按双方确认的外参完成映射，不能仅改 `frame_id` 字符串。初始航向表示车体 FLU 前向 `+x` 轴相对公共 ENU 东向 `+x` 轴的逆时针夹角，范围 `[-π,π)`。

### 4.2 UWB

上交可自定义输入消息包名，但平台间测距消息必须能由浙大适配节点无损映射到 `zju_coop_range_packet_t`；若底层来自 UWB，建议保留 NLOS 专用质量字段：

```text
builtin_interfaces/Time stamp
uint16 from_node
uint16 to_node
uint64 sequence
float64 range_m
float64 range_std_m
float32 nlos_probability
bool nlos_flag
bool has_nlos_probability
bool valid
uint8 status               # 0 OK, 1 DEGRADED, 2 INVALID
```

浙大适配节点在本机回调开始时采集 `receive_timestamp_ns`，不要求通过跨平台 ROS 2 消息传输该字段。`stamp` 与接收时刻必须在可比较的上交统一时间域。

失同步或驱动判定不可用时，应把 `valid=false`、`status=INVALID` 的包推入 SDK，使质量窗口能够统计无效数据；采样/接收时差超限时 SDK 会拒绝该包并报告 `TIME_SYNC_TIMEOUT`。

### 4.3 原始图像和点云预留接口

工程已提供以下不依赖ROS 2类型的零拷贝映射入口：

- `sensor_msgs/msg/Image` → `zju_coop_camera_image_packet_t`
  → `zju_coop_push_camera_image()`；
- `sensor_msgs/msg/PointCloud2`及其`PointField`数组
  → `zju_coop_point_cloud_packet_t`/`zju_coop_point_field_t`
  → `zju_coop_push_point_cloud()`。

两个入口当前只校验元数据、字段布局和借用缓冲区长度，成功回执均为
`ZJU_COOP_RAW_INPUT_VALIDATED_NOT_USED`。它们不会复制原始大数据、不会调用定位
`Engine`、不会改变IMU＋测距状态，也不能被表述为已经完成视觉识别或点云配准。
详细字段、所有权和扩展边界见`docs/13_原始视觉与点云预留接口说明.md`。

## 5. 浙大 ROS 2 适配节点调用模型（首版已实现）

- 每车 `zju_local_inertial_node` 从部署配置读取 node_id、公共 ENU 初始 `p/v/q` 和 `bg/ba`，创建本地惯导 handle；
- 本车 IMU 回调把 `sensor_msgs/msg/Imu` 逐字段映射到普通 C 结构并推入本地 15 维惯导，首帧只建立时间基准；
- 本地节点以 20 Hz 发布紧凑 NodeState，其中 stamp 是最后一帧被惯导接受的 IMU 测量时刻；
- 参考车 `zju_cooperative_fusion_node` 创建独立二维融合 handle，订阅三车 NodeState 和 `/uwb/range`；
- fusion 按 UWB 测量时刻插值或有限外推两端状态，接受更新后保存二维修正，并以 10 Hz 发布 `CooperativePose2DArray`；
- 两节点当前使用单线程 executor；所有 C API 返回值及拒绝 disposition 均进入节流日志；
- 算法参数由盒端配置，不由 GCS 远程改写。

当前实现位于 `ros2/zju_coop_ros2`，算法核心仍只使用普通 C/C++ 类型。首版不把 UWB 修正反馈给从车本地惯导，也不支持单车热重新初始化。

## 6. 结果消息

浙大 ROS 2 适配节点已将 C ABI 输出转换为 `cooperative_localization_msgs/msg/CooperativePose2DArray`。当前最小 GCS 展示字段为：

- 快照公共字段：`timestamp_ns`、`reference_node_id`、`frame_id=coop_ref_<reference_node_id>_enu`；
- 单车字段：`node_id`、`x_m`、`y_m`、`yaw_rad`、`position_valid`、`yaw_valid`；
- x/y 为节点相对参考节点的位置，坐标轴与公共 ENU 平行；yaw 为每辆车自身的 ENU 航向。

C ABI v1 仍可补充以下诊断与状态：

- Localization：时间、节点/参考节点、x/y/vx/vy、二维位置协方差、valid、state；该旧结构的 `yaw_valid=false`；
- Network：节点/可达/有效边数量、connected、observable、state、reason_mask；
- Observation：边、窗口、计数、比例、频率、state、action、reason_mask、scale；
- AlgorithmStatus 与 Alert：后续由浙大适配节点复用参考适配层语义，或在正式接口评审中扩展 C ABI。

首版结果消息、`/cooperative_localization/poses_2d` 和 QoS 已实现，适配时逐字段赋值，没有把 C 结构内存直接作为 ROS 2 serialized bytes。上交正式 UWB 包名、输入 QoS、时间同步状态以及 GCS 跨盒 DDS 行为仍待冻结；`zju_coop_test_msgs/UwbRange` 只能作为当前四字段接口的构建与测试替身。

## 7. 平台状态接口

NodeTimeSync、LinkState、NodeHealth 由上交系统产生，应与算法状态分开发布。建议字段见上级目录的《对交大GCS开发的需求.docx》。这些接口当前没有进入 C ABI 或正式 ROS 2 包，必须在生产联调前冻结消息包名、topic、QoS、单位和时钟域。

## 8. 盒端必做验证

1. ARM64 原生/交叉构建与 `ctest`；
2. `libzju_coop.so` 加载、C header smoke 和导出符号；
3. `struct_size/abi_version/stride` 的 ARM64 ABI 验证；
4. 三车IMU（目标频率按实物确认）＋三边20 Hz UWB输入、10 Hz输出在线smoke；
5. Pose2D v2 的三车 x/y/yaw、公共时间戳、参考节点、frame 和有效位映射；
6. 正式 ROS 2 节点、结果消息包、topic、QoS 与 DDS 跨盒 GCS 接收；
7. C++/Python ZJCL 与 ZJLG golden bytes 一致；
8. 真实 `sensor_msgs/Imu`、TLV到Range的字段、单位、坐标轴、标准差和NLOS映射；
9. `sensor_msgs/Image`和`PointCloud2`到新增C ABI预留接口的字段、stride、
   生命周期和大缓冲零拷贝映射；
10. 失同步、乱序、超时、NLOS、断边和恢复；
11. 长时间 CPU、内存、日志吞吐、磁盘轮转和 DDS/网络异常；
12. 进程自启动、退出、重启和版本查询。

完成以上实测前，项目状态只能写“具备盒端移植步骤，ARM64 待验证”。

## 9. GNSS 初始化与 RTK 真值适配

当前 C ABI 已增加与主 `Engine` 隔离的 `zju_coop_gnss_context_t`，用于：

1. 把标准 `sensor_msgs/msg/NavSatFix` 转换为失锁前的节点位置、速度和协方差初值；
2. 把后续 RTK 输出转换为固定公共 ENU 下、相对主参考节点的位置真值；
3. 保证 RTK 真值不进入 IMU＋测距 EKF。

计划中的浙大 GNSS 适配扩展必须逐字段映射上交发布的标准消息，并额外取得 `node_id`、`sequence`、
同时间基准的 `receive_timestamp_ns` 和经过设备质量门控的 `valid`。标准
`NavSatStatus` 无法区分 RTK Fixed/Float，所以上交必须通过专用 RTK Fixed
topic 或独立接收机状态消息提供解状态，浙大适配节点据此门控；算法库不猜测接收机解类型。

完整接口、默认门限、杆臂定义、初始化调用顺序和真值输出规则见
`docs/14_GNSS初始化与RTK相对真值接口说明.md`。盒端验收清单还应增加：

- 检查经纬高采用 WGS84，`altitude` 是椭球高而非未经说明的海拔高；
- 检查 `position_covariance` 的 ENU 语义、单位和 `covariance_type`；
- 实测各车最新 RTK 时刻差、回调延迟和 RTK Fixed 丢失时的 `valid/stale`；
- 测量并配置 IMU/车体原点到 GNSS 天线相位中心的 FLU 杆臂；
- 对比“协同定位估计”和“RTK 相对真值”两条独立数据流，确认真值未反馈到滤波器。

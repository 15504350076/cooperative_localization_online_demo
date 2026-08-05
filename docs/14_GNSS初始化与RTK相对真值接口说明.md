# GNSS 初始化与 RTK 相对真值接口说明

## 1. 结论和使用边界

标准 ROS 2 `sensor_msgs/msg/NavSatFix` 可以承载经纬高、定位状态和位置协方差，因此足以提供：

1. 卫星导航失锁前的多节点位置、速度和协方差初值；
2. 试验期间各节点相对主参考节点的 RTK 相对位置真值。

但是，标准 `NavSatStatus.status` 只有无定位、普通定位、SBAS 和 GBAS 等状态，不能证明接收机当前是 RTK Fixed、RTK Float 或单点定位。因此正式联调必须额外冻结以下约定之一：

- 推荐：只有已经由上交驱动判定为 RTK Fixed 的结果才发布到专用真值 topic；
- 或者：另发接收机质量状态消息，并在上交 wrapper 中只把 RTK Fixed 帧标为 `valid=true`。

浙大算法库不根据 `NavSatStatus` 猜测 RTK Fixed。`valid=true` 表示上交 wrapper 已完成设备级质量判定。

GNSS 上下文和主协同定位 `Engine` 是两个独立对象。RTK 真值接口没有主算法句柄参数，不能修改 EKF 状态，也不能作为 UWB 更新量测。该隔离用于防止“拿真值参与估计，再用同一真值评价估计”的数据泄漏。

## 2. ROS 2 字段到 C ABI 的映射

上交 ROS 2 wrapper 必须逐字段映射，不能把 ROS 2 消息内存强制转换成 C 结构体。

| ROS 2 `sensor_msgs/msg/NavSatFix` | C ABI | 说明 |
|---|---|---|
| `header.stamp` | `timestamp_ns` | 上交统一时间轴上的测量时刻，单位 ns |
| 回调接收时刻 | `receive_timestamp_ns` | 同一时间基准下的本机接收时刻 |
| `header.frame_id` | `frame_id[32]` | 最多 31 个可见字符并以 `\0` 结尾 |
| wrapper 节点配置 | `node_id` | 平台编号，不从字符串猜测 |
| wrapper 接收计数 | `sequence` | 同节点单调递增 |
| `status.status` | `navsat_status` | 保持 ROS `int8` 语义；小于 0 表示无定位 |
| `status.service` | `service` | GPS/GLONASS/COMPASS/GALILEO 位图 |
| `latitude` | `latitude_deg` | WGS84 纬度，单位 ° |
| `longitude` | `longitude_deg` | WGS84 经度，单位 ° |
| `altitude` | `altitude_m` | WGS84 椭球高，单位 m |
| `position_covariance[9]` | `position_covariance_m2[9]` | 行主序；原消息局部切平面 ENU 协方差 |
| `position_covariance_type` | 同名枚举 | 数值 0～3 与 ROS 定义一致 |
| wrapper 质量门限结果 | `valid` | 仅 RTK Fixed 且设备/同步状态有效时为 true |

当前初始化是三维初始化，要求经纬高均为有限数。若设备不提供可靠椭球高，不能填 NaN 后期待生成初始化；应先由双方确定二维模式或可靠的高程来源。

## 3. 坐标系、杆臂和协方差

- 地理坐标采用 WGS84；
- 第一次成功初始化时，以主参考节点最新有效 GNSS 天线位置建立公共 ENU；
- 初始化成功后，公共 ENU 原点和初始化结果冻结；
- 后续 GNSS 只更新相对真值，不重新初始化主滤波器；
- `antenna_lever_arm_body_m` 是从 IMU/车体状态原点指向 GNSS 天线相位中心的 FLU 向量；
- `orientation_body_to_enu_xyzw` 是车体 FLU 到公共 ENU 的四元数，顺序为 x/y/z/w；
- 算法先将 GNSS 天线位置转到公共 ENU，再减去旋转后的杆臂，得到车体/IMU 原点初值；
- 每个 `NavSatFix` 自带的局部 ENU 协方差会旋转到固定公共 ENU；
- 节点相对主参考节点的真值协方差按独立误差近似计算：
  `Cov(relative) = Cov(node) + Cov(reference)`。

如果多车 RTK 共享同一基站、改正源或大气误差，上式会偏保守，因为节点误差可能相关。当前标准 `NavSatFix` 不提供跨节点互协方差；若后续需要更严谨的真值不确定度，应扩展独立的 RTK 误差模型，而不是篡改 `NavSatFix`。

## 4. 初始化条件和默认门限

`zju_coop_gnss_config_init()` 给出的默认值为：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `reference_node_id` | 1 | 主参考节点 |
| `max_epoch_skew_ns` | 100 ms | 各节点最新初始化样本最大时间差 |
| `max_truth_age_ns` | 500 ms | 真值最大允许陈旧时间 |
| `max_future_skew_ns` | 100 ms | 测量时间比接收时间最多超前量 |
| `max_receive_delay_ns` | 500 ms | 最大接收延迟 |
| `min_velocity_dt_s` | 0.2 s | 两帧差分测速最短间隔 |
| `max_velocity_dt_s` | 5.0 s | 两帧差分测速最长间隔 |
| `max_horizontal_std_m` | 0.2 m | 东、北方向单轴标准差上限 |
| `max_vertical_std_m` | 0.5 m | 天方向标准差上限 |
| `require_known_covariance` | true | 至少要求对角已知协方差 |

每个节点至少需要两帧合格数据。速度由两帧公共 ENU 位置差除以测量时间差得到；这只是失锁前初始速度估计，不是惯导预积分。

初始化尚不具备条件时，`zju_coop_gnss_build_initializations()` 返回 `ZJU_COOP_NOT_READY`，不是程序故障。wrapper 应继续接收 GNSS，而不是销毁上下文或伪造初值。

## 5. C ABI 调用顺序

### 5.1 创建独立 GNSS 上下文

1. 每个结构体先调用对应的 `*_init()`；
2. 填写每个节点的编号、天线杆臂和初始车体姿态；
3. 填写 `zju_coop_gnss_config_t`；
4. 调用 `zju_coop_gnss_create()`。

### 5.2 接收失锁前 GNSS

每次 `NavSatFix` 回调：

1. 记录本机接收时刻；
2. 填写 `zju_coop_gnss_fix_packet_t`；
3. 调用 `zju_coop_gnss_push_fix()`；
4. 检查函数返回值和 `zju_coop_gnss_processing_result_t.disposition`。

函数返回 `ZJU_COOP_OK` 只表示 API 调用完成；数据是否被保存还要看：

- `ZJU_COOP_GNSS_STORED`；
- `ZJU_COOP_GNSS_INVALID_PACKET`；
- `ZJU_COOP_GNSS_UNKNOWN_NODE`；
- `ZJU_COOP_GNSS_DUPLICATE`；
- `ZJU_COOP_GNSS_OUT_OF_ORDER`；
- `ZJU_COOP_GNSS_TIME_REJECTED`。

### 5.3 生成主算法初值

先用空输出数组查询节点数量：

```cpp
uint32_t node_count = 0;
const zju_coop_error_code_t query_status =
    zju_coop_gnss_build_initializations(
        gnss_context,
        nullptr, 0, 0,
        nullptr, 0, 0,
        &node_count);
```

- 返回 `ZJU_COOP_NOT_READY`：样本数量、时间对齐或质量门限尚不满足；
- 返回 `ZJU_COOP_BUFFER_TOO_SMALL`：已经可以初始化，`node_count` 是所需数组长度。

然后分别准备并初始化：

- `zju_coop_node_initialization_t[node_count]`；
- `zju_coop_inertial_node_initialization_t[node_count]`。

第二次调用得到两组现有接口可直接使用的初值：

1. 把基础初值数组交给 `zju_coop_config_t.nodes`，调用 `zju_coop_create()`；
2. 把 15 维惯性初值数组交给 `zju_coop_inertial_config_t.nodes`；
3. 在首个 IMU/UWB 输入前调用 `zju_coop_configure_inertial()`；
4. GNSS 失锁后，主算法继续使用 IMU＋UWB，不再依赖 GNSS 输入。

### 5.4 查询 RTK 相对位置真值

继续把有效 RTK `NavSatFix` 推入同一个 GNSS 上下文，并定期调用：

```cpp
zju_coop_gnss_get_relative_truth(
    gnss_context, now_ns,
    truth_buffer, truth_capacity, sizeof(truth_buffer[0]),
    &truth_count);
```

输出 `position_enu_m` 是“该节点减主参考节点”的三维位置。只有 `valid=true` 且 `stale=false` 时才可作为本时刻评价真值。真值建议写入独立日志/topic，在 GCS 中与协同定位估计值分层显示，不得覆盖算法定位结果。

退出时分别调用：

```cpp
zju_coop_destroy(main_handle);
zju_coop_gnss_destroy(gnss_context);
```

两个对象生命周期独立，但同一个对象均不支持未经保护的多线程并发调用。

## 6. 建议冻结的 ROS 2 topic

具体包名可由上交确定，语义建议冻结为：

| topic 示例 | 消息 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|
| `/vehicle_N/gnss/fix` | `sensor_msgs/msg/NavSatFix` | 上交驱动 | 上交 wrapper | 失锁前初始化输入 |
| `/vehicle_N/rtk/fix` | `sensor_msgs/msg/NavSatFix` | 上交 RTK 驱动 | 上交 wrapper | 仅 RTK Fixed 的评价真值 |
| `/cooperative_localization/rtk_relative_truth` | 上交自定义结果消息 | 上交 wrapper | 日志/GCS 网关 | 固定公共 ENU 相对真值 |

初始化 topic 和真值 topic 可以来自同一接收机，但质量门限、使用阶段和下游用途必须明确分开。若共用一个 topic，则必须另有确定的 RTK 解状态消息并在 wrapper 中完成门控。

建议 QoS：

- 传感器输入：`KeepLast(10)`、`BestEffort`、`Volatile`；
- 真值输出：根据 GCS 网关是否允许丢帧确定 `BestEffort` 或 `Reliable`；
- 不依赖 ROS 到达顺序代替 `header.stamp` 和 `sequence` 检查。

QoS 仍需结合上交真实驱动发布频率、DDS 实现和跨车网络方案实测后冻结。

## 7. 当前已实现与尚未实现

已实现：

- 标准 `NavSatFix` 普通 C 结构映射；
- 数据范围、质量、重复、乱序和时间延迟检查；
- 每节点两帧缓存和差分初始速度；
- WGS84 → ECEF → 固定公共 ENU；
- 天线杆臂改正和协方差旋转；
- 现有基础/15维惯性初值输出；
- 独立 RTK 相对位置真值和陈旧标志；
- 初始化原点冻结；
- C、C++ 接口与自动测试。

尚未实现或尚未实测：

- 上交正式 ROS 2 wrapper 和最终 topic/QoS；
- 接收机厂商 RTK Fixed/Float 状态映射；
- GNSS 数据的 ZJCL/UDP 演示帧和 ZJLG 日志记录；
- GNSS 航向或静止初始对准算法；
- GNSS 失锁后的 GNSS 量测更新——当前设计本来就不使用；
- ARM64/RK3588 盒端编译、实际 RTK 数据与精度验证。

因此当前结论是“GNSS 初始化与 RTK 相对真值的算法库接口已经具备，ROS 2 和盒端实测待上交联调”，不能表述为已经在 AIBrainBox 上验证通过。

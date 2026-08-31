# 上交盒端 UWB 统一时间接入约定

> 本约定覆盖实盒 ROS 2 主链的 IMU、NodeState、UWB、GNSS 后备和协同结果。
> `zju_coop_online` 仍是基于本机 SystemTime 的旧临时 UDP 演示入口，禁止直接接入
> UWB 运行时标；2026-08-25 硬件框图已给出 LiDAR/Camera 的物理同步链路，
> 但它们到 UWB 时域的实机转换仍待上交冻结。当前
> 默认关闭且只做 `VALIDATED_NOT_USED` 接口校验，不声称已完成感知时间统一。

## 1. 冻结的算法时域

实盒协同定位内部统一使用 `UWB_SYSTEM_TIME`。它表示本次 UWB Master
同步历元开始后的系统运行时间，单位为纳秒；不是 Linux/ROS 系统 UTC，也不是
真实 GNSS UTC。固定日期 `2026-08-01 00:00:00.000 + t_SYNC` 的模拟 RMC
只用于兼容 PPS/RMC 设备边界，进入定位消息前应还原为 `t_SYNC`。

### 1.1 2026-08-25 硬件框图的最小运行口径

当前硬件参考为 [UGV 多模态传感器时钟同步框图](images/UGV多模态传感器时钟同步框图20260825.png)；
[UAV 框图](images/UAV多模态传感器时钟同步框图20260825.png) 一并归档，但不改变本 UGV 最小实现口径。

框图表明 GNSS_PPS 和 UWB_PPS 都可进入数字开关时钟分配器。这是硬件能力，
不代表定位程序可在两种时域间无状态切换。首轮联调固定为：

- 数字开关选择 **UWB_PPS**，IMU、UWB 测距和 NodeState 继续使用
  `UWB_SYSTEM_TIME`；
- 不支持运行中静默切换 PPS 源。发生源切换、Master 重启或时间跳变时，
  按新时间历元处理：停止受影响数据并整组重启定位链路；
- 真实 GNSS RMC 和 `UWB_GPRMC` 必须在驱动中明确命名和分支处理，
  不得把真实 UTC 秒值当成 `t_SYNC`；
- 在正式同步状态接口冻结前，浙大程序无法从 `header.stamp`
  反推硬件开关位置，上交驱动负责保证所声明的时间源真实有效。

本口径将工程外时间说明中“PPS 不是 GNSS 直接输出”限定为
“协同定位工作模式使用 UWB_PPS”，不否认硬件上另有 GNSS_PPS 备选支路。

上交发布的以下 `header.stamp` 必须使用同一数值时域：

- `sensor_msgs/msg/Imu`：对应实际 IMU 采样时刻，不得使用 RK3588 收到 UART
  数据的时刻；
- 正式 UWB 测距消息：对应双方冻结的实际测距事件时刻，不得使用结果计算或
  ROS 发布时间；
- GNSS 后备使用的 `cooperative_interfaces/msg/GnssPosition`：必须先把 GNSS 测量时刻转换到
  `UWB_SYSTEM_TIME`，不得把真实 UTC 原样混入；
- 浙大 `NodeState` 和协同结果会继承上述 UWB 时间戳。

同一 IMU 流、同一 `node_id` 的 NodeState 和同一无向 UWB 边的时间必须非零、
严格递增；不同车或不同边允许使用同一量测历元。网络到达时间只用于延迟和超时检查，
不得覆盖量测时间。

STM32 经 UART 发往 RK3588 的数据必须满足以下边界：

| 数据 | ROS `header.stamp` 所对应的时刻 | 禁止替代为 |
|---|---|---|
| IMU | STM32 利用 PPS 捕获和定时器计数得到的实际采样时刻 | UART 发送/接收或 ROS 发布时刻 |
| UWB TOF | 双方冻结的实际测距事件时刻 | 结果计算完成或 ROS 发布时刻 |
| GNSS 后备 | GNSS 量测时刻转换后的 `UWB_SYSTEM_TIME` | 未转换的 REAL_UTC |

UWB TOF 时间戳究竟对应 Poll TX、Response RX、Final RX 还是其他参考事件，
以及 PPS 与 `UWB_GPRMC` 的秒号配对规则，仍需上交书面冻结；在此之前不在浙大算法中猜测。

### 1.2 LiDAR/Camera 的当前边界

新框图确认了 `RK3588 LAN ↔ MID360 LAN` 的 PTP 物理链路和 MIPI 相机同步分配链路，
但仍未确认 PTP Grandmaster、RK3588/PTP 到 `UWB_SYSTEM_TIME` 的映射、
点云 Header 的时域，以及相机时戳对应曝光开始还是曝光中心。
因此只更新硬件拓扑认知，不改变两路输入“默认关闭、不参与融合”的程序设置。

## 2. 浙大程序的时间处理

本地惯导节点以传感器 `header.stamp` 和本机 `steady_clock` 估计同域接收时刻，
不会拿 Linux/ROS UTC 与 UWB 时间直接相减。参考车融合节点以参考车本地
`NodeState.header.stamp` 为 UWB 时钟锚点，并用 `steady_clock` 推进当前 UWB
时刻；远端 `NodeState`、UWB 测距、结果新鲜度都相对该时刻检查。

因此无需把 RK3588 系统 UTC 改成模拟 RMC，但参考车本地 IMU 必须持续提供有效
的 UWB 时间锚点。`max_future_skew_ms` 和 `max_receive_delay_ms` 是异常数据保护
门限，不代表 UWB 同步精度指标。

## 3. 失同步、重启与 GNSS 后备

- UWB 授时失效后，在正式质量接口尚未冻结的当前版本，上交驱动必须停止发布受影响的
  IMU/UWB 数据，使 NodeState 自然超时失效；正式消息冻结 valid/status 语义后才可改用无效标记。
  不得继续用未同步的本地计时冒充 UWB 时间。
- 当前消息尚无已冻结的 `sync_epoch/session_id`。若 PPS 源切换或 Master 重启造成时间回拨或超过配置门限的
  向前跳变，首版必须整组重启三车本地惯导节点和参考车融合节点；若启用了 GNSS 后备，
  还要清掉它保存的 NodeState/GNSS 时间基线，最简单是重启参考车整个 `vehicle.launch.py`。
  不能在旧后验或旧时间缓存上续算。
- GNSS 后备只处理“UWB 测距不可用、但 UWB SYNC/PPS 授时仍有效”的情况。
  如果 UWB 授时本身失效，GNSS 距离不能单独维持当前协同滤波时间轴。
- GNSS 后备同时要求三车 `NodeState` 新鲜且处于同一 UWB 时域；任一车状态超时
  或 GNSS 时间戳仍是 REAL_UTC 时，不生成派生距离。

## 4. 上交联调最小验收

1. 记录数字开关的 PPS 来源，证明协同定位运行期间始终为 UWB_PPS，
   且真实 GNSS RMC 与 `UWB_GPRMC` 未被混用。
2. 保存三车 IMU、三车 `NodeState`、三边 UWB 和接收日志，证明同历元数据时间差、
   严格单调和端到端延迟满足双方冻结门限。
3. 在 STM32 侧同时保存采样/测距定时器时刻和 RK3588 ROS Header，
   证明 Header 是量测时刻，不是 UART 或 ROS 到达时刻。
4. 断开一个 Anchor 的 SYNC，验证对应状态和协同位置在超时后变为无效；恢复后
   不出现旧包倒灌。
5. 重启 Master 并单独演练一次 PPS 源切换，验证时间回拨/向前跳变时都按首版约定整组重启，
   旧时间历元与新历元不混算。
6. GNSS 后备分别验证：测距故障但授时正常时可运行；授时失效或 REAL_UTC 未转换
   时必须拒绝。
7. “约 12 次中值”和“百微秒级”属于硬件实现/待验收指标，不写成浙大算法固定
   常量或同步精度承诺。

# 上交盒端 UWB 统一时间接入约定

> 本约定覆盖实盒 ROS 2 主链的 IMU、NodeState、UWB、GNSS 后备和协同结果。
> `zju_coop_online` 仍是基于本机 SystemTime 的旧临时 UDP 演示入口，禁止直接接入
> UWB 运行时标；LiDAR/Camera 到 UWB 时域的实机转换仍待上交冻结，当前
> 默认关闭且只做 `VALIDATED_NOT_USED` 接口校验，不声称已完成感知时间统一。

## 1. 冻结的算法时域

实盒协同定位内部统一使用 `UWB_SYSTEM_TIME`。它表示本次 UWB Master
同步历元开始后的系统运行时间，单位为纳秒；不是 Linux/ROS 系统 UTC，也不是
真实 GNSS UTC。固定日期 `2026-08-01 00:00:00.000 + t_SYNC` 的模拟 RMC
只用于兼容 PPS/RMC 设备边界，进入定位消息前应还原为 `t_SYNC`。

上交发布的以下 `header.stamp` 必须使用同一数值时域：

- `sensor_msgs/msg/Imu`：对应实际 IMU 采样时刻，不得使用 RK3588 收到 UART
  数据的时刻；
- 正式 UWB 测距消息：对应双方冻结的实际测距事件时刻，不得使用结果计算或
  ROS 发布时间；
- GNSS 后备使用的 `sensor_msgs/msg/NavSatFix`：必须先把 GNSS 测量时刻转换到
  `UWB_SYSTEM_TIME`，不得把真实 UTC 原样混入；
- 浙大 `NodeState` 和协同结果会继承上述 UWB 时间戳。

同一 IMU 流、同一 `node_id` 的 NodeState 和同一无向 UWB 边的时间必须非零、
严格递增；不同车或不同边允许使用同一量测历元。网络到达时间只用于延迟和超时检查，
不得覆盖量测时间。

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
- 当前消息尚无已冻结的 `sync_epoch/session_id`。若 Master 重启造成时间回拨或超过配置门限的
  向前跳变，首版必须整组重启三车本地惯导节点和参考车融合节点；若启用了 GNSS 后备，
  还要清掉它保存的 NodeState/GNSS 时间基线，最简单是重启参考车整个 `vehicle.launch.py`。
  不能在旧后验或旧时间缓存上续算。
- GNSS 后备只处理“UWB 测距不可用、但 UWB SYNC/PPS 授时仍有效”的情况。
  如果 UWB 授时本身失效，GNSS 距离不能单独维持当前协同滤波时间轴。
- GNSS 后备同时要求三车 `NodeState` 新鲜且处于同一 UWB 时域；任一车状态超时
  或 GNSS 时间戳仍是 REAL_UTC 时，不生成派生距离。

## 4. 上交联调最小验收

1. 保存三车 IMU、三车 `NodeState`、三边 UWB 和接收日志，证明同历元数据时间差、
   严格单调和端到端延迟满足双方冻结门限。
2. 断开一个 Anchor 的 SYNC，验证对应状态和协同位置在超时后变为无效；恢复后
   不出现旧包倒灌。
3. 重启 Master，分别验证新时间回拨和向前跳变时都按首版约定整组重启，旧时间历元与新历元不混算。
4. GNSS 后备分别验证：测距故障但授时正常时可运行；授时失效或 REAL_UTC 未转换
   时必须拒绝。
5. “约 12 次中值”和“百微秒级”属于硬件实现/待验收指标，不写成浙大算法固定
   常量或同步精度承诺。

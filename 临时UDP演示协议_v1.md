# ZJCL 临时 UDP 演示协议与 ZJLG 日志格式 v1

本文只定义当前三车 UWB 在线 Demo 的进程间联调格式。它用于模拟器、`zju_coop_online`、GCS 和回放测试，不替代上交 AIBrainBox 的最终 ROS 2 消息，也不表示浙大承担无线通信、路由或 ROS 2 驱动。

权威实现为 `src/protocol/wire_protocol.*` 和 `tools/zjcl_protocol.py`；二者有跨语言黄金字节测试。所有多字节整数和 IEEE-754 浮点数均为小端序。布尔值只能是 `0` 或 `1`，所有保留字段必须为 `0`，浮点数必须有限。

## 1. ZJCL 帧头

固定头长 40 字节，帧为“帧头 + 固定类型负载”。应用层规定一个 UDP 数据报只装一个完整帧，不允许拼接多个帧或带尾随字节；当前固定帧很小，不依赖应用层分片机制。

| 偏移 | 大小 | 字段 | 约束 |
|---:|---:|---|---|
| 0 | 4 | magic | ASCII `ZJCL` |
| 4 | 1 | major | `1` |
| 5 | 1 | minor | `0` |
| 6 | 2 | message_type | 见消息表 |
| 8 | 2 | header_size | `40` |
| 10 | 2 | flags | 当前在线/回放输出为 `0`；v1 尚未定义位语义 |
| 12 | 4 | payload_size | 必须等于该消息固定长度 |
| 16 | 8 | sequence | Range 按有向节点对递增；算法输出按进程全帧递增 |
| 24 | 8 | timestamp_ns | 上交统一时间轴，纳秒 |
| 32 | 2 | source_node | 源节点；网络级消息可为主参考节点 |
| 34 | 2 | target_node | 无目标时为 `0` |
| 36 | 4 | crc32 | CRC-32/IEEE |

CRC 计算输入为帧的 `[0,36)` 字节紧接全部 payload，即不包含帧头中的 CRC 字段。标准校验值 `CRC32("123456789")=0xCBF43926`。

| message_type | 名称 | payload 长度 | 方向 |
|---:|---|---:|---|
| 1 | Range | 24 | 模拟 UWB → 在线算法 |
| 100 | Localization | 64 | 算法 → GCS |
| 101 | Network | 20 | 算法 → GCS |
| 102 | Observation | 80 | 算法 → GCS |
| 103 | Alert | 40 | 算法 → GCS |
| 104 | AlgorithmStatus | 48 | 算法 → GCS |

UDP 数据报最大 65507 字节，因此 UDP 情况下 payload 上限为 65467 字节；本协议当前所有负载均远小于该值。通用硬上限为 1 MiB，但固定消息仍必须严格等于表中长度。

## 2. Range，24 字节

节点对由帧头的 `source_node`、`target_node` 给出；序号和测量时间使用帧头字段。

| 偏移 | 类型 | 字段 | 约束 |
|---:|---|---|---|
| 0 | f64 | range_m | `>0` |
| 8 | f64 | range_std_m | `>0` |
| 16 | f32 | nlos_probability | `[0,1]` |
| 20 | u8 | nlos_flag | 布尔 |
| 21 | u8 | has_nlos_probability | 布尔 |
| 22 | u8 | valid | 布尔 |
| 23 | u8 | status | `0 OK, 1 DEGRADED, 2 INVALID` |

正式 ROS 2 适配时，上交还必须在调用 C ABI 时提供本机 `receive_timestamp_ns`。它不在 UDP Range payload 中，由在线接收进程在收包时生成；最终盒上必须与测量时间使用同一时间基准。

## 3. Localization，64 字节

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0/8 | f64 | x, y（主参考节点坐标系，m） |
| 16/24 | f64 | vx, vy（m/s） |
| 32/40/48 | f64 | cov_xx, cov_xy, cov_yy |
| 56 | u8 | state：`0 未初始化, 1 正常, 2 降级, 3 不可观, 4 陈旧` |
| 57 | u8 | valid |
| 58 | u8 | yaw_valid；UWB-only 必须为 `0` |
| 59 | u8 | z_valid；UWB-only 必须为 `0` |
| 60 | u32 | capability_mask |

能力位：bit0 `UWB_RANGE`、bit1 `PLANAR_POSITION`、bit2 `VELOCITY`。当前初版输出三位，但不输出航向和高度。

## 4. Network，20 字节

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | node_count |
| 4 | u32 | reachable_node_count |
| 8 | u32 | active_edge_count |
| 12 | u8 | connected |
| 13 | u8 | observable |
| 14 | u8 | state，与 Localization 相同 |
| 15 | u8 | reserved=`0` |
| 16 | u32 | reason_mask |

拓扑必须自洽：`reachable<=node_count`，连通值必须与可达节点数一致，可观必然连通，有效边数不得超出简单无向图范围。

## 5. Observation，80 字节

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0/8 | u64 | window_start_ns, window_end_ns |
| 16/20/24/28 | u32 | expected, received, valid, nlos count |
| 32/36 | u32 | residual_rejected_count, dropped_count |
| 40/48 | f64 | nlos_ratio, valid_ratio |
| 56 | f64 | actual_rate_hz |
| 64 | f64 | covariance_scale |
| 72 | u8 | observation state |
| 73 | u8 | fusion action |
| 74 | u8 | input_overflow |
| 75 | u8 | reserved=`0` |
| 76 | u32 | reason_mask |

观测状态：`0 UNKNOWN, 1 NORMAL, 2 DEGRADED, 3 SUSPENDED, 4 REJECTED, 5 RECOVERING`。融合动作：`0 USE_NORMAL, 1 USE_DOWNWEIGHTED, 2 HOLD, 3 REJECT, 4 TRIAL_RECOVERY`。

## 6. AlgorithmStatus，48 字节

在线程序按配置的输出频率发送该消息，`demo.ini` 默认 10 Hz；回放 `stream` 模式按同一配置沿日志时间轴重新输出，`final` 模式只输出最终快照且是省略 `--output-mode` 时的默认模式，`none` 不发送 UDP。

| 偏移 | 类型 | 字段 | v1 约束 |
|---:|---|---|---|
| 0 | u32 | abi_version | `0x00010000` |
| 4 | u32 | software_version_packed | `major<<16 \| minor<<8 \| patch`；当前 `0x00000100` |
| 8 | u8 | mode | `1 UWB_ONLY_PLANAR` |
| 9 | u8 | run_state | `0 INITIALIZING, 1 RUNNING, 2 DEGRADED, 3 ERROR, 4 STOPPED` |
| 10 | u16 | reserved | `0` |
| 12 | u32 | reserved | `0` |
| 16 | u64 | accepted_ranges | `PROCESSING_PROCESSED` 累计数；可包含 NIS 拒绝（包已处理但未做测量校正） |
| 24 | u64 | rejected_ranges | 非 `PROCESSING_PROCESSED` 处理结论累计数 |
| 32 | u64 | protocol_errors | 帧/payload 解码错误累计；不含已成功解码但方向/类型不符的包 |
| 40 | u64 | uptime_ns | online 为稳态运行时长；replay 为日志时间轴经过时长 |

online 对成功解码但 `message_type` 非 Range 的帧另计 `type_rejected`，不写入 `protocol_errors`。replay 对 `direction=Output` 的历史记录计 `output_records_skipped`；Input 中成功解码但非 Range 的记录只计 `input_types_skipped`，Input 帧或 Range payload 解码失败则同时计 `input_types_skipped` 和 `protocol_errors`。

## 7. Alert，40 字节

当前只有网络级告警：`alert_code=1 NETWORK_STATE`、`source=0 ALGORITHM`。

| 偏移 | 类型 | 字段 |
|---:|---|---|
| 0 | u32 | alert_code |
| 4 | u8 | level：`0 INFO, 1 WARNING, 2 ERROR, 3 CRITICAL` |
| 5 | u8 | lifecycle：`0 ACTIVE, 1 CLEARED` |
| 6 | u8 | source |
| 7 | u8 | reserved=`0` |
| 8 | u32 | reason_mask |
| 12 | u16 | node_id；网络级无单节点时为 `0` |
| 14/16 | u16 | from_node, to_node；网络级时为 `0` |
| 18 | u16 | reserved=`0` |
| 20/28 | u64 | first_timestamp_ns, last_timestamp_ns |
| 36 | u32 | reserved=`0` |

`ACTIVE` 的 `reason_mask` 必须非零；`CLEARED` 的 `reason_mask` 必须为零；`first_timestamp_ns<=last_timestamp_ns`。网络从非正常状态恢复时只发一次 `CLEARED`，且保留对应活动告警的首次时间。启动后直接进入正常状态时不制造虚假告警。

## 8. reason_mask

| bit | 含义 |
|---:|---|
| 0 | NLOS 比例高 |
| 1 | 有效观测比例低 |
| 2 | 观测频率低 |
| 3 | 测距残差高 |
| 4 | 时间同步/时间一致性超限 |
| 5 | 协同边超时 |
| 6 | 图几何退化 |
| 7 | 节点不可达 |
| 8 | 缺少初始化 |
| 9 | 输入容量溢出 |

## 9. ZJLG v1 日志

文件头固定 8 字节：`ZJLG`、major=`1`、minor=`0`、header_size=`8`（u16 小端）。之后顺序存放记录：

| 相对偏移 | 大小 | 字段 |
|---:|---:|---|
| 0 | 4 | ZJCL 帧长度 `frame_length` |
| 4 | 1 | direction：`1 Input, 2 Output` |
| 5 | 3 | reserved=`0` |
| 8 | 8 | 本机接收/生成时间 `receive_timestamp_ns` |
| 16 | frame_length | 完整且已通过 CRC 校验的 ZJCL 帧 |

日志记录输入和输出，便于审计；回放只把 `Input Range` 重新送入算法，历史输出只用于保持时间轴，不作为输入。损坏、截断、超限或保留位非零的日志必须报错退出，不做静默容错。

## 10. 变更规则

- v1 的字段偏移、大小、枚举值和 CRC 口径为冻结项。
- 任何不兼容改动必须提升 major；增加向后兼容能力至少提升 minor，并同步 C++、Python 编解码和黄金字节测试。
- 最终 ROS 2 消息可以采用不同封装，但映射到 C ABI 时不得丢失节点号、序号、测量时间、接收时间、测距方差、NLOS 质量和有效状态。
- 在三方正式接口评审前，禁止将本文直接作为上交无线链路或交大 GCS 的最终交付协议。

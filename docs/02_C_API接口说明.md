# C ABI 接口说明

> 本文同时覆盖UWB-only和15维IMU＋UWB模式；完整声明以 `include/zju_coop/c_api.h` 为准。

公开接口位于 `include/zju_coop/c_api.h`。该头文件为纯 C，可由上交的 ROS 2 C++ wrapper、其他 C/C++ 进程或动态加载器调用。

## 1. 版本与库

| 项目 | 值 |
|---|---|
| 软件版本 | `0.1.0` |
| C ABI | `0x00010000` |
| Windows 动态库 | `zju_coop.dll` + import library |
| Linux 目标动态库 | `libzju_coop.so` |
| 线程模型 | 同一 handle 非线程安全，调用方串行化 |

调用方必须以 `zju_coop_abi_version()` 和 `zju_coop_version_string()` 检查实际加载版本。所有公开结构先调用匹配的 `*_init` 函数，不得依赖编译器默认清零或本地 packing。

## 2. 生命周期

```c
zju_coop_config_t config;
zju_coop_config_init(&config);
/* 填入参考节点、节点数组和配置 */

zju_coop_handle_t* handle = NULL;
zju_coop_create(&config, &handle);

/* 每条测距 */
zju_coop_range_packet_t range;
zju_coop_range_processing_result_t result;
zju_coop_range_packet_init(&range);
zju_coop_range_processing_result_init(&result);
zju_coop_push_range(handle, &range, &result);

/* 10 Hz 获取快照；先查询容量，再提供缓冲区 */
zju_coop_step(handle, now_ns,
              NULL, 0, 0, &localization_count,
              NULL, 0, 0, &observation_count,
              NULL);

zju_coop_destroy(handle);
```

完整可编译调用见 `examples/sdk_consumer.cpp`。

## 3. 配置与所有权

`zju_coop_create` 只在调用期间借用节点数组，并在创建成功前完成深拷贝。调用方拥有所有输入、配置和输出内存。

输出数组采用显式 byte stride。调用方应：

1. 空缓冲查询所需 `localization_count` 和 `observation_count`；
2. 按容量分配数组；
3. 对每个元素调用对应 init；
4. 以真实 `sizeof` 作为 stride 再次调用 `zju_coop_step`。

容量、结构头或转换失败时，API 不部分写入输出；成功调用才提交预测后的引擎状态和整组快照。容量查询本身不推进算法时间。

## 4. Range 输入

| 字段 | 要求 |
|---|---|
| `from_node/to_node` | 已配置节点，二者不同 |
| `sequence` | 有向链路内递增，用于重复检测 |
| `timestamp_ns` | 上交统一时间轴中的采样时刻 |
| `receive_timestamp_ns` | wrapper 本地接收时刻，必须非零且与采样时刻可比较 |
| `range_m` | 有限、>0，单位 m |
| `range_std_m` | 有限、>0，单位 m；不是方差 |
| `nlos_probability` | `[0,1]`；缺失时配合 `has_nlos_probability=false` |
| `nlos_flag` | 设备或上交前端的 NLOS 判定 |
| `valid` | 硬件、字段和同步均可用时才为 true |
| `status` | 0 OK；1 DEGRADED；2 INVALID |

`zju_coop_push_range` 返回函数调用错误码，同时通过 `zju_coop_range_processing_result_t` 返回该包的处理结论、融合动作、滤波更新结论、创新量、NIS 和协方差缩放。

## 5. 输出

### `zju_coop_localization_t`

每节点一条，包含时间、节点/参考节点、`x/y/vx/vy`、二维位置协方差、valid 和 LocalizationState。首期 yaw/z 始终无效。

### `zju_coop_network_t`

包含节点数、参考节点可达数、有效边数、连通/可观标志、reason_mask 和聚合 LocalizationState。

### `zju_coop_observation_t`

每候选边一条，包含滑动窗口计数、NLOS/有效率、实际频率、ObservationState、FusionAction、reason_mask、输入溢出和协方差缩放。

## 6. 错误处理

API 不跨 ABI 抛出 C++ 异常。公开函数返回：

| 错误码 | 含义 |
|---:|---|
| 0 | OK |
| 1 | INVALID_ARGUMENT |
| 2 | ABI_MISMATCH |
| 3 | STRUCT_SIZE_MISMATCH |
| 4 | BUFFER_TOO_SMALL |
| 5 | OUT_OF_MEMORY |
| 6 | INTERNAL_ERROR |

使用 `zju_coop_error_string()` 获取固定错误文本。上交 wrapper 应记录错误码、函数名和输入序列号，不得忽略非零返回值。

## 7. ABI 注意事项

- C 结构不是网络字节布局，禁止直接 `send(sizeof(struct))`；
- 不跨 ABI 传递 STL、C++ `bool`、引用、异常或所有权不明内存；
- `struct_size` 使用调用方实际 `sizeof`，数组使用显式 stride；
- ARM64 的结构布局、动态库加载和导出符号仍需在 AIBrainBox 真机复验。

## 8. 15维IMU＋UWB调用顺序

惯性模式必须在首个Range、IMU或成功step之前完成配置：

```text
zju_coop_create
  → zju_coop_inertial_node_initialization_init（每个节点）
  → zju_coop_inertial_config_init
  → zju_coop_configure_inertial（仅一次）
  → zju_coop_imu_packet_init / zju_coop_push_imu（每帧）
  → zju_coop_range_packet_init / zju_coop_push_range（每帧）
  → zju_coop_step（按输出频率）
  → zju_coop_destroy
```

`zju_coop_imu_packet_t` 是普通C结构，不含ROS 2类型和温度。四元数顺序为x/y/z/w；角速度单位rad/s；线加速度单位m/s²并按比力解释；三组协方差为3×3行主序；`frame_id`必须在32字节内以NUL结尾。第一帧只建立时间基准，返回 `ZJU_COOP_IMU_BASELINE_ESTABLISHED`；第二帧起在时间间隔有效时返回 `ZJU_COOP_IMU_PROPAGATED`。

同一handle不可并发调用。建议上交使用单线程executor或互斥回调组；若使用多线程executor，必须在wrapper外部串行化所有API调用。

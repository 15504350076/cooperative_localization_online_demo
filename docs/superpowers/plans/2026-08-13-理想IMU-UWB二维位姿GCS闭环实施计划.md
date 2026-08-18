# 理想 IMU-UWB 二维位姿 GCS 闭环实施计划

## 任务 1：内部航向输出

涉及文件：

- `src/core/quaternion.hpp/.cpp`
- `src/core/range_ekf.hpp`
- `src/core/cooperative_inertial_ekf.cpp`
- `src/core/engine.hpp/.cpp`
- 对应核心测试

步骤：先添加航向边界和有效性失败测试，再实现 FLU 前向轴到 ENU 的航向提取；把 `yaw_rad/yaw_valid` 贯通到内部单车估计与引擎快照。UWB-only 路径保持航向无效。

## 任务 2：新增兼容 Pose2D C ABI

涉及文件：

- `include/zju_coop/c_api.h`
- `src/api/c_api.cpp`
- `tests/test_c_api.cpp`
- `tests/c_header_smoke.c`

步骤：保留 ABI v1 布局和返回版本；添加 v2 结构初始化、版本查询和只读快照查询；验证两阶段缓冲区调用、步长检查、参考节点、时间戳和三车字段映射。

## 任务 3：理想初始航向与模拟输入

涉及文件：

- `config/demo.ini`
- 配置解析文件
- `tools/imu_uwb_simulator.py`
- 对应配置和模拟器测试

步骤：允许按节点配置初始 ENU 航向并转换成初始四元数；模拟器继续输出瞬时角速度和比力，可配置 z 轴角速度；严禁用每帧外部姿态覆盖惯导递推。

## 任务 4：临时 UDP Pose2D 协议

涉及文件：

- `include/zju_coop/protocol.hpp`
- `src/protocol/protocol.cpp`
- Python 协议解析器
- C++/Python 协议测试

步骤：新增消息类型 105 和固定 32 字节载荷；验证大小端、CRC、有效标志、能力位和旧消息兼容。

## 任务 5：在线程序和 GCS 看板

涉及文件：

- `src/apps/app_support.hpp/.cpp`
- `src/apps/online_main.cpp`
- 必要的回放显示代码
- `tools/gcs_dashboard.py`
- 在线 smoke 测试

步骤：原有 step 每周期只调用一次；随后只读查询 Pose2D；把新帧与旧遥测一起发送。看板增加 yaw 数值和方向箭头，分别处理位置与航向有效性。

## 任务 6：回归验证和交付说明

涉及文件：

- `README.md`
- `docs/02_C_API接口说明.md`
- `docs/03_临时在线协议说明.md`
- `docs/05_联调与验收说明.md`
- `docs/09_当前功能与上交联调清单.md`

验证顺序：

1. 配置并构建 Release。
2. 运行全部 CTest。
3. 运行独立自检程序。
4. 启动在线程序、理想 IMU/UWB 模拟器和 GCS 看板进行端到端 smoke。
5. 检查旧 range-only 演示与旧协议回归。
6. 审查工作树，仅列出本轮实际改动；不执行 Git 提交。

## 完成判据

所有自动测试通过，GCS 可同时显示三车相对二维位置和各自航向，旧 ABI v1 与旧演示路径保持可用，并明确标注 ROS 2/RK3588/实车尚待后续验证。

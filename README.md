# 多车协同定位在线演示系统

本工程默认运行“标准 ROS 2 IMU 瞬时量驱动的每节点 15 维 ESKF＋平台间测距联合更新”。同时保留仅测距兼容配置，供 IMU 暂不可用时回退和自动回归测试。当前已落地纯 C++ 算法核心、稳定 C ABI、临时 ZJCL/UDP＋浏览器 GCS，以及每车本地惯导、紧凑 NodeState、参考车二维 UWB 修正、ROS 2 消息/节点/bringup 和 Gazebo 三车闭环。算法核心继续与 ROS 2 消息解耦，原集中式 15N 实现保留为回归基线。

当前结论：只使用三条车间测距可以跑通在线演示。位置输出为**各节点相对参考节点的二维 x/y，坐标轴与公共 ENU 平行**，不代表经纬度或地固系绝对位置。三边测距只能确定三角形形状，整体平移、旋转和镜像仍有自由度；默认惯性演示通过参考节点 1、三车初始位置和三车公共 ENU 初始航向固定这些自由度。C ABI v2、临时 UDP Pose2D 和 GCS 当前可输出各车 ENU 航向；C ABI v1 的旧 `Localization` 仍固定 `yaw_valid=false`、`z_valid=false`。

## 已实现范围

- C++17 算法核心：默认每节点 15 维误差状态 `[δp,δv,δθ,δbg,δba]`，另保留仅测距二维恒速回退模型。
- 标准 `sensor_msgs/Imu` 字段映射：角速度、线加速度、三组协方差和可选首帧姿态；不虚构标准消息中不存在的温度。
- ENU导航系/FLU车体系、中值瞬时量积分、IMU零偏、过程噪声、15N联合协方差和事务式数值回滚。
- UWB三维欧氏距离雅可比、完整15N增益、NIS门限、Joseph协方差更新和主参考相对输出。
- 三车及配置容量上限内的稀疏多节点初始化；协同边由有效观测动态产生，没有固定三边或固定路径的算法假设。
- 网络连通性、二维几何可观性、边超时和节点可达性判断。
- UWB NLOS、有效率、频率、残差的滑窗质量评估，以及正常使用、降权、暂缓、剔除和试探恢复。
- 稳定 C ABI 动态库 `zju_coop`；核心接口只使用普通 C 结构体，不依赖 ROS 2 消息。
- 分布式首版：每车本地 15 维惯导输出紧凑 NodeState，参考车按 UWB 时刻对齐三车状态并维护独立二维修正；协同修正不覆盖本地惯导。
- 整组三车统一算法时刻必须新鲜；非参考车还只有经近期且被滤波接受的测距边连通到参考车时才设置 `position_valid=true`。任一车状态冻结或测距失联后仍保留有限数值用于诊断，但 GCS/规划不得继续使用无效结果。
- ROS 2 Humble 首版：`cooperative_localization_msgs`、`zju_coop_ros2`、`zju_coop_bringup` 和可选 `zju_coop_gazebo`，输出 GCS 所需 `CooperativePose2DArray`。
- 默认关闭的 GNSS 距离后备：只在 UWB 时间同步暂不可用时显式启动，与真实 UWB 输入严格互斥，不作为 UWB 或定位精度验收依据；当前输出没有机器可读的测距来源字段，因此该模式只供 GCS 应急显示，禁止接入规划控制。
- C ABI v1 保留定位/观测/网络输出；Pose2D C ABI v2 通过只读 `zju_coop_get_pose2d_v2()` 输出公共快照时间、参考节点、frame 和各车 x/y/yaw，不会重复推进滤波。
- 在线程序 `zju_coop_online`、回放程序 `zju_coop_replay`、UWB/IMU＋UWB模拟器和浏览器 GCS 面板；临时 UDP 类型 105 输出 Pose2D，面板按共同时间与参考节点收齐三车后原子更新，按有效位显示位置/航向，并显示参考节点、快照时间和坐标系；在线/回放适配层还根据库的网络结果生成状态和告警帧。
- 输入、输出统一事件日志；回放会跳过历史输出记录，以历史输入重新计算并重新产生定位、状态和告警。
- 严格帧长、枚举、保留位、数值范围和 CRC 校验；所有数据携带时间戳、序号和节点号。

尚未实现或未验证：上交正式 UWB 消息包替换、真实 UWB/IMU/GNSS 驱动与时间同步、三盒 DDS/5G Mesh、激光雷达/多目视觉前端、绝对位置约束，以及 RK3588/ARM64 真实盒端构建、运行和性能指标。GNSS 后备只把三车位置几何转成低置信度距离，不是 GNSS 绝对位置融合。当前 Ubuntu 22.04 x86-64 同机 ROS 2/Gazebo 结果不能替代上述验证。C ABI v1 `Localization` 仍只发布二维相对位置和速度，`yaw_valid=false`、`z_valid=false`；GCS 航向由 Pose2D v2 链路提供。

## 交付边界与分工

```text
上交 AIBrainBox 驱动、统一时间与输入消息
  标准IMU + 自定义UWB + 统一时间戳 + DDS/盒端运行环境
                │
                ▼
浙大 ROS 2 适配节点（首版已实现，RK3588待验证）
  ROS 2 消息 ↔ zju_coop C 结构体
                │
                ▼
浙大 libzju_coop.so
  校验 → 每节点15维预测 → UWB联合更新 → 质量/拓扑 → 相对结果
                │
                ▼
同一浙大 ROS 2 适配节点发布结果 → DDS → 交大 GCS
```

- 浙大已实现并计划交付 `libzju_coop.so`、`c_api.h`、调用算法库的 ROS 2 适配节点、最小结果消息定义与发布逻辑；同时负责 UWB 协同定位、质量处理、动态拓扑、定位/观测/网络结果，以及演示层状态/告警语义和算法级回放能力。
- 上交负责 AIBrainBox 内外设驱动、单平台与多平台统一时间、UWB 原始数据形成、标准/自定义输入 ROS 2 消息、DDS 与盒端构建运行环境、无线转发/路由，以及向浙大适配节点提供冻结后的输入 topic、消息和 QoS。
- 交大 GCS 负责展示定位结果、拓扑、状态和告警；GCS 不承担传感器前端、时间同步、协同定位求解或车辆控制。
- `ZJCL/UDP` 是当前不依赖 ROS 2 的临时联调与演示适配层。正式部署计划由浙大 ROS 2 适配节点在 AIBrainBox 上调用 C ABI 并发布结果消息，经上交提供的 DDS/盒端环境交给 GCS；UDP 演示协议不作为正式 ROS 2/DDS 接口。

正式适配时，`timestamp_ns` 必须来自上交统一时间轴，`receive_timestamp_ns` 必须是同一时间基准下的本机接收时刻；两者单位均为纳秒。算法会用二者检查未来时间偏差和传输延迟。时间同步无效时，上交应在双方确认的输入状态接口中报告同步失效，并停止发布或标记对应测距无效；浙大适配节点不得补造测量时间。

## 工程结构

| 路径                                                               | 内容                                                  |
| ------------------------------------------------------------------ | ----------------------------------------------------- |
| `include/zju_coop/c_api.h`                                       | 对上交交付的稳定 C ABI                                |
| `src/api`                                                        | C ABI 与 C++ 核心适配                                 |
| `src/core`                                                       | EKF、退化监测、动态拓扑和可观性                       |
| `src/protocol`                                                   | 临时 ZJCL 帧和 ZJLG 日志                              |
| `src/apps`                                                       | 在线程序、回放程序、C ABI 调用适配                    |
| `src/net`                                                        | 当前演示使用的跨平台 UDP 封装                         |
| `config/demo.ini`                                                | 默认 15 维 IMU＋测距融合参数和三车初值                |
| `config/range_only_demo.ini`                                     | 仅测距回退模式参数和三车初值                          |
| `docs`                                                           | 算法、C API、协议、盒端适配、验收、扩展和交付验证说明 |
| `tools/uwb_simulator.py`                                         | 确定性 3-4-5 三角形 UWB 模拟器                        |
| `tools/imu_uwb_simulator.py`                                     | 三车100 Hz IMU＋20 Hz UWB模拟器                       |
| `tools/gcs_dashboard.py`                                         | 零第三方依赖的二维在线面板                            |
| `tools/online_smoke_test.py`、`tools/imu_online_smoke_test.py` | 两种模式的真实进程测试                                |
| `examples/sdk_consumer.cpp`                                      | 直接调用 C ABI 的可编译示例                           |
| `tests`、`tools/test_*.py`                                     | C/C++ 与 Python 自动测试                              |
| `临时UDP演示协议_v1.md`                                          | 当前联调帧、状态、告警和日志格式                      |

建议第一次使用或接手开发时先阅读 [`docs/12_工程使用学习与编写思路.md`](docs/12_工程使用学习与编写思路.md)。该文档按“最小自检→完整在线演示→代码学习路线→二次开发原则”组织，并说明Windows、Ubuntu 22.04和RK3588的使用差异。

## 三车演示数据

`config/demo.ini` 的初始相对坐标为：节点 1 `(0,0,0)`、节点 2 `(3,0,0)`、节点 3 `(0,4,0)`；公共 ENU 初始航向分别为 `0`、`π/2`、`-3π/4` rad。默认模拟器发送三车 100 Hz 静止 IMU 和 20 Hz 的 3-4-5 m 测距；节点 1 是参考节点，算法按有效观测动态激活边。理想初始对准必须为每辆车提供公共 ENU 下的初始航向，后续航向仅由瞬时 IMU 角速度递推。UWB 标量测距不直接观测航向，静止或缺少运动激励时不能依靠测距校正初始航向误差。

若实际三车只有车间 UWB 而没有固定锚点，系统可维持该主参考坐标系中的相对队形。若要得到地图/地理绝对位置，至少还需引入已知锚点、短时有效 GNSS、地图匹配或其他绝对约束；若要消除镜像和获得可靠航向，还需 IMU、运动激励或视觉/激光方向约束。

## 构建与测试

### Windows 与 RK3588 是否使用同样代码

算法源码、C 头文件、配置字段和调用顺序相同；生成的二进制不同，必须分别编译：

| 项目          | Windows x64                            | AIBrainBox RK3588                            |
| ------------- | -------------------------------------- | -------------------------------------------- |
| 算法源码      | 同一份`src/core`、`src/api`        | 同一份`src/core`、`src/api`              |
| 对外接口      | 同一份`include/zju_coop/c_api.h`     | 同一份`include/zju_coop/c_api.h`           |
| 动态库        | `zju_coop.dll`＋导入库               | `libzju_coop.so`                           |
| 指令集/ABI    | x86-64、MSVC ABI                       | AArch64、GCC/Clang ELF ABI                   |
| ROS 2 适配节点 | 已在 Ubuntu 22.04/ROS 2 Humble x86-64 实现和测试 | 使用同一源码在 RK3588/AArch64 重建；当前未验证 |
| 临时在线入口  | ZJCL/UDP 独立 Demo                     | 可用于盒端自检；生产链路计划采用浙大 ROS 2 适配节点 |

因此不能把 Windows DLL 复制到 RK3588，也不能把 ARM64 `.so` 放到 Windows。浙大 ROS 2 适配节点源码需要在每个平台各自的 ROS 2、编译器和消息包环境中重新构建。算法库本身不包含 ROS 2 消息依赖；当前仓库已有首版节点与消息包，但尚未取得 RK3588 和上交正式 UWB 包的部署证据。

### Windows 本机

需要 Visual Studio 2022 C++ 工具链、CMake 3.16 以上和 Python 3。单配置与多配置生成器均受支持；Visual Studio 生成器示例：

```powershell
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

本机已实际使用 Visual Studio 17 2022、CMake 和 Python 3.11.9 完成 Release 编译；仅测距与IMU＋测距两条真实 UDP 在线/日志/回放进程链均通过。最终数量以本次交付记录和实际测试输出为准。

### 独立自动自检

只需验证算法动态库和最小三车 IMU＋测距闭环时，可运行独立入口。它不启动 UDP、ROS 2、GCS 或硬件驱动，也不读取或修改实车配置：

```powershell
build\Release\zju_coop_self_check.exe
```

Linux/RK3588 单配置构建对应为：

```bash
./build/zju_coop_self_check
```

当前版本全部检查通过时输出 `SELF_CHECK PASS (passed=14, failed=0)` 并返回退出码 0；新增 `[PASS] pose2d_snapshot`，旧 v1 有效位检查仍保留。详细验证项、安装树命令和适用边界见 `docs/11_独立自检程序说明.md`；自检不能替代真实传感器、ROS 2 和定位精度验收。

### AIBrainBox Ubuntu 22.04 / ARM64

```bash
sudo apt update
sudo apt install -y build-essential cmake python3
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
cmake --install build --prefix install
```

根目录算法库构建没有 ROS 2 编译依赖，便于先独立验证 SDK；`ros2/` 下的适配包再由 colcon 单独构建。Ubuntu 22.04 ARM64/AArch64 尚未在实际 AIBrainBox 上编译和压力测试，以上仅为目标构建命令；上盒后必须重新记录编译、测试、时钟源、CPU 占用和连续运行结果，不能沿用 x86-64 结果代替。

## 启动在线演示

以下以单配置 Linux/Release 路径为例；Windows Visual Studio 构建时将可执行文件替换为 `build\Release\*.exe`。

终端 1，启动 GCS 面板：

```bash
python3 -B tools/gcs_dashboard.py --udp-port 39002 --http-port 8080
```

终端 2，启动在线算法：

```bash
./build/zju_coop_online --config config/demo.ini
```

终端 3，发送三车 IMU 和平台间测距：

```bash
python3 -B tools/imu_uwb_simulator.py --host 127.0.0.1 --port 39001
```

浏览器访问 `http://127.0.0.1:8080/`。默认惯性演示中，面板应显示三车相对 x/y 和按 `yaw_valid` 绘制的 ENU 航向箭头。停止模拟器后等待超过 `edge_timeout_ns=500000000`，面板应由正常转为降级/数据陈旧，并显示活动网络告警；重新发送有效测距后会输出一次已清除告警。

完整自动烟雾测试：

```bash
python3 -B tools/imu_online_smoke_test.py \
  --online-exe build/zju_coop_online \
  --replay-exe build/zju_coop_replay \
  --config config/demo.ini
```

该测试先形成三车正常网络，再停止输入超过边超时，必须观测到算法状态帧和正常之后的活动告警；随后读取生成的 ZJLG 日志并验证回放重新输出状态/告警。

### 仅测距兼容模式

仅当 IMU 暂不可用或需要跑历史回归时，终端 2 显式使用回退配置：

```bash
./build/zju_coop_online --config config/range_only_demo.ini
```

终端 3 仅发送三车测距：

```bash
python3 -B tools/uwb_simulator.py --host 127.0.0.1 --port 39001 --rate-hz 20
```

兼容模式端到端自动验证：

```bash
python3 -B tools/online_smoke_test.py \
  --online-exe build/zju_coop_online \
  --replay-exe build/zju_coop_replay \
  --config config/range_only_demo.ini
```

## 日志回放

默认配置写入 `logs/cooperative_localization_demo.zjlg`。先让 GCS 监听输出端口，再运行：

```bash
./build/zju_coop_replay \
  --config config/demo.ini \
  --log logs/cooperative_localization_demo.zjlg \
  --speed 1 \
  --output-mode stream
```

`--speed 0` 表示不等待、尽快处理；`--output-mode stream` 按日志时间轴重新输出，`final` 只输出最终快照且是省略该参数时的默认模式，`none` 只验证/计算而不发 UDP。回放不把历史 Output 记录再次当作算法输入。

## C ABI 对接

最终上交算法适配层只需包含 `zju_coop/c_api.h` 并链接动态库。实际、可编译的调用顺序见 `examples/sdk_consumer.cpp`：

1. 对每个版本化结构体调用对应的 `*_init`；
2. 填写节点初值和 `zju_coop_config_t`，调用 `zju_coop_create`；
3. 惯性模式在任何输入前调用 `zju_coop_configure_inertial`；
4. 将 `sensor_msgs/Imu` 映射为 `zju_coop_imu_packet_t` 并调用 `zju_coop_push_imu`，将UWB消息映射为 `zju_coop_range_packet_t` 并调用 `zju_coop_push_range`；
5. 先用空数组调用 `zju_coop_step` 查询输出数量，再准备调用方拥有的数组并取得定位、观测和网络结果；
6. 惯性模式在同一次 `step` 完成后调用只读 `zju_coop_get_pose2d_v2`，先查询车辆数量，再取得公共快照和各车 x/y/yaw；该查询不推进滤波；
7. 同一个 handle 不支持并发调用，浙大 ROS 2 适配节点必须串行化；
8. 退出时调用 `zju_coop_destroy`。

库会深拷贝初始化数据；输出内存由调用方管理。基础 C ABI v1 版本为 `0x00010000`，Pose2D 扩展 ABI v2 版本为 `0x00020000`，当前软件版本为 `0.3.0`。软件发布版本与结构体 ABI 版本相互独立。安装：

```bash
cmake --install build --prefix install
```

安装树包含动态库、C 头文件、在线/回放程序、配置和演示工具。Linux 可执行程序安装了相对 RPATH，可从 `install/bin` 查找 `install/lib/libzju_coop.so`。

可执行程序的默认配置名 `config/demo.ini` 是相对当前工作目录解析的。使用安装树时必须显式给出安装后的配置路径，避免从任意目录启动时找不到配置：

```bash
./install/bin/zju_coop_online \
  --config ./install/share/zju_coop/config/demo.ini

./install/bin/zju_coop_replay \
  --config ./install/share/zju_coop/config/demo.ini \
  --log ./logs/demo.zjlg --speed 1 --output-mode stream
```

Windows 安装树同理，使用 `install\bin\zju_coop_online.exe --config install\share\zju_coop\config\demo.ini`。相对的日志路径仍以启动进程的当前工作目录为基准。

C ABI v1 的 `zju_coop_step` 当前直接输出 `Localization`、`Observation` 和 `Network` 对应结构，其中旧 `Localization` 的 `yaw_valid=false`。Pose2D C ABI v2 由 `zju_coop_get_pose2d_v2` 只读返回二维位置与各车 ENU 航向。`AlgorithmStatus` 和 `Alert` 当前由在线/回放参考适配层依据 Network 状态生成，正式 ROS 2 节点是否复用这套语义或由后续 C ABI 扩展直接提供，需在接口评审中冻结。

## 当前接口冻结点与待确认项

- C ABI v1 的字段顺序、枚举值、`struct_size/abi_version/stride` 握手和调用语义已由 C/C++ 消费者测试保护；C 结构体不是网络字节格式，ARM64 上的实际大小和 ABI 仍须上盒验证。
- 临时 ZJCL v1 已在既有遥测之外增加类型 105 Pose2D 帧；其 32 字节载荷、`[-π,π)` 航向范围和能力/有效位语义由 C++、Python 测试保护。改变布局必须升级协议版本并同步修改两端测试。
- AlgorithmStatus/Alert 已在在线、GCS 和回放演示链路实现，但尚未进入 C ABI v1 或首版 ROS 2 GCS 消息；正式映射仍为待确认项。
- 首版 ROS 2 适配节点、NodeState、GCS 结果消息和发布代码已实现；上交正式 UWB 包名、输入 topic/QoS、同步状态消息以及跨盒 GCS 的 DDS 接入方式仍需三方书面确认。
- 第一阶段建议上交至少提供：源/目标节点号、单调递增序号、统一测量时间、同时间基准接收时间、距离、标准差、NLOS 标志/概率、有效标志和设备状态。禁止仅给无时间戳的距离值。
- Python 以 `-B` 运行可避免生成 `__pycache__`；不带 `-B` 运行时 Python 可能创建该缓存目录，它不是运行依赖。

# GCS相对定位接入要求、失效处理与当前进度

给上交做首轮显示联调，先用 [GCS最基本对接说明](../GCS最基本对接说明.md)；本文件保留详细设计、历史测试和后续要求。

更新时间：2026-09-20
当前模式：三车 UWB-only，201 盒负责在线解算
当前结果 Topic：`/cooperative_localization/poses_2d`

本文用于浙大算法端与上交 GCS 端冻结第一版接入约定。它重点回答：定位无效时怎么显示、GNSS 是否必需、GCS 应显示哪些传感器状态，以及当前已经能做到哪一步。

## 1. 结论

- 现在已经具备接入 GCS、开始做相对位置显示联调的条件。
- 目前还不应称为生产验收完成：状态/诊断 Topic 尚未冻结，动态运动、故障恢复、GNSS 真值和正式 GCS 端到端试验还没有完成。
- GCS 必须同时检查 `position_valid` 和结果新鲜度，不能只要坐标字段里有数值就绘制为当前有效位置。
- 无效时可以短暂保留最后有效位置供人观察，但必须显示成灰色或黄色历史位置，标明数据年龄，且不能交给控制模块使用。
- GNSS/RTK 真值是可选评价图层，不是 UWB 相对定位的运行前提。没有 GNSS、GNSS 受强干扰或失锁时，GCS 仍应显示 UWB 相对定位；只把 GNSS 真值标记为不可用。
- GCS 应显示传感器状态，并区分“设备已连接”“数据有效”和“当前确实被算法使用”，这三件事不能混为一谈。

## 2. 当前正式位置输出

### 2.1 Topic、类型和 QoS

| 项目 | 当前约定 |
|---|---|
| Topic | `/cooperative_localization/poses_2d` |
| 消息类型 | `cooperative_localization_msgs/msg/CooperativePose2DArray` |
| QoS | Reliable、Volatile、Keep Last 1 |
| 参考节点 | `reference_node_id=1` |
| 当前坐标系 | `coop_ref_1_relative` |
| 当前输出频率 | 约 9.46 Hz |

消息定义见：

- [CooperativePose2DArray.msg](../../ros2/cooperative_localization_msgs/msg/CooperativePose2DArray.msg)
- [VehiclePose2D.msg](../../ros2/cooperative_localization_msgs/msg/VehiclePose2D.msg)

每辆车至少包含：

- `node_id`：车辆编号；
- `x_m/y_m`：相对位置；
- `yaw_rad`：车辆自身航向；
- `position_valid`：当前 x/y 能否使用；
- `yaw_valid`：当前 yaw 能否使用。

当前 UWB-only 模式下，车 1 为原点，车 1 指向车 2 的当前连线为正 X，车 3 固定取正 Y 镜像分支。它能表示三车队形，但该坐标系不是东北天、不是地图坐标系，也不表示任何车辆的车头方向。因此当前应有：

- 有效三角形：`position_valid=true`；
- 车辆自身航向不可观：`yaw_valid=false`。

GCS 不得在 `yaw_valid=false` 时把 `yaw_rad=0` 解释为车辆朝向为 0°。

### 2.2 当前无效输出的真实行为

当前程序有两种不同的失效表现：

1. 三条边齐全，但互惠测距差过大、距离突变或三边不能构成有效三角形：仍发布本历元消息，`header.stamp` 是本历元测量时刻，但所有车辆 `position_valid=false`。坐标字段保留上一帧有效坐标，首个有效解之前则为初始化值。这些数值只供诊断，不能当成新位置。
2. 同一历元缺少任意一条无向边：该历元直接丢弃，不发布 `/cooperative_localization/poses_2d`。

因此 GCS 既要处理“收到无效帧”，也要处理“长时间完全收不到帧”。只做其中一种检查都会把旧位置误当成实时结果。

对应实现见 [cooperative_fusion_node.cpp](../../ros2/zju_coop_ros2/src/cooperative_fusion_node.cpp)：`publish_uwb_formation()` 负责有效位和坐标发布，`solve_uwb_epoch()` 负责缺边、一致性和三角形检查。

## 3. GCS对无效位置的处理要求

### 3.1 推荐的首版状态

当前约 10 Hz 输出，第一版建议采用以下可配置门限：

| 状态 | 判断 | GCS 显示 | 能否供规划/控制使用 |
|---|---|---|---|
| `VALID` | 最新结果 `position_valid=true`，且该有效结果的本地接收年龄不超过 300 ms | 正常颜色显示当前位置 | 可以 |
| `DISPLAY_HOLD` | 最新帧无效或短时未更新，但最后有效位置的本地接收年龄不超过 1 s | 只把最后有效点画成灰色/黄色虚影，同时显示 `HOLD` 和年龄 | 不可以 |
| `LOST` | 最后有效位置超过 1 s 没有更新 | 红色提示定位丢失；实时位置隐藏，只可保留历史轨迹 | 不可以 |
| `OFFLINE` | 超过 2 s 没收到任何结果消息 | 标记算法输出离线，并提示检查进程或 DDS 链路 | 不可以 |

300 ms、1 s 和 2 s 是与当前约 10 Hz 数据匹配的首版工程门限，不是已经证明的性能指标。完成运动和网络抖动试验后可以调整，但无效位置绝不能因为“保持显示”重新变成有效位置。

### 3.2 GCS实现时必须注意

- 收到 `position_valid=true` 时，才更新“最后有效位置”和“最后有效位置接收时刻”。
- 收到 `position_valid=false` 时，不得用消息里的旧坐标覆盖最后有效状态，也不得用这条消息的新 `header.stamp` 把旧坐标伪装成刚更新。
- 当前跨设备统一时基尚未完全冻结，GCS 看门狗优先使用 GCS 进程自己的单调接收时钟，而不是直接计算“电脑当前 UTC 减消息 `header.stamp`”。
- `DISPLAY_HOLD` 只是一种界面呈现。若以后控制系统需要短时预测，应由状态估计器另行输出，并明确携带 `predicted`/`valid` 状态；不能由 GCS 私自把旧值外推或重打时间戳。
- 无效、丢失和恢复告警按状态变化提示即可，不应每帧弹窗。持续故障可以做周期汇总。

换句话说，合理方案不是在“保留旧帧”和“输出告警”之间二选一，而是：立即降低有效状态并告警，同时只在界面上短暂保留最后有效点。

## 4. 终端 WARN 怎么理解

这次 201 终端记录中的融合节点连续运行约 67 分钟，没有 `ERROR`。能看到三类主要 WARN：

- `dropping UWB-only epoch with a missing edge`：该历元未收齐三条边，不发布输出；
- `measurement_consistency=failed`：双向测距差或距离突变超过门限，发布 `position_valid=false`；
- `measurement_consistency=ok, formation=2`：测距通过前一层检查，但三边不能形成有效三角形，发布 `position_valid=false`。

正常有效帧只发布 ROS 消息，不逐帧打印 INFO。上述每个 WARN 调用点还分别使用了 2 秒限频，所以终端看起来几乎全是警告，并不表示所有帧都失败；反过来，WARN 行数也不能用来计算真实失败率。真实比例应从 rosbag 中统计 `position_valid`，并结合缺边输入历元数量分析。

现有 WARN 适合开发调试，不应由 GCS 解析。GCS 必须使用消息有效位、消息新鲜度和后续结构化状态 Topic。

## 5. GNSS真值在GCS中的位置

### 5.1 GNSS不是相对定位运行前提

当前算法的基本输出目标始终是相对于参考车的相对位置。即使：

- 没有安装 GNSS；
- GNSS 无卫星、失锁或受强干扰；
- GNSS 不是 RTK Fixed；

UWB 相对定位都应继续运行，GCS 也应继续显示 `coop_ref_1_relative` 下的队形。此时不能显示绝对经纬度、东北天绝对朝向或相对定位的外部精度真值。

### 5.2 有GNSS时的正确用法

若后续有可靠 RTK，建议把相对真值放在独立 Topic，例如：

`/cooperative_localization/rtk_relative_truth`

GCS 把它作为独立图层：

- 估计结果和 RTK 真值用不同颜色；
- 只有 RTK Fixed、质量字段有效且数据未超时时才计算定位误差；
- RTK Float、Single、失锁、超时或跳变时，把真值图层标记为不可用，不填 0，不沿用旧真值；
- 评价用真值不得回灌同一个定位算法，否则不能再作为独立精度基准。

标准 `sensor_msgs/msg/NavSatFix.status` 本身通常不足以完整区分接收机厂商的 RTK Fixed/Float 状态，上交驱动还需要给出明确解类型或只在满足真值条件时发布专用 Topic。现有 GNSS 算法接口进度见 [14_GNSS初始化与RTK相对真值接口说明.md](14_GNSS初始化与RTK相对真值接口说明.md)。

## 6. GCS需要显示哪些状态

需要显示，但不必一开始把所有原始字段铺满界面。至少应有以下摘要：

### 6.1 定位结果状态

- 当前模式：`uwb_only`、`imu_uwb` 或后续其他模式；
- 参考节点和 `frame_id`；
- `VALID / DISPLAY_HOLD / LOST / OFFLINE`；
- 最后有效位置年龄、最近输出频率；
- 每辆车的 `position_valid` 和 `yaw_valid`。

### 6.2 传感器状态

IMU、UWB、GNSS、激光雷达、相机分别显示：

- `configured`：是否配置接入；
- `receiving`：近期是否收到数据；
- `fresh/valid`：数据是否新鲜且通过质量检查；
- `used_by_fusion`：当前结果是否实际使用该传感器。

例如，雷达驱动进程存在只说明可能已经启动，不等于已有点云，更不等于算法正在使用点云。GNSS 同样要把“有串口/有 Topic”和“有效 RTK 真值”分开显示。

### 6.3 UWB和平台状态

建议另外显示：

- 1—2、1—3、2—3 三条无向边是否新鲜、距离、更新率；
- 双向测距差、一致性拒绝和三角形失败的时间窗计数；
- 三盒时间同步状态、DDS 链路状态和盒端设备健康状态。

其中算法有效率、拒绝原因和边状态应由浙大状态输出提供；`NodeTimeSync`、`LinkState`、`NodeHealth` 属于上交平台状态，应由上交侧提供。GCS 需要统一显示，但不能靠解析两边终端字符串拼出状态。

## 7. 建议补充的结构化状态接口

当前正式 ROS 位置消息只有位置、航向和有效位，足以开始最小显示联调，但不足以解释“为什么无效”以及“哪些传感器正在参与”。建议双方再冻结一个独立状态 Topic，暂定：

`/cooperative_localization/status`

最小字段建议包括：

- `header`、算法模式、参考节点、`frame_id`；
- 总体状态和原因位 `reason_mask`；
- 最后有效位置年龄、输出频率、时间窗有效率；
- 缺边、一致性拒绝、三角形失败计数；
- 各传感器的 `configured/receiving/fresh/used_by_fusion`、频率和数据年龄；
- 三条 UWB 边的新鲜度和状态。

这只是需要双方冻结的字段集合，当前尚未形成正式 ROS 消息，也尚未部署。第一阶段 GCS 可以先依据 `poses_2d` 完成位置和有效性显示，不能因此把状态接口标为已经完成。

## 8. 当前接入进度

| 项目 | 当前状态 | 结论 |
|---|---|---|
| 201 盒在线融合节点 | 已部署并运行 | 已具备 |
| `/cooperative_localization/poses_2d` | 实盒已收到三车结果 | 已具备 |
| Topic 类型和 QoS | 已实现 | 可开始 GCS 适配 |
| UWB-only 坐标定义 | 已写入消息注释和文档 | 可开始 GCS 适配 |
| 有效位 | 已有 `position_valid/yaw_valid` | 可开始 GCS 适配 |
| 约 60 秒在线录包 | 563 帧输出，527 帧有效，有效率约 93.61% | 最小链路已验证 |
| 无效/超时显示规则 | 本文给出首版规则 | 待上交 GCS 实现并联调 |
| 浙大协同定位监视界面 | 已实现直接订阅、有效位和超时显示；本机 ROS 2 端到端消息测试通过 | 待在201实盒及Windows浏览器联调 |
| 独立状态/诊断 Topic | 尚未冻结和实现 | 正式验收前补齐 |
| 上交正式 GCS 端到端订阅 | 尚未完成 | 现在应开始 |
| 单盒移动动态试验 | 尚未完成 | 正式验收前完成 |
| GNSS/RTK 真值 | 核心接口已有，盒端 Topic 和 RTK 状态未验收 | 可选，不阻塞相对显示 |
| IMU、雷达、相机融合 | 尚未完成实盒主链验收 | 不应在 GCS 标为正在使用 |

当前最准确的表述是：**已经具备 GCS 最小相对位置接口联调时机，但还不具备生产级状态监控和精度验收条件。**

## 9. 双方下一步

### 9.1 现在就可以做

1. 上交 GCS 安装或引入与 201 盒完全一致的 `cooperative_localization_msgs` 定义。
2. 按 Reliable、Volatile、Depth 1 订阅 `/cooperative_localization/poses_2d`。
3. 正确显示 `coop_ref_1_relative` 相对坐标和 `position_valid/yaw_valid`。
4. 实现第 3 节的有效、保持、丢失和离线状态，不让旧值进入控制。
5. 在没有 GNSS 的情况下完成一次端到端显示联调，证明 GCS 不依赖绝对位置才能工作。

### 9.2 正式验收前再完成

1. 双方冻结状态 Topic、错误原因、频率、时间和 QoS 契约。
2. 进行只移动一个盒子的动态试验，统计动态延迟、有效率和恢复时间。
3. 分别验证短时异常测距、缺边、融合节点退出、DDS 断链和恢复。
4. 有可靠 RTK 时，再接入独立真值图层并评价误差；没有可靠 RTK 时只做连续性和几何自洽评价。
5. 验证 IMU、雷达或相机真正接入后，GCS 的 `used_by_fusion` 状态会随算法模式正确变化。

## 10. GCS首版验收清单

- [ ] 正常有效帧能显示三车相对队形；
- [ ] `yaw_valid=false` 时不画虚假车头方向；
- [ ] 收到 `position_valid=false` 后立即退出有效状态；
- [ ] 短时异常只显示带年龄的最后有效位置，不供控制使用；
- [ ] 超过门限后标记 `LOST/OFFLINE`，不会永久冻结在最后一点；
- [ ] 恢复有效帧后能自动恢复显示；
- [ ] 没有 GNSS 时相对位置页面仍正常工作；
- [ ] GNSS 失锁或受干扰时只关闭真值层，不关闭 UWB 相对位置层；
- [ ] 能区分传感器“在线”“数据有效”“当前参与融合”；
- [ ] GCS 不解析终端 WARN 作为运行状态来源。

## 11. 浙大协同定位监视界面

上交正式 GCS 尚未联调时，可使用 [relative_gcs.py](../../tools/relative_gcs.py) 在浏览器查看真实 `/cooperative_localization/poses_2d`。这是**独立的最小监视页面**，不改浙大算法输出消息，不给车辆发送控制指令，也不声称已经接入 GNSS 真值、雷达、视觉或结构化传感器健康状态。界面测试见 [test_relative_gcs.py](../../tools/test_relative_gcs.py)。

### 11.1 先在开发电脑检查界面

在工程根目录执行 `python3 tools/relative_gcs.py --demo`，浏览器打开 `http://127.0.0.1:8765`。页面会明确标注“演示数据，非实车”。运行 `python3 tools/test_relative_gcs.py` 可检查无效位、短时保持、丢失、断流和坐标系切换逻辑。

### 11.2 在201盒订阅真实结果

先按文档21第7节启动201融合节点，并确认 `/cooperative_localization/poses_2d` 有发布者。在开发电脑工程根目录上传这一个脚本：

```bash
scp tools/relative_gcs.py firefly@192.168.1.201:/home/firefly/relative_gcs.py
```

在201另开SSH终端执行；它只订阅输出，**不会再启动第二份融合节点**：

```bash
source /opt/ros/humble/setup.bash
source /home/firefly/imu_ws/install/setup.bash
source /home/firefly/zju_coop_ws/install/ros2-box/setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
python3 /home/firefly/relative_gcs.py --bind 0.0.0.0 --port 8765
```

同一可信局域网内，Windows Edge 打开 `http://192.168.1.201:8765`。页面 HTTP 无认证，不要把8765端口暴露到公网；不需要远程浏览器时可省略 `--bind 0.0.0.0`，默认只允许本机访问。结束时只在此 GCS 终端按 `Ctrl+C`。首次实盒端到端浏览器验收尚未完成，不能把上述部署步骤写成已在201复验。

### 11.3 界面语义和参考项目

- 只画相对 XY：`coop_ref_1_relative` 中1号车是原点、1→2为正X、3号车取正Y侧；它不是地理地图。将来若收到 `_enu` 帧，页面只更新坐标系说明并清掉旧坐标，避免跨坐标系拼接。
- `position_valid=false` 或断流后，最近有效点最多以黄色 `HOLD` 显示1秒；之后不再绘制旧点。300毫秒、1秒、2秒分别是首版 `VALID/HOLD/LOST/OFFLINE` 门限，按本 GCS **单调接收时钟**算，不拿UWB模拟Header与真实UTC相减。
- `yaw_valid=false` 不画车头；消息 Header 显示原始纳秒值，不声称是真实UTC。GNSS真值和传感器健康尚无冻结的在线接口，页面明确标注“未接入”，不使用终端WARN冒充状态。
- [QGroundControl Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html) 的多车主视图与状态区、[Mission Planner Flight Data](https://ardupilot.org/planner/docs/mission-planner-flight-data.html) 的主视图/消息分区和 [Foxglove State Transitions](https://docs.foxglove.dev/docs/visualization/panels/state-transitions) 的失效时间线值得借鉴；地图导航、飞控按钮及尚无数据来源的状态不照搬。
- 并列工程 `205_nav_ui` 的网格态势图、多平台状态、回放组织有参考价值，但它当前按 `/swarm/{platform_id}/nav/pose` 的 `PoseStamped/Odometry`、ENU语义接入，且依赖 PySide6；不能直接读取本工程一条三车数组消息，直接复用还可能把相对坐标误认成ENU。此阶段只参考界面组织，不把整套原型并入本工程。

现有 `CooperativePose2DArray` 由浙大融合节点发布；消息定义位于 `ros2/cooperative_localization_msgs/msg/`，201盒按文档41已构建安装该包。上交 GCS 若直接以 ROS 2 订阅，就需要在其运行环境安装**同一版本**的消息包，并对齐 Domain、RMW、网络及 QoS；若采用其他通信协议，则另做适配，不修改现有算法输出。Windows 浏览器访问本界面的 HTTP 页面，不需要安装 ROS 2 消息包。

开源系统并不只输出位置：[ROS 2 的 Odometry](https://docs.ros.org/en/ros2_packages/humble/api/nav_msgs/index.html) 同时承载位姿和速度估计；[MAVLink](https://mavlink.io/en/messages/common.html) 的 `LOCAL_POSITION_NED` 有位置和速度，`ATTITUDE` 有滚转、俯仰、航向及角速度；[QGroundControl Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html) 还能按收到的遥测显示速度、姿态和航向。本工程现有对外消息只有相对 `x_m/y_m`、`yaw_rad` 和有效位，没有速度或完整三维姿态；UWB-only 时 `yaw_valid=false`，界面不得把零值当成实测车头。后续若算法产生可靠速度/姿态，再增加相应状态输出，不伪造当前没有的数据。

上交正式 GCS 仍需与我们对齐自定义消息、QoS、失效处理和未来状态接口；本界面可独立检查在线二维相对输出，但不能替代正式端到端验收。

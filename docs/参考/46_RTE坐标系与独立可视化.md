# RTE坐标系与独立可视化

日期：2026年10月23日。本文补充坐标适配与展示，不更换默认算法、默认坐标配置、原ROS消息定义或既有启动文件。

## 1. 先区分本轮的三种“仿真”

### 有真值的数值仿真：2厘米噪声的5组试验

这5组是程序生成轨迹、测距和真值，调用真实RTE库进行计算的数值仿真，不使用Gazebo，不接物理盒子。它们使用同一运动轨迹、不同随机种子0～4；不是五种独立运动工况。

- 每轮40秒，前12秒静止，之后开始运动；26～28秒中断测距，再恢复。
- 理想里程计按50赫兹直接输入各车真实的局部位置、速度和姿态，绕过IMU积分误差。
- 三条UWB边按10赫兹生成，在真实距离上独立叠加零均值、标准差0.02米的高斯噪声。2厘米是噪声标准差，不是误差上限，也不是定位精度。
- 位置RMSE使用输出为READY的对应历元，与独立保存的轨迹真值比较；不可用时段另计，不当作零误差。错误初始化但输出READY的帧仍计入误差，不删掉失败种子。

| 随机种子 | 相对位置RMSE / 米 |
|---|---:|
| 0 | 0.125839 |
| 1 | 0.097452 |
| 2 | 0.210094 |
| 3 | 0.112937 |
| 4 | 6.984906 |

它证明当前RTE库在有利输入下可以得到较小误差，也揭示错误初始化风险；不能据此宣称实盒达到厘米级。生成代码在[tools/evaluate_rte.py](../../tools/evaluate_rte.py)的synthetic函数；[精度汇总](../reports/20261023_协同定位阶段报告/RTE仿真精度汇总.csv)保留全部18组，包含IMU链路和更大测距噪声的结果。

### Gazebo物理仿真：本轮未通过的是哪几项

| 复测部分 | 直接证据 | 可以得出的结论 |
|---|---|---|
| 场景、模型、插件等资源检查 | test_gazebo_assets通过，约0.68秒 | 静态资源检查通过，不等于端到端运行通过 |
| 默认物理仿真端到端测试 | test_physics_imu_uwb_and_gcs_pipeline在第241行失败：完整有效三车位姿0帧，要求至少60帧；CTest约60秒超时 | 当前启动与数据链路未形成验收要求的有效输出 |
| 默认运行的录包完整性 | test_recording_contains_all_pipeline_topics失败：缺少clock、三路IMU、UWB、NodeState、真值、定位输出这8个预期话题的录制记录 | 本轮默认运行没有形成可用的全链路录包 |
| 默认启动/退出异常 | 录包器等待/clock，启动中的ign服务调用超时；Gazebo收到SIGINT后未及时退出，被升级为SIGTERM | 有启动/时钟及退出问题，不能据此单独确定所有故障的根因 |
| 诊断性启动：临时强制运行物理仿真 | gazebo_run_diagnostic.log中出现Sim time /clock found，并订阅全部8个预期话题；但完整有效三车位姿仍0帧，同一断言失败且CTest超时 | 时钟推进并不足以使定位链路通过；订阅成功也不等于全部消息质量和输出通过 |

日志中的EGL提示是环境线索，尚未证明是唯一根因。该端到端测试运行的是既有IMU/UWB融合链路，不是前述独立RTE数值试验。它首先在完整输出帧数检查处失败，后续位置精度、惯导改善、运动响应、失效恢复等断言没有得到本轮通过结论；不能把这些未执行的项描述成已逐项验证失败。

从项目根目录查看直接证据：

```bash
rg -n 'Passed|Timeout|complete_poses|AssertionError|waiting for /clock|Service call timed out|SIGTERM' \
  docs/reports/20261023_协同定位阶段报告/gazebo_regression.log
rg -n 'Sim time|Subscribed|complete_poses|AssertionError|Timeout' \
  docs/reports/20261023_协同定位阶段报告/gazebo_run_diagnostic.log
```

默认复测入口如下；这不是修复命令，也不承诺本次执行会通过。使用独立ROS域，避免测试节点与实盒互相发现：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-sim-check-current/setup.bash
export ROS_DOMAIN_ID=195 ROS_LOCALHOST_ONLY=1
ctest --test-dir build/ros2-sim-check-current/zju_coop_gazebo \
  --output-on-failure -R 'test_gazebo_assets|test_test_three_vehicle_gazebo.launch.py'
```

本次新增坐标工具不修复Gazebo，不更改其场景、控制逻辑或原融合程序。

### 本文网页：坐标几何示意

网页的演示模式只计算已知位置在不同坐标系中的表达，不运行RTE，不属于精度实验，也不读取实测数据。ROS模式才显示已接入的RTE输出；两种模式有显著标记，断流不会自动切回示意数据。

## 2. 保留旧坐标，新增两种RTE表达

默认物理盒201对应逻辑编号1，202对应2，203对应3。以下也支持通过--ids显式修改三个逻辑编号；参考车始终是第一个。

| 坐标系 | 原点 | X/Y轴 | 输出位置 | 保持情况 |
|---|---|---|---|---|
| coop_ref_1_relative | 201同历元当前位置 | 每帧201→202为+X，203取+Y镜像分支 | 旧UWB相对三角形 | 旧程序、配置、话题完全保留 |
| coop_rte_1_local | 201同历元当前位置 | 平行201局部里程计系L1，不是北向 | 既有RTE原始相对结果 | 原RTE程序和话题保留 |
| coop_rte_1_relative_fixed | 201同历元当前位置 | 首次有效历元201→202为+X；之后固定，+Y为平面内逆时针90度 | 新适配器的相对结果 | 独立新话题，显式启动才存在 |
| coop_rte_1_task | 首次有效历元的201位置 | 与新相对系平行，之后固定 | 三车在本次任务中的位置 | 网页和JSON提供，不放入“相对参考车”ROS消息 |

所有二维系采用右手约定，Z向上。新系不强制203在正Y：真实几何在另一侧时允许负Y，避免人为镜像翻转。航向是车辆前向相对所在坐标系+X的逆时针角，单位弧度，范围[-π,π]；它不是参考车到目标车的方位角。只在yaw_valid为true时可使用。

这里“固定”指在本次局部里程计会话中轴向定义固定，不等于具备无漂移的地理固定基准。RTE变换更新可能改变估计值；适配器只换坐标，不平滑、不滤波，也不修复错误初始化。

### 精确定义与计算

L1是参考车局部里程计系。输入RTE相对位置记为rᵢ(t)，参考车在L1中的位置记为p₁(t)。只在三车position_valid均为true、编号及原坐标匹配、参考车同Header位置可获得时锁定首次有效历元t₀。

令β=atan2(r₂ᵧ(t₀), r₂ₓ(t₀))，R(α)为二维逆时针旋转矩阵。新输出为：

```text
新相对位置：  rᵢ_fixed(t) = R(-β) rᵢ(t)
任务位置：    pᵢ_task(t) = R(-β) [p₁(t) - p₁(t₀) + rᵢ(t)]
新航向：      ψᵢ_new(t)  = wrap[ψᵢ_L1(t) - β]
```

β和p₁(t₀)只锁定一次。201在新相对系中的位置始终是零，但其航向不必为零；在任务系中201可以移动。固定任务原点选在适配器首次满足条件的历元，不冒称整个车辆启动时刻。

例如初始L1中201→202为(0,5)米，β=90度，转换后202为(5,0)。此后201沿L1正Y移动2米且相对队形不变：新相对系中202仍为(5,0)，任务系中201为(2,0)、202为(7,0)。两车距离保持5米。

## 3. 先看网页，不连接实盒

从项目根目录运行，只使用Python标准库，不需要安装Qt、前端框架或ROS：

```bash
python3 tools/rte_coordinate_viewer.py --demo
```

浏览器打开 **http://127.0.0.1:8766**。也可以直接用浏览器打开[HTML文件](../visualization/rte_coordinates.html)，无需启动任何服务；直接打开文件时只有几何演示。

页面分为“图形视图”“坐标数据”“使用说明”三个选项卡。图形视图提供播放、暂停和时间条，每张图右上角的“单图放大”可切换为大图，点击“恢复三图”返回对照布局；坐标数值和有效性在坐标数据页查看，坐标定义与接入说明在使用说明页查看。

页面提供五种情景：

1. 201、203不动，202绕201运动：看203，左图坐标变化，中/右图不变。
2. 三车整体平移并转向：中图201仍在原点，右图显示整个编队移动。
3. 大运动范围三车运动：观察整体轨迹与参考车相对坐标的变化。
4. 只有201原地转向：中图201位置仍为零，但航向箭头改变，坐标轴不旋转。
5. 203穿过201—202连线：新坐标Y可变负，旧规范三角形始终取正Y。

左图仅用同一组输入重现旧坐标的几何规范化，**不是旧UWB节点本次实测输出，也不是两套算法精度对比**。旧UWB节点的真实输出仍使用原有监视工具。

绘图区按当前车辆位置和轨迹自动调整显示范围与刻度间距。报告中的图形页截图采用1600×1000浏览器窗口，选择第一种情景、暂停并将时间调至12秒；页面显示区域截图位于`data/rte_coordinate_validation_20261028/坐标对照_桌面.png`。单图、数据页、说明页及手机截图保存在同一目录。

浏览器检查可执行：

```bash
PYTHONPATH=/tmp/rte_viewer_browser_deps python3 tools/check_rte_coordinate_page.py
```

检查工具需要Playwright及其Chromium浏览器；页面直接打开和`--demo`服务运行仍仅使用浏览器与Python标准库。

## 4. 接入已有RTE，发布新相对坐标

先按[45文档](45_三车RTE初始化与IMU航位推算链路说明.md)准备并运行独立RTE链路。另开一个终端，从项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-rte-three/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
python3 tools/rte_coordinate_viewer.py
```

实际域号必须与RTE所在域一致。若要接内置/rte_demo合成发布器，在对应演示域运行：

```bash
python3 tools/rte_coordinate_viewer.py \
  --poses-topic /rte_demo/poses_2d \
  --states-topic /rte_demo/node_state
```

这只改变订阅入口，不会使现有RTE自动通过初始化或精度门槛。静止实测数据未得到有效RTE时，网页应显示“等待/不可用”，不能用假坐标补齐。

| 项目 | 名称及语义 |
|---|---|
| 原始位姿输入 | /cooperative_localization/rte/poses_2d；CooperativePose2DArray；Reliable/Volatile/depth10 |
| 参考里程计输入 | /cooperative_localization/rte/node_state；NodeState；BestEffort/Volatile/depth5；只使用参考车，frame必须为rte_odom_1 |
| 新相对输出 | /cooperative_localization/rte/coordinates/poses_2d；CooperativePose2DArray；Reliable/Volatile/depth10；20赫兹发布当前状态，不代表20赫兹有新解 |
| 新相对frame_id | coop_rte_1_relative_fixed；位置仍相对201，沿用原消息“参考车在原点”的契约 |
| 任务输出 | http://127.0.0.1:8766/api/state中task数组及task_frame；携带valid、anchor、stamp_ns字符串等信息；不是新ROS绝对坐标话题 |

查看新结果：

```bash
ros2 topic echo --once /cooperative_localization/rte/coordinates/poses_2d
curl -s http://127.0.0.1:8766/api/state | python3 -m json.tool
```

判断顺序：先检查valid/position_valid，再看frame_id和测量Header是否持续推进，最后看x/y与有效航向。数值为0不代表有效。保留原始Header；定时重复发布不能冒充新测量。任务JSON中的纳秒时间戳用字符串表示，避免JavaScript大整数精度丢失。

任务坐标没有使用CooperativePose2DArray发布，是因为该消息原本明确要求每车位置为“本车减参考车”，不能让固定任务坐标暗中改变旧接口的含义。如合作模块后续需要正式任务系ROS输出，应另行冻结标准里程计及状态接口；本工具未自动发布TF。

停止本适配器即可停止新话题与网页，旧UWB和原RTE运行保持原样。新输出命名空间被限定在/cooperative_localization/rte/coordinates/，禁止覆盖旧UWB或原RTE输出。

## 5. 时间、有效性与会话限制

- 只用原始Header配对。参考位置要求同Header，或两侧包围且样本间隔不超过100毫秒的线性插值；不外推，不用bag接收时间或本机墙钟配对。
- 初始201→202基线至少0.2米，避免接近重合时轴向不可定义。这是适配器的几何门槛，不是硬件标定参数或精度指标。
- 上游位置无效、新输入不合法或参考位置无法匹配时，立即撤销新坐标有效性；连续0.5秒没有新有效Header时也撤销有效位。用本机单调时钟判定经过时间，不把它当同步证据。
- 原RTE的有效位沿用上游算法判断，不代表已通过本项目的动态精度验收；READY可能仍有错误解。适配器不生成额外“精度合格”标志。
- Header回退后锁定不可用。任一车局部原点、姿态初值或里程计会话重置时，必须协调重启RTE及此适配器；当前消息没有session_id，无法可靠自动识别所有保持Header递增的静默复位。
- 停止后重新启动，会重新锁定t₀和轴向；不能把两个启动会话的任务轨迹直接拼接。地图坐标或北向仍需外部变换及验证。

## 6. 检查与复现

纯数学及HTTP回归：

```bash
python3 -m unittest discover -s tools -p test_rte_coordinate_viewer.py -v
```

加载消息包后，在仅本机可见的独立ROS域验证真实消息发布/订阅、Header保持、断流撤销有效位，并检查旧话题没有新增发布者：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-rte-three/setup.bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 RTE_COORDINATE_ROS_TEST=1 \
  python3 -m unittest discover -s tools -p test_rte_coordinate_viewer.py -v
```

测试使用人为构造的坐标消息，验证坐标适配正确性，不等同于RTE精度验收。浏览器验证覆盖四个情景、桌面与手机宽度、直接打开HTML和只读HTTP入口。本轮日志、浏览器检查结果与截图保存在[data/rte_coordinate_validation_20261023](../../data/rte_coordinate_validation_20261023/)。

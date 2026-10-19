# GCS离线验证步骤

要同时检查采集完整性、各传感器时间和算法结果，先按[31_离线数据验收与算法回放步骤](31_离线数据验收与算法回放步骤.md)执行；本页只补充GCS显示检查。

没有互联网但能通过局域网连接三盒时，仍可按[21第7节](21_上交盒端传感器节点启动与可用性检查_精简执行版.md#7-201在线uwb协同定位)实测。算法和GCS都不需要互联网。

暂时连不到201时，按下面步骤在**我们自己的Ubuntu开发电脑**验证：旧包中的UWB输入 → 算法重新解算 → GCS显示。无需201、202、203在线；所需ROS 2、消息包和算法已经安装在本机。以下命令不在Windows PowerShell执行。

## 1. 🟥 新开终端1：启动算法

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source install/ros2-uwb-online/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch zju_coop_bringup uwb_only.launch.py
```

保持运行。这里用本机已有的安装版本；修改源码不会自动更新这个安装目录。

## 2. 🟥 新开终端2：启动浙大监视界面

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source install/ros2-uwb-online/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 tools/relative_gcs.py
```

在**这台开发电脑的浏览器**打开：<http://127.0.0.1:8765>。网页资源都在本机，不需要外网。回放前显示等待数据是正常的。

如果上交GCS已能在本机运行，可用它代替这个界面；安装、加载同一消息包，订阅 `/cooperative_localization/poses_2d`，通信域同样为0。

## 3. 🟥 新开终端3：回放UWB输入

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source install/ros2-uwb-online/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 bag play data/zju_online_v1_20260808_151205 --topics /uwb/range
```

该包约59秒。`--topics /uwb/range`必须保留：包里还存有旧的位置输出，本次只回放输入，由正在运行的算法生成新结果。不要把全部话题一起回放来验算法。

## 4. 🟥 新开终端4：核对新输出

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source install/ros2-uwb-online/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
timeout 10s ros2 topic echo --once /cooperative_localization/poses_2d
```

在终端3回放期间检查。通过标准：

- 收到 `frame_id: coop_ref_1_relative`、`reference_node_id: 1`，包含车辆1、2、3。
- 能收到三车 `position_valid: true` 的帧；部分异常输入产生无效帧或被拒绝允许存在。
- 有效帧中车1为 `(0,0)`，车2在正X轴，车3在正Y侧；`yaw_valid: false`。
- 网页消息计数持续增加，并显示三车位置。
- 回放结束约2秒后，浙大界面显示 `OFFLINE`，不再把旧位置当作当前有效位置。

输出频率配置为10 Hz，实际频率受完整三边历元数量影响。此流程验证软件解算、消息接口和显示，不验证201当前安装版本、实盒授时或定位绝对精度。

## 5. 停止或重新验证

在终端1、2各按一次 Ctrl+C。需要提前停止回放时，在终端3按 Ctrl+C。

再次回放同一包前，先重启终端1的算法；旧时间戳重新从包头出现，不能直接当作新的实时历元。界面可以保持运行。

这次 `ROS_LOCALHOST_ONLY=1` 用于单机离线验证。恢复三盒实测或让另一台局域网电脑运行GCS时，按21使用 `ROS_LOCALHOST_ONLY=0`，所有参与进程仍统一域0。

如果只有Windows且未安装ROS，只能先检查界面：

```powershell
python .\tools\relative_gcs.py --demo
```

打开 <http://127.0.0.1:8765>。这个演示模式使用假位置，仅验证界面，不能作为算法或ROS接口验收。

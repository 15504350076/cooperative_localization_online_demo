# AirSimAG 高保真三车接入

当前主线使用 AirSimAG；CARLA 适配入口暂时保留，但不作为当前环境依赖。

## 已接入的链路

AirSimAG 的 Unreal 场景负责三辆车的运动和画面；本工程负责把三车位姿转换为 ENU、生成 UWB 测距，再接现有融合节点：

```text
AirSimAG → /airsim/Car_N/odometry → /uwb/range
                                      → /cooperative_localization/poses_2d
```

当前新增文件：

- `ros2/zju_coop_gazebo/config/airsim_three_ugv_settings.json`：三车配置模板；
- `ros2/zju_coop_gazebo/launch/airsim_three_car.launch.py`：启动 AirSim 位姿和 UWB 适配；
- `ros2/zju_coop_gazebo/scripts/airsim_odometry_bridge.py`：AirSim RPC→ENU Odometry。

## AirSimAG 端

把配置模板复制为 AirSim 使用的配置文件：

```bash
mkdir --parents "$HOME/Documents/AirSim"
cp ros2/zju_coop_gazebo/config/airsim_three_ugv_settings.json \
  "$HOME/Documents/AirSim/settings.json"
```

在有图形显卡的电脑启动 AirSimAG Unreal 场景并保持运行。推荐直接在本工程终端1执行：

```bash
./tools/start_airsim_three_car.sh
```

该脚本使用本工程三车配置，不覆盖 `$HOME/Documents/AirSim/settings.json`。当前 VMware 电脑只能验证 AirSim RPC 可连接、三车名称和适配节点能启动；三车 PhysX 持续运行在 `-nullrhi` 软件环境下不稳定，不能作为动态三车验收或最终画面展示机。

## ROS 2 端

先构建本工程并加载环境：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-airsim-check/setup.bash
```

终端1保持 Unreal 图形窗口运行；另开终端2启动 AirSim 位姿和 UWB 适配：

```bash
ros2 launch zju_coop_gazebo airsim_three_car.launch.py \
  airsim_python_path:=/home/pdq/Code/AirSimAG/PythonClient
```

启动适配器时会把三车放到 ENU `(0,0)`、`(12,0)`、`(0,12)` 米；需要改编队时传入 `initial_positions_enu:=x,y;x,y;x,y`。

该命令要求 AirSim Python 客户端目录存在，且 AirSim RPC 服务可访问。远程仿真机时追加：

```bash
host:=<仿真机IP>
```

## Windows 宿主机显示、Ubuntu 虚拟机运行 ROS

当前电脑的 RTX 4060 在 Windows 宿主机可用，但 Ubuntu 虚拟机只识别到 VMware SVGA/llvmpipe。因此不要在虚拟机里运行 Unreal 图形窗口，按下面方式分工：

### 1. Windows 宿主机（一次性）

1. 安装 UE4.27、Visual Studio 2019（Desktop development with C++、Windows 10 SDK）。
2. 在 AirSimAG 的 Developer Command Prompt 中执行：

```powershell
cd D:\Code\AirSimAG
build.cmd
Unreal\Environments\Blocks\update_from_git.bat
```

3. 打开 `Unreal\Environments\Blocks\Blocks.sln`，选择 `Development Editor`、`Win64`，将 Blocks 设为启动项目，按 F5 编译并启动 Unreal Editor，点击 Play。
4. 把本工程的 `airsim_three_ugv_settings.json` 复制到 Windows：
   `%USERPROFILE%\Documents\AirSim\settings.json`
   并在 JSON 顶层加入 Windows 用于连接虚拟机的 VMnet 网卡地址（从 `ipconfig` 查看）：

```json
"LocalHostIp": "<WINDOWS_VMNET_IP>",
"ApiServerPortCar": 41461,
```

不要填 `127.0.0.1`，它只适用于同一台机器内连接。

5. 以管理员 PowerShell 放行 RPC 端口：

```powershell
New-NetFirewallRule -DisplayName AirSim-RPC-41461 -Direction Inbound -Protocol TCP -LocalPort 41461 -Action Allow
```

### 2. Ubuntu 虚拟机（每次启动）

先确认能访问 Windows：

```bash
ping --count 3 <WINDOWS_VMNET_IP>
```

然后启动 ROS 适配器：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-airsim-check/setup.bash
ros2 launch zju_coop_gazebo airsim_three_car.launch.py \
  host:=<WINDOWS_VMNET_IP> \
  port:=41461 \
  airsim_python_path:=/home/pdq/Code/AirSimAG/PythonClient
```

再检查数据：

```bash
ros2 topic echo --once /airsim/Car_1/odometry
ros2 topic echo --once /uwb/range
```

Windows Unreal 窗口显示三车，Ubuntu 终端应能看到 `/airsim/Car_N/odometry` 和 `/uwb/range`；随后按下面的通用 ROS 2 融合步骤检查定位输出。若只需检查 RPC，不开画面时才使用虚拟机的 `./tools/start_airsim_three_car.sh --headless`。

另开终端启动现有三车 UWB 融合节点，再检查：

```bash
ros2 topic list -t | grep --extended-regexp 'airsim|uwb|cooperative_localization'
ros2 topic echo --once /uwb/range
ros2 topic echo --once /cooperative_localization/poses_2d
```

## 黑窗口或连接失败时

如果适配节点提示 `ECONNREFUSED`，说明 Unreal 尚未启动成功；先关闭残留 ROS 启动终端，再重新按“终端1→终端2”顺序启动。若 AirSim 窗口显示“UE4Editor 无响应”，查看：

```bash
tail -50 /home/pdq/Code/AirSimAG/Unreal/Environments/Blocks/Saved/Logs/Blocks.log
```

日志若出现 `libvulkan_lvp.so`、`SIGSEGV`，就是当前电脑的 VMware/软件 Vulkan 渲染崩溃，不是三车或 ROS 配置错误；应换有硬件 GPU 的电脑运行图形场景。当前电脑只做连接冒烟测试：

```bash
./tools/start_airsim_three_car.sh --headless
```

若 Unreal 已退出但 UWB 终端仍出现 `sequence size exceeds remaining buffer`，先在对应 ROS 终端按 `Ctrl+C`；必要时结束本次残留进程后再启动，不要在旧进程上重复叠加。

## 当前边界

- AirSimAG 当前只接入三辆无人车；无人机后续可以沿用同一适配思路。
- UWB 不是 AirSim 原生传感器，而是根据三车真值位姿计算的带噪测距。
- 适配器已将 AirSim 的 NED 位置转换为项目 ENU；不要再重复转换。
- 当前融合输出仍是二维位置，暂不用于无人机高度融合。
- AirSim ROS 2 原生封装依赖 `geographic_msgs`、`mavros_msgs` 等包；本工程先使用轻量 RPC 位姿适配，避免把这些依赖带入算法工程。
- 当前配置使用 `Car_1`、`Car_2`、`Car_3`，三车初始位置已错开；运动控制和避撞策略仍由 AirSim 控制端负责。

CARLA/AWSIM 仍可通过 `carla_uwb_bridge.launch.py` 接入，但只作为备用适配，不影响 AirSimAG 主线。

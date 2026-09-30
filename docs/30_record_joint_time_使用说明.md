# 30 record_joint_time.py 使用说明

适用于当前Python 3.9及以上版本脚本。上交Windows电脑的Python 3.9.7可运行录包控制，无需安装ROS或额外Python库。

## 1. 在哪里运行、做什么

连接关系：你的电脑远程控制上交电脑，上交电脑通过SSH访问201、202、203盒子。

**脚本在上交电脑的PowerShell中执行，不是在盒子的SSH终端里执行。** 默认录制三盒各自的 `/imu/raw`，并在201录制共享 `/uwb/range` 和 `/node/id`，共四个包。201是相对定位解算参考车，因此该默认值便于检查数据到达201及后续端到端延迟；三盒IMU仍分别在本机录制。

脚本只检查已有驱动、启动本轮录包、限时结束录包、压缩和下载。它不启动或停止已有 `navcore`、`gnss_uwb`，不修改盒端程序、传感器配置或系统时钟。远端包保存于 `/home/firefly/time_test/`，下载后不会删除。

一键录包不等于时钟同步；四路启动存在先后，分析使用各消息原始时间戳。

## 2. 首次准备

### 2.1 放置脚本与检查电脑工具

仅需复制最新的 `tools/record_joint_time.py`，保持下面的目录结构：

```text
C:\Users\SXL\Desktop\浙大测试\joint_test\
└─ tools\
   └─ record_joint_time.py
```

在上交电脑PowerShell执行：

```powershell
cd "C:\Users\SXL\Desktop\浙大测试\joint_test"
python --version
ssh -V
Get-Command scp
python .\tools\record_joint_time.py --help
```

只复制代码框中的命令，不要复制 `PS …>`、`>>` 或代码框标记。最新版会自动创建 `data` 文件夹。

### 2.2 确认SSH主机身份

逐盒普通登录，成功后输入 `exit` 回到PowerShell，再连接下一盒：

```powershell
ssh firefly@192.168.1.201
```

```powershell
ssh firefly@192.168.1.202
```

```powershell
ssh firefly@192.168.1.203
```

第一次连接时核对指纹后输入 `yes`，然后输入盒子的登录密码（不会显示输入字符）。出现 `firefly@firefly:~$` 表示进入成功。

若出现 `REMOTE HOST IDENTIFICATION HAS CHANGED`，先通过已确认的MobaXterm会话或让上交在对应盒子运行：

```bash
hostname -I
ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub
```

核对IP和指纹一致后，才在PowerShell更新该IP的旧记录。例如202：

```powershell
ssh-keygen -R 192.168.1.202
ssh firefly@192.168.1.202
```

不要反复删除已核对的记录，也不要关闭主机密钥检查。

### 2.3 配置免密登录（已完成则跳过）

脚本使用 `BatchMode=yes`，不能交互输入盒子密码。MobaXterm保存了密码不代表PowerShell已支持免密登录。

在PowerShell生成电脑密钥，已有则保留：

```powershell
if (!(Test-Path "$env:USERPROFILE\.ssh\id_ed25519")) {
    ssh-keygen -t ed25519
}
```

保存位置按回车使用默认路径；若问是否覆盖已有文件，输入 `n`。专用测试电脑如使用无口令密钥，在两次passphrase提示处分别直接按回车；如果设置口令，需要通过ssh-agent加载，才能让批处理连接使用。此处不是盒子的密码。

将同一份公钥追加到三盒，分别输入各盒登录密码：

```powershell
foreach ($ip in @("192.168.1.201", "192.168.1.202", "192.168.1.203")) {
    Get-Content "$env:USERPROFILE\.ssh\id_ed25519.pub" | ssh "firefly@$ip" "umask 077; mkdir -p ~/.ssh; tr -d '\r' >> ~/.ssh/authorized_keys; chmod 700 ~/.ssh; chmod 600 ~/.ssh/authorized_keys"
}
```

该命令追加SSH授权，不替换已有公钥、不修改密码，MobaXterm仍可照常连接。只传 `.pub` 公钥，私钥保留在电脑。

验证三盒均输出 `SSH_OK`：

```powershell
foreach ($ip in @("192.168.1.201", "192.168.1.202", "192.168.1.203")) {
    Write-Host "Box: $ip"
    ssh -o BatchMode=yes "firefly@$ip" "echo SSH_OK"
}
```

## 3. 每轮先检查驱动

在PowerShell检查：

```powershell
foreach ($ip in @("192.168.1.201", "192.168.1.202", "192.168.1.203")) {
    Write-Host "Box: $ip"
    ssh -o BatchMode=yes "firefly@$ip" "pgrep -af '^/home/firefly/imu_ws/install/(gnss_uwb/lib/gnss_uwb/gnss_uwb|navcore/lib/navcore/navcore)( |$)'"
}
```

每盒应分别有一份 `gnss_uwb` 和 `navcore`。已存在时不重复启动。缺失时按 [21精简执行版](21_上交盒端传感器节点启动与可用性检查_精简执行版.md) 在对应盒子启动：先 `gnss_uwb`，确认有效递增的UWB GNRMC，再 `navcore`。

录包默认使用`ROS_DOMAIN_ID=107`和`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；本机IMU使用`ROS_LOCALHOST_ONLY=1`，共享UWB使用`0`。107只是当前联合测试约定，不是盒号，也不是ROS固定值。脚本不会修改已有驱动进程的环境。

若上交当前驱动使用其他Domain，可在不改脚本的情况下指定。例如实际进程使用默认域0：

```powershell
python .\tools\record_joint_time.py --domain-id 0
```

先按21号文档读取实际进程环境。驱动、查看终端和录包程序的Domain与RMW必须一致；若不一致，脚本即使发现进程存在，也可能完全收不到Topic。

如果专门排查“IMU先启动导致授时失败”，保持有效UWB运行，在确认现有IMU归属并允许停止后，由原终端停止并重启IMU，再进行对照录包。把启动顺序写入备注，不停止来源不明的进程。

## 4. 一键录制

在上交电脑PowerShell执行：

```powershell
cd "C:\Users\SXL\Desktop\浙大测试\joint_test"
python .\tools\record_joint_time.py
```

脚本依次完成：检查三盒SSH和进程 → 启动四路录包 → 等待订阅 → 每路录满指定时长 → 压缩 → 下载并解压。

默认录制180秒，无需输入 `y` 或等待回车。需要留出时间联系现场人员时，直接延长总时长，例如录15分钟：

```powershell
python .\tools\record_joint_time.py --duration 900
```

如果双方另行约定Domain，例如0，同时指定时长：

```powershell
python .\tools\record_joint_time.py --domain-id 0 --duration 900
```

脚本确认全部订阅后会提示录制已经开始。在这段时间内由现场人员只移动一个约定盒子，完成后保持静止；不要求赶固定秒点。采集结束后，在生成的 `实验备注.txt` 中填写：

```text
实际移动盒号、方向、大致开始/结束时间；若未移动则写三盒全程静止。
本轮启动顺序：请填UWB和IMU的实际启动顺序，以及是否重启IMU。
三盒摆放和已知距离：未知则填未知。
异常情况：请填。
```

时长从各路远端录包命令启动时计算；压缩、下载和解压需要额外时间。正常结束时有四条“已收回”和最终数据目录，PowerShell提示符返回。录制期间不要关闭PowerShell窗口。

本地工程更新不会自动更新上交电脑上的副本。请重新复制最新的 `tools/record_joint_time.py`；如果仍看到15、35、55秒依次移动三盒的提示，运行的就是旧版。

“全部就绪”仅表示日志中已看到所有指定话题的订阅信息，不保证持续出数、不保证UWB三边齐全，也不保证同步。若40秒内未就绪，脚本会等待有限时录制结束并保存日志供排查。

## 5. 输出在哪里、传哪些文件

输出根目录由脚本所在位置确定：`tools` 的上一级目录下的 `data`，不是任意当前工作目录。

默认示例：

```text
joint_test\data\joint_time_20260907_213212\
├─ manifest.json
├─ 实验备注.txt
├─ 采集状态.json
├─ joint_time_20260907_213212_imu_rec_v1.log
├─ joint_time_20260907_213212_imu_rec_v1.tar.gz
├─ joint_time_20260907_213212_imu_rec_v1\
├─ …车2、车3的IMU日志、压缩包、解压目录
└─ …uwb_rec_v1的日志、压缩包、解压目录
```

`rec_v1` 是保存共享UWB包的盒子，不表示只录涉及车1的测距。`--boxes` 是参与录本机IMU的盒子列表，也不会过滤共享DDS上其他发布者的UWB消息。实际测距来源看消息 `src_id/target_id`。

将整个实验目录传回开发电脑工程的 `data/` 下。至少保留四个原始压缩包、日志、manifest、备注和采集状态。分析要求有解压后的包目录，目录中应有 `metadata.yaml` 和 `.db3`。

命名时间来自上交电脑，仅用于实验编号；不是传感器共同时间。远端原始包及压缩包均保留，可在 `/home/firefly/time_test/` 手动取回。

## 6. 参数

| 参数               | 默认值                                              | 用途                                                         |
| ------------------ | --------------------------------------------------- | ------------------------------------------------------------ |
| `--boxes`        | `1=192.168.1.201 2=192.168.1.202 3=192.168.1.203` | 盒号与SSH地址，盒号不能重复                                  |
| `--uwb-recorder` | `1`                                               | 共享UWB录包盒，默认201，必须在boxes内                         |
| `--user`         | `firefly`                                         | SSH用户名；远端工作空间路径仍固定为`/home/firefly/imu_ws`  |
| `--run-id`       | `joint_time_电脑日期时间`                         | 新实验编号，只允许字母、数字、下划线、横线，不可复用已有目录 |
| `--duration`     | `180`                                             | 每路录包秒数，至少`max(150, 90+20×盒子数)`                |
| `--analyze`      | 未设置                                              | 仅分析已下载实验目录，不连接盒子                             |

改为202录共享UWB：

```powershell
python .\tools\record_joint_time.py --uwb-recorder 2
```

指定对照实验编号，每轮换一个：

```powershell
python .\tools\record_joint_time.py --run-id joint_static_after_imu_restart_01
```

增加盒子时，例如：

```powershell
python .\tools\record_joint_time.py --boxes 1=192.168.1.201 2=192.168.1.202 3=192.168.1.203 4=192.168.1.204 --uwb-recorder 3 --duration 180
```

仅在第四盒确实存在且已配置好驱动和SSH时使用。N盒产生N个IMU包和1个共享UWB包；电脑仍只需一个控制终端。

## 7. 在开发电脑分析

本文件第9节就是录包后的统一分析入口：`bash tools/analyze_joint_recording.sh data/joint_time_实验编号`。

录包控制与分析环境不同：Windows Python 3.9只负责控制和下载；分析在已安装ROS Humble及 `cooperative_interfaces` 的Linux开发电脑执行。不要在没有ROS的Windows里直接执行脚本最后打印的分析命令。

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/ros2-sjtu-adapter-current/setup.bash
python3 tools/record_joint_time.py --analyze data/joint_time_20260907_213212
```

将实验目录名换成实际名称。生成 `联合时间初查.md` 和 `时间戳统计.json`，包含各流消息数、原始首末时间、零值、回退、重复和区间交集。

报告只统计实际出现的数据流，需结合manifest核对IMU包是否齐全及UWB实际边。首末区间有交集不代表同步已通过；尤其发生时间回退时，首末范围不是连续覆盖证明。也不能将每个包首帧分别归零后当作同步。

当前全静止实验能检查明显时间域差异与连续性，不能测出真实跨盒同步精度。开始三盒联合录包前，先确认三盒`gnss_uwb`的`pps_source`一致；室内无共同GNSS时使用`pps_source: 1`（UWB PPS）。毫秒级验收仍需上交提供共同事件/PPS捕获及时间映射证据。

## 8. 常见问题

| 现象                                             | 处理                                                                                                                     |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------ |
| `WinError 3`，缺少data目录                     | 换最新脚本，含`directory.mkdir(parents=True, exist_ok=False)`；或先执行 `New-Item -ItemType Directory -Force .\data` |
| `REMOTE HOST IDENTIFICATION HAS CHANGED`       | 按2.2核对指纹后，仅更新对应IP记录                                                                                        |
| `Permission denied (publickey,password)`       | 按2.3配置公钥并验证BatchMode输出SSH_OK                                                                                   |
| 把`>>` 识别为命令                              | 只复制代码内容，不复制终端提示符或Markdown围栏                                                                           |
| 普通SSH能进，脚本不能进                          | 普通登录可能使用密码，脚本要求非交互认证                                                                                 |
| SSH/进程检查失败                                 | 先分别检查SSH免密与两个驱动；成功前脚本不会进入正常录包阶段                                                              |
| 40秒未全部就绪                                   | 看每路log，检查驱动DDS环境、话题和进程；不要仅因看到进程就判定有数据                                                     |
| 已订阅但消息少或为零                             | 查看包信息；订阅建立不保证发布数据                                                                                       |
| `FileExistsError`                              | 同一run-id已用过；换新编号，不覆盖旧实验                                                                                 |
| 下载失败                                         | 原包留在对应盒的`/home/firefly/time_test/`，可用MobaXterm的SFTP下载                                                    |
| 缺少`rosbag2_py` 或 `cooperative_interfaces` | 在开发电脑加载正确ROS工作空间后重跑分析，不需要重录                                                                      |

正常流程建议等待结束，不要用Ctrl+C提前中断电脑脚本。当前脚本没有完整的中断清理和断点续传；中断或断线后不能保证自动落盘、压缩、下载完整。重新录制前先核对本轮远端录包进程和文件状态，不使用宽泛的pkill/killall清理。

已有一次Windows Python 3.9.7现场采集成功记录：`joint_time_20260907_213212`，四包均已收回；采集成功不代表数据时间同步或传感器质量合格。

## 9. 录包后的统一分析

录包目录收回开发电脑后，仍在工程根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/ros2-sjtu-adapter-current/setup.bash
bash tools/analyze_joint_recording.sh data/joint_time_实验编号
```

脚本会检查各包的 `metadata.yaml`、消息数、首末时间、零值、回退、重复和时间区间，并在实验目录生成 Markdown、JSON、CSV 和可选 Foxglove 文件。没有 ROS 环境时不要在 Windows 上运行分析命令；只做原始包收集即可。

`record_joint_time.py --analyze` 只适合快速时间初查。需要逐条分析 IMU、UWB、三边几何和时间戳时，使用上面的一键分析入口。

联合分析遵循一个原则：比较消息自身的 `header.stamp`，录包接收时间只用于诊断录包链路，不能单独当作传感器同步精度。Omni-swarm 的经验也一样——协同计算依赖统一时间基准，不能把不同机器的到达时间直接当成测量时间。

## 10. 201 盒 GNSS、IMU 单独与联合包分析

本节处理 201 盒的 GNSS 单独包、IMU 单独包，以及 GNSS+IMU 联合包；只读 `.db3`，不会修改原包。

先在 Linux 开发电脑加载 ROS 2 和包含 `cooperative_interfaces` 的工作空间：

```bash
cd /home/pdq/Code/cooperative_localization_online_demo/cooperative_localization_online_demo
source /opt/ros/humble/setup.bash
source install/real-data-ros-adapter/setup.bash
```

联合包通常包含：

```text
gnss_imu_v1_static_实验编号/
├── gnss/       # /gnss/fix
├── imu/        # /imu/raw
└── record_info.txt
```

联合分析：

```bash
python3 tools/analyze_gnss_imu_bag.py \
  data/gnss_imu_v1_static_实验编号 \
  | tee data/gnss_imu_v1_static_实验编号/分析过程.txt
```

输出 `GNSS与IMU联合时间分析.md` 和 `GNSS与IMU时间对比.csv`。重点字段是：

- `nearest_imu_header_minus_gnss_header_ms`：最近 IMU 测量时间减 GNSS 测量时间；
- `nearest_imu_receive_minus_gnss_receive_ms`：最近 IMU 录包接收时间减 GNSS 录包接收时间。

分别分析单流时使用：

```bash
python3 tools/analyze_gnss_bag.py data/gnss_v1_outdoor_static_实验编号
python3 tools/analyze_imu_bag.py data/gnss_imu_v1_static_实验编号/imu \
  | tee data/gnss_imu_v1_static_实验编号/imu/imu_analysis.txt
```

GNSS 单独分析会在原目录生成 `GNSS户外静止分析.md`、`GNSS数据.csv` 和可选图；GNSS 话题改名时加 `--topic /新的gnss_topic`。判断时先看消息时间是否非零、回退或长间隔，GNSS 状态和位置是否稳定，IMU 是否约 1000 Hz；再看联合时间表。最近 Header 时间长期接近说明可以配对，差值持续增大说明频率偏差，秒级或天级偏移说明时间基准未统一。

GNSS 约 1 Hz、IMU 约 1000 Hz，不能要求两条消息序号相同；脚本按最近时间配对并保留原始时间。`receive - header` 还包含驱动、DDS、录包和调度延迟，不能单独当作硬件同步精度。没有独立真值时，静止 GNSS 结果只能说明重复性，不能称为绝对定位精度。

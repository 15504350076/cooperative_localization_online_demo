# GCS对接速查

提供完整的 cooperative_localization_msgs 消息包（含 CMakeLists.txt、package.xml）。GCS只用两种消息：CooperativePose2DArray 是整组三车结果，vehicles[] 中的 VehiclePose2D 是单车元素。NodeState 是算法内部状态，随包保留，不用订阅；不是三个GCS话题。

```text
Topic: /cooperative_localization/poses_2d
Type:  cooperative_localization_msgs/msg/CooperativePose2DArray
频率:  配置10 Hz，实际有效输出取决于完整三边测距
```

位置：读取 vehicles[] 中的 node_id、x_m、y_m，单位米。按 node_id 匹配车辆，不依赖数组顺序。

当前 frame_id=coop_ref_1_relative、reference_node_id=1：车1为(0,0)，车1→车2的当前连线为+X，车3取+Y镜像分支。坐标轴随连线变化；不是ENU、地图或车体坐标，不表示车头方向。

position_valid=false 时标记位置无效，旧值只能作历史显示；yaw_valid=false 时不画航向。当前纯UWB没有有效航向。

header.stamp 是UWB测量历元，不保证是真实UTC。超时用GCS本地单调接收时钟判断；建议1秒无有效位置提示不可用，1秒无任何消息提示断流。门限可调，恢复后更新显示。

在已部署的201盒（firefly@192.168.1.201）SSH启动，假定ROS 2通信环境已配置好、三盒UWB正常出数；已有算法节点不要重复启动：

```bash
source /home/firefly/zju_coop_ws/install/ros2-box/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch zju_coop_bringup uwb_only.launch.py
```

保持启动终端运行。另开SSH、加载同样环境后可检查：

```bash
ros2 topic echo --once /cooperative_localization/poses_2d
```

GCS侧编译、加载同一消息包，按ROS 2默认订阅QoS接入Topic即可；不需要在GCS运行算法，也不依赖GNSS。首轮确认三车队形显示、无效提示和断流恢复即可。

无外网或暂时连不到201时，按[GCS离线验证步骤](GCS离线验证步骤.md)在开发电脑回放UWB、重新解算并检查显示。

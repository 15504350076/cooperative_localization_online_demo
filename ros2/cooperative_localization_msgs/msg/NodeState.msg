# stamp：本车惯导最后一次接受的 IMU 样本时刻，不是消息发布时间或接收时间。
# frame_id：第一版固定为 common_enu；三车必须使用同一公共 ENU 定义。
std_msgs/Header header

# 车辆唯一编号；必须与 UWB src_id/target_id 使用同一编号。
uint32 node_id

# 公共 ENU 中的绝对位置 [east, north, up]，单位 m；不是相对参考车位置。
float64[3] position_enu_m

# 公共 ENU 中的速度 [east, north, up]，单位 m/s。
float64[3] velocity_enu_mps

# 右手系车体 FLU 到公共 ENU 的单位四元数 [x, y, z, w]。
float64[4] orientation_flu_to_enu_xyzw

# 整条状态是否可用；false 时接收端必须丢弃，不能当作心跳使用。
bool valid

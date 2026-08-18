# 车辆唯一编号。
uint32 node_id

# 相对于参考车的东向位置，单位 m。
float64 x_m

# 相对于参考车的北向位置，单位 m。
float64 y_m

# 车辆自身航向角，单位 rad，范围 [-pi, pi)。
float64 yaw_rad

# x_m、y_m 是否有效。
bool position_valid

# yaw_rad 是否有效。
bool yaw_valid

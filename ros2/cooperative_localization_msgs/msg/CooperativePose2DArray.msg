# stamp：本组三车结果对应的统一算法时刻。
# frame_id：输出坐标系，例如 coop_ref_1_enu。
std_msgs/Header header

# 参考车辆编号。
uint32 reference_node_id

# 同一算法时刻的全部车辆二维位姿。
cooperative_localization_msgs/VehiclePose2D[] vehicles

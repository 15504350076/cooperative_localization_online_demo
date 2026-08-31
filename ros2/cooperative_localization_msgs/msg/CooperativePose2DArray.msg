# stamp：本组三车结果对应的统一算法时刻；实盒沿用 UWB_SYSTEM_TIME，
# 不能用参考车 ROS/SystemTime 发布时间覆盖。
# frame_id：coop_ref_<reference_node_id>_enu。
# 原点位于参考车同历元位置，坐标轴始终与 common_enu 平行，
# 不随参考车车头旋转。
std_msgs/Header header

# 参考车辆编号。
uint32 reference_node_id

# 同一算法时刻的全部车辆二维位姿。
cooperative_localization_msgs/VehiclePose2D[] vehicles

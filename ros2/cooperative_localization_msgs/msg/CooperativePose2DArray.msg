# stamp：本组三车结果对应的统一算法时刻；实盒沿用 UWB_SYSTEM_TIME，
# 不能用参考车 ROS/SystemTime 发布时间覆盖。
# frame_id：IMU/UWB模式为coop_ref_<reference_node_id>_enu，坐标轴与
# common_enu平行；纯UWB模式为coop_ref_<reference_node_id>_relative，
# 参考车为原点、参考车到第二个规范节点为+X，第三节点取+Y镜像分支。
# 后一种坐标系不表示北向、地图方向或任一车辆车头方向。
# GCS位置输出的硬性约定：无论内部使用哪些传感器，每车x/y始终是
# 该车同历元位置减参考车同历元位置，不发布单车绝对坐标。
std_msgs/Header header

# 参考车辆编号。
uint32 reference_node_id

# 同一算法时刻的全部车辆二维位姿。
cooperative_localization_msgs/VehiclePose2D[] vehicles

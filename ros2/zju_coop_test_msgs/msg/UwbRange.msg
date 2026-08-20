# header.stamp：实际测距事件在 UWB_SYSTEM_TIME 中的时刻；当前仅为测试替身，
# 正式消息仍需由双方冻结具体物理事件语义，不能填写结果发布时刻。
std_msgs/Header header

# 源节点编号。
uint32 src_id

# 目标节点编号。
uint32 target_id

# 节点间距离，单位 m。
float32 distance

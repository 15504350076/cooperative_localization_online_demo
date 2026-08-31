# 与上交盒端 cooperative_interfaces/msg/GnssPosition 保持同名、同序、同类型。
std_msgs/Header header

uint32 node_id

int8 STATUS_NO_FIX=-1
int8 STATUS_FIX=0
int8 STATUS_SBAS_FIX=1
int8 STATUS_GBAS_FIX=2
int8 status

# WGS-84 纬度、经度，单位 degree；高度为椭球高，单位 m。
float64 latitude
float64 longitude
float64 altitude

# 局部 ENU 位置协方差，按行主序排列，单位 m^2。
float64[9] position_covariance

uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
uint8 position_covariance_type

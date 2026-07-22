/*
 * 模块职责：声明浙大协同定位算法库的稳定C ABI，是上交ROS 2适配层唯一必须依赖的接口。
 * 设计边界：ROS消息、通信路由和车辆控制不得进入本头文件；Windows DLL与RK3588 .so
 * 通过相同结构布局和版本检查调用。所有时间为统一时间轴纳秒，物理量采用SI单位。
 */
#ifndef ZJU_COOP_C_API_H
#define ZJU_COOP_C_API_H

/* 所有版本化结构必须先调用对应init函数；同一handle的调用由调用方串行化。 */

#include "zju_coop/export.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ZJU_COOP_ABI_VERSION_V1 UINT32_C(0x00010000)

/* ABI级错误只描述调用是否成功；量测是否被融合由各processing result另行返回。 */
typedef int32_t zju_coop_error_code_t;
#define ZJU_COOP_OK ((zju_coop_error_code_t)0)
#define ZJU_COOP_INVALID_ARGUMENT ((zju_coop_error_code_t)1)
#define ZJU_COOP_ABI_MISMATCH ((zju_coop_error_code_t)2)
#define ZJU_COOP_STRUCT_SIZE_MISMATCH ((zju_coop_error_code_t)3)
#define ZJU_COOP_BUFFER_TOO_SMALL ((zju_coop_error_code_t)4)
#define ZJU_COOP_OUT_OF_MEMORY ((zju_coop_error_code_t)5)
#define ZJU_COOP_INTERNAL_ERROR ((zju_coop_error_code_t)6)

/* C ABI固定使用单字节0/1布尔值，禁止直接暴露编译器相关的C++ bool布局。 */
typedef uint8_t zju_coop_bool_t;
#define ZJU_COOP_FALSE ((zju_coop_bool_t)0)
#define ZJU_COOP_TRUE ((zju_coop_bool_t)1)

/* 设备侧测距状态：它描述输入质量，不等同于算法最终是否接受该量测。 */
typedef uint8_t zju_coop_range_status_t;
#define ZJU_COOP_RANGE_STATUS_OK ((zju_coop_range_status_t)0)
#define ZJU_COOP_RANGE_STATUS_DEGRADED ((zju_coop_range_status_t)1)
#define ZJU_COOP_RANGE_STATUS_INVALID ((zju_coop_range_status_t)2)

/*
 * 观测状态描述滑动窗口的长期质量；融合动作描述当前包如何进入滤波器。
 * 两者分离后，恢复期可以保持RECOVERING状态，同时只对试探包执行更新。
 */
typedef int32_t zju_coop_observation_state_t;
#define ZJU_COOP_OBSERVATION_UNKNOWN ((zju_coop_observation_state_t)0)
#define ZJU_COOP_OBSERVATION_NORMAL ((zju_coop_observation_state_t)1)
#define ZJU_COOP_OBSERVATION_DEGRADED ((zju_coop_observation_state_t)2)
#define ZJU_COOP_OBSERVATION_SUSPENDED ((zju_coop_observation_state_t)3)
#define ZJU_COOP_OBSERVATION_REJECTED ((zju_coop_observation_state_t)4)
#define ZJU_COOP_OBSERVATION_RECOVERING ((zju_coop_observation_state_t)5)

typedef int32_t zju_coop_fusion_action_t;
#define ZJU_COOP_FUSION_USE_NORMAL ((zju_coop_fusion_action_t)0)
#define ZJU_COOP_FUSION_USE_DOWNWEIGHTED ((zju_coop_fusion_action_t)1)
#define ZJU_COOP_FUSION_HOLD ((zju_coop_fusion_action_t)2)
#define ZJU_COOP_FUSION_REJECT ((zju_coop_fusion_action_t)3)
#define ZJU_COOP_FUSION_TRIAL_RECOVERY ((zju_coop_fusion_action_t)4)

typedef int32_t zju_coop_localization_state_t;
#define ZJU_COOP_LOCALIZATION_UNINITIALIZED ((zju_coop_localization_state_t)0)
#define ZJU_COOP_LOCALIZATION_NORMAL ((zju_coop_localization_state_t)1)
#define ZJU_COOP_LOCALIZATION_DEGRADED ((zju_coop_localization_state_t)2)
#define ZJU_COOP_LOCALIZATION_UNOBSERVABLE ((zju_coop_localization_state_t)3)
#define ZJU_COOP_LOCALIZATION_STALE ((zju_coop_localization_state_t)4)

/*
 * processing disposition覆盖引擎入口到质量决策的整条处理链；
 * update disposition只描述真正调用滤波量测模型后的数值结果。
 */
typedef int32_t zju_coop_processing_disposition_t;
#define ZJU_COOP_PROCESSING_PROCESSED ((zju_coop_processing_disposition_t)0)
#define ZJU_COOP_PROCESSING_INVALID_PACKET \
  ((zju_coop_processing_disposition_t)1)
#define ZJU_COOP_PROCESSING_OUT_OF_ORDER ((zju_coop_processing_disposition_t)2)
#define ZJU_COOP_PROCESSING_TIME_REJECTED \
  ((zju_coop_processing_disposition_t)3)
#define ZJU_COOP_PROCESSING_DUPLICATE ((zju_coop_processing_disposition_t)4)
#define ZJU_COOP_PROCESSING_HELD ((zju_coop_processing_disposition_t)5)
#define ZJU_COOP_PROCESSING_REJECTED ((zju_coop_processing_disposition_t)6)

typedef int32_t zju_coop_update_disposition_t;
#define ZJU_COOP_UPDATE_ACCEPTED ((zju_coop_update_disposition_t)0)
#define ZJU_COOP_UPDATE_INVALID_PACKET ((zju_coop_update_disposition_t)1)
#define ZJU_COOP_UPDATE_UNKNOWN_NODE ((zju_coop_update_disposition_t)2)
#define ZJU_COOP_UPDATE_SELF_RANGE ((zju_coop_update_disposition_t)3)
#define ZJU_COOP_UPDATE_NON_POSITIVE_RANGE ((zju_coop_update_disposition_t)4)
#define ZJU_COOP_UPDATE_OUT_OF_ORDER ((zju_coop_update_disposition_t)5)
#define ZJU_COOP_UPDATE_NIS_REJECTED ((zju_coop_update_disposition_t)6)
#define ZJU_COOP_UPDATE_NUMERICAL_FAILURE ((zju_coop_update_disposition_t)7)

/* IMU处理结果；数值固定，供上交ROS 2适配层稳定映射。 */
typedef int32_t zju_coop_imu_disposition_t;
#define ZJU_COOP_IMU_BASELINE_ESTABLISHED ((zju_coop_imu_disposition_t)0)
#define ZJU_COOP_IMU_PROPAGATED ((zju_coop_imu_disposition_t)1)
#define ZJU_COOP_IMU_INVALID_PACKET ((zju_coop_imu_disposition_t)2)
#define ZJU_COOP_IMU_UNKNOWN_NODE ((zju_coop_imu_disposition_t)3)
#define ZJU_COOP_IMU_DUPLICATE ((zju_coop_imu_disposition_t)4)
#define ZJU_COOP_IMU_OUT_OF_ORDER ((zju_coop_imu_disposition_t)5)
#define ZJU_COOP_IMU_INTERVAL_REJECTED ((zju_coop_imu_disposition_t)6)
#define ZJU_COOP_IMU_FRAME_MISMATCH ((zju_coop_imu_disposition_t)7)
#define ZJU_COOP_IMU_NUMERICAL_FAILURE ((zju_coop_imu_disposition_t)8)

/* 可按位组合的退化原因；消费者必须逐位判断，不能把组合值当作单一枚举。 */
typedef uint32_t zju_coop_reason_mask_t;
#define ZJU_COOP_REASON_NONE ((zju_coop_reason_mask_t)UINT32_C(0))
#define ZJU_COOP_REASON_NLOS_RATIO_HIGH \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 0U))
#define ZJU_COOP_REASON_VALID_RATIO_LOW \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 1U))
#define ZJU_COOP_REASON_RATE_LOW \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 2U))
#define ZJU_COOP_REASON_RANGE_RESIDUAL_HIGH \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 3U))
#define ZJU_COOP_REASON_TIME_SYNC_TIMEOUT \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 4U))
#define ZJU_COOP_REASON_LINK_TIMEOUT \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 5U))
#define ZJU_COOP_REASON_GRAPH_GEOMETRY_DEGENERATE \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 6U))
#define ZJU_COOP_REASON_NODE_UNREACHABLE \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 7U))
#define ZJU_COOP_REASON_INITIALIZATION_MISSING \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 8U))
#define ZJU_COOP_REASON_INPUT_OVERFLOW \
  ((zju_coop_reason_mask_t)(UINT32_C(1) << 9U))

/* 不透明会话句柄隐藏C++对象布局，调用方只能通过本文件声明的函数访问。 */
typedef struct zju_coop_handle zju_coop_handle_t;

/* 仅测距兼容状态的二维初值；位置单位m，速度单位m/s。 */
typedef struct zju_coop_node_initialization {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t node_id;
  uint32_t reserved0;
  double x;
  double y;
  double vx;
  double vy;
  double position_std_m;
  double velocity_std_mps;
} zju_coop_node_initialization_t;

typedef struct zju_coop_config {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t reference_node_id;
  uint32_t node_count;
  uint32_t node_stride;
  uint32_t reserved_nodes;
  /*
   * nodes 只在 zju_coop_create() 调用期间借用，库会深拷贝。
   * node_stride 是相邻元素的字节间隔，不得小于 v1 节点结构体大小。
   */
  const zju_coop_node_initialization_t* nodes;
  /* 仅测距恒速回退模型参数；默认惯性路线不会用它替代IMU传播。 */
  double process_accel_std_mps2;
  double nis_gate;
  double max_prediction_step_s;
  double min_covariance_diagonal;
  /* 以下阈值共同控制每条无向协同边的滑动质量窗口与恢复状态机。 */
  uint64_t degradation_window_ns;
  double nominal_rate_hz;
  double nlos_ratio_threshold;
  double valid_ratio_threshold;
  double rate_ratio_threshold;
  double nlos_probability_threshold;
  double nlos_covariance_scale;
  uint64_t suspend_duration_ns;
  uint64_t reject_duration_ns;
  uint64_t recovery_duration_ns;
  /* 显式资源上限用于阻止异常配置导致矩阵或边缓存无限分配。 */
  uint32_t max_tracked_edges;
  uint32_t duplicate_cache_per_link;
  uint64_t edge_timeout_ns;
  uint64_t max_future_skew_ns;
  uint64_t max_receive_delay_ns;
  uint32_t max_nodes;
  uint32_t max_edges;
  uint32_t max_state_dimension;
  uint32_t reserved0;
  double rigidity_tolerance;
} zju_coop_config_t;

/*
 * 平台间测距输入；时间为上交统一时间轴纳秒，距离和标准差单位m。
 * sequence应在同一from/to链路上单调递增；timestamp_ns是测量时刻，
 * receive_timestamp_ns是同一时基下的本机接收时刻，仅用于延迟校验。
 */
typedef struct zju_coop_range_packet {
  uint32_t struct_size;
  uint32_t abi_version;
  uint16_t from_node;
  uint16_t to_node;
  uint32_t reserved0;
  uint64_t sequence;
  uint64_t timestamp_ns;
  uint64_t receive_timestamp_ns;
  double range_m;
  double range_std_m;
  float nlos_probability;
  zju_coop_bool_t nlos_flag;
  zju_coop_bool_t has_nlos_probability;
  zju_coop_bool_t valid;
  zju_coop_range_status_t status;
} zju_coop_range_packet_t;

/*
 * 单节点15维惯性状态初值；坐标为ENU，姿态为车体FLU到ENU的xyzw四元数。
 * 五组标准差对应[δp,δv,δθ,δbg,δba]，算法内部再平方为协方差对角元。
 */
typedef struct zju_coop_inertial_node_initialization {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t node_id;
  uint32_t reserved0;
  double position_n_m[3];
  double velocity_n_mps[3];
  double orientation_xyzw[4];
  double gyro_bias_rad_s[3];
  double accel_bias_m_s2[3];
  double position_std_m[3];
  double velocity_std_mps[3];
  double attitude_std_rad[3];
  double gyro_bias_std_rad_s[3];
  double accel_bias_std_m_s2[3];
} zju_coop_inertial_node_initialization_t;

/*
 * 惯性路径配置；nodes仅在configure调用期间借用，库内会深拷贝。
 * 噪声密度和随机游走均采用连续时间单位；message covariance只有在
 * use_message_covariance为真且消息协方差通过对称/半正定检查时才生效。
 */
typedef struct zju_coop_inertial_config {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t node_count;
  uint32_t node_stride;
  const zju_coop_inertial_node_initialization_t* nodes;
  uint32_t max_inertial_state_dimension;
  uint32_t reserved0;
  double gravity_mps2;
  double min_imu_dt_s;
  double max_imu_dt_s;
  double max_propagation_substep_s;
  double gyro_noise_density_rad_s_sqrt_hz;
  double accel_noise_density_m_s2_sqrt_hz;
  double gyro_bias_random_walk_rad_s2_sqrt_hz;
  double accel_bias_random_walk_m_s3_sqrt_hz;
  double min_covariance_diagonal;
  double quaternion_norm_tolerance;
  double covariance_symmetry_tolerance;
  char expected_frame_id[32];
  zju_coop_bool_t use_message_covariance;
  zju_coop_bool_t use_orientation_for_initialization;
  uint8_t reserved1[6];
} zju_coop_inertial_config_t;

/*
 * sensor_msgs/Imu到普通C结构体的无ROS映射；不含温度。
 * orientation_covariance[0]==-1表示姿态协方差不可用，全零表示未知；
 * 角速度和线加速度是采样时刻的瞬时量，适配层不得提前做预积分。
 */
typedef struct zju_coop_imu_packet {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t node_id;
  uint32_t reserved0;
  uint64_t sequence;
  uint64_t timestamp_ns;
  uint64_t receive_timestamp_ns;
  double orientation_xyzw[4];
  double orientation_covariance[9];
  double angular_velocity_rad_s[3];
  double angular_velocity_covariance[9];
  double linear_acceleration_m_s2[3];
  double linear_acceleration_covariance[9];
  char frame_id[32];
  zju_coop_bool_t orientation_valid;
  zju_coop_bool_t valid;
  uint8_t status;
  uint8_t reserved1[5];
} zju_coop_imu_packet_t;

typedef struct zju_coop_imu_processing_result {
  uint32_t struct_size;
  uint32_t abi_version;
  zju_coop_imu_disposition_t disposition;
  zju_coop_bool_t propagated;
  uint8_t reserved0[3];
  double dt_s;
} zju_coop_imu_processing_result_t;

/* 测距处理诊断；区分数据处理结论、融合动作和滤波更新结论。 */
typedef struct zju_coop_range_processing_result {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t from_node;
  uint32_t to_node;
  zju_coop_processing_disposition_t disposition;
  zju_coop_fusion_action_t fusion_action;
  zju_coop_update_disposition_t update_disposition;
  zju_coop_bool_t filter_updated;
  uint8_t reserved0[3];
  double innovation_m;
  double innovation_variance;
  double nis;
  double covariance_scale;
} zju_coop_range_processing_result_t;

/*
 * 主参考节点坐标系下的二维相对定位输出；当前yaw和z均明确标记无效。
 * x/y/vx/vy是“节点减参考节点”的ENU平面相对量，2×2位置协方差按
 * [cov_xx cov_xy; cov_xy cov_yy]解释，不能当作经纬度或地图绝对坐标。
 */
typedef struct zju_coop_localization {
  uint32_t struct_size;
  uint32_t abi_version;
  uint64_t timestamp_ns;
  uint32_t node_id;
  uint32_t reference_node_id;
  double x;
  double y;
  double vx;
  double vy;
  double cov_xx;
  double cov_xy;
  double cov_yy;
  zju_coop_bool_t valid;
  zju_coop_bool_t yaw_valid;
  zju_coop_bool_t z_valid;
  uint8_t reserved0;
  zju_coop_localization_state_t state;
} zju_coop_localization_t;

/* 当前动态协同图的连通性、可观性、活动边数和综合退化原因。 */
typedef struct zju_coop_network {
  uint32_t struct_size;
  uint32_t abi_version;
  uint64_t timestamp_ns;
  uint32_t node_count;
  uint32_t reachable_node_count;
  uint32_t active_edge_count;
  zju_coop_bool_t connected;
  zju_coop_bool_t observable;
  uint8_t reserved0[2];
  zju_coop_reason_mask_t reason_mask;
  zju_coop_localization_state_t state;
} zju_coop_network_t;

/* 单条协同边在质量窗口内的计数、比率、状态和实际融合动作。 */
typedef struct zju_coop_observation {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t from_node;
  uint32_t to_node;
  uint64_t window_start_ns;
  uint64_t window_end_ns;
  uint32_t expected_count;
  uint32_t received_count;
  uint32_t valid_count;
  uint32_t nlos_count;
  uint32_t residual_rejected_count;
  uint32_t dropped_count;
  double nlos_ratio;
  double valid_ratio;
  double actual_rate_hz;
  zju_coop_observation_state_t state;
  zju_coop_fusion_action_t fusion_action;
  zju_coop_reason_mask_t reason_mask;
  zju_coop_bool_t input_overflow;
  uint8_t reserved0[3];
  double covariance_scale;
} zju_coop_observation_t;

/* 版本查询不需要创建handle，可用于上交wrapper启动时的兼容性检查。 */
ZJU_COOP_API uint32_t ZJU_COOP_CALL zju_coop_abi_version(void);
ZJU_COOP_API const char* ZJU_COOP_CALL zju_coop_version_string(void);
ZJU_COOP_API const char* ZJU_COOP_CALL
zju_coop_error_string(zju_coop_error_code_t code);

/* 初始化函数清零保留字段并写入struct_size/abi_version，禁止调用方自行猜测布局。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_node_initialization_init(zju_coop_node_initialization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_config_init(zju_coop_config_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_range_packet_init(zju_coop_range_packet_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_range_processing_result_init(
    zju_coop_range_processing_result_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_localization_init(zju_coop_localization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_network_init(zju_coop_network_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_observation_init(zju_coop_observation_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_inertial_node_initialization_init(
    zju_coop_inertial_node_initialization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_inertial_config_init(zju_coop_inertial_config_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_imu_packet_init(zju_coop_imu_packet_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_imu_processing_result_init(zju_coop_imu_processing_result_t* value);

/*
 * 创建基础实例；成功时*out_handle归调用方并必须destroy一次。
 * 默认融合路线要求在首个IMU或测距输入前调用configure_inertial；
 * 若显式使用仅测距回退配置，则不配置惯性状态。
 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_create(const zju_coop_config_t* config,
                zju_coop_handle_t** out_handle);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_destroy(zju_coop_handle_t* handle);
/* configure成功后节点集合和状态维度被冻结，处理开始后不允许重新配置。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_configure_inertial(zju_coop_handle_t* handle,
                            const zju_coop_inertial_config_t* config);
/* IMU输入为瞬时角速度/比力，不是调用方计算好的预积分量。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_push_imu(zju_coop_handle_t* handle,
                  const zju_coop_imu_packet_t* packet,
                  zju_coop_imu_processing_result_t* result);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_push_range(zju_coop_handle_t* handle,
                    const zju_coop_range_packet_t* packet,
                    zju_coop_range_processing_result_t* result);

/*
 * 输出内存全部归调用方所有，两个stride均为字节步长且不得小于v1结构体大小。
 * 使用NULL/0/0可先查询所需数组数量；查询返回BUFFER_TOO_SMALL且不推进算法时间。
 * 缓冲区或头部错误不会造成部分写入；只有预测和全部转换成功后才整体提交状态。
 * 每个输出元素均须先调用对应init函数。同一handle不是线程安全的，ROS 2适配层
 * 必须用单线程executor、互斥回调组或外部锁保证串行调用。
 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_step(zju_coop_handle_t* handle, uint64_t now_ns,
              zju_coop_localization_t* localizations,
              uint32_t localization_capacity,
              uint32_t localization_stride,
              uint32_t* localization_count,
              zju_coop_observation_t* observations,
              uint32_t observation_capacity,
              uint32_t observation_stride,
              uint32_t* observation_count,
              zju_coop_network_t* network);

#ifdef __cplusplus
}
#endif

#endif

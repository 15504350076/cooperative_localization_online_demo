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
  uint32_t struct_size;  /* 调用方分配的结构字节数，用于兼容尾部扩展。 */
  uint32_t abi_version;  /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t node_id;      /* 该初值所属平台的业务编号，须在节点数组中唯一。 */
  uint32_t reserved0;    /* v1保留占位，调用方必须置零。 */
  double x;              /* 节点在共同导航ENU中的东向初始位置，单位m；内部再减参考节点位置。 */
  double y;              /* 节点在共同导航ENU中的北向初始位置，单位m；内部再减参考节点位置。 */
  double vx;             /* 节点在共同导航ENU中的东向初始速度，单位m/s；内部再减参考节点速度。 */
  double vy;             /* 节点在共同导航ENU中的北向初始速度，单位m/s；内部再减参考节点速度。 */
  double position_std_m; /* x、y共同采用的1σ位置初值标准差，单位m。 */
  double velocity_std_mps; /* vx、vy共同采用的1σ速度初值标准差，单位m/s。 */
} zju_coop_node_initialization_t;

typedef struct zju_coop_config {
  uint32_t struct_size;       /* 调用方分配的配置结构字节数。 */
  uint32_t abi_version;       /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t reference_node_id; /* 二维相对状态中作为原点、且不进入状态向量的平台编号。 */
  uint32_t node_count;        /* nodes数组的有效元素数，包含参考节点。 */
  uint32_t node_stride;       /* nodes相邻元素起始地址的字节间隔。 */
  uint32_t reserved_nodes;    /* v1节点布局保留位，调用方必须置零。 */
  /*
   * nodes 只在 zju_coop_create() 调用期间借用，库会深拷贝。
   * node_stride 是相邻元素的字节间隔，不得小于 v1 节点结构体大小。
   */
  const zju_coop_node_initialization_t* nodes;
  /* 仅测距恒速回退模型参数；默认惯性路线不会用它替代IMU传播。 */
  double process_accel_std_mps2; /* 白噪声加速度1σ，单位m/s²。 */
  double nis_gate;                /* 测距更新的无量纲NIS拒绝门限。 */
  double max_prediction_step_s;   /* 单次恒速预测允许的最大子步长度，单位s。 */
  double min_covariance_diagonal; /* 数值修正后协方差对角元下限。 */
  /* 以下阈值共同控制每条无向协同边的滑动质量窗口与恢复状态机。 */
  uint64_t degradation_window_ns; /* 质量统计滑窗宽度，统一时间轴纳秒。 */
  double nominal_rate_hz;         /* 单条边计算期望包数所用的标称频率，单位Hz。 */
  double nlos_ratio_threshold;    /* NLOS样本占比触发退化的[0,1]门限。 */
  double valid_ratio_threshold;   /* 有效样本占比低于该[0,1]值时触发退化。 */
  double rate_ratio_threshold;    /* 实际/标称频率比低于该值时触发退化。 */
  double nlos_probability_threshold; /* 概率达到该[0,1]值时将单包计为NLOS。 */
  double nlos_covariance_scale;   /* 降权融合时施加到测距方差的无量纲倍率。 */
  uint64_t suspend_duration_ns;   /* 坏证据首次出现到进入Suspended的累计时长门限，单位ns。 */
  uint64_t reject_duration_ns;    /* 持续退化转入拒绝状态前的时长，单位ns。 */
  uint64_t recovery_duration_ns;  /* 试探恢复需持续正常的时长，单位ns。 */
  /* 显式资源上限用于阻止异常配置导致矩阵或边缓存无限分配。 */
  uint32_t max_tracked_edges;       /* 退化监测器可保留的无向边状态上限。 */
  uint32_t duplicate_cache_per_link;/* 每条有向链保存的sequence去重条目上限。 */
  uint64_t edge_timeout_ns;         /* 边最后有效量测后仍视为活动的时长，单位ns。 */
  uint64_t max_future_skew_ns;      /* 测量时刻相对接收时刻允许超前的最大量，单位ns。 */
  uint64_t max_receive_delay_ns;    /* 接收时刻相对测量时刻允许滞后的最大量，单位ns。 */
  uint32_t max_nodes;               /* 运行期节点总数的防御上限。 */
  uint32_t max_edges;               /* 完全图无向边数量的防御上限。 */
  uint32_t max_state_dimension;     /* UWB-only 4×非参考节点状态维数上限。 */
  uint32_t reserved0;               /* v1配置尾部保留位，调用方必须置零。 */
  double rigidity_tolerance;        /* 图可观性秩判定使用的无量纲数值容差。 */
} zju_coop_config_t;

/*
 * 平台间测距输入；时间为上交统一时间轴纳秒，距离和标准差单位m。
 * sequence应在同一from/to链路上单调递增；timestamp_ns是测量时刻，
 * receive_timestamp_ns是同一时基下的本机接收时刻，仅用于延迟校验。
 */
typedef struct zju_coop_range_packet {
  uint32_t struct_size;     /* 调用方分配的测距包结构字节数。 */
  uint32_t abi_version;     /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint16_t from_node;       /* 发起/报告测距的平台编号，定义有向去重链路起点。 */
  uint16_t to_node;         /* 被测平台编号，定义有向去重链路终点。 */
  uint32_t reserved0;       /* v1保留占位，调用方必须置零。 */
  uint64_t sequence;        /* 同一from_node→to_node数据流单调递增的生产端序号。 */
  uint64_t timestamp_ns;    /* 测距发生的传感器统一时间，单位ns，用于滤波更新。 */
  uint64_t receive_timestamp_ns; /* 本机收到该包的同一统一时间，单位ns，仅用于时延校验。 */
  double range_m;           /* 两节点之间的直接距离量测，单位m。 */
  double range_std_m;       /* 量测1σ标准差，单位m，必须为正有限值。 */
  float nlos_probability;   /* NLOS概率[0,1]，仅has_nlos_probability为真时有效。 */
  zju_coop_bool_t nlos_flag;/* 设备或上游给出的NLOS硬判决。 */
  zju_coop_bool_t has_nlos_probability; /* 是否携带可信nlos_probability。 */
  zju_coop_bool_t valid;    /* 上游是否认可整条测距输入可供算法处理。 */
  zju_coop_range_status_t status; /* 设备侧OK/DEGRADED/INVALID质量状态。 */
} zju_coop_range_packet_t;

/*
 * 单节点15维惯性状态初值；坐标为ENU，姿态为车体FLU到ENU的xyzw四元数。
 * 五组标准差对应[δp,δv,δθ,δbg,δba]，算法内部再平方为协方差对角元。
 */
typedef struct zju_coop_inertial_node_initialization {
  uint32_t struct_size; /* 调用方分配的惯性节点初值结构字节数。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t node_id;     /* 初值所属平台编号，须与基础节点集合一一对应。 */
  uint32_t reserved0;   /* v1保留占位，调用方必须置零。 */
  double position_n_m[3];       /* 导航ENU坐标系[east,north,up]初始位置，单位m。 */
  double velocity_n_mps[3];     /* 导航ENU坐标系[east,north,up]初始速度，单位m/s。 */
  double orientation_xyzw[4];   /* 车体FLU到导航ENU的[x,y,z,w]单位四元数。 */
  double gyro_bias_rad_s[3];    /* 车体FLU三轴陀螺零偏初值，单位rad/s。 */
  double accel_bias_m_s2[3];    /* 车体FLU三轴加速度计零偏初值，单位m/s²。 */
  double position_std_m[3];     /* ENU三轴位置误差1σ，单位m。 */
  double velocity_std_mps[3];   /* ENU三轴速度误差1σ，单位m/s。 */
  double attitude_std_rad[3];   /* 15维误差状态δθ三轴1σ，单位rad。 */
  double gyro_bias_std_rad_s[3];/* δbg三轴1σ，单位rad/s。 */
  double accel_bias_std_m_s2[3];/* δba三轴1σ，单位m/s²。 */
} zju_coop_inertial_node_initialization_t;

/*
 * 惯性路径配置；nodes仅在configure调用期间借用，库内会深拷贝。
 * 噪声密度和随机游走均采用连续时间单位；message covariance只有在
 * use_message_covariance为真且消息协方差通过对称/半正定检查时才生效。
 */
typedef struct zju_coop_inertial_config {
  uint32_t struct_size;  /* 调用方分配的惯性配置结构字节数。 */
  uint32_t abi_version;  /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t node_count;   /* nodes数组有效元素数，必须覆盖全部惯性节点。 */
  uint32_t node_stride;  /* nodes相邻元素起始地址的字节间隔。 */
  const zju_coop_inertial_node_initialization_t* nodes; /* configure期间借用的初值数组首地址。 */
  uint32_t max_inertial_state_dimension; /* 15×node_count联合误差状态维数的分配上限。 */
  uint32_t reserved0;    /* v1保留占位，调用方必须置零。 */
  double gravity_mps2;   /* 导航ENU中采用的重力加速度标量，单位m/s²。 */
  double min_imu_dt_s;   /* 相邻采样可传播的最小正时间间隔，单位s。 */
  double max_imu_dt_s;   /* 相邻采样仍接受传播的最大时间间隔，单位s。 */
  double max_propagation_substep_s; /* 数值积分单个子步的最大时长，单位s。 */
  double gyro_noise_density_rad_s_sqrt_hz; /* 陀螺白噪声密度，单位rad/s/√Hz。 */
  double accel_noise_density_m_s2_sqrt_hz; /* 加速度计白噪声密度，单位m/s²/√Hz。 */
  double gyro_bias_random_walk_rad_s2_sqrt_hz; /* 陀螺零偏随机游走密度，单位rad/s²/√Hz。 */
  double accel_bias_random_walk_m_s3_sqrt_hz; /* 加速度计零偏随机游走密度，单位m/s³/√Hz。 */
  double min_covariance_diagonal; /* 传播/更新后15N协方差对角元下限。 */
  double quaternion_norm_tolerance; /* 输入四元数范数偏离1的允许误差。 */
  double covariance_symmetry_tolerance; /* 消息3×3协方差对称性检查容差。 */
  char expected_frame_id[32]; /* 期望IMU车体坐标系名，必须在固定容量内NUL结尾。 */
  zju_coop_bool_t use_message_covariance; /* true时通过校验的消息协方差覆盖配置噪声。 */
  zju_coop_bool_t use_orientation_for_initialization; /* true时首包有效姿态可初始化节点四元数。 */
  uint8_t reserved1[6]; /* v1尾部保留字节，调用方必须全部置零。 */
} zju_coop_inertial_config_t;

/*
 * sensor_msgs/Imu到普通C结构体的无ROS映射；不含温度。
 * orientation_covariance[0]==-1表示姿态协方差不可用，全零表示未知；
 * 角速度和线加速度是采样时刻的瞬时量，适配层不得提前做预积分。
 */
typedef struct zju_coop_imu_packet {
  uint32_t struct_size; /* 调用方分配的IMU包结构字节数。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t node_id;     /* 产生样本、且已在惯性配置中注册的平台编号。 */
  uint32_t reserved0;   /* v1保留占位，调用方必须置零。 */
  uint64_t sequence;    /* 该node_id的生产端递增序号，仅用于重复诊断。 */
  uint64_t timestamp_ns;/* 传感器采样的统一时间，单位ns，决定传播顺序。 */
  uint64_t receive_timestamp_ns; /* 本机收包的同一统一时间，单位ns，用于延迟检查。 */
  double orientation_xyzw[4]; /* 车体FLU到导航ENU的[x,y,z,w]四元数。 */
  double orientation_covariance[9]; /* 对应姿态误差的3×3行主序协方差，单位rad²；首项-1表示不可用。 */
  double angular_velocity_rad_s[3]; /* 车体FLU三轴瞬时角速度，单位rad/s，未预积分。 */
  double angular_velocity_covariance[9]; /* 对应角速度的3×3行主序协方差，单位rad²/s²。 */
  double linear_acceleration_m_s2[3]; /* 车体FLU三轴瞬时比力，单位m/s²，含对重力响应（静止水平时+Z约为+g），未预积分。 */
  double linear_acceleration_covariance[9]; /* 对应比力的3×3行主序协方差，单位m²/s⁴。 */
  char frame_id[32]; /* 样本坐标系名，必须NUL结尾并匹配expected_frame_id。 */
  zju_coop_bool_t orientation_valid; /* orientation_xyzw是否可信，独立于整包valid。 */
  zju_coop_bool_t valid; /* 角速度、比力和时间是否可供算法处理。 */
  uint8_t status;        /* 设备侧OK/DEGRADED/INVALID质量码。 */
  uint8_t reserved1[5];  /* v1尾部保留字节，调用方必须全部置零。 */
} zju_coop_imu_packet_t;

typedef struct zju_coop_imu_processing_result {
  uint32_t struct_size; /* 调用方分配的诊断结构字节数，返回时保留原值。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  zju_coop_imu_disposition_t disposition; /* 本包在校验、排序或传播阶段的最终结论。 */
  zju_coop_bool_t propagated; /* true表示该包已实际推进惯性状态和协方差。 */
  uint8_t reserved0[3]; /* v1布局填充，输出为零。 */
  double dt_s; /* 本包计算出的相邻IMU采样间隔，单位s；INTERVAL_REJECTED等未传播结果也可非零。 */
} zju_coop_imu_processing_result_t;

/* 测距处理诊断；区分数据处理结论、融合动作和滤波更新结论。 */
typedef struct zju_coop_range_processing_result {
  uint32_t struct_size; /* 调用方分配的诊断结构字节数，返回时保留原值。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t from_node;   /* 规范化无向边的较小节点编号。 */
  uint32_t to_node;     /* 规范化无向边的较大节点编号。 */
  zju_coop_processing_disposition_t disposition; /* 从入口校验到质量状态机的整包处理结论。 */
  zju_coop_fusion_action_t fusion_action; /* 当前边质量状态给出的正常/降权/保持/拒绝/试探动作。 */
  zju_coop_update_disposition_t update_disposition; /* 真正进入量测模型后的滤波数值结论。 */
  zju_coop_bool_t filter_updated; /* true表示量测更新路径已消费并判定本包；是否接受由update_disposition给出。 */
  uint8_t reserved0[3]; /* v1布局填充，输出为零。 */
  double innovation_m; /* 实测距离减预测距离的创新，单位m；未更新时为诊断默认值。 */
  double innovation_variance; /* 创新方差，单位m²。 */
  double nis; /* 创新平方除以创新方差的无量纲一致性统计量。 */
  double covariance_scale; /* 本包实际采用的测距方差倍率，可由边质量状态和单包NLOS证据共同影响。 */
} zju_coop_range_processing_result_t;

/*
 * 主参考节点坐标系下的二维相对定位输出；当前yaw和z均明确标记无效。
 * x/y/vx/vy是“节点减参考节点”的ENU平面相对量，2×2位置协方差按
 * [cov_xx cov_xy; cov_xy cov_yy]解释，不能当作经纬度或地图绝对坐标。
 */
typedef struct zju_coop_localization {
  uint32_t struct_size; /* 调用方分配的输出结构字节数，写回时保留原值。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint64_t timestamp_ns;/* 快照对应的统一算法时间，单位ns。 */
  uint32_t node_id;     /* 本条相对定位所属平台编号。 */
  uint32_t reference_node_id; /* x/y/vx/vy所相对的参考平台编号。 */
  double x;  /* node-reference在ENU平面的东向相对位置，单位m。 */
  double y;  /* node-reference在ENU平面的北向相对位置，单位m。 */
  double vx; /* node-reference的东向相对速度，单位m/s。 */
  double vy; /* node-reference的北向相对速度，单位m/s。 */
  double cov_xx; /* 平面位置2×2协方差的(0,0)项，单位m²。 */
  double cov_xy; /* 平面位置2×2协方差的对称非对角项，单位m²。 */
  double cov_yy; /* 平面位置2×2协方差的(1,1)项，单位m²。 */
  zju_coop_bool_t valid; /* true表示平面相对位置/速度当前可供业务使用。 */
  zju_coop_bool_t yaw_valid; /* true表示另有有效航向；v1当前固定为false。 */
  zju_coop_bool_t z_valid;   /* true表示另有有效高度；v1当前固定为false。 */
  uint8_t reserved0; /* v1布局填充，输出为零。 */
  zju_coop_localization_state_t state; /* 综合质量、连通性和陈旧性形成的GCS状态。 */
} zju_coop_localization_t;

/* 当前动态协同图的连通性、可观性、活动边数和综合退化原因。 */
typedef struct zju_coop_network {
  uint32_t struct_size; /* 调用方分配的输出结构字节数，写回时保留原值。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint64_t timestamp_ns;/* 拓扑快照对应的统一算法时间，单位ns。 */
  uint32_t node_count;  /* 已配置协同图中的节点总数。 */
  uint32_t reachable_node_count; /* 从参考节点经活动边可达的节点数，包含参考节点。 */
  uint32_t active_edge_count; /* 未超过edge_timeout且参与当前拓扑的无向边数。 */
  zju_coop_bool_t connected; /* true表示全部配置节点均从参考节点可达。 */
  zju_coop_bool_t observable;/* true表示当前几何约束满足平面相对定位秩条件。 */
  uint8_t reserved0[2]; /* v1布局填充，输出为零。 */
  zju_coop_reason_mask_t reason_mask; /* 可按位组合的网络退化/不可观原因。 */
  zju_coop_localization_state_t state; /* 面向GCS的当前综合定位状态。 */
} zju_coop_network_t;

/* 单条协同边在质量窗口内的计数、比率、状态和实际融合动作。 */
typedef struct zju_coop_observation {
  uint32_t struct_size; /* 调用方分配的输出结构字节数，写回时保留原值。 */
  uint32_t abi_version; /* 必须为ZJU_COOP_ABI_VERSION_V1。 */
  uint32_t from_node;   /* 规范化无向边的较小节点编号。 */
  uint32_t to_node;     /* 规范化无向边的较大节点编号。 */
  uint64_t window_start_ns; /* 当前质量滑窗起点，统一时间轴纳秒。 */
  uint64_t window_end_ns;   /* 当前质量滑窗终点，统一时间轴纳秒。 */
  uint32_t expected_count;  /* 由滑窗时长与nominal_rate_hz估算的应到包数。 */
  uint32_t received_count;  /* 滑窗内实际进入质量监测器的包数。 */
  uint32_t valid_count;     /* received_count中通过设备有效性检查的包数。 */
  uint32_t nlos_count;      /* 滑窗内被硬标志或概率门限判为NLOS的包数。 */
  uint32_t residual_rejected_count; /* 因滤波残差/NIS门限被拒绝的包数。 */
  uint32_t dropped_count;   /* 样本历史容量溢出并淘汰最旧样本的生命周期累计次数。 */
  double nlos_ratio;        /* nlos_count/received_count形成的[0,1]质量比率。 */
  double valid_ratio;       /* valid_count/expected_count；输入突发时可大于1，不承诺[0,1]。 */
  double actual_rate_hz;    /* received_count固定除以配置degradation_window_ns换算的频率，单位Hz。 */
  zju_coop_observation_state_t state; /* 该边的长期质量状态。 */
  zju_coop_fusion_action_t fusion_action; /* 当前状态对应的实际滤波动作。 */
  zju_coop_reason_mask_t reason_mask; /* 可按位组合的该边退化原因。 */
  zju_coop_bool_t input_overflow; /* true表示当前滑窗内发生过样本历史溢出淘汰，并非永久故障标志。 */
  uint8_t reserved0[3]; /* v1布局填充，输出为零。 */
  double covariance_scale; /* 当前融合动作施加到测距方差的无量纲倍率。 */
} zju_coop_observation_t;

/* 版本查询不需要创建handle，可用于上交wrapper启动时的兼容性检查。 */
ZJU_COOP_API uint32_t ZJU_COOP_CALL zju_coop_abi_version(void);
ZJU_COOP_API const char* ZJU_COOP_CALL zju_coop_version_string(void);
ZJU_COOP_API const char* ZJU_COOP_CALL
/* code为待转换的稳定C ABI错误码；返回静态只读英文字符串，调用方不得释放。 */
zju_coop_error_string(zju_coop_error_code_t code);

/* 初始化函数清零保留字段并写入struct_size/abi_version，禁止调用方自行猜测布局。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value指向调用方可写结构；成功时整体清零并建立v1头部与安全默认值。 */
zju_coop_node_initialization_init(zju_coop_node_initialization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value指向调用方可写配置；成功时写入v1布局和防御性默认参数。 */
zju_coop_config_init(zju_coop_config_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value指向调用方可写测距包；成功时建立v1头部和默认量测标准差。 */
zju_coop_range_packet_init(zju_coop_range_packet_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_range_processing_result_init(
    /* value为调用方可写诊断结构，初始化后才能传入push_range。 */
    zju_coop_range_processing_result_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方数组中的一个可写元素，初始化后才能交给step。 */
zju_coop_localization_init(zju_coop_localization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方持有的可写网络快照，初始化后才能交给step。 */
zju_coop_network_init(zju_coop_network_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方数组中的一个可写边质量元素，初始化后才能交给step。 */
zju_coop_observation_init(zju_coop_observation_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_inertial_node_initialization_init(
    /* value为调用方可写惯性节点初值，成功时写单位四元数与各状态块默认1σ。 */
    zju_coop_inertial_node_initialization_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方可写惯性配置，成功时写传播门限、连续噪声和默认帧名。 */
zju_coop_inertial_config_init(zju_coop_inertial_config_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方可写IMU包，成功时建立v1头部和单位姿态默认值。 */
zju_coop_imu_packet_init(zju_coop_imu_packet_t* value);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* value为调用方可写IMU诊断结构，初始化后才能传入push_imu。 */
zju_coop_imu_processing_result_init(zju_coop_imu_processing_result_t* value);

/*
 * 创建基础实例；成功时*out_handle归调用方并必须destroy一次。
 * 默认融合路线要求在首个IMU或测距输入前调用configure_inertial；
 * 若显式使用仅测距回退配置，则不配置惯性状态。
 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* config在调用期间只读并被深拷贝；out_handle接收成功创建且须由destroy释放的独占句柄。 */
zju_coop_create(const zju_coop_config_t* config,
                zju_coop_handle_t** out_handle);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* handle为create返回的独占句柄；成功后地址立即失效，不得再次使用或销毁。 */
zju_coop_destroy(zju_coop_handle_t* handle);
/* configure成功后节点集合和状态维度被冻结，处理开始后不允许重新配置。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* handle为尚未开始处理的会话；config仅在调用期间借用并由库深拷贝。 */
zju_coop_configure_inertial(zju_coop_handle_t* handle,
                            const zju_coop_inertial_config_t* config);
/* IMU输入为瞬时角速度/比力，不是调用方计算好的预积分量。 */
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* handle为已配置惯性的会话，packet为瞬时IMU只读输入，result为已初始化且成功时整体写回的调用方缓冲。 */
zju_coop_push_imu(zju_coop_handle_t* handle,
                  const zju_coop_imu_packet_t* packet,
                  zju_coop_imu_processing_result_t* result);
ZJU_COOP_API zju_coop_error_code_t ZJU_COOP_CALL
/* handle为算法会话，packet为只读测距输入，result为已初始化且成功时整体写回的调用方缓冲。 */
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
/*
 * handle为待推进会话；now_ns为统一时间轴目标快照时刻。localizations/observations
 * 分别指向调用方预初始化的定位/边质量数组，capacity为元素容量，stride为相邻元素
 * 字节步长；localization_count/observation_count返回所需或实写元素数。network为
 * 预初始化的单个网络快照。容量查询时两数组必须为NULL且对应capacity为0。
 */
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

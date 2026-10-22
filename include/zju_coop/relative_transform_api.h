/* Nguyen/Xie (TRO 2023) 四自由度相对坐标变换估计；独立于默认算法。
 * 单句柄估计一对节点的 T_L1_L2=[Rz(yaw),t]，位置单位m，角度rad。
 * 同一句柄须串行调用；所有结构须先init，保留字段须为0。
 */
#ifndef ZJU_COOP_RELATIVE_TRANSFORM_API_H
#define ZJU_COOP_RELATIVE_TRANSFORM_API_H

#include "zju_coop/c_api.h"

/* 与默认DLL的导出宏分开，使Windows调用方可以同时链接两个库。 */
#if defined(_WIN32) || defined(__CYGWIN__)
# if defined(ZJU_COOP_BUILDING_RTE_LIBRARY)
#  define ZJU_COOP_RTE_API __declspec(dllexport)
# else
#  define ZJU_COOP_RTE_API __declspec(dllimport)
# endif
#else
# define ZJU_COOP_RTE_API ZJU_COOP_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ZJU_COOP_RTE_ABI_VERSION_V1 UINT32_C(0x00010000)
typedef struct zju_coop_rte_handle zju_coop_rte_handle_t;
typedef int32_t zju_coop_rte_status_t;
#define ZJU_COOP_RTE_NO_ESTIMATE ((zju_coop_rte_status_t)0)
#define ZJU_COOP_RTE_READY ((zju_coop_rte_status_t)1)
#define ZJU_COOP_RTE_UNOBSERVABLE ((zju_coop_rte_status_t)2)
#define ZJU_COOP_RTE_NOT_CONVERGED ((zju_coop_rte_status_t)3)
#define ZJU_COOP_RTE_AMBIGUOUS ((zju_coop_rte_status_t)4)
#define ZJU_COOP_RTE_NUMERICAL_FAILURE ((zju_coop_rte_status_t)5)

typedef struct zju_coop_config_rte {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t host_node_id;
  uint32_t target_node_id;
  uint32_t max_samples;       /* 滑窗上限，默认512，范围[8,4096]。 */
  uint32_t minimum_samples;   /* 默认12；至少8组。 */
  uint32_t max_iterations;    /* 每个初值的LM迭代上限，默认100。 */
  uint32_t yaw_starts;        /* 全周航向初值数量，默认24。 */
  uint64_t max_epoch_skew_ns; /* NodeState配对容差，默认20ms。 */
  double range_std_m;        /* 直接sample的std=0时使用的默认值。 */
  double convergence_tol;    /* 默认1e-9。 */
  double condition_limit;    /* FIM条件数上限，默认1e10。 */
  double host_lever_arm_body_m[3];
  double target_lever_arm_body_m[3];
} zju_coop_config_rte_t;

/* 直接输入已同步、已补偿杆臂的天线位置：d=||t+Rz(yaw)p2-p1||。
 * 两个局部坐标系的z轴须对齐重力，但无需公共位置或公共航向。
 * 时间戳必须非零且严格递增；range_std_m=0使用config默认值。
 */
typedef struct zju_coop_sample_rte {
  uint32_t struct_size;
  uint32_t abi_version;
  uint64_t timestamp_ns;
  double host_antenna_position_m[3];
  double target_antenna_position_m[3];
  double range_m;
  double range_std_m;
} zju_coop_sample_rte_t;

typedef struct zju_coop_estimate_rte {
  uint32_t struct_size;
  uint32_t abi_version;
  zju_coop_rte_status_t status;
  uint32_t sample_count;
  uint32_t iterations;        /* 最佳初值实际迭代数。 */
  uint32_t reserved0;
  uint64_t timestamp_ns;      /* 窗口末样本时间；不是调用时刻。 */
  double translation_m[3];    /* L2原点在L1中的位置，不是当前车间向量。 */
  double yaw_rad;             /* L2到L1的坐标系航向差，[-pi,pi)。 */
  double rms_residual_m;
  double squared_distance_cost; /* 0.5 sum((dhat²-(d²-sigma²))²/Sii)。 */
  double condition_number;    /* FIM奇异时为DBL_MAX，始终有限。 */
  double covariance[16];      /* 行主序，状态顺序tx,ty,tz,yaw，FIM的逆。 */
  zju_coop_bool_t covariance_valid;
  uint8_t reserved1[7];
} zju_coop_estimate_rte_t;

ZJU_COOP_RTE_API uint32_t ZJU_COOP_CALL zju_coop_abi_version_rte(void);
ZJU_COOP_RTE_API const char* ZJU_COOP_CALL zju_coop_version_string_rte(void);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_config_init_rte(zju_coop_config_rte_t* value);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_sample_init_rte(zju_coop_sample_rte_t* value);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_estimate_init_rte(zju_coop_estimate_rte_t* value);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_create_rte(const zju_coop_config_rte_t* config,
                    zju_coop_rte_handle_t** out_handle);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_destroy_rte(zju_coop_rte_handle_t* handle);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_push_sample_rte(zju_coop_rte_handle_t* handle,
                         const zju_coop_sample_rte_t* sample);
/* 优化的目标是论文式(36)-(43)，用参数消元满足前四条QCQP约束。
 * 使用多初值LM；不是论文Gurobi全局QCQP求解器或SDP松弛求解器。
 * OK只表示调用成功；必须检查estimate.status==READY才能使用结果。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_step_rte(zju_coop_rte_handle_t* handle,
                  zju_coop_estimate_rte_t* estimate);

/* 适配接口复用c_api.h中的输入结构、错误码和处理回执。
 * RTE中NodeState的*_enu字段表示各车自己的重力对齐局部里程计系。
 * 先送两端状态，再送测距；按测距时间插值位置/姿态，或在容差内取最近状态。
 * 不在库内积分IMU。每端时间戳须严格递增，重置里程计坐标系须重建句柄。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_push_node_state_rte(zju_coop_rte_handle_t* handle,
                             const zju_coop_node_state_t* state);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_push_range_rte(zju_coop_rte_handle_t* handle,
                        const zju_coop_range_packet_t* packet,
                        zju_coop_range_processing_result_t* result);
/* 与既有Pose2D查询相同的缓冲区/stride约定。只读，不求解。
 * 需通过NodeState+range输入并step得到READY结果。输出两车相对位置，
 * 轴平行L1，frame_id=coop_rte_<host>_local；yaw是车体在L1中的航向。
 * 它没有北向参考，不能解释为ENU航向。直接sample模式不提供车体Pose2D。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_get_pose2d_v2_rte(
    zju_coop_rte_handle_t* handle, zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t vehicle_capacity,
    uint32_t vehicle_stride, uint32_t* vehicle_count);

#ifdef __cplusplus
}
#endif
#endif

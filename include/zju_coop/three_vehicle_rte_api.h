/* 可选三车RTE链路：独立局部里程计 -> 三车坐标对齐 -> 滑窗持续测距修正。
 * 不调用/改变纯UWB三角形方法；单句柄串行调用。所有结构先init。
 */
#ifndef ZJU_COOP_THREE_VEHICLE_RTE_API_H
#define ZJU_COOP_THREE_VEHICLE_RTE_API_H
#include "zju_coop/relative_transform_api.h"
#ifdef __cplusplus
extern "C" {
#endif
#define ZJU_COOP_RTE_STALE ((zju_coop_rte_status_t)6)
#define ZJU_COOP_RTE_RESIDUAL_REJECTED ((zju_coop_rte_status_t)7)
typedef struct zju_coop_three_handle_rte zju_coop_three_handle_rte_t;

typedef struct zju_coop_three_config_rte {
  uint32_t struct_size;
  uint32_t abi_version;
  uint32_t node_ids[3];          /* 第一个是参考车，默认201、202、203。 */
  uint32_t planar;               /* 1：已知高度平面三自由度/车；0：四自由度/车。 */
  uint32_t max_samples_per_edge; /* 默认128，上限1024。 */
  uint32_t minimum_samples;      /* 初始双车种子所需，默认12。 */
  uint32_t max_iterations;
  uint32_t yaw_starts;
  uint64_t max_epoch_skew_ns;
  uint64_t node_timeout_ns;
  uint64_t range_timeout_ns;
  uint64_t solve_interval_ns;
  uint64_t window_duration_ns;
  double local_origin_heights_m[3]; /* 平面模式：各车局部原点的已知高度，取相对首车。 */
  double antenna_lever_arms_body_m[9]; /* 每车FLU杆臂。 */
  double condition_limit;
  double max_rms_residual_m;
  double max_range_innovation_m;  /* 初始化后离群值门限，默认1m。 */
  double max_translation_std_m;   /* 仅在初次初始化时限制FIM不确定性。 */
  double max_yaw_std_rad;
} zju_coop_three_config_rte_t;

typedef struct zju_coop_three_snapshot_rte {
  uint32_t struct_size;
  uint32_t abi_version;
  zju_coop_rte_status_t status;
  uint32_t node_ids[3];
  uint64_t timestamp_ns;          /* 三车对齐的共同状态时间。 */
  uint64_t initialized_at_ns;     /* 首次通过初始化门限的时间，后续不覆盖。 */
  uint64_t last_correction_ns;    /* 最近一次成功滑窗优化的时间。 */
  uint64_t correction_count;
  uint64_t accepted_ranges;
  uint64_t rejected_ranges;
  uint32_t active_edges;
  uint32_t sample_counts[3];      /* 按01、02、12。 */
  double origin_translation_m[9];/* 三个局部系原点在参考局部系中的位置。 */
  double origin_yaw_rad[3];
  double relative_position_m[9]; /* 当前车体位置减参考车体位置，轴平行参考局部系。 */
  double relative_velocity_mps[9];
  double body_orientation_xyzw[12]; /* 车体到参考局部系，不是北向ENU。 */
  double position_std_m[9];      /* 变换平移标准差；给定里程计的条件协方差。 */
  double yaw_std_rad[3];
  double rms_residual_m;
  double condition_number;
} zju_coop_three_snapshot_rte_t;

ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_config_init_rte(zju_coop_three_config_rte_t* value);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_snapshot_init_rte(zju_coop_three_snapshot_rte_t* value);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_create_rte(const zju_coop_three_config_rte_t* config,
                          zju_coop_three_handle_rte_t** out_handle);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_destroy_rte(zju_coop_three_handle_rte_t* handle);
/* NodeState的*_enu字段在本方法表示各车自己的局部里程计系，z轴共享重力。
 * 平面模式显式约束车体局部z位移和z速度为0，保留姿态与天线杆臂的高度影响。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_push_node_state_rte(zju_coop_three_handle_rte_t* handle,
                                   const zju_coop_node_state_t* state);
/* 支持三条无向边，按测距时间对齐。HELD量测在库中有界暂存，后续step重试。
 * 重复/倒序/无效/NLOS/初始化后创新过大均拒绝；filter_updated仍为false，
 * 此方法入窗，实际三车联合修正由step执行。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_push_range_rte(zju_coop_three_handle_rte_t* handle,
                              const zju_coop_range_packet_t* packet,
                              zju_coop_range_processing_result_t* result);
/* now_ns必须与输入同一时间域。即使没有新测距，也更新共同历元和失效状态。
 * 返回OK并不保证已定位，只有snapshot.status==READY结果才可用。
 */
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_step_rte(zju_coop_three_handle_rte_t* handle, uint64_t now_ns,
                        zju_coop_three_snapshot_rte_t* snapshot);
ZJU_COOP_RTE_API zju_coop_error_code_t ZJU_COOP_CALL
zju_coop_three_get_pose2d_v2_rte(
    zju_coop_three_handle_rte_t* handle, uint64_t now_ns,
    zju_coop_pose2d_snapshot_v2_t* snapshot,
    zju_coop_vehicle_pose2d_v2_t* vehicles, uint32_t vehicle_capacity,
    uint32_t vehicle_stride, uint32_t* vehicle_count);
#ifdef __cplusplus
}
#endif
#endif

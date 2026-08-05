/*
 * 模块职责：用纯C编译器验证公开头文件、结构初始化器和基础句柄生命周期不依赖C++。
 * C初学者阅读提示：本测试只检查“C程序能否包含头文件并调用接口”，不评价定位精度。
 * main返回0代表所有调用得到预期错误码/成功码，返回非0代表对应序号的检查失败。
 */
#include "zju_coop/c_api.h"

int main(void) {
  /* config/node/packet/result：分别覆盖引擎配置、节点初值、测距输入包和测距处理输出的C初始化入口。 */
  zju_coop_config_t config;
  zju_coop_node_initialization_t node;
  zju_coop_range_packet_t packet;
  zju_coop_range_processing_result_t result;
  /* localization/network/observation：覆盖定位快照、网络状态和观测记录的版本化结构初始化。 */
  zju_coop_localization_t localization;
  zju_coop_network_t network;
  zju_coop_observation_t observation;
  /* inertial_node/inertial_config/imu_packet/imu_result：覆盖惯导扩展的节点、配置、输入包和处理回执。 */
  zju_coop_inertial_node_initialization_t inertial_node;
  zju_coop_inertial_config_t inertial_config;
  zju_coop_imu_packet_t imu_packet;
  zju_coop_imu_processing_result_t imu_result;
  /* raw_result/image/point_field/cloud：覆盖未来视觉与激光原始数据的纯C预留接口。 */
  zju_coop_raw_input_result_t raw_result;
  zju_coop_camera_image_packet_t image;
  zju_coop_point_field_t point_field;
  zju_coop_point_cloud_packet_t cloud;
  /* gnss_*：覆盖标准NavSatFix映射、独立GNSS上下文配置和RTK真值输出结构。 */
  zju_coop_gnss_node_config_t gnss_node;
  zju_coop_gnss_config_t gnss_config;
  zju_coop_gnss_fix_packet_t gnss_fix;
  zju_coop_gnss_processing_result_t gnss_result;
  zju_coop_gnss_relative_truth_t gnss_truth;

  /* `&config`取得变量地址供函数写入；`||`表示任一初始化失败就进入返回1分支。 */
  if (zju_coop_config_init(&config) != ZJU_COOP_OK ||
      zju_coop_node_initialization_init(&node) != ZJU_COOP_OK ||
      zju_coop_range_packet_init(&packet) != ZJU_COOP_OK ||
      zju_coop_range_processing_result_init(&result) != ZJU_COOP_OK ||
      zju_coop_localization_init(&localization) != ZJU_COOP_OK ||
      zju_coop_network_init(&network) != ZJU_COOP_OK ||
      zju_coop_observation_init(&observation) != ZJU_COOP_OK) {
    return 1;
  }
  /* 惯性扩展结构仍由纯C翻译单元编译，防止公开头无意引入C++语法或依赖。 */
  if (zju_coop_inertial_node_initialization_init(&inertial_node) !=
          ZJU_COOP_OK ||
      zju_coop_inertial_config_init(&inertial_config) != ZJU_COOP_OK ||
      zju_coop_imu_packet_init(&imu_packet) != ZJU_COOP_OK ||
      zju_coop_imu_processing_result_init(&imu_result) != ZJU_COOP_OK) {
    return 4;
  }
  /* 原始数据结构只要求纯C可初始化；本冒烟用例不向算法传入真实大缓冲区。 */
  if (zju_coop_raw_input_result_init(&raw_result) != ZJU_COOP_OK ||
      zju_coop_camera_image_packet_init(&image) != ZJU_COOP_OK ||
      zju_coop_point_field_init(&point_field) != ZJU_COOP_OK ||
      zju_coop_point_cloud_packet_init(&cloud) != ZJU_COOP_OK) {
    return 5;
  }
  if (zju_coop_gnss_node_config_init(&gnss_node) != ZJU_COOP_OK ||
      zju_coop_gnss_config_init(&gnss_config) != ZJU_COOP_OK ||
      zju_coop_gnss_fix_packet_init(&gnss_fix) != ZJU_COOP_OK ||
      zju_coop_gnss_processing_result_init(&gnss_result) != ZJU_COOP_OK ||
      zju_coop_gnss_relative_truth_init(&gnss_truth) != ZJU_COOP_OK) {
    return 6;
  }
  if (config.node_stride != sizeof(zju_coop_node_initialization_t)) {
    /* sizeof在编译期得到公开节点结构字节数，用来核对init填写的默认stride。 */
    return 2;
  }
  /* 三目运算符：ABI版本匹配返回成功码0，否则返回可定位的失败码3。 */
  return zju_coop_abi_version() == ZJU_COOP_ABI_VERSION_V1 ? 0 : 3;
}

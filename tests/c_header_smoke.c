/* 模块职责：用纯C编译器验证公开头文件、结构初始化器和基础句柄生命周期不依赖C++。 */
#include "zju_coop/c_api.h"

int main(void) {
  /* 关键检查：C调用方必须先init版本化结构，再创建并销毁不透明句柄。 */
  zju_coop_config_t config;
  zju_coop_node_initialization_t node;
  zju_coop_range_packet_t packet;
  zju_coop_range_processing_result_t result;
  zju_coop_localization_t localization;
  zju_coop_network_t network;
  zju_coop_observation_t observation;
  zju_coop_inertial_node_initialization_t inertial_node;
  zju_coop_inertial_config_t inertial_config;
  zju_coop_imu_packet_t imu_packet;
  zju_coop_imu_processing_result_t imu_result;

  if (zju_coop_config_init(&config) != ZJU_COOP_OK ||
      zju_coop_node_initialization_init(&node) != ZJU_COOP_OK ||
      zju_coop_range_packet_init(&packet) != ZJU_COOP_OK ||
      zju_coop_range_processing_result_init(&result) != ZJU_COOP_OK ||
      zju_coop_localization_init(&localization) != ZJU_COOP_OK ||
      zju_coop_network_init(&network) != ZJU_COOP_OK ||
      zju_coop_observation_init(&observation) != ZJU_COOP_OK) {
    return 1;
  }
  if (zju_coop_inertial_node_initialization_init(&inertial_node) !=
          ZJU_COOP_OK ||
      zju_coop_inertial_config_init(&inertial_config) != ZJU_COOP_OK ||
      zju_coop_imu_packet_init(&imu_packet) != ZJU_COOP_OK ||
      zju_coop_imu_processing_result_init(&imu_result) != ZJU_COOP_OK) {
    return 4;
  }
  if (config.node_stride != sizeof(zju_coop_node_initialization_t)) {
    return 2;
  }
  return zju_coop_abi_version() == ZJU_COOP_ABI_VERSION_V1 ? 0 : 3;
}

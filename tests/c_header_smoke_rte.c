#include "zju_coop/relative_transform_api.h"

/* 纯C调用只链接RTE库，证明它没有隐式依赖默认libzju_coop。 */
int main(void) {
  zju_coop_config_rte_t config;
  zju_coop_estimate_rte_t result;
  zju_coop_rte_handle_t* handle = 0;
  if (zju_coop_abi_version_rte() != ZJU_COOP_RTE_ABI_VERSION_V1) return 1;
  if (zju_coop_config_init_rte(&config) != ZJU_COOP_OK) return 2;
  if (zju_coop_estimate_init_rte(&result) != ZJU_COOP_OK) return 3;
  if (zju_coop_create_rte(&config, &handle) != ZJU_COOP_OK) return 4;
  if (zju_coop_step_rte(handle, &result) != ZJU_COOP_NOT_READY) return 5;
  if (result.status != ZJU_COOP_RTE_NO_ESTIMATE) return 6;
  return zju_coop_destroy_rte(handle) == ZJU_COOP_OK ? 0 : 7;
}

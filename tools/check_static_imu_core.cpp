// 从stdin读取已转换为FLU/SI的 gx gy gz ax ay az，每行一帧。
// 只调用当前工程C ABI的静止初始化；不重写初始化算法，也不启动ROS。
#include "zju_coop/c_api.h"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
  zju_coop_static_imu_config_t config{};
  zju_coop_static_imu_result_t result{};
  if (zju_coop_static_imu_config_init(&config) != ZJU_COOP_OK ||
      zju_coop_static_imu_result_init(&result) != ZJU_COOP_OK) return 1;
  config.initial_yaw_rad = argc > 1 ? std::stod(argv[1]) : 0.0;
  std::vector<zju_coop_static_imu_sample_t> samples;
  zju_coop_static_imu_sample_t sample{};
  if (zju_coop_static_imu_sample_init(&sample) != ZJU_COOP_OK) return 1;
  while (std::cin >> sample.angular_velocity_rad_s[0] >> sample.angular_velocity_rad_s[1]
         >> sample.angular_velocity_rad_s[2] >> sample.specific_force_m_s2[0]
         >> sample.specific_force_m_s2[1] >> sample.specific_force_m_s2[2]) {
    samples.push_back(sample);
  }
  if (!std::cin.eof() || samples.empty()) return 2;
  const auto code = zju_coop_initialize_static_imu(
      &config, samples.data(), static_cast<uint32_t>(samples.size()), sizeof(sample), &result);
  if (code != ZJU_COOP_OK) return 3;
  std::cout << std::setprecision(17) << "{\"disposition\":" << result.disposition
            << ",\"sample_count\":" << result.sample_count << ",\"bias\":[";
  for (int i=0; i<3; ++i) std::cout << (i ? "," : "") << result.gyro_bias_flu_rad_s[i];
  std::cout << "],\"quaternion\":[";
  for (int i=0; i<4; ++i) std::cout << (i ? "," : "") << result.orientation_flu_to_enu_xyzw[i];
  std::cout << "]}\n";
}

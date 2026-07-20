// 上交集成参考：演示不依赖 ROS 2 类型的 C ABI 初始化、输入、查询输出和销毁顺序。
#include "zju_coop/c_api.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

bool check(zju_coop_error_code_t code, const char* operation) {
  if (code == ZJU_COOP_OK) {
    return true;
  }
  std::cerr << operation << ": " << zju_coop_error_string(code) << '\n';
  return false;
}

}  // namespace

int main() {
  std::array<zju_coop_node_initialization_t, 3U> nodes{};
  for (auto& node : nodes) {
    if (!check(zju_coop_node_initialization_init(&node), "init node")) {
      return 1;
    }
    node.position_std_m = 0.1;
    node.velocity_std_mps = 0.1;
  }
  nodes[0U].node_id = 1U;
  nodes[1U].node_id = 2U;
  nodes[1U].x = 3.0;
  nodes[2U].node_id = 3U;
  nodes[2U].y = 4.0;

  zju_coop_config_t config{};
  if (!check(zju_coop_config_init(&config), "init config")) {
    return 1;
  }
  config.reference_node_id = 1U;
  config.nodes = nodes.data();
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  config.node_stride = sizeof(zju_coop_node_initialization_t);
  config.nis_gate = 1.0e9;

  zju_coop_handle_t* handle{};
  if (!check(zju_coop_create(&config, &handle), "create")) {
    return 1;
  }

  constexpr std::array<std::uint16_t, 3U> from{1U, 1U, 2U};
  constexpr std::array<std::uint16_t, 3U> to{2U, 3U, 3U};
  constexpr std::array<double, 3U> ranges{3.0, 4.0, 5.0};
  for (std::size_t index = 0U; index < ranges.size(); ++index) {
    zju_coop_range_packet_t packet{};
    zju_coop_range_processing_result_t result{};
    check(zju_coop_range_packet_init(&packet), "init range");
    check(zju_coop_range_processing_result_init(&result), "init result");
    packet.from_node = from[index];
    packet.to_node = to[index];
    packet.sequence = static_cast<std::uint64_t>(index + 1U);
    packet.timestamp_ns = 50'000'000ULL;
    packet.receive_timestamp_ns = packet.timestamp_ns;
    packet.range_m = ranges[index];
    packet.range_std_m = 0.1;
    packet.valid = ZJU_COOP_TRUE;
    if (!check(zju_coop_push_range(handle, &packet, &result), "push range")) {
      zju_coop_destroy(handle);
      return 1;
    }
  }

  std::uint32_t localization_count{};
  std::uint32_t observation_count{};
  const auto query = zju_coop_step(
      handle, 50'000'000ULL, nullptr, 0U, 0U, &localization_count, nullptr,
      0U, 0U, &observation_count, nullptr);
  if (query != ZJU_COOP_BUFFER_TOO_SMALL) {
    std::cerr << "step query: " << zju_coop_error_string(query) << '\n';
    zju_coop_destroy(handle);
    return 1;
  }

  std::vector<zju_coop_localization_t> localizations(localization_count);
  std::vector<zju_coop_observation_t> observations(observation_count);
  zju_coop_network_t network{};
  for (auto& value : localizations) {
    check(zju_coop_localization_init(&value), "init localization");
  }
  for (auto& value : observations) {
    check(zju_coop_observation_init(&value), "init observation");
  }
  check(zju_coop_network_init(&network), "init network");
  if (!check(zju_coop_step(handle, 50'000'000ULL, localizations.data(),
                           localization_count,
                           sizeof(zju_coop_localization_t),
                           &localization_count, observations.data(),
                           observation_count,
                           sizeof(zju_coop_observation_t),
                           &observation_count, &network),
             "step")) {
    zju_coop_destroy(handle);
    return 1;
  }

  for (const auto& value : localizations) {
    std::cout << "node=" << value.node_id << " ref="
              << value.reference_node_id << " x=" << value.x
              << " y=" << value.y << " yaw_valid="
              << static_cast<unsigned int>(value.yaw_valid) << " z_valid="
              << static_cast<unsigned int>(value.z_valid) << '\n';
  }
  std::cout << "connected=" << static_cast<unsigned int>(network.connected)
            << " observable=" << static_cast<unsigned int>(network.observable)
            << " active_edges=" << network.active_edge_count << '\n';

  return check(zju_coop_destroy(handle), "destroy") ? 0 : 1;
}

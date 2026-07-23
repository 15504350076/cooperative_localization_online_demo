// 模块职责：给上交集成人员演示不依赖ROS 2类型的C ABI最小生命周期。
// 示例刻意只展示结构初始化、会话创建、测距输入、两阶段step和销毁；正式默认IMU+测距
// 还须在首个输入前调用zju_coop_configure_inertial并持续输入各节点瞬时IMU。
#include "zju_coop/c_api.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

/** @param code 待检查的C ABI返回码；@param operation 失败时输出的调用名称。 */
bool check(zju_coop_error_code_t code, const char* operation) {
  if (code == ZJU_COOP_OK) {
    return true;
  }
  std::cerr << operation << ": " << zju_coop_error_string(code) << '\n';
  return false;
}

}  // namespace

int main() {
  // 阶段1：每个版本化结构先调用init，再填写业务字段，禁止直接依赖默认内存布局。
  std::array<zju_coop_node_initialization_t, 3U> nodes{};  // 三车1=(0,0)、2=(3,0)、3=(0,4)的初值数组。
  for (auto& node : nodes) {  // node为待调用版本化初始化器并设置公共噪声的槽位。
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

  zju_coop_config_t config{};  // 引用nodes并创建示例会话的版本化配置。
  if (!check(zju_coop_config_init(&config), "init config")) {
    return 1;
  }
  config.reference_node_id = 1U;
  config.nodes = nodes.data();
  config.node_count = static_cast<std::uint32_t>(nodes.size());
  config.node_stride = sizeof(zju_coop_node_initialization_t);
  config.nis_gate = 1.0e9;

  // 阶段2：create深拷贝节点数组，成功后句柄所有权归调用方。
  zju_coop_handle_t* handle{};  // create返回、调用方负责显式destroy的不透明会话句柄。
  if (!check(zju_coop_create(&config, &handle), "create")) {
    return 1;
  }

  // 阶段3：输入三车3-4-5三角形的直接测距，形成完整平面约束。
  constexpr std::array<std::uint16_t, 3U> from{1U, 1U, 2U};  // 三条无向约束的发送端编号。
  constexpr std::array<std::uint16_t, 3U> to{2U, 3U, 3U};    // 与from逐项对应的接收端编号。
  constexpr std::array<double, 3U> ranges{3.0, 4.0, 5.0};    // 与端点数组逐项对应的欧氏距离。
  // index同步选择端点和距离；packet承载当前边输入，result接收其C ABI处置。
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

  // 阶段4：先用空缓冲查询输出数量，再初始化数组并执行真正的step。
  std::uint32_t localization_count{};  // 空缓冲查询回填的定位元素数。
  std::uint32_t observation_count{};   // 空缓冲查询回填的观测元素数。
  const auto query = zju_coop_step(  // 两阶段step第一次容量查询的返回码。
      handle, 50'000'000ULL, nullptr, 0U, 0U, &localization_count, nullptr,
      0U, 0U, &observation_count, nullptr);
  if (query != ZJU_COOP_BUFFER_TOO_SMALL) {
    std::cerr << "step query: " << zju_coop_error_string(query) << '\n';
    zju_coop_destroy(handle);
    return 1;
  }

  std::vector<zju_coop_localization_t> localizations(localization_count);  // 第二次step的定位输出缓冲。
  std::vector<zju_coop_observation_t> observations(observation_count);    // 第二次step的观测输出缓冲。
  zju_coop_network_t network{};  // 与两个数组同次生成的网络输出缓冲。
  // 两个value分别是待设置版本握手字段的定位槽位和观测槽位。
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

  for (const auto& value : localizations) {  // value为逐节点展示的相对定位结果。
    std::cout << "node=" << value.node_id << " ref="
              << value.reference_node_id << " x=" << value.x
              << " y=" << value.y << " yaw_valid="
              << static_cast<unsigned int>(value.yaw_valid) << " z_valid="
              << static_cast<unsigned int>(value.z_valid) << '\n';
  }
  std::cout << "connected=" << static_cast<unsigned int>(network.connected)
            << " observable=" << static_cast<unsigned int>(network.observable)
            << " active_edges=" << network.active_edge_count << '\n';

  // 阶段5：会话必须显式销毁；销毁后不得继续使用handle。
  return check(zju_coop_destroy(handle), "destroy") ? 0 : 1;
}

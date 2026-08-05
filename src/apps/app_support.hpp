// 模块职责：封装在线/回放程序共用的C ABI会话，并把算法快照转换为临时演示遥测。
// AlgorithmSession管理句柄生命周期，TelemetryEncoder维护告警激活/恢复状态；
// 本层不实现滤波算法，也不把临时ZJCL协议带入核心库。
//
// C++初学者阅读提示：
// 1. AlgorithmSession是C ABI的C++包装器：构造时create，析构时destroy，调用者不再手工管理句柄。
// 2. push_*把一条输入交给算法；step在同一时刻取出定位、观测和网络状态的完整快照。
// 3. TelemetryEncoder把快照变成GCS演示协议帧；它不修改滤波状态。
// 4. 禁止复制会话对象，是为了避免两个对象同时销毁同一个底层handle。
// 5. `~Class()`是析构函数，作用域结束时自动调用；`Class(const Class&)=delete`
//    禁止复制；`std::vector`负责变长输出缓冲的内存分配与释放。
#pragma once

#include "config/ini_config.hpp"
#include "protocol/wire_protocol.hpp"
#include "zju_coop/c_api.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace zju::coop::apps {

/**
 * 一次C ABI step返回的定位、观测和网络原子快照。
 * vector内存由本适配层管理；结构中的struct_size仍保留调用方容量握手值。
 */
struct StepSnapshot {
  std::vector<zju_coop_localization_t> localizations;  ///< C ABI返回的各节点定位数组。
  std::vector<zju_coop_observation_t> observations;    ///< C ABI返回的各无向边质量数组。
  zju_coop_network_t network{};                        ///< 与两组数组同次step生成的网络状态。
};

/** RAII算法会话：配置、输入、step和销毁严格按C ABI生命周期执行。 */
class AlgorithmSession {
 public:
  /** @param config 用于创建基础会话并可选启用惯性路径的完整演示配置。 */
  explicit AlgorithmSession(const config::DemoConfig& config);
  ~AlgorithmSession();

  /** 未命名源会话参数刻意禁用C ABI句柄所有权复制。 */
  AlgorithmSession(const AlgorithmSession&) = delete;
  /** 未命名源会话参数刻意禁用C ABI句柄所有权复制赋值。 */
  AlgorithmSession& operator=(const AlgorithmSession&) = delete;

  /**
   * 把ZJCL公共头和测距载荷合成C ABI输入，receive_timestamp由本机收包时刻提供。
   * @param frame 提供端点、序号和统一测量时间的已校验测距帧。
   * @param payload 已解码的测距及质量载荷；@param receive_timestamp_ns 本机统一接收时间。
   */
  [[nodiscard]] zju_coop_range_processing_result_t push_range(
      const protocol::Frame& frame,
      const protocol::RangePayload& payload,
      std::uint64_t receive_timestamp_ns);
  /**
   * @param frame 提供节点、序号和统一测量时间的已校验IMU帧。
   * @param payload 已解码的瞬时IMU载荷；@param receive_timestamp_ns 本机统一接收时间。
   */
  [[nodiscard]] zju_coop_imu_processing_result_t push_imu(
      const protocol::Frame& frame, const protocol::ImuPayload& payload,
      std::uint64_t receive_timestamp_ns);
  /** @param now_ns 传给C ABI的统一输出时刻。 */
  [[nodiscard]] StepSnapshot step(std::uint64_t now_ns);

 private:
  zju_coop_handle_t* handle_{};  ///< 本会话独占并在析构时销毁的C ABI不透明句柄。
};

struct EncodedOutput {
  protocol::MessageType message_type{protocol::MessageType::kLocalization};  ///< 计数和路由使用的帧类型。
  std::vector<std::uint8_t> bytes;  ///< 已完成协议封装、可直接发送的整帧字节。
};

struct TelemetryCounters {
  std::uint64_t accepted_ranges{};  ///< 运行以来引擎处置为Processed的测距数。
  std::uint64_t rejected_ranges{};  ///< 运行以来未处置为Processed的测距数。
  std::uint64_t protocol_errors{};  ///< 运行以来协议帧或载荷解码失败数。
};

/**
 * 把网络状态和运行计数编码为状态/告警帧，并维护告警生命周期。
 * 首次非正常网络发Active，恢复正常只发一次Cleared；持续同一故障不重复制造
 * 新告警生命周期。该逻辑是演示适配语义，当前不属于C ABI v1输出。
 */
class TelemetryEncoder {
 public:
  /**
   * @param network 本周期C ABI网络快照；@param counters 应用累计运行计数。
   * @param uptime_ns 写入状态载荷的运行/回放相对时长；在线来自单调时钟，回放来自日志接收相对时间。
   * @param reference_node_id 帧头源节点。
   * @param next_sequence 跨输出帧递增的发送序号；@param max_payload_size 协议载荷上限。
   * @param mode 状态帧声明的当前算法模式。
   */
  [[nodiscard]] std::vector<EncodedOutput> encode(
      const zju_coop_network_t& network,
      const TelemetryCounters& counters, std::uint64_t uptime_ns,
      std::uint32_t reference_node_id, std::uint64_t& next_sequence,
      std::size_t max_payload_size,
      protocol::AlgorithmMode mode =
          protocol::AlgorithmMode::kUwbOnlyPlanar);

 private:
  bool alert_active_{};  ///< 是否已有尚未发出恢复事件的告警生命周期。
  std::uint64_t alert_first_timestamp_ns_{};  ///< 当前活动告警首次出现的网络快照时间。
};

/**
 * @param snapshot 待编码的同批C ABI输出；@param reference_node_id 帧头使用的主参考编号。
 * @param next_sequence 跨帧递增的发送序号；@param max_payload_size 协议编码载荷上限。
 */
[[nodiscard]] std::vector<EncodedOutput> encode_snapshot(
    const StepSnapshot& snapshot, std::uint32_t reference_node_id,
    std::uint64_t& next_sequence, std::size_t max_payload_size);

[[nodiscard]] std::uint64_t system_time_ns();

/** @param timestamp_ns 较晚的统一时间；@param start_timestamp_ns 统一时间起点。 */
[[nodiscard]] std::uint64_t elapsed_ns(
    std::uint64_t timestamp_ns, std::uint64_t start_timestamp_ns) noexcept;

}  // namespace zju::coop::apps

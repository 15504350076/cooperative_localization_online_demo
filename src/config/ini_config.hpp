// 模块职责：定义在线/回放程序共享的配置模型和严格INI加载入口。
// 所有滤波阈值、时间门限、资源上限、网络端点和日志参数均由配置文件提供；
// 算法源码不得硬编码现场调参值，解析错误必须携带类别和原始行号。
#pragma once

#include "core/engine.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace zju::coop::config {

/**
 * 临时UDP在线程序参数，不进入算法数学模型。
 * input/output rate分别用于容量/调度检查；max_payload和max_log_record是
 * 独立防御上限，后者必须容纳日志元数据加完整ZJCL帧。
 */
struct OnlineConfig {
  std::string input_bind_address;
  std::uint16_t input_port{};
  std::string output_address;
  std::uint16_t output_port{};
  double input_rate_hz{};
  double output_rate_hz{};
  bool event_log_enabled{};
  std::string event_log_path;
  std::size_t max_payload_size{};
  std::size_t max_log_record_size{};
};

/**
 * 在线Demo的可选惯性路径。默认demo.ini显式启用，只有配置节缺失或
 * enabled=false时才进入UWB-only兼容路径；optional不能被理解成IMU可随包切换。
 */
struct InertialDemoConfig {
  InertialConfig filter;
  std::vector<InertialNodeInitialization> nodes;
  std::size_t max_inertial_state_dimension{960U};
};

struct DemoConfig {
  EngineConfig engine;
  OnlineConfig online;
  std::optional<InertialDemoConfig> inertial;
};

/** 可机器判定的配置错误类别；异常文本同时保留具体节/键和原始行号。 */
enum class IniError {
  kNone,
  kIoFailure,
  kInvalidUtf8,
  kSyntax,
  kDuplicateSection,
  kDuplicateKey,
  kDuplicateNode,
  kUnknownSection,
  kUnknownKey,
  kInvalidValue,
  kMissingSection,
  kMissingKey,
  kMissingReferenceNode,
  kInvalidConfiguration,
};

class IniConfigError final : public std::runtime_error {
 public:
  IniConfigError(IniError code, std::size_t line, std::string message);

  [[nodiscard]] IniError code() const noexcept;
  [[nodiscard]] std::size_t line() const noexcept;

 private:
  IniError code_;
  std::size_t line_;
};

/**
 * 严格解析UTF-8 INI：未知节/键、重复定义、尾随字符和组合约束错误均拒绝，
 * 避免现场拼写错误被当成默认参数继续运行。
 */
[[nodiscard]] DemoConfig parse_ini_config(const std::string& text);
[[nodiscard]] DemoConfig load_ini_config(const std::filesystem::path& path);

}  // namespace zju::coop::config

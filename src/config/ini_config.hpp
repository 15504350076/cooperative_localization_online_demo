// 在线演示配置模型和严格 INI 解析入口；所有运行阈值均来自配置文件。
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

/** 在线Demo的可选惯性路径；缺省不启用，保持原UWB-only配置兼容。 */
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

[[nodiscard]] DemoConfig parse_ini_config(const std::string& text);
[[nodiscard]] DemoConfig load_ini_config(const std::filesystem::path& path);

}  // namespace zju::coop::config

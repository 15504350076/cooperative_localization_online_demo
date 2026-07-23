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
 * 独立防御上限，后者只限制完整ZJCL帧长度（40字节头+payload），不含ZJLG的
 * 4字节长度前缀和12字节记录元数据。
 */
struct OnlineConfig {
  // input_bind_address/input_port为UDP接收套接字绑定端点；地址允许配置通配监听。
  std::string input_bind_address;
  std::uint16_t input_port{};
  // output_address/output_port为编码结果发送到GCS或联调消费者的UDP目标端点。
  std::string output_address;
  std::uint16_t output_port{};
  // input_rate_hz为容量校验用预期入站频率；output_rate_hz为在线主循环发布频率，单位Hz。
  double input_rate_hz{};
  double output_rate_hz{};
  // event_log_enabled=true时按event_log_path创建/截断任务日志；路径字符串由在线程序解释。
  bool event_log_enabled{};
  std::string event_log_path;
  // max_payload_size限制单个ZJCL载荷；max_log_record_size限制完整ZJCL frame（40字节头+payload），
  // 二者单位均为字节；后者不计ZJLG的4字节长度前缀和12字节记录元数据。
  std::size_t max_payload_size{};
  std::size_t max_log_record_size{};
};

/**
 * 在线Demo的可选惯性路径。默认demo.ini显式启用，只有配置节缺失或
 * enabled=false时才进入UWB-only兼容路径；optional不能被理解成IMU可随包切换。
 */
struct InertialDemoConfig {
  // filter为15维误差状态传播参数；nodes为按node_id建立的惯性初值集合，解析完成后由DemoConfig持有。
  InertialConfig filter;
  std::vector<InertialNodeInitialization> nodes;
  // max_inertial_state_dimension限制15×节点数的联合状态维数，防止异常配置造成无界矩阵分配。
  std::size_t max_inertial_state_dimension{960U};
};

struct DemoConfig {
  // engine为UWB质量/拓扑与滤波参数；online为UDP/日志运行参数；inertial有值时启用IMU-UWB路径。
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
  // code为机器可判定类别，line为原始INI的1起始行号（非行级错误可为0），message为面向操作者的上下文文本。
  IniConfigError(IniError code, std::size_t line, std::string message);

  [[nodiscard]] IniError code() const noexcept;
  [[nodiscard]] std::size_t line() const noexcept;

 private:
  IniError code_;      // 由解析失败点固化的错误类别，随异常对象生命周期保持不变。
  std::size_t line_;   // 对应原始UTF-8文本的1起始行号，0表示无法归属单行。
};

/**
 * 严格解析UTF-8 INI：未知节/键、重复定义、尾随字符和组合约束错误均拒绝，
 * 避免现场拼写错误被当成默认参数继续运行。
 */
// text为调用方持有的完整UTF-8 INI内容，函数返回已通过组合约束校验的独立配置值。
[[nodiscard]] DemoConfig parse_ini_config(const std::string& text);
// path为待读取配置文件路径；文件内容仅在调用期间使用，返回值不借用路径或文件缓冲。
[[nodiscard]] DemoConfig load_ini_config(const std::filesystem::path& path);

}  // namespace zju::coop::config

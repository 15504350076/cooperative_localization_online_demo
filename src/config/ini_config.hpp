// 模块职责：定义在线/回放程序共享的配置模型和严格INI加载入口。
// 所有滤波阈值、时间门限、资源上限、网络端点和日志参数均由配置文件提供；
// 算法源码不得硬编码现场调参值，解析错误必须携带类别和原始行号。
//
// C++初学者阅读提示：
// 1. 每个Config结构对应INI中的一组配置，成员后的{}表示先采用安全默认值。
// 2. std::optional<T>表示该配置“可以没有”；使用前先判断has_value()或if(optional)。
// 3. std::filesystem::path专门表示文件路径，比普通字符串更适合跨Windows/Linux使用。
// 4. load_ini是本模块最主要的入口：成功时返回完整配置，失败时抛出带行号的ConfigError。
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
  kNone,                 ///< 没有错误，仅作默认/占位值。
  kIoFailure,            ///< 文件无法打开、读取失败或大小超过限制。
  kInvalidUtf8,          ///< 输入字节不是合法UTF-8编码。
  kSyntax,               ///< 节名、等号、注释或行结构不符合INI语法。
  kDuplicateSection,     ///< 同一普通节重复声明。
  kDuplicateKey,         ///< 同一节内同名键重复，拒绝“后值覆盖前值”歧义。
  kDuplicateNode,        ///< 两个动态节点节解析出相同node_id。
  kUnknownSection,       ///< 出现当前版本不认识的节。
  kUnknownKey,           ///< 已知节中出现当前版本不认识的键。
  kInvalidValue,         ///< 文本可定位到键，但类型、范围或格式不合法。
  kMissingSection,       ///< 缺少必需配置节。
  kMissingKey,           ///< 已有节中缺少必需键。
  kMissingReferenceNode, ///< reference_node_id没有对应节点节。
  kInvalidConfiguration, ///< 单字段合法，但跨字段组合约束不成立。
};

// `final`禁止继续派生；`: public`表示公开继承runtime_error，可被catch(std::exception&)捕获。
class IniConfigError final : public std::runtime_error {
 public:
  // code为机器可判定类别，line为原始INI的1起始行号（非行级错误可为0），message为面向操作者的上下文文本。
  IniConfigError(IniError code, std::size_t line, std::string message);

  /** 返回机器可判定错误类别，不修改异常对象。 */
  [[nodiscard]] IniError code() const noexcept;
  /** 返回1起始原始行号；0表示不是单行错误。 */
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

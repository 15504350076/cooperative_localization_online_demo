// 模块职责：验证UTF-8严格INI的成功加载、未知键、重复项、行号和跨字段资源约束。
// C++初学者阅读提示：测试先在内存中准备INI文本，再交给解析器；合法文本检查字段值，
// 非法文本则用EXPECT_THROW确认会抛出预期异常，而不是悄悄使用错误默认值。
#include "config/ini_config.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using zju::coop::NodeInitialization;
using zju::coop::InertialNodeInitialization;
using zju::coop::config::DemoConfig;
using zju::coop::config::IniConfigError;
using zju::coop::config::IniError;
using zju::coop::config::load_ini_config;
using zju::coop::config::parse_ini_config;

std::string valid_ini() {
  return R"ini(# Zhejiang University cooperative localization demo
[engine]
edge_timeout_ns = 500000000
max_future_skew_ns = 100000000
max_receive_delay_ns = 500000000
duplicate_cache_per_link = 128
max_nodes = 64
max_edges = 2016
max_state_dimension = 252
rigidity_tolerance = 1e-9

[filter]
reference_node_id = 1
process_accel_std_mps2 = 0.2
nis_gate = 9.21
max_prediction_step_s = 0.25
min_covariance_diagonal = 1e-12

[degradation]
window_ns = 2000000000
nominal_rate_hz = 20
nlos_ratio_threshold = 0.30
valid_ratio_threshold = 0.80
rate_ratio_threshold = 0.80
nlos_probability_threshold = 0.50
nlos_covariance_scale = 4
suspend_duration_ns = 1000000000
reject_duration_ns = 3000000000
recovery_duration_ns = 1000000000
max_tracked_edges = 2016

[online]
input_bind_address = 0.0.0.0
input_port = 39001
output_address = 127.0.0.1
output_port = 39002
input_rate_hz = 20
output_rate_hz = 10
event_log_enabled = true
event_log_path = logs/demo.zjlg
max_payload_size = 1048576
max_log_record_size = 1048616

[node.1]
x = 0
y = 0
vx = 0
vy = 0
position_std_m = 0.1
velocity_std_mps = 0.1

[node.2]
x = 3
y = 0
vx = 0
vy = 0
position_std_m = 0.1
velocity_std_mps = 0.1

[node.3] ; C is four metres north of A
x = 0
y = 4
vx = 0
vy = 0
position_std_m = 0.1
velocity_std_mps = 0.1
)ini";
}

// text是按值复制后修改的INI文本，from必须至少匹配一次且只替换首个匹配，to是注入的边界或错误片段。
std::string replace_once(std::string text, const std::string& from,
                         const std::string& to) {
  // position：目标片段的首字节位置，用于保证用例确实改到了预期配置项。
  const std::size_t position = text.find(from);
  EXPECT_TRUE(position != std::string::npos);
  text.replace(position, from.size(), to);
  return text;
}

// text是完整INI输入，token是待定位配置项；返回其一基行号供错误位置断言。
std::size_t line_of(const std::string& text, const std::string& token) {
  // position：token在原始字节串中的偏移，用于统计此前换行数。
  const std::size_t position = text.find(token);
  EXPECT_TRUE(position != std::string::npos);
  return 1U + static_cast<std::size_t>(
                  std::count(text.begin(),
                             text.begin() +
                                 static_cast<std::ptrdiff_t>(position),
                             '\n'));
}

struct CapturedIniError {
  // code/line/message：一次解析失败的分类、原始一基行号和用户诊断，默认值代表未捕获异常。
  IniError code{IniError::kNone};
  std::size_t line{};
  std::string message;
};

// text：预期解析失败的INI内容；error：catch期间借用的异常，立即复制其三个可观察字段。
CapturedIniError capture_ini_error(const std::string& text) {
  try {
    static_cast<void>(parse_ini_config(text));
  } catch (const IniConfigError& error) {
    return {error.code(), error.line(), error.what()};
  }
  return {};
}

// text是错误输入，code是期望分类；局部error保存实际分类、行号和带行号消息。
void expect_ini_error(const std::string& text, IniError code) {
  const CapturedIniError error = capture_ini_error(text);
  EXPECT_EQ(error.code, code);
  EXPECT_TRUE(error.line > 0U);
  EXPECT_TRUE(error.message.find("line " + std::to_string(error.line)) !=
              std::string::npos);
}

// config：在调用期间保持有效的配置；node_id：要查找的节点标识；candidate：逐项借用节点集合元素。
const NodeInitialization& node(const DemoConfig& config,
                               std::uint32_t node_id) {
  for (const auto& candidate : config.engine.nodes) {
    if (candidate.node_id == node_id) {
      return candidate;
    }
  }
  throw std::runtime_error("configured node was not found");
}

// config必须启用惯性配置；node_id是待查平台，返回其已由yaw转换好的15维初始状态。
const InertialNodeInitialization& inertial_node(const DemoConfig& config,
                                                std::uint32_t node_id) {
  if (!config.inertial.has_value()) {
    throw std::runtime_error("inertial configuration is disabled");
  }
  for (const auto& candidate : config.inertial->nodes) {
    if (candidate.node_id == node_id) {
      return candidate;
    }
  }
  throw std::runtime_error("configured inertial node was not found");
}

}  // namespace

// 成功路径先锁定每个配置节的强类型映射和默认/回退示例文件的实际语义。
TEST_CASE(ini_config_maps_all_engine_filter_degradation_online_and_node_values) {
  // config：由内嵌完整INI解析出的强类型配置，用于核对各主要配置节的代表性字段映射。
  const DemoConfig config = parse_ini_config(valid_ini());

  EXPECT_EQ(config.engine.filter.reference_node_id, 1U);
  EXPECT_EQ(config.engine.edge_timeout_ns, 500000000ULL);
  EXPECT_EQ(config.engine.max_future_skew_ns, 100000000ULL);
  EXPECT_EQ(config.engine.max_receive_delay_ns, 500000000ULL);
  EXPECT_EQ(config.engine.duplicate_cache_per_link, 128U);
  EXPECT_EQ(config.engine.max_nodes, 64U);
  EXPECT_EQ(config.engine.max_edges, 2016U);
  EXPECT_EQ(config.engine.max_state_dimension, 252U);
  EXPECT_TRUE(std::abs(config.engine.rigidity_tolerance - 1e-9) < 1e-20);
  EXPECT_EQ(config.engine.degradation.window_ns, 2000000000ULL);
  EXPECT_EQ(config.engine.degradation.nominal_rate_hz, 20.0);
  EXPECT_EQ(config.engine.degradation.nlos_ratio_threshold, 0.30);
  EXPECT_EQ(config.engine.degradation.valid_ratio_threshold, 0.80);
  EXPECT_EQ(config.engine.degradation.rate_ratio_threshold, 0.80);
  EXPECT_EQ(config.engine.degradation.nlos_probability_threshold, 0.50);
  EXPECT_EQ(config.engine.degradation.nlos_covariance_scale, 4.0);
  EXPECT_EQ(config.engine.nodes.size(), 3U);
  EXPECT_EQ(node(config, 1U).x, 0.0);
  EXPECT_EQ(node(config, 2U).x, 3.0);
  EXPECT_EQ(node(config, 3U).y, 4.0);

  EXPECT_EQ(config.online.input_bind_address, std::string("0.0.0.0"));
  EXPECT_EQ(config.online.input_port, 39001U);
  EXPECT_EQ(config.online.output_address, std::string("127.0.0.1"));
  EXPECT_EQ(config.online.output_port, 39002U);
  EXPECT_EQ(config.online.input_rate_hz, 20.0);
  EXPECT_EQ(config.online.output_rate_hz, 10.0);
  EXPECT_TRUE(config.online.event_log_enabled);
  EXPECT_EQ(config.online.event_log_path, std::string("logs/demo.zjlg"));
  EXPECT_EQ(config.online.max_payload_size, 1048576U);
  EXPECT_EQ(config.online.max_log_record_size, 1048616U);
}

TEST_CASE(demo_ini_loads_three_node_three_four_five_geometry_and_udp_ports) {
  // config：从交付默认路径加载的IMU+测距配置，期望保持3-4-5几何和固定UDP端口。
  const DemoConfig config = load_ini_config("config/demo.ini");
  EXPECT_EQ(config.engine.nodes.size(), 3U);
  EXPECT_EQ(config.engine.filter.reference_node_id, 1U);
  EXPECT_EQ(node(config, 1U).x, 0.0);
  EXPECT_EQ(node(config, 1U).y, 0.0);
  EXPECT_EQ(node(config, 2U).x, 3.0);
  EXPECT_EQ(node(config, 2U).y, 0.0);
  EXPECT_EQ(node(config, 3U).x, 0.0);
  EXPECT_EQ(node(config, 3U).y, 4.0);
  // demo.ini 是交付默认配置，必须直接启用 15 维 IMU+测距融合。
  EXPECT_TRUE(config.inertial.has_value());
  EXPECT_EQ(config.inertial->nodes.size(), 3U);
  EXPECT_EQ(config.inertial->filter.expected_frame_id, "imu_link");
  EXPECT_EQ(config.online.input_rate_hz, 100.0);
  EXPECT_EQ(config.online.output_rate_hz, 10.0);
  EXPECT_EQ(config.online.input_port, 39001U);
  EXPECT_EQ(config.online.output_port, 39002U);

  // 三车初始yaw由各node节给出，解析器须转换为FLU车体系到ENU导航系的内部wxyz四元数。
  const auto& q1 = inertial_node(config, 1U).orientation_b_to_n;
  const auto& q2 = inertial_node(config, 2U).orientation_b_to_n;
  const auto& q3 = inertial_node(config, 3U).orientation_b_to_n;
  EXPECT_TRUE(std::abs(q1.w - 1.0) < 1.0e-12);
  EXPECT_TRUE(std::abs(q1.z) < 1.0e-12);
  EXPECT_TRUE(std::abs(q2.w - std::cos(0.25 * 3.14159265358979323846)) <
              1.0e-12);
  EXPECT_TRUE(std::abs(q2.z - std::sin(0.25 * 3.14159265358979323846)) <
              1.0e-12);
  EXPECT_TRUE(std::abs(q3.w - std::cos(-0.375 * 3.14159265358979323846)) <
              1.0e-12);
  EXPECT_TRUE(std::abs(q3.z - std::sin(-0.375 * 3.14159265358979323846)) <
              1.0e-12);
}

TEST_CASE(range_only_demo_ini_keeps_measurement_only_fallback) {
  // config：从测距回退配置加载的结果，期望不创建惯导配置且保留三节点与20 Hz输入率。
  const DemoConfig config = load_ini_config("config/range_only_demo.ini");
  EXPECT_TRUE(!config.inertial.has_value());
  EXPECT_EQ(config.engine.nodes.size(), 3U);
  EXPECT_EQ(config.online.input_rate_hz, 20.0);
}

// 失败路径按错误类别和原始行号验证，防止拼写/重复项被默认值静默掩盖。
TEST_CASE(ini_config_rejects_duplicate_sections_keys_and_nodes) {
  expect_ini_error(valid_ini() + "\n[engine]\n",
                   IniError::kDuplicateSection);
  expect_ini_error(
      replace_once(valid_ini(), "[engine]\n",
                   "[engine]\nedge_timeout_ns = 1\n"),
      IniError::kDuplicateKey);
  expect_ini_error(replace_once(valid_ini(), "[node.3]",
                                "[node.01]"),
                   IniError::kDuplicateNode);
}

TEST_CASE(ini_config_rejects_unknown_sections_keys_and_malformed_lines) {
  expect_ini_error("[mystery]\nvalue=1\n" + valid_ini(),
                   IniError::kUnknownSection);
  expect_ini_error(
      replace_once(valid_ini(), "[engine]\n",
                   "[engine]\nsecret_threshold = 7\n"),
      IniError::kUnknownKey);
  expect_ini_error("not-an-assignment\n" + valid_ini(),
                   IniError::kSyntax);
}

TEST_CASE(ini_config_rejects_bad_numbers_booleans_ports_and_non_finite_values) {
  expect_ini_error(replace_once(valid_ini(),
                                "edge_timeout_ns = 500000000",
                                "edge_timeout_ns = -1"),
                   IniError::kInvalidValue);
  expect_ini_error(replace_once(valid_ini(), "event_log_enabled = true",
                                "event_log_enabled = yes"),
                   IniError::kInvalidValue);
  expect_ini_error(replace_once(valid_ini(), "input_port = 39001",
                                "input_port = 0"),
                   IniError::kInvalidValue);
  expect_ini_error(replace_once(valid_ini(), "output_port = 39002",
                                "output_port = 65536"),
                   IniError::kInvalidValue);
  expect_ini_error(replace_once(valid_ini(), "nis_gate = 9.21",
                                "nis_gate = nan"),
                   IniError::kInvalidValue);
  expect_ini_error(replace_once(valid_ini(), "x = 3", "x = inf"),
                   IniError::kInvalidValue);

  // boundary：把输入端口改为65535后的边界配置，期望仍能成功解析。
  const DemoConfig boundary = parse_ini_config(replace_once(
      valid_ini(), "input_port = 39001", "input_port = 65535"));
  EXPECT_EQ(boundary.online.input_port, 65535U);
}

TEST_CASE(ini_config_rejects_missing_data_and_cross_field_invariants) {
  expect_ini_error(replace_once(valid_ini(), "nis_gate = 9.21\n", ""),
                   IniError::kMissingKey);
  expect_ini_error(replace_once(valid_ini(), "reference_node_id = 1",
                                "reference_node_id = 9"),
                   IniError::kMissingReferenceNode);
  expect_ini_error(replace_once(valid_ini(), "max_nodes = 64",
                                "max_nodes = 2"),
                   IniError::kInvalidConfiguration);
  expect_ini_error(replace_once(valid_ini(),
                                "reject_duration_ns = 3000000000",
                                "reject_duration_ns = 1"),
                   IniError::kInvalidConfiguration);
  expect_ini_error(replace_once(valid_ini(),
                                "max_payload_size = 1048576",
                                "max_payload_size = 1048577"),
                   IniError::kInvalidValue);
}

TEST_CASE(ini_config_rejects_unsafe_large_log_limit) {
  expect_ini_error(replace_once(valid_ini(),
                                "max_log_record_size = 1048616",
                                "max_log_record_size = 1048617"),
                   IniError::kInvalidConfiguration);
  // tighter：把日志记录上限收紧到100字节后的合法配置。
  const DemoConfig tighter = parse_ini_config(replace_once(
      valid_ini(), "max_log_record_size = 1048616",
      "max_log_record_size = 100"));
  EXPECT_EQ(tighter.online.max_log_record_size, 100U);
}

TEST_CASE(ini_config_tracked_edge_error_reports_the_offending_entry_line) {
  // too_few_tracked：把边跟踪容量降到不足值的输入；error：期望指向该配置项原始行的跨字段错误。
  const std::string too_few_tracked = replace_once(
      valid_ini(), "max_tracked_edges = 2016", "max_tracked_edges = 2");
  const CapturedIniError error = capture_ini_error(too_few_tracked);
  EXPECT_EQ(error.code, IniError::kInvalidConfiguration);
  EXPECT_EQ(error.line, line_of(too_few_tracked, "max_tracked_edges"));
}

TEST_CASE(ini_config_node_overflow_reports_the_offending_entry_line) {
  // overflowing_std：把节点位置标准差放大到平方溢出的输入；error：应定位到该节点字段行。
  const std::string overflowing_std = replace_once(
      valid_ini(), "position_std_m = 0.1", "position_std_m = 1e200");
  const CapturedIniError error = capture_ini_error(overflowing_std);
  EXPECT_EQ(error.code, IniError::kInvalidConfiguration);
  EXPECT_EQ(error.line, line_of(overflowing_std, "position_std_m = 1e200"));
}

TEST_CASE(ini_config_rejects_invalid_utf8_and_reports_its_line) {
  // invalid：在合法INI首字节前插入0xFF的输入缓冲，期望报告UTF-8错误而非继续解析。
  std::string invalid = valid_ini();
  invalid.insert(0U, 1U, static_cast<char>(0xFF));
  expect_ini_error(invalid, IniError::kInvalidUtf8);
}

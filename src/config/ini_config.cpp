// 模块实现：以UTF-8读取严格INI，检查节名/键名/重复项/类型范围和跨字段约束。
// 关键原则：未知键与非法编码直接失败，配置错误在算法、网络和日志资源创建前暴露；
// 解析器不静默采用拼写相近的键，也不把缺失参数替换成源码中的调试值。
#include "config/ini_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace zju::coop::config {
namespace {

constexpr std::size_t kMaximumConfigBytes = 1024U * 1024U;
constexpr std::size_t kMaximumPayloadBytes = 1024U * 1024U;
constexpr std::size_t kWireHeaderBytes = 40U;
constexpr std::size_t kMaximumDuplicateCachePerLink = 4096U;
constexpr std::size_t kMaximumNodes = 64U;
constexpr std::size_t kMaximumEdges = 2016U;
constexpr std::size_t kMaximumStateDimension = 252U;
constexpr std::size_t kMaximumTrackedEdges = 1'000'000U;

struct Entry {
  std::string value;
  std::size_t line{};
};

struct Section {
  std::string name;
  std::size_t line{};
  bool node{};
  std::uint32_t node_id{};
  std::map<std::string, Entry> entries;
};

struct ParsedIni {
  std::map<std::string, Section> sections;
  std::size_t final_line{1U};
};

struct NodeEntryLines {
  std::size_t x{};
  std::size_t y{};
  std::size_t vx{};
  std::size_t vy{};
  std::size_t position_std{};
  std::size_t velocity_std{};
};

[[noreturn]] void fail(IniError code, std::size_t line,
                       const std::string& detail) {
  throw IniConfigError(code, line, detail);
}

bool continuation(unsigned char byte) {
  return byte >= 0x80U && byte <= 0xBFU;
}

std::size_t invalid_utf8_line(const std::string& text) {
  // 逐字节校验UTF-8并返回首个错误行，避免中文配置在不同平台被错误解码。
  std::size_t line = 1U;
  std::size_t index = 0U;
  while (index < text.size()) {
    const unsigned char first =
        static_cast<unsigned char>(text[index]);
    if (first <= 0x7FU) {
      if (first == static_cast<unsigned char>('\n')) {
        ++line;
      }
      ++index;
      continue;
    }

    std::size_t count = 0U;
    if (first >= 0xC2U && first <= 0xDFU) {
      count = 2U;
    } else if (first >= 0xE0U && first <= 0xEFU) {
      count = 3U;
    } else if (first >= 0xF0U && first <= 0xF4U) {
      count = 4U;
    } else {
      return line;
    }
    if (index + count > text.size()) {
      return line;
    }
    for (std::size_t offset = 1U; offset < count; ++offset) {
      if (!continuation(
              static_cast<unsigned char>(text[index + offset]))) {
        return line;
      }
    }
    const unsigned char second =
        static_cast<unsigned char>(text[index + 1U]);
    if ((first == 0xE0U && second < 0xA0U) ||
        (first == 0xEDU && second > 0x9FU) ||
        (first == 0xF0U && second < 0x90U) ||
        (first == 0xF4U && second > 0x8FU)) {
      return line;
    }
    index += count;
  }
  return 0U;
}

bool ascii_space(char character) {
  return character == ' ' || character == '\t' || character == '\r' ||
         character == '\n';
}

std::string trim(const std::string& text) {
  std::size_t first = 0U;
  while (first < text.size() && ascii_space(text[first])) {
    ++first;
  }
  std::size_t last = text.size();
  while (last > first && ascii_space(text[last - 1U])) {
    --last;
  }
  return text.substr(first, last - first);
}

bool ascii_key(const std::string& key) {
  if (key.empty()) {
    return false;
  }
  for (const char character : key) {
    const bool lower = character >= 'a' && character <= 'z';
    const bool digit = character >= '0' && character <= '9';
    if (!lower && !digit && character != '_') {
      return false;
    }
  }
  return true;
}

bool one_of(const std::string& value,
            std::initializer_list<const char*> candidates) {
  for (const char* candidate : candidates) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

bool parse_node_section(const std::string& name, std::uint32_t& node_id) {
  constexpr const char* prefix = "node.";
  constexpr std::size_t prefix_size = 5U;
  if (name.size() <= prefix_size ||
      name.compare(0U, prefix_size, prefix) != 0) {
    return false;
  }
  std::uint32_t value = 0U;
  for (std::size_t index = prefix_size; index < name.size(); ++index) {
    const char character = name[index];
    if (character < '0' || character > '9') {
      return false;
    }
    const std::uint32_t digit =
        static_cast<std::uint32_t>(character - '0');
    if (value > (std::numeric_limits<std::uint16_t>::max() - digit) /
                    10U) {
      return false;
    }
    value = value * 10U + digit;
  }
  node_id = value;
  return true;
}

bool allowed_key(const Section& section, const std::string& key) {
  if (section.node) {
    return one_of(key, {"x", "y", "vx", "vy", "position_std_m",
                        "velocity_std_mps"});
  }
  if (section.name == "engine") {
    return one_of(key, {"edge_timeout_ns", "max_future_skew_ns",
                        "max_receive_delay_ns", "duplicate_cache_per_link",
                        "max_nodes", "max_edges", "max_state_dimension",
                        "rigidity_tolerance"});
  }
  if (section.name == "filter") {
    return one_of(key, {"reference_node_id", "process_accel_std_mps2",
                        "nis_gate", "max_prediction_step_s",
                        "min_covariance_diagonal"});
  }
  if (section.name == "degradation") {
    return one_of(key, {"window_ns", "nominal_rate_hz",
                        "nlos_ratio_threshold", "valid_ratio_threshold",
                        "rate_ratio_threshold",
                        "nlos_probability_threshold",
                        "nlos_covariance_scale", "suspend_duration_ns",
                        "reject_duration_ns", "recovery_duration_ns",
                        "max_tracked_edges"});
  }
  if (section.name == "online") {
    return one_of(key, {"input_bind_address", "input_port",
                        "output_address", "output_port", "input_rate_hz",
                        "output_rate_hz", "event_log_enabled",
                        "event_log_path", "max_payload_size",
                        "max_log_record_size"});
  }
  if (section.name == "inertial") {
    return one_of(
        key,
        {"enabled", "max_inertial_state_dimension", "gravity_mps2",
         "min_imu_dt_s", "max_imu_dt_s", "max_propagation_substep_s",
         "gyro_noise_density_rad_s_sqrt_hz",
         "accel_noise_density_m_s2_sqrt_hz",
         "gyro_bias_random_walk_rad_s2_sqrt_hz",
         "accel_bias_random_walk_m_s3_sqrt_hz",
         "min_covariance_diagonal", "quaternion_norm_tolerance",
         "covariance_symmetry_tolerance", "use_message_covariance",
         "use_orientation_for_initialization", "expected_frame_id",
         "initial_z_m", "initial_vz_mps", "attitude_std_rad",
         "gyro_bias_std_rad_s", "accel_bias_std_m_s2"});
  }
  return false;
}

ParsedIni parse_sections(const std::string& source) {
  // 阶段1：处理UTF-8/BOM并逐行识别节和键值，同时保留行号用于错误报告。
  const std::size_t invalid_line = invalid_utf8_line(source);
  if (invalid_line != 0U) {
    fail(IniError::kInvalidUtf8, invalid_line,
         "input is not valid UTF-8");
  }

  std::string text = source;
  if (text.size() >= 3U &&
      static_cast<unsigned char>(text[0U]) == 0xEFU &&
      static_cast<unsigned char>(text[1U]) == 0xBBU &&
      static_cast<unsigned char>(text[2U]) == 0xBFU) {
    text.erase(0U, 3U);
  }

  ParsedIni parsed{};
  std::unordered_map<std::uint32_t, std::size_t> node_lines;
  Section* current = nullptr;
  std::istringstream input(text);
  std::string raw_line;
  std::size_t line_number = 0U;
  while (std::getline(input, raw_line)) {
    ++line_number;
    const std::size_t hash = raw_line.find('#');
    const std::size_t semicolon = raw_line.find(';');
    std::size_t comment = std::string::npos;
    if (hash != std::string::npos) {
      comment = hash;
    }
    if (semicolon != std::string::npos) {
      comment = comment == std::string::npos ? semicolon
                                             : std::min(comment, semicolon);
    }
    if (comment != std::string::npos) {
      raw_line.erase(comment);
    }
    const std::string line = trim(raw_line);
    if (line.empty()) {
      continue;
    }

    if (line.front() == '[') {
      if (line.size() < 3U || line.back() != ']') {
        fail(IniError::kSyntax, line_number,
             "section declaration is malformed");
      }
      const std::string section_name = trim(line.substr(1U, line.size() - 2U));
      if (parsed.sections.find(section_name) != parsed.sections.end()) {
        fail(IniError::kDuplicateSection, line_number,
             "section '" + section_name + "' is duplicated");
      }
      Section section{};
      section.name = section_name;
      section.line = line_number;
      if (!one_of(section_name,
                  {"engine", "filter", "degradation", "online",
                   "inertial"})) {
        std::uint32_t node_id = 0U;
        if (!parse_node_section(section_name, node_id)) {
          fail(IniError::kUnknownSection, line_number,
               "section '" + section_name + "' is unknown");
        }
        if (node_lines.find(node_id) != node_lines.end()) {
          fail(IniError::kDuplicateNode, line_number,
               "node identifier is duplicated");
        }
        node_lines.emplace(node_id, line_number);
        section.node = true;
        section.node_id = node_id;
      }
      auto insertion =
          parsed.sections.emplace(section_name, std::move(section));
      current = &insertion.first->second;
      continue;
    }

    if (current == nullptr) {
      fail(IniError::kSyntax, line_number,
           "assignment appears before any section");
    }
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) {
      fail(IniError::kSyntax, line_number,
           "configuration line is not an assignment");
    }
    const std::string key = trim(line.substr(0U, equals));
    const std::string value = trim(line.substr(equals + 1U));
    if (!ascii_key(key)) {
      fail(IniError::kSyntax, line_number,
           "configuration key must be lowercase ASCII");
    }
    // 白名单拒绝未知键，防止拼写错误导致参数看似生效、实际使用默认值。
    if (!allowed_key(*current, key)) {
      fail(IniError::kUnknownKey, line_number,
           "key '" + key + "' is unknown in section '" +
               current->name + "'");
    }
    if (current->entries.find(key) != current->entries.end()) {
      fail(IniError::kDuplicateKey, line_number,
           "key '" + key + "' is duplicated");
    }
    current->entries.emplace(key, Entry{value, line_number});
  }
  parsed.final_line = std::max<std::size_t>(1U, line_number);
  return parsed;
}

const Section& require_section(const ParsedIni& parsed,
                               const std::string& name) {
  const auto found = parsed.sections.find(name);
  if (found == parsed.sections.end()) {
    fail(IniError::kMissingSection, parsed.final_line,
         "required section '" + name + "' is missing");
  }
  return found->second;
}

const Entry& require_entry(const Section& section, const std::string& key) {
  const auto found = section.entries.find(key);
  if (found == section.entries.end()) {
    fail(IniError::kMissingKey, section.line,
         "required key '" + key + "' is missing from section '" +
             section.name + "'");
  }
  return found->second;
}

std::uint64_t unsigned_value(const Entry& entry, std::uint64_t minimum,
                             std::uint64_t maximum,
                             const char* description) {
  if (entry.value.empty()) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " is empty");
  }
  std::uint64_t value = 0U;
  for (const char character : entry.value) {
    if (character < '0' || character > '9') {
      fail(IniError::kInvalidValue, entry.line,
           std::string(description) + " is not an unsigned integer");
    }
    const std::uint64_t digit =
        static_cast<std::uint64_t>(character - '0');
    if (value > (std::numeric_limits<std::uint64_t>::max() - digit) /
                    10U) {
      fail(IniError::kInvalidValue, entry.line,
           std::string(description) + " overflows");
    }
    value = value * 10U + digit;
  }
  if (value < minimum || value > maximum) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " is out of range");
  }
  return value;
}

std::size_t size_value(const Entry& entry, std::size_t minimum,
                       std::size_t maximum, const char* description) {
  return static_cast<std::size_t>(unsigned_value(
      entry, static_cast<std::uint64_t>(minimum),
      static_cast<std::uint64_t>(maximum), description));
}

double finite_value(const Entry& entry, const char* description) {
  std::istringstream input(entry.value);
  input.imbue(std::locale::classic());
  double value = 0.0;
  input >> value;
  if (!input || !input.eof() || !std::isfinite(value)) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " must be finite");
  }
  return value;
}

double positive_value(const Entry& entry, const char* description,
                      bool allow_zero = false) {
  const double value = finite_value(entry, description);
  if (allow_zero ? value < 0.0 : value <= 0.0) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " is out of range");
  }
  return value;
}

double unit_value(const Entry& entry, const char* description) {
  const double value = finite_value(entry, description);
  if (value < 0.0 || value > 1.0) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " must be in [0,1]");
  }
  return value;
}

bool boolean_value(const Entry& entry, const char* description) {
  if (entry.value == "true" || entry.value == "1") {
    return true;
  }
  if (entry.value == "false" || entry.value == "0") {
    return false;
  }
  fail(IniError::kInvalidValue, entry.line,
       std::string(description) + " must be true, false, 0, or 1");
}

std::string nonempty_value(const Entry& entry, const char* description) {
  if (entry.value.empty()) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " must not be empty");
  }
  return entry.value;
}

void validate_engine_config(const DemoConfig& config,
                            const Section& engine_section,
                            const Section& filter_section,
                            const Section& degradation_section,
                            const Section& online_section,
                            const std::unordered_map<std::uint32_t,
                                                     NodeEntryLines>&
                                node_entry_lines) {
  const std::size_t node_count = config.engine.nodes.size();
  const Entry& reference_entry =
      require_entry(filter_section, "reference_node_id");
  const auto reference = std::find_if(
      config.engine.nodes.begin(), config.engine.nodes.end(),
      [&config](const NodeInitialization& node) {
        return node.node_id == config.engine.filter.reference_node_id;
      });
  if (reference == config.engine.nodes.end()) {
    fail(IniError::kMissingReferenceNode, reference_entry.line,
         "reference node has no initialization section");
  }

  if (node_count == 0U || node_count > config.engine.max_nodes) {
    fail(IniError::kInvalidConfiguration,
         require_entry(engine_section, "max_nodes").line,
         "node count exceeds configured limit");
  }
  const std::size_t nonreference_count = node_count - 1U;
  if (nonreference_count > config.engine.max_state_dimension / 4U) {
    fail(IniError::kInvalidConfiguration,
         require_entry(engine_section, "max_state_dimension").line,
         "filter state dimension exceeds configured limit");
  }
  const std::size_t edge_count = node_count * (node_count - 1U) / 2U;
  if (edge_count > config.engine.max_edges) {
    fail(IniError::kInvalidConfiguration,
         require_entry(engine_section, "max_edges").line,
         "candidate edge count exceeds configured limit");
  }
  if (edge_count > config.engine.degradation.max_tracked_edges) {
    fail(IniError::kInvalidConfiguration,
         require_entry(degradation_section, "max_tracked_edges").line,
         "candidate edge count exceeds tracked-edge limit");
  }
  if (config.engine.degradation.reject_duration_ns <
      config.engine.degradation.suspend_duration_ns) {
    fail(IniError::kInvalidConfiguration,
         require_entry(degradation_section, "reject_duration_ns").line,
         "reject duration must not be shorter than suspend duration");
  }

  const double process_variance =
      config.engine.filter.process_accel_std_mps2 *
      config.engine.filter.process_accel_std_mps2;
  if (!std::isfinite(process_variance) || process_variance <= 0.0) {
    fail(IniError::kInvalidConfiguration,
         require_entry(filter_section, "process_accel_std_mps2").line,
         "process acceleration variance is not representable");
  }
  for (const NodeInitialization& node : config.engine.nodes) {
    const auto lines = node_entry_lines.find(node.node_id);
    if (lines == node_entry_lines.end()) {
      fail(IniError::kInvalidConfiguration, reference_entry.line,
           "node source metadata is missing");
    }
    const double position_variance = node.position_std_m * node.position_std_m;
    const double velocity_variance =
        node.velocity_std_mps * node.velocity_std_mps;
    if (!std::isfinite(position_variance)) {
      fail(IniError::kInvalidConfiguration, lines->second.position_std,
           "node position variance is not representable");
    }
    if (!std::isfinite(velocity_variance)) {
      fail(IniError::kInvalidConfiguration, lines->second.velocity_std,
           "node velocity variance is not representable");
    }
    if (!std::isfinite(node.x - reference->x)) {
      fail(IniError::kInvalidConfiguration, lines->second.x,
           "node relative x coordinate is not representable");
    }
    if (!std::isfinite(node.y - reference->y)) {
      fail(IniError::kInvalidConfiguration, lines->second.y,
           "node relative y coordinate is not representable");
    }
    if (!std::isfinite(node.vx - reference->vx)) {
      fail(IniError::kInvalidConfiguration, lines->second.vx,
           "node relative x velocity is not representable");
    }
    if (!std::isfinite(node.vy - reference->vy)) {
      fail(IniError::kInvalidConfiguration, lines->second.vy,
           "node relative y velocity is not representable");
    }
  }

  const long double expected =
      static_cast<long double>(config.engine.degradation.nominal_rate_hz) *
      static_cast<long double>(config.engine.degradation.window_ns) /
      1.0e9L;
  constexpr long double maximum_expected =
      static_cast<long double>((1'000'000U - 16U) / 8U);
  if (!std::isfinite(expected) || std::floor(expected + 0.5L) < 1.0L ||
      std::floor(expected + 0.5L) > maximum_expected) {
    fail(IniError::kInvalidConfiguration,
         require_entry(degradation_section, "nominal_rate_hz").line,
         "degradation window sample count is not representable");
  }

  if (config.online.max_log_record_size >
      kWireHeaderBytes + config.online.max_payload_size) {
    fail(IniError::kInvalidConfiguration,
         require_entry(online_section, "max_log_record_size").line,
         "log record limit exceeds the largest allowed wire frame");
  }
}

}  // namespace

IniConfigError::IniConfigError(IniError code, std::size_t line,
                               std::string message)
    : std::runtime_error("line " + std::to_string(std::max<std::size_t>(1U, line)) +
                         ": " + message),
      code_(code),
      line_(std::max<std::size_t>(1U, line)) {}

IniError IniConfigError::code() const noexcept { return code_; }

std::size_t IniConfigError::line() const noexcept { return line_; }

DemoConfig parse_ini_config(const std::string& text) {
  // 阶段2：按功能节把字符串转换为强类型配置，缺失或越界值立即抛出。
  const ParsedIni parsed = parse_sections(text);
  const Section& engine_section = require_section(parsed, "engine");
  const Section& filter_section = require_section(parsed, "filter");
  const Section& degradation_section =
      require_section(parsed, "degradation");
  const Section& online_section = require_section(parsed, "online");
  const auto inertial_section_entry = parsed.sections.find("inertial");

  DemoConfig config{};
  std::unordered_map<std::uint32_t, NodeEntryLines> node_entry_lines;
  config.engine.edge_timeout_ns = unsigned_value(
      require_entry(engine_section, "edge_timeout_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "edge timeout");
  config.engine.max_future_skew_ns = unsigned_value(
      require_entry(engine_section, "max_future_skew_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "maximum future skew");
  config.engine.max_receive_delay_ns = unsigned_value(
      require_entry(engine_section, "max_receive_delay_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "maximum receive delay");
  config.engine.duplicate_cache_per_link = size_value(
      require_entry(engine_section, "duplicate_cache_per_link"), 1U,
      kMaximumDuplicateCachePerLink, "duplicate cache per link");
  config.engine.max_nodes = size_value(
      require_entry(engine_section, "max_nodes"), 1U, kMaximumNodes,
      "maximum node count");
  config.engine.max_edges = size_value(
      require_entry(engine_section, "max_edges"), 1U, kMaximumEdges,
      "maximum edge count");
  config.engine.max_state_dimension = size_value(
      require_entry(engine_section, "max_state_dimension"), 1U,
      kMaximumStateDimension, "maximum state dimension");
  config.engine.rigidity_tolerance = positive_value(
      require_entry(engine_section, "rigidity_tolerance"),
      "rigidity tolerance");

  config.engine.filter.reference_node_id = static_cast<std::uint32_t>(
      unsigned_value(require_entry(filter_section, "reference_node_id"),
                     0U, std::numeric_limits<std::uint16_t>::max(),
                     "reference node identifier"));
  config.engine.filter.process_accel_std_mps2 = positive_value(
      require_entry(filter_section, "process_accel_std_mps2"),
      "process acceleration standard deviation");
  config.engine.filter.nis_gate = positive_value(
      require_entry(filter_section, "nis_gate"), "NIS gate", true);
  config.engine.filter.max_prediction_step_s = positive_value(
      require_entry(filter_section, "max_prediction_step_s"),
      "maximum prediction step");
  config.engine.filter.min_covariance_diagonal = positive_value(
      require_entry(filter_section, "min_covariance_diagonal"),
      "minimum covariance diagonal");

  config.engine.degradation.window_ns = unsigned_value(
      require_entry(degradation_section, "window_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "degradation window");
  config.engine.degradation.nominal_rate_hz = positive_value(
      require_entry(degradation_section, "nominal_rate_hz"),
      "nominal observation rate");
  config.engine.degradation.nlos_ratio_threshold = unit_value(
      require_entry(degradation_section, "nlos_ratio_threshold"),
      "NLOS ratio threshold");
  config.engine.degradation.valid_ratio_threshold = unit_value(
      require_entry(degradation_section, "valid_ratio_threshold"),
      "valid ratio threshold");
  config.engine.degradation.rate_ratio_threshold = unit_value(
      require_entry(degradation_section, "rate_ratio_threshold"),
      "rate ratio threshold");
  config.engine.degradation.nlos_probability_threshold = unit_value(
      require_entry(degradation_section, "nlos_probability_threshold"),
      "NLOS probability threshold");
  config.engine.degradation.nlos_covariance_scale = positive_value(
      require_entry(degradation_section, "nlos_covariance_scale"),
      "NLOS covariance scale");
  if (config.engine.degradation.nlos_covariance_scale < 1.0) {
    fail(IniError::kInvalidValue,
         require_entry(degradation_section, "nlos_covariance_scale").line,
         "NLOS covariance scale must be at least one");
  }
  config.engine.degradation.suspend_duration_ns = unsigned_value(
      require_entry(degradation_section, "suspend_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "suspend duration");
  config.engine.degradation.reject_duration_ns = unsigned_value(
      require_entry(degradation_section, "reject_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "reject duration");
  config.engine.degradation.recovery_duration_ns = unsigned_value(
      require_entry(degradation_section, "recovery_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "recovery duration");
  config.engine.degradation.max_tracked_edges = size_value(
      require_entry(degradation_section, "max_tracked_edges"), 1U,
      kMaximumTrackedEdges, "maximum tracked edges");

  config.online.input_bind_address = nonempty_value(
      require_entry(online_section, "input_bind_address"),
      "input bind address");
  config.online.input_port = static_cast<std::uint16_t>(unsigned_value(
      require_entry(online_section, "input_port"), 1U,
      std::numeric_limits<std::uint16_t>::max(), "input UDP port"));
  config.online.output_address = nonempty_value(
      require_entry(online_section, "output_address"), "output address");
  config.online.output_port = static_cast<std::uint16_t>(unsigned_value(
      require_entry(online_section, "output_port"), 1U,
      std::numeric_limits<std::uint16_t>::max(), "output UDP port"));
  config.online.input_rate_hz = positive_value(
      require_entry(online_section, "input_rate_hz"), "input rate");
  config.online.output_rate_hz = positive_value(
      require_entry(online_section, "output_rate_hz"), "output rate");
  config.online.event_log_enabled = boolean_value(
      require_entry(online_section, "event_log_enabled"),
      "event log enabled");
  config.online.event_log_path = nonempty_value(
      require_entry(online_section, "event_log_path"), "event log path");
  config.online.max_payload_size = size_value(
      require_entry(online_section, "max_payload_size"), 1U,
      kMaximumPayloadBytes, "maximum payload size");
  config.online.max_log_record_size = size_value(
      require_entry(online_section, "max_log_record_size"),
      kWireHeaderBytes, static_cast<std::size_t>(
                            std::numeric_limits<std::uint32_t>::max()),
      "maximum log record size");

  for (const auto& section_entry : parsed.sections) {
    const Section& section = section_entry.second;
    if (!section.node) {
      continue;
    }
    NodeInitialization node{};
    node.node_id = section.node_id;
    const Entry& x_entry = require_entry(section, "x");
    const Entry& y_entry = require_entry(section, "y");
    const Entry& vx_entry = require_entry(section, "vx");
    const Entry& vy_entry = require_entry(section, "vy");
    const Entry& position_std_entry =
        require_entry(section, "position_std_m");
    const Entry& velocity_std_entry =
        require_entry(section, "velocity_std_mps");
    node.x = finite_value(x_entry, "node x");
    node.y = finite_value(y_entry, "node y");
    node.vx = finite_value(vx_entry, "node vx");
    node.vy = finite_value(vy_entry, "node vy");
    node.position_std_m = positive_value(
        position_std_entry, "node position standard deviation");
    node.velocity_std_mps = positive_value(
        velocity_std_entry, "node velocity standard deviation");
    node_entry_lines.emplace(
        node.node_id,
        NodeEntryLines{x_entry.line, y_entry.line, vx_entry.line,
                       vy_entry.line, position_std_entry.line,
                       velocity_std_entry.line});
    config.engine.nodes.push_back(node);
  }
  std::sort(config.engine.nodes.begin(), config.engine.nodes.end(),
            [](const NodeInitialization& left,
               const NodeInitialization& right) {
              return left.node_id < right.node_id;
            });

  if (inertial_section_entry != parsed.sections.end()) {
    const Section& section = inertial_section_entry->second;
    const bool enabled = boolean_value(require_entry(section, "enabled"),
                                       "inertial enabled");
    if (enabled) {
      InertialDemoConfig inertial{};
      inertial.max_inertial_state_dimension = size_value(
          require_entry(section, "max_inertial_state_dimension"), 15U,
          15U * kMaximumNodes, "maximum inertial state dimension");
      inertial.filter.gravity_mps2 = positive_value(
          require_entry(section, "gravity_mps2"), "gravity magnitude");
      inertial.filter.min_imu_dt_s = positive_value(
          require_entry(section, "min_imu_dt_s"), "minimum IMU interval");
      inertial.filter.max_imu_dt_s = positive_value(
          require_entry(section, "max_imu_dt_s"), "maximum IMU interval");
      inertial.filter.max_propagation_substep_s = positive_value(
          require_entry(section, "max_propagation_substep_s"),
          "maximum IMU propagation substep");
      inertial.filter.gyro_noise_density_rad_s_sqrt_hz = positive_value(
          require_entry(section, "gyro_noise_density_rad_s_sqrt_hz"),
          "gyro noise density");
      inertial.filter.accel_noise_density_m_s2_sqrt_hz = positive_value(
          require_entry(section, "accel_noise_density_m_s2_sqrt_hz"),
          "accelerometer noise density");
      inertial.filter.gyro_bias_random_walk_rad_s2_sqrt_hz = positive_value(
          require_entry(section, "gyro_bias_random_walk_rad_s2_sqrt_hz"),
          "gyro bias random walk");
      inertial.filter.accel_bias_random_walk_m_s3_sqrt_hz = positive_value(
          require_entry(section, "accel_bias_random_walk_m_s3_sqrt_hz"),
          "accelerometer bias random walk");
      inertial.filter.min_covariance_diagonal = positive_value(
          require_entry(section, "min_covariance_diagonal"),
          "inertial covariance floor");
      inertial.filter.quaternion_norm_tolerance = positive_value(
          require_entry(section, "quaternion_norm_tolerance"),
          "quaternion norm tolerance");
      inertial.filter.covariance_symmetry_tolerance = positive_value(
          require_entry(section, "covariance_symmetry_tolerance"),
          "covariance symmetry tolerance");
      inertial.filter.use_message_covariance = boolean_value(
          require_entry(section, "use_message_covariance"),
          "use message covariance");
      inertial.filter.use_orientation_for_initialization = boolean_value(
          require_entry(section, "use_orientation_for_initialization"),
          "use orientation for initialization");
      inertial.filter.expected_frame_id = nonempty_value(
          require_entry(section, "expected_frame_id"), "IMU frame id");
      if (inertial.filter.expected_frame_id.size() >= 32U ||
          inertial.filter.expected_frame_id.find('\0') != std::string::npos) {
        fail(IniError::kInvalidValue,
             require_entry(section, "expected_frame_id").line,
             "IMU frame id must fit in 31 bytes");
      }
      const double initial_z = finite_value(
          require_entry(section, "initial_z_m"), "initial z");
      const double initial_vz = finite_value(
          require_entry(section, "initial_vz_mps"), "initial vertical velocity");
      const double attitude_std = positive_value(
          require_entry(section, "attitude_std_rad"), "attitude standard deviation");
      const double gyro_bias_std = positive_value(
          require_entry(section, "gyro_bias_std_rad_s"), "gyro bias standard deviation");
      const double accel_bias_std = positive_value(
          require_entry(section, "accel_bias_std_m_s2"), "accelerometer bias standard deviation");
      if (inertial.filter.min_imu_dt_s > inertial.filter.max_imu_dt_s ||
          inertial.filter.max_propagation_substep_s >
              inertial.filter.max_imu_dt_s ||
          config.engine.nodes.size() >
              inertial.max_inertial_state_dimension / 15U) {
        fail(IniError::kInvalidConfiguration, section.line,
             "inertial time or resource limits are inconsistent");
      }
      inertial.nodes.reserve(config.engine.nodes.size());
      for (const auto& source : config.engine.nodes) {
        InertialNodeInitialization node{};
        node.node_id = source.node_id;
        node.position_n_m = {source.x, source.y, initial_z};
        node.velocity_n_mps = {source.vx, source.vy, initial_vz};
        node.position_std_m = {source.position_std_m, source.position_std_m,
                               source.position_std_m};
        node.velocity_std_mps = {source.velocity_std_mps,
                                 source.velocity_std_mps,
                                 source.velocity_std_mps};
        node.attitude_std_rad = {attitude_std, attitude_std, attitude_std};
        node.gyro_bias_std_rad_s = {gyro_bias_std, gyro_bias_std,
                                    gyro_bias_std};
        node.accel_bias_std_m_s2 = {accel_bias_std, accel_bias_std,
                                    accel_bias_std};
        inertial.nodes.push_back(node);
      }
      config.inertial = std::move(inertial);
    }
  }

  // 阶段3：单字段解析后再检查节点、边、状态维数和时间门限的组合约束。
  validate_engine_config(config, engine_section, filter_section,
                         degradation_section, online_section,
                         node_entry_lines);
  return config;
}

DemoConfig load_ini_config(const std::filesystem::path& path) {
  // 文件按二进制读取，编码、换行和BOM统一交给parse_ini_config处理。
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    fail(IniError::kIoFailure, 1U,
         "configuration file could not be opened");
  }
  std::string text;
  char character = 0;
  while (input.get(character)) {
    if (text.size() >= kMaximumConfigBytes) {
      fail(IniError::kInvalidConfiguration, 1U,
           "configuration file exceeds size limit");
    }
    text.push_back(character);
  }
  if (!input.eof()) {
    fail(IniError::kIoFailure, 1U,
         "configuration file could not be read");
  }
  return parse_ini_config(text);
}

}  // namespace zju::coop::config

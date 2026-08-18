// 模块实现：以UTF-8读取严格INI，检查节名/键名/重复项/类型范围和跨字段约束。
// 关键原则：未知键与非法编码直接失败，配置错误在算法、网络和日志资源创建前暴露；
// 解析器不静默采用拼写相近的键，也不把缺失参数替换成源码中的调试值。
//
// C++初学者建议按“读取文本 -> 去空白/注释 -> 识别[节]和key=value
// -> 把字符串转成数值 -> 检查范围及跨字段关系 -> 生成AppConfig”的顺序阅读。
// 匿名namespace中的小函数只供本.cpp使用，避免把解析细节暴露给其他模块。
//
// 本文件常见难点语法：
// - `std::map`保存有序键值对，`unordered_map`用哈希快速查找；
// - 迭代器表示容器中的位置，`end()`是未找到/越过末尾的哨兵，不能解引用；
// - Lambda是现场定义的匿名函数，`[&config]`表示按引用使用外部config；
// - `[[noreturn]]`告诉编译器fail必定抛异常而不会正常返回；
// - `std::optional`明确表示惯性配置可能缺失，不用空指针或特殊整数暗示。
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

// kMaximumConfigBytes限制INI文件；kMaximumPayloadBytes限制ZJCL载荷；kWireHeaderBytes为协议固定头长，单位均为字节。
constexpr std::size_t kMaximumConfigBytes = 1024U * 1024U;
constexpr std::size_t kMaximumPayloadBytes = 1024U * 1024U;
constexpr std::size_t kWireHeaderBytes = 40U;
// 下列上限分别约束单链去重缓存、节点数、完全图边数、4×非参考节点状态维数和质量边状态数。
constexpr std::size_t kMaximumDuplicateCachePerLink = 4096U;
constexpr std::size_t kMaximumNodes = 64U;
constexpr std::size_t kMaximumEdges = 2016U;
constexpr std::size_t kMaximumStateDimension = 252U;
constexpr std::size_t kMaximumTrackedEdges = 1'000'000U;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct Entry {
  std::string value;  // 去除键名与等号两侧ASCII空白后的原始值文本。
  std::size_t line{}; // 该赋值在原始INI中的1起始行号。
};

struct Section {
  std::string name;   // 方括号内规范化后的节名。
  std::size_t line{}; // 节声明在原始INI中的1起始行号。
  bool node{};        // true表示名称匹配node.<id>动态节点节。
  std::uint32_t node_id{}; // node=true时解析出的平台业务编号。
  std::map<std::string, Entry> entries; // 本节按键名索引的值与来源行，解析结果独占。
};

struct ParsedIni {
  std::map<std::string, Section> sections; // 完整INI按唯一节名索引的所有节。
  std::size_t final_line{1U}; // 至少为1的末行号，用于缺失节这类无直接行号错误。
};

struct NodeEntryLines {
  // x/y/vx/vy分别记录对应平面初值键的1起始行；position_std/velocity_std记录两类1σ键行号。
  std::size_t x{};
  std::size_t y{};
  std::size_t vx{};
  std::size_t vy{};
  std::size_t position_std{};
  std::size_t velocity_std{};
};

// code为机器错误类别，line为原始INI行号，detail为包含节/键语境的诊断文本。
[[noreturn]] void fail(IniError code, std::size_t line,
                       const std::string& detail) {
  // throw构造并抛出项目异常；[[noreturn]]保证调用点之后无需再写return。
  throw IniConfigError(code, line, detail);
}

// byte为待判断是否处于0x80..0xBF范围的UTF-8后续字节。
bool continuation(unsigned char byte) {
  // UTF-8后续字节固定为二进制10xxxxxx，对应十六进制闭区间80..BF。
  return byte >= 0x80U && byte <= 0xBFU;
}

// text是完整配置字节串；line记录1起始来源行，index按码点推进，返回0表示全部合法。
std::size_t invalid_utf8_line(const std::string& text) {
  std::size_t line = 1U;
  std::size_t index = 0U;
  while (index < text.size()) {
    // first识别当前码点类别并在ASCII换行时推进line；count随后保存该码点总字节数。
    const unsigned char first =
        // char在某些平台是有符号类型，显式转unsigned char避免0x80以上字节变成负数。
        static_cast<unsigned char>(text[index]);
    if (first <= 0x7FU) {
      if (first == static_cast<unsigned char>('\n')) {
        ++line;
      }
      ++index;
      continue;  // ASCII已完整处理，直接进入while下一字节。
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
    // offset遍历当前多字节码点中首字节之后的每个后续字节。
    for (std::size_t offset = 1U; offset < count; ++offset) {
      if (!continuation(
              static_cast<unsigned char>(text[index + offset]))) {
        return line;
      }
    }
    // second与first联合拒绝过长编码、UTF-16代理项和超出U+10FFFF的组合。
    const unsigned char second =
        static_cast<unsigned char>(text[index + 1U]);
    if ((first == 0xE0U && second < 0xA0U) ||
        (first == 0xEDU && second > 0x9FU) ||
        (first == 0xF0U && second < 0x90U) ||
        (first == 0xF4U && second > 0x8FU)) {
      return line;
    }
    index += count;  // 当前码点验证完毕，跳过其全部1/2/3/4字节。
  }
  return 0U;  // 0不是有效源文件行号，在此专门表示整串UTF-8合法。
}

// character为待判定的INI修剪字符，仅接受四种ASCII空白。
bool ascii_space(char character) {
  return character == ' ' || character == '\t' || character == '\r' ||
         character == '\n';
}

// text是待修剪片段；first/last夹定首尾ASCII空白后的半开区间，返回独立副本。
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

// key是等号左侧修剪后的键名；character逐字节遍历，lower/digit标识允许的字母和数字类别。
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

// value是待匹配文本；candidate遍历candidates中生命周期覆盖本次调用的白名单字面量。
bool one_of(const std::string& value,
            std::initializer_list<const char*> candidates) {
  for (const char* candidate : candidates) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

// name是节名；prefix/prefix_size限定“node.”前缀，node_id仅在完整解析uint16十进制编号后写入。
bool parse_node_section(const std::string& name, std::uint32_t& node_id) {
  constexpr const char* prefix = "node.";
  constexpr std::size_t prefix_size = 5U;
  if (name.size() <= prefix_size ||
      name.compare(0U, prefix_size, prefix) != 0) {
    return false;
  }
  // value是十进制累加器；index遍历前缀后的字符，character/digit提供乘加前的数字与溢出检查。
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

// section提供当前节类别，key为待验证键名；仅白名单项可继续解析。
bool allowed_key(const Section& section, const std::string& key) {
  if (section.node) {
    return one_of(key, {"x", "y", "vx", "vy", "position_std_m",
                        "velocity_std_mps", "initial_yaw_rad"});
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

// source为调用方完整UTF-8配置文本；返回拥有全部节/值副本和原始行号的中间表示。
ParsedIni parse_sections(const std::string& source) {
  // 阶段1：处理UTF-8/BOM并逐行识别节和键值，同时保留行号用于错误报告。
  // invalid_line为首个UTF-8错误所在行，0表示编码合法。
  const std::size_t invalid_line = invalid_utf8_line(source);
  if (invalid_line != 0U) {
    fail(IniError::kInvalidUtf8, invalid_line,
         "input is not valid UTF-8");
  }

  // text为可移除UTF-8 BOM的私有副本，不修改调用方source。
  std::string text = source;
  if (text.size() >= 3U &&
      static_cast<unsigned char>(text[0U]) == 0xEFU &&
      static_cast<unsigned char>(text[1U]) == 0xBBU &&
      static_cast<unsigned char>(text[2U]) == 0xBFU) {
    text.erase(0U, 3U);
  }

  // parsed累计解析结果；node_lines记录node_id首次声明行以检测跨节重复。
  ParsedIni parsed{};
  std::unordered_map<std::uint32_t, std::size_t> node_lines;
  // current借用parsed中当前节，新增节前可为空；input逐行读取私有text副本。
  Section* current = nullptr;
  std::istringstream input(text);
  // raw_line接收含注释的当前行；line_number为原始文本1起始行号计数器。
  std::string raw_line;
  std::size_t line_number = 0U;
  while (std::getline(input, raw_line)) {
    ++line_number;
    // hash/semicolon为两种注释标记的首下标；comment选择其中最靠前者作为截断游标。
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
    // line为去注释并修剪后的有效语法行。
    const std::string line = trim(raw_line);
    if (line.empty()) {
      continue;
    }

    if (line.front() == '[') {
      if (line.size() < 3U || line.back() != ']') {
        fail(IniError::kSyntax, line_number,
             "section declaration is malformed");
      }
      // section_name为方括号内修剪后的唯一节名。
      const std::string section_name = trim(line.substr(1U, line.size() - 2U));
      if (parsed.sections.find(section_name) != parsed.sections.end()) {
        fail(IniError::kDuplicateSection, line_number,
             "section '" + section_name + "' is duplicated");
      }
      // section在完成类别/节点号校验后整体移入parsed，避免半成品可见。
      Section section{};
      section.name = section_name;
      section.line = line_number;
      if (!one_of(section_name,
                  {"engine", "filter", "degradation", "online",
                   "inertial"})) {
        // node_id接收node.<id>动态节解析出的平台编号。
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
      // insertion携带map中新节迭代器，current随后借用其稳定节点地址。
      auto insertion =
          parsed.sections.emplace(section_name, std::move(section));
      current = &insertion.first->second;
      continue;
    }

    if (current == nullptr) {
      fail(IniError::kSyntax, line_number,
           "assignment appears before any section");
    }
    // equals为当前赋值行首个等号的0起始下标。
    const std::size_t equals = line.find('=');
    if (equals == std::string::npos) {
      fail(IniError::kSyntax, line_number,
           "configuration line is not an assignment");
    }
    // key/value分别为等号两侧修剪后的键名和仍保持文本形式的值。
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

// parsed为只读中间表示，name为必需节名；返回引用与parsed同生命周期。
const Section& require_section(const ParsedIni& parsed,
                               const std::string& name) {
  // found为name在节索引中的查找位置。
  const auto found = parsed.sections.find(name);
  if (found == parsed.sections.end()) {
    fail(IniError::kMissingSection, parsed.final_line,
         "required section '" + name + "' is missing");
  }
  return found->second;
}

// section为只读节，key为必需键名；返回引用与section同生命周期。
const Entry& require_entry(const Section& section, const std::string& key) {
  // found为key在当前节键值索引中的查找位置。
  const auto found = section.entries.find(key);
  if (found == section.entries.end()) {
    fail(IniError::kMissingKey, section.line,
         "required key '" + key + "' is missing from section '" +
             section.name + "'");
  }
  return found->second;
}

// entry提供十进制文本与行号，minimum/maximum定义闭区间，description用于错误语境。
std::uint64_t unsigned_value(const Entry& entry, std::uint64_t minimum,
                             std::uint64_t maximum,
                             const char* description) {
  if (entry.value.empty()) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " is empty");
  }
  // 手工十进制累加能在乘法前检查溢出，也明确拒绝符号、空白和0x前缀。
  // value为十进制乘加累加器；character依次遍历每个必须为数字的值字节。
  std::uint64_t value = 0U;
  for (const char character : entry.value) {
    if (character < '0' || character > '9') {
      fail(IniError::kInvalidValue, entry.line,
           std::string(description) + " is not an unsigned integer");
    }
    // digit为当前字符的0..9数值，参与乘加前溢出检查。
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

// entry提供无符号文本，minimum/maximum为本平台size_t闭区间，description用于错误消息。
std::size_t size_value(const Entry& entry, std::size_t minimum,
                       std::size_t maximum, const char* description) {
  return static_cast<std::size_t>(unsigned_value(
      entry, static_cast<std::uint64_t>(minimum),
      static_cast<std::uint64_t>(maximum), description));
}

// entry提供待按classic locale解析的浮点文本，description标识字段业务名称。
double finite_value(const Entry& entry, const char* description) {
  // 固定classic locale，保证Windows和Ubuntu都用点作小数分隔符；eof检查
  // 拒绝“1.0abc”这类前缀可解析但尾部无效的现场配置。
  // input拥有值文本的解析流副本；value接收要求完整消费且有限的数值。
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

// entry/description同finite_value；allow_zero=true时闭区间从0开始，否则要求严格正值。
double positive_value(const Entry& entry, const char* description,
                      bool allow_zero = false) {
  // value为已排除NaN/Inf和尾随字符的候选物理量。
  const double value = finite_value(entry, description);
  if (allow_zero ? value < 0.0 : value <= 0.0) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " is out of range");
  }
  return value;
}

// entry为概率/比率文本，description用于错误语境；返回值必须位于[0,1]。
double unit_value(const Entry& entry, const char* description) {
  // value为已完成有限性检查的候选无量纲比率。
  const double value = finite_value(entry, description);
  if (value < 0.0 || value > 1.0) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " must be in [0,1]");
  }
  return value;
}

// entry为布尔文本，description用于错误语境；只接受true/false/1/0四种形式。
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

// entry为已修剪字符串值，description用于错误语境；返回独立非空副本。
std::string nonempty_value(const Entry& entry, const char* description) {
  if (entry.value.empty()) {
    fail(IniError::kInvalidValue, entry.line,
         std::string(description) + " must not be empty");
  }
  return entry.value;
}

// config为已完成单字段转换的候选配置；engine_section提供节点/边/状态维数上限键行号，
// filter_section提供参考节点和过程噪声键行号，degradation_section提供退化时长/边缓存/频率键行号，
// online_section提供ZJCL帧长度上限键行号；node_entry_lines提供各节点初值组合错误的原始行号。
void validate_engine_config(const DemoConfig& config,
                            const Section& engine_section,
                            const Section& filter_section,
                            const Section& degradation_section,
                            const Section& online_section,
                            const std::unordered_map<std::uint32_t,
                                                     NodeEntryLines>&
                                node_entry_lines) {
  // 单字段范围已在解析时检查；这里验证节点/边/状态维度、保持时间和记录大小
  // 等跨字段约束，避免每个值合法但组合后无法分配或语义矛盾。
  // node_count为配置平台总数；reference_entry提供参考节点键的原始行号。
  const std::size_t node_count = config.engine.nodes.size();
  const Entry& reference_entry =
      require_entry(filter_section, "reference_node_id");
  // reference为参考节点在已排序初值数组中的迭代器；lambda借用config并逐个检查node_id。
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
  // nonreference_count决定UWB-only每节点4维状态的块数。
  const std::size_t nonreference_count = node_count - 1U;
  if (nonreference_count > config.engine.max_state_dimension / 4U) {
    fail(IniError::kInvalidConfiguration,
         require_entry(engine_section, "max_state_dimension").line,
         "filter state dimension exceeds configured limit");
  }
  // edge_count为全部配置节点形成的完全无向图边数。
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

  // process_variance为过程加速度标准差process_accel_std_mps2的平方，用于确认过程方差可有限表示。
  const double process_variance =
      config.engine.filter.process_accel_std_mps2 *
      config.engine.filter.process_accel_std_mps2;
  if (!std::isfinite(process_variance) || process_variance <= 0.0) {
    fail(IniError::kInvalidConfiguration,
         require_entry(filter_section, "process_accel_std_mps2").line,
         "process acceleration variance is not representable");
  }
  // node依次引用每个平台初值，用来源行号检查平方和相对量的可表示性。
  for (const NodeInitialization& node : config.engine.nodes) {
    // lines为当前node_id的六个原始键行号查找结果。
    const auto lines = node_entry_lines.find(node.node_id);
    if (lines == node_entry_lines.end()) {
      fail(IniError::kInvalidConfiguration, reference_entry.line,
           "node source metadata is missing");
    }
    // position_variance/velocity_variance为1σ平方后的协方差对角初值。
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

  // expected为窗口内理论样本数；maximum_expected由单边质量历史的内存预算推导。
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
  // parsed拥有带来源行号的中间表示；四个section引用其必需功能节。
  const ParsedIni parsed = parse_sections(text);
  const Section& engine_section = require_section(parsed, "engine");
  const Section& filter_section = require_section(parsed, "filter");
  const Section& degradation_section =
      require_section(parsed, "degradation");
  const Section& online_section = require_section(parsed, "online");
  // inertial_section_entry标识可选惯性节是否存在，不存在时明确走UWB-only路径。
  const auto inertial_section_entry = parsed.sections.find("inertial");

  // config为逐字段构造并最终返回的强类型值；node_entry_lines保留节点组合校验的原始行号。
  DemoConfig config{};
  std::unordered_map<std::uint32_t, NodeEntryLines> node_entry_lines;
  // initial_yaw_by_node保存可选节点ENU航向；缺省为0，只用于构造惯性初始四元数。
  std::unordered_map<std::uint32_t, double> initial_yaw_by_node;
  // edge_timeout_ns决定多久没有新测距后把协同边从活动拓扑中移除。
  config.engine.edge_timeout_ns = unsigned_value(
      require_entry(engine_section, "edge_timeout_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "edge timeout");
  // max_future_skew_ns防止时钟错误让“未来消息”污染当前滤波状态。
  config.engine.max_future_skew_ns = unsigned_value(
      require_entry(engine_section, "max_future_skew_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "maximum future skew");
  // max_receive_delay_ns限制测量时刻到接收时刻的容许延迟。
  config.engine.max_receive_delay_ns = unsigned_value(
      require_entry(engine_section, "max_receive_delay_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "maximum receive delay");
  // duplicate_cache_per_link限制每条有向链路保存多少个历史sequence用于去重。
  config.engine.duplicate_cache_per_link = size_value(
      require_entry(engine_section, "duplicate_cache_per_link"), 1U,
      kMaximumDuplicateCachePerLink, "duplicate cache per link");
  // max_nodes给节点容器设置硬上限，防止错误配置造成无界内存分配。
  config.engine.max_nodes = size_value(
      require_entry(engine_section, "max_nodes"), 1U, kMaximumNodes,
      "maximum node count");
  // max_edges给无向协同边数量设置硬上限。
  config.engine.max_edges = size_value(
      require_entry(engine_section, "max_edges"), 1U, kMaximumEdges,
      "maximum edge count");
  // max_state_dimension限制二维滤波器4×(N-1)状态的总维数。
  config.engine.max_state_dimension = size_value(
      require_entry(engine_section, "max_state_dimension"), 1U,
      kMaximumStateDimension, "maximum state dimension");
  // rigidity_tolerance是几何秩判断时把极小数视为零的数值容差。
  config.engine.rigidity_tolerance = positive_value(
      require_entry(engine_section, "rigidity_tolerance"),
      "rigidity tolerance");

  // reference_node_id定义所有平面相对位置所使用的原点节点。
  config.engine.filter.reference_node_id = static_cast<std::uint32_t>(
      unsigned_value(require_entry(filter_section, "reference_node_id"),
                     0U, std::numeric_limits<std::uint16_t>::max(),
                     "reference node identifier"));
  // process_accel_std_mps2控制常速度模型允许多大的随机加速度变化。
  config.engine.filter.process_accel_std_mps2 = positive_value(
      require_entry(filter_section, "process_accel_std_mps2"),
      "process acceleration standard deviation");
  // nis_gate是创新归一化平方的统计门限，值越小越容易拒绝测距。
  config.engine.filter.nis_gate = positive_value(
      require_entry(filter_section, "nis_gate"), "NIS gate", true);
  // max_prediction_step_s把长时间预测拆成多个较小步骤，以提高离散传播稳定性。
  config.engine.filter.max_prediction_step_s = positive_value(
      require_entry(filter_section, "max_prediction_step_s"),
      "maximum prediction step");
  // min_covariance_diagonal防止协方差对角线因浮点误差降到零或负数。
  config.engine.filter.min_covariance_diagonal = positive_value(
      require_entry(filter_section, "min_covariance_diagonal"),
      "minimum covariance diagonal");

  // window_ns定义NLOS比例、有效率和实际频率统计所覆盖的时间窗。
  config.engine.degradation.window_ns = unsigned_value(
      require_entry(degradation_section, "window_ns"), 1U,
      std::numeric_limits<std::uint64_t>::max(), "degradation window");
  // nominal_rate_hz是判断掉频时使用的期望单边观测频率。
  config.engine.degradation.nominal_rate_hz = positive_value(
      require_entry(degradation_section, "nominal_rate_hz"),
      "nominal observation rate");
  // nlos_ratio_threshold规定窗口内NLOS比例超过多少算退化。
  config.engine.degradation.nlos_ratio_threshold = unit_value(
      require_entry(degradation_section, "nlos_ratio_threshold"),
      "NLOS ratio threshold");
  // valid_ratio_threshold规定窗口内有效包比例至少应达到多少。
  config.engine.degradation.valid_ratio_threshold = unit_value(
      require_entry(degradation_section, "valid_ratio_threshold"),
      "valid ratio threshold");
  // rate_ratio_threshold比较实际频率/名义频率，用于发现掉频。
  config.engine.degradation.rate_ratio_threshold = unit_value(
      require_entry(degradation_section, "rate_ratio_threshold"),
      "rate ratio threshold");
  // nlos_probability_threshold把单包概率转换为是否计入NLOS的布尔判断。
  config.engine.degradation.nlos_probability_threshold = unit_value(
      require_entry(degradation_section, "nlos_probability_threshold"),
      "NLOS probability threshold");
  // nlos_covariance_scale在量测仍可用但可疑时放大方差，从而降低滤波权重。
  config.engine.degradation.nlos_covariance_scale = positive_value(
      require_entry(degradation_section, "nlos_covariance_scale"),
      "NLOS covariance scale");
  if (config.engine.degradation.nlos_covariance_scale < 1.0) {
    fail(IniError::kInvalidValue,
         require_entry(degradation_section, "nlos_covariance_scale").line,
         "NLOS covariance scale must be at least one");
  }
  // suspend_duration_ns规定退化持续多久后进入暂缓状态。
  config.engine.degradation.suspend_duration_ns = unsigned_value(
      require_entry(degradation_section, "suspend_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "suspend duration");
  // reject_duration_ns规定严重退化持续多久后完全拒绝该边。
  config.engine.degradation.reject_duration_ns = unsigned_value(
      require_entry(degradation_section, "reject_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "reject duration");
  // recovery_duration_ns要求连续正常一段时间再恢复，避免状态来回跳变。
  config.engine.degradation.recovery_duration_ns = unsigned_value(
      require_entry(degradation_section, "recovery_duration_ns"), 0U,
      std::numeric_limits<std::uint64_t>::max(), "recovery duration");
  // max_tracked_edges限制质量状态机能同时保存的边数。
  config.engine.degradation.max_tracked_edges = size_value(
      require_entry(degradation_section, "max_tracked_edges"), 1U,
      kMaximumTrackedEdges, "maximum tracked edges");

  // input_bind_address是在线进程接收ZJCL UDP数据时绑定的本机地址。
  config.online.input_bind_address = nonempty_value(
      require_entry(online_section, "input_bind_address"),
      "input bind address");
  // input_port是接收端口；先按64位解析和范围检查，再显式收窄为16位。
  config.online.input_port = static_cast<std::uint16_t>(unsigned_value(
      require_entry(online_section, "input_port"), 1U,
      std::numeric_limits<std::uint16_t>::max(), "input UDP port"));
  // output_address是定位、网络、观测和告警结果的目标地址。
  config.online.output_address = nonempty_value(
      require_entry(online_section, "output_address"), "output address");
  // output_port是发往上海交大ROS 2适配层或GCS转发端的UDP端口。
  config.online.output_port = static_cast<std::uint16_t>(unsigned_value(
      require_entry(online_section, "output_port"), 1U,
      std::numeric_limits<std::uint16_t>::max(), "output UDP port"));
  // input_rate_hz用于在线循环调度和输入超时预期，不会改变实际传感器时间戳。
  config.online.input_rate_hz = positive_value(
      require_entry(online_section, "input_rate_hz"), "input rate");
  // output_rate_hz规定算法快照和遥测的发布节奏。
  config.online.output_rate_hz = positive_value(
      require_entry(online_section, "output_rate_hz"), "output rate");
  // event_log_enabled决定是否保存可供任务后回放的二进制原始帧日志。
  config.online.event_log_enabled = boolean_value(
      require_entry(online_section, "event_log_enabled"),
      "event log enabled");
  // event_log_path给出事件日志文件位置，即使禁用日志也保持配置字段明确。
  config.online.event_log_path = nonempty_value(
      require_entry(online_section, "event_log_path"), "event log path");
  // max_payload_size限制单个ZJCL载荷，先于分配拒绝异常大数据包。
  config.online.max_payload_size = size_value(
      require_entry(online_section, "max_payload_size"), 1U,
      kMaximumPayloadBytes, "maximum payload size");
  // max_log_record_size限制单条回放记录，防止损坏日志声明巨大长度。
  config.online.max_log_record_size = size_value(
      require_entry(online_section, "max_log_record_size"),
      kWireHeaderBytes, static_cast<std::size_t>(
                            std::numeric_limits<std::uint32_t>::max()),
      "maximum log record size");

  // section_entry遍历所有节索引项，只把node.<id>节转换为平面节点初值。
  for (const auto& section_entry : parsed.sections) {
    // section引用当前节对象，其生命周期由parsed覆盖整个解析函数。
    const Section& section = section_entry.second;
    if (!section.node) {
      continue;
    }
    // node为当前动态节点节构造的二维初值，完成全部字段校验后才追加。
    NodeInitialization node{};
    node.node_id = section.node_id;  // 节名node.<id>已经解析为唯一平台编号。
    // 六个*_entry分别提供x/y/vx/vy及位置/速度1σ的文本和值来源行。
    const Entry& x_entry = require_entry(section, "x");
    const Entry& y_entry = require_entry(section, "y");
    const Entry& vx_entry = require_entry(section, "vx");
    const Entry& vy_entry = require_entry(section, "vy");
    const Entry& position_std_entry =
        require_entry(section, "position_std_m");
    const Entry& velocity_std_entry =
        require_entry(section, "velocity_std_mps");
    node.x = finite_value(x_entry, "node x");  // 读取东向初始相对位置，允许正负但不允许NaN/Inf。
    node.y = finite_value(y_entry, "node y");  // 读取北向初始相对位置。
    node.vx = finite_value(vx_entry, "node vx");  // 读取东向初始相对速度。
    node.vy = finite_value(vy_entry, "node vy");  // 读取北向初始相对速度。
    node.position_std_m = positive_value(
        position_std_entry, "node position standard deviation");
    node.velocity_std_mps = positive_value(
        velocity_std_entry, "node velocity standard deviation");
    // initial_yaw_rad允许每车提供不同初始朝向；采用[-pi,pi)唯一规范范围。
    double initial_yaw_rad = 0.0;
    const auto yaw_entry = section.entries.find("initial_yaw_rad");
    if (yaw_entry != section.entries.end()) {
      initial_yaw_rad = finite_value(yaw_entry->second, "node initial yaw");
      if (initial_yaw_rad < -kPi || initial_yaw_rad >= kPi) {
        fail(IniError::kInvalidValue, yaw_entry->second.line,
             "node initial yaw must be in [-pi, pi)");
      }
    }
    initial_yaw_by_node.emplace(node.node_id, initial_yaw_rad);
    node_entry_lines.emplace(
        node.node_id,
        NodeEntryLines{x_entry.line, y_entry.line, vx_entry.line,
                       vy_entry.line, position_std_entry.line,
                       velocity_std_entry.line});
    config.engine.nodes.push_back(node);  // 完整节点才追加，前面任一字段失败都不会留下半节点。
  }
  // 排序lambda不捕获外部状态；left/right为待比较节点，按node_id建立确定配置顺序。
  std::sort(config.engine.nodes.begin(), config.engine.nodes.end(),
            [](const NodeInitialization& left,
               const NodeInitialization& right) {
              return left.node_id < right.node_id;
            });

  if (inertial_section_entry != parsed.sections.end()) {
    // section引用可选惯性节；enabled=true才构造15N状态配置。
    const Section& section = inertial_section_entry->second;
    const bool enabled = boolean_value(require_entry(section, "enabled"),
                                       "inertial enabled");
    // enabled=false等价于完全不构造惯性optional，Engine因此明确走仅测距回退；
    // 运行过程中不能靠切换输入消息类型在两种状态模型之间动态切换。
    if (enabled) {
      // inertial为完成全部时序、噪声和资源校验后才移入config.optional的候选值。
      InertialDemoConfig inertial{};
      // max_inertial_state_dimension必须至少容纳每节点15维误差状态，并限制总内存。
      inertial.max_inertial_state_dimension = size_value(
          require_entry(section, "max_inertial_state_dimension"), 15U,
          15U * kMaximumNodes, "maximum inertial state dimension");
      // gravity_mps2是ENU导航系中用于补偿重力的正标量大小。
      inertial.filter.gravity_mps2 = positive_value(
          require_entry(section, "gravity_mps2"), "gravity magnitude");
      // min_imu_dt_s用于拒绝重复或几乎重复的IMU时间戳。
      inertial.filter.min_imu_dt_s = positive_value(
          require_entry(section, "min_imu_dt_s"), "minimum IMU interval");
      // max_imu_dt_s用于发现IMU长时间断流，防止跨大间隔盲目积分。
      inertial.filter.max_imu_dt_s = positive_value(
          require_entry(section, "max_imu_dt_s"), "maximum IMU interval");
      // max_propagation_substep_s把合法长间隔继续拆小，减小非线性积分误差。
      inertial.filter.max_propagation_substep_s = positive_value(
          require_entry(section, "max_propagation_substep_s"),
          "maximum IMU propagation substep");
      // gyro_noise_density描述陀螺瞬时白噪声，进入姿态误差过程噪声。
      inertial.filter.gyro_noise_density_rad_s_sqrt_hz = positive_value(
          require_entry(section, "gyro_noise_density_rad_s_sqrt_hz"),
          "gyro noise density");
      // accel_noise_density描述加速度计白噪声，进入速度/位置误差过程噪声。
      inertial.filter.accel_noise_density_m_s2_sqrt_hz = positive_value(
          require_entry(section, "accel_noise_density_m_s2_sqrt_hz"),
          "accelerometer noise density");
      // gyro_bias_random_walk描述陀螺零偏随时间漂移的强度。
      inertial.filter.gyro_bias_random_walk_rad_s2_sqrt_hz = positive_value(
          require_entry(section, "gyro_bias_random_walk_rad_s2_sqrt_hz"),
          "gyro bias random walk");
      // accel_bias_random_walk描述加速度计零偏随时间漂移的强度。
      inertial.filter.accel_bias_random_walk_m_s3_sqrt_hz = positive_value(
          require_entry(section, "accel_bias_random_walk_m_s3_sqrt_hz"),
          "accelerometer bias random walk");
      // 惯性协方差下限用于抑制浮点误差造成的零/负对角项。
      inertial.filter.min_covariance_diagonal = positive_value(
          require_entry(section, "min_covariance_diagonal"),
          "inertial covariance floor");
      // 四元数范数容差用于判断输入姿态是否足够接近单位四元数。
      inertial.filter.quaternion_norm_tolerance = positive_value(
          require_entry(section, "quaternion_norm_tolerance"),
          "quaternion norm tolerance");
      // 协方差对称容差只吸收浮点舍入误差，不应掩盖错误的消息布局。
      inertial.filter.covariance_symmetry_tolerance = positive_value(
          require_entry(section, "covariance_symmetry_tolerance"),
          "covariance symmetry tolerance");
      // 此开关决定是否采用ROS 2 Imu消息中的3×3协方差来调整过程噪声。
      inertial.filter.use_message_covariance = boolean_value(
          require_entry(section, "use_message_covariance"),
          "use message covariance");
      // 此开关仅允许首包orientation初始化姿态，不把orientation当作连续姿态量测。
      inertial.filter.use_orientation_for_initialization = boolean_value(
          require_entry(section, "use_orientation_for_initialization"),
          "use orientation for initialization");
      // expected_frame_id防止把非FLU或错误安装坐标系的IMU数据直接送进积分。
      inertial.filter.expected_frame_id = nonempty_value(
          require_entry(section, "expected_frame_id"), "IMU frame id");
      if (inertial.filter.expected_frame_id.size() >= 32U ||
          inertial.filter.expected_frame_id.find('\0') != std::string::npos) {
        fail(IniError::kInvalidValue,
             require_entry(section, "expected_frame_id").line,
             "IMU frame id must fit in 31 bytes");
      }
      // initial_z/initial_vz为所有节点共用的ENU竖直位置/速度初值，单位m和m/s。
      const double initial_z = finite_value(
          require_entry(section, "initial_z_m"), "initial z");
      const double initial_vz = finite_value(
          require_entry(section, "initial_vz_mps"), "initial vertical velocity");
      // attitude_std、gyro_bias_std、accel_bias_std分别填充δθ/δbg/δba三轴1σ。
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
      inertial.nodes.reserve(config.engine.nodes.size());  // 节点数已知，预分配可避免循环中反复扩容。
      // source遍历已排序二维节点；每项扩展成ENU/FLU约定的15维惯性初值。
      for (const auto& source : config.engine.nodes) {
        // node为当前平台的惯性初值副本，完成所有三轴块后追加到inertial.nodes。
        InertialNodeInitialization node{};
        node.node_id = source.node_id;  // 保持二维滤波与15维惯性滤波使用同一节点主键。
        node.position_n_m = {source.x, source.y, initial_z};  // 把二维x/y和公共z扩展为ENU三维位置。
        node.velocity_n_mps = {source.vx, source.vy, initial_vz};  // 把二维速度和公共vz扩展为ENU三维速度。
        // 纯yaw的主动旋转q_b_to_n把FLU前向+x映射到ENU中的[cos(yaw),sin(yaw),0]。
        const double initial_yaw_rad = initial_yaw_by_node.at(source.node_id);
        const double half_yaw_rad = 0.5 * initial_yaw_rad;
        node.orientation_b_to_n = {std::cos(half_yaw_rad), 0.0, 0.0,
                                   std::sin(half_yaw_rad)};
        node.position_std_m = {source.position_std_m, source.position_std_m,
                               source.position_std_m};
        node.velocity_std_mps = {source.velocity_std_mps,
                                 source.velocity_std_mps,
                                 source.velocity_std_mps};
        node.attitude_std_rad = {attitude_std, attitude_std, attitude_std};  // 尚无分轴标定时三轴采用同一姿态1σ。
        node.gyro_bias_std_rad_s = {gyro_bias_std, gyro_bias_std,
                                    gyro_bias_std};
        node.accel_bias_std_m_s2 = {accel_bias_std, accel_bias_std,
                                    accel_bias_std};
        inertial.nodes.push_back(node);  // 追加完整15维初值；默认姿态和零偏保持结构中的安全默认值。
      }
      config.inertial = std::move(inertial);  // optional获得配置所有权，避免复制整个节点数组。
    }
  }

  // 阶段3：单字段解析后再检查节点、边、状态维数和时间门限的组合约束。
  validate_engine_config(config, engine_section, filter_section,
                         degradation_section, online_section,
                         node_entry_lines);
  return config;  // 按值返回；编译器通常使用返回值优化，不会复制大型配置。
}

DemoConfig load_ini_config(const std::filesystem::path& path) {
  // 文件按二进制读取，编码、换行和BOM统一交给parse_ini_config处理。
  // input为函数独占的二进制文件流，避免平台文本模式改写换行或BOM。
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    fail(IniError::kIoFailure, 1U,
         "configuration file could not be opened");
  }
  // text累计不超过1 MiB的原始配置字节；character为逐字节读取缓冲。
  std::string text;
  char character = 0;
  while (input.get(character)) {
    if (text.size() >= kMaximumConfigBytes) {
      fail(IniError::kInvalidConfiguration, 1U,
           "configuration file exceeds size limit");
    }
    text.push_back(character);  // 逐字节保留原文件内容，让解析层统一处理UTF-8、BOM和CRLF。
  }
  if (!input.eof()) {
    fail(IniError::kIoFailure, 1U,
         "configuration file could not be read");
  }
  return parse_ini_config(text);  // 文件I/O与文本语义解析分离，便于单元测试直接传字符串。
}

}  // namespace zju::coop::config

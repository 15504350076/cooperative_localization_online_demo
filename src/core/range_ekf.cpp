// 模块实现：二维恒速状态传播与平台间欧氏距离EKF更新，用于无IMU兼容模式。
// 关键原则：长时间隔按等效分段过程噪声计算，协方差采用Joseph形式并做正定稳定化；
// 该路径不与15维惯性路径同时运行，不能作为当前默认IMU+测距实现的替代说明。
// 维度记号：M为非参考节点数，参考节点固定为零状态且不占四维状态块。
//
// 初学者阅读主线：predict_to()按匀速模型更新x/y，update_range()比较预测距离和实测距离，
// 计算创新、NIS和Kalman增益，最后用Joseph形式更新协方差。其数学流程与15N路径相似，
// 但状态更小、没有姿态和IMU零偏，只用于兼容回退。
#include "core/range_ekf.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <utility>

namespace zju::coop {
namespace {

// constexpr要求编译期常量；quiet_NaN生成可传播但不会触发硬件异常的“非数字”标记。
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();  // 未知节点估计的无效数值标记。
constexpr double kMinimumRangeForDerivative = 1.0e-12;  // 距离雅可比可线性化的最小二维基线，单位m。
constexpr double kNanosecondsPerSecond = 1.0e9;  // 统一传感器时间由ns换算为s的倍率。

// `value`是待检查能否安全进入滤波计算的标量。
// 单行包装统一项目内写法；std::isfinite只对普通有限数返回true。
bool finite(double value) { return std::isfinite(value); }

// `factors`是需要稳定相乘的一组严格正long double因子。
long double scaled_positive_product(
    std::initializer_list<long double> factors) {
  // 把每个正因子拆成尾数和二进制指数后再相乘，避免dt高次项在中间步骤
  // 先上溢/下溢；最终结果仍必须能表示为long double和后续double。
  // `mantissa`累计规格化二进制尾数；`exponent`累计对应二进制指数。
  long double mantissa = 1.0L;
  long long exponent = 0LL;
  // `factor`依次取待乘的每个严格正因子。
  for (const long double factor : factors) {
    // initializer_list可能由花括号临时构造；范围for逐个只读取得其中因子。
    if (!std::isfinite(factor) || factor <= 0.0L) {
      throw std::overflow_error("scaled product factor is invalid");
    }
    // `factor_exponent`接收当前因子的二进制指数；
    // `factor_mantissa`是frexp分解后的规格化尾数。
    int factor_exponent = 0;
    const long double factor_mantissa =
        // `&factor_exponent`传入变量地址，让frexp通过指针写回指数部分。
        std::frexp(factor, &factor_exponent);
    // `*=`和`+=`都是复合赋值，分别累计尾数乘积与二进制指数和。
    mantissa *= factor_mantissa;
    exponent += factor_exponent;

    // `normalization_exponent`接收累计尾数重新规格化时移出的二进制指数。
    int normalization_exponent = 0;
    mantissa = std::frexp(mantissa, &normalization_exponent);
    exponent += normalization_exponent;
  }
  if (exponent > std::numeric_limits<int>::max() ||
      exponent < std::numeric_limits<int>::min()) {
    throw std::overflow_error("scaled product exponent is out of range");
  }
  // `product`是将累计尾数和指数重新组合后的最终正乘积。
  const long double product =
      // static_cast<int>是显式窄化；前面的范围检查保证转换不会溢出。
      std::scalbn(mantissa, static_cast<int>(exponent));
  if (!std::isfinite(product) || product <= 0.0L) {
    throw std::overflow_error("scaled product is not representable");
  }
  return product;
}

// `disposition`是本次测距的最终分类；`covariance_scale`是原样回传的量测方差放大倍数。
UpdateResult result(UpdateDisposition disposition, double covariance_scale) {
  // `value`是仅填充处置和降权信息的早退诊断。
  UpdateResult value{};
  // 点号访问局部结构体成员；这两个字段赋值后，其余字段继续保持{}产生的零值。
  value.disposition = disposition;
  value.covariance_scale = covariance_scale;
  return value;
}

// `matrix`是待做严格正定检查的方阵；`minimum_pivot`是绝对主元下限；
// `require_scaled_margin=true`时还要求相对当前对角尺度的舍入裕量。
bool has_strict_cholesky_factor(const DenseMatrix& matrix,
                                double minimum_pivot = 0.0,
                                bool require_scaled_margin = false) {
  if (matrix.rows() != matrix.cols()) {
    // 非方阵不存在本函数所需的Cholesky分解，直接用false报告。
    return false;
  }
  // `lower`是与输入同维的Cholesky下三角工作矩阵。
  DenseMatrix lower(matrix.rows(), matrix.cols());
  // `row`遍历待分解矩阵行；`col`遍历当前行的下三角列。
  for (std::size_t row = 0U; row < matrix.rows(); ++row) {
    for (std::size_t col = 0U; col <= row; ++col) {
      // `residual`是扣除已知下三角内积后的当前分解残差。
      double residual = matrix(row, col);
      // `inner`遍历当前(row,col)之前已求得的下三角列。
      for (std::size_t inner = 0U; inner < col; ++inner) {
        residual -= lower(row, inner) * lower(col, inner);
      }
      if (!finite(residual)) {
        return false;
      }
      if (row == col) {
        // `pivot_threshold`是当前对角主元必须严格超过的数值下限。
        double pivot_threshold = minimum_pivot;
        if (require_scaled_margin) {
          // max在绝对下限与“当前尺度×机器精度”之间取更严格的一项。
          pivot_threshold = std::max(
              pivot_threshold,
              std::numeric_limits<double>::epsilon() *
                  std::abs(matrix(row, row)) * 8.0);
        }
        if (residual <= pivot_threshold) {
          return false;
        }
        lower(row, col) = std::sqrt(residual);
      } else {
        // 非对角元素使用已求出的对角主元L(col,col)完成标准Cholesky递推。
        lower(row, col) = residual / lower(col, col);
        if (!finite(lower(row, col))) {
          return false;
        }
      }
    }
  }
  return true;  // 所有主元与非对角项均成功求出，说明矩阵通过严格正定检查。
}

// `covariance`是原位稳定化的状态协方差；`minimum_diagonal`是对角线与初始抖动下限。
bool stabilize_positive_definite(DenseMatrix& covariance,
                                 double minimum_diagonal) {
  // 先恢复有限对角线和对称性，再逐级增加极小抖动直到Cholesky检查通过。
  // `scale`累计协方差对角线最大绝对尺度，用于设置机器精度抖动。
  double scale = minimum_diagonal;
  // `row`遍历协方差行及对应对角线；`col`遍历该行全部列做有限值检查。
  for (std::size_t row = 0U; row < covariance.rows(); ++row) {
    for (std::size_t col = 0U; col < covariance.cols(); ++col) {
      if (!finite(covariance(row, col))) {
        return false;
      }
    }
    covariance(row, row) =
        std::max(covariance(row, row), minimum_diagonal);
    scale = std::max(scale, std::abs(covariance(row, row)));
  }
  covariance = covariance.symmetrized();
  // `jitter`是每次试探统一加入主对角线的正数抖动。
  double jitter = std::max(
      minimum_diagonal,
      std::numeric_limits<double>::epsilon() * scale * 8.0);
  if (has_strict_cholesky_factor(covariance, minimum_diagonal, true)) {
    // 原矩阵已经满足正定和尺度裕量，不需要额外抖动。
    return true;
  }
  // `attempt`遍历最多六档、每档放大十倍的对角抖动试探。
  for (unsigned int attempt = 0U; attempt < 6U; ++attempt) {
    if (!finite(jitter) || jitter <= 0.0) {
      return false;
    }
    // `jittered`是本档抖动下的协方差候选，不通过时不改原矩阵。
    DenseMatrix jittered = covariance;
    // `index`同步遍历候选协方差的全部主对角行列位置。
    for (std::size_t index = 0U; index < jittered.rows(); ++index) {
      jittered(index, index) += jitter;
      if (!finite(jittered(index, index))) {
        return false;
      }
    }
    if (has_strict_cholesky_factor(jittered)) {
      // std::move把候选矩阵的内部vector所有权转交给covariance，避免复制全部元素。
      covariance = std::move(jittered);
      return true;
    }
    jitter *= 10.0;  // 当前档仍不正定，下一轮把对角抖动扩大一个数量级。
  }
  return false;
}

}  // namespace

RangeEkf::RangeEkf(FilterConfig config,
                   std::vector<NodeInitialization> initializations)
    // config和initializations按值接收，使构造函数拥有独立副本；成员初始化列表先保存配置并建立0×0矩阵。
    : config_(config), covariance_(0U, 0U) {
  // 构造时锁定节点顺序、参考节点和状态维度，运行中不动态增加平台。
  if (!finite(config_.process_accel_std_mps2) ||
      config_.process_accel_std_mps2 <= 0.0 || !finite(config_.nis_gate) ||
      config_.nis_gate < 0.0 || !finite(config_.max_prediction_step_s) ||
      config_.max_prediction_step_s <= 0.0 ||
      !finite(config_.min_covariance_diagonal) ||
      config_.min_covariance_diagonal <= 0.0) {
    throw std::invalid_argument("RangeEkf configuration is invalid");
  }
  // `process_variance`是平面各轴白噪声加速度方差，单位m²/s⁴。
  const double process_variance = config_.process_accel_std_mps2 *
                                  config_.process_accel_std_mps2;
  if (!finite(process_variance) || process_variance <= 0.0) {
    throw std::invalid_argument(
        "process acceleration variance is not representable");
  }

  // `reference_iterator`定位定义相对坐标原点的初始化记录。
  // Lambda捕获`this`以读取固定参考编号；参数`node`借用当前候选初始化项。
  const auto reference_iterator = std::find_if(
      // begin/end组成半开搜索区间[begin,end)，end本身不是有效元素。
      initializations.begin(), initializations.end(),
      [this](const NodeInitialization& node) {
        // `[this]`是Lambda捕获列表，使匿名函数能读取当前对象的config_成员。
        return node.node_id == config_.reference_node_id;
      });
  if (reference_iterator == initializations.end()) {
    throw std::invalid_argument("reference node is not initialized");
  }
  // `origin_x`、`origin_y`是参考节点初始平面位置(m)；
  // `origin_vx`、`origin_vy`是参考节点初始平面速度(m/s)。
  // `iterator->member`等价于`(*iterator).member`，这里读取找到的参考节点字段。
  const double origin_x = reference_iterator->x;
  const double origin_y = reference_iterator->y;
  const double origin_vx = reference_iterator->vx;
  const double origin_vy = reference_iterator->vy;
  if (!finite(origin_x) || !finite(origin_y) || !finite(origin_vx) ||
      !finite(origin_vy)) {
    throw std::invalid_argument("reference state must be finite");
  }

  // `nonreference_count`累计实际占用[x,y,vx,vy]四维状态块的平台数。
  std::size_t nonreference_count = 0U;
  // `initialization`依次借用每个平台初值以校验并固定节点记录顺序。
  for (const auto& initialization : initializations) {
    // `const auto&`让编译器推导NodeInitialization只读引用，避免每轮复制整个结构。
    if (!finite(initialization.x) || !finite(initialization.y) ||
        !finite(initialization.vx) || !finite(initialization.vy) ||
        !finite(initialization.position_std_m) ||
        initialization.position_std_m <= 0.0 ||
        !finite(initialization.velocity_std_mps) ||
        initialization.velocity_std_mps <= 0.0) {
      throw std::invalid_argument("node initialization is invalid");
    }
    if (node_lookup_.find(initialization.node_id) != node_lookup_.end()) {
      throw std::invalid_argument("node identifiers must be unique");
    }

    // `is_reference=true`表示该平台固定为零状态，不占用四维块；
    // `offset`是当前非参考节点在4M状态中的候选四维块起始下标。
    const bool is_reference =
        initialization.node_id == config_.reference_node_id;
    const std::size_t offset = nonreference_count * 4U;
    node_lookup_.emplace(initialization.node_id, nodes_.size());
    // 花括号按NodeRecord的node_id/offset/reference顺序构造记录，再追加到vector。
    nodes_.push_back({initialization.node_id, offset, is_reference});
    if (!is_reference) {
      ++nonreference_count;
    }
  }

  // assign把vector长度设为4M并把每项初始化为0；参考节点不占状态元素。
  state_.assign(nonreference_count * 4U, 0.0);
  covariance_ = DenseMatrix(state_.size(), state_.size());
  // `node_index`同步遍历节点记录与原初始化数组。
  for (std::size_t node_index = 0U; node_index < nodes_.size(); ++node_index) {
    // `node`借用当前节点记录；`initialization`借用同序原始初值。
    const NodeRecord& node = nodes_[node_index];
    if (node.reference) {
      // 参考节点固定为相对零状态，没有四维块可写。
      continue;
    }
    // `initialization`借用与当前节点记录同序的初始平面状态和标准差。
    const NodeInitialization& initialization = initializations[node_index];
    // `offset`是该非参考节点[x,y,vx,vy]四维状态块的起始下标。
    const std::size_t offset = node.offset;
    state_[offset] = initialization.x - origin_x;
    state_[offset + 1U] = initialization.y - origin_y;
    state_[offset + 2U] = initialization.vx - origin_vx;
    state_[offset + 3U] = initialization.vy - origin_vy;
    // `position_variance`是x/y共用初始方差(m²)；
    // `velocity_variance`是vx/vy共用初始方差(m²/s²)。
    const double position_variance =
        initialization.position_std_m * initialization.position_std_m;
    const double velocity_variance =
        initialization.velocity_std_mps * initialization.velocity_std_mps;
    if (!finite(position_variance) || !finite(velocity_variance)) {
      throw std::invalid_argument("node initialization variance overflows");
    }
    covariance_(offset, offset) =
        std::max(position_variance, config_.min_covariance_diagonal);
    covariance_(offset + 1U, offset + 1U) =
        std::max(position_variance, config_.min_covariance_diagonal);
    covariance_(offset + 2U, offset + 2U) =
        std::max(velocity_variance, config_.min_covariance_diagonal);
    covariance_(offset + 3U, offset + 3U) =
        std::max(velocity_variance, config_.min_covariance_diagonal);
  }
  if (!finite_state_and_covariance()) {
    throw std::invalid_argument(
        "relative initialization produced a non-finite value");
  }
}

void RangeEkf::predict_to(std::uint64_t timestamp_ns) {
  // 已建立时间基准后拒绝倒退；此void兼容接口选择保持原状态而不是抛异常。
  if (has_timebase_ && timestamp_ns < last_timestamp_ns_) {
    return;
  }
  if (!has_timebase_) {
    // 第一帧只建立时间原点，没有时间差，因而不能执行运动预测。
    last_timestamp_ns_ = timestamp_ns;
    has_timebase_ = true;
    return;
  }
  if (timestamp_ns == last_timestamp_ns_) {
    // 时间差为0时状态转移是单位阵，直接返回避免无意义矩阵运算。
    return;
  }

  // 统一纳秒时间轴转换为秒，并按最大预测步长推导等效分段数。
  // `total_seconds`是目标时间与当前状态统一传感器时间之差，单位s；
  // `required_steps`是满足最大预测步长的向上取整等效子步数。
  const double total_seconds =
      static_cast<double>(timestamp_ns - last_timestamp_ns_) /
      kNanosecondsPerSecond;
  const long double required_steps = std::ceil(
      static_cast<long double>(total_seconds) /
      static_cast<long double>(config_.max_prediction_step_s));
  // `state_before`保存预测前按[x,y,vx,vy]分块的4M状态；
  // `covariance_before`保存与该状态同序的4M×4M联合协方差；
  // `timestamp_before`保存预测前统一传感器时间，单位ns；
  // `had_timebase=true`表示预测前`last_timestamp_ns_`已建立有效时间基准。
  const std::vector<double> state_before = state_;
  const DenseMatrix covariance_before = covariance_;
  const std::uint64_t timestamp_before = last_timestamp_ns_;
  const bool had_timebase = has_timebase_;
  try {
    // try块内任何标准或非标准异常都由下面catch(...)回滚后重新抛出。
    predict_interval(total_seconds, required_steps);
    if (!finite_state_and_covariance()) {
      throw std::overflow_error("prediction produced a non-finite value");
    }
  } catch (...) {
    // `catch(...)`捕获所有异常类型；恢复四份事务快照后用裸throw保留原异常继续上抛。
    state_ = state_before;
    covariance_ = covariance_before;
    last_timestamp_ns_ = timestamp_before;
    has_timebase_ = had_timebase;
    throw;  // 裸throw只能在catch中使用，表示重新抛出当前正在处理的异常。
  }
  last_timestamp_ns_ = timestamp_ns;
}

void RangeEkf::predict_interval(double total_seconds,
                                long double step_count) {
  if (!finite(total_seconds) || total_seconds <= 0.0 ||
      !std::isfinite(step_count) || step_count < 1.0L) {
    throw std::overflow_error("prediction interval is not representable");
  }

  // 使用long double分解计算dt高次项，避免长时间间隔的过程噪声中间量溢出。
  // 这里直接计算n个等长恒速子步的闭式累计Q，避免真的循环极大的step_count。
  // `duration`是long double精度的总预测时长(s)；`q`是轴向加速度过程方差(m²/s⁴)。
  const long double duration = static_cast<long double>(total_seconds);
  const long double q =
      static_cast<long double>(config_.process_accel_std_mps2) *
      static_cast<long double>(config_.process_accel_std_mps2);
  // `inverse_count`为1/n，`inverse_count2`为1/n²，用于闭式累计过程噪声。
  const long double inverse_count = 1.0L / step_count;
  const long double inverse_count2 = inverse_count * inverse_count;
  // `process_pp`、`process_pv`、`process_vv`分别是每轴累计位置方差(m²)、
  // 位置-速度协方差(m²/s)与速度方差(m²/s²)。
  const long double process_pp = scaled_positive_product(
      {q, duration, duration, duration, duration, inverse_count,
       (4.0L - inverse_count2) / 12.0L});
  const long double process_pv = scaled_positive_product(
      {q, duration, duration, duration, inverse_count, 0.5L});
  const long double process_vv = scaled_positive_product(
      {q, duration, duration, inverse_count});
  // `intermediates`集中保存闭式计算的全部long double中间量，供统一有限值检查。
  const long double intermediates[] = {
      duration,       q,          inverse_count, inverse_count2,
      process_pp,     process_pv, process_vv,
  };
  // `value`依次取一个过程噪声闭式中间量。
  for (const long double value : intermediates) {
    if (!std::isfinite(value)) {
      throw std::overflow_error(
          "segmented process-noise calculation overflowed");
    }
  }
  // `process_pp_double`、`process_pv_double`、`process_vv_double`是写入
  // double协方差矩阵前的三类累计过程噪声。
  const double process_pp_double = static_cast<double>(process_pp);
  const double process_pv_double = static_cast<double>(process_pv);
  const double process_vv_double = static_cast<double>(process_vv);
  if (!finite(process_pp_double) || !finite(process_pv_double) ||
      !finite(process_vv_double)) {
    throw std::overflow_error("process-noise result exceeds double range");
  }

  // `transition`是4M×4M恒速状态转移，块内行列顺序均为[x,y,vx,vy]。
  DenseMatrix transition = DenseMatrix::identity(state_.size());
  // `node`依次借用节点记录，为每个非参考四维块写入状态转移并推进名义状态。
  for (const auto& node : nodes_) {
    if (node.reference) {
      // 参考节点没有状态块，恒定保持零原点。
      continue;
    }
    transition(node.offset, node.offset + 2U) = total_seconds;
    transition(node.offset + 1U, node.offset + 3U) = total_seconds;
    state_[node.offset] += state_[node.offset + 2U] * total_seconds;
    state_[node.offset + 1U] +=
        state_[node.offset + 3U] * total_seconds;
  }

  // 运算符从左结合：先算F*P，再右乘Fᵀ，完成P=F P Fᵀ。
  covariance_ = transition * covariance_ * transition.transpose();
  // `node`再次遍历非参考节点，为对应4×4对角块加入轴向过程噪声。
  for (const auto& node : nodes_) {
    if (node.reference) {
      continue;
    }
    // `x`、`y`、`vx`、`vy`分别是当前四维块的位置与速度分量下标。
    const std::size_t x = node.offset;
    const std::size_t y = node.offset + 1U;
    const std::size_t vx = node.offset + 2U;
    const std::size_t vy = node.offset + 3U;
    covariance_(x, x) += process_pp_double;
    covariance_(y, y) += process_pp_double;
    covariance_(x, vx) += process_pv_double;
    covariance_(vx, x) += process_pv_double;
    covariance_(y, vy) += process_pv_double;
    covariance_(vy, y) += process_pv_double;
    covariance_(vx, vx) += process_vv_double;
    covariance_(vy, vy) += process_vv_double;
  }
  if (!stabilize_positive_definite(covariance_,
                                   config_.min_covariance_diagonal)) {
    throw std::overflow_error(
        "predicted covariance is not positive definite");
  }
}

UpdateResult RangeEkf::update(const RangePacket& packet,
                              double covariance_scale) {
  // 阶段1：先推进到量测时刻并检查输入，失败时返回明确诊断而非静默丢弃。
  if (has_timebase_ && packet.timestamp_ns < last_timestamp_ns_) {
    return result(UpdateDisposition::OutOfOrder, covariance_scale);
  }
  try {
    predict_to(packet.timestamp_ns);
  } catch (const std::exception&) {
    // `const std::exception&`只读借用异常基类；此处不需要具体what文本，只映射为数值失败。
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  if (!packet.valid || !finite(packet.range_m) ||
      !finite(packet.range_std_m) || packet.range_std_m <= 0.0 ||
      !finite(covariance_scale) || covariance_scale < 1.0) {
    return result(UpdateDisposition::InvalidPacket, covariance_scale);
  }
  if (packet.range_m <= 0.0) {
    return result(UpdateDisposition::NonPositiveRange, covariance_scale);
  }

  // `from`与`to`分别借用测距起点和终点节点记录；指针仅在节点表不变时有效。
  const NodeRecord* from = find_node(packet.from_node);
  const NodeRecord* to = find_node(packet.to_node);
  // nullptr是空指针字面量；任一查找失败都不能解引用节点记录。
  if (from == nullptr || to == nullptr) {
    return result(UpdateDisposition::UnknownNode, covariance_scale);
  }
  // 指针相等说明两端查到同一个NodeRecord，也就是自测距。
  if (from == to) {
    return result(UpdateDisposition::SelfRange, covariance_scale);
  }
  if (!finite_state_and_covariance()) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  // `from_x`、`from_y`与`to_x`、`to_y`是两端相对参考节点的平面位置，单位m；
  // 参考节点不占状态块，按固定零位置参与几何计算。
  // 三目运算符：若端点是参考节点取固定0，否则读取其状态块位置。
  const double from_x = from->reference ? 0.0 : state_[from->offset];
  const double from_y = from->reference ? 0.0 : state_[from->offset + 1U];
  const double to_x = to->reference ? 0.0 : state_[to->offset];
  const double to_y = to->reference ? 0.0 : state_[to->offset + 1U];
  // `delta_x`与`delta_y`组成从from指向to的二维测距差向量(m)；
  // `expected_range`是该向量模长，即滤波器预测距离(m)。
  const double delta_x = to_x - from_x;
  const double delta_y = to_y - from_y;
  const double expected_range = std::hypot(delta_x, delta_y);
  // hypot比手写sqrt(dx*dx+dy*dy)更能避免极端数值中间溢出/下溢。
  if (!finite(expected_range) ||
      expected_range <= kMinimumRangeForDerivative) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }

  // 阶段2：距离雅可比只作用于两节点的x/y位置分量，参考节点保持零状态。
  // `jacobian`是一维距离量测对完整4M状态的1×4M雅可比；
  // `unit_x`与`unit_y`是测距差向量沿x/y轴的单位方向分量。
  std::vector<double> jacobian(state_.size(), 0.0);
  const double unit_x = delta_x / expected_range;
  const double unit_y = delta_y / expected_range;
  if (!from->reference) {
    jacobian[from->offset] = -unit_x;
    jacobian[from->offset + 1U] = -unit_y;
  }
  if (!to->reference) {
    jacobian[to->offset] = unit_x;
    jacobian[to->offset + 1U] = unit_y;
  }

  // `covariance_times_jacobian`是P*Hᵀ的4M列向量，包含跨节点协方差作用。
  std::vector<double> covariance_times_jacobian(state_.size(), 0.0);
  // `row`遍历P*Hᵀ输出状态分量；`col`遍历协方差列及H分量。
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      covariance_times_jacobian[row] +=
          covariance_(row, col) * jacobian[col];
    }
  }
  // `projected_variance`累计H*P*Hᵀ，单位m²。
  double projected_variance = 0.0;
  // `index`遍历完整4M状态以完成标量二次型。
  for (std::size_t index = 0U; index < state_.size(); ++index) {
    projected_variance +=
        jacobian[index] * covariance_times_jacobian[index];
  }
  // `measurement_variance`是经质量倍数放大的距离量测方差R，单位m²。
  double measurement_variance = 0.0;
  try {
    // `scaled_measurement_variance`用long double稳定计算range_std²*covariance_scale。
    const long double scaled_measurement_variance = scaled_positive_product(
        {static_cast<long double>(packet.range_std_m),
         static_cast<long double>(packet.range_std_m),
         static_cast<long double>(covariance_scale)});
    measurement_variance =
        static_cast<double>(scaled_measurement_variance);
  } catch (const std::exception&) {
    return result(UpdateDisposition::NumericalFailure, covariance_scale);
  }
  // S=H*P*H^T+R，R=(range_std²)*covariance_scale；
  // 质量层的降权只放大R，不直接修改几何雅可比或状态。
  // `innovation_variance`是S，单位m²；`innovation`是实测减预测距离，单位m。
  const double innovation_variance =
      projected_variance + measurement_variance;
  const double innovation = packet.range_m - expected_range;

  // `update_result`保留创新、S、NIS、降权倍数和最终互斥处置。
  UpdateResult update_result{};
  update_result.innovation_m = innovation;
  update_result.innovation_variance = innovation_variance;
  update_result.covariance_scale = covariance_scale;
  if (!finite(measurement_variance) || measurement_variance <= 0.0 ||
      !finite(innovation_variance) || innovation_variance <= 0.0 ||
      !finite(innovation)) {
    update_result.disposition = UpdateDisposition::NumericalFailure;
    return update_result;
  }

  // 先算标准化残差再平方，可在innovation²本身可能溢出时保持可诊断结果。
  // `standardized_residual`是|innovation|/sqrt(S)的无量纲绝对标准化残差；
  // `maximum_root`是平方仍可由double表示的最大正数边界。
  const double standardized_residual =
      std::abs(innovation) / std::sqrt(innovation_variance);
  const double maximum_root =
      std::sqrt(std::numeric_limits<double>::max());
  update_result.nis =
      // 多行三目表达式：残差不可平方时饱和到double最大值，否则正常平方。
      !finite(standardized_residual) || standardized_residual > maximum_root
          ? std::numeric_limits<double>::max()
          : standardized_residual * standardized_residual;
  // 阶段3：NIS门限隔离几何异常量测；拒绝时保留已完成的时间预测，
  // 但不再施加该测距的后验状态和协方差修正。
  if (standardized_residual > std::sqrt(config_.nis_gate)) {
    update_result.disposition = UpdateDisposition::NisRejected;
    return update_result;
  }

  // `candidate_state`是在接纳量测后待提交的4M状态副本；
  // `gain`是完整4M×1 Kalman增益。
  std::vector<double> candidate_state = state_;
  std::vector<double> gain(state_.size(), 0.0);
  // `row`遍历联合状态分量，计算对应增益与后验误差修正。
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    // 一维量测的K=P Hᵀ/S；同一创新通过不同K分量修正每个相关状态。
    gain[row] = covariance_times_jacobian[row] / innovation_variance;
    candidate_state[row] += gain[row] * innovation;
  }

  // `identity_minus_gain_jacobian`是4M×4M矩阵I-KH。
  DenseMatrix identity_minus_gain_jacobian =
      DenseMatrix::identity(state_.size());
  // `row`遍历Kalman增益分量及矩阵行；`col`遍历H分量及矩阵列。
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      identity_minus_gain_jacobian(row, col) -=
          gain[row] * jacobian[col];
    }
  }
  // 阶段4：在候选副本上用Joseph形式更新，有限值和正定性通过后才提交。
  // `candidate_covariance`是Joseph公式生成的4M×4M后验联合协方差副本。
  DenseMatrix candidate_covariance =
      identity_minus_gain_jacobian * covariance_ *
      identity_minus_gain_jacobian.transpose();
  // `row`与`col`遍历Joseph公式KRKᵀ项的全部4M×4M元素。
  for (std::size_t row = 0U; row < state_.size(); ++row) {
    for (std::size_t col = 0U; col < state_.size(); ++col) {
      candidate_covariance(row, col) +=
          gain[row] * measurement_variance * gain[col];
    }
  }

  // `value`依次检查候选4M状态中的位置(m)或速度(m/s)分量是否有限。
  for (double value : candidate_state) {
    if (!finite(value)) {
      update_result.disposition = UpdateDisposition::NumericalFailure;
      return update_result;
    }
  }
  if (!stabilize_positive_definite(candidate_covariance,
                                   config_.min_covariance_diagonal)) {
    update_result.disposition = UpdateDisposition::NumericalFailure;
    return update_result;
  }

  state_ = std::move(candidate_state);
  // 两个候选均通过有限值和正定检查后才移动提交，前面的拒绝路径不会污染原状态。
  covariance_ = std::move(candidate_covariance);
  update_result.disposition = UpdateDisposition::Accepted;
  return update_result;
}

const RangeEkf::NodeRecord* RangeEkf::find_node(
    std::uint32_t node_id) const {
  // `iterator`是平台编号到`nodes_`下标的查找结果。
  const auto iterator = node_lookup_.find(node_id);
  if (iterator == node_lookup_.end()) {
    return nullptr;
  }
  // `&`取得vector元素地址；节点表构造后不再扩容，因此该指针在滤波器生命周期内稳定。
  return &nodes_[iterator->second];
}

NodeEstimate RangeEkf::estimate(std::uint32_t node_id) const {
  // `node`借用目标平台记录；未知编号时为空。
  const NodeRecord* node = find_node(node_id);
  if (node == nullptr) {
    // `invalid`为未知平台构造`valid=false`且状态/协方差为NaN的诊断输出。
    NodeEstimate invalid{};
    invalid.node_id = node_id;
    invalid.timestamp_ns = last_timestamp_ns_;
    invalid.x = kNan;
    invalid.y = kNan;
    invalid.vx = kNan;
    invalid.vy = kNan;
    invalid.cov_xx = kNan;
    invalid.cov_xy = kNan;
    invalid.cov_yy = kNan;
    return invalid;
  }
  // `*node`解引用指针得到NodeRecord对象，再以const引用传给提取函数。
  return make_estimate(*node);
}

std::vector<NodeEstimate> RangeEkf::estimates() const {
  // `values`按初始化节点顺序收集全部相对参考估计。
  std::vector<NodeEstimate> values;
  values.reserve(nodes_.size());
  // `node`依次借用固定节点表中的记录。
  for (const auto& node : nodes_) {
    // push_back把当前按值返回的NodeEstimate追加到动态数组末尾。
    values.push_back(make_estimate(node));
  }
  return values;
}

const DenseMatrix& RangeEkf::covariance() const noexcept {
  // 返回const引用避免复制大矩阵；调用者只能读，不能直接改内部协方差。
  return covariance_;
}

NodeEstimate RangeEkf::make_estimate(const NodeRecord& node) const {
  // `value`是从当前4M状态及协方差提取的单节点输出。
  NodeEstimate value{};
  value.node_id = node.node_id;
  value.timestamp_ns = last_timestamp_ns_;
  value.valid = true;
  if (node.reference) {
    // value已经是全零且valid=true，恰好表示主参考相对自身的位置与速度。
    return value;
  }

  value.x = state_[node.offset];
  value.y = state_[node.offset + 1U];
  value.vx = state_[node.offset + 2U];
  value.vy = state_[node.offset + 3U];
  value.cov_xx = covariance_(node.offset, node.offset);
  value.cov_xy = covariance_(node.offset, node.offset + 1U);
  value.cov_yy = covariance_(node.offset + 1U, node.offset + 1U);
  return value;
}

bool RangeEkf::finite_state_and_covariance() const {
  // `value`依次检查4M状态中的位置(m)与速度(m/s)分量。
  for (double value : state_) {
    // 这里按值复制单个double成本极低，同时禁止循环体意外修改state_。
    if (!finite(value)) {
      return false;
    }
  }
  // `row`与`col`遍历4M×4M联合协方差全部元素，包括跨节点块。
  for (std::size_t row = 0U; row < covariance_.rows(); ++row) {
    for (std::size_t col = 0U; col < covariance_.cols(); ++col) {
      if (!finite(covariance_(row, col))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace zju::coop

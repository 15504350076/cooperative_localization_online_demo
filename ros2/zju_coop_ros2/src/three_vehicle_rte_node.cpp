// 可选RTE融合入口。只订阅专用局部NodeState，不参与uwb_only原有节点/话题。
#include "zju_coop/three_vehicle_rte_api.h"
#include "cooperative_interfaces/msg/uwb_range.hpp"
#include "cooperative_localization_msgs/msg/cooperative_pose2_d_array.hpp"
#include "cooperative_localization_msgs/msg/node_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_time_bridge.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
uint64_t stamp_ns(const builtin_interfaces::msg::Time& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) return 0;
  return static_cast<uint64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}
uint64_t steady_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}
void require(zju_coop_error_code_t rc, const char* operation) {
  if (rc != ZJU_COOP_OK) throw std::runtime_error(std::string(operation)+": "+zju_coop_error_string(rc));
}
const char* status_name(int status) {
  switch (status) {
    case ZJU_COOP_RTE_READY: return "TRACKING";
    case ZJU_COOP_RTE_UNOBSERVABLE: return "NEED_MORE_MOTION";
    case ZJU_COOP_RTE_STALE: return "STALE";
    case ZJU_COOP_RTE_AMBIGUOUS: return "AMBIGUOUS";
    case ZJU_COOP_RTE_RESIDUAL_REJECTED: return "RESIDUAL_REJECTED";
    case ZJU_COOP_RTE_NOT_CONVERGED: return "NOT_CONVERGED";
    default: return "INITIALIZING";
  }
}

class ThreeVehicleRteNode : public rclcpp::Node {
 public:
  ThreeVehicleRteNode():Node("zju_three_vehicle_rte_node"),handle_(nullptr,zju_coop_three_destroy_rte) {
    zju_coop_three_config_init_rte(&config_);
    const auto ids=declare_parameter<std::vector<int64_t>>("node_ids",{1,2,3});
    if(ids.size()!=3) throw std::invalid_argument("node_ids must contain exactly three IDs; first is reference");
    for(int i=0;i<3;++i) {
      if(ids[i]<=0 || ids[i]>65535) throw std::invalid_argument("node ID out of range");
      config_.node_ids[i]=static_cast<uint32_t>(ids[i]);
    }
    config_.planar=declare_parameter<bool>("planar",true);
    auto positive_u32=[this](const char* name,uint32_t value) {
      const auto n=declare_parameter<int64_t>(name,value);
      if(n<=0 || n>4096) throw std::invalid_argument(std::string(name)+" out of range");
      return static_cast<uint32_t>(n);
    };
    config_.max_samples_per_edge=positive_u32("max_samples_per_edge",128);
    config_.minimum_samples=positive_u32("minimum_samples",12);
    config_.max_iterations=positive_u32("max_iterations",80);
    config_.yaw_starts=positive_u32("yaw_starts",24);
    auto ms=[this](const char* name,uint64_t value) {
      const auto n=declare_parameter<int64_t>(name,static_cast<int64_t>(value/1000000));
      if(n<=0 || n>600000) throw std::invalid_argument(std::string(name)+" out of range");
      return static_cast<uint64_t>(n)*1000000;
    };
    config_.max_epoch_skew_ns=ms("max_epoch_skew_ms",config_.max_epoch_skew_ns);
    config_.node_timeout_ns=ms("node_timeout_ms",config_.node_timeout_ns);
    config_.range_timeout_ns=ms("range_timeout_ms",config_.range_timeout_ns);
    config_.solve_interval_ns=ms("solve_interval_ms",config_.solve_interval_ns);
    config_.window_duration_ns=ms("window_duration_ms",config_.window_duration_ns);
    config_.condition_limit=declare_parameter<double>("condition_limit",1e10);
    config_.max_rms_residual_m=declare_parameter<double>("max_rms_residual_m",.3);
    config_.max_range_innovation_m=declare_parameter<double>("max_range_innovation_m",1.0);
    config_.max_translation_std_m=declare_parameter<double>("max_translation_std_m",.25);
    config_.max_yaw_std_rad=declare_parameter<double>("max_yaw_std_rad",.1);
    const auto heights=declare_parameter<std::vector<double>>("local_origin_heights_m",{0,0,0});
    const auto levers=declare_parameter<std::vector<double>>("antenna_lever_arms_body_m",std::vector<double>(9,0.0));
    if(heights.size()!=3 || levers.size()!=9) throw std::invalid_argument("invalid heights/lever arms array length");
    std::copy(heights.begin(),heights.end(),config_.local_origin_heights_m);
    std::copy(levers.begin(),levers.end(),config_.antenna_lever_arms_body_m);
    range_std_=declare_parameter<double>("range_std_m",.1);
    frame_prefix_=declare_parameter<std::string>("local_frame_prefix","rte_odom_");
    const double rate=declare_parameter<double>("publish_rate_hz",20.0);
    if(!std::isfinite(range_std_) || range_std_<1e-6 || range_std_>1000 ||
       !std::isfinite(rate) || rate<=0 || rate>200 || frame_prefix_.empty()) throw std::invalid_argument("invalid rate, range standard deviation or local frame prefix");
    zju_coop_three_handle_rte_t* raw=nullptr;
    require(zju_coop_three_create_rte(&config_,&raw),"three_create_rte");
    handle_.reset(raw);
    poses_=create_publisher<cooperative_localization_msgs::msg::CooperativePose2DArray>("poses_2d",10);
    status_=create_publisher<std_msgs::msg::String>("status",10);
    states_=create_subscription<cooperative_localization_msgs::msg::NodeState>("node_state",rclcpp::SensorDataQoS(),
        [this](const cooperative_localization_msgs::msg::NodeState::SharedPtr m){on_state(*m);});
    ranges_=create_subscription<cooperative_interfaces::msg::UwbRange>("uwb_range",rclcpp::SensorDataQoS(),
        [this](const cooperative_interfaces::msg::UwbRange::SharedPtr m){on_range(*m);});
    timer_=create_wall_timer(std::chrono::duration<double>(1.0/rate),[this](){publish();});
    RCLCPP_INFO(get_logger(),"RTE enabled for %u/%u/%u; mode=%s; requires local odometry and observable motion",
                config_.node_ids[0],config_.node_ids[1],config_.node_ids[2],config_.planar?"known-height planar":"4DoF spatial");
  }
 private:
  uint64_t current(uint64_t steady) const {
    if(get_parameter("use_sim_time").as_bool()) {
      const auto value=now().nanoseconds(); return value>0?static_cast<uint64_t>(value):0;
    }
    return bridge_.current_time_ns(steady);
  }
  bool plausible(uint64_t time,uint64_t steady) const {
    if(!time) return false;
    const auto now=current(steady);
    if(!now) return true;
    return time>now ? time-now<=100000000 : now-time<=config_.node_timeout_ns;
  }
  void on_state(const cooperative_localization_msgs::msg::NodeState& m) {
    const uint64_t time=stamp_ns(m.header.stamp), steady=steady_ns();
    if(m.header.frame_id!=frame_prefix_+std::to_string(m.node_id) || !plausible(time,steady)) return;
    zju_coop_node_state_t state;
    zju_coop_node_state_init(&state);
    state.node_id=m.node_id; state.timestamp_ns=time;
    state.receive_timestamp_ns=std::max(time,current(steady)); state.valid=m.valid;
    std::copy(m.position_enu_m.begin(),m.position_enu_m.end(),state.position_enu_m);
    std::copy(m.velocity_enu_mps.begin(),m.velocity_enu_mps.end(),state.velocity_enu_mps);
    std::copy(m.orientation_flu_to_enu_xyzw.begin(),m.orientation_flu_to_enu_xyzw.end(),state.orientation_flu_to_enu_xyzw);
    const auto rc=zju_coop_three_push_node_state_rte(handle_.get(),&state);
    if(rc==ZJU_COOP_OK && m.node_id==config_.node_ids[0]) bridge_.observe(time,steady);
  }
  void on_range(const cooperative_interfaces::msg::UwbRange& m) {
    const uint64_t time=stamp_ns(m.header.stamp), steady=steady_ns();
    if(m.src_id>65535 || m.target_id>65535 || !plausible(time,steady)) return;
    zju_coop_range_packet_t range;
    zju_coop_range_packet_init(&range);
    range.from_node=static_cast<uint16_t>(m.src_id); range.to_node=static_cast<uint16_t>(m.target_id);
    range.timestamp_ns=time; range.receive_timestamp_ns=std::max(time,current(steady));
    range.range_m=m.distance; range.range_std_m=range_std_; range.valid=ZJU_COOP_TRUE;
    zju_coop_range_processing_result_t receipt;
    zju_coop_range_processing_result_init(&receipt);
    const auto rc=zju_coop_three_push_range_rte(handle_.get(),&range,&receipt);
    if(rc!=ZJU_COOP_OK) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"RTE range rejected: %d",rc);
  }
  void publish() {
    const auto time=current(steady_ns());
    if(!time) return;
    zju_coop_three_snapshot_rte_t state;
    zju_coop_three_snapshot_init_rte(&state);
    if(zju_coop_three_step_rte(handle_.get(),time,&state)!=ZJU_COOP_OK) return;
    zju_coop_pose2d_snapshot_v2_t header;
    zju_coop_pose2d_snapshot_v2_init(&header);
    std::array<zju_coop_vehicle_pose2d_v2_t,3> vehicles;
    for(auto& v:vehicles) zju_coop_vehicle_pose2d_v2_init(&v);
    uint32_t count=0;
    if(zju_coop_three_get_pose2d_v2_rte(handle_.get(),time,&header,vehicles.data(),3,sizeof(vehicles[0]),&count)!=ZJU_COOP_OK) return;
    cooperative_localization_msgs::msg::CooperativePose2DArray output;
    output.header.stamp.sec=static_cast<int32_t>(header.timestamp_ns/1000000000);
    output.header.stamp.nanosec=static_cast<uint32_t>(header.timestamp_ns%1000000000);
    output.header.frame_id=header.frame_id; output.reference_node_id=header.reference_node_id;
    for(const auto& v:vehicles) {
      cooperative_localization_msgs::msg::VehiclePose2D pose;
      pose.node_id=v.node_id; pose.x_m=v.x_m; pose.y_m=v.y_m; pose.yaw_rad=v.yaw_rad;
      pose.position_valid=v.position_valid; pose.yaw_valid=v.yaw_valid; output.vehicles.push_back(pose);
    }
    poses_->publish(output);
    std::ostringstream text;
    text<<"{\"status\":\""<<status_name(state.status)<<"\",\"code\":"<<state.status
        <<",\"initialized\":"<<(state.initialized_at_ns ? "true" : "false")
        <<",\"timestamp_ns\":"<<state.timestamp_ns<<",\"initialized_at_ns\":"<<state.initialized_at_ns
        <<",\"corrections\":"<<state.correction_count<<",\"active_edges\":"<<state.active_edges
        <<",\"sample_counts\":["<<state.sample_counts[0]<<","<<state.sample_counts[1]<<","<<state.sample_counts[2]<<"]"
        <<",\"rms_m\":"<<state.rms_residual_m<<",\"condition\":"<<state.condition_number
        <<",\"translation_std_m\":["<<state.position_std_m[0]<<","<<state.position_std_m[1]<<","<<state.position_std_m[3]
        <<","<<state.position_std_m[4]<<","<<state.position_std_m[6]<<","<<state.position_std_m[7]<<"]"
        <<",\"yaw_std_rad\":["<<state.yaw_std_rad[0]<<","<<state.yaw_std_rad[1]<<","<<state.yaw_std_rad[2]<<"]}"
        ;
    std_msgs::msg::String diagnostic; diagnostic.data=text.str(); status_->publish(diagnostic);
    if(last_status_!=state.status) {
      RCLCPP_INFO(get_logger(),"RTE state: %s; corrections=%llu rms=%.3fm",status_name(state.status),
          static_cast<unsigned long long>(state.correction_count),state.rms_residual_m);
      last_status_=state.status;
    }
  }
  zju_coop_three_config_rte_t config_{};
  std::unique_ptr<zju_coop_three_handle_rte_t,decltype(&zju_coop_three_destroy_rte)> handle_;
  zju_coop_ros2::SensorTimeBridge bridge_;
  std::string frame_prefix_;
  double range_std_{};
  int last_status_{-1};
  rclcpp::Publisher<cooperative_localization_msgs::msg::CooperativePose2DArray>::SharedPtr poses_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<cooperative_localization_msgs::msg::NodeState>::SharedPtr states_;
  rclcpp::Subscription<cooperative_interfaces::msg::UwbRange>::SharedPtr ranges_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<ThreeVehicleRteNode>()); }
  catch(const std::exception& e) { std::fprintf(stderr,"RTE node: %s\n",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}

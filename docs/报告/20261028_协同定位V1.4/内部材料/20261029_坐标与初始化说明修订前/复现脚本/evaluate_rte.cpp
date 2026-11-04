// 独立离线评估入口：只调用公开SDK；不修改默认算法或ROS节点。
// 输入CSV: kind,timestamp_ns,node_id,target_id,v0,...,v9。
// I: FLU角速度+比力；O: 局部p/v/q；R: 距离与标准差。
#include "zju_coop/three_vehicle_rte_api.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

void ok(zju_coop_error_code_t rc) {
  if (rc != ZJU_COOP_OK) throw std::runtime_error(zju_coop_error_string(rc));
}
struct Local {
  std::unique_ptr<zju_coop_handle_t, decltype(&zju_coop_destroy)> handle{nullptr,zju_coop_destroy};
  std::vector<zju_coop_static_imu_sample_t> samples;
  zju_coop_node_state_t state{};
  uint64_t start{}, previous{}, published{}, sequence{}, rejected{};
  int calibration{-1};
  bool ready{}, fault{};
  explicit Local(int id) {
    zju_coop_node_state_init(&state); state.node_id=id;
    zju_coop_node_initialization_t node;
    zju_coop_node_initialization_init(&node); node.node_id=id;
    zju_coop_config_t config;
    zju_coop_config_init(&config); config.reference_node_id=id;
    config.nodes=&node; config.node_count=1; config.node_stride=sizeof(node);
    zju_coop_handle_t* h=nullptr; ok(zju_coop_create(&config,&h)); handle.reset(h);
  }
  bool imu(uint64_t time, const std::array<double,10>& v) {
    if (fault) return false;
    if (!time || (previous && time<=previous)) {
      ++rejected; if(time<previous) fault=true; return false;
    }
    if(previous && time-previous>100000000) {
      if(ready) { fault=true; ++rejected; return false; }
      samples.clear(); start=0;
    }
    previous=time;
    if(!ready) {
      if(!start) start=time;
      zju_coop_static_imu_sample_t sample;
      zju_coop_static_imu_sample_init(&sample);
      std::copy_n(v.begin(),3,sample.angular_velocity_rad_s);
      std::copy_n(v.begin()+3,3,sample.specific_force_m_s2);
      samples.push_back(sample);
      if(time-start<10000000000ULL || samples.size()<200) return false;
      zju_coop_static_imu_config_t c; zju_coop_static_imu_config_init(&c);
      zju_coop_static_imu_result_t result; zju_coop_static_imu_result_init(&result);
      ok(zju_coop_initialize_static_imu(&c,samples.data(),samples.size(),sizeof(sample),&result));
      calibration=result.disposition;
      if(calibration!=ZJU_COOP_STATIC_IMU_READY) { samples.clear(); start=0; return false; }
      zju_coop_inertial_node_initialization_t initial;
      zju_coop_inertial_node_initialization_init(&initial); initial.node_id=state.node_id;
      std::copy_n(result.orientation_flu_to_enu_xyzw,4,initial.orientation_xyzw);
      std::copy_n(result.gyro_bias_flu_rad_s,3,initial.gyro_bias_rad_s);
      zju_coop_inertial_config_t cfg; zju_coop_inertial_config_init(&cfg);
      cfg.node_count=1; cfg.node_stride=sizeof(initial); cfg.nodes=&initial;
      std::strcpy(cfg.expected_frame_id,"imu_link");
      ok(zju_coop_configure_inertial(handle.get(),&cfg)); ready=true; samples.clear();
    }
    zju_coop_imu_packet_t packet; zju_coop_imu_packet_init(&packet);
    packet.node_id=state.node_id; packet.sequence=++sequence;
    packet.timestamp_ns=packet.receive_timestamp_ns=time; packet.valid=ZJU_COOP_TRUE;
    std::strcpy(packet.frame_id,"imu_link");
    std::copy_n(v.begin(),3,packet.angular_velocity_rad_s);
    std::copy_n(v.begin()+3,3,packet.linear_acceleration_m_s2);
    zju_coop_imu_processing_result_t result; zju_coop_imu_processing_result_init(&result);
    if(zju_coop_push_imu(handle.get(),&packet,&result)!=ZJU_COOP_OK) { ++rejected; return false; }
    if(time-published<20000000) return false;
    ok(zju_coop_get_node_state(handle.get(),state.node_id,&state));
    state.receive_timestamp_ns=time; published=time;
    return state.valid;
  }
};

int main(int argc,char** argv) {
  try {
    if(argc!=3 && argc!=4) throw std::runtime_error("usage: evaluate_rte input.csv output.csv [window_seconds]");
    std::ifstream in(argv[1]); std::ofstream out(argv[2]);
    if(!in || !out) throw std::runtime_error("cannot open input/output");
    std::array<Local,3> local{Local(1),Local(2),Local(3)};
    zju_coop_three_config_rte_t config; zju_coop_three_config_init_rte(&config);
    for(int n=0;n<3;++n) config.node_ids[n]=n+1;
    // 与ROS rte_fusion.yaml相同的128样本/边、5秒窗、200ms解算间隔、质量门限。
    config.max_samples_per_edge=128; config.window_duration_ns=5000000000ULL;
    if(argc==4) {
      const double seconds=std::stod(argv[3]);
      if(!std::isfinite(seconds) || seconds<1.0 || seconds>10.0)
        throw std::runtime_error("window_seconds must be between 1 and 10");
      config.window_duration_ns=static_cast<uint64_t>(seconds*1e9);
    }
    config.minimum_initialization_motion_span_m=10.0;
    config.solve_interval_ns=200000000;
    zju_coop_three_handle_rte_t* raw=nullptr; ok(zju_coop_three_create_rte(&config,&raw));
    std::unique_ptr<zju_coop_three_handle_rte_t,decltype(&zju_coop_three_destroy_rte)> rte(raw,zju_coop_three_destroy_rte);
    out<<"timestamp_ns,state_timestamp_ns,elapsed_s,status,initialized,corrections,rms_m,active_edges,samples_12,samples_13,samples_23,accepted_ranges,rejected_ranges,solve_ms";
    for(int n=1;n<=3;++n) out<<",local_ready_"<<n<<",calibration_"<<n<<",fault_"<<n<<",local_x_"<<n<<",local_y_"<<n<<",x_"<<n<<",y_"<<n<<",yaw_"<<n;
    out<<'\n'<<std::setprecision(17);
    uint64_t start=0,previous=0,last_step=0;
    auto step=[&](uint64_t time) {
      zju_coop_three_snapshot_rte_t s; zju_coop_three_snapshot_init_rte(&s);
      const auto begin=std::chrono::steady_clock::now();
      ok(zju_coop_three_step_rte(rte.get(),time,&s));
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      out<<time<<','<<s.timestamp_ns<<','<<(time-start)/1e9<<','<<s.status<<','<<(s.initialized_at_ns!=0)<<','<<s.correction_count<<','<<s.rms_residual_m<<','<<s.active_edges;
      for(auto count:s.sample_counts) out<<','<<count;
      out<<','<<s.accepted_ranges<<','<<s.rejected_ranges<<','<<ms;
      for(int n=0;n<3;++n) {
        const auto& l=local[n]; const auto* q=s.body_orientation_xyzw+4*n;
        const double yaw=std::atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]));
        out<<','<<l.ready<<','<<l.calibration<<','<<l.fault<<','<<l.state.position_enu_m[0]<<','<<l.state.position_enu_m[1]<<','<<s.relative_position_m[3*n]<<','<<s.relative_position_m[3*n+1]<<','<<yaw;
      }
      out<<'\n'; last_step=time;
    };
    std::string line; std::getline(in,line);
    while(std::getline(in,line)) {
      if(line.empty()) continue;
      std::replace(line.begin(),line.end(),',',' '); std::istringstream row(line);
      char kind; uint64_t time; int id,target; std::array<double,10> v{};
      if(!(row>>kind>>time>>id>>target) || id<1 || id>3 || !time) throw std::runtime_error("bad row");
      for(double& value:v) if(!(row>>value) || !std::isfinite(value)) throw std::runtime_error("bad numeric value");
      if(previous && time<previous) throw std::runtime_error("input is not header-time ordered");
      if(!start) start=time;
      if(previous && time!=previous && previous-last_step>=50000000) step(previous);
      auto& l=local[id-1];
      if(kind=='I') {
        if(l.imu(time,v)) ok(zju_coop_three_push_node_state_rte(rte.get(),&l.state));
      } else if(kind=='O') {
        if(l.previous && time<=l.previous) throw std::runtime_error("duplicate odometry");
        l.previous=time; l.ready=true; l.state.timestamp_ns=l.state.receive_timestamp_ns=time; l.state.valid=ZJU_COOP_TRUE;
        std::copy_n(v.begin(),3,l.state.position_enu_m); std::copy_n(v.begin()+3,3,l.state.velocity_enu_mps);
        std::copy_n(v.begin()+6,4,l.state.orientation_flu_to_enu_xyzw);
        ok(zju_coop_three_push_node_state_rte(rte.get(),&l.state));
      } else if(kind=='R') {
        if(target<1 || target>3 || target==id) throw std::runtime_error("bad range target");
        zju_coop_range_packet_t p; zju_coop_range_packet_init(&p);
        p.timestamp_ns=p.receive_timestamp_ns=time; p.from_node=id; p.to_node=target;
        p.range_m=v[0]; p.range_std_m=v[1]; p.valid=ZJU_COOP_TRUE;
        zju_coop_range_processing_result_t receipt; zju_coop_range_processing_result_init(&receipt);
        const auto rc=zju_coop_three_push_range_rte(rte.get(),&p,&receipt);
        if(rc!=ZJU_COOP_OK && rc!=ZJU_COOP_INVALID_ARGUMENT) ok(rc);
      } else throw std::runtime_error("unknown kind");
      previous=time;
    }
    if(previous) { step(previous); step(previous+1000000000ULL); }
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

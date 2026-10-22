#include "zju_coop/three_vehicle_rte_api.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>

#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)
using Vec = std::array<double, 3>;
using Session = std::unique_ptr<zju_coop_three_handle_rte_t, decltype(&zju_coop_three_destroy_rte)>;
constexpr std::array<std::array<int, 2>, 3> edges{{{0, 1}, {0, 2}, {1, 2}}};
constexpr double pi = 3.141592653589793;
const std::array<Vec, 3> origins{{{0, 0, 0}, {4, -1, .4}, {-2, 3, -.2}}};
const std::array<double, 3> headings{0, .8, -1.2};
Vec rotate(Vec p, double yaw) {
  return {std::cos(yaw)*p[0]-std::sin(yaw)*p[1], std::sin(yaw)*p[0]+std::cos(yaw)*p[1], p[2]};
}
Vec world(int n, Vec p) {
  p = rotate(p, headings[n]);
  for (int i=0; i<3; ++i) p[i] += origins[n][i];
  return p;
}
double distance(Vec a, Vec b) { return std::sqrt(std::pow(a[0]-b[0],2)+std::pow(a[1]-b[1],2)+std::pow(a[2]-b[2],2)); }
struct Motion { Vec p{}, v{}, a{}; double yaw{}, rate{}; };
Motion motion(int n, double t) {
  Motion out;
  if (t <= 2) return out;
  t -= 2;
  const double wx=.6+.13*n, wy=.43+.17*n, ax=2+.5*n, ay=1.5+.3*n;
  out.p = {ax*(1-std::cos(wx*t)), ay*(std::sin(wy*t)-wy*t), 0};
  out.v = {ax*wx*std::sin(wx*t), ay*wy*(std::cos(wy*t)-1), 0};
  out.a = {ax*wx*wx*std::cos(wx*t), -ay*wy*wy*std::sin(wy*t), 0};
  out.rate = .04*(n+1);
  out.yaw = out.rate*t;
  return out;
}
Session make_session(bool spatial=false) {
  zju_coop_three_config_rte_t c;
  CHECK(zju_coop_three_config_init_rte(&c) == ZJU_COOP_OK);
  for (int n=0;n<3;++n) c.local_origin_heights_m[n] = origins[n][2];
  c.planar = !spatial;
  c.solve_interval_ns = 500000000;
  c.window_duration_ns = 8000000000ULL;
  zju_coop_three_handle_rte_t* h=nullptr;
  CHECK(zju_coop_three_create_rte(&c, &h) == ZJU_COOP_OK);
  return Session(h, zju_coop_three_destroy_rte);
}
zju_coop_range_processing_result_t send_range(Session& session, int e, uint64_t time, double d) {
  zju_coop_range_packet_t packet;
  zju_coop_range_packet_init(&packet);
  packet.timestamp_ns = packet.receive_timestamp_ns = time;
  packet.from_node = 201+edges[e][0]; packet.to_node=201+edges[e][1];
  packet.range_m=d; packet.range_std_m=.02; packet.valid=ZJU_COOP_TRUE;
  zju_coop_range_processing_result_t result;
  zju_coop_range_processing_result_init(&result);
  CHECK(zju_coop_three_push_range_rte(session.get(), &packet, &result) == ZJU_COOP_OK);
  return result;
}
zju_coop_three_snapshot_rte_t step(Session& session, uint64_t time) {
  zju_coop_three_snapshot_rte_t out;
  zju_coop_three_snapshot_init_rte(&out);
  CHECK(zju_coop_three_step_rte(session.get(), time, &out) == ZJU_COOP_OK);
  return out;
}
void check_pose(Session& s, uint64_t now, bool valid) {
  zju_coop_pose2d_snapshot_v2_t header;
  zju_coop_pose2d_snapshot_v2_init(&header);
  std::array<zju_coop_vehicle_pose2d_v2_t,3> poses;
  for (auto& p:poses) zju_coop_vehicle_pose2d_v2_init(&p);
  uint32_t count=0;
  CHECK(zju_coop_three_get_pose2d_v2_rte(s.get(),now,&header,nullptr,0,0,&count)==ZJU_COOP_BUFFER_TOO_SMALL && count==3);
  CHECK(zju_coop_three_get_pose2d_v2_rte(s.get(),now,&header,poses.data(),3,sizeof(poses[0]),&count)==ZJU_COOP_OK);
  CHECK(std::strcmp(header.frame_id,"coop_rte_201_local")==0);
  for (int n=0;n<3;++n) {
    CHECK(poses[n].node_id==static_cast<uint32_t>(201+n));
    CHECK(static_cast<bool>(poses[n].position_valid)==valid);
    CHECK(static_cast<bool>(poses[n].yaw_valid)==valid);
    CHECK(std::isfinite(poses[n].x_m) && std::isfinite(poses[n].yaw_rad));
  }
}

void imu_end_to_end() {
  auto session=make_session();
  using INS=std::unique_ptr<zju_coop_handle_t, decltype(&zju_coop_destroy)>;
  std::array<INS,3> ins{{{nullptr,zju_coop_destroy},{nullptr,zju_coop_destroy},{nullptr,zju_coop_destroy}}};
  for (int n=0;n<3;++n) {
    zju_coop_node_initialization_t node;
    zju_coop_node_initialization_init(&node); node.node_id=201+n;
    zju_coop_config_t config;
    zju_coop_config_init(&config); config.reference_node_id=node.node_id;
    config.node_count=1; config.node_stride=sizeof(node); config.nodes=&node;
    zju_coop_handle_t* h=nullptr;
    CHECK(zju_coop_create(&config,&h)==ZJU_COOP_OK); ins[n].reset(h);
    zju_coop_inertial_node_initialization_t initial;
    zju_coop_inertial_node_initialization_init(&initial); initial.node_id=node.node_id;
    zju_coop_inertial_config_t inertial;
    zju_coop_inertial_config_init(&inertial);
    inertial.node_count=1; inertial.node_stride=sizeof(initial); inertial.nodes=&initial;
    std::strcpy(inertial.expected_frame_id,"imu_link");
    CHECK(zju_coop_configure_inertial(ins[n].get(),&inertial)==ZJU_COOP_OK);
  }
  double max_error=0, frozen_error=0, tracked_error=0, max_time=0;
  int max_node=0;
  zju_coop_three_snapshot_rte_t frozen{};
  bool initialized=false, dropout_seen=false, recovered=false;
  for (int tick=0;tick<=2400;++tick) {
    const double t=tick*.01;
    const uint64_t time=1000000000ULL+static_cast<uint64_t>(tick)*10000000ULL;
    std::array<Motion,3> truth;
    std::array<Vec,3> positions;
    std::array<zju_coop_node_state_t,3> state;
    for (int n=0;n<3;++n) {
      truth[n]=motion(n,t); positions[n]=world(n,truth[n].p);
      zju_coop_imu_packet_t imu;
      zju_coop_imu_packet_init(&imu);
      imu.node_id=201+n; imu.timestamp_ns=imu.receive_timestamp_ns=time;
      imu.sequence=tick+1; imu.valid=ZJU_COOP_TRUE;
      std::strcpy(imu.frame_id,"imu_link");
      const Vec specific=rotate(truth[n].a,-truth[n].yaw);
      imu.angular_velocity_rad_s[2]=truth[n].rate;
      imu.linear_acceleration_m_s2[0]=specific[0]+(n==1?.002:0);
      imu.linear_acceleration_m_s2[1]=specific[1]; imu.linear_acceleration_m_s2[2]=9.80665;
      zju_coop_imu_processing_result_t result;
      zju_coop_imu_processing_result_init(&result);
      CHECK(zju_coop_push_imu(ins[n].get(),&imu,&result)==ZJU_COOP_OK);
      CHECK(result.propagated || result.disposition==ZJU_COOP_IMU_BASELINE_ESTABLISHED);
      zju_coop_node_state_init(&state[n]);
      CHECK(zju_coop_get_node_state(ins[n].get(),201+n,&state[n])==ZJU_COOP_OK);
      if (tick%10==0) CHECK(zju_coop_three_push_node_state_rte(session.get(),&state[n])==ZJU_COOP_OK);
    }
    if (tick%10) continue;
    const bool dropout=t>16 && t<18;
    if (!dropout) {
      for (int e=0;e<3;++e) {
        const double d=distance(positions[edges[e][0]],positions[edges[e][1]]);
        const auto receipt=send_range(session,e,time,d+.003*std::sin(t+e));
        CHECK(receipt.disposition==ZJU_COOP_PROCESSING_PROCESSED);
      }
    }
    const auto out=step(session,time);
    if (t<2) CHECK(out.status!=ZJU_COOP_RTE_READY);
    if (t>16.6 && t<18) {
      CHECK(out.status==ZJU_COOP_RTE_STALE); check_pose(session,time,false); dropout_seen=true;
    }
    if (out.status==ZJU_COOP_RTE_READY) {
      if (!initialized) { frozen=out; initialized=true; std::printf("Three-RTE initialized at %.1fs\n",t); }
      if (t>18) recovered=true;
      CHECK(out.initialized_at_ns==frozen.initialized_at_ns);
      for (int n=1;n<3;++n) {
        const Vec expected{positions[n][0]-positions[0][0],positions[n][1]-positions[0][1],positions[n][2]-positions[0][2]};
        const Vec actual{out.relative_position_m[3*n],out.relative_position_m[3*n+1],out.relative_position_m[3*n+2]};
        if (distance(actual,expected)>max_error) {max_error=distance(actual,expected);max_time=t;max_node=n;}
        if (tick%200==0 && n==1) std::printf("t=%.1f error=%.3f yaw=%.3f rms=%.3f std=%.3f\n", t, distance(actual,expected), out.origin_yaw_rad[n],out.rms_residual_m,out.yaw_std_rad[n]);
        if (tick==2400 && n==1) {
          Vec p{state[n].position_enu_m[0],state[n].position_enu_m[1],state[n].position_enu_m[2]};
          p=rotate(p,frozen.origin_yaw_rad[n]);
          for (int axis=0;axis<3;++axis) p[axis]+=frozen.origin_translation_m[3*n+axis]-state[0].position_enu_m[axis];
          frozen_error=distance(p,expected); tracked_error=distance(actual,expected);
        }
      }
      check_pose(session,time,true);
    }
    if (tick==2400) {
      CHECK(out.status==ZJU_COOP_RTE_READY && out.correction_count>15);
      const auto rejected=send_range(session,0,time+1,100);
      CHECK(rejected.disposition==ZJU_COOP_PROCESSING_REJECTED);
      CHECK(send_range(session,0,time,1).disposition==ZJU_COOP_PROCESSING_DUPLICATE);
      CHECK(step(session,time).rejected_ranges>=2);
    }
  }
  std::printf("Three-RTE IMU pipeline: max position error %.4fm; frozen %.4fm -> tracked %.4fm\n",max_error,frozen_error,tracked_error);
  std::printf("Maximum error at t=%.1f node=%d\n",max_time,201+max_node);
  CHECK(initialized && dropout_seen && recovered && max_error<.5);
  CHECK(tracked_error<frozen_error);
}

void pending_and_spatial() {
  auto s=make_session(true);
  for(int tick=0;tick<65;++tick) {
    const uint64_t time=1000000000ULL+static_cast<uint64_t>(tick)*100000000ULL;
    std::array<Vec,3> position;
    for(int n=0;n<3;++n) position[n]={2*std::sin(.23*tick+n),std::cos(.31*tick+n),std::sin(.37*tick+2*n)};
    // 量测先到，稍后由step在收到状态后自动出队，不得丢弃。
    for(int e=0;e<3;++e) CHECK(send_range(s,e,time,distance(world(edges[e][0],position[edges[e][0]]),world(edges[e][1],position[edges[e][1]]))).disposition==ZJU_COOP_PROCESSING_HELD);
    for(int n=0;n<3;++n) {
      zju_coop_node_state_t node;
      zju_coop_node_state_init(&node); node.node_id=201+n; node.timestamp_ns=time;
      node.valid=ZJU_COOP_TRUE; node.orientation_flu_to_enu_xyzw[3]=1;
      std::copy(position[n].begin(),position[n].end(),node.position_enu_m);
      CHECK(zju_coop_three_push_node_state_rte(s.get(),&node)==ZJU_COOP_OK);
      CHECK(zju_coop_three_push_node_state_rte(s.get(),&node)==ZJU_COOP_INVALID_ARGUMENT);
    }
    const auto out=step(s,time);
    if(tick==64) {
      CHECK(out.status==ZJU_COOP_RTE_READY && out.accepted_ranges==195);
      for(int n=1;n<3;++n) {
        for(int a=0;a<3;++a) CHECK(std::abs(out.origin_translation_m[3*n+a]-origins[n][a])<.02);
        CHECK(std::abs(std::remainder(out.origin_yaw_rad[n]-headings[n],2*pi))<.02);
      }
      CHECK(step(s,time+1000000000ULL).status==ZJU_COOP_RTE_STALE);
    }
  }
}
int main() {
  try { pending_and_spatial(); imu_end_to_end(); std::puts("Three-RTE end-to-end self-check passed."); return 0; }
  catch(const std::exception& e) { std::fprintf(stderr,"Three-RTE failure: %s\n",e.what()); return 1; }
}

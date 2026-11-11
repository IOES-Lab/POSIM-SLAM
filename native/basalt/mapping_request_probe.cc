// SPDX-License-Identifier: Apache-2.0
#include "persistent_mapper.hpp"
#include <tbb/global_control.h>
#include <cassert>
#include <iostream>

namespace posim_slam {
struct ReadGate {
  std::mutex mutex;
  std::condition_variable changed;
  bool entered=false,released=false,fail=false;
  void read() {
    std::unique_lock<std::mutex> lock(mutex);entered=true;changed.notify_all();
    if(!changed.wait_for(lock,std::chrono::seconds(5),[&]{return released;}))
      throw std::runtime_error("probe_gate_timeout");
    if(fail)throw std::runtime_error("probe_mapping_source_failure");
  }
  void wait() {
    std::unique_lock<std::mutex> lock(mutex);
    assert(changed.wait_for(lock,std::chrono::seconds(5),[&]{return entered;}));
  }
  void release(){std::lock_guard<std::mutex> lock(mutex);released=true;changed.notify_all();}
};

// Use actual store commits and exclusive frame leases. Only their read is
// gated, so concurrency assertions do not depend on sleeping or CPU speed.
struct PersistentMapperProbe {
  static MappingInputStore::Snapshot append(PersistentMapper& map,const basalt::MargData& source,
      MappingInputStore::Snapshot previous={},int64_t stamp=0,std::shared_ptr<ReadGate> gate={}) {
    const auto committed=map.store_->append(source);
    auto staged=std::make_shared<MappingInputStore::Data>(*committed);
    if(previous)for(const auto& [time,frame]:previous->frames)staged->frames.at(time)=frame;
    if(gate) {
      const auto frame=staged->frames.at(stamp);
      staged->frames.at(stamp)=MappingFrameReference(frame.metadata(),[frame,gate](size_t maximum) {
        gate->read();return frame.read(maximum);
      });
    }
    std::lock_guard<std::mutex> lock(map.mutex_);
    map.retained_.input=staged;map.retained_.revision=staged->revision;map.changed_.notify_all();return staged;
  }
};
}
using namespace posim_slam;
static basalt::MargData sample(int64_t first,int64_t second) {
  basalt::MargData m;m.use_imu=true;m.kfs_all={first,second};m.kfs_to_marg={first};
  m.aom.abs_order_map={{first,{0,6}},{second,{6,6}}};m.aom.total_size=12;m.aom.items=2;
  m.abs_H=Eigen::MatrixXd::Identity(12,12);m.abs_b=Eigen::VectorXd::Zero(12);
  for(auto stamp:m.kfs_all) {
    m.frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,Sophus::SE3d()));
    auto flow=std::make_shared<basalt::OpticalFlowResult>();flow->t_ns=stamp;
    flow->input_images=std::make_shared<basalt::OpticalFlowInput>();flow->input_images->t_ns=stamp;
    flow->input_images->img_data.resize(1);
    auto image=std::make_shared<basalt::ManagedImage<uint16_t>>(4,4);image->Fill(100);
    flow->input_images->img_data[0].img=image;m.opt_flow_res.push_back(flow);
  }
  return m;
}
static PersistentMapper::Snapshot wait(PersistentMapper& map,uint64_t revision) {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  for(;;) {
    const auto snapshot=map.snapshot();
    if(!snapshot->error.empty() || snapshot->revision>=revision)return snapshot;
    assert(std::chrono::steady_clock::now()<deadline);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
int main() {
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  basalt::Calibration<double> calibration;calibration.intrinsics.resize(1);calibration.T_i_c.resize(1);
  calibration.resolution.push_back(Eigen::Vector2i(4,4));basalt::VioConfig config;
  PersistentMapLimits limits;limits.update_retirements=1;
  {
    PersistentMapper map(calibration,config,.1,limits);map.setReference(10,Sophus::SE3d());
    auto gate=std::make_shared<ReadGate>();
    const auto first=PersistentMapperProbe::append(map,sample(10,20),{},10,gate);gate->wait();
    PersistentMapperProbe::append(map,sample(20,30),first);
    assert(map.requestUpdate()==2);gate->release();
    const auto result=wait(map,2);
    assert(result->error.empty() && result->revision==2 && result->keyframes==3);
    assert(result->background_updates_superseded==1);
    assert(first->revision==1 && first->frames.size()==2);map.finish();
  }
  {
    PersistentMapper map(calibration,config,.1,limits);
    auto first_gate=std::make_shared<ReadGate>(),second_gate=std::make_shared<ReadGate>();
    const auto first=PersistentMapperProbe::append(map,sample(10,20),{},10,first_gate);
    assert(map.requestUpdate()==1);map.setReference(10,Sophus::SE3d());first_gate->wait();
    PersistentMapperProbe::append(map,sample(20,30),first,30,second_gate);
    assert(map.requestUpdate()==2);first_gate->release();second_gate->wait();
    const auto progress=map.snapshot();
    assert(progress->revision==1 && progress->error.empty() && progress->background_updates_superseded==0);
    second_gate->release();const auto final=wait(map,2);
    assert(final->revision==2 && final->error.empty() && final->background_updates_superseded==0);map.finish();
  }
  {
    PersistentMapper map(calibration,config,.1,limits);map.setReference(10,Sophus::SE3d());
    auto gate=std::make_shared<ReadGate>();gate->fail=true;
    const auto first=PersistentMapperProbe::append(map,sample(10,20),{},10,gate);gate->wait();
    PersistentMapperProbe::append(map,sample(20,30),first);assert(map.requestUpdate()==2);gate->release();
    const auto failed=wait(map,2);
    assert(failed->error=="probe_mapping_source_failure" && failed->background_updates_superseded==0);map.finish();
  }
  std::cout<<"Map request priority, finite requested progress and genuine source failures pass.\n";
}

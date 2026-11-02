// SPDX-License-Identifier: Apache-2.0
#include "mapping_input_store.hpp"
#include <cassert>
#include <iostream>

namespace {
using Store=posim_slam::MappingInputStore;
basalt::MargData::Ptr sample(int64_t first=10,int64_t second=20,bool inertial=false) {
  auto m=std::make_shared<basalt::MargData>();m->use_imu=true;
  m->kfs_all={first,second};m->kfs_to_marg={first};
  m->aom.abs_order_map={{first,{0,6}},{second,{6,inertial?15:6}}};
  m->aom.total_size=inertial?21:12;m->aom.items=2;
  m->abs_H=2*Eigen::MatrixXd::Identity(m->aom.total_size,m->aom.total_size);
  m->abs_b=Eigen::VectorXd::LinSpaced(m->aom.total_size,-.1,.1);
  if(inertial) {
    // Velocity and bias cross information must survive the same elimination
    // as the original retained-input mapper, rather than being discarded.
    m->abs_H(0,12)=m->abs_H(12,0)=.25;
    m->abs_H(4,17)=m->abs_H(17,4)=.3;
  }
  for(int64_t stamp:m->kfs_all) {
    if(inertial && stamp==second)
      m->frame_states.emplace(stamp,basalt::PoseVelBiasStateWithLin<double>(stamp,
        Sophus::SE3d(),Eigen::Vector3d(.1,.2,.3),Eigen::Vector3d(.01,.02,.03),Eigen::Vector3d(.04,.05,.06),false));
    else m->frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,Sophus::SE3d()));
    auto f=std::make_shared<basalt::OpticalFlowResult>();f->t_ns=stamp;
    f->input_images=std::make_shared<basalt::OpticalFlowInput>();f->input_images->t_ns=stamp;
    f->input_images->img_data.resize(1);
    f->input_images->img_data[0].img=std::make_shared<basalt::ManagedImage<uint16_t>>(4,4);
    f->input_images->img_data[0].img->Fill(100);m->opt_flow_res.push_back(f);
  }
  return m;
}
template<class F> void rejects(F action,const std::string& expected) {
  bool rejected=false;
  try {action();}catch(const std::exception& error){assert(error.what()==expected);rejected=true;}
  assert(rejected);
}
template<class A,class B> void equal(const A& a,const B& b) {
  assert(a.rows()==b.rows() && a.cols()==b.cols() && (a.array()==b.array()).all());
}
void compare(const Store::Snapshot& compact,const basalt::NfrMapper& raw) {
  assert(compact->relative.size()==raw.rel_pose_factors.size());
  assert(compact->roll_pitch.size()==raw.roll_pitch_factors.size());
  for(size_t i=0;i<compact->relative.size();++i) {
    const auto& a=compact->relative[i];const auto& b=raw.rel_pose_factors[i];
    assert(a.t_i_ns==b.t_i_ns && a.t_j_ns==b.t_j_ns);
    equal(a.T_i_j.matrix(),b.T_i_j.matrix());equal(a.cov_inv,b.cov_inv);
  }
  for(size_t i=0;i<compact->roll_pitch.size();++i) {
    const auto& a=compact->roll_pitch[i];const auto& b=raw.roll_pitch_factors[i];
    assert(a.t_ns==b.t_ns);equal(a.R_w_i_meas.matrix(),b.R_w_i_meas.matrix());equal(a.cov_inv,b.cov_inv);
  }
}
} // namespace

int main() {
  basalt::Calibration<double> calibration;calibration.intrinsics.resize(1);calibration.T_i_c.resize(1);
  calibration.resolution.push_back(Eigen::Vector2i(4,4));
  basalt::VioConfig config;
  for(bool inertial:{false,true}) {
    Store store(calibration,config);auto source=sample(10,20,inertial);
    const Eigen::MatrixXd h=source->abs_H;const Eigen::VectorXd b=source->abs_b;
    const auto pixel_hash=Store::imageDigest(*source->opt_flow_res[0]->input_images->img_data[0].img);
    std::weak_ptr<basalt::ManagedImage<uint16_t>> borrowed=source->opt_flow_res[0]->input_images->img_data[0].img;
    const auto first=store.append(*source);equal(h,source->abs_H);equal(b,source->abs_b);
    assert(pixel_hash==Store::imageDigest(*source->opt_flow_res[0]->input_images->img_data[0].img));
    basalt::NfrMapper raw(calibration,Store::configuration(config));
    auto owned=std::make_shared<basalt::MargData>(*source);raw.addMargData(owned);compare(first,raw);
    source.reset();owned.reset();raw.img_data.clear();assert(borrowed.expired());
    auto second=sample(20,30);store.append(*second);
    assert(first->revision==1 && first->frames.size()==2 && !first->frames.count(30));
    assert(store.snapshot()->revision==2 && store.snapshot()->frames.size()==3);
    assert(store.makeMapper(first)->img_data.empty());
    const auto before=store.snapshot();rejects([&]{store.append(*second);},"mapping_retirement_reused");
    assert(store.snapshot()==before);
    Store other(calibration,config);
    rejects([&]{other.makeMapper(first);},"mapping_input_snapshot_owner");
    rejects([&]{store.makeMapper(nullptr);},"mapping_input_snapshot_owner");
  }
  {
    Store store(calibration,config);const auto first=store.append(*sample());
    auto changed=sample();changed->kfs_to_marg={20};
    (*changed->opt_flow_res[0]->input_images->img_data[0].img)(0,0)=101;
    rejects([&]{store.append(*changed);},"mapping_image_identity_changed");assert(store.snapshot()==first);
  }
  {
    posim_slam::MappingInputLimits limits;limits.compact_bytes=2;
    Store store(calibration,config,limits);const auto empty=store.snapshot();
    rejects([&]{store.append(*sample());},"mapping_compact_input_capacity");assert(store.snapshot()==empty);
  }
  {
    posim_slam::MappingInputLimits limits;limits.retirements=limits.update_retirements=1;
    Store store(calibration,config,limits);const auto first=store.append(*sample());
    rejects([&]{store.append(*sample(20,30));},"mapping_retirement_capacity");assert(store.snapshot()==first);
  }
  std::cout<<"Compact mapping inputs: exact 6/15-state recovery, immutable snapshots, image ownership/identity and atomic capacity failures pass.\n";
}

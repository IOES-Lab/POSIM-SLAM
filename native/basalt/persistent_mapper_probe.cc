// SPDX-License-Identifier: Apache-2.0
#include "persistent_mapper.hpp"
#include <cassert>
#include <iostream>
basalt::MargData::Ptr sample(int64_t first=10,int64_t second=20) {
 auto m=std::make_shared<basalt::MargData>();m->use_imu=true;
 m->kfs_all={first,second};m->kfs_to_marg={first};
 m->aom.abs_order_map={{first,{0,6}},{second,{6,6}}};m->aom.total_size=12;m->aom.items=2;
 m->abs_H=Eigen::MatrixXd::Identity(12,12);m->abs_b=Eigen::VectorXd::Zero(12);
 for(auto stamp:m->kfs_all) {
  m->frame_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,Sophus::SE3d()));
  auto f=std::make_shared<basalt::OpticalFlowResult>();f->t_ns=stamp;
  f->input_images=std::make_shared<basalt::OpticalFlowInput>();f->input_images->t_ns=stamp;
  f->input_images->img_data.resize(1);
  f->input_images->img_data[0].img=std::make_shared<basalt::ManagedImage<uint16_t>>(4,4);
  f->input_images->img_data[0].img->Fill(100);m->opt_flow_res.push_back(f);
 }
 return m;
}
int main(int argc,char** argv) {
 size_t archive_id=0;
 auto archive=[&]() -> std::filesystem::path {
  if(argc!=2)return {};
  return std::filesystem::absolute(argv[1])/("mapper-"+std::to_string(++archive_id)+".sqlite");
 };
 basalt::Calibration<double> calibration;calibration.intrinsics.resize(1);calibration.T_i_c.resize(1);
 calibration.resolution.push_back(Eigen::Vector2i(4,4));
 basalt::VioConfig config;
 {
  posim_slam::PersistentMapLimits limits;limits.update_retirements=limits.retirements;
  posim_slam::PersistentMapper map(calibration,config,.1,limits,{},archive());map.setReference(10,Sophus::SE3d());
  map.input.push(sample());
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  uint64_t requested=0;
  while(!requested && std::chrono::steady_clock::now()<deadline) {
   requested=map.requestUpdate();std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(requested==1);
  while(map.snapshot()->revision<requested && map.snapshot()->error.empty() &&
      std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  assert(map.snapshot()->revision>=requested && map.snapshot()->error.empty());
  map.finish();assert(map.requestUpdate()==0);
 }
 auto run=[&](std::vector<basalt::MargData::Ptr> values,const std::string& expected,size_t frames=2) {
  posim_slam::PersistentMapLimits limits;limits.keyframes=frames;limits.update_retirements=limits.retirements;
  posim_slam::PersistentMapper map(calibration,config,.1,limits,{},archive());map.setReference(10,Sophus::SE3d());
  for(auto value:values)map.input.push(value);
  map.finish();map.finish();const auto result=map.snapshot();
  assert(result->error==expected && result->points.empty() && result->observed==0);
 };
 auto first=sample();run({first,first},"mapping_retirement_reused");
 auto changed=sample();changed->kfs_to_marg={20};(*changed->opt_flow_res[0]->input_images->img_data[0].img)(0,0)=900;
 run({first,changed},"mapping_image_identity_changed");
 run({first,sample(20,30)},"mapping_image_capacity");
 auto malformed=sample();malformed->aom.abs_order_map.at(20).first=5;
 run({malformed},"overlapping_mapping_state_coordinates");
 auto nonfinite=sample();nonfinite->abs_H(0,0)=std::numeric_limits<double>::quiet_NaN();
 run({nonfinite},"invalid_persistent_mapping_input");
 {
  posim_slam::PersistentMapper map(calibration,config,.1,{}, {},archive());
  map.invalidate("mapping_handoff_backlog");
  for(int i=0;i<30;++i)map.input.push(first);
  map.finish();assert(map.snapshot()->error=="mapping_handoff_backlog" && map.snapshot()->points.empty());
 }
 if(argc==2) {
  posim_slam::PersistentMapper map(calibration,config,.1,{}, {},
    std::filesystem::absolute(argv[1])/"absent-parent"/"inputs.sqlite");
  for(int i=0;i<30;++i)map.input.push(first);
  map.finish();assert(map.snapshot()->error=="mapping_input_storage_path" && map.snapshot()->points.empty());
 }
 std::cout<<"Persistent mapping: partial-batch update, duplicate ownership, image identity, budgets, invalid layout and failure draining pass.\n";
}

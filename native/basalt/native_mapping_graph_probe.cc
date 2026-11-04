// SPDX-License-Identifier: Apache-2.0
#include "native_mapping_graph.hpp"
#include <tbb/global_control.h>
#include <cassert>
#include <iostream>
#include <random>
using namespace posim_slam;
template<class F> void rejects(F call,const std::string& expected) {
  bool caught=false;try{call();}catch(const std::exception& error){assert(error.what()==expected);caught=true;}assert(caught);
}
int main() {
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  basalt::Calibration<double> calibration;auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(400,400,320,240));calibration.intrinsics.push_back(camera);
  calibration.T_i_c.emplace_back();calibration.resolution.emplace_back(640,480);basalt::VioConfig config;
  config.mapper_min_track_length=2;basalt::HashBow<256> vocabulary(config.mapper_bow_num_bits);
  std::map<int64_t,MappingFrameReference> frames;
  for(int64_t stamp=1;stamp<=6;++stamp) {
    auto frame=std::make_shared<MappingInputFrame>();frame->stamp=stamp;frame->width=640;frame->height=480;frame->digest.fill(stamp);
    auto& features=frame->features;std::mt19937 random(37);
    for(int i=0;i<64;++i) {
      const Eigen::Vector3d point((i%8-3.5)*.3-stamp*.2,(i/8-3.5)*.25,4+.05*(i%11));
      features.corners.emplace_back(400*point.x()/point.z()+320,400*point.y()/point.z()+240);
      features.corner_angles.push_back(0);features.corner_descriptors.emplace_back();
      for(size_t bit=0;bit<256;++bit)features.corner_descriptors.back()[bit]=random()&1;
    }
    std::vector<bool> success;camera.unproject(features.corners,features.corners_3d,success);
    vocabulary.compute_bow(features.corner_descriptors,features.hashes,features.bow_vector);frames.emplace(stamp,frame);
  }
  auto batch=[&](int count) {
    auto input=std::make_shared<MappingInputStore::Data>();input->revision=count/2;
    for(int64_t stamp=1;stamp<=count;++stamp) {
      input->frames.emplace(stamp,frames.at(stamp));const Sophus::SE3d pose(Sophus::SO3d(),Eigen::Vector3d(stamp*.2,0,0));
      input->poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose));
      if(stamp>1) {
        basalt::RelPoseFactor factor;factor.t_i_ns=stamp-1;factor.t_j_ns=stamp;
        factor.T_i_j=Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.2,0,0));factor.cov_inv=100*Eigen::Matrix<double,6,6>::Identity();
        input->relative.push_back(factor);
      }
      basalt::RollPitchFactor factor;factor.t_ns=stamp;factor.R_w_i_meas=Sophus::SO3d();factor.cov_inv=100*Eigen::Matrix2d::Identity();
      input->roll_pitch.push_back(factor);
    }
    return input;
  };
  const auto first=batch(2),second=batch(4),third=batch(6);
  MappingMatchHistory matching(calibration,config);NativeMappingGraph graph(matching);const auto address=&graph.mapper();
  auto result=graph.update(first,0);assert(result.new_frames==2 && result.initialized_landmarks==64);
  assert(graph.mapper().feature_tracks.size()==64 && graph.mapper().lmdb.numObservations()==128);
  const auto id=graph.mapper().lmdb.getLandmarks().begin()->first;
  auto& point=graph.mapper().lmdb.getLandmark(id);point.direction.x()+=.001;point.inv_dist*=1.01;
  const Eigen::Vector3d parameters(point.direction.x(),point.direction.y(),point.inv_dist);
  const auto state=nativeMappingStateIdentity(graph.mapper()),source=nativeMappingSourceIdentity(graph.mapper());
  const auto committed_matching=graph.matchingSnapshot();const double lambda=graph.mapper().lambda;
  assert(graph.update(first,0).reused && nativeMappingStateIdentity(graph.mapper())==state);
  int new_state_seen=0;
  rejects([&]{graph.update(second,10,[&]{return graph.mapper().getFramePoses().size()>2 && ++new_state_seen>=2;});},"native_graph_cancelled");
  assert(new_state_seen==2 && &graph.mapper()==address && graph.lastUpdate().revision==1 && graph.matchingSnapshot()==committed_matching);
  assert(nativeMappingStateIdentity(graph.mapper())==state && nativeMappingSourceIdentity(graph.mapper())==source && graph.mapper().lambda==lambda);
  result=graph.update(second,0);
  const auto& kept=graph.mapper().lmdb.getLandmark(id);
  assert(Eigen::Vector3d(kept.direction.x(),kept.direction.y(),kept.inv_dist)==parameters);
  assert(&graph.mapper()==address && result.new_frames==2 && result.preserved_landmarks==64 && result.initialized_landmarks==0 && result.extended_tracks==64);
  assert(graph.mapper().lmdb.numObservations()==256 && graph.mapper().rel_pose_factors.size()==3);
  auto broken=std::make_shared<MappingInputStore::Data>(*third);broken->relative.front().cov_inv*=2;
  const auto before=nativeMappingStateIdentity(graph.mapper());
  rejects([&]{graph.update(broken,0);},"native_graph_source_changed");
  assert(graph.lastUpdate().revision==2 && nativeMappingStateIdentity(graph.mapper())==before);
  broken=std::make_shared<MappingInputStore::Data>(*third);broken->relative.push_back(broken->relative.back());
  rejects([&]{graph.update(broken,0);},"native_graph_factor");
  result=graph.update(third,10);assert(&graph.mapper()==address && result.new_frames==2 && result.preserved_landmarks==64);
  assert(graph.mapper().feature_tracks.size()==64 && graph.mapper().lmdb.numObservations()==384 && graph.mapper().rel_pose_factors.size()==5);
  MappingMatchHistory limited_matching(calibration,config);NativeMappingGraph limited(limited_matching,1);
  rejects([&]{limited.update(first,0);},"native_graph_numeric_capacity");
  assert(!limited.lastUpdate().revision && limited.mapper().getFramePoses().empty() && !limited.mapper().lmdb.numLandmarks());
  graph.mapper().lmdb.getLandmark(id).obs.begin()->second.x()+=1;
  rejects([&]{graph.update(third,0);},"native_graph_originals_changed");
  std::cout<<"Persistent native graph: one mapper, preserved nonlinear seeds, new observations, no-input reuse, full post-optimization rollback, source/factor/capacity guards pass.\n";
}

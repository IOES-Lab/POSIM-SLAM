// SPDX-License-Identifier: Apache-2.0
#include "native_mapping_graph.hpp"
#include "mapping_graph.hpp"
#include "native_mapping_window.hpp"
#include <tbb/global_control.h>
#include <cassert>
#include <iostream>
#include <random>
using namespace posim_slam;
template<class F> void rejects(F call,const std::string& expected) {
  bool caught=false;try{call();}catch(const std::exception& error){assert(error.what()==expected);caught=true;}assert(caught);
}
int main(int argc,char** argv) {
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  basalt::Calibration<double> calibration;auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(400,400,320,240));calibration.intrinsics.push_back(camera);
  calibration.T_i_c.emplace_back();calibration.resolution.emplace_back(640,480);basalt::VioConfig config;
  config.mapper_min_track_length=2;basalt::HashBow<256> vocabulary(config.mapper_bow_num_bits);
  std::map<int64_t,MappingFrameReference> frames;
  for(int64_t stamp=1;stamp<=6;++stamp) {
    auto frame=std::make_shared<MappingInputFrame>();frame->stamp=stamp;frame->width=640;frame->height=480;frame->digest.fill(stamp);
    auto& features=frame->features;std::mt19937 random(37);
    for(int i=0;i<(stamp>=4?80:64);++i) {
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
  assert(matching.snapshot()==committed_matching);
  result=graph.update(second,0);
  const auto& kept=graph.mapper().lmdb.getLandmark(id);
  assert(Eigen::Vector3d(kept.direction.x(),kept.direction.y(),kept.inv_dist)==parameters);
  assert(&graph.mapper()==address && result.new_frames==2 && result.preserved_landmarks==64 && result.initialized_landmarks==0 && result.extended_tracks==64);
  assert(graph.mapper().lmdb.numObservations()==256 && graph.mapper().rel_pose_factors.size()==3);
  graph.mapper().lambda=7; // Preserve current solver controls, not constructor defaults.
  graph.mapper().getFramePoses().at(4)=basalt::PoseStateWithLin<double>(4,
      Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.803,0,0)));
  MappingGraphRegistry registry(calibration,config,"native-graph-checkpoint");
  const auto captured=registry.capture(second,graph.mapper(),1,second->poses.at(1).getPose(),Sophus::SE3d(),nullptr,&graph);
  const auto document=MappingGraphRegistry::document(captured);
  const auto native_saved=NativeMappingGraph::checkpoint(*graph.captureState(second));
  assert(MappingGraphRegistry::restoreInputs(captured)==second);
  MappingMatchHistory restored_matching(calibration,config);
  const auto empty=restored_matching.snapshot();
  auto wrong=MappingGraphRegistry::restore(document);wrong->lmdb.getLandmark(id).inv_dist*=1.02;
  rejects([&]{NativeMappingGraph::restore(restored_matching,second,native_saved,std::move(wrong));},"native_graph_checkpoint_source");
  auto wrong_seeds=std::make_shared<MappingInputStore::Data>(*second);
  wrong_seeds->poses.at(4)=basalt::PoseStateWithLin<double>(4,Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.81,0,0)));
  rejects([&]{MappingGraphRegistry::restoreNativeGraph(captured,restored_matching,wrong_seeds);},"native_graph_checkpoint_source");
  rejects([&]{MappingGraphRegistry::restoreNativeGraph(captured,restored_matching,second,[]{return true;});},"mapping_match_cancelled");
  assert(restored_matching.snapshot()==empty);
  auto restored=MappingGraphRegistry::restoreNativeGraph(captured,restored_matching,second);
  assert(nativeMappingStateIdentity(restored->mapper())==nativeMappingStateIdentity(graph.mapper()));
  assert(nativeMappingSourceIdentity(restored->mapper())==nativeMappingSourceIdentity(graph.mapper()));
  assert(restored->mapper().lambda==7 && restored->update(second,0).reused);
  assert(restored->mapper().feature_corners.at(basalt::TimeCamId(4,0)).corners==frames.at(4).read()->features.corners);
  assert(NativeMappingGraph::checkpoint(*restored->captureState(second))==native_saved);
  if(argc==2) {
    const std::filesystem::path directory=argv[1],path=directory/"native.pmap";
    MappingGraphRegistry::writeWithInputs(path,captured,1,4);
    const auto loaded=MappingGraphRegistry::read(path,"native-graph-checkpoint");
    assert(loaded.graph->hasNativeGraph() && loaded.graph->hasInputFrames());
    const auto inputs=MappingGraphRegistry::restoreInputs(loaded.graph);
    assert(inputs->poses.at(4).getPose().translation().x()==.8);
    MappingMatchHistory disk_matching(calibration,config);
    const auto disk=MappingGraphRegistry::restoreNativeGraph(loaded.graph,disk_matching,inputs);
    assert(NativeMappingGraph::checkpoint(*disk->captureState(inputs))==native_saved);
    assert(disk->mapper().feature_corners.at(basalt::TimeCamId(4,0)).corners==frames.at(4).read()->features.corners);
    MappingGraphRegistry::write(directory/"graph-only.pmap",captured,1,4);
    const auto plain=MappingGraphRegistry::read(directory/"graph-only.pmap");
    assert(!plain.graph->hasNativeGraph() && !plain.graph->hasInputFrames());
    rejects([&]{MappingGraphRegistry::restoreInputs(plain.graph);},"native_graph_checkpoint_missing");
    std::filesystem::copy_file(path,directory/"corrupted.pmap",std::filesystem::copy_options::overwrite_existing);
    {std::fstream file(directory/"corrupted.pmap",std::ios::in|std::ios::out|std::ios::binary);
      file.seekg(-1,std::ios::end);char byte;file.get(byte);byte^=1;file.seekp(-1,std::ios::end);file.put(byte);}
    const auto corrupt=MappingGraphRegistry::read(directory/"corrupted.pmap");
    const auto corrupt_inputs=MappingGraphRegistry::restoreInputs(corrupt.graph);MappingMatchHistory untouched(calibration,config);
    rejects([&]{MappingGraphRegistry::restoreNativeGraph(corrupt.graph,untouched,corrupt_inputs);},"mapping_input_page_digest");
    assert(!untouched.snapshot()->revision);
  }
  MappingGraphRegistry restored_registry(calibration,config,"native-graph-checkpoint");restored_registry.adopt(captured);
  auto broken=std::make_shared<MappingInputStore::Data>(*third);broken->relative.front().cov_inv*=2;
  const auto before=nativeMappingStateIdentity(graph.mapper());
  rejects([&]{graph.update(broken,0);},"native_graph_source_changed");
  assert(graph.lastUpdate().revision==2 && nativeMappingStateIdentity(graph.mapper())==before);
  broken=std::make_shared<MappingInputStore::Data>(*third);broken->relative.push_back(broken->relative.back());
  rejects([&]{graph.update(broken,0);},"native_graph_factor");
  result=graph.update(third,10);assert(&graph.mapper()==address && result.new_frames==2 && result.preserved_landmarks==64);
  assert(graph.mapper().feature_tracks.size()==80 && graph.mapper().lmdb.numObservations()==432 && graph.mapper().rel_pose_factors.size()==5);
  const auto resumed=restored->update(third,10);assert(resumed.new_tracks==16 && resumed.preserved_landmarks==64 && resumed.initialized_landmarks==16);
  assert(restored->mapper().feature_tracks==graph.mapper().feature_tracks);
  assert(nativeMappingSourceIdentity(restored->mapper())==nativeMappingSourceIdentity(graph.mapper()));
  for(const auto& [track,point]:graph.mapper().lmdb.getLandmarks()) {
    const auto& other=restored->mapper().lmdb.getLandmark(track);
    assert((point.direction-other.direction).norm()<1e-10 && std::abs(point.inv_dist-other.inv_dist)<1e-10);
  }
  const auto next=registry.capture(third,graph.mapper(),1,third->poses.at(1).getPose(),Sophus::SE3d(),nullptr,&graph);
  const auto resumed_next=restored_registry.capture(third,restored->mapper(),1,third->poses.at(1).getPose(),Sophus::SE3d(),nullptr,restored.get());
  assert(next->stable_ids==resumed_next->stable_ids);
  MappingMatchHistory limited_matching(calibration,config);NativeMappingGraph limited(limited_matching,1);
  rejects([&]{limited.update(first,0);},"native_graph_numeric_capacity");
  assert(!limited.lastUpdate().revision && limited.mapper().getFramePoses().empty() && !limited.mapper().lmdb.numLandmarks());
  assert(!limited_matching.snapshot()->revision);
  MappingMatchHistory window_matching(calibration,config);NativeMappingGraph rolling(window_matching,0,2);
  rolling.update(first,0);assert(rolling.window());
  const auto old_window=rolling.window();const auto old_numeric=old_window->snapshot();
  const auto old_matching=window_matching.snapshot();const auto old_state=nativeMappingStateIdentity(rolling.mapper());
  const auto context=old_window->worldLinearization();const auto first_id=rolling.mapper().lmdb.getLandmarks().begin()->first;
  int staged_checks=0;
  rejects([&]{rolling.update(second,1,[&]{return rolling.mapper().getFramePoses().size()>2 && ++staged_checks>12;});},"submap_factor_cancelled");
  assert(rolling.window()==old_window && rolling.window()->snapshot()==old_numeric);
  assert(window_matching.snapshot()==old_matching && nativeMappingStateIdentity(rolling.mapper())==old_state);
  rolling.update(second,1);
  assert(rolling.window()->chart().offsets().at(1)==0 && rolling.window()->chart().offsets().at(2)==6);
  rejects([&]{rolling.window()->worldInformation(context,{int64_t(first_id)});},"native_window_stale_world_linearization");
  rolling.update(third,1);
  assert(rolling.window()->chart().poses().size()==6 && rolling.window()->snapshot()->layout.frontier.size()==12);
  assert(rolling.window()->snapshot()->information->measurements.size()==rolling.mapper().lmdb.numObservations()+11);
  assert(rolling.window()->mapInformation(.3).points.size()==80);
  rejects([&]{rolling.captureState(third);},"native_graph_window_checkpoint_required");
  MappingGraphRegistry window_registry(calibration,config,"growing-window");
  rejects([&]{window_registry.capture(third,rolling.mapper(),1,third->poses.at(1).getPose(),Sophus::SE3d(),nullptr,&rolling);},
      "native_graph_window_checkpoint_required");
  std::cout<<"Source window: actual native growth, stable pose coordinates, shared source ownership, covariance queries and matching/native/window cancellation rollback pass.\n";
  graph.mapper().lmdb.getLandmark(id).obs.begin()->second.x()+=1;
  rejects([&]{graph.update(third,0);},"native_graph_originals_changed");
  std::cout<<"Persistent native graph: one mapper, preserved nonlinear seeds, new observations, no-input reuse, full post-optimization rollback, source/factor/capacity guards pass.\n";
  std::cout<<"Native graph checkpoint: exact nonlinear state, solver controls, matching decisions and next track ID; continued growth and registry identities; source/cancellation rejection pass.\n";
}

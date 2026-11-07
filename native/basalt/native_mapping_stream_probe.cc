// SPDX-License-Identifier: Apache-2.0
#include "native_mapping_graph.hpp"
#include "mapping_graph.hpp"
#include "mapping_input_archive.hpp"
#include "native_mapping_window.hpp"
#include "submap_row_store.hpp"
#include <tbb/global_control.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <random>

namespace {
using namespace posim_slam;
using Json=nlohmann::json;
constexpr size_t bytes=96*1024*1024,cache_bytes=2*1024*1024;
constexpr size_t frames=240,features=64,views=4;
template<class F> void rejects(F call,const std::string& expected) {
  bool caught=false;
  try {call();} catch(const std::exception& error) {
    if(error.what()!=expected)std::cerr<<"Expected "<<expected<<", received "<<error.what()<<'\n';
    assert(error.what()==expected);caught=true;
  }
  assert(caught);
}
basalt::Calibration<double> calibration() {
  basalt::Calibration<double> result;auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(100,100,80,60));result.intrinsics.push_back(camera);
  result.T_i_c.emplace_back();result.resolution.emplace_back(160,120);return result;
}
Eigen::Vector3d truth(int64_t stamp,int64_t feature) {
  return {double((stamp-1)/views)*views*.2+(feature%8-3.5)*.3,
      (feature/8-3.5)*.25,4+.05*(feature%11)};
}
// Projected detector features isolate matching/graph ownership from image
// detection. Each four-view group sees distinct landmarks; no truth enters
// matching or covariance. Actual image ingestion has a separate producer test.
struct Stream {
  basalt::Calibration<double> calib=calibration();
  basalt::VioConfig config=MappingInputStore::configuration(basalt::VioConfig{});
  basalt::HashBow<256> vocabulary{size_t(config.mapper_bow_num_bits)};
  std::shared_ptr<MappingInputArchive> archive;
  uint64_t page_revision=0;
  explicit Stream(const std::filesystem::path& file)
      :archive(std::make_shared<MappingInputArchive>(file,Json{{"fixture","projected-stream"}},false)) {}
  MappingInputStore::Snapshot append(const MappingInputStore::Snapshot& previous,size_t count) {
    auto next=previous?std::make_shared<MappingInputStore::Data>(*previous):std::make_shared<MappingInputStore::Data>();
    for(int64_t stamp=next->frames.size()+1;stamp<=int64_t(count);++stamp) {
      MappingInputFrame frame;frame.stamp=stamp;frame.width=160;frame.height=120;frame.digest.fill(uint8_t(stamp));
      const Sophus::SE3d pose(Sophus::SO3d(),Eigen::Vector3d(stamp*.2,0,0));
      std::mt19937 random(uint32_t(37+(stamp-1)/views));
      for(int64_t i=0;i<int64_t(features);++i) {
        Eigen::Vector2d pixel;assert(calib.intrinsics[0].project(pose.inverse()*truth(stamp,i),pixel));
        auto& f=frame.features;f.corners.push_back(pixel);f.corner_angles.push_back(0);f.corner_descriptors.emplace_back();
        for(size_t bit=0;bit<256;++bit)f.corner_descriptors.back()[bit]=random()&1;
      }
      auto& f=frame.features;std::vector<bool> valid;
      calib.intrinsics[0].unproject(f.corners,f.corners_3d,valid);assert(std::all_of(valid.begin(),valid.end(),[](bool x){return x;}));
      vocabulary.compute_bow(f.corner_descriptors,f.hashes,f.bow_vector);
      auto page=MappingInputArchive::page(frame);const auto identity=page.identity;
      archive->commit(++page_revision,Json{{"stamp",stamp}},{std::move(page)});
      auto reference=archive->reference(identity,page_revision);assert(!reference.resident());
      auto lease=reference.read();assert(mapping_frame_io::encode(*lease)==mapping_frame_io::encode(frame));
      std::weak_ptr<const MappingInputFrame> weak=lease;lease.reset();assert(weak.expired());
      next->frames.emplace(stamp,std::move(reference));next->poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose));
      if(stamp>1) {
        basalt::RelPoseFactor factor;factor.t_i_ns=stamp-1;factor.t_j_ns=stamp;
        factor.T_i_j=Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.2,0,0));
        factor.cov_inv=100*Eigen::Matrix<double,6,6>::Identity();next->relative.push_back(factor);
      }
      basalt::RollPitchFactor tilt;tilt.t_ns=stamp;tilt.R_w_i_meas=Sophus::SO3d();
      tilt.cov_inv=100*Eigen::Matrix2d::Identity();next->roll_pitch.push_back(tilt);
      next->source_image_bytes+=160*120*sizeof(uint16_t);
    }
    next->revision=count;assert(next->source_image_bytes<bytes);return next;
  }
};
Json output(NativeMappingGraph& graph,const MappingGraphRegistry::Snapshot& saved) {
  const auto map=graph.window()->mapInformation(.3);
  const auto context=graph.window()->worldLinearization();
  assert(map.points.size()==frames/views*features && graph.mapper().lmdb.numObservations()==frames*features);
  assert(graph.window()->snapshot()->layout.frontier.size()==8*6);
  assert(graph.window()->snapshot()->information->measurements.size()==frames*features+2*frames-1);
  Json result={{"points",Json::array()},{"source",nativeMappingSourceIdentity(graph.mapper())},
      {"stable_ids",saved->stable_ids},{"frames",frames},{"observations",map.observations},{"eligible",0}};
  double maximum_error=0;
  for(const auto& [id,point]:map.points) {
    const auto world=graph.window()->worldInformation(context,{id});
    const auto& track=graph.mapper().feature_tracks.at(id);assert(track.size()==views);
    const auto first=*track.begin();const auto expected=truth(first.first.frame_id,first.second);
    maximum_error=std::max(maximum_error,(world.positions-Eigen::MatrixXd(expected)).norm());
    assert(world.positions.allFinite() && world.covariance.allFinite());
    result["points"].push_back({id,std::vector<double>(world.positions.data(),world.positions.data()+3),
      std::vector<double>(world.covariance.data(),world.covariance.data()+9),std::vector<double>(world.unbounded_gram.data(),world.unbounded_gram.data()+9),
      world.observed,point.mature,point.eligible});
    if(point.eligible)result["eligible"]=result["eligible"].get<size_t>()+1;
  }
  assert(maximum_error<1e-7);result["maximum_position_error_m"]=maximum_error;
  return result;
}
void compare(const Json& expected,const Json& actual) {
  for(const char* key:{"frames","observations","source","stable_ids","eligible"})assert(expected.at(key)==actual.at(key));
  assert(expected.at("points").size()==actual.at("points").size());
  double position=0,covariance=0;
  for(size_t i=0;i<expected.at("points").size();++i) {
    const auto& a=expected.at("points")[i];const auto& b=actual.at("points")[i];
    for(int field:{0,4,5,6})assert(a[field]==b[field]);
    for(int field:{1,2,3})for(size_t j=0;j<a[field].size();++j) {
      const double x=a[field][j],y=b[field][j],error=std::abs(x-y);
      assert(error<1e-8*std::max(1.,std::abs(x)));
      if(field==1)position=std::max(position,error);else covariance=std::max(covariance,error);
    }
  }
  std::cout<<"Restored full output: points="<<actual.at("points").size()<<" max_position_difference="<<position
    <<" max_covariance_difference="<<covariance<<'\n';
}
} // namespace
int main(int argc,char** argv) {
  assert(argc==3);tbb::global_control threads(tbb::global_control::max_allowed_parallelism,2);
  const std::filesystem::path directory=std::filesystem::absolute(argv[1]);const std::string mode=argv[2];
  assert(mode=="create" || mode=="resume");
  if(mode=="create"){assert(!std::filesystem::exists(directory));std::filesystem::create_directories(directory);}
  Stream stream(directory/(mode+"-inputs.sqlite"));
  MappingMatchHistory matching(stream.calib,stream.config,bytes,frames);
  auto rows=std::make_shared<SubmapRowStore>(directory/(mode+"-rows"),cache_bytes);
  MappingGraphRegistry registry(stream.calib,stream.config,"continuous-paged-map");
  std::unique_ptr<NativeMappingGraph> graph;
  MappingInputStore::Snapshot input,old_input;MappingGraphRegistry::Snapshot old_graph;
  if(mode=="create") {
    rejects([&]{NativeMappingGraph dense(matching);},"native_graph_capacity");
    graph=std::make_unique<NativeMappingGraph>(matching,0,8,rows);
    for(size_t count:{40,200,208}) {
      input=stream.append(input,count);const auto update=graph->update(input,1);
      assert(update.new_frames==count-(old_input?old_input->frames.size():0));
      assert(graph->mapper().lmdb.numLandmarks()==count/views*features);
      const auto captured=registry.capture(input,graph->mapper(),1,input->poses.at(1).getPose(),Sophus::SE3d(),nullptr,graph.get());
      if(!old_graph)old_graph=captured;
      if(count==208)MappingGraphRegistry::writeWithInputs(directory/"continuation.pmap",captured,count,count);
      old_input=input;std::cout<<"Committed frames="<<count<<" landmarks="<<graph->mapper().lmdb.numLandmarks()<<'\n'<<std::flush;
    }
    assert(MappingGraphRegistry::restoreInputs(old_graph)->frames.size()==40);
  } else {
    const auto saved=MappingGraphRegistry::read(directory/"continuation.pmap","continuous-paged-map");
    input=MappingGraphRegistry::restoreInputs(saved.graph);assert(input->frames.size()==208);
    rejects([&]{MappingGraphRegistry::restore(MappingGraphRegistry::document(saved.graph));},"mapping_graph_capacity");
    graph=MappingGraphRegistry::restoreNativeGraph(saved.graph,matching,input,{},rows);
    registry.adopt(saved.graph);
  }
  const auto next=stream.append(input,frames);
  if(mode=="create") {
    const auto identity=nativeMappingStateIdentity(graph->mapper()),source=nativeMappingSourceIdentity(graph->mapper());
    const auto matches=matching.snapshot();int staged=0;bool cancelled=false;
    try {graph->update(next,1,[&]{return graph->mapper().getFramePoses().size()>208 && ++staged>=2;});}
    catch(const std::exception& error){assert(std::string(error.what()).find("cancelled")!=std::string::npos);cancelled=true;}
    assert(cancelled && matching.snapshot()==matches && graph->lastUpdate().revision==208);
    assert(nativeMappingStateIdentity(graph->mapper())==identity && nativeMappingSourceIdentity(graph->mapper())==source);
  }
  graph->update(next,1);
  const auto saved=registry.capture(next,graph->mapper(),1,next->poses.at(1).getPose(),Sophus::SE3d(),nullptr,graph.get());
  const auto report=output(*graph,saved);
  if(mode=="create") {std::ofstream file(directory/"expected.json");file<<report.dump()<<'\n';assert(file);}
  else {std::ifstream file(directory/"expected.json");Json expected;file>>expected;compare(expected,report);}
  const auto before=matching.snapshot();const auto excess=stream.append(next,frames+1);
  rejects([&]{graph->update(excess,0);},"native_graph_input");assert(matching.snapshot()==before && graph->lastUpdate().revision==frames);
  const auto cache=rows->statistics();
  assert(cache.pages>0 && cache.reads>0 && cache.peak_resident_bytes>0 && cache.peak_resident_bytes<=cache_bytes);
  std::cout<<"Derived-row cache: peak_bytes="<<cache.peak_resident_bytes<<" pages="<<cache.pages<<" reads="<<cache.reads<<'\n';
  std::cout<<"Paged continuous mapping "<<mode<<": 240 frames, complete landmarks/observations, immutable snapshots, cancellation and capacity pass.\n";
}

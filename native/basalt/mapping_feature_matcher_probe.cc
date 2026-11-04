// SPDX-License-Identifier: Apache-2.0
#include "mapping_feature_matcher.hpp"
#include <tbb/global_control.h>
#include <cassert>
#include <atomic>
#include <iostream>
#include <random>

namespace {
using Frame=posim_slam::MappingInputFrame;using Reference=posim_slam::MappingFrameReference;
struct Leases {std::atomic<int> active{0},peak{0},reads{0};};
Reference monitored(const std::shared_ptr<const Frame>& source,const std::shared_ptr<Leases>& state) {
  const Reference original(source);
  return {original.metadata(),[source,state](size_t maximum) {
    assert(maximum==SIZE_MAX);auto copy=std::make_shared<Frame>(*source);++state->reads;
    const int active=++state->active;int peak=state->peak;
    while(peak<active && !state->peak.compare_exchange_weak(peak,active)){}
    return Reference::Lease(copy.get(),[copy,state](const Frame*){--state->active;});
  }};
}
template<class F> void rejects(F call,const std::string& expected) {
  bool caught=false;try{call();}catch(const std::exception& error){assert(error.what()==expected);caught=true;}assert(caught);
}
}
int main() {
  tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
  basalt::Calibration<double> calibration;auto camera=basalt::GenericCamera<double>::fromString("pinhole");
  camera.setFromInit(Eigen::Vector4d(400,400,320,240));calibration.intrinsics.push_back(camera);
  calibration.T_i_c.emplace_back();calibration.resolution.emplace_back(640,480);basalt::VioConfig config;
  basalt::HashBow<256> vocabulary(config.mapper_bow_num_bits);
  std::map<int64_t,Reference> frames;auto state=std::make_shared<Leases>();
  auto resident=std::make_unique<basalt::NfrMapper>(calibration,config);
  for(int64_t stamp=1;stamp<=6;++stamp) {
    auto frame=std::make_shared<Frame>();frame->stamp=stamp;frame->width=640;frame->height=480;frame->digest.fill(stamp);
    auto& features=frame->features;std::mt19937 random(37);
    for(int i=0;i<64;++i) {
      const Eigen::Vector3d point((i%8-3.5)*.3-stamp*.05,(i/8-3.5)*.25,4+.05*(i%11));
      features.corners.emplace_back(400*point.x()/point.z()+320,400*point.y()/point.z()+240);
      features.corner_angles.push_back(0);features.corner_descriptors.emplace_back();
      for(size_t bit=0;bit<256;++bit)features.corner_descriptors.back()[bit]=random()&1;
    }
    std::vector<bool> success;camera.unproject(features.corners,features.corners_3d,success);
    vocabulary.compute_bow(features.corner_descriptors,features.hashes,features.bow_vector);
    const basalt::TimeCamId id(stamp,0);resident->feature_corners[id]=features;
    resident->hash_bow_database->add_to_database(id,features.bow_vector);frames.emplace(stamp,monitored(frame,state));
  }
  resident->match_all();const auto paged=posim_slam::matchMappingFrames(calibration,config,frames);
  assert(state->active==0 && state->peak>0 && state->peak<=4 && state->reads>6);
  assert(paged->feature_matches.size()==15 && paged->feature_matches.size()==resident->feature_matches.size());
  for(const auto& [pair,match]:resident->feature_matches) {
    const auto& other=paged->feature_matches.at(pair);
    assert(match.matches==other.matches);
    auto first=match.inliers,second=other.inliers;std::sort(first.begin(),first.end());std::sort(second.begin(),second.end());
    assert(first.size()==64 && first==second);
  }
  for(const auto& [id,features]:paged->feature_corners) {
    const auto& original=resident->feature_corners.at(id);assert(features.corners==original.corners);
    assert(features.corner_angles.empty() && features.corner_descriptors.empty() && features.corners_3d.empty() &&
      features.hashes.empty() && features.bow_vector.empty());
  }
  std::vector<std::pair<basalt::TimeCamId,double>> scores;
  paged->hash_bow_database->querry_database(resident->feature_corners.begin()->second.bow_vector,20,scores);assert(scores.empty());
  std::atomic<int> checked{0};std::unique_ptr<basalt::NfrMapper> published;
  rejects([&]{published=posim_slam::matchMappingFrames(calibration,config,frames,[&]{return ++checked>15;});},"mapping_match_cancelled");
  assert(!published && state->active==0);
  auto broken=frames;broken.at(3)=Reference(frames.at(3).metadata(),[](size_t)->Reference::Lease {
    throw std::runtime_error("mapping_input_page_digest");
  });
  rejects([&]{published=posim_slam::matchMappingFrames(calibration,config,broken);},"mapping_input_page_digest");
  assert(!published && state->active==0);
  std::cout<<"Paged matching: 15 original pairs, exact descriptors/inliers/pixels, at most four leased frames, cancellation and failure release pass.\n";
}

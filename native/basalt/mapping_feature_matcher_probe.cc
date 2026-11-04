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
  {
    posim_slam::MappingMatchHistory history(calibration,config);
    std::map<int64_t,Reference> prefix;
    for(int64_t stamp=1;stamp<=2;++stamp)prefix.emplace(stamp,frames.at(stamp));
    const auto first=history.advance(1,prefix);assert(first->new_frames==2 && first->new_pairs==1 && first->matches.size()==1);
    rejects([&]{history.advance(2,broken);},"mapping_input_page_digest");
    assert(history.snapshot()==first && state->active==0);
    const auto immutable_pair=first->matches.begin()->second;const auto first_pixels=first->frames.begin()->second.pixels;
    for(int64_t stamp=3;stamp<=4;++stamp)prefix.emplace(stamp,frames.at(stamp));
    checked=0;rejects([&]{history.advance(2,prefix,[&]{return ++checked>4;});},"mapping_match_cancelled");
    assert(history.snapshot()==first && state->active==0);
    const auto second=history.advance(2,prefix);
    assert(second->new_frames==2 && second->new_pairs==5 && second->evaluated_pairs==6 && second->matches.size()==6);
    assert(second->matches.at(first->matches.begin()->first)==immutable_pair && second->frames.begin()->second.pixels==first_pixels);
    const int reads=state->reads;assert(history.advance(2,prefix)==second && state->reads==reads);
    const auto factors_only=history.advance(3,prefix);assert(!factors_only->new_frames && !factors_only->new_pairs && state->reads==reads);
    const auto full=history.advance(8,frames);assert(full->new_frames==2 && full->new_pairs==9 && full->evaluated_pairs==15);
    assert(full->matches.size()==15 && first->frames.size()==2 && first->matches.size()==1);
    const auto complete=history.makeMapper(full);
    for(const auto& [pair,match]:paged->feature_matches) {
      const auto& found=complete->feature_matches.at(pair);assert(found.matches==match.matches);
      auto a=found.inliers,b=match.inliers;std::sort(a.begin(),a.end());std::sort(b.begin(),b.end());assert(a==b && a.size()==64);
    }
    assert(state->active==0 && state->peak<=4);
    rejects([&]{history.advance(7,frames);},"mapping_match_revision");
    auto changed=frames;changed.erase(1);rejects([&]{history.advance(9,changed);},"mapping_match_source_changed");
    changed=frames;changed.at(1)=Reference(std::make_shared<Frame>(*frames.at(1).read()));
    rejects([&]{history.advance(9,changed);},"mapping_match_source_changed");assert(history.snapshot()==full);
    posim_slam::MappingMatchHistory other(calibration,config);
    rejects([&]{other.makeMapper(full);},"mapping_match_owner");
    posim_slam::MappingMatchHistory limited(calibration,config,first->numeric_bytes);
    std::map<int64_t,Reference> two{{1,frames.at(1)},{2,frames.at(2)}};const auto bounded=limited.advance(1,two);
    rejects([&]{limited.advance(2,frames);},"mapping_match_capacity");assert(limited.snapshot()==bounded && state->active==0);
    posim_slam::MappingMatchHistory pair_limited(calibration,config,full->numeric_bytes-1);
    const auto pair_bound=pair_limited.advance(1,two);
    rejects([&]{pair_limited.advance(2,frames);},"mapping_match_capacity");
    assert(pair_limited.snapshot()==pair_bound && state->active==0);
    posim_slam::MappingMatchHistory ordered(calibration,config);two={{1,frames.at(1)},{3,frames.at(3)}};const auto gap=ordered.advance(1,two);
    const auto old_pair=gap->matches.begin()->second;two.emplace(2,frames.at(2));const auto delayed=ordered.advance(2,two);
    assert(delayed->new_frames==1 && delayed->queried_frames==2 && delayed->new_pairs==2 && delayed->evaluated_pairs==3);
    assert(delayed->matches.at(gap->matches.begin()->first)==old_pair);
    auto no_inliers=config;no_inliers.mapper_min_matches=1000;posim_slam::MappingMatchHistory rejected_pairs(calibration,no_inliers);
    two.erase(2);const auto unsuccessful=rejected_pairs.advance(1,two);assert(unsuccessful->matches.empty() && unsuccessful->evaluated_pairs==1);
    two.emplace(2,frames.at(2));const auto retried=rejected_pairs.advance(2,two);
    assert(retried->matches.empty() && retried->new_pairs==2 && retried->evaluated_pairs==3 && state->active==0);
  }
  std::cout<<"Incremental matching: original pairs evaluated once, unchanged pair/pixel owners, chronological and delayed retrieval, unsuccessful-pair reuse, no-input reuse, atomic cancellation/source/capacity controls and four-frame lease bound pass.\n";
  std::cout<<"Paged matching: 15 original pairs, exact descriptors/inliers/pixels, at most four leased frames, cancellation and failure release pass.\n";
}

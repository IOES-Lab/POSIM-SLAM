/**
BSD 3-Clause License

This file is part of the Basalt project.
https://gitlab.com/VladyslavUsenko/basalt.git

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// Adapted from NfrMapper::match_all and findInliersRansac at Basalt revision
// 0f3b2b52c807f70ff4e2973ce253c73329eea7bc. Preserve retrieval, descriptor
// thresholds, geometric solver and refinement. Each frame pair owns its RANSAC
// random stream, independently of wall time, scheduling and process lifetime.
#pragma once
#include "mapping_frame_io.hpp"
#include "estimator_budget.hpp"
#include "mapping_history_budget.hpp"
#include <basalt/hash_bow/hash_bow.h>
#include <basalt/utils/keypoints.h>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <atomic>
#include <mutex>

namespace posim_slam {
inline uint32_t mappingPairSeed(const MappingInputFrame& first,const MappingInputFrame& second) {
  InputDigest digest;
  for(const auto* frame:{&first,&second}) {
    uint8_t stamp[8];for(int i=0;i<8;++i)stamp[i]=uint8_t(uint64_t(frame->stamp)>>(8*i));
    digest.update(stamp,8);digest.update(frame->digest.data(),frame->digest.size());
  }
  const auto hash=digest.finish();uint32_t seed=0;for(int i=0;i<4;++i)seed=(seed<<8)|hash[i];return seed;
}
inline void mappingPairInliers(const MappingInputFrame& first,const MappingInputFrame& second,
    double threshold,int minimum,basalt::MatchData& match) {
  using Problem=opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem;
  struct SeededProblem:Problem {
    SeededProblem(adapter_t& adapter,uint32_t seed):Problem(adapter,STEWENIUS,false) {
      rng_alg_.seed(seed);
      // OpenGV binds a copy of the generator. Rebind after seeding that copy.
      rng_gen_=std::make_shared<std::function<int()>>(std::bind(*rng_dist_,rng_alg_));
    }
  };
  match.inliers.clear();opengv::bearingVectors_t bearings1,bearings2;
  for(const auto& [i,j]:match.matches) {
    bearings1.push_back(first.features.corners_3d[i].head<3>());
    bearings2.push_back(second.features.corners_3d[j].head<3>());
  }
  opengv::relative_pose::CentralRelativeAdapter adapter(bearings1,bearings2);
  opengv::sac::Ransac<Problem> ransac;
  ransac.sac_model_=std::make_shared<SeededProblem>(adapter,mappingPairSeed(first,second));
  ransac.threshold_=threshold;ransac.max_iterations_=100;ransac.computeModel();
  const auto before=ransac.inliers_.size();
  adapter.sett12(ransac.model_coefficients_.topRightCorner<3,1>());
  adapter.setR12(ransac.model_coefficients_.topLeftCorner<3,3>());
  const auto refined=opengv::relative_pose::optimize_nonlinear(adapter,ransac.inliers_);
  ransac.sac_model_->selectWithinDistance(refined,ransac.threshold_,ransac.inliers_);
  if(ransac.inliers_.size()+2<before)
    std::cout<<"Warning: non-linear refinement reduced the relative pose ransac inlier count from "
      <<before<<" to "<<ransac.inliers_.size()<<"."<<std::endl;
  match.T_i_j=Sophus::SE3d(refined.topLeftCorner<3,3>(),refined.topRightCorner<3,1>().normalized());
  if(int64_t(ransac.inliers_.size())>=minimum)
    for(auto index:ransac.inliers_)match.inliers.emplace_back(match.matches[index]);
}
// Pixels and calibration are unchanged for an already evaluated image pair.
// HashBoW queries only frames strictly earlier than the query timestamp. Keep
// those accepted matches; newly appended timestamps cannot change that query.
// A late frame reopens only affected later retrieval queries. Already evaluated
// pairs, including unsuccessful ones, are not rerun; valid old pairs remain.
class MappingMatchHistory {
  friend class MappingInputStore;
  friend class NativeMappingGraph;
 public:
  using Pair=std::pair<basalt::TimeCamId,basalt::TimeCamId>;
  struct Frame {
    MappingFrameReference source;
    std::shared_ptr<const basalt::KeypointsData> pixels;
    basalt::HashBowVector bow;
  };
  struct Data {
    uint64_t revision=0;
    std::map<int64_t,Frame> frames;
    std::map<Pair,std::shared_ptr<const basalt::MatchData>> matches;
    size_t evaluated_pairs=0,new_frames=0,queried_frames=0,new_pairs=0,numeric_bytes=0;
    const std::set<Pair>& evaluatedPairs() const {return evaluated;}
   private:
    friend class MappingMatchHistory;
    std::shared_ptr<const int> owner;
    std::set<Pair> evaluated;
  };
  using Snapshot=std::shared_ptr<const Data>;
  MappingMatchHistory(const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
      size_t maximum_bytes=96*1024*1024,size_t maximum_frames=kMaxJointColumns/6)
      :calibration_(calibration),config_(config),maximum_bytes_(maximum_bytes),maximum_frames_(maximum_frames),
       database_(std::make_shared<basalt::HashBow<256>>(config.mapper_bow_num_bits)) {
    if(calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1 || calibration.resolution.size()!=1 ||
        !maximum_bytes || maximum_bytes>96*1024*1024 || maximum_frames<2 ||
        maximum_frames>kMaxMappingHistoryFrames)throw std::invalid_argument("mapping_match_input");
    auto empty=std::make_shared<Data>();empty->owner=identity_;data_=empty;
  }
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  // Serialize immutable decisions only for explicit checkpoints. Complete
  // original-feature hashes bind restored pairs, including failed attempts.
  static nlohmann::json checkpoint(const Snapshot& state,
      const basalt::Calibration<double>& calibration,const basalt::VioConfig& config);
  static void validateCheckpoint(const nlohmann::json& document);
  Snapshot restore(uint64_t revision,const std::map<int64_t,MappingFrameReference>& frames,
      const nlohmann::json& document,const std::function<bool()>& cancelled={});
  Snapshot advance(uint64_t revision,const std::map<int64_t,MappingFrameReference>& frames,
      const std::function<bool()>& cancelled={},
      const std::function<void(const Snapshot&)>& before_commit={}) {
    std::lock_guard<std::mutex> writer(writer_);
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("mapping_match_cancelled");};check();
    const auto old=snapshot();
    if(revision<old->revision || frames.size()>maximum_frames_)throw std::invalid_argument("mapping_match_revision");
    for(const auto& [stamp,frame]:old->frames) {
      auto found=frames.find(stamp);
      if(found==frames.end() || !frame.source.sameSource(found->second))throw std::invalid_argument("mapping_match_source_changed");
    }
    std::vector<int64_t> incoming;
    for(const auto& [stamp,frame]:frames)if(!old->frames.count(stamp)) {
      if(stamp<=0)throw std::invalid_argument("mapping_input_frame_identity");
      incoming.push_back(stamp);
    }
    if(revision==old->revision) {
      if(!incoming.empty())throw std::invalid_argument("mapping_match_revision");
      if(before_commit)before_commit(old);return old;
    }
    auto next=std::make_shared<Data>(*old);next->revision=revision;next->new_frames=incoming.size();next->new_pairs=0;next->queried_frames=0;
    // Copy only the compact retrieval index for the transaction. Descriptor
    // arrays remain paged, and previous pair results remain immutable/shared.
    auto database=incoming.empty()?database_:std::make_shared<basalt::HashBow<256>>(*database_);
    for(auto stamp:incoming) {
      check();const auto frame=frames.at(stamp).read();mapping_frame_io::shape(*frame);
      if(stamp!=frame->stamp || calibration_.resolution[0]!=Eigen::Vector2i(frame->width,frame->height))
        throw std::invalid_argument("mapping_input_frame_identity");
      auto pixels=std::make_shared<basalt::KeypointsData>();pixels->corners=frame->features.corners;
      next->numeric_bytes+=pixels->corners.size()*sizeof(pixels->corners[0])+frame->features.bow_vector.size()*
        (sizeof(frame->features.bow_vector[0])+sizeof(std::pair<basalt::TimeCamId,double>));
      if(next->numeric_bytes>maximum_bytes_)throw std::runtime_error("mapping_match_capacity");
      next->frames.emplace(stamp,Frame{frames.at(stamp),std::move(pixels),frame->features.bow_vector});
      database->add_to_database(basalt::TimeCamId(stamp,0),frame->features.bow_vector);
    }
    std::vector<int64_t> queries;
    if(!incoming.empty())for(const auto& [stamp,frame]:frames)if(stamp>=incoming.front())queries.push_back(stamp);
    next->queried_frames=queries.size();
    struct Result {std::vector<std::pair<Pair,std::shared_ptr<const basalt::MatchData>>> matches;std::vector<Pair> evaluated;};
    std::vector<Result> results(queries.size());
    std::atomic<size_t> staged_bytes{next->numeric_bytes};
    tbb::task_arena arena(2);
    arena.execute([&]{tbb::parallel_for(tbb::blocked_range<size_t>(0,queries.size()),[&](const auto& range) {
      for(size_t i=range.begin();i!=range.end();++i) {
        check();const auto stamp=queries[i];const basalt::TimeCamId first_id(stamp,0);
        MappingFrameReference::Lease first;std::vector<std::pair<basalt::TimeCamId,double>> candidates;
        database->querry_database(next->frames.at(stamp).bow,config_.mapper_num_frames_to_match,candidates,&stamp);
        for(const auto& [second_id,score]:candidates) {
          check();if(!(score>config_.mapper_frames_to_match_threshold))continue;
          if(second_id.cam_id!=0 || second_id.frame_id>=stamp)throw std::logic_error("mapping_match_temporal_candidate");
          const Pair pair{first_id,second_id};if(old->evaluated.count(pair))continue;
          if(staged_bytes.fetch_add(sizeof(Pair))+sizeof(Pair)>maximum_bytes_)throw std::runtime_error("mapping_match_capacity");
          if(!first)first=frames.at(stamp).read();
          const auto second=frames.at(second_id.frame_id).read();auto match=std::make_shared<basalt::MatchData>();
          basalt::matchDescriptors(first->features.corner_descriptors,second->features.corner_descriptors,match->matches,70,1.2);
          if(int(match->matches.size())>config_.mapper_min_matches)
            mappingPairInliers(*first,*second,config_.mapper_ransac_threshold,config_.mapper_min_matches,*match);
          results[i].evaluated.push_back(pair);
          if(!match->inliers.empty()) {
            const size_t bytes=sizeof(basalt::MatchData)+match->matches.size()*sizeof(match->matches[0])+match->inliers.size()*sizeof(match->inliers[0]);
            if(staged_bytes.fetch_add(bytes)+bytes>maximum_bytes_)throw std::runtime_error("mapping_match_capacity");
            results[i].matches.push_back({pair,std::move(match)});
          }
        }
      }
    });});
    for(const auto& result:results) {
      next->new_pairs+=result.evaluated.size();next->numeric_bytes+=result.evaluated.size()*sizeof(Pair);
      for(const auto& pair:result.evaluated)if(!next->evaluated.insert(pair).second)throw std::logic_error("mapping_match_pair_reused");
      for(const auto& [pair,match]:result.matches) {
        next->numeric_bytes+=sizeof(basalt::MatchData)+match->matches.size()*sizeof(match->matches[0])+match->inliers.size()*sizeof(match->inliers[0]);
        if(next->numeric_bytes>maximum_bytes_)throw std::runtime_error("mapping_match_capacity");
        if(!next->matches.emplace(pair,match).second)throw std::logic_error("mapping_match_pair_reused");
      }
    }
    next->evaluated_pairs+=next->new_pairs;check();
    Snapshot committed=next;
    // The exclusive native graph owner can stage its matching-dependent state
    // here. A failure keeps both the retrieval index and public snapshot intact.
    if(before_commit)before_commit(committed);
    database_=std::move(database);std::atomic_store(&data_,committed);return committed;
  }
  std::unique_ptr<basalt::NfrMapper> makeMapper(const Snapshot& state) const {
    if(!state || state->owner!=identity_)throw std::invalid_argument("mapping_match_owner");
    if(state->frames.size()>size_t(kMaxJointColumns/6))throw std::runtime_error("mapping_whole_graph_capacity");
    auto mapper=std::make_unique<basalt::NfrMapper>(calibration_,config_);
    for(const auto& [stamp,frame]:state->frames)mapper->feature_corners[basalt::TimeCamId(stamp,0)].corners=frame.pixels->corners;
    for(const auto& [pair,match]:state->matches)mapper->feature_matches.emplace(pair,*match);
    return mapper;
  }
 private:
  basalt::Calibration<double> calibration_;basalt::VioConfig config_;size_t maximum_bytes_,maximum_frames_;
  std::shared_ptr<const int> identity_=std::make_shared<const int>(0),source_identity_;
  std::shared_ptr<basalt::HashBow<256>> database_;mutable std::mutex writer_;Snapshot data_;
};
// Returns a newly owned mapper only after complete matching. Pixels stay resident
// for track construction/optimization; descriptor, bearing and orientation arrays
// are leased. At most two tasks own two frame leases each. The cancellation
// callback may be called concurrently and must be thread-safe.
inline std::unique_ptr<basalt::NfrMapper> matchMappingFrames(
    const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
    const std::map<int64_t,MappingFrameReference>& frames,
    const std::function<bool()>& cancelled={}) {
  auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("mapping_match_cancelled");};check();
  if(frames.size()>size_t(kMaxJointColumns/6) || calibration.resolution.size()!=1 ||
      calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1)
    throw std::invalid_argument("mapping_match_input");
  auto mapper=std::make_unique<basalt::NfrMapper>(calibration,config);
  for(const auto& [stamp,reference]:frames) {
    check();const auto frame=reference.read();
    mapping_frame_io::shape(*frame);
    if(stamp!=frame->stamp || calibration.resolution[0]!=Eigen::Vector2i(frame->width,frame->height))
      throw std::invalid_argument("mapping_input_frame_identity");
    const basalt::TimeCamId id(stamp,0);
    mapper->feature_corners[id].corners=frame->features.corners;
    mapper->hash_bow_database->add_to_database(id,frame->features.bow_vector);
  }
  std::vector<basalt::TimeCamId> keys;
  for(const auto& [id,unused]:mapper->feature_corners)keys.push_back(id);
  tbb::task_arena arena(2);
  arena.execute([&]{tbb::parallel_for(tbb::blocked_range<size_t>(0,keys.size()),[&](const auto& range) {
    for(size_t i=range.begin();i!=range.end();++i) {
      check();const auto id1=keys[i];const auto first=frames.at(id1.frame_id).read();
      std::vector<std::pair<basalt::TimeCamId,double>> candidates;
      mapper->hash_bow_database->querry_database(first->features.bow_vector,
        config.mapper_num_frames_to_match,candidates,&id1.frame_id);
      for(const auto& [id2,score]:candidates) {
        check();
        if(id2.frame_id==id1.frame_id || !(score>config.mapper_frames_to_match_threshold))continue;
        if(id2.cam_id!=0)throw std::invalid_argument("mapping_match_camera");
        const auto second=frames.at(id2.frame_id).read();basalt::MatchData match;
        // The pinned all-frame path uses these values directly. The similarly
        // named configurable fields govern its separate stereo matching path.
        basalt::matchDescriptors(first->features.corner_descriptors,second->features.corner_descriptors,
          match.matches,70,1.2);
        if(int(match.matches.size())>config.mapper_min_matches)
          mappingPairInliers(*first,*second,
            config.mapper_ransac_threshold,config.mapper_min_matches,match);
        if(!match.inliers.empty())mapper->feature_matches[std::make_pair(id1,id2)]=std::move(match);
      }
    }
  });});
  check();
  // Track construction and optimization use pixels, poses and accepted pairs.
  // Retrieval is rebuilt from immutable input on the next requested revision.
  mapper->hash_bow_database=std::make_shared<basalt::HashBow<256>>(config.mapper_bow_num_bits);
  return mapper;
}
} // namespace posim_slam

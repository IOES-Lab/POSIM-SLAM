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
// Adapted from NfrMapper::match_all at Basalt revision
// 0f3b2b52c807f70ff4e2973ce253c73329eea7bc. Preserve its retrieval, descriptor
// thresholds and RANSAC calls; load complete features only for the current pair.
#pragma once
#include "mapping_frame_io.hpp"
#include "estimator_budget.hpp"
#include <basalt/hash_bow/hash_bow.h>
#include <basalt/utils/keypoints.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

namespace posim_slam {
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
          basalt::findInliersRansac(first->features,second->features,
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

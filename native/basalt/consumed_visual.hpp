// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cstdint>
#include <map>
#include <stdexcept>

namespace posim_slam {
// Per-track cutoff, not a permanent blacklist. A retired track may be
// triangulated again using only observations newer than its consumed history.
class ConsumedVisualObservations {
 public:
  void record(int64_t id,int64_t stamp) {
    if(id<0 || stamp<=0)throw std::invalid_argument("invalid_consumed_visual_observation");
    auto& cutoff=through_[id];cutoff=std::max(cutoff,stamp);
  }
  bool allows(int64_t id,int64_t stamp) const {
    const auto found=through_.find(id);
    return found==through_.end() || stamp>found->second;
  }
  size_t size() const {return through_.size();}

  // Remove a cutoff only after no buffered observation can replay it. The
  // estimator delivers frames in timestamp order. State is bounded by tracks
  // present in the optical-flow history, not by total voyage duration.
  template<class History> void prune(const History& history) {
    for(auto it=through_.begin();it!=through_.end();) {
      bool buffered=false;
      for(const auto& [stamp,frame]:history) {
        if(stamp>it->second)break;
        for(const auto& camera:frame->observations)
          if(camera.count(it->first)){buffered=true;break;}
        if(buffered)break;
      }
      if(buffered)++it;else it=through_.erase(it);
    }
  }
 private:
  std::map<int64_t,int64_t> through_;
};
}

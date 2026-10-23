// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>

// Segment-local sparse odometry archive. IDs update existing coordinates; they
// never append another copy. Three distinct inlier exposures admit a landmark.
// Active coordinates follow optimization; departed coordinates freeze. There is
// no loop closure or retrospective correction of departed landmarks.
class LandmarkArchive {
 public:
  using Point=std::array<float,3>;
  struct Entry {Point point;unsigned support=0;int64_t last=0;};
  explicit LandmarkArchive(size_t budget=100000):budget_(budget) {
    if(!budget || budget>100000)throw std::runtime_error("archive_budget");
  }
  void update(int64_t stamp,const std::vector<int>& ids,
              const std::vector<Point>& points,const std::set<int>& inliers) {
    if(stamp<=stamp_ || ids.size()!=points.size() || ids.size()>100000)
      throw std::runtime_error("archive_input");
    std::set<int> active,invalid,seen;
    for(size_t i=0;i<ids.size();++i) {
      if(ids[i]<0 || !seen.insert(ids[i]).second)throw std::runtime_error("archive_ids");
      bool valid=true;
      for(float x:points[i])if(!std::isfinite(x) || std::abs(x)>=10000)valid=false;
      (valid?active:invalid).insert(ids[i]);
    }
    rejected_+=invalid.size();
    // Unconfirmed departed tracks are not mapping evidence.
    for(auto it=entries_.begin();it!=entries_.end();) {
      if(invalid.count(it->first) || (!active.count(it->first) && it->second.support<3)) {
        order_.erase({it->second.last,it->first});it=entries_.erase(it);
      } else ++it;
    }
    for(size_t i=0;i<ids.size();++i) {
      const int id=ids[i];if(invalid.count(id))continue;
      auto it=entries_.find(id);
      if(it==entries_.end()) {
        if(!inliers.count(id))continue;
        if(entries_.size()==budget_) {
          const auto oldest=*order_.begin();
          if(entries_.at(oldest.second).support>=3)++evicted_;
          entries_.erase(oldest.second);order_.erase(order_.begin());
        }
        it=entries_.emplace(id,Entry{points[i],0,stamp}).first;
      } else order_.erase({it->second.last,id});
      auto& entry=it->second;entry.point=points[i];entry.last=stamp;
      if(inliers.count(id) && entry.support<3)++entry.support;
      order_.insert({stamp,id});
    }
    stamp_=stamp;
  }
  std::vector<Point> points() const {
    std::vector<Point> result;
    for(const auto& item:entries_)if(item.second.support>=3)result.push_back(item.second.point);
    return result;
  }
  static std::vector<Point> display_sample(const std::vector<Point>& points) {
    const size_t count=std::min(points.size(),size_t(1000));
    std::vector<Point> result;result.reserve(count);
    for(size_t i=0;i<count;++i)
      result.push_back(points[count<=1?0:i*(points.size()-1)/(count-1)]);
    return result;
  }
  size_t retained() const{return entries_.size();}
  size_t evicted() const{return evicted_;}
  size_t rejected() const{return rejected_;}
 private:
  size_t budget_,evicted_=0,rejected_=0;int64_t stamp_=0;
  std::map<int,Entry> entries_;
  std::set<std::pair<int64_t,int>> order_;
};

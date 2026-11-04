// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "native_mapping_linearization.hpp"

namespace posim_slam {
// Topology only: keep active originals replaceable, then consume historical
// groups in last-observation order. A pose can retire after its last incident
// original has arrived. This neither changes associations nor adds information.
struct NativeMappingSchedule {
  struct Group {
    NativeMappingRowSource source;
    std::vector<int64_t> poses;
    bool active=false;
    int64_t event=0;
  };
  std::vector<Group> groups;
  std::vector<NativeMappingRowSource> order;
  std::set<int64_t> frontier,unobserved;
  std::map<int64_t,int64_t> last_use;
  std::map<int64_t,std::set<int64_t>> event_poses;
  NativeMappingSchedule(basalt::NfrMapper& mapper,const std::set<int64_t>& active) {
    const auto& states=mapper.getFramePoses();
    if(states.empty() || states.size()>size_t(kMaxJointColumns/6) ||
        mapper.lmdb.getLandmarks().size()>100000 ||
        mapper.rel_pose_factors.size()+mapper.roll_pitch_factors.size()>65536)
      throw std::invalid_argument("native_mapping_linearization_configuration");
    for(const auto& [stamp,state]:states)unobserved.insert(stamp);
    auto append=[&](NativeMappingRowSource source,std::set<int64_t> poses) {
      Group group;group.source=source;group.poses.assign(poses.begin(),poses.end());
      group.active=std::includes(active.begin(),active.end(),poses.begin(),poses.end());
      group.event=*poses.rbegin();
      for(auto stamp:poses) {
        if(!states.count(stamp))throw std::invalid_argument("native_mapping_pose_missing");
        unobserved.erase(stamp);
        if(group.active)frontier.insert(stamp);
        else {last_use[stamp]=std::max(last_use[stamp],group.event);event_poses[group.event].insert(stamp);}
      }
      groups.push_back(std::move(group));
    };
    if(mapper.config.mapper_use_factors) {
      for(size_t i=0;i<mapper.rel_pose_factors.size();++i) {
        const auto& f=mapper.rel_pose_factors[i];append({NativeMappingRowSource::relative,int64_t(i)},{f.t_i_ns,f.t_j_ns});
      }
      for(size_t i=0;i<mapper.roll_pitch_factors.size();++i)
        append({NativeMappingRowSource::roll_pitch,int64_t(i)},{mapper.roll_pitch_factors[i].t_ns});
    }
    size_t originals=0;
    for(const auto& [id,point]:mapper.lmdb.getLandmarks()) {
      if(id>size_t(std::numeric_limits<int64_t>::max()) || point.obs.empty() ||
          point.obs.size()>2048 || (originals+=point.obs.size())>1000000)
        throw std::invalid_argument("native_mapping_landmark");
      std::set<int64_t> poses{point.host_kf_id.frame_id};
      for(const auto& [image,pixel]:point.obs)poses.insert(image.frame_id);
      if(6*poses.size()+3>size_t(kMaxJointColumns))throw std::runtime_error("native_mapping_landmark_span");
      append({NativeMappingRowSource::landmark,int64_t(id)},std::move(poses));
    }
    std::stable_sort(groups.begin(),groups.end(),[](const Group& a,const Group& b) {
      if(a.active!=b.active)return a.active;
      return !a.active && a.event<b.event;
    });
    for(const auto& group:groups)order.push_back(group.source);
    // The previous whole-pose QR also registered unused pose columns as free.
    // Preserve them as zero columns in an existing batch, without a new key or
    // likelihood. With no historical batch, the original ownership check applies.
    if(!event_poses.empty())for(auto stamp:unobserved) {
      const auto event=event_poses.begin()->first;
      event_poses.begin()->second.insert(stamp);last_use[stamp]=event;
    }
  }
};
} // namespace posim_slam

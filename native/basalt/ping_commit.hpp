// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"

namespace posim_slam {
// Owner-thread transition. The supplied prior must have been built from exactly
// the planned factors at the current reference state, without intervening edits.
// All semantic checks and allocating copies precede changes to live containers.
// Caller separately owns keyframe IDs, optical-flow history and output queues.
template<class Estimator,class ImuMap> void installPingMarginalPrior(
    Estimator& owner,basalt::MargLinData<double>& current_prior,
    std::vector<JointPingGroupSpec>& groups,ImuMap& imu,
    const PingRetirementPlan& plan,const PingMarginalLayout& layout,
    const std::set<int64_t>& removed,const std::set<int64_t>& pose_only,
    const basalt::MargLinData<double>& next_prior) {
  if(owner.posim_ping_trial_active)throw std::logic_error("ping_commit_during_trial");
  const auto checked=planPingRetirement(owner,groups,removed,removed,plan.landmarks);
  if(checked.planes!=plan.planes || checked.landmarks!=plan.landmarks ||
     checked.observations!=plan.observations || checked.required_frames!=plan.required_frames)
    throw std::invalid_argument("changed_ping_retirement_plan");
  std::vector<std::pair<int64_t,int64_t>> intervals;
  for(const auto& [start,measurement]:imu) {
    if(start!=measurement.get_start_t_ns())throw std::invalid_argument("invalid_ping_imu_identity");
    intervals.emplace_back(start,start+measurement.get_dt_ns());
  }
  const auto expected=planPingMarginalLayout(owner,removed,pose_only,plan.required_frames,intervals);
  if(expected.source.abs_order_map!=layout.source.abs_order_map ||
     expected.retained.abs_order_map!=layout.retained.abs_order_map ||
     expected.consumed_imu_starts!=layout.consumed_imu_starts)
    throw std::invalid_argument("changed_ping_marginal_layout");
  requireStateOrder(next_prior.order);
  if(!next_prior.is_sqrt || next_prior.order.abs_order_map!=layout.retained.abs_order_map ||
     next_prior.H.cols()!=Eigen::Index(next_prior.order.total_size) || next_prior.H.rows()<1 ||
     next_prior.H.rows()!=next_prior.b.size() || !next_prior.H.allFinite() || !next_prior.b.allFinite())
    throw std::invalid_argument("invalid_ping_installed_prior");
  for(auto id:plan.landmarks)if(!owner.lmdb.landmarkExists(id))
    throw std::invalid_argument("missing_ping_retired_landmark");
  auto states=owner.frame_states;auto poses=owner.frame_poses;
  auto consumed_visual=owner.posim_consumed_visual;
  for(auto id:plan.landmarks)
    for(const auto& [target,pixel]:owner.lmdb.getLandmark(id).obs)
      consumed_visual.record(id,target.frame_id);
  auto landmarks=owner.lmdb;auto planes=owner.posim_ping_planes;
  auto next_groups=groups;auto next_imu=imu;auto prior=next_prior;
  for(int64_t t:pose_only) {
    poses.emplace(t,basalt::PoseStateWithLin<double>(states.at(t)));states.erase(t);
  }
  for(int64_t t:removed){states.erase(t);poses.erase(t);}
  for(auto& [t,state]:states)if(!state.isLinearized())state.setLinTrue();
  for(auto& [t,pose]:poses)if(!pose.isLinearized())pose.setLinTrue();
  for(auto id:plan.landmarks)landmarks.removeLandmark(id);
  landmarks.removeKeyframes(removed,removed,removed);
  for(int id:plan.planes)planes.erase(id);
  next_groups.erase(std::remove_if(next_groups.begin(),next_groups.end(),
    [&](const auto& group){return plan.planes.count(group.plane_id);}),next_groups.end());
  for(int64_t start:layout.consumed_imu_starts)next_imu.erase(start);
  // Container swaps do not allocate. No semantic failure is possible below.
  owner.frame_states.swap(states);owner.frame_poses.swap(poses);
  std::swap(owner.lmdb,landmarks);std::swap(owner.posim_ping_planes,planes);
  std::swap(owner.posim_consumed_visual,consumed_visual);
  groups.swap(next_groups);imu.swap(next_imu);
  current_prior.H.swap(prior.H);current_prior.b.swap(prior.b);
  current_prior.order.abs_order_map.swap(prior.order.abs_order_map);
  current_prior.order.total_size=next_prior.order.total_size;
  current_prior.order.items=next_prior.order.items;current_prior.is_sqrt=true;
  ++owner.posim_ping_state_revision;
}
}

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_joint.hpp"
#include <basalt/utils/imu_types.h>
#include <set>
#include <vector>

namespace posim_slam {
inline void requireStateOrder(const basalt::AbsOrderMap& order) {
  if(order.total_size<6 || order.total_size>512 || order.items!=order.abs_order_map.size())
    throw std::invalid_argument("invalid_joint_state_order");
  std::vector<bool> occupied(order.total_size,false);
  for(const auto& [stamp,block]:order.abs_order_map) {
    const auto [offset,size]=block;
    if(stamp<=0 || offset<0 || (size!=6 && size!=15) || (size_t(offset)+size)>order.total_size)
      throw std::invalid_argument("invalid_joint_state_block");
    for(int i=offset;i<offset+size;++i) {
      if(occupied[i])throw std::invalid_argument("overlapping_joint_state_blocks");
      occupied[i]=true;
    }
  }
  if(std::find(occupied.begin(),occupied.end(),false)!=occupied.end())
    throw std::invalid_argument("incomplete_joint_state_order");
}

struct PingMarginalLayout {
  basalt::AbsOrderMap source,retained;
  std::set<int64_t> consumed_imu_starts;
};

// Keep all live pose columns, including recent group dependencies. Pose-only
// states precede full inertial states, matching Basalt's optimizer ordering.
// Consume an IMU interval only when one endpoint loses velocity/bias variables;
// intervals between fully retained states remain live and must not enter prior.
template<class Estimator> PingMarginalLayout planPingMarginalLayout(
    const Estimator& owner,const std::set<int64_t>& removed,
    const std::set<int64_t>& pose_only,const std::set<int64_t>& required,
    const std::vector<std::pair<int64_t,int64_t>>& imu_intervals) {
  if(owner.posim_ping_trial_active)throw std::logic_error("ping_layout_during_trial");
  PingMarginalLayout out;
  auto add=[](basalt::AbsOrderMap& order,int64_t t,int size) {
    if(!order.abs_order_map.emplace(t,std::make_pair(order.total_size,size)).second)
      throw std::invalid_argument("duplicate_ping_layout_state");
    order.total_size+=size;++order.items;
  };
  for(const auto& [t,state]:owner.frame_poses)add(out.source,t,6);
  for(const auto& [t,state]:owner.frame_states)add(out.source,t,15);
  requireStateOrder(out.source);
  for(int64_t t:required)if(!out.source.abs_order_map.count(t))throw std::invalid_argument("missing_ping_layout_dependency");
  for(int64_t t:removed)if(!out.source.abs_order_map.count(t))throw std::invalid_argument("unknown_ping_removed_state");
  for(int64_t t:pose_only)
    if(removed.count(t) || !owner.frame_states.count(t))throw std::invalid_argument("invalid_ping_pose_only_state");
  std::set<int64_t> retained_poses;
  for(const auto& [t,state]:owner.frame_poses)if(!removed.count(t))retained_poses.insert(t);
  retained_poses.insert(pose_only.begin(),pose_only.end());
  for(int64_t t:retained_poses)add(out.retained,t,6);
  for(const auto& [t,state]:owner.frame_states)
    if(!removed.count(t) && !pose_only.count(t))add(out.retained,t,15);
  requireStateOrder(out.retained);
  std::set<int64_t> starts;
  for(const auto& [start,end]:imu_intervals) {
    if(start>=end || !starts.insert(start).second || !owner.frame_states.count(start) || !owner.frame_states.count(end))
      throw std::invalid_argument("invalid_ping_imu_interval");
    if(removed.count(start) || removed.count(end) || pose_only.count(start) || pose_only.count(end))
      out.consumed_imu_starts.insert(start);
  }
  return out;
}

// Input columns: source pose/inertial states, then ALL shared plane/landmark
// coordinates being retired. Include each raw factor exactly once, including
// the existing prior and any relevant inertial rows. Do not append a second
// independently eliminated copy of the same visual rows.
// The caller owns factor retirement and FEJ Jacobians. delta is the retained
// state's current displacement from its existing Basalt linearization point.
inline basalt::MargLinData<double> marginalizeJointSystem(
    const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& residual,
    const basalt::AbsOrderMap& source,const basalt::AbsOrderMap& retained,
    const Eigen::VectorXd& delta) {
  requireStateOrder(source);requireStateOrder(retained);
  if(jacobian.rows()<1 || jacobian.rows()>4096 || jacobian.cols()>1024 ||
     jacobian.cols()<=Eigen::Index(source.total_size) ||
     delta.size()!=Eigen::Index(retained.total_size) || !delta.allFinite())
    throw std::invalid_argument("invalid_joint_marginal_layout");
  std::vector<int> keep(retained.total_size);std::vector<bool> used(jacobian.cols(),false);
  for(const auto& [stamp,target]:retained.abs_order_map) {
    const auto found=source.abs_order_map.find(stamp);
    if(found==source.abs_order_map.end() || target.second>found->second.second)
      throw std::invalid_argument("unknown_joint_retained_state");
    // A 15-state inertial block may retain only its first six pose coordinates.
    for(int i=0;i<target.second;++i) {
      const int column=found->second.first+i;
      keep[target.first+i]=column;used[column]=true;
    }
  }
  Eigen::MatrixXd ordered(jacobian.rows(),jacobian.cols());int column=0;
  for(int index:keep)ordered.col(column++)=jacobian.col(index);
  for(Eigen::Index index=0;index<jacobian.cols();++index)
    if(!used[index])ordered.col(column++)=jacobian.col(index);
  const auto reduced=eliminateJointPatch(ordered,residual,retained.total_size);
  basalt::MargLinData<double> prior;prior.is_sqrt=true;prior.order=retained;
  prior.H=reduced.jacobian;
  // Basalt evaluates H*(delta + increment)+b, not H*increment+b.
  prior.b=reduced.residual-prior.H*delta;
  if(!prior.H.allFinite() || !prior.b.allFinite())throw std::runtime_error("nonfinite_joint_marginal_prior");
  return prior;
}
}  // namespace posim_slam

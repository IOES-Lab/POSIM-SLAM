// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"
#include "sonar_factors.hpp"
#include <memory>
#include <basalt/imu/preintegration.h>

namespace posim_slam {
struct PreparedPingPrior {
  PingRetirementPlan retirement;
  std::optional<SonarFactorStore::Retirement> acoustic;
  PingMarginalLayout layout;
  basalt::MargLinData<double> prior;
};

// Read-only preparation on the estimator thread. Specs must have been rebuilt
// for the supplied current state order. Commit immediately, without intervening
// geometry/topology changes, using installPingMarginalPrior.
inline PreparedPingPrior preparePingPrior(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& group_order,const std::vector<JointPingGroupSpec>& groups,
    const basalt::MargLinData<double>& previous,const basalt::ImuLinData<double>& imu,
    const std::set<int64_t>& removed,const std::set<int64_t>& pose_only,
    const std::unordered_set<basalt::KeypointId>& lost,SonarFactorStore* sonar=nullptr) {
  PreparedPingPrior result;
  result.retirement=planPingRetirement(owner,groups,removed,removed,lost);
  std::vector<std::pair<int64_t,int64_t>> intervals;
  for(const auto& [start,measurement]:imu.imu_meas) {
    if(!measurement || measurement->get_start_t_ns()!=start)
      throw std::invalid_argument("invalid_ping_prior_imu");
    intervals.emplace_back(start,start+measurement->get_dt_ns());
  }
  auto required=result.retirement.required_frames;
  if(sonar) {
    result.acoustic=sonar->planRetirement(removed);
    required.insert(result.acoustic->required_poses.begin(),result.acoustic->required_poses.end());
  }
  result.layout=planPingMarginalLayout(owner,removed,pose_only,required,intervals);
  requireStateOrder(group_order);
  if(group_order.abs_order_map!=result.layout.source.abs_order_map)
    throw std::invalid_argument("changed_ping_prior_group_order");
  // Include complete independent landmarks hosted by removed frames, too.
  // This makes the installation's explicit removal list match factor ownership.
  for(const auto& [id,point]:owner.lmdb.getLandmarks())
    if(removed.count(point.host_kf_id.frame_id))result.retirement.landmarks.insert(id);
  for(auto id:result.retirement.landmarks)
    if(!owner.lmdb.landmarkExists(id))throw std::invalid_argument("missing_ping_prior_landmark");
  auto consumed=imu;consumed.imu_meas.clear();
  for(int64_t start:result.layout.consumed_imu_starts)
    consumed.imu_meas.emplace(start,imu.imu_meas.at(start));
  PingLinearization::Options options;
  options.lb_options.huber_parameter=owner.huber_thresh;
  options.lb_options.obs_std_dev=owner.obs_std_dev;
  std::unique_ptr<basalt::LinearizationBase<double,6>> rows;
  if(result.retirement.groups.empty())
    rows=std::make_unique<basalt::LinearizationAbsQR<double,6>>(&owner,result.layout.source,
      options,&previous,&consumed,&removed,&result.retirement.landmarks);
  else
    rows=std::make_unique<PingLinearization>(&owner,result.layout.source,options,
      result.retirement.groups,&previous,&consumed,&removed,&result.retirement.landmarks);
  bool valid=false;rows->linearizeProblem(&valid);
  if(!valid)throw std::runtime_error("invalid_ping_prior_linearization");
  rows->performQR();Eigen::MatrixXd j;Eigen::VectorXd r;
  rows->get_dense_Q2Jp_Q2r(j,r);
  if(result.acoustic && !result.acoustic->ids.empty()) {
    SonarPoses poses,references;
    for(const auto& [stamp,block]:result.layout.source.abs_order_map) {
      const auto state=owner.getPoseStateWithLin(stamp);
      poses.emplace(stamp,SonarPose{state.getPose().translation(),state.getPose().rotationMatrix()});
      references.emplace(stamp,SonarPose{state.getPoseLin().translation(),state.getPoseLin().rotationMatrix()});
    }
    const auto acoustic=sonar->linearize(poses,&references);
    for(auto id:result.acoustic->ids) {
      const auto& reduced=acoustic.reduced.at(id);
      const auto start=j.rows(),count=reduced.residual.size();
      j.conservativeResize(start+count,Eigen::NoChange);r.conservativeResize(start+count);
      j.bottomRows(count).setZero();r.tail(count)=reduced.residual;
      for(size_t i=0;i<acoustic.pose_stamps.size();++i)
        j.block(start,result.layout.source.abs_order_map.at(acoustic.pose_stamps[i]).first,count,6)=
          reduced.pose_jacobian.middleCols(6*i,6);
    }
  }
  // New references will be frozen at the current state by installation; their
  // displacement is zero. Existing FEJ references keep their current delta.
  Eigen::VectorXd delta=Eigen::VectorXd::Zero(result.layout.retained.total_size);
  for(const auto& [stamp,block]:result.layout.retained.abs_order_map) {
    if(block.second==15) {
      const auto& state=owner.frame_states.at(stamp);
      if(state.isLinearized())delta.segment<15>(block.first)=state.getDelta();
    } else {
      const auto pose=owner.getPoseStateWithLin(stamp);
      if(pose.isLinearized())delta.segment<6>(block.first)=pose.getDelta();
    }
  }
  result.prior=marginalizeJointSystem(j,r,result.layout.source,result.layout.retained,delta);
  return result;
}
}

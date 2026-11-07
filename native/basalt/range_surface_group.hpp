// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "range_surface.hpp"
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"

namespace posim_slam {
struct RangeSurfaceSupport {
  std::vector<surface_ranges::Key> keys;
  double maximum_deviation,maximum_distance;
};

// One owner, covariance block and elimination for all associated surfaces.
// All source/support poses must still be resident. Retiring a source retires
// the complete group through the existing correlated marginal system.
// require_support=false is only for diagnostic derivative/gauge controls.
inline JointPingGroupSpec makeOwnedRangeSurfaces(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,const std::map<int,RangeSurfaceSupport>& support,
    const std::vector<surface_ranges::Observation>& observations,const Eigen::MatrixXd& covariance,
    const std::string& uncertainty_source,bool require_support=true) {
  requireStateOrder(order);
  if(owner.posim_ping_trial_active || support.empty() || support.size()>8 || observations.empty())
    throw std::invalid_argument("invalid_owned_surface_ranges");
  JointPingGroupSpec spec;spec.range_only=true;spec.plane_id=support.begin()->first;
  std::map<int,PingPlaneReference> references;
  for(const auto& [id,unused]:support) {
    if(id<0)throw std::invalid_argument("invalid_surface_plane_id");
    references.emplace(id,owner.posim_ping_planes.at(id).reference);
    if(id!=spec.plane_id)spec.additional_plane_ids.push_back(id);
  }
  for(const auto& o:observations) {
    if(!order.abs_order_map.count(o.key.stamp) ||
       (!owner.frame_states.count(o.key.stamp) && !owner.frame_poses.count(o.key.stamp)))
      throw std::invalid_argument("surface_dependency_pose_retired");
    owner.getPoseStateWithLin(o.key.stamp);spec.observation_pose_stamps.insert(o.key.stamp);
  }
  spec.build=[&owner,order,support,observations,covariance,uncertainty_source,require_support,references](const auto& blocks) {
    if(!blocks.empty())throw std::invalid_argument("surface_range_group_has_landmarks");
    std::map<int64_t,surface_ranges::Pose> poses;std::map<int,surface_ranges::Plane> planes;
    for(const auto& o:observations) {
      if(poses.count(o.key.stamp))continue;
      if(!owner.frame_states.count(o.key.stamp) && !owner.frame_poses.count(o.key.stamp))
        throw std::invalid_argument("surface_dependency_pose_retired");
      const auto state=owner.getPoseStateWithLin(o.key.stamp);const auto pose=state.getPose();
      surface_ranges::Pose p;p.position=pose.translation();p.attitude=pose.rotationMatrix();
      if(state.isLinearized())p.coordinates=surface_ranges::leftJacobian(state.getDelta().tail<3>());
      poses.emplace(o.key.stamp,p);
    }
    for(const auto& [id,guard]:support) {
      const auto& plane=owner.posim_ping_planes.at(id);const auto& reference=references.at(id);
      if((plane.reference.normal-reference.normal).norm()>1e-12 || std::abs(plane.reference.distance-reference.distance)>1e-12)
        throw std::invalid_argument("changed_surface_plane_reference");
      surface_ranges::Plane p;p.normal=plane.normal;p.distance=plane.distance;
      const auto basis=surface_ranges::normalBasis(reference.normal);
      p.coordinates=surface_ranges::leftJacobian(basis*plane.delta.head<2>())*basis;
      p.support_keys=guard.keys;p.max_plane_deviation=guard.maximum_deviation;p.max_support_distance=guard.maximum_distance;
      planes.emplace(id,p);
    }
    const auto block=surface_ranges::linearize(poses,planes,observations,covariance,uncertainty_source,require_support);
    NativePingGroup out;out.jacobian=Eigen::MatrixXd::Zero(block.jacobian.rows(),order.total_size+3*planes.size());
    out.residual=block.residual;out.auxiliary_cost=.5*out.residual.squaredNorm();
    for(size_t i=0;i<block.state_stamps.size();++i) {
      const int64_t stamp=block.state_stamps[i];out.observation_stamps.push_back(stamp);
      out.jacobian.middleCols(order.abs_order_map.at(stamp).first,6)=block.jacobian.middleCols(6*i,6);
    }
    out.jacobian.rightCols(3*planes.size())=block.jacobian.rightCols(3*planes.size());
    out.reference=references.at(block.plane_ids.front());
    for(size_t i=1;i<block.plane_ids.size();++i)out.additional_references.push_back(references.at(block.plane_ids[i]));
    return out;
  };
  // Validate the complete initial measurement/geometry/covariance boundary
  // before returning a callable that could be installed by an owner.
  spec.build({});return spec;
}
}  // namespace posim_slam

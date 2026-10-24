// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_input.hpp"
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"

namespace posim_slam {
struct OwnedPingObservation {
  int64_t stamp;
  double measured;
  PingInputCalibration calibration;
  double bias,return_sigma;
  std::optional<double> per_ray_sigma;
};

// Snapshot factor membership, measurement values and weights for a whole trial.
// State/landmark geometry is read freshly on each linearization. The owner must
// already own the plane, and must retain these exact factors until retirement.
inline JointPingGroupSpec makeOwnedPingGroup(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,int plane_id,const std::set<basalt::KeypointId>& ids,
    const std::vector<OwnedPingObservation>& observations,double surface_sigma,double support_radius) {
  requireStateOrder(order);
  if(owner.posim_ping_trial_active || ids.size()<8 || ids.size()>64 || observations.empty() || observations.size()>64 ||
     !std::isfinite(surface_sigma) || surface_sigma<=0 || !std::isfinite(support_radius) || support_radius<=0)
    throw std::invalid_argument("invalid_owned_ping_group");
  const auto reference=owner.posim_ping_planes.at(plane_id).reference;
  JointPingGroupSpec spec;spec.plane_id=plane_id;spec.landmark_ids=ids;
  for(const auto& observation:observations) {
    if(observation.stamp<=0 || !order.abs_order_map.count(observation.stamp) ||
       !spec.observation_pose_stamps.insert(observation.stamp).second ||
       !std::isfinite(observation.measured) || !std::isfinite(observation.bias) ||
       observation.measured<observation.calibration.minimum || observation.measured>observation.calibration.maximum ||
       !std::isfinite(observation.return_sigma) || observation.return_sigma<=0 ||
       (observation.per_ray_sigma && (!std::isfinite(*observation.per_ray_sigma) || *observation.per_ray_sigma<=0)))
      throw std::invalid_argument("invalid_owned_ping_observation");
    selectPingPatch({},Sophus::SE3d(),observation.calibration.mount,observation.calibration.rays,
      observation.calibration.minimum,observation.calibration.maximum);
    owner.getPoseStateWithLin(observation.stamp);
  }
  for(auto id:ids) {
    if(!owner.lmdb.landmarkExists(id))throw std::invalid_argument("missing_owned_ping_landmark");
    const auto& point=owner.lmdb.getLandmark(id);
    if(!order.abs_order_map.count(point.host_kf_id.frame_id))throw std::invalid_argument("missing_owned_ping_host");
    for(const auto& [target,pixel]:point.obs)
      if(!order.abs_order_map.count(target.frame_id))throw std::invalid_argument("missing_owned_ping_visual_target");
  }
  spec.build=[&owner,order,plane_id,ids,observations,surface_sigma,support_radius,reference](const auto& blocks) {
    if(blocks.size()!=ids.size())throw std::invalid_argument("changed_owned_ping_membership");
    const auto& plane=owner.posim_ping_planes.at(plane_id);
    if((plane.reference.normal-reference.normal).norm()>1e-12 || std::abs(plane.reference.distance-reference.distance)>1e-12)
      throw std::invalid_argument("changed_owned_ping_reference");
    std::vector<PingGroupLandmark> landmarks;
    for(auto id:ids) {
      const auto& point=owner.lmdb.getLandmark(id);const auto host=point.host_kf_id;
      const auto pose=owner.getPoseStateWithLin(host.frame_id);const auto& mount=owner.calib.T_i_c.at(host.cam_id);
      landmarks.push_back({id,blocks.at(id),worldLandmark(pose.getPose(),mount,point.direction,point.inv_dist),
        order.abs_order_map.at(host.frame_id).first,worldLandmark(pose.getPoseLin(),mount,point.direction,point.inv_dist)});
    }
    std::vector<PingGroupObservation> rows;
    for(const auto& observation:observations) {
      const auto pose=owner.getPoseStateWithLin(observation.stamp);const auto& c=observation.calibration;
      rows.push_back({observation.stamp,observation.stamp,order.abs_order_map.at(observation.stamp).first,
        pose.getPose(),c.mount,c.rays,observation.measured,observation.bias,observation.return_sigma,c.minimum,c.maximum,
        pose.getPoseLin(),observation.per_ray_sigma});
    }
    return assemblePingGroup(order.total_size,plane.normal,plane.distance,landmarks,rows,surface_sigma,support_radius,reference);
  };
  return spec;
}
}

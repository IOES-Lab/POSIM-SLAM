// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "surface_prior.hpp"
#include "ping_linearization.hpp"

namespace posim_slam::surface_prior {
struct ScalarCoordinate {int64_t id;double reference,value;};
struct Coordinates {std::vector<Variable> order;Eigen::VectorXd delta;};

// Read-only bridge to actual Basalt owner coordinates. The fixed references
// belong to the owner, not to a caller's fitted world-plane snapshot. A pressure
// datum remains an explicit scalar with the caller-owned reference and value.
// This does not retire transport factors or install a production marginal prior.
inline Coordinates ownedCoordinates(const basalt::BundleAdjustmentBase<double>& owner,
    const std::vector<Variable>& expected,const std::optional<ScalarCoordinate>& pressure=std::nullopt) {
  const auto shape=layout(expected);Coordinates out;out.delta=Eigen::VectorXd::Zero(shape.count);
  size_t states=0,planes=0,datums=0;Eigen::Index column=0;
  auto poseReference=[](const Sophus::SE3d& pose,Eigen::Index size) {
    Eigen::VectorXd reference(size);reference.head<3>()=pose.translation();
    const auto r=pose.rotationMatrix();
    for(int i=0;i<3;++i)for(int j=0;j<3;++j)reference[3+3*i+j]=r(i,j);
    return reference;
  };
  for(const auto& v:expected) {
    Variable current=v;
    if(v.kind==Kind::State) {
      ++states;
      if(v.size==6) {
        const auto found=owner.frame_poses.find(v.id);
        if(found==owner.frame_poses.end())throw std::invalid_argument("persistent_owner_pose_missing_or_wrong_size");
        current.reference=poseReference(found->second.getPoseLin(),12);
        out.delta.segment<6>(column)=found->second.getDelta();
      } else {
        const auto found=owner.frame_states.find(v.id);
        if(found==owner.frame_states.end())throw std::invalid_argument("persistent_owner_inertial_state_missing");
        const auto& reference=found->second.getStateLin();current.reference=poseReference(reference.T_w_i,21);
        current.reference.segment<3>(12)=reference.vel_w_i;
        current.reference.segment<3>(15)=reference.bias_gyro;current.reference.segment<3>(18)=reference.bias_accel;
        out.delta.segment<15>(column)=found->second.getDelta();
      }
    } else if(v.kind==Kind::Plane) {
      ++planes;
      if(v.id>std::numeric_limits<int>::max())throw std::invalid_argument("persistent_owner_plane_id");
      const auto& plane=owner.posim_ping_planes.at(int(v.id));current.reference.resize(4);
      current.reference<<plane.reference.normal,plane.reference.distance;
      out.delta.segment<3>(column)=plane.delta;
    } else {
      ++datums;
      if(!pressure || pressure->id!=v.id || !std::isfinite(pressure->reference) || !std::isfinite(pressure->value))
        throw std::invalid_argument("explicit_owned_pressure_coordinate_required");
      current.reference=Eigen::VectorXd::Constant(1,pressure->reference);out.delta[column]=pressure->value-pressure->reference;
    }
    column+=v.size;out.order.push_back(current);
  }
  if(states!=owner.frame_poses.size()+owner.frame_states.size() || planes!=owner.posim_ping_planes.size() ||
     datums!=size_t(pressure.has_value()))throw std::invalid_argument("complete_persistent_owner_order_required");
  layout(out.order);return out;
}
}  // namespace posim_slam::surface_prior

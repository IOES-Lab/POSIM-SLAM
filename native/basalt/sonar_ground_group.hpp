// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"
#include <basalt/utils/sophus_utils.hpp>

namespace posim_slam {
struct GroundEntryObservation {
  int64_t stamp;
  std::vector<int> beam_ids;
  Eigen::VectorXd azimuth,ranges;
  int boundary_sign;
};
struct GroundEntryCalibration {
  Sophus::SE3d mount;
  double vertical_fov,minimum,maximum;
};

// Fixed membership and covariance throughout an optimization/marginalization
// transaction. Beam endpoints are conditional aperture intersections, not fixed
// 3D scatterers. No normal/height prior or fitted-plane covariance is installed.
inline JointPingGroupSpec makeOwnedGroundGroup(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,int plane_id,const GroundEntryCalibration& calibration,
    const std::vector<GroundEntryObservation>& observations,const Eigen::MatrixXd& covariance) {
  requireStateOrder(order);
  constexpr double pi=3.14159265358979323846;
  if(owner.posim_ping_trial_active || observations.empty() || observations.size()>16 ||
     !calibration.mount.matrix().allFinite() || !std::isfinite(calibration.vertical_fov) ||
     calibration.vertical_fov<=0 || calibration.vertical_fov>=pi ||
     !std::isfinite(calibration.minimum) || !std::isfinite(calibration.maximum) ||
     !(0<calibration.minimum && calibration.minimum<calibration.maximum))
    throw std::invalid_argument("invalid_ground_group");
  JointPingGroupSpec spec;spec.plane_id=plane_id;spec.range_only=true;
  Eigen::Index count=0;int64_t previous=0;
  for(const auto& o:observations) {
    if(o.stamp<=previous || !order.abs_order_map.count(o.stamp) ||
       o.ranges.size()<8 || o.ranges.size()>128 || o.azimuth.size()!=o.ranges.size() ||
       o.beam_ids.size()!=size_t(o.ranges.size()) || !o.ranges.allFinite() || !o.azimuth.allFinite() ||
       o.ranges.minCoeff()<calibration.minimum || o.ranges.maxCoeff()>calibration.maximum ||
       o.azimuth.cwiseAbs().maxCoeff()>=pi/2 || std::abs(o.boundary_sign)!=1)
      throw std::invalid_argument("invalid_ground_observation");
    std::set<int> ids;
    for(Eigen::Index k=0;k<o.ranges.size();++k)
      if(o.beam_ids[k]<0 || !ids.insert(o.beam_ids[k]).second || (k && o.azimuth[k]<=o.azimuth[k-1]))
        throw std::invalid_argument("invalid_ground_beam_identity");
    owner.getPoseStateWithLin(o.stamp);spec.observation_pose_stamps.insert(o.stamp);
    previous=o.stamp;count+=o.ranges.size();
  }
  if(count>512 || covariance.rows()!=count || covariance.cols()!=count || !covariance.allFinite() ||
     (covariance-covariance.transpose()).cwiseAbs().maxCoeff()>1e-10)
    throw std::invalid_argument("invalid_ground_covariance");
  Eigen::LLT<Eigen::MatrixXd> factor(covariance);
  if(factor.info()!=Eigen::Success)throw std::invalid_argument("positive_ground_covariance_required");
  const Eigen::MatrixXd lower=factor.matrixL();
  const auto reference=owner.posim_ping_planes.at(plane_id).reference;
  spec.build=[&owner,order,plane_id,calibration,observations,lower,reference,count](const auto& blocks) {
    if(!blocks.empty())throw std::invalid_argument("ground_group_has_landmarks");
    const auto& plane=owner.posim_ping_planes.at(plane_id);
    if((plane.reference.normal-reference.normal).norm()>1e-12 || std::abs(plane.reference.distance-reference.distance)>1e-12)
      throw std::invalid_argument("changed_ground_reference");
    NativePingGroup out;out.reference=reference;
    out.jacobian=Eigen::MatrixXd::Zero(count,order.total_size+3);
    out.residual=Eigen::VectorXd::Zero(count);Eigen::Index row=0;
    // The plane has no retained prior: it is eliminated when its group retires.
    // Its fixed reference defines additive coordinates, not frozen geometry.
    // Differentiate Exp(B_ref * delta) n_ref at the current accumulated delta.
    const auto reference_basis=planeBasis(reference.normal);
    Mat3 plane_left;Sophus::leftJacobianSO3(reference_basis*plane.delta.head<2>(),plane_left);
    const Eigen::Matrix2d plane_coordinates=planeBasis(plane.normal).transpose()*plane_left*reference_basis;
    for(const auto& o:observations) {
      const auto state=owner.getPoseStateWithLin(o.stamp);const auto pose=state.getPose();
      Mat3 pose_coordinates=Mat3::Identity();
      if(state.isLinearized())Sophus::leftJacobianSO3(state.getDelta().tail<3>(),pose_coordinates);
      const auto sensor=pose*calibration.mount;
      const Vec3 local_normal=sensor.rotationMatrix().transpose()*plane.normal;
      const double a=plane.distance-plane.normal.dot(sensor.translation());
      const double side=a>0 ? 1. : -1.;
      out.observation_stamps.push_back(o.stamp);
      const double half=calibration.vertical_fov/2,elevation=o.boundary_sign*half;
      for(Eigen::Index k=0;k<o.ranges.size();++k,++row) {
        const double az=o.azimuth[k];Eigen::Matrix<double,1,3> ray;
        ray<<std::cos(az)*std::cos(elevation),std::sin(az)*std::cos(elevation),std::sin(elevation);
        // Closest positive intersection maximizes side*n.dot(ray) over the
        // entire vertical aperture, including an interior stationary point.
        const double b=side*(local_normal.x()*std::cos(az)+local_normal.y()*std::sin(az));
        const double c=side*local_normal.z();
        auto denominator=[&](double e){return b*std::cos(e)+c*std::sin(e);};
        double maximum=std::max(denominator(-half),denominator(half));
        const double stationary=std::atan2(c,b);
        if(stationary>=-half && stationary<=half)maximum=std::max(maximum,denominator(stationary));
        const auto current=pingRange(pose,calibration.mount,ray,plane.normal,plane.distance,0.);
        if(maximum<=0 || std::abs(current.prediction-std::abs(a)/maximum)>1e-7)
          throw std::invalid_argument("ground_first_return_branch_changed");
        if(current.prediction<calibration.minimum || current.prediction>calibration.maximum)
          throw std::invalid_argument("predicted_ground_out_of_range");
        // Active nonlinear measurements use current geometry. Retained pose
        // states add increments to an accumulated rotation vector, so apply
        // its left Jacobian rather than assuming a fresh zero increment.
        // Marginalization separately rebases the resulting linear prior.
        out.residual[row]=current.prediction-o.ranges[k];
        const auto column=order.abs_order_map.at(o.stamp).first;
        out.jacobian.block<1,3>(row,column)=current.pose_jacobian.head<3>();
        out.jacobian.block<1,3>(row,column+3)=current.pose_jacobian.tail<3>()*pose_coordinates;
        out.jacobian.block<1,2>(row,order.total_size)=current.plane_jacobian.head<2>()*plane_coordinates;
        out.jacobian(row,order.total_size+2)=current.plane_jacobian[2];
      }
    }
    out.residual=lower.triangularView<Eigen::Lower>().solve(out.residual).eval();
    out.jacobian=lower.triangularView<Eigen::Lower>().solve(out.jacobian).eval();
    out.auxiliary_cost=.5*out.residual.squaredNorm();
    if(!out.residual.allFinite() || !out.jacobian.allFinite() || !std::isfinite(out.auxiliary_cost))
      throw std::invalid_argument("nonfinite_ground_linearization");
    return out;
  };
  return spec;
}
}  // namespace posim_slam

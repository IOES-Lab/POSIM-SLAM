// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"
#include <basalt/utils/sophus_utils.hpp>

namespace posim_slam {
struct SonarPlaneReturn {
  int source_id,plane_id;
  double range;
  Eigen::MatrixXd rays;
};
struct SonarPlaneFrame {int64_t stamp;std::vector<SonarPlaneReturn> returns;};

// Full row covariance spans every source, surface and timestamp. One group owns
// the whole acoustic frame set; no independently whitened per-surface factors.
inline JointPingGroupSpec makeOwnedSonarPlanes(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,const Sophus::SE3d& mount,double minimum,double maximum,
    const std::vector<SonarPlaneFrame>& frames,const Eigen::MatrixXd& covariance) {
  requireStateOrder(order);
  if(owner.posim_ping_trial_active || frames.empty() || frames.size()>16 ||
     !mount.matrix().allFinite() || !std::isfinite(minimum) || !std::isfinite(maximum) ||
     !(0<minimum && minimum<maximum))throw std::invalid_argument("invalid_sonar_plane_group");
  std::map<int,int> source_planes;
  std::set<int> ids;std::set<int64_t> stamps;int64_t previous=0;Eigen::Index count=0;
  for(const auto& frame:frames) {
    if(frame.stamp<=previous || !order.abs_order_map.count(frame.stamp) || frame.returns.empty() || frame.returns.size()>128)
      throw std::invalid_argument("invalid_sonar_plane_frame");
    previous=frame.stamp;stamps.insert(frame.stamp);owner.getPoseStateWithLin(frame.stamp);
    std::set<int> sources;
    for(const auto& row:frame.returns) {
      if(row.source_id<0 || !sources.insert(row.source_id).second || row.plane_id<0 ||
         !std::isfinite(row.range) || row.range<minimum || row.range>maximum ||
         row.rays.rows()<1 || row.rays.rows()>512 || row.rays.cols()!=3 || !row.rays.allFinite() ||
         (row.rays.rowwise().squaredNorm().array()-1.).abs().maxCoeff()>1e-8)
        throw std::invalid_argument("invalid_sonar_plane_return");
      const auto [it,inserted]=source_planes.emplace(row.source_id,row.plane_id);
      if(!inserted && it->second!=row.plane_id)throw std::invalid_argument("sonar_source_plane_changed");
      ids.insert(row.plane_id);++count;
    }
  }
  if(ids.empty() || ids.size()>8 || count<3 || count>512 || covariance.rows()!=count || covariance.cols()!=count ||
     !covariance.allFinite() || (covariance-covariance.transpose()).cwiseAbs().maxCoeff()>1e-10)
    throw std::invalid_argument("invalid_sonar_plane_covariance_or_budget");
  Eigen::LLT<Eigen::MatrixXd> factor(covariance);
  if(factor.info()!=Eigen::Success)throw std::invalid_argument("positive_sonar_plane_covariance_required");
  const Eigen::MatrixXd lower=factor.matrixL();
  JointPingGroupSpec spec;spec.range_only=true;spec.plane_id=*ids.begin();
  spec.additional_plane_ids.assign(std::next(ids.begin()),ids.end());spec.observation_pose_stamps=stamps;
  const auto planes=spec.planeIds();std::vector<PingPlaneReference> references;
  for(int id:planes)references.push_back(owner.posim_ping_planes.at(id).reference);
  spec.build=[&owner,order,mount,minimum,maximum,frames,lower,count,planes,references](const auto& blocks) {
    if(!blocks.empty())throw std::invalid_argument("sonar_plane_group_has_landmarks");
    NativePingGroup out;out.reference=references.front();
    out.additional_references.assign(std::next(references.begin()),references.end());
    out.jacobian=Eigen::MatrixXd::Zero(count,order.total_size+3*planes.size());out.residual=Eigen::VectorXd::Zero(count);
    std::vector<Eigen::Matrix<double,3,2>> plane_coordinates;
    for(size_t k=0;k<planes.size();++k) {
      const auto& p=owner.posim_ping_planes.at(planes[k]);const auto& ref=references[k];
      if((p.reference.normal-ref.normal).norm()>1e-12 || std::abs(p.reference.distance-ref.distance)>1e-12)
        throw std::invalid_argument("changed_sonar_plane_reference");
      Mat3 left;const auto basis=planeBasis(ref.normal);Sophus::leftJacobianSO3(basis*p.delta.head<2>(),left);
      plane_coordinates.push_back(left*basis);
    }
    Eigen::Index row=0;
    for(const auto& frame:frames) {
      out.observation_stamps.push_back(frame.stamp);
      const auto state=owner.getPoseStateWithLin(frame.stamp);const auto pose=state.getPose();
      const auto sensor=pose*mount;const Vec3 origin=sensor.translation();
      Mat3 pose_coordinates=Mat3::Identity();
      if(state.isLinearized())Sophus::leftJacobianSO3(state.getDelta().tail<3>(),pose_coordinates);
      for(const auto& observation:frame.returns) {
        const size_t pi=std::find(planes.begin(),planes.end(),observation.plane_id)-planes.begin();
        const auto& plane=owner.posim_ping_planes.at(observation.plane_id);const Vec3 n=plane.normal;
        const Eigen::MatrixXd rays=observation.rays*sensor.rotationMatrix().transpose();
        const Eigen::VectorXd denominators=rays*n;const double a=plane.distance-n.dot(origin);
        double nearest=std::numeric_limits<double>::infinity();Eigen::Index selected=-1;int ties=0;
        for(Eigen::Index i=0;i<rays.rows();++i) {
          if(std::abs(denominators[i])<=1e-8)continue;
          const double distance=a/denominators[i];
          if(distance<minimum || distance>maximum)continue;
          if(distance<nearest-1e-9) {nearest=distance;selected=i;ties=1;}
          else if(std::abs(distance-nearest)<=1e-9)++ties;
        }
        if(selected<0)throw std::invalid_argument("sonar_plane_return_unavailable");
        if(ties!=1)throw std::invalid_argument("sonar_plane_branch_ambiguous");
        const double b=denominators[selected];const Vec3 direction=rays.row(selected).transpose();
        const Vec3 hit=origin+nearest*direction;
        const Eigen::RowVector3d jo=-n.transpose()/b,jd=nearest*jo;
        const auto column=order.abs_order_map.at(frame.stamp).first;
        out.residual[row]=nearest-observation.range;
        out.jacobian.block<1,3>(row,column)=jo;
        out.jacobian.block<1,3>(row,column+3)=(-jo*crossMatrix(pose.rotationMatrix()*mount.translation())-jd*crossMatrix(direction))*pose_coordinates;
        out.jacobian.block<1,2>(row,order.total_size+3*pi)=hit.transpose()*crossMatrix(n)/b*plane_coordinates[pi];
        out.jacobian(row,order.total_size+3*pi+2)=1/b;++row;
      }
    }
    out.residual=lower.triangularView<Eigen::Lower>().solve(out.residual).eval();
    out.jacobian=lower.triangularView<Eigen::Lower>().solve(out.jacobian).eval();
    out.auxiliary_cost=.5*out.residual.squaredNorm();
    if(!out.residual.allFinite() || !out.jacobian.allFinite() || !std::isfinite(out.auxiliary_cost))
      throw std::invalid_argument("nonfinite_sonar_plane_linearization");
    return out;
  };
  return spec;
}
} // namespace posim_slam

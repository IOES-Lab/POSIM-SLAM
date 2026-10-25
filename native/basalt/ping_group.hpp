// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_geometry.hpp"
#include "ping_joint.hpp"
#include <basalt/linearization/landmark_block.hpp>
#include <set>
#include <vector>
#include <optional>

namespace posim_slam {
struct PingGroupLandmark {
  basalt::KeypointId id;
  basalt::LandmarkBlock<double>* block;
  WorldLandmark geometry;
  Eigen::Index host_column;
  std::optional<WorldLandmark> reference_geometry;
};
enum class PingRangeLoss { Huber, Cauchy };
struct PingGroupObservation {
  int64_t stamp_ns;
  int64_t pose_stamp_ns;
  Eigen::Index pose_column;
  Sophus::SE3d pose,mount;
  Eigen::MatrixXd rays;
  double measured,bias,sigma,minimum,maximum;
  std::optional<Sophus::SE3d> reference_pose;
  // Optional expected-minimum model. sigma remains the fixed return residual
  // weight, including any separately justified model uncertainty; it is not
  // silently replaced by the geometry-dependent ray-selection variance.
  std::optional<double> per_ray_sigma;
  std::optional<double> range_loss_scale=3.;
  PingRangeLoss range_loss=PingRangeLoss::Huber;
};

inline void requireNativeSurfaceSupport(const Eigen::MatrixXd& points,const Eigen::MatrixXd& queries,
                                       const Vec3& normal,double radius) {
  if(!std::isfinite(radius) || radius<=0 || !points.allFinite() || !queries.allFinite())
    throw std::invalid_argument("invalid_surface_support");
  const auto basis=planeBasis(normal);const Vec3 origin=points.colwise().mean().transpose();
  const Eigen::MatrixXd p=(points.rowwise()-origin.transpose())*basis/radius;
  const Eigen::MatrixXd q=(queries.rowwise()-origin.transpose())*basis/radius;
  using Point=std::pair<double,double>;std::set<Point> sorted;
  for(Eigen::Index i=0;i<p.rows();++i)sorted.emplace(p(i,0),p(i,1));
  auto cross=[](Point a,Point b,Point c){return (b.first-a.first)*(c.second-a.second)-(b.second-a.second)*(c.first-a.first);};
  std::vector<Point> order(sorted.begin(),sorted.end());
  auto chain=[&](const std::vector<Point>& sequence) {
    std::vector<Point> result;
    for(const auto& x:sequence) {
      while(result.size()>=2 && cross(result[result.size()-2],result.back(),x)<=0)result.pop_back();
      result.push_back(x);
    }
    if(!result.empty())result.pop_back();return result;
  };
  auto hull=chain(order);std::reverse(order.begin(),order.end());const auto upper=chain(order);
  hull.insert(hull.end(),upper.begin(),upper.end());
  if(hull.size()<3)throw std::invalid_argument("degenerate_surface_support");
  for(Eigen::Index i=0;i<q.rows();++i) {
    Point query(q(i,0),q(i,1));
    for(size_t k=0;k<hull.size();++k)
      if(cross(hull[k],hull[(k+1)%hull.size()],query)<-1e-10)
        throw std::invalid_argument("surface_does_not_cover_ping");
    if((p.rowwise()-q.row(i)).rowwise().squaredNorm().minCoeff()>1.+1e-10)
      throw std::invalid_argument("surface_support_gap");
  }
}

struct NativePingGroup {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  std::vector<basalt::LandmarkBlock<double>*> blocks;
  std::vector<int64_t> observation_stamps;
  double auxiliary_cost=0.;
  std::optional<PingPlaneReference> reference;
  // Called only by the estimator owner thread after accepting a whole trial.
  // The first three nuisance coordinates belong to the caller-owned plane.
  bool applyLandmarks(const Eigen::VectorXd& nuisance) const {
    if(nuisance.size()!=3+3*Eigen::Index(blocks.size()) || !nuisance.allFinite())return false;
    for(size_t i=0;i<blocks.size();++i)
      if(!blocks[i]->jointIncrementValid(nuisance.segment<3>(3+3*i)))return false;
    for(size_t i=0;i<blocks.size();++i)blocks[i]->applyJointIncrement(nuisance.segment<3>(3+3*i));
    return true;
  }
};

inline NativePingGroup assemblePingGroup(Eigen::Index pose_columns,const Vec3& normal,double distance,
    const std::vector<PingGroupLandmark>& landmarks,const std::vector<PingGroupObservation>& observations,
    double surface_sigma,double support_radius,
    const std::optional<PingPlaneReference>& reference=std::nullopt,bool range_only=false) {
  planeBasis(normal);
  if(reference) {planeBasis(reference->normal);if(!std::isfinite(reference->distance))throw std::invalid_argument("invalid_plane_reference");}
  if(pose_columns<6 || pose_columns>kMaxStateColumns || (range_only ? !landmarks.empty() : landmarks.size()<3) || landmarks.size()>kMaxPingLandmarks ||
     observations.empty() || observations.size()>64 || !std::isfinite(distance) ||
     !std::isfinite(surface_sigma) || surface_sigma<=0)
    throw std::invalid_argument("invalid_ping_group");
  const Eigen::Index count=landmarks.size(),columns=pose_columns+3+3*count;
  std::set<basalt::KeypointId> ids;std::set<basalt::LandmarkBlock<double>*> blocks;std::set<int64_t> stamps;
  std::vector<Eigen::MatrixXd> jp(count),jl(count);std::vector<Eigen::VectorXd> rv(count);
  Eigen::MatrixXd points(count,3);Eigen::Index rows=count+observations.size();
  for(Eigen::Index i=0;i<count;++i) {
    const auto& l=landmarks[i];
    if(l.reference_geometry.has_value()!=reference.has_value())throw std::invalid_argument("mixed_ping_linearization_reference");
    if(!l.block || !ids.insert(l.id).second || !blocks.insert(l.block).second ||
       l.host_column<0 || l.host_column+6>pose_columns)
      throw std::invalid_argument("duplicate_or_invalid_ping_landmark");
    l.block->exportJointLinearization(jp[i],jl[i],rv[i]);
    if(jp[i].cols()!=pose_columns || jl[i].cols()!=3 || jp[i].rows()!=jl[i].rows() ||
       rv[i].size()!=jp[i].rows())throw std::invalid_argument("joint_visual_row_layout");
    rows+=rv[i].size();points.row(i)=l.geometry.point.transpose();
  }
  if(rows>4096)throw std::invalid_argument("ping_group_row_limit");
  NativePingGroup group;group.reference=reference;group.jacobian=Eigen::MatrixXd::Zero(rows,columns);
  group.residual=Eigen::VectorXd::Zero(rows);Eigen::Index offset=0;
  for(Eigen::Index i=0;i<count;++i) {
    const auto size=rv[i].size();group.jacobian.block(offset,0,size,pose_columns)=jp[i];
    group.jacobian.block(offset,pose_columns+3+3*i,size,3)=jl[i];
    group.residual.segment(offset,size)=rv[i];offset+=size;group.blocks.push_back(landmarks[i].block);
    auto row=surfaceRow(landmarks[i].geometry,normal,distance);
    if(reference) {
      auto fixed=surfaceRow(*landmarks[i].reference_geometry,reference->normal,reference->distance);
      fixed.residual=row.residual;row=fixed;
    }
    group.residual[offset]=row.residual/surface_sigma;
    group.auxiliary_cost+=.5*group.residual[offset]*group.residual[offset];
    group.jacobian.block<1,6>(offset,landmarks[i].host_column)=row.host_jacobian/surface_sigma;
    group.jacobian.block<1,3>(offset,pose_columns)=row.plane_jacobian/surface_sigma;
    group.jacobian.block<1,3>(offset,pose_columns+3+3*i)=row.landmark_jacobian/surface_sigma;++offset;
  }
  for(const auto& observation:observations) {
    if(observation.reference_pose.has_value()!=reference.has_value())throw std::invalid_argument("mixed_ping_linearization_reference");
    if(observation.stamp_ns<=0 || observation.stamp_ns!=observation.pose_stamp_ns ||
       !stamps.insert(observation.stamp_ns).second ||
       observation.pose_column<0 || observation.pose_column+6>pose_columns ||
       !std::isfinite(observation.sigma) || observation.sigma<=0 ||
       (observation.range_loss_scale && (!std::isfinite(*observation.range_loss_scale) || *observation.range_loss_scale<=0)) ||
       !std::isfinite(observation.minimum) || !std::isfinite(observation.maximum) ||
       !(observation.minimum>0 && observation.minimum<observation.maximum) ||
       !std::isfinite(observation.measured) || observation.measured<observation.minimum || observation.measured>observation.maximum)
      throw std::invalid_argument("invalid_or_duplicate_ping_observation");
    auto predict=[&](const Sophus::SE3d& pose,const Vec3& n,double d) {
      if(observation.per_ray_sigma)
        return expectedPingRange(pose,observation.mount,observation.rays,n,d,observation.bias,
          *observation.per_ray_sigma,observation.minimum,observation.maximum).range;
      return pingRange(pose,observation.mount,observation.rays,n,d,observation.bias);
    };
    auto row=predict(observation.pose,normal,distance);
    if(reference) {
      // Evaluate at the fixed pose/plane reference used by the marginal prior.
      // The hard-minimum path retains its current active beam; expected
      // return evaluates all ray probabilities at the reference.
      const auto fixed=observation.per_ray_sigma ?
          predict(*observation.reference_pose,reference->normal,reference->distance) :
          pingRange(*observation.reference_pose,observation.mount,
            observation.rays.row(row.active_ray),reference->normal,reference->distance,observation.bias);
      row.pose_jacobian=fixed.pose_jacobian;row.plane_jacobian=fixed.plane_jacobian;
    }
    if(row.prediction<observation.minimum || row.prediction>observation.maximum ||
       row.prediction-observation.bias<observation.minimum || row.prediction-observation.bias>observation.maximum)
      throw std::invalid_argument("predicted_ping_out_of_range");
    if(!range_only)requireNativeSurfaceSupport(points,row.intersections,normal,support_radius);
    const double error=(row.prediction-observation.measured)/observation.sigma;
    const double magnitude=std::abs(error);
    const bool tail=observation.range_loss_scale && magnitude>*observation.range_loss_scale;
    double weight=tail ? *observation.range_loss_scale/magnitude : 1.;
    double cost=tail ? *observation.range_loss_scale*(magnitude-.5 * (*observation.range_loss_scale)) : .5*error*error;
    if(observation.range_loss_scale && observation.range_loss==PingRangeLoss::Cauchy) {
      const double c=*observation.range_loss_scale,ratio=error/c;
      weight=1./(1.+ratio*ratio);cost=.5*c*c*std::log1p(ratio*ratio);
    }
    // IRLS rows use the current robust weight, while trial acceptance evaluates
    // the actual robust cost. The FEJ geometric Jacobians remain unchanged.
    group.auxiliary_cost+=cost;
    const double scale=std::sqrt(weight)/observation.sigma;
    group.residual[offset]=std::sqrt(weight)*error;
    group.observation_stamps.push_back(observation.stamp_ns);
    group.jacobian.block<1,6>(offset,observation.pose_column)=row.pose_jacobian*scale;
    group.jacobian.block<1,3>(offset,pose_columns)=row.plane_jacobian*scale;++offset;
  }
  if(!group.jacobian.allFinite() || !group.residual.allFinite())throw std::invalid_argument("nonfinite_ping_group");
  return group;
}
}  // namespace posim_slam

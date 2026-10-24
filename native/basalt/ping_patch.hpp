// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_group.hpp"
#include <Eigen/Eigenvalues>
#include <random>

namespace posim_slam {
struct PingPatchPoint {basalt::KeypointId id;Vec3 point;};
struct PingPatchDiagnostics {size_t local_points=0,max_inliers=0;std::map<std::string,size_t> rejected;};
struct PingPatch {
  Vec3 normal;double distance,rms,predicted_range,support_radius;
  std::vector<basalt::KeypointId> ids;
};

// Estimate the spatial resolution from independent visual samples, before any
// solver/display downsampling. Dense patches retain small hole sensitivity;
// sparse patches permit interpolation only at their measured sampling scale.
// The range-relative ceiling prevents an arbitrarily sparse patch from claiming
// full beam coverage. This is a resolution bound, not proof of solid terrain.
inline double pingPatchSupportRadius(const Eigen::MatrixXd& points,const Vec3& normal,double range) {
  if(points.rows()<8 || points.cols()!=3 || !points.allFinite() || !std::isfinite(range) || range<=0)
    throw std::invalid_argument("invalid_patch_resolution");
  const Eigen::MatrixXd projected=points*planeBasis(normal);
  std::vector<double> spacing;
  for(Eigen::Index i=0;i<points.rows();++i) {
    double nearest=std::numeric_limits<double>::infinity();
    for(Eigen::Index j=0;j<points.rows();++j)if(i!=j) {
      const double d=(projected.row(i)-projected.row(j)).norm();
      if(d>1e-8*range)nearest=std::min(nearest,d);
    }
    if(std::isfinite(nearest))spacing.push_back(nearest);
  }
  if(spacing.size()<8)throw std::invalid_argument("degenerate_patch_resolution");
  std::sort(spacing.begin(),spacing.end());
  const double median=.5*(spacing[(spacing.size()-1)/2]+spacing[spacing.size()/2]);
  return std::min(.10*range,1.5*median);
}

// Candidates must be active, repeatedly supported, full-weight visual tracks.
// The fit only initializes a shared variable; it is never an independent plane
// measurement. No horizontal seabed or measured-range scale is assumed here.
inline std::optional<PingPatch> selectPingPatch(const std::vector<PingPatchPoint>& input,
    const Sophus::SE3d& pose,const Sophus::SE3d& mount,const Eigen::MatrixXd& rays,
    double minimum,double maximum,double surface_tolerance=.03,PingPatchDiagnostics* diagnostics=nullptr) {
  if(diagnostics)*diagnostics={};
  if(input.size()>100000 || !pose.matrix().allFinite() || !mount.matrix().allFinite() ||
     rays.rows()<1 || rays.rows()>256 || rays.cols()!=3 || !rays.allFinite() ||
     !std::isfinite(minimum) || !std::isfinite(maximum) || !(0<minimum && minimum<maximum) ||
     !std::isfinite(surface_tolerance) || surface_tolerance<=0)
    throw std::invalid_argument("invalid_ping_patch_input");
  const Sophus::SE3d sensor=pose*mount;const Vec3 origin=sensor.translation();
  Vec3 axis=rays.colwise().mean().transpose();
  if(axis.norm()<.5)throw std::invalid_argument("invalid_ping_patch_rays");axis.normalize();
  double cosine=1.;
  for(Eigen::Index i=0;i<rays.rows();++i) {
    if(std::abs(rays.row(i).squaredNorm()-1.)>1e-8)throw std::invalid_argument("invalid_ping_patch_rays");
    cosine=std::min(cosine,rays.row(i).dot(axis));
  }
  if(cosine<.5)throw std::invalid_argument("unsupported_ping_patch_cone");
  const double cone=std::acos(cosine),expanded=std::cos(cone+std::atan(.08));
  const Vec3 world_axis=sensor.rotationMatrix()*axis;
  std::set<basalt::KeypointId> unique;std::vector<PingPatchPoint> local;
  for(const auto& point:input) {
    if(!unique.insert(point.id).second || !point.point.allFinite())throw std::invalid_argument("invalid_ping_patch_points");
    const Vec3 relative=point.point-origin;const double length=relative.norm();
    if(length>=minimum && length<=maximum && relative.dot(world_axis)>=expanded*length)local.push_back(point);
  }
  if(diagnostics)diagnostics->local_points=local.size();
  if(local.size()<8)return std::nullopt;
  std::sort(local.begin(),local.end(),[](const auto& a,const auto& b){return a.id<b.id;});
  // Spatial sampling keeps edge coverage rather than taking the first IDs.
  auto spread=[](const std::vector<PingPatchPoint>& points,size_t limit) {
    std::vector<PingPatchPoint> result;std::vector<double> nearest(points.size(),std::numeric_limits<double>::infinity());
    std::vector<bool> chosen(points.size(),false);size_t next=0;
    while(result.size()<std::min(points.size(),limit)) {
      chosen[next]=true;result.push_back(points[next]);double farthest=-1.;size_t candidate=next;
      for(size_t i=0;i<points.size();++i)if(!chosen[i]) {
        nearest[i]=std::min(nearest[i],(points[i].point-points[next].point).squaredNorm());
        if(nearest[i]>farthest){farthest=nearest[i];candidate=i;}
      }
      next=candidate;
    }
    return result;
  };
  const auto sampled=spread(local,128);std::optional<PingPatch> best;
  std::mt19937 random(47);std::uniform_int_distribution<size_t> index(0,sampled.size()-1);
  for(int attempt=0;attempt<256;++attempt) {
    const auto a=index(random),b=index(random),c=index(random);if(a==b || a==c || b==c)continue;
    Vec3 normal=(sampled[b].point-sampled[a].point).cross(sampled[c].point-sampled[a].point);
    if(normal.norm()<1e-8)continue;normal.normalize();double distance=normal.dot(sampled[a].point);
    std::vector<PingPatchPoint> inliers;
    for(const auto& point:sampled)if(std::abs(normal.dot(point.point)-distance)<=surface_tolerance)inliers.push_back(point);
    if(diagnostics)diagnostics->max_inliers=std::max(diagnostics->max_inliers,inliers.size());
    if(inliers.size()<8)continue;
    Vec3 center=Vec3::Zero();for(const auto& point:inliers)center+=point.point;center/=double(inliers.size());
    Mat3 scatter=Mat3::Zero();for(const auto& point:inliers) {const Vec3 d=point.point-center;scatter.noalias()+=d*d.transpose();}
    Eigen::SelfAdjointEigenSolver<Mat3> fit(scatter);
    if(fit.info()!=Eigen::Success || fit.eigenvalues()[1]<=1e-8)continue;
    normal=fit.eigenvectors().col(0);if(normal.dot(world_axis)<0)normal=-normal;distance=normal.dot(center);
    double squared=0.;bool coherent=true;
    for(const auto& point:inliers) {
      const double error=normal.dot(point.point)-distance;squared+=error*error;
      if(std::abs(error)>surface_tolerance)coherent=false;
    }
    if(!coherent)continue;
    try {
      const auto projected=pingRange(pose,mount,rays,normal,distance,0.);
      if(projected.prediction<minimum || projected.prediction>maximum) {if(diagnostics)++diagnostics->rejected["range_bounds"];continue;}
      Eigen::MatrixXd observed(inliers.size(),3);
      for(size_t i=0;i<inliers.size();++i)observed.row(i)=inliers[i].point.transpose();
      const double support=pingPatchSupportRadius(observed,normal,projected.prediction);
      auto selected=spread(inliers,64);Eigen::MatrixXd points(selected.size(),3);
      for(size_t i=0;i<selected.size();++i)points.row(i)=selected[i].point.transpose();
      requireNativeSurfaceSupport(points,projected.intersections,normal,support);
      // A supported farther plane must not explain a closer observed object.
      bool occluded=false;
      for(const auto& point:local) {
        const Vec3 direction=point.point-origin;const double range=direction.norm();
        if(direction.dot(world_axis)<cosine*range)continue;
        const double denominator=normal.dot(direction/range);
        if(denominator>.1 && range<(distance-normal.dot(origin))/denominator-3*surface_tolerance) {occluded=true;break;}
      }
      if(occluded) {if(diagnostics)++diagnostics->rejected["foreground_occlusion"];continue;}
      const double rms=std::sqrt(squared/inliers.size());
      if(best && projected.prediction>best->predicted_range+surface_tolerance)continue;
      if(best && std::abs(projected.prediction-best->predicted_range)<=surface_tolerance && rms>=best->rms)continue;
      PingPatch patch{normal,distance,rms,projected.prediction,support,{}};
      for(const auto& point:selected)patch.ids.push_back(point.id);std::sort(patch.ids.begin(),patch.ids.end());best=std::move(patch);
      if(inliers.size()==sampled.size())return best;
    }catch(const std::invalid_argument& error) {if(diagnostics)++diagnostics->rejected[error.what()];continue;}
  }
  return best;
}
}  // namespace posim_slam

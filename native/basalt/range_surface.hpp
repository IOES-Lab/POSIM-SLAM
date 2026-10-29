// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

// Conditional per-beam ranges with unknown local surfaces. This header has no
// Basalt, simulator or application dependency. Echo identity/noise provenance
// are caller responsibilities; nominal support seeds are never prior rows.
namespace posim_slam::surface_ranges {
using V3=Eigen::Vector3d;
using M3=Eigen::Matrix3d;
using B32=Eigen::Matrix<double,3,2>;
constexpr double pi=3.14159265358979323846;
inline M3 cross(const V3& v) {
  M3 m;m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m;
}
inline M3 exp(const V3& v) {
  const double a=v.norm(),a2=a*a;const M3 k=cross(v);
  const double b=a<1e-7 ? 1-a2/6+a2*a2/120 : std::sin(a)/a;
  const double c=a<1e-7 ? .5-a2/24+a2*a2/720 : (1-std::cos(a))/a2;
  return M3::Identity()+b*k+c*k*k;
}
inline M3 leftJacobian(const V3& v) {
  const double a=v.norm(),a2=a*a;const M3 k=cross(v);
  const double b=a<1e-7 ? .5-a2/24 : (1-std::cos(a))/a2;
  const double c=a<1e-7 ? 1./6-a2/120 : (a-std::sin(a))/(a2*a);
  return M3::Identity()+b*k+c*k*k;
}
inline bool rotation(const M3& r) {
  M3 tolerance=M3::Constant(1e-5);tolerance.diagonal().setConstant(2e-5);
  return r.allFinite() && ((r.transpose()*r-M3::Identity()).cwiseAbs().array()<=tolerance.array()).all() &&
    std::abs(r.determinant()-1)<=1e-5;
}
inline B32 normalBasis(const V3& n) {
  if(!n.allFinite() || std::abs(n.squaredNorm()-1)>1e-8)
    throw std::invalid_argument("unit_surface_normal_required");
  Eigen::Index i;n.cwiseAbs().minCoeff(&i);const V3 u=n.cross(V3::Unit(i)).normalized();
  B32 b;b<<u,n.cross(u);return b;
}
struct Key {
  int64_t stamp=0;int sensor=-1,beam=-1;
  bool operator<(const Key& other) const {
    return std::tie(stamp,sensor,beam)<std::tie(other.stamp,other.sensor,other.beam);
  }
};
struct Pose {V3 position=V3::Zero();M3 attitude=M3::Identity(),coordinates=M3::Identity();};
struct Plane {
  V3 normal=V3::UnitZ();double distance=0.;B32 coordinates=normalBasis(normal);
  std::vector<Key> support_keys;
  double max_plane_deviation=0.,max_support_distance=0.;
};
enum class Model {Cone,Fan};
enum class Role {Factor,SupportOnly};
struct Observation {
  Key key;Model model=Model::Cone;Role role=Role::Factor;
  int plane_id=-1;bool surface_hypothesis=false;
  double measured=0.,minimum=0.,maximum=0.;
  V3 lever=V3::Zero(),axis=-V3::UnitZ();M3 mount=M3::Identity();
  double aperture=0.,azimuth=0.,vertical_fov=0.;
};
struct Prediction {
  double range=0.;Eigen::Matrix<double,1,6> pose_jacobian;
  Eigen::Matrix<double,1,4> plane_jacobian;
  V3 hit,footprint_center;B32 footprint_basis;Eigen::Matrix2d footprint_shape;
};
inline Prediction predict(const Pose& pose,const Plane& plane,const Observation& o) {
  const V3 origin=pose.position+pose.attitude*o.lever;
  const V3 n0=plane.normal;const double h0=plane.distance-n0.dot(origin);
  if(std::abs(h0)<1e-9)throw std::invalid_argument("surface_origin_on_plane");
  const double side=h0>0 ? 1. : -1.;Prediction out;
  if(o.model==Model::Cone) {
    if(!o.axis.allFinite() || std::abs(o.axis.squaredNorm()-1)>1e-8 ||
       !std::isfinite(o.aperture) || o.aperture<0 || o.aperture>=pi)
      throw std::invalid_argument("invalid_surface_cone_geometry");
    const V3 n=side*n0,body=o.mount*o.axis,axis=pose.attitude*body;
    const double h=side*h0,sa=std::sin(o.aperture/2),ca=std::cos(o.aperture/2);
    const double cosine=std::clamp(n.dot(axis),-1.,1.);
    if(cosine<=sa+1e-10)throw std::invalid_argument("unbounded_dvl_plane_footprint");
    double maximum=1.,derivative=0.;V3 direction=n;
    if(cosine<ca) {
      const double sine=std::sqrt(std::max(0.,1-cosine*cosine));
      maximum=cosine*ca+sine*sa;derivative=ca-cosine/sine*sa;
      direction=ca*axis+sa*(n-cosine*axis)/sine;
    }
    out.range=h/maximum;
    const Eigen::RowVector3d jo=-n.transpose()/maximum;
    const V3 gradient=-h*derivative*n/(maximum*maximum);
    out.pose_jacobian.head<3>()=jo;
    out.pose_jacobian.tail<3>()=(-jo*pose.attitude*cross(o.lever)-
      gradient.transpose()*pose.attitude*cross(body))*pose.attitude.transpose();
    const V3 ambient=-origin/maximum-h*derivative*axis/(maximum*maximum);
    out.plane_jacobian.head<3>()=-ambient.transpose()*cross(n);
    out.plane_jacobian[3]=side/maximum;out.hit=origin+out.range*direction;
    out.footprint_basis=normalBasis(n);const Eigen::Vector2d b=out.footprint_basis.transpose()*axis;
    const double denominator=cosine*cosine-sa*sa;
    const Eigen::Matrix2d q=ca*ca*Eigen::Matrix2d::Identity()-b*b.transpose();
    out.footprint_center=origin+h*n+out.footprint_basis*(h*cosine*b/denominator);
    out.footprint_shape=(h*h*ca*ca*sa*sa/denominator)*q.inverse();
  } else if(o.model==Model::Fan) {
    if(!std::isfinite(o.azimuth) || std::abs(o.azimuth)>=pi/2 ||
       !std::isfinite(o.vertical_fov) || o.vertical_fov<=0 || o.vertical_fov>=pi)
      throw std::invalid_argument("invalid_surface_fan_geometry");
    const V3 local=side*(pose.attitude*o.mount).transpose()*n0;
    const double a=local.x()*std::cos(o.azimuth)+local.y()*std::sin(o.azimuth),b=local.z();
    const double half=o.vertical_fov/2,stationary=std::atan2(b,a);
    std::vector<double> candidates{-half,half};
    if(stationary>-half && stationary<half)candidates.push_back(stationary);
    double maximum=-1e300,elevation=0.;
    for(double e:candidates) {const double v=a*std::cos(e)+b*std::sin(e);if(v>maximum){maximum=v;elevation=e;}}
    if(maximum<=1e-10)throw std::invalid_argument("surface_fan_has_no_forward_return");
    const V3 direction(std::cos(o.azimuth)*std::cos(elevation),std::sin(o.azimuth)*std::cos(elevation),std::sin(elevation));
    const V3 body=o.mount*direction,world=pose.attitude*body;
    const double denominator=n0.dot(world);out.range=h0/denominator;
    const Eigen::RowVector3d jo=-n0.transpose()/denominator,jd=out.range*jo;
    out.pose_jacobian.head<3>()=jo;
    out.pose_jacobian.tail<3>()=(-jo*pose.attitude*cross(o.lever)-jd*pose.attitude*cross(body))*pose.attitude.transpose();
    out.hit=origin+out.range*world;
    const V3 ambient=-out.hit/denominator;
    out.plane_jacobian.head<3>()=-ambient.transpose()*cross(n0);out.plane_jacobian[3]=1/denominator;
  } else throw std::invalid_argument("unknown_surface_range_model");
  if(!std::isfinite(out.range) || out.range<o.minimum || out.range>o.maximum)
    throw std::invalid_argument("surface_first_return_out_of_range");
  return out;
}
using Point=Eigen::Vector2d;
inline double cross2(const Point& a,const Point& b,const Point& c) {
  const Point u=b-a,v=c-a;return u.x()*v.y()-u.y()*v.x();
}
inline std::vector<Point> hull(const Eigen::MatrixXd& samples) {
  std::set<std::pair<double,double>> sorted;
  for(Eigen::Index i=0;i<samples.rows();++i)sorted.emplace(samples(i,0),samples(i,1));
  std::vector<Point> order;for(const auto& p:sorted)order.emplace_back(p.first,p.second);
  auto chain=[](const std::vector<Point>& sequence) {
    std::vector<Point> result;
    for(const auto& p:sequence) {
      while(result.size()>1 && cross2(result[result.size()-2],result.back(),p)<=0)result.pop_back();
      result.push_back(p);
    }
    if(!result.empty())result.pop_back();return result;
  };
  auto result=chain(order);std::reverse(order.begin(),order.end());const auto upper=chain(order);
  result.insert(result.end(),upper.begin(),upper.end());return result;
}
inline void requireSupport(const Plane& plane,const Observation& o,const Prediction& f,const Eigen::MatrixXd& points) {
  if(points.cols()!=3 || points.rows()<3 || points.rows()>4096 || !points.allFinite() ||
     !std::isfinite(plane.max_plane_deviation) || plane.max_plane_deviation<=0 ||
     !std::isfinite(plane.max_support_distance) || plane.max_support_distance<=0)
    throw std::invalid_argument("invalid_surface_support");
  if((points*plane.normal-Eigen::VectorXd::Constant(points.rows(),plane.distance)).cwiseAbs().maxCoeff()>plane.max_plane_deviation)
    throw std::invalid_argument("surface_support_not_planar");
  const double scale=plane.max_support_distance;
  if(o.model==Model::Cone) {
    const Eigen::MatrixXd samples=(points.rowwise()-f.footprint_center.transpose())*f.footprint_basis/scale;
    const Eigen::Matrix2d shape=f.footprint_shape/(scale*scale);const auto boundary=hull(samples);
    if(boundary.size()<3)throw std::invalid_argument("degenerate_surface_support");
    for(size_t k=0;k<boundary.size();++k) {
      const Point first=boundary[k],edge=boundary[(k+1)%boundary.size()]-first;
      const Point outward(edge.y(),-edge.x());
      const double radius=std::sqrt(std::max(0.,(outward.transpose()*shape*outward).value()));
      if(radius>outward.dot(first)+1e-10*outward.norm())
        throw std::invalid_argument("dvl_cone_outside_surface_support");
    }
    const double radius=std::sqrt(std::max(0.,Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d>(shape).eigenvalues().maxCoeff()));
    if(samples.rowwise().norm().minCoeff()+radius>1+1e-10)
      throw std::invalid_argument("dvl_surface_support_gap");
  } else {
    const B32 basis=normalBasis(plane.normal);const V3 center=points.colwise().mean().transpose();
    const Eigen::MatrixXd samples=(points.rowwise()-center.transpose())*basis/scale;
    const Point query=basis.transpose()*(f.hit-center)/scale;const auto boundary=hull(samples);
    if(boundary.size()<3)throw std::invalid_argument("degenerate_surface_support");
    for(size_t k=0;k<boundary.size();++k)
      if(cross2(boundary[k],boundary[(k+1)%boundary.size()],query)<-1e-10)
        throw std::invalid_argument("surface_does_not_cover_ping");
    if((samples.rowwise()-query.transpose()).rowwise().squaredNorm().minCoeff()>1+1e-10)
      throw std::invalid_argument("surface_support_gap");
  }
}
struct Block {
  Eigen::MatrixXd jacobian,raw_jacobian,measurement_covariance;
  Eigen::VectorXd residual,raw_residual,predicted_ranges;
  Eigen::MatrixXd conditional_hits;
  std::vector<int64_t> state_stamps;std::vector<int> plane_ids;
  std::vector<Key> source_keys;std::vector<Eigen::Index> factor_indices,support_only_indices;
  std::map<int,Eigen::MatrixXd> support_seeds;
};
inline Block linearize(const std::map<int64_t,Pose>& poses,const std::map<int,Plane>& planes,
    const std::vector<Observation>& observations,const Eigen::MatrixXd& covariance,
    const std::string& uncertainty_source,bool require_support=true) {
  const Eigen::Index count=observations.size();
  if(poses.empty() || poses.size()>16 || planes.empty() || planes.size()>8 || count<1 || count>512 ||
     uncertainty_source.find_first_not_of(" \t\r\n")==std::string::npos)
    throw std::invalid_argument("bounded_surface_block_and_explicit_noise_required");
  Block out;std::map<int64_t,Eigen::Index> pose_columns;std::map<int,Eigen::Index> plane_columns;
  Eigen::Index column=0;
  for(const auto& [stamp,pose]:poses) {
    if(stamp<=0 || !pose.position.allFinite() || !rotation(pose.attitude) || !pose.coordinates.allFinite())
      throw std::invalid_argument("invalid_surface_pose");
    out.state_stamps.push_back(stamp);pose_columns[stamp]=column;column+=6;
  }
  for(const auto& [id,plane]:planes) {
    normalBasis(plane.normal);
    if(id<0 || !std::isfinite(plane.distance) || !plane.coordinates.allFinite())
      throw std::invalid_argument("invalid_surface_plane");
    out.plane_ids.push_back(id);plane_columns[id]=column;column+=3;
  }
  if(covariance.rows()!=count || covariance.cols()!=count || !covariance.allFinite() ||
     (covariance-covariance.transpose()).cwiseAbs().maxCoeff()>1e-10)
    throw std::invalid_argument("invalid_surface_range_covariance");
  Eigen::LLT<Eigen::MatrixXd> full_noise(covariance);
  if(full_noise.info()!=Eigen::Success)throw std::invalid_argument("positive_surface_range_covariance_required");
  std::map<Key,Eigen::Index> lookup;
  for(Eigen::Index i=0;i<count;++i) {
    const auto& o=observations[i];
    if(!poses.count(o.key.stamp) || o.key.sensor<0 || o.key.beam<0 || !lookup.emplace(o.key,i).second)
      throw std::invalid_argument("missing_pose_duplicate_or_unresolved_surface_row");
    if(!std::isfinite(o.minimum) || !std::isfinite(o.maximum) || !std::isfinite(o.measured) ||
       !(0<o.minimum && o.minimum<o.maximum) || o.measured<o.minimum || o.measured>o.maximum ||
       !o.lever.allFinite() || !rotation(o.mount))throw std::invalid_argument("invalid_surface_range_measurement");
    if(o.role==Role::SupportOnly) {
      if(o.model!=Model::Cone || !o.axis.allFinite() || std::abs(o.axis.squaredNorm()-1)>1e-8 ||
         !std::isfinite(o.aperture) || o.aperture<0 || o.aperture>=pi)
        throw std::invalid_argument("invalid_raw_surface_support_cone");
      out.support_only_indices.push_back(i);
    } else if(o.role==Role::Factor && o.surface_hypothesis && planes.count(o.plane_id))out.factor_indices.push_back(i);
    else throw std::invalid_argument("missing_pose_duplicate_or_unresolved_surface_row");
    out.source_keys.push_back(o.key);
  }
  const Eigen::Index rows=out.factor_indices.size();
  if(!rows)throw std::invalid_argument("no_surface_factor_rows");
  out.raw_jacobian=Eigen::MatrixXd::Zero(rows,column);out.raw_residual.resize(rows);
  out.predicted_ranges.resize(rows);out.conditional_hits.resize(rows,3);
  std::vector<Prediction> predictions;
  for(Eigen::Index row=0;row<rows;++row) {
    const auto& o=observations[out.factor_indices[row]];const auto& pose=poses.at(o.key.stamp);const auto& plane=planes.at(o.plane_id);
    const auto f=predict(pose,plane,o);predictions.push_back(f);
    const Eigen::Index pc=pose_columns.at(o.key.stamp),nc=plane_columns.at(o.plane_id);
    out.raw_residual[row]=f.range-o.measured;out.predicted_ranges[row]=f.range;out.conditional_hits.row(row)=f.hit.transpose();
    out.raw_jacobian.block<1,3>(row,pc)=f.pose_jacobian.head<3>();
    out.raw_jacobian.block<1,3>(row,pc+3)=f.pose_jacobian.tail<3>()*pose.coordinates;
    out.raw_jacobian.block<1,2>(row,nc)=f.plane_jacobian.head<3>()*plane.coordinates;
    out.raw_jacobian(row,nc+2)=f.plane_jacobian[3];
  }
  if(require_support) {
    for(const auto& [id,plane]:planes) {
      if(plane.support_keys.size()<3 || plane.support_keys.size()>4096 ||
         std::set<Key>(plane.support_keys.begin(),plane.support_keys.end()).size()!=plane.support_keys.size())
        throw std::invalid_argument("explicit_distinct_raw_surface_support_required");
      Eigen::MatrixXd points(plane.support_keys.size(),3);Eigen::Index row=0;
      for(const auto& key:plane.support_keys) {
        if(!lookup.count(key))throw std::invalid_argument("surface_support_pose_or_measurement_retired");
        const auto& o=observations[lookup.at(key)];const auto& p=poses.at(key.stamp);
        if(o.model!=Model::Cone)throw std::invalid_argument("unmeasured_fan_elevation_is_not_surface_support");
        points.row(row++)=(p.position+p.attitude*(o.lever+o.measured*o.mount*o.axis)).transpose();
      }
      out.support_seeds[id]=points;
    }
    for(Eigen::Index row=0;row<rows;++row) {
      const auto& o=observations[out.factor_indices[row]];
      requireSupport(planes.at(o.plane_id),o,predictions[row],out.support_seeds.at(o.plane_id));
    }
  }
  out.measurement_covariance.resize(rows,rows);
  for(Eigen::Index i=0;i<rows;++i)for(Eigen::Index j=0;j<rows;++j)
    out.measurement_covariance(i,j)=covariance(out.factor_indices[i],out.factor_indices[j]);
  Eigen::LLT<Eigen::MatrixXd> noise(out.measurement_covariance);
  if(noise.info()!=Eigen::Success)throw std::invalid_argument("positive_surface_range_covariance_required");
  out.jacobian=noise.matrixL().solve(out.raw_jacobian);out.residual=noise.matrixL().solve(out.raw_residual);
  if(!out.jacobian.allFinite() || !out.residual.allFinite())throw std::invalid_argument("nonfinite_surface_range_linearization");
  return out;
}
}  // namespace posim_slam::surface_ranges

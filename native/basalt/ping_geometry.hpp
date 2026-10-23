// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <basalt/camera/stereographic_param.hpp>
#include <cmath>
#include <stdexcept>

namespace posim_slam {
using Vec3=Eigen::Vector3d;
using Mat3=Eigen::Matrix3d;
using Mat36=Eigen::Matrix<double,3,6>;

inline Mat3 crossMatrix(const Vec3& v) {return Sophus::SO3d::hat(v);}

struct PingPlaneReference {Vec3 normal;double distance;};

struct WorldLandmark {
  Vec3 point;
  Mat36 host_jacobian;
  Mat3 landmark_jacobian;
};

inline WorldLandmark worldLandmark(const Sophus::SE3d& host,const Sophus::SE3d& camera,
                                  const Eigen::Vector2d& direction,double inverse_distance) {
  if (!host.matrix().allFinite() || !camera.matrix().allFinite() || !direction.allFinite() ||
      !std::isfinite(inverse_distance) || inverse_distance<=0)
    throw std::invalid_argument("finite_ping_landmark_required");
  Eigen::Matrix<double,4,2> unproject_j;
  const Vec3 unit=basalt::StereographicParam<double>::unproject(direction,&unproject_j).head<3>();
  const Vec3 local=camera*(unit/inverse_distance);
  const Mat3 r=host.rotationMatrix(),rc=camera.rotationMatrix();
  WorldLandmark out;out.point=host*local;
  out.host_jacobian.leftCols<3>().setIdentity();
  // Basalt adds map translation and applies map-frame LEFT rotation.
  out.host_jacobian.rightCols<3>()=-crossMatrix(r*local);
  out.landmark_jacobian.leftCols<2>()=r*rc*unproject_j.topRows<3>()/inverse_distance;
  out.landmark_jacobian.col(2)=-r*rc*unit/(inverse_distance*inverse_distance);
  return out;
}

inline Eigen::Matrix<double,3,2> planeBasis(const Vec3& normal) {
  if (!normal.allFinite() || std::abs(normal.squaredNorm()-1.)>1e-8)
    throw std::invalid_argument("unit_plane_normal_required");
  Eigen::Index index;normal.cwiseAbs().minCoeff(&index);
  const Vec3 u=normal.cross(Vec3::Unit(index)).normalized();
  Eigen::Matrix<double,3,2> basis;basis<<u,normal.cross(u);return basis;
}

struct NativePingRange {
  double prediction;
  Eigen::Matrix<double,1,6> pose_jacobian;
  Eigen::RowVector3d plane_jacobian;
  Eigen::MatrixXd intersections;
  Eigen::Index active_ray;
};

inline NativePingRange pingRange(const Sophus::SE3d& pose,const Sophus::SE3d& mount,
    const Eigen::MatrixXd& rays,const Vec3& normal,double distance,double bias) {
  const auto basis=planeBasis(normal);
  if (!pose.matrix().allFinite() || !mount.matrix().allFinite() ||
      rays.cols()!=3 || rays.rows()<1 || rays.rows()>256 || !rays.allFinite() ||
      !std::isfinite(distance) || !std::isfinite(bias))
    throw std::invalid_argument("invalid_ping_geometry");
  const Vec3 lever=pose.rotationMatrix()*mount.translation();
  const Vec3 origin=pose.translation()+lever;
  const double a=distance-normal.dot(origin);
  if(std::abs(a)<1e-9)throw std::invalid_argument("unsupported_ping_plane");
  Eigen::VectorXd ranges(rays.rows());Eigen::MatrixXd world(rays.rows(),3);
  NativePingRange out;out.intersections.resize(rays.rows(),3);
  for(Eigen::Index i=0;i<rays.rows();++i) {
    const Vec3 ray=rays.row(i).transpose();
    if(std::abs(ray.squaredNorm()-1.)>1e-8)throw std::invalid_argument("unit_ping_ray_required");
    const Vec3 v=pose.rotationMatrix()*mount.rotationMatrix()*ray;world.row(i)=v.transpose();
    const double b=normal.dot(v);
    if((a>0?b:-b)<.1)throw std::invalid_argument("unsupported_ping_plane");
    ranges[i]=a/b;out.intersections.row(i)=(origin+ranges[i]*v).transpose();
  }
  const double t=ranges.minCoeff(&out.active_ray);
  if(((ranges.array()-t).abs()<=1e-9).count()!=1)
    throw std::invalid_argument("ping_beam_switch_requires_reassociation");
  const Vec3 v=world.row(out.active_ray).transpose();const double b=normal.dot(v);
  out.prediction=t+bias;out.pose_jacobian.leftCols<3>()=-normal.transpose()/b;
  out.pose_jacobian.rightCols<3>()=(normal.transpose()*crossMatrix(lever)+t*normal.transpose()*crossMatrix(v))/b;
  out.plane_jacobian.head<2>()=(origin+t*v).transpose()*crossMatrix(normal)*basis/b;
  out.plane_jacobian[2]=1./b;
  return out;
}

struct SurfaceRow {
  double residual;
  Eigen::Matrix<double,1,6> host_jacobian;
  Eigen::RowVector3d landmark_jacobian,plane_jacobian;
};

inline SurfaceRow surfaceRow(const WorldLandmark& landmark,const Vec3& normal,double distance) {
  const auto basis=planeBasis(normal);
  if(!std::isfinite(distance))throw std::invalid_argument("finite_plane_distance_required");
  SurfaceRow out;out.residual=normal.dot(landmark.point)-distance;
  out.host_jacobian=normal.transpose()*landmark.host_jacobian;
  out.landmark_jacobian=normal.transpose()*landmark.landmark_jacobian;
  out.plane_jacobian.head<2>()=-landmark.point.transpose()*crossMatrix(normal)*basis;
  out.plane_jacobian[2]=-1.;return out;
}
}  // namespace posim_slam

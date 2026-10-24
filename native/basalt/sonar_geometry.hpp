// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace posim_slam {
class SonarPointOutsideSupport : public std::invalid_argument {
 public: using std::invalid_argument::invalid_argument;
};
struct SonarPointRows {
  Eigen::Vector2d residual;
  Eigen::Matrix<double,2,6> pose_jacobian;
  Eigen::Matrix<double,2,3> point_jacobian;
};

// Predicted-minus-measured residual. Basalt uses additive map translation and
// LEFT map-frame rotation, unlike the right-IMU rotation in the Python reference.
// Sonar axes: X forward, Y left, Z up. Elevation is constrained only by FOV.
inline SonarPointRows sonarPointRows(const Eigen::Vector3d& position,
    const Eigen::Matrix3d& attitude,const Eigen::Vector3d& lever,
    const Eigen::Matrix3d& sonar_to_imu,const Eigen::Vector3d& point,
    double measured_range,double measured_azimuth,double minimum,double maximum,
    double horizontal_fov,double vertical_fov) {
  auto rotation=[](const Eigen::Matrix3d& r) {
    return r.allFinite() && (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-8 && std::abs(r.determinant()-1)<1e-8;
  };
  constexpr double pi=3.14159265358979323846;
  if(!position.allFinite() || !lever.allFinite() || !point.allFinite() || !rotation(attitude) || !rotation(sonar_to_imu) ||
      !std::isfinite(measured_range) || !std::isfinite(measured_azimuth) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
      !std::isfinite(horizontal_fov) || !std::isfinite(vertical_fov) || !(0<minimum && minimum<maximum) ||
      measured_range<minimum || measured_range>maximum || !(0<horizontal_fov && horizontal_fov<=pi) ||
      !(0<vertical_fov && vertical_fov<pi) || std::abs(measured_azimuth)>horizontal_fov/2)
    throw std::invalid_argument("invalid_native_sonar_geometry");
  const Eigen::Vector3d difference=point-position;
  const Eigen::Vector3d local=sonar_to_imu.transpose()*(attitude.transpose()*difference-lever);
  const double x=local.x(),y=local.y(),z=local.z(),rho2=x*x+y*y,range=local.norm();
  if(x<=0 || rho2<1e-12 || range<minimum || range>maximum)
    throw SonarPointOutsideSupport("unsupported_native_sonar_point");
  const double azimuth=std::atan2(y,x),elevation=std::atan2(z,std::sqrt(rho2));
  if(std::abs(azimuth)>horizontal_fov/2 || std::abs(elevation)>vertical_fov/2)
    throw SonarPointOutsideSupport("native_sonar_point_outside_fov");
  Eigen::Matrix<double,2,3> projection;
  projection.row(0)=local.transpose()/range;projection.row(1)<<-y/rho2,x/rho2,0.;
  Eigen::Matrix3d cross;
  cross<<0,-difference.z(),difference.y(),difference.z(),0,-difference.x(),-difference.y(),difference.x(),0;
  SonarPointRows out;
  out.residual<<range-measured_range,std::atan2(std::sin(azimuth-measured_azimuth),std::cos(azimuth-measured_azimuth));
  out.point_jacobian=projection*sonar_to_imu.transpose()*attitude.transpose();
  out.pose_jacobian.leftCols<3>()=-out.point_jacobian;
  out.pose_jacobian.rightCols<3>()=out.point_jacobian*cross;
  return out;
}
}

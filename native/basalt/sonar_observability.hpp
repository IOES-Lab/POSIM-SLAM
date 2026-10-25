// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace posim_slam {
// Local, fixed-pose linearized uncertainty from measurement rows only. No LM
// damping, FOV prior, or pose prior may be included in the supplied Jacobian.
// This is a necessary admission check, not joint map/pose uncertainty.
inline double sonarConditionalElevationSigma(const Eigen::MatrixXd& jacobian,
    const Eigen::Vector3d& local,const Eigen::Matrix3d& world_to_sonar) {
  if(jacobian.rows()<3 || jacobian.cols()!=3 || !jacobian.allFinite() ||
      !local.allFinite() || !world_to_sonar.allFinite() ||
      (world_to_sonar*world_to_sonar.transpose()-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
      std::abs(world_to_sonar.determinant()-1.)>1e-8)
    throw std::invalid_argument("invalid_sonar_elevation_information");
  const double rho=local.head<2>().norm(),r2=local.squaredNorm();
  if(rho<1e-6 || r2<1e-12)throw std::invalid_argument("undefined_sonar_elevation_gradient");
  Eigen::Vector3d gradient(-local.x()*local.z()/rho/r2,
                          -local.y()*local.z()/rho/r2,rho/r2);
  gradient=world_to_sonar.transpose()*gradient;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian,Eigen::ComputeFullV);
  const auto singular=svd.singularValues();
  if(singular[2]<=1e-10*singular[0])return std::numeric_limits<double>::infinity();
  // g^T (J^T J)^-1 g, evaluated without normal equations or regularization.
  return ((svd.matrixV().transpose()*gradient).array()/singular.array()).matrix().norm();
}
}

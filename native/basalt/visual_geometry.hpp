// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/SVD>
#include <cmath>
#include <limits>

namespace posim_slam {
// Conditional camera-row information only: poses are fixed. This is not the
// joint VIO posterior or a map/navigation accuracy certificate.
struct VisualPointSupport {
  bool eligible=false;
  size_t exposures=0;
  double relative_range_sigma=std::numeric_limits<double>::infinity();
  bool joint_position_observed=false;
  bool joint_information_evaluated=false;
  double joint_position_sigma_m=std::numeric_limits<double>::infinity();
  bool joint_host_position_observed=false;
  double joint_host_position_sigma_m=std::numeric_limits<double>::infinity();
  double conditional_host_position_sigma_m=std::numeric_limits<double>::infinity();
};
inline VisualPointSupport visualPointSupport(const Eigen::MatrixXd& whitened,
    const Eigen::Vector3d& host_to_point,size_t exposures) {
  VisualPointSupport result;result.exposures=exposures;
  const double range=host_to_point.norm();
  if(exposures<2 || whitened.rows()<4 || whitened.cols()!=3 || !whitened.allFinite()
      || !host_to_point.allFinite() || !std::isfinite(range) || range<=0)return result;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(whitened,Eigen::ComputeThinV);
  const auto singular=svd.singularValues();
  if(singular.size()!=3 || singular[0]<=0 || singular[2]<=1e-10*singular[0])return result;
  const Eigen::Vector3d direction=host_to_point/range;
  // grad(1/r)=-direction/r^2. sigma_r/r == sigma_(1/r)/(1/r).
  // SVD avoids squaring the small-parallax condition number in J^T J.
  const Eigen::Vector3d sensitivity=svd.matrixV().transpose()*direction;
  result.relative_range_sigma=sensitivity.cwiseQuotient(singular).norm()/range;
  result.eligible=std::isfinite(result.relative_range_sigma) && 3*result.relative_range_sigma<1;
  return result;
}
}  // namespace posim_slam

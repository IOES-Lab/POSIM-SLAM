// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace posim_slam {
struct GroundSurfaceReference {Eigen::MatrixXd points;Eigen::VectorXd incidence;};
inline Eigen::MatrixXd surfaceRangeCovariance(const Eigen::MatrixXd& points,
    const Eigen::VectorXd& incidence,double sigma,double length) {
  const auto count=points.rows();
  if(points.cols()!=3 || count<8 || count>512 || incidence.size()!=count ||
     !points.allFinite() || !incidence.allFinite() ||
     (incidence.array().abs()<1e-4).any() || (incidence.array().abs()>1.+1e-8).any() ||
     !std::isfinite(sigma) || sigma<0 || sigma>.2 ||
     !std::isfinite(length) || length<.05 || length>5.)
    throw std::invalid_argument("invalid_ground_surface_covariance");
  Eigen::MatrixXd result(count,count);
  for(Eigen::Index i=0;i<count;++i)for(Eigen::Index j=0;j<=i;++j) {
    const double value=sigma*sigma/(incidence[i]*incidence[j])*
      std::exp(-(points.row(i)-points.row(j)).squaredNorm()/(2*length*length));
    result(i,j)=result(j,i)=value;
  }
  return result;
}
}  // namespace posim_slam

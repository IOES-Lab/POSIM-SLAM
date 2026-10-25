// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/QR>
#include <cmath>
#include <stdexcept>

namespace posim_slam {
struct GroundPredictiveCheck {double score,limit;int new_rows;};

// The prefix and complete system use the same lower-Cholesky whitening.
// Profiling the free plane in each nested system is equivalent to a Gaussian
// posterior-predictive innovation, including plane-fit uncertainty and shared
// measurement noise. Poses and the linearization are conditional here.
inline GroundPredictiveCheck groundPredictiveCheck(const Eigen::MatrixXd& plane,
    const Eigen::VectorXd& residual,Eigen::Index old_rows,double sigma) {
  if(plane.cols()!=3 || plane.rows()!=residual.size() || old_rows<3 ||
     old_rows>=plane.rows() || plane.rows()>512 || !plane.allFinite() ||
     !residual.allFinite() || !std::isfinite(sigma) || sigma<=0)
    throw std::invalid_argument("invalid_ground_predictive_system");
  auto cost=[&](Eigen::Index rows) {
    const Eigen::MatrixXd j=plane.topRows(rows);
    const Eigen::VectorXd r=residual.head(rows);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(j);qr.setThreshold(1e-10);
    if(qr.rank()!=3)throw std::invalid_argument("ground_predictive_plane_rank");
    return (r-j*qr.solve(r)).squaredNorm();
  };
  const double previous=cost(old_rows),joined=cost(plane.rows()),score=joined-previous;
  if(!std::isfinite(score) || score < -1e-8*(1+previous+joined))
    throw std::invalid_argument("ground_predictive_negative_innovation");
  const int count=int(plane.rows()-old_rows);
  // Laurent-Massart upper tail, x=sigma^2/2:
  // P(chi_square(count) > count + sigma*sqrt(2*count) + sigma^2)
  // <= exp(-sigma^2/2), under the conditional Gaussian model.
  return {std::max(0.,score),count+sigma*std::sqrt(2.*count)+sigma*sigma,count};
}
}  // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <stdexcept>

namespace posim_slam {
// H = L L^T and C = J H^-1 J^T = (L^-1 J^T)^T (L^-1 J^T).
// Form only one triangle of each Gram matrix. No averaging, eigenvalue floor,
// additional prior or damping changes the represented marginal likelihood.
// The caller retains its original rank decision before constructing this owner.
class NfrCovarianceProjection {
 public:
  explicit NfrCovarianceProjection(const Eigen::MatrixXd& information) {
    if (!information.rows() || information.rows() != information.cols() ||
        !information.allFinite() ||
        !information.isApprox(information.transpose(), 1e-12))
      throw std::invalid_argument("nfr_projection_information");
    root_.compute(information);
    if (root_.info() != Eigen::Success)
      throw std::invalid_argument("nfr_projection_information_not_positive_definite");
  }

  Eigen::MatrixXd information(const Eigen::MatrixXd& jacobian) const {
    if (!jacobian.rows() || jacobian.cols() != root_.rows() || !jacobian.allFinite())
      throw std::invalid_argument("nfr_projection_jacobian");
    const Eigen::MatrixXd projected = root_.matrixL().solve(jacobian.transpose());
    const Eigen::MatrixXd covariance = gram(projected);
    const Eigen::LLT<Eigen::MatrixXd> local(covariance);
    if (local.info() != Eigen::Success)
      throw std::invalid_argument("nfr_projection_covariance_not_positive_definite");
    const Eigen::MatrixXd inverse_root = local.matrixL().solve(
        Eigen::MatrixXd::Identity(covariance.rows(), covariance.cols()));
    const Eigen::MatrixXd result = gram(inverse_root);
    if (!result.allFinite()) throw std::invalid_argument("nfr_projection_nonfinite");
    return result;
  }

 private:
  static Eigen::MatrixXd gram(const Eigen::MatrixXd& rows) {
    Eigen::MatrixXd triangle = Eigen::MatrixXd::Zero(rows.cols(), rows.cols());
    triangle.selfadjointView<Eigen::Lower>().rankUpdate(rows.transpose());
    return triangle.selfadjointView<Eigen::Lower>();
  }
  Eigen::LLT<Eigen::MatrixXd> root_;
};
}  // namespace posim_slam

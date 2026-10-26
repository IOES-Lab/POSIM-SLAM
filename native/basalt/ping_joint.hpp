// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "estimator_budget.hpp"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace posim_slam {

// The group must include all raw visual/Ping rows sharing its landmarks.
// Passing already independently marginalized visual rows loses correlations.
struct JointPatchReduction {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  Eigen::MatrixXd backsub_matrix;
  Eigen::VectorXd backsub_offset;
  Eigen::Index nuisance_rank;
  Eigen::Index nuisance_nullity;

  Eigen::VectorXd backSubstitute(const Eigen::VectorXd& retained_delta) const {
    if (retained_delta.size()!=backsub_matrix.cols() || !retained_delta.allFinite())
      throw std::invalid_argument("invalid_retained_increment");
    return backsub_offset+backsub_matrix*retained_delta;
  }
};

inline JointPatchReduction eliminateJointPatch(
    const Eigen::MatrixXd& j,const Eigen::VectorXd& residual,Eigen::Index retained,double damping=0.) {
  if (j.rows()<1 || j.rows()>4096 || j.cols()<2 || j.cols()>kMaxJointColumns ||
      residual.size()!=j.rows() || !j.allFinite() || !residual.allFinite() ||
      retained<1 || retained>j.cols() || !std::isfinite(damping) || damping<0 || damping>1e12)
    throw std::invalid_argument("invalid_joint_patch_system");
  // No nuisance variables remain after an earlier joint reduction.
  if(retained==j.cols())
    return {j,residual,Eigen::MatrixXd(0,retained),Eigen::VectorXd(0),0,0};
  if(damping>0) {
    // LM regularizes the complete trial, including eliminated plane/landmark
    // increments. These rows are never observations or marginal-prior input.
    const auto count=j.cols()-retained;
    Eigen::MatrixXd augmented=Eigen::MatrixXd::Zero(j.rows()+count,j.cols());
    augmented.topRows(j.rows())=j;
    augmented.bottomRightCorner(count,count).diagonal()=
        (j.rightCols(count).colwise().squaredNorm().transpose().array()*damping).max(1e-9).sqrt();
    Eigen::VectorXd values=Eigen::VectorXd::Zero(residual.size()+count);values.head(residual.size())=residual;
    // Damping supplies an independent positive row for every nuisance column.
    // Its smallest diagonal bounds sigma_min from below; the Frobenius norm
    // bounds sigma_max from above. Use QR only when these bounds certify the
    // same full rank that the SVD cutoff below would select (with margin).
    const Eigen::MatrixXd nuisance=augmented.rightCols(count);
    const double lower=augmented.bottomRightCorner(count,count).diagonal().minCoeff();
    const double cutoff=std::max(nuisance.rows(),nuisance.cols())*
        std::numeric_limits<double>::epsilon()*nuisance.norm();
    if(augmented.rows()<=4096 && std::isfinite(cutoff) && lower>16.*cutoff) {
      Eigen::HouseholderQR<Eigen::MatrixXd> qr(nuisance);
      Eigen::MatrixXd combined(augmented.rows(),retained+1);
      combined.leftCols(retained)=augmented.leftCols(retained);combined.col(retained)=values;
      combined=(qr.householderQ().adjoint()*combined).eval();
      const Eigen::MatrixXd backsub=-qr.matrixQR().topRows(count)
          .triangularView<Eigen::Upper>().solve(combined.topRows(count));
      const auto remaining=augmented.rows()-count;
      return {combined.bottomRows(remaining).leftCols(retained),combined.col(retained).tail(remaining),
              backsub.leftCols(retained),backsub.col(retained),count,0};
    }
    return eliminateJointPatch(augmented,values,retained);
  }
  const Eigen::MatrixXd nuisance=j.rightCols(j.cols()-retained);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(nuisance,Eigen::ComputeThinU|Eigen::ComputeThinV);
  if (svd.info()!=Eigen::Success)throw std::runtime_error("joint_patch_svd_failed");
  const auto& singular=svd.singularValues();
  const double tolerance=std::max(nuisance.rows(),nuisance.cols())*
      std::numeric_limits<double>::epsilon()*singular[0];
  const Eigen::Index rank=(singular.array()>tolerance).count();
  // Only the observed nuisance span is needed. Forming full U (rows x rows)
  // and its explicit nullspace makes tall groups unnecessarily quadratic in
  // observation count. Apply the orthogonal complement through reflectors.
  // SVD still decides rank with the identical cutoff and minimum-norm solve.
  Eigen::MatrixXd combined(j.rows(),retained+1);
  combined.leftCols(retained)=j.leftCols(retained);combined.col(retained)=residual;
  Eigen::MatrixXd backsub=Eigen::MatrixXd::Zero(nuisance.cols(),retained+1);
  if(rank) {
    const Eigen::MatrixXd observed=svd.matrixU().leftCols(rank);
    backsub.noalias()=-(svd.matrixV().leftCols(rank)*singular.head(rank).cwiseInverse().asDiagonal())*
        (observed.transpose()*combined);
    if(rank<j.rows()) {
      Eigen::HouseholderQR<Eigen::MatrixXd> qr(observed);
      combined=(qr.householderQ().adjoint()*combined).eval();
    }
  }
  const auto remaining=j.rows()-rank;
  return {combined.bottomRows(remaining).leftCols(retained),combined.col(retained).tail(remaining),
          backsub.leftCols(retained),backsub.col(retained),rank,nuisance.cols()-rank};
}
}  // namespace posim_slam

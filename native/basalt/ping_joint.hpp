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
    return eliminateJointPatch(augmented,values,retained);
  }
  const Eigen::MatrixXd nuisance=j.rightCols(j.cols()-retained);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(nuisance,Eigen::ComputeFullU|Eigen::ComputeFullV);
  if (svd.info()!=Eigen::Success)throw std::runtime_error("joint_patch_svd_failed");
  const auto& singular=svd.singularValues();
  const double tolerance=std::max(nuisance.rows(),nuisance.cols())*
      std::numeric_limits<double>::epsilon()*singular[0];
  const Eigen::Index rank=(singular.array()>tolerance).count();
  const Eigen::MatrixXd null=svd.matrixU().rightCols(j.rows()-rank);
  const Eigen::MatrixXd inverse=svd.matrixV().leftCols(rank)*
      singular.head(rank).cwiseInverse().asDiagonal()*svd.matrixU().leftCols(rank).transpose();
  return {null.transpose()*j.leftCols(retained),null.transpose()*residual,
          -inverse*j.leftCols(retained),-inverse*residual,rank,nuisance.cols()-rank};
}
}  // namespace posim_slam

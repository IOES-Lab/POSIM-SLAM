// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_geometry.hpp"
#include <vector>

namespace posim_slam {
struct SonarTrackRows {
  Eigen::MatrixXd pose_jacobian;
  Eigen::MatrixXd point_jacobian;
  Eigen::VectorXd residual;
};
struct SonarReducedRows {
  Eigen::MatrixXd pose_jacobian;
  Eigen::VectorXd residual;
  Eigen::Index point_rank;
};

// Covariance order is [range, azimuth] for each observation, including temporal
// correlations. Each observation addresses one shared pose slot. Caller owns
// association, timestamps and covariance calibration; no diagonal fallback.
inline SonarTrackRows whitenSonarTrack(const std::vector<SonarPointRows>& rows,
    const std::vector<int>& pose_slots,int pose_count,
    const Eigen::MatrixXd& covariance) {
  const Eigen::Index n=rows.size(),m=2*n;
  if(n<1 || n>64 || pose_slots.size()!=rows.size() || pose_count<1 || pose_count>64 ||
      covariance.rows()!=m || covariance.cols()!=m || !covariance.allFinite() ||
      !(covariance.diagonal().array()>0).all())
    throw std::invalid_argument("invalid_sonar_track_covariance");
  // Normalize units before checking symmetry/positive definiteness: range and
  // azimuth variances may differ by many orders of magnitude.
  const Eigen::VectorXd scales=covariance.diagonal().array().sqrt();
  const Eigen::VectorXd inverse=scales.cwiseInverse();
  Eigen::MatrixXd correlation=inverse.asDiagonal()*covariance*inverse.asDiagonal();
  if(!correlation.allFinite() || (correlation-correlation.transpose()).cwiseAbs().maxCoeff()>1e-10)
    throw std::invalid_argument("asymmetric_sonar_track_covariance");
  correlation=(0.5*(correlation+correlation.transpose())).eval();
  Eigen::LLT<Eigen::MatrixXd> factor(correlation);
  if(factor.info()!=Eigen::Success)
    throw std::invalid_argument("nonpositive_sonar_track_covariance");
  Eigen::MatrixXd packed=Eigen::MatrixXd::Zero(m,6*pose_count+4);
  for(Eigen::Index i=0;i<n;++i) {
    const auto& row=rows[i];const int slot=pose_slots[i];
    if(slot<0 || slot>=pose_count || !row.residual.allFinite() ||
        !row.pose_jacobian.allFinite() || !row.point_jacobian.allFinite())
      throw std::invalid_argument("invalid_sonar_track_rows");
    packed.block<2,6>(2*i,6*slot)=row.pose_jacobian;
    packed.block<2,3>(2*i,6*pose_count)=row.point_jacobian;
    packed.block<2,1>(2*i,6*pose_count+3)=row.residual;
  }
  packed=factor.matrixL().solve((inverse.asDiagonal()*packed).eval()).eval();
  if(!packed.allFinite())throw std::invalid_argument("nonfinite_whitened_sonar_track");
  return {packed.leftCols(6*pose_count),packed.middleCols(6*pose_count,3),packed.rightCols(1)};
}

// Eliminate the point only after ALL its owned rows (including any prior or
// other sensor rows) have been assembled. This projection retains inter-pose
// correlations and constant residual cost without inventing elevation data.
inline SonarReducedRows eliminateSonarPoint(const SonarTrackRows& rows,
    double relative_rank_tolerance=1e-10) {
  const auto m=rows.residual.size();
  if(m<1 || rows.pose_jacobian.rows()!=m || rows.point_jacobian.rows()!=m ||
      rows.point_jacobian.cols()!=3 || !rows.pose_jacobian.allFinite() ||
      !rows.point_jacobian.allFinite() || !rows.residual.allFinite() ||
      !std::isfinite(relative_rank_tolerance) || relative_rank_tolerance<=0 || relative_rank_tolerance>=1)
    throw std::invalid_argument("invalid_sonar_elimination_rows");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(rows.point_jacobian,Eigen::ComputeFullU);
  const double cutoff=relative_rank_tolerance*svd.singularValues()[0];
  const Eigen::Index rank=(svd.singularValues().array()>cutoff).count();
  const Eigen::MatrixXd nullspace=svd.matrixU().rightCols(m-rank).transpose();
  return {nullspace*rows.pose_jacobian,nullspace*rows.residual,rank};
}
}

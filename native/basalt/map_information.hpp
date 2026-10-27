// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "estimator_budget.hpp"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace posim_slam {

// Square roots retain null directions explicitly. A pseudoinverse alone would
// incorrectly assign zero variance to unobserved map coordinates.
struct MapPoseInformation {
  Eigen::MatrixXd covariance_root, nullspace;
};
inline MapPoseInformation mapPoseInformation(const Eigen::MatrixXd& normal) {
  const auto n=normal.rows();
  if(n<1 || n>kMaxJointColumns || normal.cols()!=n || !normal.allFinite() ||
      (normal-normal.transpose()).norm()>1e-10*std::max(1.,normal.norm()))
    throw std::invalid_argument("invalid_map_pose_normal");
  const double diagonal=normal.diagonal().cwiseAbs().maxCoeff();
  if(normal.diagonal().minCoeff()<-1e-12*std::max(1.,diagonal))
    throw std::invalid_argument("indefinite_map_pose_normal");
  Eigen::VectorXd scale(n);
  for(Eigen::Index i=0;i<n;++i)scale[i]=normal(i,i)>0 ? 1/std::sqrt(normal(i,i)) : 1.;
  const Eigen::MatrixXd symmetric=.5*(normal+normal.transpose());
  const Eigen::MatrixXd scaled=scale.asDiagonal()*symmetric*scale.asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(scaled);
  if(eigen.info()!=Eigen::Success)throw std::runtime_error("map_pose_eigendecomposition_failed");
  const double cutoff=64*n*std::numeric_limits<double>::epsilon()*
      std::max(1.,eigen.eigenvalues().cwiseAbs().maxCoeff());
  if(eigen.eigenvalues().minCoeff()<-cutoff)
    throw std::invalid_argument("indefinite_map_pose_normal");
  const Eigen::Index rank=(eigen.eigenvalues().array()>cutoff).count(),nullity=n-rank;
  MapPoseInformation result;
  result.covariance_root=scale.asDiagonal()*eigen.eigenvectors().rightCols(rank);
  for(Eigen::Index i=0;i<rank;++i)
    result.covariance_root.col(i)/=std::sqrt(eigen.eigenvalues()[nullity+i]);
  result.nullspace=scale.asDiagonal()*eigen.eigenvectors().leftCols(nullity);
  return result;
}

struct MapNuisanceInformation {
  Eigen::MatrixXd backsub, conditional_root, nullspace;
};
inline MapNuisanceInformation mapNuisanceInformation(
    const Eigen::MatrixXd& pose,const Eigen::MatrixXd& nuisance) {
  if(pose.rows()<1 || pose.rows()>4096 || pose.cols()<1 || pose.cols()>kMaxJointColumns ||
      nuisance.rows()!=pose.rows() || nuisance.cols()<1 || nuisance.cols()>24+3*kMaxPingLandmarks ||
      !pose.allFinite() || !nuisance.allFinite())
    throw std::invalid_argument("invalid_map_nuisance_rows");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(nuisance,Eigen::ComputeThinU|Eigen::ComputeFullV);
  if(svd.info()!=Eigen::Success)throw std::runtime_error("map_nuisance_svd_failed");
  const auto& singular=svd.singularValues();
  const double cutoff=std::max(nuisance.rows(),nuisance.cols())*
      std::numeric_limits<double>::epsilon()*singular[0];
  const Eigen::Index rank=(singular.array()>cutoff).count();
  MapNuisanceInformation result;
  result.conditional_root=svd.matrixV().leftCols(rank);
  for(Eigen::Index i=0;i<rank;++i)result.conditional_root.col(i)/=singular[i];
  result.backsub=-result.conditional_root*(svd.matrixU().leftCols(rank).transpose()*pose);
  result.nullspace=svd.matrixV().rightCols(nuisance.cols()-rank);
  return result;
}

struct MapPointInformation {
  bool observed=false;
  Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
  double maximum_sigma_m=std::numeric_limits<double>::infinity();
};
// A local bearing/range certificate cannot certify a retained world map point.
// Keep both diagnostics; admission continues to use the world result.
struct MapLandmarkInformation {
  MapPointInformation world,host_camera,conditional_host_camera;
};
inline MapPointInformation mapPointInformation(const MapPoseInformation& pose,
    const MapNuisanceInformation& local,const Eigen::MatrixXd& world_pose,
    const Eigen::MatrixXd& world_local) {
  const auto n=world_pose.cols(),m=world_local.cols();
  if(world_pose.rows()!=3 || world_local.rows()!=3 || local.backsub.rows()!=m ||
      local.backsub.cols()!=n || local.conditional_root.rows()!=m || local.nullspace.rows()!=m ||
      pose.covariance_root.rows()!=n || pose.nullspace.rows()!=n ||
      !world_pose.allFinite() || !world_local.allFinite())
    throw std::invalid_argument("invalid_map_world_derivatives");
  MapPointInformation result;
  const Eigen::MatrixXd total=world_pose+world_local*local.backsub;
  const auto affected=[](const Eigen::MatrixXd& j,const Eigen::MatrixXd& nullspace) {
    return nullspace.cols() && (j*nullspace).norm()>
        1e-8*std::max(1.,j.norm()*nullspace.norm());
  };
  if(affected(world_local,local.nullspace) || affected(total,pose.nullspace))return result;
  const Eigen::MatrixXd conditional=world_local*local.conditional_root;
  const Eigen::MatrixXd shared=total*pose.covariance_root;
  result.covariance=conditional*conditional.transpose()+shared*shared.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(result.covariance);
  if(eigen.info()!=Eigen::Success || !result.covariance.allFinite())
    throw std::runtime_error("invalid_map_point_covariance");
  result.maximum_sigma_m=std::sqrt(std::max(0.,eigen.eigenvalues().maxCoeff()));
  result.observed=std::isfinite(result.maximum_sigma_m);
  return result;
}
}  // namespace posim_slam

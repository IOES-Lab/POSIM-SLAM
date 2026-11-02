// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "map_information.hpp"
#include "sparse_normal_information.hpp"
#include "visual_geometry.hpp"
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace posim_slam {

// Identity is an image measurement, not a landmark ID: tracks may be rebuilt.
using MappingObservation = std::tuple<int64_t, int, int64_t>;
struct MappingPointRows {
  int64_t id = -1;
  Eigen::Vector3d world, host_origin;
  std::vector<int> pose_columns;
  std::vector<MappingObservation> observations;
  std::vector<bool> full_weight;
  // Two whitened rows per observation, using the optimizer's robust weight.
  Eigen::MatrixXd pose, point;
};
struct MappingPointCertificate {
  VisualPointSupport geometry;
  MapPointInformation world;
  bool mature = false, eligible = false;
};
struct MappingInformationResult {
  std::map<int64_t, MappingPointCertificate> points;
  Eigen::Index pose_nullity = 0;
  size_t observations = 0, pose_factors = 0;
};

// One graph linearization. Rebuild it after optimization; never append an old
// snapshot to a new one. Measurement IDs enforce ownership inside this graph,
// but do not establish independence from a separately recovered VIO prior.
class MappingInformation {
 public:
  explicit MappingInformation(int pose_columns) : columns_(pose_columns),normal_(pose_columns) {
    unreduced_diagonal_ = Eigen::VectorXd::Zero(columns_);
  }
  void addPoseFactor(const std::string& identity, const std::vector<int>& columns,
                     const Eigen::MatrixXd& whitened) {
    validateColumns(columns);
    if (identity.empty() || factors_.count(identity) || factors_.size() >= 65536 ||
        whitened.rows() < 1 || whitened.rows() > 4096 ||
        whitened.cols() != static_cast<Eigen::Index>(columns.size()) || !whitened.allFinite())
      throw std::invalid_argument("mapping_pose_factor");
    scatter(columns, whitened.transpose() * whitened);
    for (size_t i = 0; i < columns.size(); ++i)
      unreduced_diagonal_[columns[i]] += whitened.col(i).squaredNorm();
    factors_.insert(identity);
  }
  void addPoint(const MappingPointRows& rows) {
    validateColumns(rows.pose_columns);
    const auto count = rows.observations.size();
    if (rows.id < 0 || points_.count(rows.id) || points_.size() >= 100000 ||
        count < 2 || count > 2048 || owners_.size() + count > 1000000 ||
        rows.full_weight.size() != count || rows.pose.rows() != 2 * count ||
        rows.point.rows() != 2 * count || rows.point.cols() != 3 ||
        rows.pose.cols() != static_cast<Eigen::Index>(rows.pose_columns.size()) ||
        !rows.pose.allFinite() || !rows.point.allFinite() ||
        !rows.world.allFinite() || !rows.host_origin.allFinite())
      throw std::invalid_argument("mapping_point_rows");
    std::set<MappingObservation> pending;
    std::set<std::pair<int64_t,int>> images;
    std::set<int64_t> exposures;
    size_t full = 0;
    for (size_t i = 0; i < count; ++i) {
      const auto& key = rows.observations[i];
      const auto [stamp, camera, feature] = key;
      if (stamp <= 0 || camera < 0 || feature < 0 || owners_.count(key) ||
          !pending.insert(key).second || !images.emplace(stamp,camera).second)
        throw std::invalid_argument("mapping_observation_reused");
      if (rows.full_weight[i]) { exposures.insert(stamp); ++full; }
    }
    Eigen::MatrixXd accepted(2 * full, 3);
    size_t j = 0;
    for (size_t i = 0; i < count; ++i)
      if (rows.full_weight[i]) accepted.middleRows(2 * j++, 2) = rows.point.middleRows(2 * i, 2);
    Local local;
    local.columns = rows.pose_columns;
    local.geometry = visualPointSupport(accepted, rows.world - rows.host_origin, exposures.size());
    local.nuisance = mapNuisanceInformation(rows.pose, rows.point);
    const Eigen::MatrixXd reduced = rows.pose + rows.point * local.nuisance.backsub;
    // QR/SVD projection, without normal-equation subtraction or LM damping.
    scatter(rows.pose_columns, reduced.transpose() * reduced);
    for (size_t i = 0; i < rows.pose_columns.size(); ++i)
      unreduced_diagonal_[rows.pose_columns[i]] += rows.pose.col(i).squaredNorm();
    points_.emplace(rows.id, std::move(local));
    owners_.insert(pending.begin(), pending.end());
  }
  MappingInformationResult evaluate(double three_sigma_m) const {
    if (!std::isfinite(three_sigma_m) || three_sigma_m <= 0)
      throw std::invalid_argument("mapping_world_uncertainty_limit");
    // Scale from the original rows. Scaling a roundoff-only Schur diagonal to
    // one would turn a completely unobserved translation into information.
    Eigen::VectorXd scale(columns_);
    for (int i = 0; i < columns_; ++i)
      scale[i] = unreduced_diagonal_[i] > 0 ? 1 / std::sqrt(unreduced_diagonal_[i]) : 1;
    const bool sparse=normal_.factorize(scale);
    MapPoseInformation joint;
    if(!sparse) {
      if(columns_>kMaxJointColumns)throw std::runtime_error("mapping_sparse_rank_unresolved");
      joint=mapPoseInformation(Eigen::MatrixXd(normal_.unscaledNormal()),scale);
    }
    MappingInformationResult result;
    result.pose_nullity = sparse?0:joint.nullspace.cols();
    result.observations = owners_.size(); result.pose_factors = factors_.size();
    for (const auto& [id, local] : points_) {
      MappingPointCertificate certificate;
      certificate.geometry = local.geometry;
      if(sparse && !local.nuisance.nullspace.cols()) {
        const auto selected=normal_.covariance(local.columns,local.nuisance.backsub);
        if(selected.observed) {
          certificate.world.covariance=local.nuisance.conditional_root*local.nuisance.conditional_root.transpose()+selected.covariance;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(certificate.world.covariance);
          if(eigen.info()!=Eigen::Success || !certificate.world.covariance.allFinite())
            throw std::runtime_error("invalid_map_point_covariance");
          certificate.world.maximum_sigma_m=std::sqrt(std::max(0.,eigen.eigenvalues().maxCoeff()));
          certificate.world.observed=std::isfinite(certificate.world.maximum_sigma_m);
        }
      }else if(!sparse) {
        MapPoseInformation pose;
        pose.covariance_root.resize(local.columns.size(), joint.covariance_root.cols());
        pose.nullspace.resize(local.columns.size(), joint.nullspace.cols());
        for(size_t i=0;i<local.columns.size();++i) {
          pose.covariance_root.row(i)=joint.covariance_root.row(local.columns[i]);
          pose.nullspace.row(i)=joint.nullspace.row(local.columns[i]);
        }
        certificate.world=mapPointInformation(pose,local.nuisance,
          Eigen::MatrixXd::Zero(3,local.columns.size()),Eigen::Matrix3d::Identity());
      }
      // Match the archive's three-exposure maturity requirement. A two-view
      // component may constrain the graph without becoming a retained point.
      certificate.mature = local.geometry.exposures >= 3;
      certificate.eligible = certificate.mature && local.geometry.eligible &&
          certificate.world.observed && 3 * certificate.world.maximum_sigma_m <= three_sigma_m;
      result.points.emplace(id, certificate);
    }
    return result;
  }
  const Eigen::MatrixXd& poseNormal() const {
    if(columns_>kMaxJointColumns)throw std::runtime_error("dense_mapping_diagnostic_budget");
    diagnostic_normal_=Eigen::MatrixXd(normal_.unscaledNormal());return diagnostic_normal_;
  }

 private:
  struct Local {
    std::vector<int> columns;
    VisualPointSupport geometry;
    MapNuisanceInformation nuisance;
  };
  void validateColumns(const std::vector<int>& columns) const {
    std::set<int> seen;
    if (columns.empty() || columns.size() > static_cast<size_t>(columns_))
      throw std::invalid_argument("mapping_columns");
    for (int column : columns)
      if (column < 0 || column >= columns_ || !seen.insert(column).second)
        throw std::invalid_argument("mapping_columns");
  }
  void scatter(const std::vector<int>& columns, const Eigen::MatrixXd& normal) {
    normal_.addNormal(columns,normal);
  }
  int columns_;
  mutable SparseNormalInformation normal_;
  mutable Eigen::MatrixXd diagnostic_normal_;
  Eigen::VectorXd unreduced_diagonal_;
  std::set<std::string> factors_;
  std::set<MappingObservation> owners_;
  std::map<int64_t, Local> points_;
};
}  // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "surface_prior_owner.hpp"

namespace posim_slam {
struct RetiredPoseConditional {
  uint64_t id=0;
  std::vector<surface_prior::Variable> variables;
  std::vector<int> kept,removed;
  Eigen::MatrixXd regression,noise,unbounded;
  Eigen::VectorXd offset;
  double marginal_information_relative_error=0.,marginal_gradient_relative_error=0.;
};

// Capture exact retiring-state information BEFORE those state columns disappear.
// The pressure coordinate uses a fixed zero reference; its live datum may move.
// The input is the same undamped, consumed system used for marginalization.
inline std::optional<RetiredPoseConditional> captureRetiredPoseConditional(
    const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& source,
    const basalt::AbsOrderMap& retained,const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& residual,
    const std::optional<double>& pressure_datum,const Eigen::MatrixXd& actual_prior_j,const Eigen::VectorXd& actual_prior_b) {
  if(!owner.lmdb.getLandmarks().empty())throw std::invalid_argument("pose_conditional_capture_requires_no_visual_landmarks");
  const auto pressure=pressure_datum ? std::optional<surface_prior::ScalarCoordinate>(surface_prior::ScalarCoordinate{0,0.,*pressure_datum}) : std::nullopt;
  const auto coordinates=surface_prior::ownedRetirementCoordinates(owner,source,pressure);
  if(jacobian.cols()!=coordinates.delta.size() || jacobian.rows()!=residual.size() || !jacobian.rows())
    throw std::invalid_argument("pose_conditional_source_layout");
  RetiredPoseConditional out;out.variables=coordinates.order;
  std::vector<bool> used(jacobian.cols(),false);std::map<int,int64_t> by_column;
  for(const auto& [stamp,block]:retained.abs_order_map)by_column.emplace(block.first,stamp);
  for(const auto& [column,stamp]:by_column) {
    const auto from=source.abs_order_map.at(stamp),to=retained.abs_order_map.at(stamp);
    for(int i=0;i<to.second;++i){out.kept.push_back(from.first+i);used[from.first+i]=true;}
  }
  if(pressure_datum){out.kept.push_back(source.total_size);used[source.total_size]=true;}
  for(Eigen::Index i=0;i<jacobian.cols();++i)if(!used[i])out.removed.push_back(i);
  if(out.removed.empty())return std::nullopt;
  Eigen::MatrixXd ordered(jacobian.rows(),jacobian.cols());Eigen::Index column=0;
  for(int i:out.kept)ordered.col(column++)=jacobian.col(i);
  for(int i:out.removed)ordered.col(column++)=jacobian.col(i);
  Eigen::VectorXd b=residual-jacobian*coordinates.delta;
  if(ordered.rows()>ordered.cols()+1) {
    Eigen::MatrixXd augmented(ordered.rows(),ordered.cols()+1);augmented<<ordered,b;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
    const Eigen::MatrixXd compact=qr.matrixQR().topRows(augmented.cols()).template triangularView<Eigen::Upper>();
    ordered=compact.leftCols(jacobian.cols());b=compact.col(jacobian.cols());
  }
  const auto split=splitJointInformation(ordered,b,out.kept.size());
  out.regression=split.reduced.backsub_matrix;out.offset=split.reduced.backsub_offset;
  out.noise=split.observable_noise_factor;out.unbounded=split.unobservable_basis;
  if(actual_prior_j.cols()!=Eigen::Index(out.kept.size()) || actual_prior_j.rows()!=actual_prior_b.size())
    throw std::invalid_argument("pose_conditional_actual_prior_layout");
  const Eigen::MatrixXd expected_h=split.reduced.jacobian.transpose()*split.reduced.jacobian;
  const Eigen::MatrixXd actual_h=actual_prior_j.transpose()*actual_prior_j;
  const Eigen::VectorXd expected_g=split.reduced.jacobian.transpose()*split.reduced.residual;
  const Eigen::VectorXd actual_g=actual_prior_j.transpose()*actual_prior_b;
  out.marginal_information_relative_error=(expected_h-actual_h).norm()/std::max(1.,actual_h.norm());
  out.marginal_gradient_relative_error=(expected_g-actual_g).norm()/std::max(1.,actual_g.norm());
  if(!std::isfinite(out.marginal_information_relative_error) || !std::isfinite(out.marginal_gradient_relative_error))
    throw std::invalid_argument("nonfinite_pose_conditional_check");
  return out;
}
}  // namespace posim_slam

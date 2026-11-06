// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_joint.hpp"

namespace posim_slam {
struct AffineSupport {Eigen::VectorXd point;Eigen::MatrixXd free,normal;};
inline AffineSupport affineSupport(const Eigen::MatrixXd& c,const Eigen::VectorXd& target,Eigen::Index n) {
  if(n<0 || n>kMaxJointColumns || c.cols()!=n || c.rows()>1024 || target.size()!=c.rows() || !c.allFinite() || !target.allFinite())
    throw std::invalid_argument("finite_bounded_affine_constraints_required");
  if(!c.rows())return {Eigen::VectorXd::Zero(n),Eigen::MatrixXd::Identity(n,n),Eigen::MatrixXd::Zero(n,0)};
  if(!n) {
    if(target.norm()>2e-10)throw std::invalid_argument("inconsistent_affine_constraints");
    return {Eigen::VectorXd::Zero(0),Eigen::MatrixXd::Zero(0,0),Eigen::MatrixXd::Zero(0,0)};
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(c,Eigen::ComputeFullU|Eigen::ComputeFullV);
  const auto s=svd.singularValues();const double tolerance=std::numeric_limits<double>::epsilon()*std::max(c.rows(),c.cols())*(s.size() ? s[0] : 0.);
  const Eigen::Index rank=(s.array()>tolerance).count();AffineSupport out;
  out.normal=svd.matrixV().leftCols(rank);out.free=svd.matrixV().rightCols(n-rank);
  out.point=Eigen::VectorXd::Zero(n);
  if(rank)out.point=out.normal*s.head(rank).cwiseInverse().asDiagonal()*svd.matrixU().leftCols(rank).transpose()*target;
  if((c*out.point-target).norm()>2e-10*std::max(1.,target.norm()))throw std::invalid_argument("inconsistent_affine_constraints");
  return out;
}
struct ConstrainedInformationSplit {
  JointPatchReduction reduced;
  Eigen::MatrixXd noise,unbounded,deterministic,constraint_jacobian;
  Eigen::VectorXd constraint_target;
  double source_roundoff=0.;
};
inline ConstrainedInformationSplit splitConstrainedInformation(const Eigen::MatrixXd& j,const Eigen::VectorXd& r,
    Eigen::Index kept,const Eigen::MatrixXd& c,const Eigen::VectorXd& target,double source_roundoff=0.) {
  if(j.rows()>4096 || j.cols()>kMaxJointColumns || r.size()!=j.rows() || kept<0 || kept>j.cols() || !j.allFinite() || !r.allFinite() ||
      !std::isfinite(source_roundoff) || source_roundoff<0)
    throw std::invalid_argument("invalid_constrained_information_split");
  affineSupport(c,target,j.cols());const auto gone=j.cols()-kept;ConstrainedInformationSplit out;out.source_roundoff=source_roundoff;
  if(!c.rows()) {
    const auto split=splitJointInformation(j,r,kept,true,source_roundoff);out.reduced=split.reduced;
    out.noise=split.observable_noise_factor;out.unbounded=split.unobservable_basis;
    out.deterministic=Eigen::MatrixXd::Zero(gone,0);out.constraint_jacobian=Eigen::MatrixXd::Zero(0,kept);out.constraint_target.resize(0);return out;
  }
  const Eigen::MatrixXd cr=c.rightCols(gone),ck=c.leftCols(kept);
  Eigen::MatrixXd inverse=Eigen::MatrixXd::Zero(gone,c.rows()),z,hard,remaining;
  if(gone) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(cr,Eigen::ComputeFullU|Eigen::ComputeFullV);
    const auto s=svd.singularValues();const double tolerance=std::numeric_limits<double>::epsilon()*std::max(cr.rows(),cr.cols())*(s.size() ? s[0] : 0.);
    const Eigen::Index rank=(s.array()>tolerance).count();hard=svd.matrixV().leftCols(rank);z=svd.matrixV().rightCols(gone-rank);
    if(rank)inverse=hard*s.head(rank).cwiseInverse().asDiagonal()*svd.matrixU().leftCols(rank).transpose();
    remaining=svd.matrixU().rightCols(c.rows()-rank).transpose();
  } else {z.resize(0,0);hard.resize(0,0);remaining=Eigen::MatrixXd::Identity(c.rows(),c.rows());}
  const Eigen::MatrixXd p=-inverse*ck;const Eigen::VectorXd q=inverse*target;
  const auto root=affineSupport(remaining*ck,remaining*target,kept);
  out.constraint_jacobian=root.normal.transpose();out.constraint_target=out.constraint_jacobian*root.point;
  Eigen::MatrixXd reduced(j.rows(),kept+z.cols());reduced<<j.leftCols(kept)+j.rightCols(gone)*p,j.rightCols(gone)*z;
  // The feasible-coordinate map is [I,0; p,z]. Because p lies in the
  // constrained row space and z is its orthonormal nullspace, its operator
  // norm is sqrt(1 + ||p||_2^2). Carry the same source error through that map.
  double transformed_roundoff=source_roundoff;
  if(source_roundoff>0 && p.rows() && p.cols()) {
    Eigen::JacobiSVD<Eigen::MatrixXd> scale(p);
    if(scale.info()!=Eigen::Success)throw std::runtime_error("constraint_roundoff_svd_failed");
    transformed_roundoff*=std::hypot(1.,scale.singularValues()[0]);
  }
  const auto split=splitJointInformation(reduced,r+j.rightCols(gone)*q,kept,true,transformed_roundoff);out.reduced=split.reduced;
  out.source_roundoff=transformed_roundoff;
  out.reduced.backsub_matrix=p+z*split.reduced.backsub_matrix;out.reduced.backsub_offset=q+z*split.reduced.backsub_offset;
  out.noise=z*split.observable_noise_factor;out.unbounded=z*split.unobservable_basis;out.deterministic=hard;
  return out;
}
}  // namespace posim_slam

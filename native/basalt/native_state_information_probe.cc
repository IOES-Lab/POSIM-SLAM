// SPDX-License-Identifier: Apache-2.0
#include "native_state_information.hpp"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cassert>
#include <iostream>

using namespace posim_slam;
using Eigen::MatrixXd;

int main() {
  // Full marginal, not the inverse of a selected principal information block.
  MatrixXd h=MatrixXd::Identity(18,18);
  h(3,17)=h(17,3)=.4;h(12,17)=h(17,12)=.3;h(3,12)=h(12,3)=.2;
  const auto pose=mapPoseInformation(h);
  const auto result=selectedStateInformation(pose,0);
  const MatrixXd full=h.ldlt().solve(MatrixXd::Identity(18,18));
  assert((result.observable_covariance-full.topLeftCorner(15,15)).norm()<1e-12);
  assert(std::abs(result.observable_covariance(3,12))>.01);
  assert((result.observable_covariance-h.topLeftCorner(15,15).inverse()).norm()>.1);
  for(const auto& block:result.blocks)assert(block.observed);
  assert(result.unbounded.cols()==0 && result.source_columns==18);

  // A shared gravity/accelerometer-bias direction is unbounded in both blocks.
  h.setIdentity();h(3,3)=h(12,12)=h(3,12)=h(12,3)=1.;
  const auto confounded=selectedStateInformation(mapPoseInformation(h),0);
  assert(!confounded.blocks[1].observed && !confounded.blocks[4].observed);
  assert(confounded.blocks[0].observed && confounded.blocks[2].observed && confounded.blocks[3].observed);
  assert(confounded.unbounded.cols()==1);

  // Free nuisance coordinates outside the selected state need not affect it.
  h.setIdentity();h.bottomRightCorner(3,3).setZero();
  const auto outside=selectedStateInformation(mapPoseInformation(h),0);
  for(const auto& block:outside.blocks)assert(block.observed);
  assert(outside.unbounded.norm()==0 && outside.unbounded.cols()==3);

  // Keep cross terms through a nontrivial current-coordinate transformation.
  Eigen::Matrix<double,15,15> derivative=Eigen::Matrix<double,15,15>::Identity();
  derivative(3,4)=.3;derivative(4,3)=-.2;derivative(9,9)=2.;
  const auto transformed=selectedStateInformation(pose,0,derivative);
  assert((transformed.observable_covariance-derivative*result.observable_covariance*derivative.transpose()).norm()<1e-12);
  for(int i=0;i<5;++i)assert((transformed.blocks[i].covariance-
      transformed.observable_covariance.block<3,3>(3*i,3*i)).norm()<1e-12);

  int rejected=0;
  for(int offset:{-1,4})try {selectedStateInformation(pose,offset);}catch(const std::invalid_argument&){++rejected;}
  derivative(0,0)=std::numeric_limits<double>::quiet_NaN();
  try {selectedStateInformation(pose,0,derivative);}catch(const std::invalid_argument&){++rejected;}
  auto invalid=pose;invalid.nullspace.resize(17,0);
  try {selectedStateInformation(invalid,0);}catch(const std::invalid_argument&){++rejected;}
  assert(rejected==4);
  std::cout<<"PASS native state full marginal, cross terms, confounded/free modes, coordinates and four refusals\n";
}

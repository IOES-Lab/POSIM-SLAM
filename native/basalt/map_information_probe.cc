// SPDX-License-Identifier: Apache-2.0
#include "map_information.hpp"
#include <Eigen/QR>
#include <cassert>
#include <cstdlib>
#include <iostream>

using Eigen::MatrixXd;
using namespace posim_slam;

void dense_control(int poses,int nuisance,int rows) {
  MatrixXd jp=MatrixXd::Random(rows,poses),jl=MatrixXd::Random(rows,nuisance);
  const MatrixXd projection=jl.completeOrthogonalDecomposition().solve(jp);
  const MatrixXd reduced=jp-jl*projection;
  MatrixXd anchor=MatrixXd::Random(poses+3,poses);
  const MatrixXd prior=anchor.transpose()*anchor+.1*MatrixXd::Identity(poses,poses);
  const MatrixXd h=reduced.transpose()*reduced+prior;
  const auto p=mapPoseInformation(h);const auto l=mapNuisanceInformation(jp,jl);
  MatrixXd xp=MatrixXd::Random(3,poses),xl=MatrixXd::Random(3,nuisance);
  const auto result=mapPointInformation(p,l,xp,xl);assert(result.observed);
  MatrixXd full(poses+nuisance,poses+nuisance);
  full.topLeftCorner(poses,poses)=jp.transpose()*jp+prior;
  full.bottomRightCorner(nuisance,nuisance)=jl.transpose()*jl;
  full.topRightCorner(poses,nuisance)=jp.transpose()*jl;
  full.bottomLeftCorner(nuisance,poses)=jl.transpose()*jp;
  MatrixXd derivative(3,poses+nuisance);derivative<<xp,xl;
  const MatrixXd oracle=derivative*full.ldlt().solve(derivative.transpose());
  assert((oracle-result.covariance).norm()<1e-9*std::max(1.,oracle.norm()));
  // Changing pose parameter units leaves world uncertainty unchanged.
  Eigen::VectorXd units=Eigen::VectorXd::LinSpaced(poses,.001,100.);
  const auto changed=mapPoseInformation(units.asDiagonal()*h*units.asDiagonal());
  const auto local=mapNuisanceInformation(jp*units.asDiagonal(),jl);
  const auto rescaled=mapPointInformation(changed,local,xp*units.asDiagonal(),xl);
  assert(rescaled.observed && (rescaled.covariance-result.covariance).norm()<1e-8);
}
void coordinate_control() {
  // Unknown common translation g, uncertain inter-camera displacement d,
  // and measured local point l-d. A host-coordinate query cancels g, but
  // must retain d uncertainty. Fixing all poses would hide that uncertainty.
  MatrixXd jp=MatrixXd::Zero(3,6);jp.rightCols(3)=-100*MatrixXd::Identity(3,3);
  const MatrixXd jl=100*MatrixXd::Identity(3,3);
  const auto local=mapNuisanceInformation(jp,jl);
  MatrixXd normal=MatrixXd::Zero(6,6);
  normal.topLeftCorner(3,3)=.01*MatrixXd::Identity(3,3);
  normal.bottomRightCorner(3,3)=25*MatrixXd::Identity(3,3);
  MatrixXd direct=MatrixXd::Zero(3,6);direct.leftCols(3).setIdentity();
  const MatrixXd zero=MatrixXd::Zero(3,6),identity=MatrixXd::Identity(3,3);
  const auto pose=mapPoseInformation(normal);
  const auto world=mapPointInformation(pose,local,direct,identity);
  const auto host=mapPointInformation(pose,local,zero,identity);
  const MapPoseInformation fixed{MatrixXd::Zero(6,0),MatrixXd::Zero(6,0)};
  const auto conditional=mapPointInformation(fixed,local,zero,identity);
  assert(world.observed && host.observed && conditional.observed);
  assert((world.covariance-100.0401*identity).norm()<1e-9);
  assert((host.covariance-.0401*identity).norm()<1e-12);
  assert((conditional.covariance-.0001*identity).norm()<1e-12);
  normal.topLeftCorner(3,3).setZero();
  const auto gauge=mapPoseInformation(normal);
  assert(!mapPointInformation(gauge,local,direct,identity).observed);
  assert(mapPointInformation(gauge,local,zero,identity).observed);
}
int main() {
  std::srand(47);
  dense_control(6,3,20);dense_control(15,12,60);dense_control(30,24,100);
  coordinate_control();
  MatrixXd identity=MatrixXd::Identity(3,3),zero=MatrixXd::Zero(3,3);
  const auto unknown_pose=mapPoseInformation(zero);
  const auto relative=mapNuisanceInformation(-identity,identity);
  assert(!mapPointInformation(unknown_pose,relative,zero,identity).observed);
  const auto gauge_cancelled=mapPointInformation(unknown_pose,relative,-identity,identity);
  assert(gauge_cancelled.observed && (gauge_cancelled.covariance-identity).norm()<1e-12);
  MatrixXd deficient=identity;deficient(2,2)=0;
  const auto unknown_depth=mapNuisanceInformation(zero,deficient);
  assert(!mapPointInformation(mapPoseInformation(identity),unknown_depth,zero,identity).observed);
  MatrixXd xy=identity;xy(2,2)=0;
  assert(mapPointInformation(mapPoseInformation(identity),unknown_depth,zero,xy).observed);
  bool invalid=false;
  try {mapPoseInformation(-identity);}catch(const std::invalid_argument&){invalid=true;}
  assert(invalid);
  std::cout<<"map information dense, correlated-group, gauge, rank and unit controls passed\n";
}

// SPDX-License-Identifier: Apache-2.0
#include "sonar_track.hpp"
#include <cassert>
#include <iostream>
int main() {
  using namespace posim_slam;
  std::vector<SonarPointRows> observations;
  const Eigen::Vector3d point(4,.7,.3);
  for(int i=0;i<3;++i)
    observations.push_back(sonarPointRows(Eigen::Vector3d(0,.2*i,0),Eigen::Matrix3d::Identity(),
      Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),point,4.1,.1,.2,10.,2.1,.35));
  Eigen::MatrixXd covariance=Eigen::MatrixXd::Identity(6,6);
  covariance(0,2)=covariance(2,0)=.4;
  Eigen::VectorXd scale(6);scale<<.02,.003,.02,.003,.02,.003;
  covariance=scale.asDiagonal()*covariance*scale.asDiagonal();
  auto rows=whitenSonarTrack(observations,{0,1,2},3,covariance);
  Eigen::VectorXd raw(6);for(int i=0;i<3;++i)raw.segment<2>(2*i)=observations[i].residual;
  assert(std::abs(rows.residual.squaredNorm()-raw.dot(covariance.ldlt().solve(raw)))<1e-9);
  auto reduced=eliminateSonarPoint(rows);
  assert(reduced.point_rank==3);
  // Check minimized cost at multiple pose perturbations against independent QR.
  for(int k=0;k<8;++k) {
    Eigen::VectorXd delta(18);for(int j=0;j<18;++j)delta[j]=.001*std::sin(k+2*j);
    Eigen::VectorXd b=rows.residual+rows.pose_jacobian*delta;
    Eigen::VectorXd dx=rows.point_jacobian.completeOrthogonalDecomposition().solve(-b);
    const double cost=(b+rows.point_jacobian*dx).squaredNorm();
    assert(std::abs(cost-(reduced.residual+reduced.pose_jacobian*delta).squaredNorm())<1e-8);
  }
  Eigen::MatrixXd h=reduced.pose_jacobian.transpose()*reduced.pose_jacobian;
  assert(h.block(0,6,6,6).norm()>1e-3);
  // A common world translation remains a gauge after point elimination.
  Eigen::MatrixXd gauge=Eigen::MatrixXd::Zero(18,3);
  for(int i=0;i<3;++i)gauge.block<3,3>(6*i,0).setIdentity();
  assert((reduced.pose_jacobian*gauge).norm()<1e-9);
  auto stationary=whitenSonarTrack({observations[0],observations[0]}, {0,1},2,Eigen::MatrixXd::Identity(4,4));
  assert(eliminateSonarPoint(stationary).point_rank==2);
  auto single=whitenSonarTrack({observations[0]}, {0},1,Eigen::MatrixXd::Identity(2,2));
  assert(eliminateSonarPoint(single).residual.size()==0);
  for(int mode=0;mode<3;++mode) {
    auto bad=covariance;
    if(mode==0)bad(0,2)+=.1;
    if(mode==1)bad(0,0)=-1;
    if(mode==2)bad(0,2)=bad(2,0)=1;
    bool rejected=false;
    try {whitenSonarTrack(observations,{0,1,2},3,bad);}
    catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  std::cout<<"PASS correlated sonar whitening, joint elimination, rank and covariance rejection\n";
}

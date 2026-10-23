// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_marginal.hpp"
#include <basalt/vi_estimator/ba_base.h>
#include <basalt/vi_estimator/marg_helper.h>
#include <iostream>
#include <random>

int main() {
  try {
    using namespace posim_slam;
    // Two full IMU states plus one plane and one landmark. Retain the second
    // IMU state first, then only the first state's pose (different column order).
    basalt::AbsOrderMap source,keep;source.items=keep.items=2;
    source.total_size=30;source.abs_order_map[100]={0,15};source.abs_order_map[200]={15,15};
    keep.total_size=21;keep.abs_order_map[100]={15,6};keep.abs_order_map[200]={0,15};
    std::mt19937 generator(47);std::normal_distribution<double> normal;
    Eigen::MatrixXd j(60,36);Eigen::VectorXd r(60);
    for(int row=0;row<60;++row) {
      r[row]=normal(generator);
      for(int col=0;col<36;++col)j(row,col)=normal(generator);
    }
    basalt::BundleAdjustmentBase<double> estimator;
    for(int64_t t:{100,200}) {
      estimator.frame_states.emplace(t,basalt::PoseVelBiasStateWithLin<double>(
          t,Sophus::SE3d(),Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),true));
      estimator.frame_states.at(t).applyInc(Eigen::Matrix<double,15,1>::Constant(.01*(t/100)));
    }
    estimator.frame_poses.emplace(100,basalt::PoseStateWithLin<double>(estimator.frame_states.at(100)));
    estimator.frame_states.erase(100);
    Eigen::VectorXd delta;estimator.computeDelta(keep,delta);
    const auto prior=marginalizeJointSystem(j,r,source,keep,delta);
    std::set<int> retained,marginal;
    for(int i=0;i<36;++i)if(i<6 || (i>=15 && i<30))retained.insert(i);else marginal.insert(i);
    Eigen::MatrixXd raw=j,reference;Eigen::VectorXd residual=r,reference_r;
    basalt::MargHelper<double>::marginalizeHelperSqrtToSqrt(raw,residual,retained,marginal,reference,reference_r);
    Eigen::MatrixXd reordered(reference.rows(),21);
    for(int i=0;i<21;++i)reordered.col(i)=reference.col(i<15 ? 6+i : i-15);
    const double information_error=(prior.H.transpose()*prior.H-reordered.transpose()*reordered).norm();
    const double gradient_error=(prior.H.transpose()*(prior.H*delta+prior.b)-reordered.transpose()*reference_r).norm();
    double before,after;estimator.computeMargPriorError(prior,before);
    const Eigen::Matrix<double,15,1> increment=Eigen::Matrix<double,15,1>::Constant(.005);
    estimator.frame_states.at(200).applyInc(increment);
    estimator.computeMargPriorError(prior,after);
    Eigen::VectorXd changed;estimator.computeDelta(keep,changed);
    const double expected=.5*((prior.H*changed+prior.b).squaredNorm()-(prior.H*delta+prior.b).squaredNorm());
    const double change_error=std::abs((after-before)-expected);
    bool invalid_state=false;auto bad=keep;bad.abs_order_map[100]={15,15};
    try {marginalizeJointSystem(j,r,source,bad,delta);}catch(const std::invalid_argument&){invalid_state=true;}
    bool passed=information_error<1e-9 && gradient_error<1e-9 && change_error<1e-10 && invalid_state;
    std::cout<<Json({{"passed",passed},{"imu_states",2},{"retained_columns",21},
      {"basalt_information_error",information_error},{"basalt_gradient_error",gradient_error},
      {"rebased_trial_cost_change_error",change_error},{"invalid_state_order_rejected",invalid_state}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

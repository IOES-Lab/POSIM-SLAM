// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_commit.hpp"
#include <basalt/imu/preintegration.h>
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
    // The runtime linearization has already jointly eliminated each group's
    // landmarks and plane. Eliminating old states must retain the same system.
    const auto joint=eliminateJointPatch(j,r,source.total_size);
    const auto staged=marginalizeJointSystem(joint.jacobian,joint.residual,source,keep,delta);
    const double staged_information_error=(prior.H.transpose()*prior.H-staged.H.transpose()*staged.H).norm();
    const double staged_gradient_error=(prior.H.transpose()*(prior.H*delta+prior.b)-staged.H.transpose()*(staged.H*delta+staged.b)).norm();
    // A retirement can consume factors without dropping any state columns.
    const auto unchanged=marginalizeJointSystem(joint.jacobian,joint.residual,source,source,Eigen::VectorXd::Zero(30));
    const bool no_elimination_ok=unchanged.H.rows()<=30 &&
      (unchanged.H.transpose()*unchanged.H-joint.jacobian.transpose()*joint.jacobian).norm()<1e-9 &&
      (unchanged.H.transpose()*unchanged.b-joint.jacobian.transpose()*joint.residual).norm()<1e-9;
    auto repeated=unchanged;
    for(int repeat=0;repeat<20;++repeat) {
      Eigen::MatrixXd stacked(repeated.H.rows()+joint.jacobian.rows(),30);
      Eigen::VectorXd stacked_r(repeated.b.size()+joint.residual.size());
      stacked<<repeated.H,joint.jacobian;stacked_r<<repeated.b,joint.residual;
      repeated=marginalizeJointSystem(stacked,stacked_r,source,source,Eigen::VectorXd::Zero(30));
    }
    const bool repeated_prior_bounded=repeated.H.rows()<=30 &&
      (repeated.H.transpose()*repeated.H-21.*joint.jacobian.transpose()*joint.jacobian).norm()<1e-8 &&
      (repeated.H.transpose()*repeated.b-21.*joint.jacobian.transpose()*joint.residual).norm()<1e-8;
    // No artificial information may appear in an unobservable retained column.
    Eigen::MatrixXd gauge_j=j;gauge_j.col(15).setZero();
    const auto gauge_group=eliminateJointPatch(gauge_j,r,source.total_size);
    const auto gauge_prior=marginalizeJointSystem(gauge_group.jacobian,gauge_group.residual,source,keep,delta);
    const bool gauge_preserved=gauge_prior.H.col(0).norm()==0;
    bool malformed_rejected=false;
    try{marginalizeJointSystem(joint.jacobian,Eigen::VectorXd::Zero(1),source,source,Eigen::VectorXd::Zero(30));}
    catch(const std::invalid_argument&){malformed_rejected=true;}

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
    basalt::BundleAdjustmentBase<double> live;
    live.frame_poses.emplace(50,basalt::PoseStateWithLin<double>(50,Sophus::SE3d(),true));
    for(int64_t t:{100,200,300,400})live.frame_states.emplace(t,basalt::PoseVelBiasStateWithLin<double>(
      t,Sophus::SE3d(),Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),true));
    const auto layout=planPingMarginalLayout(live,{100},{200},{50,100,200,300,400},{{100,200},{200,300},{300,400}});
    const bool layout_ok=layout.source.total_size==66 && layout.retained.total_size==42 &&
      layout.retained.abs_order_map.at(200)==std::make_pair(size_t(6),size_t(6)) &&
      layout.retained.abs_order_map.at(300)==std::make_pair(size_t(12),size_t(15)) &&
      layout.retained.abs_order_map.at(400)==std::make_pair(size_t(27),size_t(15)) &&
      layout.consumed_imu_starts==std::set<int64_t>({100,200}) && live.frame_states.size()==4;
    bool dependency_rejected=false;
    try{planPingMarginalLayout(live,{100},{200},{500},{});}catch(const std::invalid_argument&){dependency_rejected=true;}
    Eigen::MatrixXd expanded_j(110,72);Eigen::VectorXd expanded_r(110);
    for(int i=0;i<110;++i){expanded_r[i]=normal(generator);for(int k=0;k<72;++k)expanded_j(i,k)=normal(generator);}
    const auto expanded_prior=marginalizeJointSystem(expanded_j,expanded_r,layout.source,layout.retained,Eigen::VectorXd::Zero(42));
    const Eigen::MatrixXd covariance=(expanded_j.transpose()*expanded_j).ldlt().solve(Eigen::MatrixXd::Identity(72,72));
    std::vector<int> columns;
    // Retained order: old pose 50, demoted pose 200, full states 300 and 400.
    for(int k=0;k<6;++k)columns.push_back(k);
    for(int k=21;k<27;++k)columns.push_back(k);
    for(int k=36;k<66;++k)columns.push_back(k);
    Eigen::MatrixXd expected_covariance(42,42);
    for(int i=0;i<42;++i)for(int k=0;k<42;++k)expected_covariance(i,k)=covariance(columns[i],columns[k]);
    const double expanded_error=(expected_covariance-(expanded_prior.H.transpose()*expanded_prior.H).ldlt().solve(Eigen::MatrixXd::Identity(42,42))).norm();
    live.posim_ping_planes.set(1,Vec3(0,0,1),3.);
    JointPingGroupSpec commit_group{1,{0,1,2},[](const auto&){return NativePingGroup{};},{300}};
    std::vector<JointPingGroupSpec> commit_groups={commit_group};
    for(int id=0;id<3;++id) {
      basalt::Keypoint<double> point;point.host_kf_id={200,0};point.direction=Eigen::Vector2d(.1*id,0);point.inv_dist=1./3.;
      live.lmdb.addLandmark(id,point);
      for(int64_t t:{200,300,400})live.lmdb.addObservation({t,0},basalt::KeypointObservation<double>{id,Eigen::Vector2d::Zero()});
    }
    std::map<int64_t,basalt::IntegratedImuMeasurement<double>> commit_imu;
    for(int64_t t:{100,200,300}) {
      auto [it,inserted]=commit_imu.emplace(t,basalt::IntegratedImuMeasurement<double>(t,Vec3::Zero(),Vec3::Zero()));
      basalt::ImuData<double> data;data.t_ns=t+100;data.gyro.setZero();data.accel=Vec3(0,0,9.80665);
      it->second.integrate(data,Vec3::Ones(),Vec3::Ones());
    }
    const auto retirement=planPingRetirement(live,commit_groups,{100},{100},{0});
    basalt::MargLinData<double> installed;installed.H=Eigen::MatrixXd::Identity(66,66);installed.b=Eigen::VectorXd::Zero(66);installed.order=layout.source;installed.is_sqrt=true;
    auto broken_prior=expanded_prior;broken_prior.b[0]=std::numeric_limits<double>::quiet_NaN();bool commit_failure_atomic=false;
    try{installPingMarginalPrior(live,installed,commit_groups,commit_imu,retirement,layout,{100},{200},broken_prior);}
    catch(const std::invalid_argument&){commit_failure_atomic=live.posim_consumed_visual.size()==0 && live.frame_states.size()==4 && live.lmdb.numLandmarks()==3 && commit_imu.size()==3 && installed.H.cols()==66 && live.posim_ping_planes.size()==1;}
    live.frame_states.at(300).applyInc(Eigen::Matrix<double,15,1>::Constant(.003));
    live.frame_states.at(400).setLinFalse();
    live.frame_states.at(400).applyInc(Eigen::Matrix<double,15,1>::Constant(.004));
    const auto retained_pose=live.frame_states.at(300).getState().T_w_i;
    const auto retained_reference=live.frame_states.at(300).getStateLin().T_w_i;
    const auto newly_fixed_pose=live.frame_states.at(400).getState().T_w_i;
    // Rebase the synthetic prior for the newly frozen fourth state's reference.
    auto commit_prior=expanded_prior;Eigen::VectorXd reference_shift=Eigen::VectorXd::Zero(42);
    reference_shift.tail<15>().setConstant(.004);commit_prior.b+=commit_prior.H*reference_shift;
    const auto revision=live.posim_ping_state_revision;
    installPingMarginalPrior(live,installed,commit_groups,commit_imu,retirement,layout,{100},{200},commit_prior);
    bool repeat_rejected=false;
    try{installPingMarginalPrior(live,installed,commit_groups,commit_imu,retirement,layout,{100},{200},commit_prior);}
    catch(const std::invalid_argument&){repeat_rejected=true;}
    const bool commit_ok=commit_failure_atomic && repeat_rejected && live.frame_states.size()==2 &&
      live.frame_poses.size()==2 && live.frame_poses.count(200) && live.lmdb.numLandmarks()==0 &&
      live.posim_ping_planes.size()==0 && commit_groups.empty() && commit_imu.size()==1 && commit_imu.count(300) &&
      live.posim_ping_state_revision==revision+1 &&
      (live.frame_states.at(300).getState().T_w_i.matrix()-retained_pose.matrix()).norm()==0 &&
      (live.frame_states.at(300).getStateLin().T_w_i.matrix()-retained_reference.matrix()).norm()==0 &&
      live.frame_states.at(400).isLinearized() &&
      (live.frame_states.at(400).getState().T_w_i.matrix()-newly_fixed_pose.matrix()).norm()==0 &&
      (installed.H-commit_prior.H).norm()==0 &&
      (installed.b-commit_prior.b).norm()==0 && installed.order.abs_order_map==layout.retained.abs_order_map;
    bool passed=repeated_prior_bounded && gauge_preserved && staged_information_error<1e-9 && staged_gradient_error<1e-9 && no_elimination_ok && malformed_rejected && commit_ok && layout_ok && dependency_rejected && expanded_error<1e-9 && information_error<1e-9 && gradient_error<1e-9 && change_error<1e-10 && invalid_state;
    std::cout<<Json({{"repeated_prior_bounded",repeated_prior_bounded},{"gauge_preserved",gauge_preserved},{"staged_information_error",staged_information_error},{"staged_gradient_error",staged_gradient_error},{"no_elimination_preserved",no_elimination_ok},{"malformed_rejected",malformed_rejected},{"prior_installation_passed",commit_ok},{"invalid_prior_left_state_unchanged",commit_failure_atomic},{"repeated_installation_rejected",repeat_rejected},{"expanded_layout_passed",layout_ok},{"expanded_covariance_error",expanded_error},{"missing_dependency_rejected",dependency_rejected},{"passed",passed},{"imu_states",2},{"retained_columns",21},
      {"basalt_information_error",information_error},{"basalt_gradient_error",gradient_error},
      {"rebased_trial_cost_change_error",change_error},{"invalid_state_order_rejected",invalid_state}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

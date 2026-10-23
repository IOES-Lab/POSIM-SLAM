// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_state.hpp"
#include <basalt/vi_estimator/ba_base.h>
#include <iostream>

int main() {
  try {
    basalt::BundleAdjustmentBase<double> estimator;
    estimator.frame_poses.emplace(100,basalt::PoseStateWithLin<double>(100,Sophus::SE3d(),true));
    estimator.frame_states.emplace(200,basalt::PoseVelBiasStateWithLin<double>(
        200,Sophus::SE3d(),Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),true));
    basalt::Keypoint<double> landmark;landmark.host_kf_id={100,0};landmark.direction.setZero();landmark.inv_dist=.5;
    estimator.lmdb.addLandmark(7,landmark);
    estimator.posim_ping_planes.set(1,Vec3(0,0,1),2.);
    auto capture=[&] {
      std::vector<double> values;
      auto append=[&](const auto& x){for(Eigen::Index i=0;i<x.size();++i)values.push_back(x.derived().coeff(i));};
      append(estimator.frame_poses.at(100).getPose().matrix());
      append(estimator.frame_poses.at(100).getDelta());
      const auto& state=estimator.frame_states.at(200).getState();
      append(state.T_w_i.matrix());append(state.vel_w_i);append(state.bias_gyro);append(state.bias_accel);
      append(estimator.frame_states.at(200).getDelta());
      append(estimator.lmdb.getLandmark(7).direction);values.push_back(estimator.lmdb.getLandmark(7).inv_dist);
      append(estimator.posim_ping_planes.at(1).normal);values.push_back(estimator.posim_ping_planes.at(1).distance);
      append(estimator.posim_ping_planes.at(1).reference.normal);values.push_back(estimator.posim_ping_planes.at(1).reference.distance);
      append(estimator.posim_ping_planes.at(1).delta);
      return values;
    };
    auto mutate=[&] {
      Eigen::Matrix<double,6,1> pose;pose<<.1,.2,.3,.01,.02,.03;estimator.frame_poses.at(100).applyInc(pose);
      Eigen::Matrix<double,15,1> state=Eigen::Matrix<double,15,1>::Constant(.01);
      estimator.frame_states.at(200).applyInc(state);
      estimator.lmdb.getLandmark(7).direction+=Eigen::Vector2d(.02,.03);
      estimator.lmdb.getLandmark(7).inv_dist+=.1;
      estimator.posim_ping_planes.apply(1,Vec3(.02,.01,.2));
    };
    const auto initial=capture();bool rejected=false,nested=false;
    {
      posim_slam::JointPingTrial trial(estimator);mutate();rejected=!trial.accept(1.,2.,1.);
    }
    bool reject_restored=rejected && capture()==initial && !estimator.posim_ping_trial_active;
    try {
      posim_slam::JointPingTrial trial(estimator);mutate();throw std::runtime_error("injected_cost_failure");
    }catch(const std::runtime_error&){}
    bool exception_restored=capture()==initial && !estimator.posim_ping_trial_active;
    bool accepted=false;
    {
      posim_slam::JointPingTrial trial(estimator);
      try {posim_slam::JointPingTrial duplicate(estimator);}catch(const std::logic_error&){nested=true;}
      mutate();accepted=trial.accept(4.,.05,3.9);
    }
    const auto committed=capture();accepted=accepted && committed!=initial;
    bool invalid_costs=true;
    for(int kind=0;kind<3;++kind) {
      posim_slam::JointPingTrial trial(estimator);mutate();
      double before=2.,after=1.,prediction=1.;
      if(kind==0)after=std::numeric_limits<double>::quiet_NaN();
      if(kind==1)prediction=0.;
      if(kind==2)after=before;
      invalid_costs=invalid_costs && !trial.accept(before,after,prediction) && capture()==committed;
    }
    {posim_slam::JointPingTrial trial(estimator);mutate();}
    bool abandoned_restored=capture()==committed;
    basalt::BundleAdjustmentBase<double> prior_estimator;
    prior_estimator.frame_poses.emplace(100,basalt::PoseStateWithLin<double>(100,Sophus::SE3d(),true));
    basalt::MargLinData<double> prior;prior.order.total_size=6;prior.order.items=1;prior.order.abs_order_map[100]={0,6};
    prior.H=Eigen::MatrixXd::Identity(6,6);prior.b=Eigen::VectorXd::Constant(6,-10.);
    double before_cost,after_cost;prior_estimator.computeMargPriorError(prior,before_cost);
    bool negative_prior_cost_accepted=false;
    {
      posim_slam::JointPingTrial trial(prior_estimator);
      prior_estimator.frame_poses.at(100).applyInc(Eigen::Matrix<double,6,1>::Constant(.01));
      prior_estimator.computeMargPriorError(prior,after_cost);
      negative_prior_cost_accepted=after_cost<0 && trial.accept(before_cost,after_cost,before_cost-after_cost);
    }
    bool passed=negative_prior_cost_accepted && reject_restored && exception_restored && accepted && invalid_costs && nested && abandoned_restored;
    std::cout<<Json({{"passed",passed},{"rejected_trial_restored",reject_restored},
      {"exception_restored",exception_restored},{"accepted_trial_retained",accepted},
      {"invalid_costs_restored",invalid_costs},{"nested_trial_rejected",nested},
      {"abandoned_trial_restored",abandoned_restored},{"negative_basalt_prior_cost_accepted",negative_prior_cost_accepted},
      {"basalt_prior_cost_before",before_cost},{"basalt_prior_cost_after",after_cost}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"
#include "sonar_factors.hpp"
#include <memory>

namespace posim_slam {
struct PingOptimizationOptions {int iterations=6,trials=6;double initial_damping=1e-3;};
struct PingOptimizationReport {
  int accepted=0,rejected=0;double initial_cost=0,final_cost=0;
  std::string termination="iteration_limit";
};

inline PingOptimizationReport optimizePingGroups(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,const std::vector<JointPingGroupSpec>& groups,
    const basalt::MargLinData<double>& prior,const basalt::ImuLinData<double>& imu,
    const PingOptimizationOptions& options={},SonarFactorStore* sonar=nullptr) {
  requireStateOrder(order);
  if(owner.posim_ping_trial_active || options.iterations<1 || options.iterations>20 ||
     options.trials<1 || options.trials>10 || !std::isfinite(options.initial_damping) ||
     options.initial_damping<=0 || options.initial_damping>1e12)
    throw std::invalid_argument("invalid_ping_optimization_options");
  for(const auto& [stamp,block]:order.abs_order_map)
    if((block.second==6 && !owner.frame_poses.count(stamp)) ||
       (block.second==15 && !owner.frame_states.count(stamp)))throw std::invalid_argument("invalid_ping_optimization_state");
  PingLinearization::Options settings;settings.lb_options.huber_parameter=owner.huber_thresh;
  settings.lb_options.obs_std_dev=owner.obs_std_dev;
  std::unique_ptr<basalt::LinearizationBase<double,6>> linearization;
  if(groups.empty())linearization=std::make_unique<basalt::LinearizationAbsQR<double,6>>(&owner,order,settings,&prior,&imu);
  else linearization=std::make_unique<PingLinearization>(&owner,order,settings,groups,&prior,&imu);
  auto sonar_poses=[&] {
    SonarPoses result;
    for(const auto& [stamp,block]:order.abs_order_map) {
      const auto pose=owner.getPoseStateWithLin(stamp).getPose();
      result.emplace(stamp,SonarPose{pose.translation(),pose.rotationMatrix()});
    }
    return result;
  };
  PingOptimizationReport report;double damping=options.initial_damping;
  for(int iteration=0;iteration<options.iterations;++iteration) {
    bool accepted=false;
    for(int attempt=0;attempt<options.trials;++attempt) {
      // Rebuild after every rejection; trial-state rows cannot survive rollback.
      bool valid=false;double before=linearization->linearizeProblem(&valid);
      std::optional<SonarFactorStore::Linearization> sonar_rows;
      Eigen::MatrixXd sonar_to_state;
      if(sonar && sonar->size()) {
        sonar_rows=sonar->linearize(sonar_poses());
        sonar_to_state=Eigen::MatrixXd::Zero(6*sonar_rows->pose_stamps.size(),order.total_size);
        for(size_t i=0;i<sonar_rows->pose_stamps.size();++i)
          sonar_to_state.block<6,6>(6*i,order.abs_order_map.at(sonar_rows->pose_stamps[i]).first).setIdentity();
        for(const auto& [id,rows]:sonar_rows->rows)before+=.5*rows.residual.squaredNorm();
      }
      if(!valid)throw std::runtime_error("invalid_ping_initial_linearization");
      if(iteration==0 && attempt==0)report.initial_cost=report.final_cost=before;
      linearization->performQR();Eigen::MatrixXd h;Eigen::VectorXd b;
      linearization->get_dense_H_b(h,b);
      if(sonar_rows)for(const auto& [id,rows]:sonar_rows->reduced) {
        const Eigen::MatrixXd jacobian=rows.pose_jacobian*sonar_to_state;
        h.noalias()+=jacobian.transpose()*jacobian;
        b.noalias()+=jacobian.transpose()*rows.residual;
      }
      h.diagonal().array()+=(h.diagonal().array().abs()*damping).max(1e-9);
      Eigen::LDLT<Eigen::MatrixXd> solve(h);const Eigen::VectorXd increment=solve.solve(-b);
      if(solve.info()!=Eigen::Success || !increment.allFinite()) {
        ++report.rejected;damping=std::min(1e12,damping*10.);continue;
      }
      Eigen::VectorXd sonar_increment;
      double point_step=0.;
      if(sonar_rows) {
        sonar_increment=sonar_to_state*increment;
        for(const auto& [id,rows]:sonar_rows->reduced)
          point_step=std::max(point_step,rows.backSubstitute(sonar_increment).lpNorm<Eigen::Infinity>());
      }
      if(increment.lpNorm<Eigen::Infinity>()<1e-9 && point_step<1e-9) {report.termination="small_step";return report;}
      double after=before;
      {
        JointPingTrial trial(owner);
        std::unique_ptr<SonarFactorStore::Trial> sonar_trial;
        if(sonar_rows)sonar_trial=std::make_unique<SonarFactorStore::Trial>(*sonar);
        try {
          double predicted=linearization->backSubstitute(increment);
          if(sonar_rows) {
            for(const auto& [id,rows]:sonar_rows->rows) {
              const Eigen::Vector3d point_delta=sonar_rows->reduced.at(id).backSubstitute(sonar_increment);
              const Eigen::VectorXd residual=rows.residual+rows.pose_jacobian*sonar_increment+rows.point_jacobian*point_delta;
              predicted+=.5*(rows.residual.squaredNorm()-residual.squaredNorm());
            }
            sonar->apply(*sonar_rows,sonar_increment);
          }
          for(const auto& [stamp,block]:order.abs_order_map) {
            if(block.second==6)owner.frame_poses.at(stamp).applyInc(increment.segment<6>(block.first));
            else owner.frame_states.at(stamp).applyInc(increment.segment<15>(block.first));
          }
          after=linearization->linearizeProblem(&valid);
          if(sonar_rows)after+=sonar->cost(sonar_poses());
          accepted=trial.accept(before,valid ? after : std::numeric_limits<double>::infinity(),predicted);
          if(sonar_trial)sonar_trial->finish(accepted);
        }catch(const std::invalid_argument&) {
          // A trial can move the beam outside supported geometry. RAII restores
          // every state/landmark/plane before another damping attempt.
        }
      }
      if(accepted) {
        ++report.accepted;report.final_cost=after;damping=std::max(1e-12,damping/3.);break;
      }
      ++report.rejected;damping=std::min(1e12,damping*10.);
    }
    if(!accepted){report.termination="no_accepted_step";return report;}
  }
  return report;
}
}

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "ping_marginal.hpp"
#include "sonar_factors.hpp"
#include "dvl_factors.hpp"
#include "pressure_factors.hpp"
#include "surface_owner.hpp"
#include <basalt/utils/ba_utils.h>
#include <algorithm>
#include <memory>
#include <cstdlib>
#include <iostream>

namespace posim_slam {
// Shared measurement assembly. Optimization adds its numerical damping only
// after this function; map information uses these undamped observations.
inline void addAuxiliaryNormal(Eigen::MatrixXd& h,Eigen::VectorXd& b,
    const std::optional<DvlFactorStore::Rows>& dvl,
    const std::optional<SonarFactorStore::Linearization>& sonar,
    const Eigen::MatrixXd& sonar_to_state,
    const std::optional<PressureFactorStore::Rows>& pressure) {
  if(dvl)for(size_t i=0;i<dvl->offsets.size();++i) {
    const int offset=dvl->offsets[i];
    const auto j=dvl->jacobian.block<3,15>(3*i,offset);
    h.block<15,15>(offset,offset).noalias()+=j.transpose()*j;
    b.segment<15>(offset).noalias()+=j.transpose()*dvl->residual.segment<3>(3*i);
  }
  if(sonar)for(const auto& [id,rows]:sonar->reduced) {
    const Eigen::MatrixXd jacobian=rows.pose_jacobian*sonar_to_state;
    h.noalias()+=jacobian.transpose()*jacobian;
    b.noalias()+=jacobian.transpose()*rows.residual;
  }
  if(pressure) {
    const auto n=h.cols();h.conservativeResize(n+1,n+1);b.conservativeResize(n+1);
    h.row(n).setZero();h.col(n).setZero();b[n]=0;
    h.noalias()+=pressure->jacobian.transpose()*pressure->jacobian;
    b.noalias()+=pressure->jacobian.transpose()*pressure->residual;
  }
}
using VisualProjectionSupport=std::set<std::pair<basalt::KeypointId,basalt::TimeCamId>>;

// A trial must retain every visual residual that contributed to its baseline
// cost. Basalt otherwise ignores invalid projections without a numerical error.
inline VisualProjectionSupport visualProjectionSupport(
    const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order) {
  VisualProjectionSupport support;
  for(const auto& [host,targets]:owner.lmdb.getObservations()) {
    if(!order.abs_order_map.count(host.frame_id))continue;
    for(const auto& [target,ids]:targets) {
      if(!order.abs_order_map.count(target.frame_id))continue;
      const Eigen::Matrix4d transform=basalt::computeRelPose(
        owner.getPoseStateWithLin(host.frame_id).getPose(),owner.calib.T_i_c.at(host.cam_id),
        owner.getPoseStateWithLin(target.frame_id).getPose(),owner.calib.T_i_c.at(target.cam_id)).matrix();
      std::visit([&,observation_ids=ids,observation_target=target](const auto& camera) {
        for(const auto id:observation_ids) {
          const auto& point=owner.lmdb.getLandmark(id);Eigen::Vector2d residual;
          if(basalt::linearizePoint(point.obs.at(observation_target),point,transform,camera,residual))
            support.emplace(id,observation_target);
        }
      },owner.calib.intrinsics.at(target.cam_id).variant);
    }
  }
  return support;
}

inline bool preservesVisualProjectionSupport(const VisualProjectionSupport& before,
                                             const VisualProjectionSupport& after) {
  return std::includes(after.begin(),after.end(),before.begin(),before.end());
}

struct PingOptimizationOptions {int iterations=6,trials=6;double initial_damping=1e-3;};
struct PingOptimizationReport {
  int accepted=0,rejected=0;double initial_cost=0,final_cost=0;
  int projection_rejections=0;
  std::string termination="iteration_limit";
};

inline PingOptimizationReport optimizePingGroups(basalt::BundleAdjustmentBase<double>& owner,
    const basalt::AbsOrderMap& order,const std::vector<JointPingGroupSpec>& groups,
    const basalt::MargLinData<double>& prior,const basalt::ImuLinData<double>& imu,
    const PingOptimizationOptions& options={},SonarFactorStore* sonar=nullptr,DvlFactorStore* dvl=nullptr,
    PressureFactorStore* pressure=nullptr,SurfaceMarginalOwner* surface=nullptr) {
  requireStateOrder(order);
  if(surface) {
    if(!pressure || !surface->active() || !groups.empty() || !owner.lmdb.getLandmarks().empty() || (sonar && sonar->size()))
      throw std::invalid_argument("exclusive_camera_free_surface_optimization_required");
    SurfaceMarginalOwner::requireZero(prior,*pressure);
  }
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
  auto sonar_poses=[&](bool reference=false) {
    SonarPoses result;
    for(const auto& [stamp,block]:order.abs_order_map) {
      const auto state=owner.getPoseStateWithLin(stamp);
      const auto pose=reference ? state.getPoseLin() : state.getPose();
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
        const auto references=sonar_poses(true);
        sonar_rows=sonar->linearize(sonar_poses(),&references);
        for(const auto& [id,rows]:sonar_rows->rows)
          sonar_rows->reduced.at(id)=eliminateSonarPoint(rows,1e-10,damping);
        sonar_to_state=Eigen::MatrixXd::Zero(6*sonar_rows->pose_stamps.size(),order.total_size);
        for(size_t i=0;i<sonar_rows->pose_stamps.size();++i)
          sonar_to_state.block<6,6>(6*i,order.abs_order_map.at(sonar_rows->pose_stamps[i]).first).setIdentity();
        before+=sonar->cost(sonar_poses());
      }
      if(!valid)throw std::runtime_error("invalid_ping_initial_linearization");
      std::optional<DvlFactorStore::Rows> dvl_rows;
      if(dvl) {dvl_rows=dvl->linearize(owner,order);before+=dvl_rows->cost;}
      std::optional<PressureFactorStore::Rows> pressure_rows;
      if(pressure) {pressure_rows=pressure->linearize(owner,order);before+=pressure_rows->cost;}
      std::optional<surface_prior::Rows> surface_rows;
      if(surface) {surface_rows=surface->linearize(owner,order,*pressure);before+=.5*surface_rows->residual.squaredNorm();}
      const auto visual_support=visualProjectionSupport(owner,order);
      if(iteration==0 && attempt==0)report.initial_cost=report.final_cost=before;
      if(groups.empty())linearization->performQR();
      else static_cast<PingLinearization&>(*linearization).performQR(damping);
      Eigen::MatrixXd h;Eigen::VectorXd b;
      linearization->get_dense_H_b(h,b);
      addAuxiliaryNormal(h,b,dvl_rows,sonar_rows,sonar_to_state,pressure_rows);
      if(surface_rows) {
        const auto old=h.cols(),count=surface_rows->jacobian.cols();
        h.conservativeResize(count,count);b.conservativeResize(count);
        h.rightCols(count-old).setZero();h.bottomRows(count-old).setZero();b.tail(count-old).setZero();
        h.noalias()+=surface_rows->jacobian.transpose()*surface_rows->jacobian;
        b.noalias()+=surface_rows->jacobian.transpose()*surface_rows->residual;
      }
      h.diagonal().array()+=(h.diagonal().array().abs()*damping).max(1e-9);
      Eigen::LDLT<Eigen::MatrixXd> solve(h);const Eigen::VectorXd full_increment=solve.solve(-b);
      if(solve.info()!=Eigen::Success || !full_increment.allFinite()) {
        ++report.rejected;damping=std::min(1e12,damping*10.);continue;
      }
      const Eigen::VectorXd increment=full_increment.head(order.total_size);
      Eigen::VectorXd sonar_increment;
      double point_step=0.;
      if(sonar_rows) {
        sonar_increment=sonar_to_state*increment;
        for(const auto& [id,rows]:sonar_rows->reduced)
          point_step=std::max(point_step,rows.backSubstitute(sonar_increment).lpNorm<Eigen::Infinity>());
      }
      if(full_increment.lpNorm<Eigen::Infinity>()<1e-9 && point_step<1e-9) {report.termination="small_step";return report;}
      double after=before;
      {
        JointPingTrial trial(owner);
        std::unique_ptr<SonarFactorStore::Trial> sonar_trial;
        if(sonar_rows)sonar_trial=std::make_unique<SonarFactorStore::Trial>(*sonar);
        std::unique_ptr<PressureFactorStore::Trial> pressure_trial;
        if(pressure)pressure_trial=std::make_unique<PressureFactorStore::Trial>(*pressure);
        try {
          double predicted=linearization->backSubstitute(increment);
          if(pressure_rows) {
            predicted+=.5*(pressure_rows->residual.squaredNorm()-
              (pressure_rows->residual+pressure_rows->jacobian*full_increment.head(pressure_rows->jacobian.cols())).squaredNorm());
            pressure_trial->apply(full_increment[order.total_size]);
          }
          if(surface_rows) {
            predicted+=.5*(surface_rows->residual.squaredNorm()-
              (surface_rows->residual+surface_rows->jacobian*full_increment).squaredNorm());
            const auto coordinates=surface->coordinates(owner,order,*pressure);Eigen::Index offset=0;
            for(const auto& v:coordinates.order) {
              if(v.kind==surface_prior::Kind::Plane)owner.posim_ping_planes.apply(int(v.id),full_increment.segment<3>(offset));
              offset+=v.size;
            }
          }
          if(dvl_rows)predicted+=.5*(dvl_rows->residual.squaredNorm()-
              (dvl_rows->residual+dvl_rows->jacobian*increment).squaredNorm());
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
          if(dvl)after+=dvl->linearize(owner,order).cost;
          if(pressure)after+=pressure->linearize(owner,order).cost;
          if(surface)after+=.5*surface->linearize(owner,order,*pressure).residual.squaredNorm();
          if(!preservesVisualProjectionSupport(visual_support,visualProjectionSupport(owner,order))) {
            valid=false;++report.projection_rejections;
          }
          accepted=trial.accept(before,valid ? after : std::numeric_limits<double>::infinity(),predicted);
          if(sonar_trial)sonar_trial->finish(accepted);
          if(pressure_trial)pressure_trial->finish(accepted);
        }catch(const std::invalid_argument& error) {
          if(std::getenv("POSIM_SLAM_TRACE_JOINT"))std::cerr
            << "POSIM_JOINT rejected_geometry " << error.what()
            << " point_step " << point_step << " damping " << damping << std::endl;
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

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "map_information.hpp"
#include "ping_optimizer.hpp"

namespace posim_slam {

// Native optimization coordinates, including an already-linearized host pose.
// Copying the host avoids changing any state/linearization point while querying
// uncertainty. Central differences include its current SO(3) delta convention.
inline std::pair<Eigen::Matrix<double,3,6>,Eigen::Matrix3d> mapWorldDerivatives(
    const basalt::BundleAdjustmentBase<double>& owner,basalt::KeypointId id) {
  const auto& point=owner.lmdb.getLandmark(id);
  if(!std::isfinite(point.inv_dist) || point.inv_dist<=0)
    throw std::invalid_argument("map_point_at_infinity");
  const auto host=owner.getPoseStateWithLin(point.host_kf_id.frame_id);
  const auto& mount=owner.calib.T_i_c.at(point.host_kf_id.cam_id);
  const Eigen::Vector3d parameters(point.direction[0],point.direction[1],point.inv_dist);
  const auto world=[&](const Eigen::Vector3d& local,const Sophus::SE3d& pose) {
    Eigen::Vector4d ray=basalt::StereographicParam<double>::unproject(local.head<2>());
    ray[3]=local[2];const Eigen::Vector4d value=(pose*mount).matrix()*ray;
    return Eigen::Vector3d(value.head<3>()/value[3]);
  };
  Eigen::Matrix<double,3,6> pose_jacobian;Eigen::Matrix3d local_jacobian;
  for(int k=0;k<6;++k) {
    Eigen::Matrix<double,6,1> delta=Eigen::Matrix<double,6,1>::Zero();delta[k]=1e-6;
    auto plus=host,minus=host;plus.applyInc(delta);minus.applyInc(-delta);
    pose_jacobian.col(k)=(world(parameters,plus.getPose())-world(parameters,minus.getPose()))/(2e-6);
  }
  for(int k=0;k<3;++k) {
    const double step=k==2 ? std::min(1e-6,point.inv_dist*.01) : 1e-6;
    auto plus=parameters,minus=parameters;plus[k]+=step;minus[k]-=step;
    local_jacobian.col(k)=(world(plus,host.getPose())-world(minus,host.getPose()))/(2*step);
  }
  if(!pose_jacobian.allFinite() || !local_jacobian.allFinite())
    throw std::runtime_error("invalid_map_world_jacobian");
  return {pose_jacobian,local_jacobian};
}

inline std::map<int,MapLandmarkInformation> jointMapInformation(
    basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
    const std::vector<JointPingGroupSpec>& groups,const basalt::MargLinData<double>& prior,
    const basalt::ImuLinData<double>& imu,SonarFactorStore* sonar=nullptr,
    DvlFactorStore* dvl=nullptr,PressureFactorStore* pressure=nullptr) {
  requireStateOrder(order);
  // This development query is bounded independently of the estimator window.
  if(order.total_size>384)throw std::runtime_error("map_information_column_budget");
  PingLinearization::Options settings;settings.lb_options.huber_parameter=owner.huber_thresh;
  settings.lb_options.obs_std_dev=owner.obs_std_dev;
  PingLinearization rows(&owner,order,settings,groups,&prior,&imu);
  bool valid=false;rows.linearizeProblem(&valid);
  if(!valid)throw std::runtime_error("invalid_map_information_linearization");
  struct Block {std::vector<basalt::KeypointId> ids;int planes;MapNuisanceInformation local;};
  std::vector<Block> blocks;
  rows.visitMapRows([&](const auto& ids,int planes,const auto& pose,const auto& local) {
    blocks.push_back({ids,planes,mapNuisanceInformation(pose,local)});
  });
  // No LM/trial damping, prior replacement or state update is performed here.
  rows.performQR();Eigen::MatrixXd h;Eigen::VectorXd b;rows.get_dense_H_b(h,b);
  std::optional<SonarFactorStore::Linearization> sonar_rows;
  Eigen::MatrixXd sonar_to_state;
  if(sonar && sonar->size()) {
    SonarPoses poses,references;
    for(const auto& [stamp,block]:order.abs_order_map) {
      const auto state=owner.getPoseStateWithLin(stamp);
      poses.emplace(stamp,SonarPose{state.getPose().translation(),state.getPose().rotationMatrix()});
      references.emplace(stamp,SonarPose{state.getPoseLin().translation(),state.getPoseLin().rotationMatrix()});
    }
    sonar_rows=sonar->linearize(poses,&references);
    sonar_to_state=Eigen::MatrixXd::Zero(6*sonar_rows->pose_stamps.size(),order.total_size);
    for(size_t i=0;i<sonar_rows->pose_stamps.size();++i)
      sonar_to_state.block<6,6>(6*i,order.abs_order_map.at(sonar_rows->pose_stamps[i]).first).setIdentity();
  }
  std::optional<DvlFactorStore::Rows> dvl_rows;
  if(dvl)dvl_rows=dvl->linearize(owner,order);
  std::optional<PressureFactorStore::Rows> pressure_rows;
  if(pressure)pressure_rows=pressure->linearize(owner,order);
  addAuxiliaryNormal(h,b,dvl_rows,sonar_rows,sonar_to_state,pressure_rows);
  const auto pose_information=mapPoseInformation(h);
  std::map<int,MapLandmarkInformation> result;
  const MapPoseInformation fixed_pose{Eigen::MatrixXd::Zero(h.cols(),0),
      Eigen::MatrixXd::Zero(h.cols(),0)};
  for(auto& block:blocks) {
    // The optional pressure datum is a retained column without direct visual
    // sensitivity; its correlations still enter the joint pose covariance.
    const auto previous=block.local.backsub.cols();
    block.local.backsub.conservativeResize(Eigen::NoChange,h.cols());
    if(h.cols()>previous)block.local.backsub.rightCols(h.cols()-previous).setZero();
    for(size_t i=0;i<block.ids.size();++i) {
      const auto id=block.ids[i];result.emplace(id,MapLandmarkInformation{});
      if(owner.lmdb.getLandmark(id).inv_dist<=0)continue;
      const auto derivatives=mapWorldDerivatives(owner,id);
      Eigen::MatrixXd world_pose=Eigen::MatrixXd::Zero(3,h.cols());
      const auto stamp=owner.lmdb.getLandmark(id).host_kf_id.frame_id;
      world_pose.middleCols<6>(order.abs_order_map.at(stamp).first)=derivatives.first;
      Eigen::MatrixXd world_local=Eigen::MatrixXd::Zero(3,block.local.backsub.rows());
      world_local.middleCols<3>(3*block.planes+3*i)=derivatives.second;
      auto& information=result.at(id);
      information.world=mapPointInformation(pose_information,block.local,world_pose,world_local);
      const auto& point=owner.lmdb.getLandmark(id);
      const auto camera=owner.getPoseStateWithLin(stamp).getPose()*owner.calib.T_i_c.at(point.host_kf_id.cam_id);
      const Eigen::MatrixXd host_local=camera.rotationMatrix().transpose()*world_local;
      const Eigen::MatrixXd no_direct_pose=Eigen::MatrixXd::Zero(3,h.cols());
      // The point parameters live in this host camera. The host pose cancels,
      // but relative-pose uncertainty still enters through nuisance backsub.
      information.host_camera=mapPointInformation(pose_information,block.local,no_direct_pose,host_local);
      information.conditional_host_camera=mapPointInformation(fixed_pose,block.local,no_direct_pose,host_local);
    }
  }
  return result;
}
}  // namespace posim_slam

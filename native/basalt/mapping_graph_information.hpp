// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_information.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <numeric>

namespace posim_slam {
template<class Consumer>
inline void mappingGraphRows(basalt::NfrMapper& mapper,
    const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
    int64_t reference_stamp,Consumer& graph) {
  const auto& poses=mapper.getFramePoses();
  std::map<int64_t,int> offsets;
  for(const auto& [stamp,pose]:poses)offsets.emplace(stamp,6*offsets.size());
  auto columns=[&](int64_t stamp) {
    std::vector<int> cols(6);std::iota(cols.begin(),cols.end(),offsets.at(stamp));return cols;
  };
  for(const auto& factor:mapper.rel_pose_factors) {
    Sophus::Matrix6d first,second;
    basalt::relPoseError(factor.T_i_j,poses.at(factor.t_i_ns).getPose(),
                        poses.at(factor.t_j_ns).getPose(),&first,&second);
    Eigen::MatrixXd rows(6,12);rows<<first,second;
    Eigen::LLT<Eigen::MatrixXd> weight(factor.cov_inv);
    if(weight.info()!=Eigen::Success)throw std::runtime_error("mapping_relative_information_not_positive");
    auto cols=columns(factor.t_i_ns);const auto target=columns(factor.t_j_ns);
    cols.insert(cols.end(),target.begin(),target.end());
    graph.addPoseFactor("relative:"+std::to_string(factor.t_i_ns)+":"+std::to_string(factor.t_j_ns),
                        cols,weight.matrixU()*rows);
  }
  for(const auto& factor:mapper.roll_pitch_factors) {
    Eigen::Matrix<double,2,6> rows;
    basalt::rollPitchError(poses.at(factor.t_ns).getPose(),factor.R_w_i_meas,&rows);
    Eigen::LLT<Eigen::MatrixXd> weight(factor.cov_inv);
    if(weight.info()!=Eigen::Success)throw std::runtime_error("mapping_roll_pitch_information_not_positive");
    graph.addPoseFactor("roll-pitch:"+std::to_string(factor.t_ns),
                        columns(factor.t_ns),weight.matrixU()*rows);
  }
  for(const auto& [id,landmark]:mapper.lmdb.getLandmarks()) {
    const auto host=poses.at(landmark.host_kf_id.frame_id).getPose()*calibration.T_i_c.at(landmark.host_kf_id.cam_id);
    auto local=basalt::StereographicParam<double>::unproject(landmark.direction);local[3]=landmark.inv_dist;
    const auto homogeneous=(host*local).eval();
    const Eigen::Vector3d world=homogeneous.head<3>()/homogeneous[3];
    if(!world.allFinite() || landmark.inv_dist<=0)continue;
    struct Row {
      MappingObservation key;
      Eigen::Matrix<double,2,6> pose;
      Eigen::Matrix<double,2,3> point;
      bool full;
    };
    std::vector<Row> rows;
    for(const auto& [image,pixel]:landmark.obs) {
      const auto pose=poses.at(image.frame_id).getPose();
      const auto inverse=(pose*calibration.T_i_c.at(image.cam_id)).inverse();
      Eigen::Vector4d camera;camera.head<3>()=inverse*world;camera[3]=1;
      Eigen::Vector2d projected;Eigen::Matrix<double,2,4> derivative;
      if(!calibration.intrinsics.at(image.cam_id).project(camera,projected,&derivative) ||
          !projected.allFinite() || !derivative.allFinite() || camera.z()<=0)continue;
      const double error=(projected-pixel).norm();
      const double weight=std::sqrt(error<mapper.huber_thresh?1:mapper.huber_thresh/error)
                           /mapper.obs_std_dev;
      Row row;row.key={image.frame_id,image.cam_id,mapper.feature_tracks.at(id).at(image)};
      row.point=derivative.leftCols<3>()*inverse.rotationMatrix();
      row.pose.template leftCols<3>()=-row.point;
      row.pose.template rightCols<3>()=row.point*Sophus::SO3d::hat(world-pose.translation());
      row.point*=weight;row.pose*=weight;row.full=error<=mapper.huber_thresh;
      rows.push_back(row);
    }
    if(rows.size()<2)continue;
    MappingPointRows point;point.id=id;point.world=world;point.host_origin=host.translation();
    point.pose=Eigen::MatrixXd::Zero(2*rows.size(),6*rows.size());point.point.resize(2*rows.size(),3);
    for(size_t i=0;i<rows.size();++i) {
      const auto cols=columns(std::get<0>(rows[i].key));
      point.pose_columns.insert(point.pose_columns.end(),cols.begin(),cols.end());
      point.observations.push_back(rows[i].key);point.full_weight.push_back(rows[i].full);
      point.pose.block<2,6>(2*i,6*i)=rows[i].pose;point.point.middleRows(2*i,2)=rows[i].point;
    }
    graph.addPoint(point);
  }
  // Reuse the estimator's original position/yaw reference once. No roll/pitch
  // prior, fixed pressure datum, LM damping or truth-derived reference is added.
  Eigen::MatrixXd reference=Eigen::MatrixXd::Zero(4,6);
  reference.topLeftCorner<3,3>().setIdentity();reference(3,5)=1;
  reference*=std::sqrt(config.vio_init_pose_weight);
  graph.addPoseFactor("initial-vio-world-reference",columns(reference_stamp),reference);
}
inline MappingInformationResult mappingGraphInformation(basalt::NfrMapper& mapper,
    const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
    int64_t reference_stamp,double three_sigma_m) {
  MappingInformation graph(6*mapper.getFramePoses().size());
  mappingGraphRows(mapper,calibration,config,reference_stamp,graph);
  return graph.evaluate(three_sigma_m);
}
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_information.hpp"
#include "submap_linearization.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <basalt/utils/ba_utils.h>

namespace posim_slam {
// Rows use Basalt's host stereographic direction/inverse-distance landmark
// coordinates and its world-translation/left-rotation pose increment. They are
// optimizer likelihoods, not a finite-world-point export or covariance gate.
struct NativeMappingPoseRows {
  std::string original;
  std::vector<int64_t> poses;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  double cost=0;
};
struct NativeMappingPointRows {
  int64_t id=-1;
  basalt::TimeCamId host;
  std::vector<int64_t> poses;
  std::vector<MappingObservation> originals;
  std::vector<bool> valid,full_weight;
  // Keep two row slots for every original, including invalid projections.
  // Invalid slots contain zeros; the original identities remain owned.
  Eigen::MatrixXd pose,landmark;
  Eigen::VectorXd residual;
  double cost=0;
};
struct NativeMappingRows {
  std::vector<int64_t> poses;
  std::vector<NativeMappingPoseRows> factors;
  std::vector<NativeMappingPointRows> points;
  double vision_cost=0,pose_cost=0;
  size_t originals=0,valid_projections=0;
};
inline NativeMappingRows nativeMappingRows(basalt::NfrMapper& mapper) {
  NativeMappingRows out;
  const auto& states=mapper.getFramePoses();
  if(states.empty() || 6*states.size()>size_t(kMaxJointColumns) ||
      mapper.lmdb.getLandmarks().size()>100000 ||
      mapper.rel_pose_factors.size()+mapper.roll_pitch_factors.size()>65536 ||
      !std::isfinite(mapper.obs_std_dev) || mapper.obs_std_dev<=0 ||
      !std::isfinite(mapper.huber_thresh) || mapper.huber_thresh<=0)
    throw std::invalid_argument("native_mapping_linearization_configuration");
  for(const auto& [stamp,state]:states) {
    if(stamp<=0 || !state.getPose().matrix().allFinite() || state.isLinearized())
      throw std::invalid_argument("native_mapping_current_tangent_required");
    out.poses.push_back(stamp);
  }
  std::set<std::string> factors;
  auto add_factor=[&](NativeMappingPoseRows rows,const Eigen::MatrixXd& information) {
    if(rows.original.empty() || !factors.insert(rows.original).second ||
        !information.allFinite() || !information.isApprox(information.transpose(),1e-12))
      throw std::invalid_argument("native_mapping_factor");
    Eigen::LLT<Eigen::MatrixXd> weight(information);
    if(weight.info()!=Eigen::Success)throw std::invalid_argument("native_mapping_factor_information");
    rows.jacobian=(weight.matrixU()*rows.jacobian).eval();
    rows.residual=(weight.matrixU()*rows.residual).eval();
    if(!rows.jacobian.allFinite() || !rows.residual.allFinite())
      throw std::invalid_argument("native_mapping_factor_nonfinite");
    rows.cost=rows.residual.squaredNorm();out.pose_cost+=rows.cost;
    out.factors.push_back(std::move(rows));
  };
  if(mapper.config.mapper_use_factors) {
    for(const auto& factor:mapper.rel_pose_factors) {
      Sophus::Matrix6d first,second;NativeMappingPoseRows rows;
      rows.original="relative:"+std::to_string(factor.t_i_ns)+":"+std::to_string(factor.t_j_ns);
      rows.poses={factor.t_i_ns,factor.t_j_ns};
      if(factor.t_i_ns==factor.t_j_ns)throw std::invalid_argument("native_mapping_relative_identity");
      rows.residual=basalt::relPoseError(factor.T_i_j,states.at(factor.t_i_ns).getPose(),
                                       states.at(factor.t_j_ns).getPose(),&first,&second);
      rows.jacobian.resize(6,12);rows.jacobian<<first,second;
      add_factor(std::move(rows),factor.cov_inv);
    }
    for(const auto& factor:mapper.roll_pitch_factors) {
      Eigen::Matrix<double,2,6> derivative;NativeMappingPoseRows rows;
      rows.original="roll-pitch:"+std::to_string(factor.t_ns);rows.poses={factor.t_ns};
      rows.residual=basalt::rollPitchError(states.at(factor.t_ns).getPose(),factor.R_w_i_meas,&derivative);
      rows.jacobian=derivative;add_factor(std::move(rows),factor.cov_inv);
    }
  }
  std::set<MappingObservation> owners;
  for(const auto& [id,point]:mapper.lmdb.getLandmarks()) {
    if(id<0 || point.obs.empty() || point.obs.size()>2048 ||
        out.originals+point.obs.size()>1000000 || !point.direction.allFinite() ||
        !std::isfinite(point.inv_dist) || point.inv_dist<0)
      throw std::invalid_argument("native_mapping_landmark");
    NativeMappingPointRows rows;rows.id=id;rows.host=point.host_kf_id;
    std::set<int64_t> stamps{rows.host.frame_id};
    for(const auto& [image,pixel]:point.obs)stamps.insert(image.frame_id);
    rows.poses.assign(stamps.begin(),stamps.end());
    if(6*rows.poses.size()+3>size_t(kMaxJointColumns))
      throw std::runtime_error("native_mapping_landmark_span");
    std::map<int64_t,int> offset;
    for(size_t i=0;i<rows.poses.size();++i) {
      if(!states.count(rows.poses[i]))throw std::invalid_argument("native_mapping_pose_missing");
      offset[rows.poses[i]]=int(6*i);
    }
    rows.pose=Eigen::MatrixXd::Zero(2*point.obs.size(),6*rows.poses.size());
    rows.landmark=Eigen::MatrixXd::Zero(2*point.obs.size(),3);
    rows.residual=Eigen::VectorXd::Zero(2*point.obs.size());
    size_t index=0;
    for(const auto& [image,pixel]:point.obs) {
      const auto feature=mapper.feature_tracks.at(id).at(image);
      const MappingObservation key{image.frame_id,image.cam_id,feature};
      if(image.cam_id<0 || feature<0 || !pixel.allFinite() || !owners.insert(key).second)
        throw std::invalid_argument("native_mapping_original_reused");
      rows.originals.push_back(key);
      Sophus::Matrix6d d_host=Sophus::Matrix6d::Zero(),d_target=Sophus::Matrix6d::Zero();
      Eigen::Matrix4d relative=Eigen::Matrix4d::Identity();
      if(rows.host!=image) {
        relative=basalt::computeRelPose(states.at(rows.host.frame_id).getPose(),
          mapper.calib.T_i_c.at(rows.host.cam_id),states.at(image.frame_id).getPose(),
          mapper.calib.T_i_c.at(image.cam_id),&d_host,&d_target).matrix();
      }
      Eigen::Vector2d residual;
      Eigen::Matrix<double,2,6> pose;
      Eigen::Matrix<double,2,3> landmark;
      const auto& observation=pixel;const auto& native_point=point;
      const bool valid=std::visit([&](const auto& camera) {
        return basalt::linearizePoint(observation,native_point,relative,camera,residual,&pose,&landmark);
      },mapper.calib.intrinsics.at(image.cam_id).variant);
      rows.valid.push_back(valid);
      bool full=false;
      if(valid) {
        if(!pose.allFinite() || !landmark.allFinite() || !residual.allFinite())
          throw std::invalid_argument("native_mapping_projection_nonfinite");
        const double error=residual.norm();
        const double huber=error<mapper.huber_thresh?1:mapper.huber_thresh/error;
        const double weight=std::sqrt(huber)/mapper.obs_std_dev;
        full=error<=mapper.huber_thresh;
        rows.residual.segment<2>(2*index)=weight*residual;
        rows.landmark.middleRows<2>(2*index)=weight*landmark;
        rows.pose.block<2,6>(2*index,offset.at(rows.host.frame_id))+=weight*pose*d_host;
        rows.pose.block<2,6>(2*index,offset.at(image.frame_id))+=weight*pose*d_target;
        rows.cost+=.5*(2-huber)*weight*weight*residual.squaredNorm();
        ++out.valid_projections;
      }
      rows.full_weight.push_back(full);++index;
    }
    out.originals+=rows.originals.size();out.vision_cost+=rows.cost;out.points.push_back(std::move(rows));
  }
  return out;
}
struct NativeMappingPointReduction {
  SubmapLinearization::Rows likelihood;
  Eigen::MatrixXd backsub;
  Eigen::VectorXd offset;
  Eigen::MatrixXd unbounded;
};
inline std::map<int64_t,int> nativeMappingPoseOffsets(const std::vector<int64_t>& poses) {
  std::map<int64_t,int> offsets;
  if(poses.empty() || 6*poses.size()>size_t(kMaxJointColumns))
    throw std::invalid_argument("native_mapping_pose_order");
  for(size_t i=0;i<poses.size();++i)
    if(poses[i]<=0 || !offsets.emplace(poses[i],int(6*i)).second)
      throw std::invalid_argument("native_mapping_pose_order");
  return offsets;
}
inline std::vector<int64_t> nativeMappingColumns(const std::vector<int64_t>& poses,
    const std::map<int64_t,int>& offsets) {
  std::vector<int64_t> columns;
  for(auto pose:poses)for(int i=0;i<6;++i)columns.push_back(offsets.at(pose)+i);
  return columns;
}
// The NFR graph contains relative pose and gravity information. Its common
// translation and world-yaw are gauge freedoms. Express a mean increment in
// the existing reference pose's position/yaw convention without adding an
// observation, damping or information to the covariance model. This helper is
// not for graphs with absolute-position/yaw measurements.
inline Eigen::VectorXd nativeMappingReferenceIncrement(basalt::NfrMapper& mapper,
    const std::vector<int64_t>& poses,const Eigen::VectorXd& increment,int64_t reference_stamp) {
  const auto offsets=nativeMappingPoseOffsets(poses);
  if(increment.size()!=6*poses.size() || !increment.allFinite())
    throw std::invalid_argument("native_mapping_increment");
  const auto& states=mapper.getFramePoses();
  const auto reference=states.at(reference_stamp).getPose();
  const int first=offsets.at(reference_stamp);
  const Eigen::Vector3d heading=reference.rotationMatrix().col(0);
  const double horizontal=heading.head<2>().squaredNorm();
  if(horizontal<=std::numeric_limits<double>::epsilon())
    throw std::invalid_argument("native_mapping_reference_yaw_undefined");
  const Eigen::Vector3d yaw_derivative(-heading.x()*heading.z()/horizontal,
                                     -heading.y()*heading.z()/horizontal,1);
  const double yaw=yaw_derivative.dot(increment.segment<3>(first+3));
  const Eigen::Vector3d translation=increment.segment<3>(first);
  Eigen::VectorXd result=increment;
  for(const auto& [stamp,index]:offsets) {
    result.segment<3>(index)-=translation+yaw*Eigen::Vector3d::UnitZ().cross(
      states.at(stamp).getPose().translation()-reference.translation());
    result[index+5]-=yaw;
  }
  return result;
}
inline SubmapLinearization::Rows nativeMappingFactor(const NativeMappingPoseRows& source,
    const std::map<int64_t,int>& offsets) {
  SubmapLinearization::Rows out;
  out.originals={source.original};out.columns=nativeMappingColumns(source.poses,offsets);
  out.jacobian=source.jacobian;out.residual=source.residual;return out;
}
inline NativeMappingPointReduction reduceNativeMappingPoint(const NativeMappingPointRows& source,
    const std::map<int64_t,int>& offsets) {
  const auto n=source.pose.cols();
  if(source.landmark.cols()!=3 || source.landmark.rows()!=source.pose.rows() ||
      source.residual.size()!=source.pose.rows() || source.pose.rows()!=2*source.originals.size())
    throw std::invalid_argument("native_mapping_point_dimensions");
  Eigen::MatrixXd joint(source.pose.rows(),n+3);joint<<source.pose,source.landmark;
  const auto reduced=splitJointInformation(joint,source.residual,n);
  NativeMappingPointReduction out;
  out.likelihood.columns=nativeMappingColumns(source.poses,offsets);
  for(const auto& [stamp,camera,feature]:source.originals)
    out.likelihood.originals.push_back("pixel:"+std::to_string(stamp)+":"+std::to_string(camera)+":"+std::to_string(feature));
  out.likelihood.jacobian=reduced.reduced.jacobian;out.likelihood.residual=reduced.reduced.residual;
  out.backsub=reduced.reduced.backsub_matrix;out.offset=reduced.reduced.backsub_offset;
  out.unbounded=reduced.unobservable_basis;
  return out;
}
} // namespace posim_slam


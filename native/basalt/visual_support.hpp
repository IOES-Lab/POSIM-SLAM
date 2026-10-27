// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "calibration.hpp"
#include "landmark_archive.hpp"
#include <limits>
#include <set>

inline std::vector<std::pair<int,double>> reprojection_residuals(
    const basalt::VioVisualizationData& data,int width,int height) {
  std::vector<std::pair<int,double>> result;
  if(!data.opt_flow_res || data.opt_flow_res->observations.empty() || data.projections.empty())return result;
  const auto& observations=data.opt_flow_res->observations[0];
  for(const auto& p:data.projections[0]) {
    if(!p.allFinite() || p[2]<=0 || p[3]<0 || p[3]>double(std::numeric_limits<int>::max()) ||
       p[3]!=std::floor(p[3]) || p[0]<0 || p[0]>=width || p[1]<0 || p[1]>=height)continue;
    auto found=observations.find(int(p[3]));if(found==observations.end())continue;
    const double residual=(p.head<2>()-found->second.translation().template cast<double>()).norm();
    if(std::isfinite(residual))result.emplace_back(int(p[3]),residual);
  }
  return result;
}

// Residual diagnostics do not establish tracking confidence or observability.
// Display-decimated point counts never decide estimator continuity.
inline Json visual_support(const basalt::VioVisualizationData& data,int width,int height,double huber_px) {
  const auto residuals=reprojection_residuals(data,width,height);
  size_t full_weight=0;double sum=0;
  for(const auto& item:residuals) {
    sum+=item.second*item.second;if(item.second<=huber_px)++full_weight;
  }
  return {{"reprojected_landmarks",residuals.size()},
          {"full_weight_reprojections",full_weight},{"huber_threshold_px",huber_px},
          {"reprojection_rmse_px",residuals.empty()?Json(nullptr):Json(std::sqrt(sum/residuals.size()))},
          {"visual_measurements_present",!residuals.empty()},
          {"observability_qualified",false}};
}

// Only full-weight current observations can mature archive points.
inline std::set<int> mapping_inliers(const basalt::VioVisualizationData& data,int width,int height,double huber_px) {
  std::set<int> result;
  for(const auto& item:reprojection_residuals(data,width,height))if(item.second<=huber_px)result.insert(item.first);
  return result;
}

// Both streaming and recorded exports use the same optimized-window geometry.
// An absent certificate is a protocol error, never permission to admit a point.
inline std::set<int> unqualified_visual_points(const basalt::VioVisualizationData& data) {
  std::set<int> result;
  for(int id:data.point_ids) {
    const auto found=data.map_quality.find(id);
    if(found==data.map_quality.end())throw std::runtime_error("visual_geometry_identity_missing");
    if(!found->second.eligible)result.insert(id);
  }
  return result;
}

// A missed feature is not counter-evidence. A received pixel for a retained ID
// is: test its stored coordinate even after its optimizer landmark has retired.
// Read immutable estimated poses from the visualization packet, never the owner.
template<class Calibration> std::set<int> archive_observation_outliers(
    const LandmarkArchive& archive,const basalt::VioVisualizationData& data,
    const Calibration& calibration,int width,int height,double huber_px) {
  if(!std::isfinite(huber_px) || huber_px<=0)throw std::invalid_argument("archive_reprojection_threshold");
  if(!data.opt_flow_res || data.opt_flow_res->observations.empty())return {};
  const auto& observations=data.opt_flow_res->observations[0];std::set<int> received;
  for(const auto& [id,observation]:observations) {
    if(id<0 || id>std::numeric_limits<int>::max())throw std::invalid_argument("archive_observation_id");
    if(!observation.translation().allFinite())throw std::invalid_argument("archive_observation_pixel");
    received.insert(int(id));
  }
  const auto matches=archive.observed_points(received);
  if(matches.empty())return {};
  if(data.states.empty())throw std::runtime_error("archive_observation_pose_missing");
  const auto camera=(data.states.back()*calibration.T_i_c.at(0)).inverse();
  std::set<int> rejected;
  for(const auto& [id,point]:matches) {
    const Eigen::Vector3d local=camera*Eigen::Vector3d(point[0],point[1],point[2]);
    Eigen::Vector4d ray;ray.head<3>()=local;ray[3]=1.;Eigen::Vector2d predicted;
    const auto pixel=observations.at(id).translation().template cast<double>();
    if(!calibration.intrinsics.at(0).project(ray,predicted) || !predicted.allFinite() ||
        predicted.x()<0 || predicted.x()>=width || predicted.y()<0 || predicted.y()>=height ||
        (predicted-pixel).norm()>huber_px)rejected.insert(id);
  }
  return rejected;
}

// Optional diagnostic output uses actual frontend IDs and optimized points.
// It never changes feature selection, weights, triangulation or map admission.
inline Json visual_track_trace(const basalt::VioVisualizationData& data,int width,int height,double huber_px,
                               const std::set<int>& archived) {
  Json rows=Json::array();
  if(!data.opt_flow_res || data.opt_flow_res->observations.empty())return rows;
  std::map<int,double> residuals;
  for(const auto& item:reprojection_residuals(data,width,height))residuals[item.first]=item.second;
  std::map<int,Json> points;
  if(data.points.size()!=data.point_ids.size())throw std::runtime_error("trace_point_identity_mismatch");
  for(size_t i=0;i<data.points.size();++i) {
    const auto& p=data.points[i];
    if(p.allFinite())points[data.point_ids[i]]={p.x(),p.y(),p.z()};
  }
  for(const auto& [id,observation]:data.opt_flow_res->observations[0]) {
    const auto pixel=observation.translation();
    if(!pixel.allFinite())continue;
    const auto residual=residuals.find(id);const auto point=points.find(id);
    rows.push_back({{"id",id},{"pixel",{pixel.x(),pixel.y()}},
      {"point",point==points.end()?Json(nullptr):point->second},
      {"reprojection_error_px",residual==residuals.end()?Json(nullptr):Json(residual->second)},
      {"full_weight",residual!=residuals.end() && residual->second<=huber_px},
      {"archived",archived.count(id)>0}});
    const auto quality=data.map_quality.find(id);
    if(quality!=data.map_quality.end() && quality->second.joint_information_evaluated) {
      rows.back()["joint_position_observed"]=quality->second.joint_position_observed;
      rows.back()["joint_position_sigma_m"]=std::isfinite(quality->second.joint_position_sigma_m) ?
          Json(quality->second.joint_position_sigma_m) : Json(nullptr);
      rows.back()["joint_host_position_observed"]=quality->second.joint_host_position_observed;
      rows.back()["joint_host_position_sigma_m"]=std::isfinite(quality->second.joint_host_position_sigma_m) ?
          Json(quality->second.joint_host_position_sigma_m) : Json(nullptr);
      rows.back()["conditional_host_position_sigma_m"]=std::isfinite(quality->second.conditional_host_position_sigma_m) ?
          Json(quality->second.conditional_host_position_sigma_m) : Json(nullptr);
    }
  }
  return rows;
}

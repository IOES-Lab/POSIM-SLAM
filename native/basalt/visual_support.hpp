// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "calibration.hpp"
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

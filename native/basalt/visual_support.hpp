// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "calibration.hpp"
#include <limits>

// These are residual diagnostics, not a tracking-confidence or observability
// test. Display-decimated point counts never decide estimator continuity.
inline Json visual_support(const basalt::VioVisualizationData& data,int width,int height,double huber_px) {
  std::vector<double> residuals;
  size_t full_weight=0;
  if(data.opt_flow_res && !data.opt_flow_res->observations.empty() && !data.projections.empty()) {
    const auto& observations=data.opt_flow_res->observations[0];
    for(const auto& projection:data.projections[0]) {
      if(!projection.allFinite() || projection[2]<=0 || projection[3]<0 ||
         projection[3]>double(std::numeric_limits<int>::max()) ||
         projection[3]!=std::floor(projection[3]) ||
         projection[0]<0 || projection[0]>=width || projection[1]<0 || projection[1]>=height)continue;
      auto observation=observations.find(int(projection[3]));
      if(observation==observations.end())continue;
      const auto pixel=observation->second.translation().template cast<double>();
      const double residual=(projection.head<2>()-pixel).norm();
      if(!std::isfinite(residual))continue;
      residuals.push_back(residual);
      if(residual<=huber_px)++full_weight;
    }
  }
  double sum=0;for(double residual:residuals)sum+=residual*residual;
  return {{"reprojected_landmarks",residuals.size()},
          {"full_weight_reprojections",full_weight},{"huber_threshold_px",huber_px},
          {"reprojection_rmse_px",residuals.empty()?Json(nullptr):Json(std::sqrt(sum/residuals.size()))},
          {"visual_measurements_present",!residuals.empty()},
          {"observability_qualified",false}};
}

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace posim_slam {
inline constexpr const char* kMappingAlignment="source_pinned_map_local_mean_v1";

// Both anchor poses belong to the same pinned mapping transaction. This is a
// mean-coordinate transform, never a new independent observation or covariance.
inline nlohmann::json mappingAlignment(const std::string& epoch,uint64_t revision,
    int64_t stamp,const Eigen::Matrix4d& mapped,const Eigen::Matrix4d& local) {
  const auto rigid=[](const Eigen::Matrix4d& t) {
    const Eigen::Matrix3d r=t.topLeftCorner<3,3>();
    return t.allFinite() && (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-10 &&
      (r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-8 && std::abs(r.determinant()-1)<1e-8;
  };
  if(epoch.empty() || !revision || revision>=(uint64_t(1)<<63) || stamp<=0 || !rigid(mapped) || !rigid(local))
    throw std::invalid_argument("mapping_alignment_source");
  Eigen::Matrix4d inverse=Eigen::Matrix4d::Identity();
  inverse.topLeftCorner<3,3>()=local.topLeftCorner<3,3>().transpose();
  inverse.topRightCorner<3,1>()=-inverse.topLeftCorner<3,3>()*local.topRightCorner<3,1>();
  const Eigen::Matrix4d transform=mapped*inverse;
  nlohmann::json rows=nlohmann::json::array();
  for(int row=0;row<4;++row)rows.push_back({transform(row,0),transform(row,1),transform(row,2),transform(row,3)});
  return {{"schema",kMappingAlignment},{"epoch",epoch},{"optimization_revision",revision},
    {"anchor_stamp_ns",stamp},{"map_from_local",rows},{"source","pinned_mapping_input_pose"},
    {"uncertainty_available",false},{"navigation_ready",false}};
}
} // namespace posim_slam

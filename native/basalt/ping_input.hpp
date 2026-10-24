// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_patch.hpp"
#include <nlohmann/json.hpp>

namespace posim_slam {
struct PingInputCalibration {
  Sophus::SE3d mount;Eigen::MatrixXd rays;double minimum,maximum;
};
inline PingInputCalibration pingInputCalibration(const nlohmann::json& input) {
  if(input.at("schema")!="posim-slam-ping-calibration-1")throw std::invalid_argument("ping_calibration_schema");
  Mat3 rotation;Vec3 translation;
  for(int i=0;i<3;++i) {
    translation[i]=input.at("imu_to_ping").at(i).get<double>();
    for(int k=0;k<3;++k)rotation(i,k)=input.at("ping_to_imu").at(i).at(k).get<double>();
  }
  if(!translation.allFinite() || !rotation.allFinite() ||
     (rotation.transpose()*rotation-Mat3::Identity()).norm()>1e-8 || std::abs(rotation.determinant()-1)>1e-8)
    throw std::invalid_argument("ping_calibration_mount");
  const auto& rays=input.at("rays");
  if(!rays.is_array() || rays.empty() || rays.size()>256)throw std::invalid_argument("ping_calibration_rays");
  PingInputCalibration out;out.mount=Sophus::SE3d(rotation,translation);out.rays.resize(rays.size(),3);
  for(Eigen::Index i=0;i<out.rays.rows();++i) {
    if(!rays.at(i).is_array() || rays.at(i).size()!=3)throw std::invalid_argument("ping_calibration_rays");
    for(int k=0;k<3;++k)out.rays(i,k)=rays.at(i).at(k).get<double>();
  }
  out.minimum=input.at("range_min");out.maximum=input.at("range_max");
  selectPingPatch({},Sophus::SE3d(),out.mount,out.rays,out.minimum,out.maximum);
  return out;
}

// Current raw tracks, independent of the display archive or viewport limit.
// Require the current camera observation and three distinct full-weight frames.
template<class Estimator> std::vector<PingPatchPoint> activePingPoints(
    const Estimator& owner,int64_t stamp,int width,int height) {
  if(width<=0 || height<=0 || !std::isfinite(owner.huber_thresh) || owner.huber_thresh<=0)
    throw std::invalid_argument("invalid_ping_visual_support");
  std::vector<PingPatchPoint> result;
  for(const auto& [id,point]:owner.lmdb.getLandmarks()) {
    if(!point.obs.count({stamp,0}) || point.obs.size()<3 || !std::isfinite(point.inv_dist) || point.inv_dist<=0)continue;
    const auto& host=point.host_kf_id;
    if(host.cam_id!=0)continue;
    const auto world=worldLandmark(owner.getPoseStateWithLin(host.frame_id).getPose(),
      owner.calib.T_i_c.at(0),point.direction,point.inv_dist).point;
    std::set<int64_t> supported;bool current=false;
    for(const auto& [target,observed]:point.obs) {
      if(target.cam_id!=0 || !observed.allFinite())continue;
      const auto camera=owner.getPoseStateWithLin(target.frame_id).getPose()*owner.calib.T_i_c.at(0);
      Eigen::Vector2d pixel;const Vec3 local=camera.inverse()*world;
      if(local.z()<=0 || !owner.calib.intrinsics.at(0).project(local,pixel) || !pixel.allFinite() ||
         pixel.x()<0 || pixel.y()<0 || pixel.x()>=width || pixel.y()>=height ||
         (pixel-observed).norm()>owner.huber_thresh)continue;
      supported.insert(target.frame_id);current=current || target.frame_id==stamp;
    }
    if(current && supported.size()>=3)result.push_back({id,world});
  }
  return result;
}
}

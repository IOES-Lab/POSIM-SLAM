// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_factors.hpp"
#include <nlohmann/json.hpp>

namespace posim_slam {
inline SonarCalibration sonarInputCalibration(const nlohmann::json& input) {
  if(input.at("schema")!="posim-slam-sonar-calibration-1")
    throw std::invalid_argument("sonar_calibration_schema");
  const auto& lever=input.at("imu_to_sonar");const auto& rotation=input.at("sonar_to_imu");
  if(!lever.is_array() || lever.size()!=3 || !rotation.is_array() || rotation.size()!=3)
    throw std::invalid_argument("sonar_calibration_shape");
  SonarCalibration out;
  for(int i=0;i<3;++i) {
    out.lever[i]=lever.at(i).get<double>();
    if(!rotation.at(i).is_array() || rotation.at(i).size()!=3)
      throw std::invalid_argument("sonar_calibration_shape");
    for(int j=0;j<3;++j)out.sonar_to_imu(i,j)=rotation.at(i).at(j).get<double>();
  }
  out.minimum=input.at("range_min");out.maximum=input.at("range_max");
  out.horizontal_fov=input.at("horizontal_fov");out.vertical_fov=input.at("vertical_fov");
  // Exercise the same rigid-transform/range/FOV contract as actual factors.
  const double range=.5*(out.minimum+out.maximum);
  sonarPointRows(Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),out.lever,out.sonar_to_imu,
    out.lever+out.sonar_to_imu*Eigen::Vector3d(range,0,0),range,0.,out.minimum,out.maximum,
    out.horizontal_fov,out.vertical_fov);
  return out;
}
}

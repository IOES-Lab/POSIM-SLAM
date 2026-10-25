// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "dvl_factors.hpp"
#include <nlohmann/json.hpp>
#include <cstring>

namespace posim_slam {
inline DvlCalibration dvlInputCalibration(const nlohmann::json& input) {
  if(input.at("schema")!="posim-slam-dvl-calibration-1")throw std::invalid_argument("dvl_calibration_schema");
  DvlCalibration c;
  const auto rotation=input.at("dvl_to_imu"),lever=input.at("imu_to_dvl");
  if(!rotation.is_array() || rotation.size()!=3 || !lever.is_array() || lever.size()!=3)
    throw std::invalid_argument("dvl_calibration_shape");
  for(int i=0;i<3;++i) {
    if(!rotation.at(i).is_array() || rotation.at(i).size()!=3)throw std::invalid_argument("dvl_calibration_shape");
    for(int j=0;j<3;++j)c.dvl_to_imu(i,j)=rotation.at(i).at(j);
    c.imu_to_dvl[i]=lever.at(i);
  }
  validateDvlCalibration(c);return c;
}
inline DvlObservation decodeDvlObservation(const std::vector<char>& bytes,const DvlCalibration& c) {
  if(bytes.size()!=216)throw std::invalid_argument("dvl_packet_size");
  size_t offset=0;
  auto integer=[&](size_t width) {uint64_t value=0;for(size_t i=0;i<width;++i)value|=uint64_t(uint8_t(bytes[offset++]))<<(8*i);return value;};
  auto number=[&] {const auto bits=integer(8);double value;std::memcpy(&value,&bits,8);return value;};
  const auto stamp=integer(8),gyro_stamp=integer(8),locked=integer(4),beams=integer(4);
  if(!stamp || stamp>uint64_t(INT64_MAX) || gyro_stamp!=stamp || locked!=1 || (beams!=3 && beams!=4))
    throw std::invalid_argument("dvl_clock_or_bottom_lock");
  DvlObservation result;result.stamp=int64_t(stamp);
  for(int i=0;i<3;++i)result.velocity[i]=number();
  for(int i=0;i<3;++i)result.gyro[i]=number();
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)result.covariance(i,j)=number();
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)result.gyro_covariance(i,j)=number();
  dvlWhitening(c,result);return result;
}
}

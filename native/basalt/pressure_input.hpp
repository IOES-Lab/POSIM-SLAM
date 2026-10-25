// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "pressure_factors.hpp"
#include <nlohmann/json.hpp>
#include <cstring>
namespace posim_slam {
inline PressureCalibration pressureInputCalibration(const nlohmann::json& input) {
  if(input.at("schema")!="posim-slam-pressure-calibration-1")throw std::invalid_argument("pressure_calibration_schema");
  const auto lever=input.at("imu_to_pressure");
  if(!lever.is_array() || lever.size()!=3)throw std::invalid_argument("pressure_calibration_shape");
  PressureCalibration c;for(int i=0;i<3;++i)c.imu_to_pressure[i]=lever.at(i);
  validatePressure(c,{1,0,1});return c;
}
inline PressureObservation decodePressureObservation(const std::vector<char>& bytes,const PressureCalibration& c) {
  if(bytes.size()!=24)throw std::invalid_argument("pressure_packet_size");
  size_t offset=0;
  auto integer=[&] {uint64_t value=0;for(int i=0;i<8;++i)value|=uint64_t(uint8_t(bytes[offset++]))<<(8*i);return value;};
  auto number=[&] {const auto bits=integer();double value;std::memcpy(&value,&bits,8);return value;};
  const auto stamp=integer();if(!stamp || stamp>uint64_t(INT64_MAX))throw std::invalid_argument("pressure_clock");
  PressureObservation o{int64_t(stamp),number(),number()};validatePressure(c,o);return o;
}
}

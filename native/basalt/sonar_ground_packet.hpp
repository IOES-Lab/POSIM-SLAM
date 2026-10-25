// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_ground_group.hpp"
#include "sonar_geometry.hpp"
#include <cstring>
#include <climits>
namespace posim_slam {
struct GroundPacket {GroundEntryObservation observation;Vec3 normal;double distance;};
inline GroundPacket decodeGroundPacket(const std::vector<char>& bytes,const GroundEntryCalibration& c,double horizontal_fov) {
  if(bytes.size()<48 || bytes.size()>2608)throw std::invalid_argument("ground_packet_size");
  size_t offset=0;
  auto integer=[&](size_t width){uint64_t x=0;for(size_t i=0;i<width;++i)x|=uint64_t(uint8_t(bytes[offset++]))<<(8*i);return x;};
  auto number=[&]{const auto bits=integer(8);double x;std::memcpy(&x,&bits,8);return x;};
  const auto stamp=integer(8),count=integer(4),sign=integer(4);
  if(!stamp || stamp>uint64_t(INT64_MAX) || count<8 || count>128 || bytes.size()!=48+20*count ||
     (sign!=1 && sign!=UINT32_MAX))throw std::invalid_argument("ground_packet_header");
  GroundPacket out;out.observation.stamp=int64_t(stamp);out.observation.boundary_sign=sign==1?1:-1;
  for(int i=0;i<3;++i)out.normal[i]=number();out.distance=number();
  if(!out.normal.allFinite() || std::abs(out.normal.squaredNorm()-1)>1e-8 || !std::isfinite(out.distance))
    throw std::invalid_argument("ground_packet_seed");
  auto& o=out.observation;o.ranges.resize(count);o.azimuth.resize(count);std::set<int> ids;
  for(size_t i=0;i<count;++i) {
    const auto id=integer(4);const double az=number(),range=number();
    if(id>uint64_t(INT_MAX) || !ids.insert(int(id)).second || !sonarWithinFov(az,horizontal_fov) ||
       (i && az<=o.azimuth[i-1]) || !std::isfinite(range) || range<c.minimum || range>c.maximum)
      throw std::invalid_argument("ground_packet_beam");
    o.beam_ids.push_back(int(id));o.azimuth[i]=az;o.ranges[i]=range;
  }
  return out;
}
}  // namespace posim_slam

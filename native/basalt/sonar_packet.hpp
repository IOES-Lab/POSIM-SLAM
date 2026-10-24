// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_factors.hpp"
#include <cstring>

namespace posim_slam {
constexpr size_t sonarSnapshotLimit=16+16*(16+64*24+128*128*8);
struct SonarSnapshotTrack {
  int64_t id;
  std::vector<SonarObservation> observations;
  Eigen::MatrixXd covariance;
};
struct SonarSnapshot {int64_t stamp;std::vector<SonarSnapshotTrack> tracks;};
inline SonarSnapshot decodeSonarSnapshot(const std::vector<char>& bytes) {
  if(bytes.size()<16 || bytes.size()>sonarSnapshotLimit)throw std::invalid_argument("sonar_snapshot_size");
  size_t offset=0;
  auto integer=[&](size_t width) {
    if(width>bytes.size()-offset)throw std::invalid_argument("truncated_sonar_snapshot");
    uint64_t value=0;for(size_t i=0;i<width;++i)value|=uint64_t(uint8_t(bytes[offset++]))<<(8*i);
    return value;
  };
  auto number=[&] {const uint64_t bits=integer(8);double value;std::memcpy(&value,&bits,8);return value;};
  auto stamp=integer(8),count=integer(4),reserved=integer(4);
  if(!stamp || stamp>uint64_t(INT64_MAX) || count<1 || count>16 || reserved)
    throw std::invalid_argument("invalid_sonar_snapshot_header");
  SonarSnapshot result{int64_t(stamp),{}};int64_t previous_id=-1;
  for(size_t t=0;t<count;++t) {
    const auto id=integer(8),n=integer(4),padding=integer(4);
    if(id>uint64_t(INT64_MAX) || int64_t(id)<=previous_id || n<1 || n>64 || padding)
      throw std::invalid_argument("invalid_sonar_snapshot_track");
    const size_t dimension=2*n,required=24*n+8*dimension*dimension;
    if(required>bytes.size()-offset)throw std::invalid_argument("truncated_sonar_snapshot");
    SonarSnapshotTrack track;track.id=int64_t(id);int64_t previous=0;
    for(size_t i=0;i<n;++i) {
      const auto time=integer(8);const double range=number(),azimuth=number();
      if(time>stamp || time<=uint64_t(previous) || !std::isfinite(range) || range<=0 || range>1000 ||
          !std::isfinite(azimuth) || std::abs(azimuth)>1.57079632679489661923)
        throw std::invalid_argument("invalid_sonar_snapshot_observation");
      track.observations.push_back({int64_t(time),range,azimuth});previous=int64_t(time);
    }
    if(previous!=result.stamp)throw std::invalid_argument("sonar_snapshot_image_stamp");
    track.covariance.resize(dimension,dimension);
    for(size_t row=0;row<dimension;++row)for(size_t col=0;col<dimension;++col)track.covariance(row,col)=number();
    sonarNoise(track.covariance,dimension);
    result.tracks.push_back(std::move(track));previous_id=int64_t(id);
  }
  if(offset!=bytes.size())throw std::invalid_argument("sonar_snapshot_trailing_bytes");
  return result;
}
}

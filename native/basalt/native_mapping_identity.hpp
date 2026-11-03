// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "input_digest.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <set>

namespace posim_slam {
namespace native_mapping_identity_detail {
// Canonical little-endian values, with lengths and sorted landmark IDs. The
// source identity excludes optimized variables; the state identity includes
// those variables separately. Neither digest is an authentication signature.
class Digest {
 public:
  void integer(uint64_t value) {
    uint8_t bytes[8];for(int i=0;i<8;++i)bytes[i]=uint8_t(value>>(8*i));hash.update(bytes,8);
  }
  void text(const std::string& value) {integer(value.size());hash.update(value.data(),value.size());}
  void numbers(const double* values,size_t size) {
    integer(size);for(size_t i=0;i<size;++i){uint64_t bits;std::memcpy(&bits,values+i,8);integer(bits);}
  }
  void image(const basalt::TimeCamId& value) {integer(value.frame_id);integer(value.cam_id);}
  InputDigest::Value finish() const {return hash.finish();}
 private:
  InputDigest hash;
};
inline std::set<int64_t> points(basalt::NfrMapper& mapper) {
  std::set<int64_t> ids;for(const auto& [id,point]:mapper.lmdb.getLandmarks())ids.insert(id);return ids;
}
} // namespace native_mapping_identity_detail

inline InputDigest::Value nativeMappingSourceIdentity(basalt::NfrMapper& mapper) {
  using namespace native_mapping_identity_detail;Digest hash;hash.text("native-mapping-original-likelihood-1");
  hash.integer(mapper.getFramePoses().size());for(const auto& [stamp,pose]:mapper.getFramePoses())hash.integer(stamp);
  hash.integer(mapper.calib.intrinsics.size());
  for(const auto& camera:mapper.calib.intrinsics) {
    hash.text(camera.getName());const auto parameters=camera.getParam();hash.numbers(parameters.data(),parameters.size());
  }
  hash.integer(mapper.calib.T_i_c.size());for(const auto& pose:mapper.calib.T_i_c)hash.numbers(pose.data(),7);
  hash.integer(mapper.calib.resolution.size());for(const auto& resolution:mapper.calib.resolution) {
    hash.integer(resolution.x());hash.integer(resolution.y());
  }
  const double noise[]={mapper.obs_std_dev,mapper.huber_thresh,mapper.config.vio_init_pose_weight};hash.numbers(noise,3);
  hash.integer(mapper.config.mapper_use_factors);hash.integer(mapper.config.mapper_no_factor_weights);
  hash.integer(mapper.rel_pose_factors.size());for(const auto& factor:mapper.rel_pose_factors) {
    hash.integer(factor.t_i_ns);hash.integer(factor.t_j_ns);hash.numbers(factor.T_i_j.data(),7);hash.numbers(factor.cov_inv.data(),36);
  }
  hash.integer(mapper.roll_pitch_factors.size());for(const auto& factor:mapper.roll_pitch_factors) {
    hash.integer(factor.t_ns);hash.numbers(factor.R_w_i_meas.data(),4);hash.numbers(factor.cov_inv.data(),4);
  }
  const auto ids=points(mapper);hash.integer(ids.size());
  for(auto id:ids) {
    const auto& point=mapper.lmdb.getLandmark(id);hash.integer(id);hash.image(point.host_kf_id);hash.integer(point.obs.size());
    for(const auto& [image,pixel]:point.obs) {
      hash.image(image);hash.integer(mapper.feature_tracks.at(id).at(image));hash.numbers(pixel.data(),2);
    }
  }
  return hash.finish();
}
inline InputDigest::Value nativeMappingStateIdentity(basalt::NfrMapper& mapper) {
  using namespace native_mapping_identity_detail;Digest hash;hash.text("native-mapping-current-state-1");
  hash.integer(mapper.getFramePoses().size());for(const auto& [stamp,state]:mapper.getFramePoses()) {
    hash.integer(stamp);hash.integer(state.isLinearized());hash.numbers(state.getPose().data(),7);
  }
  const auto ids=points(mapper);hash.integer(ids.size());for(auto id:ids) {
    const auto& point=mapper.lmdb.getLandmark(id);hash.integer(id);
    const double value[]={point.direction.x(),point.direction.y(),point.inv_dist};hash.numbers(value,3);
  }
  return hash.finish();
}
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "input_digest.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <functional>
#include <memory>

namespace posim_slam {
struct MappingInputFrame {
  int64_t stamp=0;int width=0,height=0;InputDigest::Value digest;
  basalt::KeypointsData features;
};
// A lease owns the decoded frame only for its caller's use. A saved graph may
// retain a reference without retaining the detector arrays in application RAM.
class MappingFrameReference {
 public:
  using Frame=MappingInputFrame;
  using Lease=std::shared_ptr<const Frame>;
  struct Metadata {int64_t stamp;int width,height;size_t corners;InputDigest::Value digest;};
  MappingFrameReference()=default;
  MappingFrameReference(Lease frame):resident_(std::move(frame)) {}
  MappingFrameReference(std::shared_ptr<Frame> frame):resident_(std::move(frame)) {}
  MappingFrameReference(Metadata metadata,std::function<Lease(size_t)> reader)
      :metadata_(metadata),reader_(std::move(reader)) {}
  explicit operator bool() const {return bool(resident_) || bool(reader_);}
  bool resident() const {return bool(resident_);}
  Metadata metadata() const {
    if(resident_)return {resident_->stamp,resident_->width,resident_->height,resident_->features.corners.size(),resident_->digest};
    if(!reader_)throw std::invalid_argument("mapping_input_frame_missing");return metadata_;
  }
  Lease read(size_t maximum=SIZE_MAX) const;
 private:
  Lease resident_;Metadata metadata_{};std::function<Lease(size_t)> reader_;
};
} // namespace posim_slam

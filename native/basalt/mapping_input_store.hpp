// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "estimator_budget.hpp"
#include "mapping_frame.hpp"
#include <filesystem>
#include <basalt/vi_estimator/nfr_mapper.h>
#include <basalt/hash_bow/hash_bow.h>
#include <atomic>
#include <memory>
#include <mutex>

namespace posim_slam {
class MappingInputArchive;
class MappingMatchHistory;
struct MappingInputLimits {
  size_t keyframes=kMaxJointColumns/6,image_bytes=96*1024*1024,retirements=512,update_retirements=8;
  size_t compact_bytes=96*1024*1024;
};
// One producer commits immutable compact input snapshots. Expensive recovery
// and feature extraction never hold the map publication mutex. Raw images and
// marginalization matrices belong to the caller and are not retained here.
class MappingInputStore {
 public:
  using Frame=MappingInputFrame;
  struct Data {
    uint64_t revision=0;
    std::map<int64_t,MappingFrameReference> frames;
    Eigen::aligned_map<int64_t,basalt::PoseStateWithLin<double>> poses;
    Eigen::aligned_vector<basalt::RelPoseFactor> relative;
    Eigen::aligned_vector<basalt::RollPitchFactor> roll_pitch;
    std::set<int64_t> retired;
    size_t source_image_bytes=0,source_factor_matrix_bytes=0,feature_payload_bytes=0;
    size_t compactPayloadBytes() const {
      return feature_payload_bytes+relative.size()*sizeof(basalt::RelPoseFactor)+
        roll_pitch.size()*sizeof(basalt::RollPitchFactor)+poses.size()*sizeof(basalt::PoseStateWithLin<double>);
    }
   private:
    friend class MappingInputStore;
    friend class NativeMappingGraph;
    std::shared_ptr<const int> identity;
  };
  using Snapshot=std::shared_ptr<const Data>;
  static basalt::VioConfig configuration(const basalt::VioConfig& supplied) {
    auto config=supplied;config.mapper_min_track_length=2;
    config.mapper_obs_std_dev=config.vio_obs_std_dev;config.mapper_obs_huber_thresh=config.vio_obs_huber_thresh;
    return config;
  }
  MappingInputStore(const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
                    MappingInputLimits limits={},std::filesystem::path archive_path={},bool restore=false,std::string epoch={});
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  Snapshot append(const basalt::MargData& source);
  std::unique_ptr<basalt::NfrMapper> makeMapper(const Snapshot& input) const;
  std::unique_ptr<basalt::NfrMapper> makeMatchedMapper(const Snapshot& input,
      const std::function<bool()>& cancelled={}) const;
  std::unique_ptr<MappingMatchHistory> makeMatchHistory() const;
  std::unique_ptr<basalt::NfrMapper> makeMatchedMapper(const Snapshot& input,MappingMatchHistory& history,
      const std::function<bool()>& cancelled={}) const;
  static InputDigest::Value imageDigest(const basalt::ManagedImage<uint16_t>& image) {
    InputDigest digest;const uint32_t dimensions[2]={uint32_t(image.w),uint32_t(image.h)};
    uint8_t bytes[4];for(uint32_t value:dimensions){for(int j=0;j<4;++j)bytes[j]=uint8_t(value>>(8*j));digest.update(bytes,4);}
    std::vector<uint8_t> row(2*image.w);
    for(size_t y=0;y<image.h;++y) {
      for(size_t x=0;x<image.w;++x){const auto pixel=image.RowPtr(y)[x];row[2*x]=uint8_t(pixel);row[2*x+1]=uint8_t(pixel>>8);}
      digest.update(row.data(),row.size());
    }
    return digest.finish();
  }
 private:
  void restoreArchive();
  std::shared_ptr<MappingInputArchive> archive_;
  Snapshot initial() const {auto value=std::make_shared<Data>();value->identity=identity_;return value;}
  static size_t payloadBytes(const Frame& frame) {
    const auto& f=frame.features;
    return sizeof(Frame)+f.corners.size()*sizeof(f.corners[0])+f.corner_angles.size()*sizeof(double)+
      f.corner_descriptors.size()*sizeof(f.corner_descriptors[0])+f.corners_3d.size()*sizeof(f.corners_3d[0])+
      f.hashes.size()*sizeof(f.hashes[0])+f.bow_vector.size()*sizeof(f.bow_vector[0]);
  }
  static void validate(const basalt::MargData& source,const Data& previous) {
    if(!source.use_imu || source.kfs_all.size()<2 || source.kfs_to_marg.empty() ||
        !source.abs_H.allFinite() || !source.abs_b.allFinite() || source.abs_H.rows()!=source.abs_H.cols() ||
        source.abs_H.rows()!=source.abs_b.size() || source.abs_H.rows()!=Eigen::Index(source.aom.total_size) ||
        source.aom.total_size>size_t(kMaxJointColumns) || source.opt_flow_res.size()!=source.kfs_all.size())
      throw std::invalid_argument("invalid_persistent_mapping_input");
    std::vector<bool> coordinates(source.aom.total_size,false);
    for(const auto& [stamp,block]:source.aom.abs_order_map) {
      const auto [offset,width]=block;
      if((width!=6 && width!=15) || width>source.aom.total_size || offset>source.aom.total_size-width ||
          (width==6 && (!source.frame_poses.count(stamp) || source.frame_states.count(stamp))) ||
          (width==15 && (!source.frame_states.count(stamp) || source.frame_poses.count(stamp))))
        throw std::invalid_argument("invalid_mapping_state_layout");
      for(size_t c=offset;c<offset+width;++c){if(coordinates[c])throw std::invalid_argument("overlapping_mapping_state_coordinates");coordinates[c]=true;}
      const auto pose=width==6?source.frame_poses.at(stamp).getPose():source.frame_states.at(stamp).getState().T_w_i;
      if(!pose.matrix().allFinite())throw std::invalid_argument("invalid_mapping_state_pose");
    }
    if(std::find(coordinates.begin(),coordinates.end(),false)!=coordinates.end())throw std::invalid_argument("incomplete_mapping_state_layout");
    for(int64_t stamp:source.kfs_all)if(!source.aom.abs_order_map.count(stamp))throw std::invalid_argument("missing_mapping_keyframe_state");
    for(int64_t stamp:source.kfs_to_marg)if(!source.kfs_all.count(stamp) || previous.retired.count(stamp))
      throw std::invalid_argument("mapping_retirement_reused");
  }
  basalt::Calibration<double> calibration_;basalt::VioConfig config_;MappingInputLimits limits_;
  mutable std::mutex writer_;const std::shared_ptr<const int> identity_=std::make_shared<const int>(0);Snapshot data_;
};
} // namespace posim_slam

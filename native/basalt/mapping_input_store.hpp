// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "estimator_budget.hpp"
#include "input_digest.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <basalt/hash_bow/hash_bow.h>
#include <atomic>
#include <memory>
#include <mutex>

namespace posim_slam {
struct MappingInputLimits {
  size_t keyframes=kMaxJointColumns/6,image_bytes=96*1024*1024,retirements=512,update_retirements=8;
  size_t compact_bytes=96*1024*1024;
};
// One producer commits immutable compact input snapshots. Expensive recovery
// and feature extraction never hold the map publication mutex. Raw images and
// marginalization matrices belong to the caller and are not retained here.
class MappingInputStore {
 public:
  struct Frame {
    int64_t stamp=0;int width=0,height=0;InputDigest::Value digest;
    basalt::KeypointsData features;
  };
  struct Data {
    uint64_t revision=0;
    std::map<int64_t,std::shared_ptr<const Frame>> frames;
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
    std::shared_ptr<const int> identity;
  };
  using Snapshot=std::shared_ptr<const Data>;
  static basalt::VioConfig configuration(const basalt::VioConfig& supplied) {
    auto config=supplied;config.mapper_min_track_length=2;
    config.mapper_obs_std_dev=config.vio_obs_std_dev;config.mapper_obs_huber_thresh=config.vio_obs_huber_thresh;
    return config;
  }
  MappingInputStore(const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
                    MappingInputLimits limits={})
      :calibration_(calibration),config_(configuration(config)),limits_(limits),data_(initial()) {
    if(calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1 || calibration.resolution.size()!=1 ||
        limits.keyframes<2 || limits.keyframes>size_t(kMaxJointColumns/6) ||
        limits.image_bytes<2 || limits.image_bytes>96*1024*1024 ||
        !limits.retirements || limits.retirements>512 || !limits.update_retirements ||
        limits.compact_bytes<2 || limits.compact_bytes>96*1024*1024 ||
        limits.update_retirements>limits.retirements || !config.mapper_use_factors || config.mapper_no_factor_weights)
      throw std::invalid_argument("invalid_persistent_mapping_configuration");
  }
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  Snapshot append(const basalt::MargData& source) {
    std::lock_guard<std::mutex> writer(writer_);
    auto next=std::make_shared<Data>(*snapshot());validate(source,*next);
    if(next->revision>=limits_.retirements)throw std::runtime_error("mapping_retirement_capacity");
    auto owned=std::make_shared<basalt::MargData>(source);owned->opt_flow_res.clear();
    basalt::NfrMapper recovered(calibration_,config_);recovered.addMargData(owned);
    if(recovered.roll_pitch_factors.size()!=1)throw std::runtime_error("mapping_recovered_information_singular");
    std::set<int64_t> stamps;std::map<int64_t,InputDigest::Value> identities;
    for(const auto& flow:source.opt_flow_res) {
      if(!flow || !flow->input_images || flow->t_ns!=flow->input_images->t_ns ||
          !source.kfs_all.count(flow->t_ns) || !stamps.insert(flow->t_ns).second ||
          flow->input_images->img_data.size()!=1 || !flow->input_images->img_data[0].img)
        throw std::invalid_argument("invalid_persistent_mapping_image");
      const auto& image=*flow->input_images->img_data[0].img;
      if(image.w<1 || image.h<1 || image.w>1920 || image.h>1080 ||
          int(image.w)!=calibration_.resolution[0].x() || int(image.h)!=calibration_.resolution[0].y())
        throw std::invalid_argument("invalid_persistent_mapping_image_shape");
      const auto identity=imageDigest(image);
      auto found=next->frames.find(flow->t_ns);
      if(found!=next->frames.end()) {
        if(identity!=found->second->digest)throw std::invalid_argument("mapping_image_identity_changed");
        continue;
      }
      const size_t bytes=size_t(image.w)*image.h*sizeof(uint16_t);
      if(next->frames.size()>=limits_.keyframes || bytes>limits_.image_bytes-next->source_image_bytes)
        throw std::runtime_error("mapping_image_capacity");
      // Reuse the pinned detector, camera unprojection and HashBoW. Its input
      // image is borrowed only during this append and released before commit.
      recovered.img_data.emplace(flow->t_ns,flow->input_images);
      identities.emplace(flow->t_ns,identity);
      next->source_image_bytes+=bytes;
    }
    recovered.detect_keypoints();
    for(const auto& [id,features]:recovered.feature_corners) {
      auto frame=std::make_shared<Frame>();frame->stamp=id.frame_id;
      const auto& image=*recovered.img_data.at(id.frame_id)->img_data[0].img;
      frame->width=image.w;frame->height=image.h;frame->digest=identities.at(id.frame_id);frame->features=features;
      next->feature_payload_bytes+=payloadBytes(*frame);next->frames.emplace(id.frame_id,std::move(frame));
    }
    for(const auto& [stamp,pose]:recovered.getFramePoses())next->poses[stamp]=pose;
    if(next->poses.size()!=next->frames.size())throw std::runtime_error("mapping_pose_image_or_reference_mismatch");
    next->relative.insert(next->relative.end(),recovered.rel_pose_factors.begin(),recovered.rel_pose_factors.end());
    next->roll_pitch.insert(next->roll_pitch.end(),recovered.roll_pitch_factors.begin(),recovered.roll_pitch_factors.end());
    next->retired.insert(source.kfs_to_marg.begin(),source.kfs_to_marg.end());
    next->source_factor_matrix_bytes+=(source.abs_H.size()+source.abs_b.size())*sizeof(double);
    if(next->compactPayloadBytes()>limits_.compact_bytes)throw std::runtime_error("mapping_compact_input_capacity");
    ++next->revision;Snapshot committed=next;std::atomic_store(&data_,committed);return committed;
  }
  std::unique_ptr<basalt::NfrMapper> makeMapper(const Snapshot& input) const {
    if(!input || input->identity!=identity_)throw std::invalid_argument("mapping_input_snapshot_owner");
    auto mapper=std::make_unique<basalt::NfrMapper>(calibration_,config_);
    mapper->getFramePoses()=input->poses;mapper->rel_pose_factors=input->relative;mapper->roll_pitch_factors=input->roll_pitch;
    for(const auto& [stamp,frame]:input->frames) {
      basalt::TimeCamId id(stamp,0);mapper->feature_corners.emplace(id,frame->features);
      mapper->hash_bow_database->add_to_database(id,frame->features.bow_vector);
    }
    return mapper;
  }
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

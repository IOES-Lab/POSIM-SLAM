// SPDX-License-Identifier: Apache-2.0
#include "mapping_input_store.hpp"
#include "mapping_input_archive.hpp"
#include "mapping_graph.hpp"

namespace posim_slam {
MappingInputStore::MappingInputStore(const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
                    MappingInputLimits limits,std::filesystem::path archive_path,bool restore,std::string epoch)
      :calibration_(calibration),config_(configuration(config)),limits_(limits),data_(initial()) {
    if(calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1 || calibration.resolution.size()!=1 ||
        limits.keyframes<2 || limits.keyframes>size_t(kMaxJointColumns/6) ||
        limits.image_bytes<2 || limits.image_bytes>96*1024*1024 ||
        !limits.retirements || limits.retirements>512 || !limits.update_retirements ||
        limits.compact_bytes<2 || limits.compact_bytes>96*1024*1024 ||
        limits.update_retirements>limits.retirements || !config.mapper_use_factors || config.mapper_no_factor_weights)
      throw std::invalid_argument("invalid_persistent_mapping_configuration");
    if(restore && archive_path.empty())throw std::invalid_argument("mapping_input_storage_path");
    if(!archive_path.empty()) {
      if(epoch.empty() || epoch.size()>256)throw std::invalid_argument("mapping_input_storage_epoch");
      using namespace mapping_graph_detail;
      const auto& camera=calibration_.intrinsics[0];const auto parameters=camera.getParam();
      const Json metadata={{"schema","posim-slam-mapping-inputs-1"},{"epoch",epoch},
        {"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
        {"config",mapping_graph_detail::config(config_)},
        {"camera",{camera.getName(),doubles(parameters.data(),parameters.size()),doubles(calibration_.T_i_c[0].data(),7),
          calibration_.resolution[0].x(),calibration_.resolution[0].y()}},
        {"limits",{limits.keyframes,limits.image_bytes,limits.retirements,limits.update_retirements,limits.compact_bytes}}};
      archive_=std::make_shared<MappingInputArchive>(archive_path,metadata,restore);
      if(restore)restoreArchive();
    }
  }
MappingInputStore::Snapshot MappingInputStore::append(const basalt::MargData& source) {
    std::lock_guard<std::mutex> writer(writer_);
    const auto previous=snapshot();auto next=std::make_shared<Data>(*previous);validate(source,*next);
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
        if(identity!=found->second.metadata().digest)throw std::invalid_argument("mapping_image_identity_changed");
        continue;
      }
      const size_t bytes=size_t(image.w)*image.h*sizeof(uint16_t);
      if(next->frames.size()+identities.size()>=limits_.keyframes || bytes>limits_.image_bytes-next->source_image_bytes)
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
    ++next->revision;
    if(archive_) {
      using namespace mapping_graph_detail;
      Json batch={{"revision",next->revision},{"retired",source.kfs_to_marg},
        {"counts",{next->source_image_bytes,next->source_factor_matrix_bytes,next->feature_payload_bytes}}};
      for(const char* name:{"frames","poses","relative","roll_pitch"})batch[name]=Json::array();
      std::vector<MappingInputArchive::Page> pages;pages.reserve(identities.size());
      for(const auto& [stamp,unused]:identities) {
        auto& reference=next->frames.at(stamp);auto page=MappingInputArchive::page(*reference.read());
        batch["frames"].push_back(page.identity);reference=archive_->reference(page.identity,next->revision);
        pages.push_back(std::move(page));
      }
      for(const auto& [stamp,pose]:recovered.getFramePoses())batch["poses"].push_back({stamp,doubles(pose.getPose().data(),7)});
      for(const auto& f:recovered.rel_pose_factors)batch["relative"].push_back({f.t_i_ns,f.t_j_ns,doubles(f.T_i_j.data(),7),doubles(f.cov_inv.data(),36)});
      for(const auto& f:recovered.roll_pitch_factors)batch["roll_pitch"].push_back({f.t_ns,doubles(f.R_w_i_meas.data(),4),doubles(f.cov_inv.data(),4)});
      // All allocations and validations precede the disk transaction. Publishing
      // the immutable snapshot after a successful COMMIT cannot expose half a batch.
      archive_->commit(next->revision,batch,pages);
    }
    Snapshot committed=next;std::atomic_store(&data_,committed);return committed;
  }
std::unique_ptr<basalt::NfrMapper> MappingInputStore::makeMapper(const Snapshot& input) const {
    if(!input || input->identity!=identity_)throw std::invalid_argument("mapping_input_snapshot_owner");
    auto mapper=std::make_unique<basalt::NfrMapper>(calibration_,config_);
    mapper->getFramePoses()=input->poses;mapper->rel_pose_factors=input->relative;mapper->roll_pitch_factors=input->roll_pitch;
    for(const auto& [stamp,reference]:input->frames) {
      const auto frame=reference.read();
      basalt::TimeCamId id(stamp,0);mapper->feature_corners.emplace(id,frame->features);
      mapper->hash_bow_database->add_to_database(id,frame->features.bow_vector);
    }
    return mapper;
  }
void MappingInputStore::restoreArchive() {
  using namespace mapping_graph_detail;
  auto next=std::make_shared<Data>(*initial());
  archive_->replay(limits_.retirements,[&](const auto& batch) {
    if(integer(batch.at("revision"),1,limits_.retirements)!=int64_t(next->revision+1))
      throw std::invalid_argument("mapping_input_storage_revision");
    ++next->revision;
    for(const auto& row:batch.at("frames")) {
      auto reference=archive_->reference(row,next->revision);const auto metadata=reference.metadata();
      if(calibration_.resolution[0]!=Eigen::Vector2i(metadata.width,metadata.height) ||
          !next->frames.emplace(metadata.stamp,std::move(reference)).second || next->frames.size()>limits_.keyframes)
        throw std::invalid_argument("mapping_input_frame_identity");
    }
    for(const auto& row:batch.at("poses")) {
      const auto stamp=integer(row.at(0),1,INT64_MAX);
      if(!next->frames.count(stamp))throw std::invalid_argument("mapping_pose_image_or_reference_mismatch");
      next->poses[stamp]=basalt::PoseStateWithLin<double>(stamp,pose(row.at(1)));
    }
    for(const auto& row:batch.at("relative")) {
      basalt::RelPoseFactor f;f.t_i_ns=integer(row.at(0),1,INT64_MAX);f.t_j_ns=integer(row.at(1),1,INT64_MAX);
      if(f.t_i_ns==f.t_j_ns || !next->frames.count(f.t_i_ns) || !next->frames.count(f.t_j_ns))
        throw std::invalid_argument("mapping_input_factor_identity");
      f.T_i_j=pose(row.at(2));const auto values=doubles(row.at(3),36);std::copy(values.begin(),values.end(),f.cov_inv.data());
      next->relative.push_back(f);
    }
    for(const auto& row:batch.at("roll_pitch")) {
      basalt::RollPitchFactor f;f.t_ns=integer(row.at(0),1,INT64_MAX);
      if(!next->frames.count(f.t_ns))throw std::invalid_argument("mapping_input_factor_identity");
      const auto rotation=doubles(row.at(1),4),values=doubles(row.at(2),4);
      if(std::abs(Eigen::Map<const Eigen::Vector4d>(rotation.data()).squaredNorm()-1)>1e-10)
        throw std::invalid_argument("mapping_graph_rotation");
      std::copy(rotation.begin(),rotation.end(),f.R_w_i_meas.data());std::copy(values.begin(),values.end(),f.cov_inv.data());next->roll_pitch.push_back(f);
    }
    if(!batch.at("retired").is_array() || batch.at("retired").empty())throw std::invalid_argument("mapping_retirement_reused");
    for(const auto& row:batch.at("retired")) {
      const auto stamp=integer(row,1,INT64_MAX);
      if(!next->frames.count(stamp) || !next->retired.insert(stamp).second)throw std::invalid_argument("mapping_retirement_reused");
    }
    const auto& counts=batch.at("counts");
    next->source_image_bytes=integer(counts.at(0),next->source_image_bytes,limits_.image_bytes);
    next->source_factor_matrix_bytes=integer(counts.at(1),next->source_factor_matrix_bytes,INT64_MAX);
    next->feature_payload_bytes=integer(counts.at(2),next->feature_payload_bytes,limits_.compact_bytes);
    if(next->poses.size()!=next->frames.size() || next->compactPayloadBytes()>limits_.compact_bytes)
      throw std::invalid_argument("mapping_compact_input_capacity");
  });
  Snapshot committed=next;std::atomic_store(&data_,committed);
}
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_input_store.hpp"
#include "mapping_information.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <type_traits>

namespace posim_slam {
namespace mapping_graph_detail {
using Json=nlohmann::json;
inline Json doubles(const double* values,size_t count) {
  std::vector<uint8_t> bytes(8*count);
  for(size_t i=0;i<count;++i) {
    uint64_t bits;std::memcpy(&bits,values+i,8);
    for(int j=0;j<8;++j)bytes[8*i+j]=uint8_t(bits>>(8*j));
  }
  return Json::binary(bytes);
}
inline std::vector<double> doubles(const Json& value,size_t count,bool finite=true) {
  if(!value.is_binary() || value.get_binary().size()!=8*count)throw std::invalid_argument("mapping_graph_binary_shape");
  const auto& bytes=value.get_binary();std::vector<double> out(count);
  for(size_t i=0;i<count;++i) {
    uint64_t bits=0;for(int j=0;j<8;++j)bits|=uint64_t(bytes[8*i+j])<<(8*j);
    std::memcpy(out.data()+i,&bits,8);
    if(finite && !std::isfinite(out[i]))throw std::invalid_argument("mapping_graph_nonfinite_input");
  }
  return out;
}
inline int64_t integer(const Json& value,int64_t low,int64_t high) {
  if(!value.is_number_integer())throw std::invalid_argument("mapping_graph_integer");
  const auto number=value.get<int64_t>();
  if(number<low || number>high)throw std::invalid_argument("mapping_graph_integer");return number;
}
inline Sophus::SE3d pose(const Json& value) {
  const auto data=doubles(value,7);Sophus::SE3d out;
  if(std::abs(Eigen::Map<const Eigen::Vector4d>(data.data()).squaredNorm()-1)>1e-10)
    throw std::invalid_argument("mapping_graph_rotation");
  std::copy(data.begin(),data.end(),out.data());return out;
}
inline MappingObservation key(const Json& row) {
  if(!row.is_array() || row.size()<3)throw std::invalid_argument("mapping_graph_observation");
  return {integer(row[0],1,INT64_MAX),int(integer(row[1],0,0)),integer(row[2],0,999999)};
}
inline Json key(const MappingObservation& value) {return Json::array({std::get<0>(value),std::get<1>(value),std::get<2>(value)});}
inline bool same(const Eigen::Vector2d& first,const Eigen::Vector2d& second) {
  return std::memcmp(first.data(),second.data(),2*sizeof(double))==0;
}
inline std::string epoch() {
  std::random_device source;std::ostringstream out;out<<std::hex;
  for(int i=0;i<4;++i){out.width(8);out.fill('0');out<<uint32_t(source());}return out.str();
}
// These are the complete mapper controls consumed by the pinned NfrMapper.
// Navigation-only controls are not a saved VIO restart state.
#define POSIM_MAPPING_CONFIG_FIELDS(F) \
 F(mapper_obs_std_dev) F(mapper_obs_huber_thresh) F(mapper_detection_num_points) \
 F(mapper_num_frames_to_match) F(mapper_frames_to_match_threshold) F(mapper_min_matches) \
 F(mapper_ransac_threshold) F(mapper_min_track_length) F(mapper_max_hamming_distance) \
 F(mapper_second_best_test_ratio) F(mapper_bow_num_bits) F(mapper_min_triangulation_dist) \
 F(mapper_no_factor_weights) F(mapper_use_factors) F(mapper_use_lm) \
 F(mapper_lm_lambda_min) F(mapper_lm_lambda_max) F(vio_init_pose_weight)
inline Json config(const basalt::VioConfig& value) {
  Json result;
#define POSIM_SAVE_FIELD(name) result[#name]=value.name;
  POSIM_MAPPING_CONFIG_FIELDS(POSIM_SAVE_FIELD)
#undef POSIM_SAVE_FIELD
  return result;
}
inline basalt::VioConfig config(const Json& value) {
  basalt::VioConfig result;
#define POSIM_LOAD_FIELD(name) result.name=value.at(#name).get<decltype(result.name)>();
  POSIM_MAPPING_CONFIG_FIELDS(POSIM_LOAD_FIELD)
#undef POSIM_LOAD_FIELD
  if(!std::isfinite(result.mapper_obs_std_dev) || result.mapper_obs_std_dev<=0 ||
      !std::isfinite(result.mapper_obs_huber_thresh) || result.mapper_obs_huber_thresh<=0 ||
      !result.mapper_use_factors || result.mapper_no_factor_weights)
    throw std::invalid_argument("mapping_graph_configuration");
  return result;
}
#undef POSIM_MAPPING_CONFIG_FIELDS
} // namespace mapping_graph_detail

struct MappingGraph {
  using Json=nlohmann::json;
  Json data;
  std::map<int64_t,int> stable_ids;
 private:
  friend class MappingGraphRegistry;
  bool validated=false;
};

// Segment-local pixel identities survive internal TrackBuilder renumbering.
// A point ID names the minimum original observation in its current track;
// reassociation/splits replace the complete graph, not append old likelihoods.
// One optimizer thread owns the registry; readers hold immutable snapshots.
class MappingGraphRegistry {
 public:
  using Snapshot=std::shared_ptr<const MappingGraph>;
  MappingGraphRegistry(const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,
      std::string epoch={}) :calibration_(calibration),config_(config),epoch_(epoch.empty()?mapping_graph_detail::epoch():std::move(epoch)) {
    if(calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1 || calibration.resolution.size()!=1 || epoch_.size()>256)
      throw std::invalid_argument("mapping_graph_configuration");
  }
  Snapshot capture(const MappingInputStore::Snapshot& input,basalt::NfrMapper& mapper,
      int64_t reference_stamp,const Sophus::SE3d& reference_pose,const Sophus::SE3d& gauge) {
    using namespace mapping_graph_detail;
    if(!input || input->revision<last_input_ || !mapper.getFramePoses().count(reference_stamp) ||
        mapper.getFramePoses().size()!=input->frames.size() || mapper.feature_tracks.size()>100000 ||
        mapping_graph_detail::config(mapper.config)!=mapping_graph_detail::config(config_))
      throw std::invalid_argument("mapping_graph_source");
    auto anchors=anchors_;int next_id=next_id_;
    Json frames=Json::array(),poses=Json::array(),relative=Json::array(),roll_pitch=Json::array(),pixels=Json::array(),tracks=Json::array(),landmarks=Json::array();
    for(const auto& [stamp,frame]:input->frames) {
      if(calibration_.resolution[0]!=Eigen::Vector2i(frame->width,frame->height))throw std::invalid_argument("mapping_graph_frame_shape");
      frames.push_back({stamp,frame->width,frame->height,frame->features.corners.size(),Json::binary(std::vector<uint8_t>(frame->digest.begin(),frame->digest.end()))});
      const auto value=mapper.getFramePoses().at(stamp).getPose();poses.push_back({stamp,doubles(value.data(),7)});
    }
    std::set<std::pair<int64_t,int64_t>> factor_ids;
    for(const auto& factor:mapper.rel_pose_factors) {
      if(!factor_ids.emplace(factor.t_i_ns,factor.t_j_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
      relative.push_back({factor.t_i_ns,factor.t_j_ns,doubles(factor.T_i_j.data(),7),doubles(factor.cov_inv.data(),36)});
    }
    std::set<int64_t> roll_ids;
    for(const auto& factor:mapper.roll_pitch_factors) {
      if(!roll_ids.insert(factor.t_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
      roll_pitch.push_back({factor.t_ns,doubles(factor.R_w_i_meas.data(),4),doubles(factor.cov_inv.data(),4)});
    }
    auto result=std::make_shared<MappingGraph>();std::map<MappingObservation,size_t> owners;
    std::map<MappingObservation,std::pair<int64_t,const basalt::FeatureTrack*>> ordered;
    for(const auto& [native,track]:mapper.feature_tracks) {
      if(native<0 || native>INT32_MAX || track.size()<2)throw std::invalid_argument("mapping_graph_track");
      const auto& first=*track.begin();MappingObservation anchor{first.first.frame_id,first.first.cam_id,first.second};
      if(!ordered.emplace(anchor,std::make_pair(native,&track)).second)throw std::invalid_argument("mapping_graph_observation_reused");
    }
    for(const auto& [anchor,item]:ordered) {
      const auto [native,track]=item;auto found=anchors.find(anchor);
      if(found==anchors.end()) {
        if(anchors.size()>=100000)throw std::runtime_error("mapping_graph_identity_capacity");
        found=anchors.emplace(anchor,next_id++).first;
      }
      result->stable_ids.emplace(native,found->second);Json members=Json::array();
      for(const auto& [image,feature]:*track) {
        const auto frame=input->frames.find(image.frame_id);
        if(image.cam_id!=0 || frame==input->frames.end() || feature<0 || size_t(feature)>=frame->second->features.corners.size())
          throw std::invalid_argument("mapping_graph_observation_source");
        const auto& pixel=frame->second->features.corners[feature];
        if(!pixel.allFinite() || !same(pixel,mapper.feature_corners.at(image).corners.at(feature)))
          throw std::invalid_argument("mapping_source_pixel_changed");
        MappingObservation identity{image.frame_id,image.cam_id,feature};
        if(owners.size()>=1000000 || !owners.emplace(identity,pixels.size()).second)
          throw std::invalid_argument("mapping_graph_observation_reused");
        members.push_back(pixels.size());auto row=key(identity);row.push_back(doubles(pixel.data(),2));pixels.push_back(std::move(row));
      }
      tracks.push_back({native,found->second,members});
    }
    for(const auto& [native,landmark]:mapper.lmdb.getLandmarks()) {
      if(!result->stable_ids.count(native))throw std::invalid_argument("mapping_graph_landmark_track");
      Json observations=Json::array();const auto& track=mapper.feature_tracks.at(native);
      for(const auto& [image,pixel]:landmark.obs) {
        const auto feature=track.at(image);MappingObservation identity{image.frame_id,image.cam_id,feature};
        if(!same(pixel,input->frames.at(image.frame_id)->features.corners.at(feature)))throw std::invalid_argument("mapping_source_pixel_changed");
        observations.push_back(owners.at(identity));
      }
      const double parameters[3]={landmark.direction.x(),landmark.direction.y(),landmark.inv_dist};
      landmarks.push_back({native,landmark.host_kf_id.frame_id,landmark.host_kf_id.cam_id,doubles(parameters,3),observations});
    }
    const auto& camera=calibration_.intrinsics[0];const auto parameters=camera.getParam();Json ledger=Json::array();
    for(const auto& [anchor,id]:anchors){auto row=key(anchor);row.push_back(id);ledger.push_back(std::move(row));}
    result->data={{"schema","posim-slam-mapping-graph-1"},{"posim_slam_version","0.1.0"},{"epoch",epoch_},{"input_revision",input->revision},{"graph_revision",revision_+1},
      {"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},{"coordinates","native_vio_world"},{"map_qualified",false},
      {"camera",{camera.getName(),doubles(parameters.data(),parameters.size()),doubles(calibration_.T_i_c[0].data(),7)}},
      {"config",mapping_graph_detail::config(config_)},{"reference",{reference_stamp,doubles(reference_pose.data(),7)}},{"gauge",doubles(gauge.data(),7)},
      {"optimizer",{mapper.lambda,mapper.min_lambda,mapper.max_lambda,mapper.lambda_vee}},
      {"frames",frames},{"poses",poses},{"relative",relative},{"roll_pitch",roll_pitch},{"pixels",pixels},{"tracks",tracks},{"landmarks",landmarks},{"identities",ledger}};
    // Validate the complete replacement before advancing original identity state.
    restore(result->data);result->validated=true;anchors_.swap(anchors);next_id_=next_id;last_input_=input->revision;++revision_;
    return result;
  }
  static std::unique_ptr<basalt::NfrMapper> restore(const nlohmann::json& data,basalt::Calibration<double>* calibration_out=nullptr);
  void adopt(const Snapshot& graph);
  static void write(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,int64_t tracking_stamp);
  struct Restored {Snapshot graph;uint64_t request;int64_t tracking_stamp;};
  static Restored read(const std::filesystem::path& path,const std::string& expected_epoch={});
  const std::string& epoch() const {return epoch_;}
 private:
  basalt::Calibration<double> calibration_;basalt::VioConfig config_;std::string epoch_;
  std::map<MappingObservation,int> anchors_;int next_id_=0;uint64_t revision_=0,last_input_=0;
};
} // namespace posim_slam

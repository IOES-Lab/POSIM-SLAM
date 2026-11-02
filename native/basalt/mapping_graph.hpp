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
  return Json::binary(std::move(bytes));
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
  std::map<int64_t,int> stable_ids;
 private:
  friend class MappingGraphRegistry;
  using Json=nlohmann::json;
  using Ledger=std::map<MappingObservation,int>;
  struct Captured {
    struct Frame {int64_t stamp;int width,height;size_t corners;InputDigest::Value digest;};
    struct Pixel {MappingObservation identity;Eigen::Vector2d value;};
    struct Track {int64_t native;int stable;std::vector<size_t> pixels;};
    struct Landmark {int64_t native,host;int camera;Eigen::Vector3d parameters;std::vector<size_t> pixels;};
    std::string epoch;uint64_t input_revision=0,graph_revision=0;
    basalt::Calibration<double> calibration;basalt::VioConfig config;
    std::vector<Frame> frames;Eigen::aligned_vector<std::pair<int64_t,Sophus::SE3d>> poses;
    Eigen::aligned_vector<basalt::RelPoseFactor> relative;Eigen::aligned_vector<basalt::RollPitchFactor> roll_pitch;
    Eigen::aligned_vector<Pixel> pixels;std::vector<Track> tracks;Eigen::aligned_vector<Landmark> landmarks;
    std::shared_ptr<const Ledger> identities;
    int64_t reference_stamp=0;Sophus::SE3d reference_pose,gauge;std::array<double,4> optimizer;
  };
  std::shared_ptr<const Captured> captured;
  Json imported;
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
      int64_t reference_stamp,const Sophus::SE3d& reference_pose,const Sophus::SE3d& gauge);
  // Captured snapshots own typed immutable values. JSON is materialized only
  // for explicit serialization/inspection, never for normal map publication.
  static nlohmann::json document(const Snapshot& graph);
  static std::unique_ptr<basalt::NfrMapper> restore(const nlohmann::json& data,basalt::Calibration<double>* calibration_out=nullptr);
  void adopt(const Snapshot& graph);
  static void write(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,int64_t tracking_stamp);
  struct Restored {Snapshot graph;uint64_t request;int64_t tracking_stamp;};
  static Restored read(const std::filesystem::path& path,const std::string& expected_epoch={});
  const std::string& epoch() const {return epoch_;}
 private:
  basalt::Calibration<double> calibration_;basalt::VioConfig config_;std::string epoch_;
  std::shared_ptr<const MappingGraph::Ledger> anchors_=std::make_shared<const MappingGraph::Ledger>();
  int next_id_=0;uint64_t revision_=0,last_input_=0;
};
} // namespace posim_slam

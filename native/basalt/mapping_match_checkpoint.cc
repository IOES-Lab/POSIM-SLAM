// SPDX-License-Identifier: Apache-2.0
#include "mapping_feature_matcher.hpp"
#include "mapping_graph.hpp"

namespace posim_slam {
namespace {
using namespace mapping_graph_detail;
Json profile(const basalt::Calibration<double>& calibration,const basalt::VioConfig& configuration) {
  if(calibration.intrinsics.size()!=1 || calibration.T_i_c.size()!=1 || calibration.resolution.size()!=1)
    throw std::invalid_argument("mapping_match_input");
  const auto& camera=calibration.intrinsics[0];const auto parameters=camera.getParam();
  return {{"camera",{camera.getName(),doubles(parameters.data(),parameters.size()),
      doubles(calibration.T_i_c[0].data(),7),calibration.resolution[0].x(),calibration.resolution[0].y()}},
      {"config",config(configuration)}};
}
Json frameIdentity(const MappingInputFrame& frame,const InputDigest::Value& digest) {
  return {frame.stamp,frame.width,frame.height,frame.features.corners.size(),
      mapping_frame_io::binary(frame.digest),mapping_frame_io::binary(digest)};
}
Json frameIdentity(const MappingInputFrame& frame) {
  return frameIdentity(frame,mapping_frame_io::digest(mapping_frame_io::encode(frame)));
}
const Json& checked(const Json& document) {
  if(!document.is_object() || document.size()!=3 || document.at("schema")!="posim-slam-matching-1")
    throw std::invalid_argument("mapping_match_checkpoint_schema");
  const auto& data=document.at("data");
  const size_t maximum_frames=kMaxMappingHistoryFrames;
  if(!data.is_object() || data.size()!=5 || !data.at("frames").is_array() ||
      data.at("frames").empty() || data.at("frames").size()>maximum_frames ||
      !data.at("evaluated").is_array() ||
      data.at("evaluated").size()>data.at("frames").size()*(data.at("frames").size()-1)/2 ||
      !data.at("matches").is_array() || data.at("matches").size()>data.at("evaluated").size())
    throw std::invalid_argument("mapping_match_checkpoint_shape");
  integer(data.at("revision"),1,INT64_MAX);
  const auto bytes=Json::to_cbor(data);
  if(bytes.size()>96*1024*1024)throw std::runtime_error("mapping_match_capacity");
  if(document.at("sha256")!=mapping_frame_io::binary(mapping_frame_io::digest(bytes)))
    throw std::invalid_argument("mapping_match_checkpoint_digest");
  return data;
}
}

nlohmann::json MappingMatchHistory::checkpoint(const Snapshot& state,
    const basalt::Calibration<double>& calibration,const basalt::VioConfig& config,const FrameVisitor& frame_visitor) {
  using namespace mapping_graph_detail;
  if(!state || !state->owner || !state->revision || state->frames.empty() ||
      state->evaluated_pairs!=state->evaluated.size())throw std::invalid_argument("mapping_match_checkpoint_source");
  Json data={{"profile",profile(calibration,config)},{"revision",state->revision},
      {"frames",Json::array()},{"evaluated",Json::array()},{"matches",Json::array()}};
  for(const auto& [stamp,retained]:state->frames) {
    const auto frame=retained.source.read();mapping_frame_io::shape(*frame);
    if(frame->stamp!=stamp || !retained.pixels || frame->features.corners!=retained.pixels->corners ||
        frame->features.bow_vector!=retained.bow)throw std::invalid_argument("mapping_match_checkpoint_source");
    const auto encoded=mapping_frame_io::encode(*frame);const auto digest=mapping_frame_io::digest(encoded);
    data["frames"].push_back(frameIdentity(*frame,digest));
    if(frame_visitor)frame_visitor(retained.source,*frame,encoded,digest);
  }
  for(const auto& pair:state->evaluated)data["evaluated"].push_back({pair.first.frame_id,pair.second.frame_id});
  for(const auto& [pair,match]:state->matches) {
    if(!state->evaluated.count(pair) || !match)throw std::invalid_argument("mapping_match_checkpoint_source");
    data["matches"].push_back({pair.first.frame_id,pair.second.frame_id,match->matches,match->inliers,
        doubles(match->T_i_j.data(),7)});
  }
  auto bytes=Json::to_cbor(data);Json document={{"schema","posim-slam-matching-1"},{"data",std::move(data)},
      {"sha256",mapping_frame_io::binary(mapping_frame_io::digest(bytes))}};
  checked(document);return document;
}

void MappingMatchHistory::validateCheckpoint(const nlohmann::json& document) {checked(document);}

MappingMatchHistory::Snapshot MappingMatchHistory::restore(uint64_t revision,
    const std::map<int64_t,MappingFrameReference>& frames,const nlohmann::json& document,
    const std::function<bool()>& cancelled) {
  using namespace mapping_graph_detail;
  std::lock_guard<std::mutex> writer(writer_);
  auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("mapping_match_cancelled");};check();
  const auto old=snapshot();
  if(old->revision || !old->frames.empty())throw std::invalid_argument("mapping_match_restore_requires_empty");
  if(frames.size()>maximum_frames_)throw std::invalid_argument("mapping_match_revision");
  const auto& data=checked(document);
  if(data.at("revision")!=revision || data.at("profile")!=profile(calibration_,config_) ||
      data.at("frames").size()!=frames.size())throw std::invalid_argument("mapping_match_checkpoint_source");
  auto next=std::make_shared<Data>();next->owner=identity_;next->revision=revision;
  auto database=std::make_shared<basalt::HashBow<256>>(config_.mapper_bow_num_bits);
  auto reserve=[&](size_t bytes) {
    if(next->numeric_bytes>maximum_bytes_ || bytes>maximum_bytes_-next->numeric_bytes)
      throw std::runtime_error("mapping_match_capacity");
    next->numeric_bytes+=bytes;
  };
  for(const auto& row:data.at("frames")) {
    check();if(!row.is_array() || row.size()!=6)throw std::invalid_argument("mapping_match_checkpoint_shape");
    const int64_t stamp=integer(row[0],1,INT64_MAX);const auto found=frames.find(stamp);
    if(found==frames.end() || next->frames.count(stamp))throw std::invalid_argument("mapping_match_checkpoint_source");
    const auto frame=found->second.read();mapping_frame_io::shape(*frame);
    if(frameIdentity(*frame)!=row || calibration_.resolution[0]!=Eigen::Vector2i(frame->width,frame->height))
      throw std::invalid_argument("mapping_match_checkpoint_source");
    reserve(frame->features.corners.size()*sizeof(Eigen::Vector2d)+frame->features.bow_vector.size()*
        (sizeof(frame->features.bow_vector[0])+sizeof(std::pair<basalt::TimeCamId,double>)));
    auto pixels=std::make_shared<basalt::KeypointsData>();pixels->corners=frame->features.corners;
    next->frames.emplace(stamp,Frame{found->second,std::move(pixels),frame->features.bow_vector});
    database->add_to_database(basalt::TimeCamId(stamp,0),frame->features.bow_vector);
  }
  auto pair=[&](const Json& row) {
    const int64_t first=integer(row[0],1,INT64_MAX),second=integer(row[1],1,INT64_MAX);
    if(first<=second || !frames.count(first) || !frames.count(second))
      throw std::invalid_argument("mapping_match_checkpoint_pair");
    return Pair{basalt::TimeCamId(first,0),basalt::TimeCamId(second,0)};
  };
  reserve(data.at("evaluated").size()*sizeof(Pair));
  for(const auto& row:data.at("evaluated")) {
    if(!row.is_array() || row.size()!=2 || !next->evaluated.insert(pair(row)).second)
      throw std::invalid_argument("mapping_match_checkpoint_pair");
  }
  for(const auto& row:data.at("matches")) {
    check();if(!row.is_array() || row.size()!=5)throw std::invalid_argument("mapping_match_checkpoint_shape");
    const auto key=pair(row);
    if(!next->evaluated.count(key) || next->matches.count(key) || !row[2].is_array() ||
        !row[3].is_array() || row[3].empty() || row[3].size()>row[2].size() ||
        row[2].size()>frames.at(key.first.frame_id).metadata().corners ||
        !frames.at(key.second.frame_id).metadata().corners)
      throw std::invalid_argument("mapping_match_checkpoint_pair");
    reserve(sizeof(basalt::MatchData)+(row[2].size()+row[3].size())*sizeof(std::pair<int,int>));
    auto match=std::make_shared<basalt::MatchData>();
    std::set<std::pair<int,int>> originals,inliers;
    auto decode=[&](const Json& rows,auto& output,std::set<std::pair<int,int>>& unique) {
      for(const auto& item:rows) {
        if(!item.is_array() || item.size()!=2)throw std::invalid_argument("mapping_match_checkpoint_pair");
        const auto value=std::make_pair(int(integer(item[0],0,frames.at(key.first.frame_id).metadata().corners-1)),
            int(integer(item[1],0,frames.at(key.second.frame_id).metadata().corners-1)));
        if(!unique.insert(value).second)throw std::invalid_argument("mapping_match_checkpoint_pair");output.push_back(value);
      }
    };
    decode(row[2],match->matches,originals);decode(row[3],match->inliers,inliers);
    if(!std::includes(originals.begin(),originals.end(),inliers.begin(),inliers.end()))
      throw std::invalid_argument("mapping_match_checkpoint_pair");
    match->T_i_j=pose(row[4]);next->matches.emplace(key,std::move(match));
  }
  next->evaluated_pairs=next->evaluated.size();check();Snapshot committed=next;
  database_=std::move(database);std::atomic_store(&data_,committed);return committed;
}
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_graph.hpp"
#include "native_mapping_window.hpp"

namespace posim_slam {
// Companion to the current-state mapping graph, not a replacement for it.
// Keep the original graph once, plus original linearization anchors and active
// membership. Reconstruct sealed information at those anchors, then replace
// only active originals at the saved current state. No dense matrices, images,
// descriptors or additional physical priors are duplicated in this document.
class NativeMappingCheckpoint {
 public:
  using Json=nlohmann::json;
  static Json document(const NativeMappingWindow& window) {
    using namespace mapping_graph_detail;
    if(nativeMappingSourceIdentity(window.mapper_)!=window.source_identity_)
      throw std::invalid_argument("native_window_checkpoint_source_changed");
    Json value={{"schema","posim-slam-native-window-1"},
      {"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
      {"reference",window.chart_.reference_},{"active",window.active_poses_},
      {"source",binary(window.source_identity_)},{"state",binary(nativeMappingStateIdentity(window.mapper_))},
      {"poses",Json::array()},{"points",Json::array()}};
    for(const auto& [stamp,pose]:window.chart_.anchors_)value["poses"].push_back({stamp,doubles(pose.data(),7)});
    for(const auto& [id,point]:window.chart_.points_) {
      const auto host=window.chart_.hosts_.at(id);value["points"].push_back({id,host.frame_id,host.cam_id,doubles(point.data(),3)});
    }
    return {{"model",value},{"sha256",digest(value)}};
  }
  // The caller restores the matching graph first and exclusively owns it.
  // Rejection or cancellation leaves its poses and landmarks byte-identical.
  // Limits remain caller policy, not a value trusted from the saved document.
  static std::unique_ptr<NativeMappingWindow> restore(basalt::NfrMapper& mapper,const Json& saved,
      SubmapInformation::Limits limits={},const std::function<bool()>& cancelled={}) {
    using namespace mapping_graph_detail;
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};check();
    const auto& value=saved.at("model");
    if(value.at("schema")!="posim-slam-native-window-1" ||
        value.at("basalt_revision")!="0f3b2b52c807f70ff4e2973ce253c73329eea7bc")
      throw std::invalid_argument("native_window_checkpoint_schema");
    const auto& poses=value.at("poses");const auto& points=value.at("points");const auto& active=value.at("active");
    if(!poses.is_array() || poses.size()!=mapper.getFramePoses().size() || poses.size()>kMaxJointColumns/6 ||
        !points.is_array() || points.size()!=mapper.lmdb.getLandmarks().size() || points.size()>100000 ||
        !active.is_array() || active.empty() || active.size()>poses.size())
      throw std::invalid_argument("native_window_checkpoint_shape");
    if(saved.at("sha256")!=digest(value))throw std::invalid_argument("native_window_checkpoint_digest");
    if(value.at("source")!=binary(nativeMappingSourceIdentity(mapper)))throw std::invalid_argument("native_window_checkpoint_source");
    if(value.at("state")!=binary(nativeMappingStateIdentity(mapper)))throw std::invalid_argument("native_window_checkpoint_state");
    const auto reference=integer(value.at("reference"),1,INT64_MAX);std::set<int64_t> selected,pose_ids,point_ids;
    for(const auto& stamp:active)if(!selected.insert(integer(stamp,1,INT64_MAX)).second)
      throw std::invalid_argument("native_window_checkpoint_active");
    NativeMappingWindow::Backup backup(mapper);
    for(const auto& row:poses) {
      if(!row.is_array() || row.size()!=2)throw std::invalid_argument("native_window_checkpoint_shape");
      const auto stamp=integer(row[0],1,INT64_MAX);
      if(!mapper.getFramePoses().count(stamp) || !pose_ids.insert(stamp).second)
        throw std::invalid_argument("native_window_checkpoint_pose");
      mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,pose(row[1]));
    }
    for(const auto& row:points) {
      if(!row.is_array() || row.size()!=4)throw std::invalid_argument("native_window_checkpoint_shape");
      const auto id=integer(row[0],0,INT32_MAX);
      if(!mapper.lmdb.getLandmarks().count(id) || !point_ids.insert(id).second)
        throw std::invalid_argument("native_window_checkpoint_point");
      auto& point=mapper.lmdb.getLandmark(id);const auto parameters=doubles(row[3],3);
      if(point.host_kf_id!=basalt::TimeCamId(integer(row[1],1,INT64_MAX),integer(row[2],0,INT32_MAX)) || parameters[2]<0)
        throw std::invalid_argument("native_window_checkpoint_point");
      point.direction=Eigen::Vector2d(parameters[0],parameters[1]);point.inv_dist=parameters[2];
    }
    check();auto window=std::make_unique<NativeMappingWindow>(mapper,reference,selected,limits);
    backup.restore();window->chart_.coordinates(mapper);check();
    window->replaceActive(nativeMappingRows(mapper,&window->selection_),check);
    backup.committed=true;return window;
  }
 private:
  static Json binary(const InputDigest::Value& value) {return Json::binary(std::vector<uint8_t>(value.begin(),value.end()));}
  static Json digest(const Json& value) {
    const auto bytes=Json::to_cbor(value);
    if(bytes.size()>8*1024*1024)throw std::invalid_argument("native_window_checkpoint_capacity");
    InputDigest hash;hash.update(bytes.data(),bytes.size());return binary(hash.finish());
  }
};
} // namespace posim_slam

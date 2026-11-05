// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_graph.hpp"
#include "native_mapping_window.hpp"

namespace posim_slam {
// Companion to the current-state mapping graph, not a replacement for it.
// Keep the original graph once, plus its fixed chart and the source anchors of
// groups retired after nonlinear updates. Replay those retirements, then replace
// only active originals at the saved current state. No dense matrices, images,
// descriptors or additional physical priors are duplicated in this document.
class NativeMappingCheckpoint {
 public:
  using Json=nlohmann::json;
  static Json document(const NativeMappingWindow& window) {
    using namespace mapping_graph_detail;
    if(window.growth_revision_)throw std::invalid_argument("native_window_growth_checkpoint_required");
    if(nativeMappingSourceIdentity(window.mapper_)!=window.source_identity_)
      throw std::invalid_argument("native_window_checkpoint_source_changed");
    const auto state=nativeMappingStateIdentity(window.mapper_);
    if(state!=window.linearization_state_)
      throw std::invalid_argument("native_window_checkpoint_state_changed");
    Json value={{"schema","posim-slam-native-window-2"},
      {"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
      {"reference",window.chart_.reference_},{"active",window.active_poses_},
      {"source",binary(window.source_identity_)},{"state",binary(state)},
      {"initial_active",window.initial_active_poses_},{"retirements",Json::array()},
      {"poses",Json::array()},{"points",Json::array()}};
    for(const auto& [stamp,pose]:window.chart_.anchors_)value["poses"].push_back({stamp,doubles(pose.data(),7)});
    for(const auto& [id,point]:window.chart_.points_) {
      const auto host=window.chart_.hosts_.at(id);value["points"].push_back({id,host.frame_id,host.cam_id,doubles(point.data(),3)});
    }
    for(const auto& record:window.retirements_) {
      Json event={{"kept",record->kept},{"poses",Json::array()},{"points",Json::array()}};
      for(const auto& [stamp,pose]:record->poses)event["poses"].push_back({stamp,doubles(pose.data(),7)});
      for(const auto& [id,point]:record->points)event["points"].push_back({id,doubles(point.data(),3)});
      value["retirements"].push_back(std::move(event));
    }
    return {{"model",value},{"sha256",digest(value)}};
  }
  // Called while the registry captures the same exclusively owned mapper.
  // The resulting graph holds both values even after the live owner advances.
  static Json capture(const NativeMappingWindow& window,basalt::NfrMapper& mapper,int64_t reference) {
    if(&mapper!=&window.mapper_ || reference!=window.chart_.reference_)
      throw std::invalid_argument("native_window_checkpoint_owner");
    return document(window);
  }
  static void validate(basalt::NfrMapper& mapper,const Json& saved,int64_t reference) {
    if(decode(mapper,saved).reference!=reference)
      throw std::invalid_argument("native_window_checkpoint_reference");
  }
  static std::unique_ptr<NativeMappingWindow> restore(basalt::NfrMapper& mapper,
      const MappingGraphRegistry::Snapshot& graph,SubmapInformation::Limits limits={},
      const std::function<bool()>& cancelled={}) {
    if(!graph || !graph->validated || !graph->hasNativeWindow())
      throw std::invalid_argument("native_window_checkpoint_missing");
    return restore(mapper,graph->native_window,limits,cancelled);
  }
  // The caller restores the matching graph first and exclusively owns it.
  // Rejection or cancellation leaves its poses and landmarks byte-identical.
  // Limits remain caller policy, not a value trusted from the saved document.
  static std::unique_ptr<NativeMappingWindow> restore(basalt::NfrMapper& mapper,const Json& saved,
      SubmapInformation::Limits limits={},const std::function<bool()>& cancelled={}) {
    using namespace mapping_graph_detail;
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};check();
    const auto parsed=decode(mapper,saved);
    NativeMappingWindow::Backup backup(mapper);
    for(const auto& [stamp,anchor]:parsed.poses)
      mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,anchor);
    for(const auto& [id,parameters]:parsed.points) {
      auto& point=mapper.lmdb.getLandmark(id);point.direction=parameters.head<2>();point.inv_dist=parameters[2];
    }
    check();auto window=std::make_unique<NativeMappingWindow>(mapper,parsed.reference,parsed.initial_active,limits);
    for(const auto& record:parsed.retirements) {
      check();auto plan=window->retirementPlan(record->kept);
      for(const auto& [stamp,anchor]:record->poses)
        mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,anchor);
      for(const auto& [id,value]:record->points) {
        auto& point=mapper.lmdb.getLandmark(id);point.direction=value.head<2>();point.inv_dist=value.z();
      }
      // Only this event's originals are relinearized. Other historical groups
      // retain their own anchors; remaining active groups are restored below.
      if(!plan.originals.empty())window->replaceSelected(nativeMappingRows(mapper,&plan.selected),plan.originals,check);
      check();window->commitRetirement(std::move(plan),record);
    }
    backup.restore();window->chart_.coordinates(mapper);check();
    window->replaceActive(nativeMappingRows(mapper,&window->selection_),check);
    backup.committed=true;return window;
  }
 private:
  struct Parsed {
    int64_t reference;
    std::set<int64_t> active,initial_active;
    Eigen::aligned_map<int64_t,Sophus::SE3d> poses;
    Eigen::aligned_map<int64_t,Eigen::Vector3d> points;
    std::vector<std::shared_ptr<const NativeMappingWindow::Retirement>> retirements;
  };
  // Decode once without mutating the supplied mapper. Archive reads and native
  // restoration enforce the same bindings and structural checks.
  static Parsed decode(basalt::NfrMapper& mapper,const Json& saved) {
    using namespace mapping_graph_detail;
    const auto& value=saved.at("model");
    const bool legacy=value.at("schema")=="posim-slam-native-window-1";
    if((!legacy && value.at("schema")!="posim-slam-native-window-2") ||
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
    Parsed result;result.reference=integer(value.at("reference"),1,INT64_MAX);
    if(!mapper.getFramePoses().count(result.reference))throw std::invalid_argument("native_window_checkpoint_reference");
    auto active_set=[&](const Json& input) {
      if(!input.is_array() || input.empty() || input.size()>poses.size())
        throw std::invalid_argument("native_window_checkpoint_shape");
      std::set<int64_t> out;
      for(const auto& stamp:input)if(!out.insert(integer(stamp,1,INT64_MAX)).second)
        throw std::invalid_argument("native_window_checkpoint_active");
      for(auto stamp:out)if(!mapper.getFramePoses().count(stamp))throw std::invalid_argument("native_window_unknown_pose");
      return out;
    };
    result.active=active_set(active);
    result.initial_active=legacy?result.active:active_set(value.at("initial_active"));
    for(const auto& row:poses) {
      if(!row.is_array() || row.size()!=2)throw std::invalid_argument("native_window_checkpoint_shape");
      const auto stamp=integer(row[0],1,INT64_MAX);
      if(!mapper.getFramePoses().count(stamp) || !result.poses.emplace(stamp,pose(row[1])).second)
        throw std::invalid_argument("native_window_checkpoint_pose");
    }
    for(const auto& row:points) {
      if(!row.is_array() || row.size()!=4)throw std::invalid_argument("native_window_checkpoint_shape");
      const auto id=integer(row[0],0,INT32_MAX);
      if(!mapper.lmdb.getLandmarks().count(id) || result.points.count(id))
        throw std::invalid_argument("native_window_checkpoint_point");
      const auto& point=mapper.lmdb.getLandmark(id);const auto parameters=doubles(row[3],3);
      if(point.host_kf_id!=basalt::TimeCamId(integer(row[1],1,INT64_MAX),integer(row[2],0,INT32_MAX)) || parameters[2]<0)
        throw std::invalid_argument("native_window_checkpoint_point");
      result.points.emplace(id,Eigen::Vector3d(parameters[0],parameters[1],parameters[2]));
    }
    if(!legacy) {
      const auto& events=value.at("retirements");
      if(!events.is_array() || events.size()>=result.initial_active.size())
        throw std::invalid_argument("native_window_checkpoint_retirements");
      auto previous=result.initial_active;const NativeMappingSchedule schedule(mapper,result.initial_active);
      for(const auto& event:events) {
        auto record=std::make_shared<NativeMappingWindow::Retirement>();record->kept=active_set(event.at("kept"));
        if(record->kept.size()>=previous.size() ||
            !std::includes(previous.begin(),previous.end(),record->kept.begin(),record->kept.end()))
          throw std::invalid_argument("native_window_retirement_subset");
        std::set<int64_t> support,points;size_t remaining=0;
        for(const auto& group:schedule.groups) {
          if(!std::includes(previous.begin(),previous.end(),group.poses.begin(),group.poses.end()))continue;
          if(std::includes(record->kept.begin(),record->kept.end(),group.poses.begin(),group.poses.end()))++remaining;
          else {
            support.insert(group.poses.begin(),group.poses.end());
            if(group.source.kind==NativeMappingRowSource::landmark)points.insert(group.source.id);
          }
        }
        if(!remaining)throw std::invalid_argument("native_window_no_active_likelihoods");
        const auto& pose_rows=event.at("poses");const auto& point_rows=event.at("points");
        if(!pose_rows.is_array() || pose_rows.size()!=support.size() ||
            !point_rows.is_array() || point_rows.size()!=points.size())
          throw std::invalid_argument("native_window_retirement_anchors");
        for(const auto& row:pose_rows) {
          if(!row.is_array() || row.size()!=2)throw std::invalid_argument("native_window_checkpoint_shape");
          const auto stamp=integer(row[0],1,INT64_MAX);
          if(!support.count(stamp) || !record->poses.emplace(stamp,pose(row[1])).second)
            throw std::invalid_argument("native_window_retirement_anchors");
        }
        for(const auto& row:point_rows) {
          if(!row.is_array() || row.size()!=2)throw std::invalid_argument("native_window_checkpoint_shape");
          const auto id=integer(row[0],0,INT32_MAX);const auto parameters=doubles(row[1],3);
          if(!points.count(id) || parameters[2]<0 ||
              !record->points.emplace(id,Eigen::Vector3d(parameters[0],parameters[1],parameters[2])).second)
            throw std::invalid_argument("native_window_retirement_anchors");
        }
        previous=record->kept;result.retirements.push_back(std::move(record));
      }
      if(previous!=result.active)throw std::invalid_argument("native_window_checkpoint_active");
    }
    return result;
  }
  static Json binary(const InputDigest::Value& value) {return Json::binary(std::vector<uint8_t>(value.begin(),value.end()));}
  static Json digest(const Json& value) {
    const auto bytes=Json::to_cbor(value);
    if(bytes.size()>8*1024*1024)throw std::invalid_argument("native_window_checkpoint_capacity");
    InputDigest hash;hash.update(bytes.data(),bytes.size());return binary(hash.finish());
  }
};
} // namespace posim_slam

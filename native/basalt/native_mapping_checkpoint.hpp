// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_graph.hpp"
#include "native_mapping_window.hpp"

namespace posim_slam {
// Source anchors accompany one immutable current graph. Restore original
// groups at their own last linearization, then rebuild their saved elimination
// order. Numeric caches and old event histories are not duplicated. Schemas 1/2
// remain readable through the original fixed-source retirement decoder.
// Schema 4 also retains strict supernodes of complete six-coordinate poses;
// schema 3's single-pose elimination blocks remain readable.
class NativeMappingCheckpoint {
 public:
  using Json=nlohmann::json;
  using State=NativeMappingWindowState;
  static std::shared_ptr<const State> captureState(const NativeMappingWindow& window,
      basalt::NfrMapper& mapper,int64_t reference) {
    if(&mapper!=&window.mapper_ || reference!=window.chart_.reference_)
      throw std::invalid_argument("native_window_checkpoint_owner");
    if(nativeMappingSourceIdentity(mapper)!=window.source_identity_)
      throw std::invalid_argument("native_window_checkpoint_source_changed");
    const auto nonlinear=nativeMappingStateIdentity(mapper);
    if(nonlinear!=window.linearization_state_)throw std::invalid_argument("native_window_checkpoint_state_changed");
    auto state=std::make_shared<State>(window.chart_);const auto numeric=window.snapshot();
    state->active=window.active_poses_;state->layout=numeric->layout;state->anchors=window.source_anchors_;
    state->source=window.source_identity_;state->nonlinear=nonlinear;
    numeric->eachUnassigned([&](const auto& key){state->unassigned.push_back(key);});
    if(state->anchors.size()!=numeric->factors.size())throw std::logic_error("native_window_source_anchors");
    return state;
  }
  static Json document(const NativeMappingWindow& window) {
    return document(*captureState(window,window.mapper_,window.chart_.reference()));
  }
  static Json document(const State& state) {
    using namespace mapping_graph_detail;
    Json value={{"schema","posim-slam-native-window-4"},
      {"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
      {"reference",state.chart.reference_},{"active",state.active},
      {"source",binary(state.source)},{"state",binary(state.nonlinear)},
      {"pose_order",state.chart.poses_},{"poses",Json::array()},{"points",Json::array()},
      {"layout",{state.layout.eliminated,state.layout.frontier}},
      {"unassigned",state.unassigned},{"groups",Json::array()}};
    for(const auto& [stamp,pose]:state.chart.anchors_)value["poses"].push_back({stamp,doubles(pose.data(),7)});
    for(const auto& [id,point]:state.chart.points_) {
      const auto host=state.chart.hosts_.at(id);value["points"].push_back({id,host.frame_id,host.cam_id,doubles(point.data(),3)});
    }
    for(const auto& [id,anchor]:state.anchors) {
      Json poses=Json::array();for(const auto& [stamp,pose]:anchor->poses)poses.push_back({stamp,doubles(pose.data(),7)});
      value["groups"].push_back({id,std::move(poses),doubles(anchor->point.data(),anchor->has_point?3:0)});
    }
    return {{"model",value},{"sha256",digest(value)}};
  }
  static Json capture(const NativeMappingWindow& window,basalt::NfrMapper& mapper,int64_t reference) {
    return document(*captureState(window,mapper,reference));
  }
  static void validate(basalt::NfrMapper& mapper,const Json& saved,int64_t reference) {
    if(decode(mapper,saved).reference!=reference)throw std::invalid_argument("native_window_checkpoint_reference");
  }
  static std::unique_ptr<NativeMappingWindow> restore(basalt::NfrMapper& mapper,
      const MappingGraphRegistry::Snapshot& graph,SubmapInformation::Limits limits={},
      const std::function<bool()>& cancelled={},std::shared_ptr<SubmapRowStore> row_store={}) {
    if(!graph || !graph->validated || !graph->hasNativeWindow())
      throw std::invalid_argument("native_window_checkpoint_missing");
    return restore(mapper,graph->native_window_state?document(*graph->native_window_state):graph->native_window,limits,cancelled,std::move(row_store));
  }
  // The caller restores the matching graph first and exclusively owns it.
  // Rejection or cancellation leaves its poses and landmarks byte-identical.
  // Limits remain caller policy, not a value trusted from the saved document.
  static std::unique_ptr<NativeMappingWindow> restore(basalt::NfrMapper& mapper,const Json& saved,
      SubmapInformation::Limits limits={},const std::function<bool()>& cancelled={},std::shared_ptr<SubmapRowStore> row_store={}) {
    using namespace mapping_graph_detail;
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};check();
    const auto parsed=decode(mapper,saved);
    if(parsed.source_owned)return restoreSources(mapper,parsed,limits,cancelled,std::move(row_store));
    NativeMappingWindow::Backup backup(mapper);
    for(const auto& [stamp,anchor]:parsed.poses)
      mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,anchor);
    for(const auto& [id,parameters]:parsed.points) {
      auto& point=mapper.lmdb.getLandmark(id);point.direction=parameters.head<2>();point.inv_dist=parameters[2];
    }
    check();auto window=std::make_unique<NativeMappingWindow>(mapper,parsed.reference,parsed.initial_active,limits,std::move(row_store));
    for(const auto& record:parsed.retirements) {
      check();auto plan=window->retirementPlan(record->kept);
      for(const auto& [stamp,anchor]:record->poses)
        mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,anchor);
      for(const auto& [id,value]:record->points) {
        auto& point=mapper.lmdb.getLandmark(id);point.direction=value.head<2>();point.inv_dist=value.z();
      }
      // Only this event's originals are relinearized. Other historical groups
      // retain their own anchors; remaining active groups are restored below.
      if(!plan.originals.empty())window->replaceSelected(nativeMappingRows(mapper,&plan.selected,limits.coordinates),plan.originals,check);
      check();window->commitRetirement(std::move(plan));
    }
    backup.restore();window->chart_.coordinates(mapper);check();
    window->replaceActive(nativeMappingRows(mapper,&window->selection_,limits.coordinates),check);
    backup.committed=true;return window;
  }
 private:
  struct Parsed {
    int64_t reference;
    std::set<int64_t> active,initial_active;
    Eigen::aligned_map<int64_t,Sophus::SE3d> poses;
    Eigen::aligned_map<int64_t,Eigen::Vector3d> points;
    std::vector<std::shared_ptr<const NativeMappingWindow::Retirement>> retirements;
    bool source_owned=false;
    std::vector<int64_t> pose_order;
    SubmapFactorization::Layout layout;
    std::map<std::string,std::shared_ptr<const NativeMappingSourceAnchor>> anchors;
    std::vector<std::string> unassigned;
  };
  static std::unique_ptr<NativeMappingWindow> restoreSources(basalt::NfrMapper& mapper,
      const Parsed& parsed,SubmapInformation::Limits limits,const std::function<bool()>& cancelled,
      std::shared_ptr<SubmapRowStore> row_store) {
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};check();
    NativeMappingChart chart(mapper,parsed.reference,limits.coordinates);chart.anchors_=parsed.poses;
    chart.points_.clear();chart.points_.insert(parsed.points.begin(),parsed.points.end());
    chart.poses_=parsed.pose_order;chart.offsets_=nativeMappingPoseAddresses(chart.poses_,limits.coordinates);
    chart.reference_angles_=chart.angles(chart.anchors_.at(parsed.reference));chart.coordinates(mapper);
    auto window=std::unique_ptr<NativeMappingWindow>(new NativeMappingWindow(mapper,std::move(chart),parsed.active,
        limits,&parsed.layout,parsed.unassigned,std::move(row_store)));
    const NativeMappingSchedule schedule(mapper,parsed.active,limits.coordinates);NativeMappingWindow::Backup backup(mapper);
    std::vector<SubmapFactorization::Factor> factors;
    for(const auto& group:schedule.groups) {
      check();const auto id=window->groupId(group.source);const auto& anchor=*parsed.anchors.at(id);
      for(const auto& [stamp,pose]:anchor.poses)mapper.getFramePoses().at(stamp)=basalt::PoseStateWithLin<double>(stamp,pose);
      if(anchor.has_point) {
        auto& point=mapper.lmdb.getLandmark(group.source.id);point.direction=anchor.point.head<2>();point.inv_dist=anchor.point.z();
      }
      const std::vector<NativeMappingRowSource> order{group.source};
      NativeMappingSelection selected;
      if(group.source.kind==NativeMappingRowSource::landmark)selected.points.insert(group.source.id);
      else selected.factors.insert(id.substr(7));
      window->chart_.streamRows(mapper,[&](const NativeMappingPoseRows& row) {
        factors.push_back({id,nativeMappingFactor(row,window->chart_.offsets())});
        if(group.active)window->selection_.factors.insert(row.original);
      },[&](const NativeMappingPointRows& row) {
        auto reduced=reduceNativeMappingPoint(row,window->chart_.offsets());
        factors.push_back({id,reduced.likelihood});window->points_.emplace(row.id,NativeMappingWindow::Conditional(std::move(reduced)));
        if(group.active)window->selection_.points.insert(row.id);
      },&order,&selected);
    }
    check();window->owner_->replace({},factors,{},nullptr,cancelled);
    window->source_anchors_=parsed.anchors;window->originals_=window->snapshot()->information->measurements.size();
    backup.restore();window->chart_.coordinates(mapper);check();
    if(window->numericBytes()>limits.numeric_bytes)throw std::runtime_error("native_window_numeric_capacity");
    backup.committed=true;return window;
  }
  // Decode once without mutating the supplied mapper. Archive reads and native
  // restoration enforce the same bindings and structural checks.
  static Parsed decode(basalt::NfrMapper& mapper,const Json& saved) {
    using namespace mapping_graph_detail;
    const auto& value=saved.at("model");
    const bool legacy=value.at("schema")=="posim-slam-native-window-1";
    const bool source_owned=value.at("schema")=="posim-slam-native-window-3" || value.at("schema")=="posim-slam-native-window-4";
    if((!legacy && !source_owned && value.at("schema")!="posim-slam-native-window-2") ||
        value.at("basalt_revision")!="0f3b2b52c807f70ff4e2973ce253c73329eea7bc")
      throw std::invalid_argument("native_window_checkpoint_schema");
    const auto& poses=value.at("poses");const auto& points=value.at("points");const auto& active=value.at("active");
    if(!poses.is_array() || poses.size()!=mapper.getFramePoses().size() || poses.size()>kMaxNativeMappingPoseCoordinates/6 ||
        !points.is_array() || points.size()!=mapper.lmdb.getLandmarks().size() || points.size()>100000 ||
        !active.is_array() || active.empty() || active.size()>poses.size())
      throw std::invalid_argument("native_window_checkpoint_shape");
    if(saved.at("sha256")!=digest(value))throw std::invalid_argument("native_window_checkpoint_digest");
    if(value.at("source")!=binary(nativeMappingSourceIdentity(mapper)))throw std::invalid_argument("native_window_checkpoint_source");
    if(value.at("state")!=binary(nativeMappingStateIdentity(mapper)))throw std::invalid_argument("native_window_checkpoint_state");
    Parsed result;result.source_owned=source_owned;result.reference=integer(value.at("reference"),1,INT64_MAX);
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
    result.initial_active=(legacy || source_owned)?result.active:active_set(value.at("initial_active"));
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
    if(source_owned) {
      decodeSources(mapper,value,result);return result;
    }
    if(!legacy) {
      const auto& events=value.at("retirements");
      if(!events.is_array() || events.size()>=result.initial_active.size())
        throw std::invalid_argument("native_window_checkpoint_retirements");
      auto previous=result.initial_active;const NativeMappingSchedule schedule(mapper,result.initial_active,kMaxNativeMappingPoseCoordinates);
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
  static void decodeSources(basalt::NfrMapper& mapper,const Json& value,Parsed& out) {
    using namespace mapping_graph_detail;
    const auto& order=value.at("pose_order");
    if(!order.is_array() || order.size()!=out.poses.size())throw std::invalid_argument("native_window_checkpoint_pose_order");
    for(const auto& stamp:order)out.pose_order.push_back(integer(stamp,1,INT64_MAX));
    const auto offsets=nativeMappingPoseAddresses(out.pose_order);
    for(const auto& [stamp,index]:offsets)if(!out.poses.count(stamp))throw std::invalid_argument("native_window_checkpoint_pose_order");
    const auto& layout=value.at("layout");
    if(!layout.is_array() || layout.size()!=2 || !layout[0].is_array() || !layout[1].is_array() ||
        layout[0].size()+out.active.size()>out.poses.size() || layout[1].size()!=6*out.active.size())
      throw std::invalid_argument("native_window_checkpoint_layout");
    std::set<int64_t> seen;
    for(const auto& block:layout[0]) {
      if(!block.is_array() || block.empty() || block.size()%6 || block.size()>size_t(kMaxJointColumns) ||
          (value.at("schema")=="posim-slam-native-window-3" && block.size()!=6))
        throw std::invalid_argument("native_window_checkpoint_layout");
      std::vector<int64_t> columns;
      for(const auto& item:block)columns.push_back(integer(item,0,6*out.poses.size()-1));
      for(size_t i=0;i<columns.size();i+=6) {
        const auto stamp=out.pose_order.at(columns[i]/6);const auto expected=nativeMappingColumns({stamp},offsets);
        if(out.active.count(stamp) || !seen.insert(stamp).second || !std::equal(expected.begin(),expected.end(),columns.begin()+i))
          throw std::invalid_argument("native_window_checkpoint_layout");
      }
      out.layout.eliminated.push_back(std::move(columns));
    }
    if(seen.size()+out.active.size()!=out.poses.size())throw std::invalid_argument("native_window_checkpoint_layout");
    std::set<int64_t> actual;
    for(const auto& item:layout[1]) {
      const auto column=integer(item,0,6*out.poses.size()-1);
      if(!out.active.count(out.pose_order.at(column/6)) || !actual.insert(column).second)
        throw std::invalid_argument("native_window_checkpoint_layout");
      out.layout.frontier.push_back(column);
    }
    const NativeMappingSchedule schedule(mapper,out.active,kMaxNativeMappingPoseCoordinates);
    std::map<std::string,std::pair<std::set<int64_t>,bool>> expected;
    for(const auto& group:schedule.groups) {
      const auto& source=group.source;std::string id;
      if(source.kind==NativeMappingRowSource::landmark)id="point:"+std::to_string(source.id);
      else if(source.kind==NativeMappingRowSource::relative) {
        const auto& f=mapper.rel_pose_factors.at(source.id);id="factor:relative:"+std::to_string(f.t_i_ns)+":"+std::to_string(f.t_j_ns);
      } else id="factor:roll-pitch:"+std::to_string(mapper.roll_pitch_factors.at(source.id).t_ns);
      expected.emplace(id,std::make_pair(std::set<int64_t>(group.poses.begin(),group.poses.end()),source.kind==NativeMappingRowSource::landmark));
    }
    const auto& groups=value.at("groups");
    if(!groups.is_array() || groups.size()!=expected.size())throw std::invalid_argument("native_window_checkpoint_groups");
    for(const auto& row:groups) {
      if(!row.is_array() || row.size()!=3 || !row[0].is_string() || !row[1].is_array() || !row[2].is_binary())
        throw std::invalid_argument("native_window_checkpoint_groups");
      const auto id=row[0].get<std::string>();const auto found=expected.find(id);
      if(found==expected.end() || out.anchors.count(id) || row[1].size()!=found->second.first.size())
        throw std::invalid_argument("native_window_checkpoint_groups");
      auto anchor=std::make_shared<NativeMappingSourceAnchor>();anchor->has_point=found->second.second;
      for(const auto& saved:row[1]) {
        if(!saved.is_array() || saved.size()!=2)throw std::invalid_argument("native_window_checkpoint_anchors");
        const auto stamp=integer(saved[0],1,INT64_MAX);
        if(!found->second.first.count(stamp) || !anchor->poses.emplace(stamp,pose(saved[1])).second)
          throw std::invalid_argument("native_window_checkpoint_anchors");
      }
      const auto point=doubles(row[2],anchor->has_point?3:0);
      if(anchor->has_point) {
        if(point[2]<0)throw std::invalid_argument("native_window_checkpoint_point");
        anchor->point={point[0],point[1],point[2]};
      }
      out.anchors.emplace(id,std::move(anchor));
    }
    std::set<std::string> assigned;
    for(const auto& [id,group]:expected)if(id.compare(0,7,"factor:")==0)assigned.insert(id.substr(7));
    for(const auto& [id,point]:mapper.lmdb.getLandmarks())for(const auto& [image,pixel]:point.obs)
      assigned.insert("pixel:"+std::to_string(image.frame_id)+":"+std::to_string(image.cam_id)+":"+
                      std::to_string(mapper.feature_tracks.at(id).at(image)));
    const auto& extra=value.at("unassigned");
    if(!extra.is_array() || extra.size()>1000000)throw std::invalid_argument("native_window_checkpoint_originals");
    std::set<std::string> unused;
    for(const auto& item:extra) {
      if(!item.is_string())throw std::invalid_argument("native_window_checkpoint_originals");
      const auto key=item.get<std::string>();
      if(key.empty() || key.back()==':' || key.size()>256 || assigned.count(key) || !unused.insert(key).second)
        throw std::invalid_argument("native_window_checkpoint_originals");
      std::vector<std::string> fields;std::string field;std::istringstream stream(key);
      while(std::getline(stream,field,':'))fields.push_back(field);
      auto number=[&](size_t index,int64_t minimum,int64_t maximum) {
        int64_t result=0;
        try {result=std::stoll(fields.at(index));}
        catch(const std::exception&) {throw std::invalid_argument("native_window_checkpoint_originals");}
        if(result<minimum || result>maximum || std::to_string(result)!=fields[index])
          throw std::invalid_argument("native_window_checkpoint_originals");
        return result;
      };
      if(fields.size()==4 && fields[0]=="pixel") {
        const auto stamp=number(1,1,INT64_MAX),camera=number(2,0,INT32_MAX),feature=number(3,0,INT32_MAX);
        const auto frame=mapper.feature_corners.find(basalt::TimeCamId(stamp,camera));
        if(frame==mapper.feature_corners.end() || size_t(feature)>=frame->second.corners.size())
          throw std::invalid_argument("native_window_checkpoint_originals");
      } else if(fields.size()==3 && fields[0]=="relative") {
        if(!out.poses.count(number(1,1,INT64_MAX)) || !out.poses.count(number(2,1,INT64_MAX)))
          throw std::invalid_argument("native_window_checkpoint_originals");
      } else if(fields.size()==2 && fields[0]=="roll-pitch") {
        if(!out.poses.count(number(1,1,INT64_MAX)))throw std::invalid_argument("native_window_checkpoint_originals");
      } else throw std::invalid_argument("native_window_checkpoint_originals");
      out.unassigned.push_back(key);
    }
  }
  static Json binary(const InputDigest::Value& value) {return Json::binary(std::vector<uint8_t>(value.begin(),value.end()));}
  static Json digest(const Json& value) {
    const auto bytes=Json::to_cbor(value);
    if(bytes.size()>8*1024*1024)throw std::invalid_argument("native_window_checkpoint_capacity");
    InputDigest hash;hash.update(bytes.data(),bytes.size());return binary(hash.finish());
  }
};
} // namespace posim_slam

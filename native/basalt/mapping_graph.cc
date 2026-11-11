// SPDX-License-Identifier: Apache-2.0
#include "mapping_graph.hpp"
#include "native_mapping_checkpoint.hpp"
#include "mapping_frame_io.hpp"
#include "native_mapping_graph.hpp"
#include <algorithm>
#include <cstring>
#include <limits>

namespace posim_slam {
namespace {
using namespace mapping_graph_detail;
void count(const Json& value,size_t maximum,size_t minimum=0) {
  if(!value.is_array() || value.size()<minimum || value.size()>maximum)throw std::invalid_argument("mapping_graph_capacity");
}
template<int N> Eigen::Matrix<double,N,N> information(const Json& value) {
  const auto raw=doubles(value,N*N);Eigen::Matrix<double,N,N> out=Eigen::Map<const Eigen::Matrix<double,N,N>>(raw.data());
  if((out-out.transpose()).norm()>1e-10*std::max(1.,out.norm()) || Eigen::LLT<Eigen::Matrix<double,N,N>>(out).info()!=Eigen::Success)
    throw std::invalid_argument("mapping_graph_information");return out;
}
constexpr size_t archive_limit=96*1024*1024;
size_t frameCapacity(const MappingGraphRegistry::Snapshot& graph) {
  return graph && graph->hasNativeWindow()?kMaxMappingHistoryFrames:size_t(kMaxJointColumns/6);
}
constexpr char magic[8]={'P','S','I','M','M','A','P','1'};
constexpr char window_magic[8]={'P','S','I','M','M','A','P','2'};
constexpr char input_magic[8]={'P','S','I','M','M','A','P','3'};
constexpr char owner_magic[8]={'P','S','I','M','M','A','P','4'};
void validPose(const Sophus::SE3d& value) {
  if(!Eigen::Map<const Eigen::Matrix<double,7,1>>(value.data()).allFinite())throw std::invalid_argument("mapping_graph_nonfinite_input");
  if(std::abs(Eigen::Map<const Eigen::Vector4d>(value.data()).squaredNorm()-1)>1e-10)throw std::invalid_argument("mapping_graph_rotation");
}
template<int N> void validInformation(const Eigen::Matrix<double,N,N>& value) {
  if(!value.allFinite())throw std::invalid_argument("mapping_graph_nonfinite_input");
  if((value-value.transpose()).norm()>1e-10*std::max(1.,value.norm()) || Eigen::LLT<Eigen::Matrix<double,N,N>>(value).info()!=Eigen::Success)
    throw std::invalid_argument("mapping_graph_information");
}
} // namespace

MappingGraphRegistry::Snapshot MappingGraphRegistry::capture(const MappingInputStore::Snapshot& input,basalt::NfrMapper& mapper,
    int64_t reference_stamp,const Sophus::SE3d& reference_pose,const Sophus::SE3d& gauge,
    const NativeMappingWindow* window,NativeMappingGraph* owner) {
  using namespace mapping_graph_detail;
  if(!input || input->revision<last_input_ || !mapper.getFramePoses().count(reference_stamp) ||
      mapper.getFramePoses().size()!=input->frames.size() || mapper.feature_tracks.size()>100000 ||
      mapping_graph_detail::config(mapper.config)!=mapping_graph_detail::config(config_))
    throw std::invalid_argument("mapping_graph_source");
  // The archived calibration/configuration must describe the live likelihood,
  // including fields that NfrMapper copies out of its configuration.
  if(mapper.obs_std_dev!=config_.mapper_obs_std_dev || mapper.huber_thresh!=config_.mapper_obs_huber_thresh ||
      mapper.calib.intrinsics.size()!=1 || mapper.calib.T_i_c.size()!=1 || mapper.calib.resolution.size()!=1 ||
      mapper.calib.intrinsics[0].getName()!=calibration_.intrinsics[0].getName() ||
      mapper.calib.intrinsics[0].getParam()!=calibration_.intrinsics[0].getParam() ||
      !mapper.calib.T_i_c[0].matrix().isApprox(calibration_.T_i_c[0].matrix(),0) ||
      mapper.calib.resolution[0]!=calibration_.resolution[0])
    throw std::invalid_argument("mapping_graph_source");
  if(input->revision<1 || input->revision>512 || revision_>=uint64_t(INT64_MAX))throw std::invalid_argument("mapping_graph_integer");
  const size_t maximum_frames=(window || (owner && owner->window()))?kMaxMappingHistoryFrames:size_t(kMaxJointColumns/6);
  if(input->frames.size()<2 || input->frames.size()>maximum_frames || mapper.rel_pose_factors.size()>65536 ||
      mapper.roll_pitch_factors.size()>512 || mapper.lmdb.getLandmarks().size()>mapper.feature_tracks.size())
    throw std::invalid_argument("mapping_graph_capacity");
  mapping_graph_detail::config(mapping_graph_detail::config(config_));
  if(!calibration_.intrinsics[0].getParam().allFinite())throw std::invalid_argument("mapping_graph_nonfinite_input");
  validPose(calibration_.T_i_c[0]);validPose(reference_pose);validPose(gauge);
  auto data=std::make_shared<MappingGraph::Captured>();auto result=std::make_shared<MappingGraph>();
  data->epoch=epoch_;data->input_revision=input->revision;data->graph_revision=revision_+1;
  data->calibration=calibration_;data->config=config_;data->reference_stamp=reference_stamp;data->reference_pose=reference_pose;data->gauge=gauge;
  data->optimizer={mapper.lambda,mapper.min_lambda,mapper.max_lambda,mapper.lambda_vee};
  for(double value:data->optimizer)if(!std::isfinite(value) || value<0)throw std::invalid_argument("mapping_graph_optimizer");
  size_t corners=0;
  for(const auto& [stamp,reference]:input->frames) {
    const auto frame=reference.read();
    if(!frame || stamp<1 || frame->stamp!=stamp || frame->features.corners.size()>1000000 || (corners+=frame->features.corners.size())>1000000)
      throw std::invalid_argument("mapping_graph_frame_identity");
    if(frame->width<1 || frame->width>1920 || frame->height<1 || frame->height>1080 ||
        calibration_.resolution[0]!=Eigen::Vector2i(frame->width,frame->height))throw std::invalid_argument("mapping_graph_frame_shape");
    const auto value=mapper.getFramePoses().at(stamp).getPose();validPose(value);
    data->frames.push_back({stamp,frame->width,frame->height,frame->features.corners.size(),frame->digest});data->poses.emplace_back(stamp,value);
    // Validate once per leased frame. The observation loop then uses the
    // mapper's checked pixels, without a disk read for every observation.
    const auto& mapped=mapper.feature_corners.at(basalt::TimeCamId(stamp,0)).corners;
    if(mapped.size()!=frame->features.corners.size())throw std::invalid_argument("mapping_graph_observation_source");
    for(size_t i=0;i<mapped.size();++i)if(!same(mapped[i],frame->features.corners[i]))
      throw std::invalid_argument("mapping_source_pixel_changed");
    data->inputs.emplace(stamp,reference);
  }
  if(!input->frames.count(reference_stamp))throw std::invalid_argument("mapping_graph_reference");
  std::set<std::pair<int64_t,int64_t>> factor_ids;
  for(const auto& factor:mapper.rel_pose_factors) {
    if(factor.t_i_ns==factor.t_j_ns || !input->frames.count(factor.t_i_ns) || !input->frames.count(factor.t_j_ns) ||
        !factor_ids.emplace(factor.t_i_ns,factor.t_j_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
    validPose(factor.T_i_j);validInformation<6>(factor.cov_inv);data->relative.push_back(factor);
  }
  std::set<int64_t> roll_ids;
  for(const auto& factor:mapper.roll_pitch_factors) {
    if(!input->frames.count(factor.t_ns) || !roll_ids.insert(factor.t_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
    const Eigen::Map<const Eigen::Vector4d> rotation(factor.R_w_i_meas.data());
    if(!rotation.allFinite())throw std::invalid_argument("mapping_graph_nonfinite_input");
    if(std::abs(rotation.squaredNorm()-1)>1e-10)throw std::invalid_argument("mapping_graph_rotation");
    validInformation<2>(factor.cov_inv);data->roll_pitch.push_back(factor);
  }
  auto anchors=std::make_shared<MappingGraph::Ledger>(*anchors_);int next_id=next_id_;
  std::map<MappingObservation,size_t> owners;
  std::map<MappingObservation,std::pair<int64_t,const basalt::FeatureTrack*>> ordered;
  for(const auto& [native,track]:mapper.feature_tracks) {
    if(native<0 || native>INT32_MAX || track.size()<2 || track.size()>2048)throw std::invalid_argument("mapping_graph_track");
    const auto& first=*track.begin();MappingObservation anchor{first.first.frame_id,first.first.cam_id,first.second};
    if(!ordered.emplace(anchor,std::make_pair(native,&track)).second)throw std::invalid_argument("mapping_graph_observation_reused");
  }
  for(const auto& [anchor,item]:ordered) {
    const auto [native,track]=item;auto found=anchors->find(anchor);
    if(found==anchors->end()) {
      if(anchors->size()>=100000 || next_id>=100000)throw std::runtime_error("mapping_graph_identity_capacity");
      found=anchors->emplace(anchor,next_id++).first;
    }
    MappingGraph::Captured::Track saved{native,found->second,{}};saved.pixels.reserve(track->size());result->stable_ids.emplace(native,found->second);
    for(const auto& [image,feature]:*track) {
      const auto frame=input->frames.find(image.frame_id);
      if(image.cam_id!=0 || frame==input->frames.end() || feature<0 || size_t(feature)>=frame->second.metadata().corners)
        throw std::invalid_argument("mapping_graph_observation_source");
      const auto& pixel=mapper.feature_corners.at(image).corners.at(feature);
      if(!pixel.allFinite())throw std::invalid_argument("mapping_source_pixel_changed");
      MappingObservation identity{image.frame_id,image.cam_id,feature};
      if(owners.size()>=1000000 || !owners.emplace(identity,data->pixels.size()).second)throw std::invalid_argument("mapping_graph_observation_reused");
      saved.pixels.push_back(data->pixels.size());data->pixels.push_back({identity,pixel});
    }
    data->tracks.push_back(std::move(saved));
  }
  for(const auto& [native,landmark]:mapper.lmdb.getLandmarks()) {
    if(!result->stable_ids.count(native))throw std::invalid_argument("mapping_graph_landmark_track");
    const auto& track=mapper.feature_tracks.at(native);
    if(!track.count(landmark.host_kf_id))throw std::invalid_argument("mapping_graph_landmark_track");
    if(landmark.obs.size()>2048)throw std::invalid_argument("mapping_graph_capacity");
    MappingGraph::Captured::Landmark saved{int64_t(native),landmark.host_kf_id.frame_id,int(landmark.host_kf_id.cam_id),
      Eigen::Vector3d(landmark.direction.x(),landmark.direction.y(),landmark.inv_dist),{}};
    saved.pixels.reserve(landmark.obs.size());
    for(const auto& [image,pixel]:landmark.obs) {
      const auto feature=track.at(image);MappingObservation identity{image.frame_id,image.cam_id,feature};
      if(!same(pixel,mapper.feature_corners.at(image).corners.at(feature)))throw std::invalid_argument("mapping_source_pixel_changed");
      saved.pixels.push_back(owners.at(identity));
    }
    data->landmarks.push_back(std::move(saved));
  }
  if(owner) {
    if(&owner->mapper()!=&mapper)throw std::invalid_argument("native_graph_checkpoint_owner");
    result->native_owner=owner->captureState(input);
    if(result->native_owner->window) {
      if((window && window!=owner->window()) || reference_stamp!=owner->window()->chart().reference())
        throw std::invalid_argument("native_window_checkpoint_owner");
      result->native_window_state=result->native_owner->window;
    }
  }
  if(window && !result->native_window_state)result->native_window_state=NativeMappingCheckpoint::captureState(*window,mapper,reference_stamp);
  data->identities=anchors;result->captured=std::move(data);result->validated=true;
  // Commit only after all original measurements and typed values are validated.
  anchors_=std::move(anchors);next_id_=next_id;last_input_=input->revision;++revision_;
  return result;
}

nlohmann::json MappingGraphRegistry::document(const Snapshot& graph) {
  using namespace mapping_graph_detail;
  if(!graph || !graph->validated)throw std::invalid_argument("mapping_graph_export");
  if(!graph->captured)return graph->imported;
  const auto& data=*graph->captured;const auto& camera=data.calibration.intrinsics[0];const auto parameters=camera.getParam();
  Json result={{"schema","posim-slam-mapping-graph-1"},{"posim_slam_version","0.1.0"},{"epoch",data.epoch},
    {"input_revision",data.input_revision},{"graph_revision",data.graph_revision},{"basalt_revision","0f3b2b52c807f70ff4e2973ce253c73329eea7bc"},
    {"coordinates","native_vio_world"},{"map_qualified",false},
    {"camera",{camera.getName(),doubles(parameters.data(),parameters.size()),doubles(data.calibration.T_i_c[0].data(),7)}},
    {"config",mapping_graph_detail::config(data.config)},{"reference",{data.reference_stamp,doubles(data.reference_pose.data(),7)}},
    {"gauge",doubles(data.gauge.data(),7)},{"optimizer",data.optimizer}};
  for(const char* name:{"frames","poses","relative","roll_pitch","pixels","tracks","landmarks","identities"})result[name]=Json::array();
  for(const auto& frame:data.frames)result["frames"].push_back({frame.stamp,frame.width,frame.height,frame.corners,Json::binary(std::vector<uint8_t>(frame.digest.begin(),frame.digest.end()))});
  for(const auto& [stamp,value]:data.poses)result["poses"].push_back({stamp,doubles(value.data(),7)});
  for(const auto& factor:data.relative)result["relative"].push_back({factor.t_i_ns,factor.t_j_ns,doubles(factor.T_i_j.data(),7),doubles(factor.cov_inv.data(),36)});
  for(const auto& factor:data.roll_pitch)result["roll_pitch"].push_back({factor.t_ns,doubles(factor.R_w_i_meas.data(),4),doubles(factor.cov_inv.data(),4)});
  for(const auto& pixel:data.pixels){auto row=key(pixel.identity);row.push_back(doubles(pixel.value.data(),2));result["pixels"].push_back(std::move(row));}
  for(const auto& track:data.tracks)result["tracks"].push_back({track.native,track.stable,track.pixels});
  for(const auto& point:data.landmarks)result["landmarks"].push_back({point.native,point.host,point.camera,doubles(point.parameters.data(),3),point.pixels});
  for(const auto& [anchor,id]:*data.identities){auto row=key(anchor);row.push_back(id);result["identities"].push_back(std::move(row));}
  return result;
}

std::unique_ptr<basalt::NfrMapper> MappingGraphRegistry::restore(const nlohmann::json& data,basalt::Calibration<double>* calibration_out,
    size_t maximum_frames) {
  using namespace mapping_graph_detail;
  if(maximum_frames<2 || maximum_frames>kMaxMappingHistoryFrames)throw std::invalid_argument("mapping_graph_capacity");
  if(data.at("schema")!="posim-slam-mapping-graph-1" || data.at("posim_slam_version")!="0.1.0" ||
      data.at("basalt_revision")!="0f3b2b52c807f70ff4e2973ce253c73329eea7bc" ||
      data.at("coordinates")!="native_vio_world" || data.at("map_qualified")!=false)
    throw std::invalid_argument("mapping_graph_schema");
  const auto epoch=data.at("epoch").get<std::string>();
  if(epoch.empty() || epoch.size()>256)throw std::invalid_argument("mapping_graph_epoch");
  integer(data.at("input_revision"),1,512);integer(data.at("graph_revision"),1,INT64_MAX);
  const auto& frames=data.at("frames");const auto& poses=data.at("poses");const auto& pixels=data.at("pixels");
  const auto& tracks=data.at("tracks");const auto& landmarks=data.at("landmarks");
  count(frames,maximum_frames,2);count(poses,frames.size(),frames.size());count(pixels,1000000);
  count(tracks,100000);count(landmarks,tracks.size());count(data.at("relative"),65536);count(data.at("roll_pitch"),512);
  basalt::Calibration<double> calibration;const auto& camera=data.at("camera");count(camera,3,3);
  auto model=basalt::GenericCamera<double>::fromString(camera[0].get<std::string>());
  if(model.getName()!=camera[0].get<std::string>())throw std::invalid_argument("mapping_graph_camera");
  const auto parameters=doubles(camera[1],model.getParam().size());
  std::visit([&](auto& value) {
    auto p=value.getParam();for(Eigen::Index i=0;i<p.size();++i)p[i]=parameters[i];
    value=std::decay_t<decltype(value)>(p);
  },model.variant);
  calibration.intrinsics.push_back(model);calibration.T_i_c.push_back(pose(camera[2]));
  std::map<int64_t,size_t> frame_sizes;size_t all_corners=0;
  for(const auto& row:frames) {
    count(row,5,5);const auto stamp=integer(row[0],1,INT64_MAX);const int width=integer(row[1],1,1920),height=integer(row[2],1,1080);
    const size_t corners=integer(row[3],0,1000000);
    if(!row[4].is_binary() || row[4].get_binary().size()!=32 || !frame_sizes.emplace(stamp,corners).second || (all_corners+=corners)>1000000)
      throw std::invalid_argument("mapping_graph_frame_identity");
    if(calibration.resolution.empty())calibration.resolution.emplace_back(width,height);
    if(calibration.resolution[0]!=Eigen::Vector2i(width,height))throw std::invalid_argument("mapping_graph_frame_shape");
  }
  auto mapper=std::make_unique<basalt::NfrMapper>(calibration,mapping_graph_detail::config(data.at("config")));
  for(const auto& [stamp,corners]:frame_sizes) {
    basalt::KeypointsData frame;frame.corners.resize(corners,Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN()));
    mapper->feature_corners.emplace(basalt::TimeCamId(stamp,0),std::move(frame));
  }
  for(const auto& row:poses) {
    count(row,2,2);const auto stamp=integer(row[0],1,INT64_MAX);
    if(!frame_sizes.count(stamp) || !mapper->getFramePoses().emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose(row[1]))).second)
      throw std::invalid_argument("mapping_graph_pose");
  }
  const auto& reference=data.at("reference");count(reference,2,2);
  if(!frame_sizes.count(integer(reference[0],1,INT64_MAX)))throw std::invalid_argument("mapping_graph_reference");
  pose(reference[1]);pose(data.at("gauge"));const auto& optimizer=data.at("optimizer");count(optimizer,4,4);
  for(const auto& value:optimizer)if(!value.is_number() || !std::isfinite(value.get<double>()) || value.get<double>()<0)
    throw std::invalid_argument("mapping_graph_optimizer");
  mapper->lambda=optimizer[0];mapper->min_lambda=optimizer[1];mapper->max_lambda=optimizer[2];mapper->lambda_vee=optimizer[3];
  std::set<std::pair<int64_t,int64_t>> relative_ids;
  for(const auto& row:data.at("relative")) {
    count(row,4,4);basalt::RelPoseFactor factor;factor.t_i_ns=integer(row[0],1,INT64_MAX);factor.t_j_ns=integer(row[1],1,INT64_MAX);
    if(factor.t_i_ns==factor.t_j_ns || !frame_sizes.count(factor.t_i_ns) || !frame_sizes.count(factor.t_j_ns) ||
        !relative_ids.emplace(factor.t_i_ns,factor.t_j_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
    factor.T_i_j=pose(row[2]);factor.cov_inv=information<6>(row[3]);mapper->rel_pose_factors.push_back(std::move(factor));
  }
  std::set<int64_t> roll_ids;
  for(const auto& row:data.at("roll_pitch")) {
    count(row,3,3);basalt::RollPitchFactor factor;factor.t_ns=integer(row[0],1,INT64_MAX);
    if(!frame_sizes.count(factor.t_ns) || !roll_ids.insert(factor.t_ns).second)throw std::invalid_argument("mapping_graph_factor_reused");
    const auto rotation=doubles(row[1],4);
    if(std::abs(Eigen::Map<const Eigen::Vector4d>(rotation.data()).squaredNorm()-1)>1e-10)throw std::invalid_argument("mapping_graph_rotation");
    std::copy(rotation.begin(),rotation.end(),factor.R_w_i_meas.data());factor.cov_inv=information<2>(row[2]);mapper->roll_pitch_factors.push_back(std::move(factor));
  }
  std::vector<MappingObservation> observations;std::set<MappingObservation> unique;
  for(const auto& row:pixels) {
    count(row,4,4);const auto identity=key(row);const auto [stamp,cam,feature]=identity;const auto values=doubles(row[3],2);
    if(!frame_sizes.count(stamp) || size_t(feature)>=frame_sizes.at(stamp) || !unique.insert(identity).second)
      throw std::invalid_argument("mapping_graph_observation_reused");
    mapper->feature_corners.at(basalt::TimeCamId(stamp,cam)).corners[feature]=Eigen::Vector2d(values[0],values[1]);observations.push_back(identity);
  }
  const auto& identities=data.at("identities");count(identities,100000);
  std::map<MappingObservation,int> ledger;std::set<int> ids;
  for(const auto& row:identities) {
    count(row,4,4);const auto anchor=key(row);const auto id=int(integer(row[3],0,99999));
    if(!ledger.emplace(anchor,id).second || !ids.insert(id).second)throw std::invalid_argument("mapping_graph_identity_reused");
  }
  std::map<int64_t,std::set<size_t>> track_pixels;std::set<size_t> used;
  for(const auto& row:tracks) {
    count(row,3,3);const auto native=integer(row[0],0,INT32_MAX);const auto stable=integer(row[1],0,99999);count(row[2],2048,2);
    basalt::FeatureTrack track;std::set<size_t> members;
    for(const auto& index:row[2]) {
      const auto item=size_t(integer(index,0,observations.size()?observations.size()-1:0));
      if(item>=observations.size() || !used.insert(item).second)throw std::invalid_argument("mapping_graph_observation_reused");
      const auto [stamp,cam,feature]=observations[item];members.insert(item);
      if(!track.emplace(basalt::TimeCamId(stamp,cam),feature).second)throw std::invalid_argument("mapping_graph_observation_reused");
    }
    const auto& first=*track.begin();MappingObservation anchor{first.first.frame_id,first.first.cam_id,first.second};
    if(!ledger.count(anchor) || ledger.at(anchor)!=stable || !mapper->feature_tracks.emplace(native,std::move(track)).second)
      throw std::invalid_argument("mapping_graph_track_identity");
    track_pixels.emplace(native,std::move(members));
  }
  if(used.size()!=pixels.size())throw std::invalid_argument("mapping_graph_unowned_measurement");
  std::set<int64_t> landmarks_seen;
  for(const auto& row:landmarks) {
    count(row,5,5);const auto native=integer(row[0],0,INT32_MAX);const auto host=integer(row[1],1,INT64_MAX);const auto cam=int(integer(row[2],0,0));
    if(!track_pixels.count(native) || !landmarks_seen.insert(native).second || !mapper->feature_tracks.at(native).count(basalt::TimeCamId(host,cam)))
      throw std::invalid_argument("mapping_graph_landmark_track");
    const auto parameters=doubles(row[3],3,false);basalt::Keypoint<double> point;
    point.direction=Eigen::Vector2d(parameters[0],parameters[1]);point.inv_dist=parameters[2];point.host_kf_id=basalt::TimeCamId(host,size_t(cam));
    mapper->lmdb.addLandmark(native,point);count(row[4],2048);std::set<size_t> seen;
    for(const auto& index:row[4]) {
      const auto item=size_t(integer(index,0,observations.size()?observations.size()-1:0));
      if(!track_pixels.at(native).count(item) || !seen.insert(item).second)throw std::invalid_argument("mapping_graph_landmark_observation");
      const auto [stamp,cam,feature]=observations[item];const basalt::TimeCamId image(stamp,cam);basalt::KeypointObservation<double> observation;
      observation.kpt_id=native;observation.pos=mapper->feature_corners.at(image).corners[feature];mapper->lmdb.addObservation(image,observation);
    }
  }
  if(calibration_out)*calibration_out=calibration;
  return mapper;
}

void MappingGraphRegistry::adopt(const Snapshot& graph) {
  using namespace mapping_graph_detail;
  if(!graph || revision_)throw std::invalid_argument("mapping_graph_adoption_requires_empty_registry");
  const auto data=document(graph);restore(data,nullptr,frameCapacity(graph));
  if(data.at("epoch")!=epoch_)throw std::invalid_argument("mapping_graph_epoch_mismatch");
  const auto parameters=calibration_.intrinsics[0].getParam();
  const Json camera={calibration_.intrinsics[0].getName(),doubles(parameters.data(),parameters.size()),doubles(calibration_.T_i_c[0].data(),7)};
  if(data.at("camera")!=camera || data.at("config")!=mapping_graph_detail::config(config_) ||
      calibration_.resolution[0]!=Eigen::Vector2i(data.at("frames")[0][1],data.at("frames")[0][2]))
    throw std::invalid_argument("mapping_graph_configuration");
  auto anchors=std::make_shared<MappingGraph::Ledger>();int next=0;
  for(const auto& row:data.at("identities")) {
    const auto id=int(integer(row[3],0,99999));anchors->emplace(key(row),id);next=std::max(next,id+1);
  }
  anchors_=std::move(anchors);next_id_=next;revision_=integer(data.at("graph_revision"),1,INT64_MAX);
  last_input_=integer(data.at("input_revision"),1,512);
}

void MappingGraphRegistry::write(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,int64_t tracking_stamp) {
  writeArchive(path,graph,request,tracking_stamp,false);
}
void MappingGraphRegistry::writeWithInputs(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,int64_t tracking_stamp) {
  writeArchive(path,graph,request,tracking_stamp,true);
}
void MappingGraphRegistry::writeWithInputsAtCapture(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,
    const std::function<int64_t()>& capture_tracking_stamp) {
  if(!capture_tracking_stamp)throw std::invalid_argument("mapping_graph_capture_missing");
  // A callback can change the caller variables. Own the immutable graph,
  // destination and callable until this operation has finished.
  const auto target=path;const Snapshot prepared_graph=graph;
  const auto capture=capture_tracking_stamp;
  writeArchive(target,prepared_graph,request,0,true,capture);
}
std::shared_ptr<const MappingInputStore::Frame> MappingGraphRegistry::inputFrame(const Snapshot& graph,int64_t stamp,size_t maximum) {
  if(!graph || !graph->validated || !graph->hasInputFrames())throw std::invalid_argument("mapping_input_frames_missing");
  if(graph->pages)return graph->pages->read(stamp,maximum);
  const auto found=graph->captured->inputs.find(stamp);
  if(found==graph->captured->inputs.end())throw std::invalid_argument("mapping_input_frame_missing");
  return found->second.read(maximum);
}
std::unique_ptr<basalt::NfrMapper> MappingGraphRegistry::restoreForMatching(const Snapshot& graph,const std::function<bool()>& cancelled) {
  if(!graph || !graph->validated || !graph->hasInputFrames())throw std::invalid_argument("mapping_input_frames_missing");
  auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("mapping_input_restore_cancelled");};check();
  const auto data=document(graph);auto mapper=restore(data);check();
  for(const auto& row:data.at("frames")) {
    check();const int64_t stamp=row[0];const auto frame=inputFrame(graph,stamp);
    const basalt::TimeCamId id(stamp,0);mapper->feature_corners.at(id)=frame->features;
    mapper->hash_bow_database->add_to_database(id,frame->features.bow_vector);
  }
  // These are the original observation owners, including untriangulated tracks.
  // No correspondence may silently acquire a different pixel after hydration.
  for(const auto& row:data.at("pixels")) {
    const auto [stamp,camera,feature]=mapping_graph_detail::key(row);
    const auto values=mapping_graph_detail::doubles(row[3],2);
    if(!mapping_graph_detail::same(mapper->feature_corners.at(basalt::TimeCamId(stamp,camera)).corners.at(feature),
        Eigen::Vector2d(values[0],values[1])))throw std::invalid_argument("mapping_source_pixel_changed");
  }
  check();return mapper;
}
std::unique_ptr<NativeMappingGraph> MappingGraphRegistry::restoreNativeGraph(const Snapshot& graph,
    MappingMatchHistory& matching,const MappingInputStore::Snapshot& input,const std::function<bool()>& cancelled,
    std::shared_ptr<SubmapRowStore> row_store) {
  if(!graph || !graph->validated || !graph->hasNativeGraph())
    throw std::invalid_argument("native_graph_checkpoint_missing");
  const auto saved=graph->native_owner?NativeMappingGraph::checkpoint(*graph->native_owner):graph->native_owner_saved;
  const auto window=graph->native_window_state?NativeMappingCheckpoint::document(*graph->native_window_state):graph->native_window;
  return NativeMappingGraph::restore(matching,input,saved,restore(document(graph),nullptr,frameCapacity(graph)),cancelled,window,std::move(row_store));
}

MappingInputStore::Snapshot MappingGraphRegistry::restoreInputs(const Snapshot& graph) {
  if(!graph || !graph->validated || !graph->hasNativeGraph() || !graph->hasInputFrames())
    throw std::invalid_argument("native_graph_checkpoint_missing");
  if(graph->native_owner)return graph->native_owner->input;
  const auto data=document(graph);auto mapper=restore(data,nullptr,frameCapacity(graph));auto input=std::make_shared<MappingInputStore::Data>();
  input->revision=data.at("input_revision");input->relative=std::move(mapper->rel_pose_factors);
  input->roll_pitch=std::move(mapper->roll_pitch_factors);
  for(const auto& row:graph->native_owner_saved.at("data").at("input_poses")) {
    const int64_t stamp=row[0];input->poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,pose(row[1])));
  }
  for(const auto& row:data.at("frames")) {
    InputDigest::Value hash;std::copy(row[4].get_binary().begin(),row[4].get_binary().end(),hash.begin());
    const int64_t stamp=row[0];MappingFrameReference::Metadata metadata{stamp,row[1],row[2],row[3],hash};
    input->frames.emplace(stamp,MappingFrameReference(metadata,[graph,stamp](size_t maximum) {return inputFrame(graph,stamp,maximum);}));
  }
  return input;
}

void MappingGraphRegistry::writeArchive(const std::filesystem::path& path,const Snapshot& graph,uint64_t request,int64_t tracking_stamp,bool inputs,
    const std::function<int64_t()>& capture_tracking_stamp) {
  using namespace mapping_graph_detail;
  if(!graph || !graph->validated || !request || (!capture_tracking_stamp && tracking_stamp<=0))
    throw std::invalid_argument("mapping_graph_export");
  const auto temporary=path.string()+".tmp";
  std::fstream output;int64_t latest_frame_stamp=0;uint64_t page_bytes=0;
  const bool owner=inputs && graph->hasNativeGraph();
  const auto prepare=[&] {
  const auto data=document(graph);
  for(const auto& frame:data.at("frames")) {
    latest_frame_stamp=std::max(latest_frame_stamp,frame[0].get<int64_t>());
    if(!capture_tracking_stamp && frame[0].get<int64_t>()>tracking_stamp)
      throw std::invalid_argument("mapping_graph_export_timing");
  }
  if(std::filesystem::is_symlink(path) || std::filesystem::is_symlink(temporary))throw std::invalid_argument("mapping_graph_archive_path");
  output.open(temporary,std::ios::binary|std::ios::in|std::ios::out|std::ios::trunc);
  if(!output)throw std::runtime_error("mapping_graph_archive_unwritable");
  Json pages=Json::array();
  const auto append_frame=[&](const MappingInputFrame& frame,const std::vector<uint8_t>& encoded,const InputDigest::Value& digest) {
    if(pages.size()>=data.at("frames").size() ||
        Json::array({frame.stamp,frame.width,frame.height,frame.features.corners.size(),mapping_frame_io::binary(frame.digest)})!=data.at("frames")[pages.size()])
      throw std::invalid_argument("mapping_input_frame_identity");
    if(encoded.size()>archive_limit-page_bytes)throw std::runtime_error("mapping_graph_archive_capacity");
    pages.push_back({frame.stamp,page_bytes,encoded.size(),mapping_frame_io::binary(digest)});page_bytes+=encoded.size();
    output.write(reinterpret_cast<const char*>(encoded.data()),encoded.size());
    if(!output)throw std::runtime_error("mapping_graph_archive_write_failed");
  };
  // Stream each original frame once. Native matching uses these exact encoded
  // bytes for its identity, then the archive appends them before the lease ends.
  // Restored and graph-only owners use the same writer without a matching visit.
  Json owner_checkpoint;
  if(owner && graph->native_owner) {
    owner_checkpoint=NativeMappingGraph::checkpoint(*graph->native_owner,
      [&](const MappingFrameReference& source,const MappingInputFrame& frame,
          const std::vector<uint8_t>& encoded,const InputDigest::Value& digest) {
        if(!graph->captured || !graph->captured->inputs.count(frame.stamp) ||
            !graph->captured->inputs.at(frame.stamp).sameSource(source))
          throw std::invalid_argument("native_graph_checkpoint_source");
        append_frame(frame,encoded,digest);
      });
  } else if(inputs) {
    for(const auto& row:data.at("frames")) {
      const auto frame=inputFrame(graph,row[0]);const auto encoded=mapping_frame_io::encode(*frame);
      append_frame(*frame,encoded,mapping_frame_io::digest(encoded));
    }
    if(owner)owner_checkpoint=graph->native_owner_saved;
  }
  if(inputs && pages.size()!=data.at("frames").size())throw std::invalid_argument("mapping_input_frame_identity");
  // The optional window belongs to this same immutable graph revision.
  // A distinct signature makes older readers reject unsupported window state.
  // Keep the ordinary three-entry CBOR archive byte-for-byte compatible.
  // Serialize the immutable graph directly instead of copying it into an envelope.
  std::vector<uint8_t> bytes{uint8_t(0xa3+int(graph->hasNativeWindow())+int(inputs)+int(owner))};
  Json::to_cbor("graph",bytes);Json::to_cbor(data,bytes);
  if(inputs) {Json::to_cbor("input_pages",bytes);Json::to_cbor(pages,bytes);}
  if(graph->hasNativeWindow()) {Json::to_cbor("native_window",bytes);Json::to_cbor(graph->native_window_state?NativeMappingCheckpoint::document(*graph->native_window_state):graph->native_window,bytes);}
  if(owner) {
    Json::to_cbor("native_graph",bytes);
    Json::to_cbor(owner_checkpoint,bytes);
  }
  return bytes;
  };
  // Release graph/window/matching JSON before selecting the tracking snapshot.
  // All source reads, identity validation and complete page writes precede it.
  auto bytes=prepare();
  if(bytes.size()>archive_limit-page_bytes)throw std::runtime_error("mapping_graph_archive_capacity");
  InputDigest hash;hash.update(bytes.data(),bytes.size());
  const size_t prepared_bytes=bytes.size();
  // Final keys plus two int64 values need at most 49 CBOR bytes. Reserve
  // before capture so appending identity cannot copy the large envelope.
  bytes.reserve(prepared_bytes+64);
  if(capture_tracking_stamp) {
    tracking_stamp=capture_tracking_stamp();
    if(tracking_stamp<=0 || tracking_stamp<latest_frame_stamp)
      throw std::invalid_argument("mapping_graph_export_timing");
  }
  Json::to_cbor("request_id",bytes);Json::to_cbor(request,bytes);
  Json::to_cbor("tracking_stamp_ns",bytes);Json::to_cbor(tracking_stamp,bytes);
  if(bytes.size()>archive_limit-page_bytes)throw std::runtime_error("mapping_graph_archive_capacity");
  hash.update(bytes.data()+prepared_bytes,bytes.size()-prepared_bytes);const auto digest=hash.finish();
  // Prepend the unchanged envelope inside the same unpublished temporary file.
  // Moving backwards handles overlap with one bounded buffer; decoded frames
  // and page payloads are never accumulated into a second in-memory archive.
  const uint64_t prefix=48+bytes.size();std::vector<char> buffer(std::min<uint64_t>(1024*1024,page_bytes));
  for(uint64_t end=page_bytes;end;) {
    const auto count=std::min<uint64_t>(end,buffer.size());const auto start=end-count;
    output.seekg(start);output.read(buffer.data(),count);
    if(!output)throw std::runtime_error("mapping_graph_archive_write_failed");
    output.seekp(prefix+start);output.write(buffer.data(),count);
    if(!output)throw std::runtime_error("mapping_graph_archive_write_failed");end=start;
  }
  output.seekp(0);
  output.write(owner?owner_magic:inputs?input_magic:graph->hasNativeWindow()?window_magic:magic,8);output.write(reinterpret_cast<const char*>(digest.data()),digest.size());
  uint8_t size[8];for(int i=0;i<8;++i)size[i]=uint8_t(uint64_t(bytes.size())>>(8*i));output.write(reinterpret_cast<const char*>(size),8);
  output.write(reinterpret_cast<const char*>(bytes.data()),bytes.size());
  output.flush();
  if(!output)throw std::runtime_error("mapping_graph_archive_write_failed");output.close();
  if(!output)throw std::runtime_error("mapping_graph_archive_write_failed");std::filesystem::rename(temporary,path);
}

MappingGraphRegistry::Restored MappingGraphRegistry::read(const std::filesystem::path& path,const std::string& expected_epoch) {
  using namespace mapping_graph_detail;
  if(std::filesystem::is_symlink(path))throw std::invalid_argument("mapping_graph_archive_path");
  const auto size=std::filesystem::file_size(path);
  if(size<48 || size>archive_limit+48)throw std::invalid_argument("mapping_graph_archive_capacity");
  std::ifstream input(path,std::ios::binary);char signature[8];std::array<uint8_t,32> expected;uint8_t length[8];
  input.read(signature,8);input.read(reinterpret_cast<char*>(expected.data()),32);input.read(reinterpret_cast<char*>(length),8);
  uint64_t count=0;for(int i=0;i<8;++i)count|=uint64_t(length[i])<<(8*i);
  const bool with_window=std::memcmp(signature,window_magic,8)==0;
  const bool with_owner=std::memcmp(signature,owner_magic,8)==0;
  const bool with_inputs=with_owner || std::memcmp(signature,input_magic,8)==0;
  if(!input || (!with_inputs && !with_window && std::memcmp(signature,magic,8)) ||
      (with_inputs?(!count || count>size-48):count!=size-48))throw std::invalid_argument("mapping_graph_archive_header");
  std::vector<uint8_t> bytes(count);input.read(reinterpret_cast<char*>(bytes.data()),bytes.size());
  InputDigest digest;digest.update(bytes.data(),bytes.size());
  if(!input || digest.finish()!=expected)throw std::invalid_argument("mapping_graph_archive_digest");
  auto document=Json::from_cbor(bytes);auto graph=std::make_shared<MappingGraph>();
  if(!with_inputs && with_window!=document.contains("native_window"))throw std::invalid_argument("mapping_graph_archive_window");
  if(with_inputs!=document.contains("input_pages"))throw std::invalid_argument("mapping_input_page_index");
  if(with_owner!=document.contains("native_graph"))throw std::invalid_argument("native_graph_checkpoint_missing");
  graph->imported=std::move(document.at("graph"));
  auto mapper=restore(graph->imported,nullptr,document.contains("native_window")?kMaxMappingHistoryFrames:size_t(kMaxJointColumns/6));
  if(document.contains("native_window")) {
    graph->native_window=std::move(document.at("native_window"));
    NativeMappingCheckpoint::validate(*mapper,graph->native_window,integer(graph->imported.at("reference")[0],1,INT64_MAX));
  }
  if(with_owner) {
    graph->native_owner_saved=std::move(document.at("native_graph"));
    NativeMappingGraph::validateCheckpoint(*mapper,graph->native_owner_saved,
        integer(graph->imported.at("input_revision"),1,512),graph->native_window);
  }
  graph->validated=true;
  if(!expected_epoch.empty() && graph->imported.at("epoch")!=expected_epoch)throw std::invalid_argument("mapping_graph_epoch_mismatch");
  for(const auto& row:graph->imported.at("tracks"))graph->stable_ids.emplace(integer(row[0],0,INT32_MAX),integer(row[1],0,99999));
  const auto tracking_stamp=integer(document.at("tracking_stamp_ns"),1,INT64_MAX);
  for(const auto& frame:graph->imported.at("frames"))if(frame[0].get<int64_t>()>tracking_stamp)throw std::invalid_argument("mapping_graph_export_timing");
  if(with_inputs)graph->pages=std::make_shared<MappingFramePages>(std::move(input),48+count,size,document.at("input_pages"),graph->imported.at("frames"));
  return {graph,uint64_t(integer(document.at("request_id"),1,INT64_MAX)),tracking_stamp};
}
} // namespace posim_slam

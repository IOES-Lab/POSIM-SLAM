// SPDX-License-Identifier: Apache-2.0
#include "native_mapping_graph.hpp"
#include "mapping_graph.hpp"
#include "native_mapping_checkpoint.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <climits>
#include <cmath>

namespace posim_slam {
NativeMappingGraph::NativeMappingGraph(MappingMatchHistory& matching,size_t maximum_bytes,size_t active_window,
    std::shared_ptr<SubmapRowStore> row_store)
    :matching_(matching),mapper_(matching.calibration_,matching.config_),
     maximum_bytes_(maximum_bytes?maximum_bytes:matching.maximum_bytes_),row_store_(std::move(row_store)),window_size_(active_window) {
  if(!maximum_bytes_ || maximum_bytes_>matching.maximum_bytes_ ||
      (!active_window && matching.maximum_frames_>size_t(kMaxJointColumns/6)) ||
      active_window>SubmapInformation::Limits{}.active_columns/6)
    throw std::invalid_argument("native_graph_capacity");
  initial_solver_={mapper_.lambda,mapper_.min_lambda,mapper_.max_lambda,mapper_.lambda_vee};
}
NativeMappingGraph::~NativeMappingGraph()=default;
namespace {
const nlohmann::json& checkedGraphCheckpoint(const nlohmann::json& document) {
  using namespace mapping_graph_detail;
  const bool window=document.value("schema","")=="posim-slam-native-graph-2";
  if(!document.is_object() || document.size()!=3 || (!window && document.at("schema")!="posim-slam-native-graph-1"))
    throw std::invalid_argument("native_graph_checkpoint_schema");
  const auto& data=document.at("data");
  if(!data.is_object() || data.size()!=(window?8:6))throw std::invalid_argument("native_graph_checkpoint_schema");
  if(window) {
    integer(data.at("active_window"),1,SubmapInformation::Limits{}.active_columns/6);
    if(!data.at("window_sha256").is_binary() || data.at("window_sha256").get_binary().size()!=32)
      throw std::invalid_argument("native_graph_checkpoint_window");
  }
  const auto bytes=Json::to_cbor(data);
  if(bytes.size()>96*1024*1024)throw std::runtime_error("native_graph_numeric_capacity");
  if(document.at("sha256")!=mapping_frame_io::binary(mapping_frame_io::digest(bytes)))
    throw std::invalid_argument("native_graph_checkpoint_digest");
  MappingMatchHistory::validateCheckpoint(data.at("matching"));return data;
}
}

std::shared_ptr<const NativeMappingGraphState> NativeMappingGraph::captureState(const MappingInputStore::Snapshot& input) {
  if(!input_ || input!=input_ || !matched_ || matched_->revision!=last_.revision ||
      nativeMappingSourceIdentity(mapper_)!=source_identity_)
    throw std::invalid_argument("native_graph_checkpoint_owner");
  auto state=std::make_shared<NativeMappingGraphState>();state->matching=matched_;state->input=input;
  state->calibration=matching_.calibration_;state->config=matching_.config_;
  state->revision=last_.revision;state->next_track=next_track_;
  state->source=source_identity_;state->nonlinear=nativeMappingStateIdentity(mapper_);
  state->active_window=window_size_;
  if(window_size_) {
    if(!window_)throw std::invalid_argument("native_graph_checkpoint_window");
    state->window=NativeMappingCheckpoint::captureState(*window_,mapper_,window_->chart().reference());
  }
  return state;
}

nlohmann::json NativeMappingGraph::checkpoint(const NativeMappingGraphState& state) {
  using namespace mapping_graph_detail;
  Json data={{"revision",state.revision},{"next_track",state.next_track},
      {"source",mapping_frame_io::binary(state.source)},{"nonlinear",mapping_frame_io::binary(state.nonlinear)},
      {"matching",MappingMatchHistory::checkpoint(state.matching,state.calibration,state.config)},
      {"input_poses",Json::array()}};
  if(!state.input || state.input->revision!=state.revision)throw std::invalid_argument("native_graph_checkpoint_source");
  for(const auto& [stamp,pose]:state.input->poses)data["input_poses"].push_back({stamp,doubles(pose.getPose().data(),7)});
  if(bool(state.active_window)!=bool(state.window))throw std::invalid_argument("native_graph_checkpoint_window");
  if(state.window) {
    data["active_window"]=state.active_window;
    data["window_sha256"]=NativeMappingCheckpoint::document(*state.window).at("sha256");
  }
  const auto bytes=Json::to_cbor(data);
  return {{"schema",state.window?"posim-slam-native-graph-2":"posim-slam-native-graph-1"},{"data",std::move(data)},
      {"sha256",mapping_frame_io::binary(mapping_frame_io::digest(bytes))}};
}

void NativeMappingGraph::validateCheckpoint(basalt::NfrMapper& mapper,const nlohmann::json& document,uint64_t revision,
    const nlohmann::json& window) {
  using namespace mapping_graph_detail;
  const auto& data=checkedGraphCheckpoint(document);
  if(data.contains("active_window")) {
    if(window.is_null() || data.at("window_sha256")!=window.at("sha256"))
      throw std::invalid_argument("native_graph_checkpoint_window");
    const auto reference=integer(window.at("model").at("reference"),1,INT64_MAX);
    NativeMappingCheckpoint::validate(mapper,window,reference);
    std::set<int64_t> expected;const size_t count=data.at("active_window");
    for(auto it=mapper.getFramePoses().rbegin();it!=mapper.getFramePoses().rend() && expected.size()<count;++it)expected.insert(it->first);
    if(window.at("model").at("active")!=Json(expected))throw std::invalid_argument("native_graph_checkpoint_window");
  }
  if(data.at("revision")!=revision || data.at("matching").at("data").at("revision")!=revision ||
      data.at("source")!=mapping_frame_io::binary(nativeMappingSourceIdentity(mapper)) ||
      data.at("nonlinear")!=mapping_frame_io::binary(nativeMappingStateIdentity(mapper)))
    throw std::invalid_argument("native_graph_checkpoint_source");
  const int64_t next=integer(data.at("next_track"),0,int64_t(INT_MAX)+1);
  for(const auto& [id,track]:mapper.feature_tracks)
    if(id<0 || id>=next)throw std::invalid_argument("native_graph_checkpoint_identity");
  const auto& seeds=data.at("input_poses");std::set<int64_t> seen;
  if(!seeds.is_array() || seeds.size()!=mapper.getFramePoses().size())throw std::invalid_argument("native_graph_checkpoint_source");
  for(const auto& row:seeds) {
    if(!row.is_array() || row.size()!=2)throw std::invalid_argument("native_graph_checkpoint_source");
    const auto stamp=integer(row[0],1,INT64_MAX);pose(row[1]);
    if(!mapper.getFramePoses().count(stamp) || !seen.insert(stamp).second)throw std::invalid_argument("native_graph_checkpoint_source");
  }
}

std::unique_ptr<NativeMappingGraph> NativeMappingGraph::restore(MappingMatchHistory& matching,
    const MappingInputStore::Snapshot& input,const nlohmann::json& document,
    std::unique_ptr<basalt::NfrMapper> mapper,const std::function<bool()>& cancelled,
    const nlohmann::json& window,std::shared_ptr<SubmapRowStore> row_store) {
  if(!input || !mapper || !input->revision || input->frames.empty() ||
      input->frames.size()!=input->poses.size() || input->frames.size()!=mapper->getFramePoses().size() ||
      (matching.source_identity_ && matching.source_identity_!=input->identity))
    throw std::invalid_argument("native_graph_checkpoint_source");
  validateCheckpoint(*mapper,document,input->revision,window);
  for(const auto& row:document.at("data").at("input_poses")) {
    const auto found=input->poses.find(row[0].get<int64_t>());
    if(found==input->poses.end() || row[1]!=mapping_graph_detail::doubles(found->second.getPose().data(),7))
      throw std::invalid_argument("native_graph_checkpoint_source");
  }
  unchanged(input->relative,mapper->rel_pose_factors);unchanged(mapper->rel_pose_factors,input->relative);
  unchanged(input->roll_pitch,mapper->roll_pitch_factors);unchanged(mapper->roll_pitch_factors,input->roll_pitch);
  for(const auto& [stamp,pose]:mapper->getFramePoses())
    if(!input->poses.count(stamp) || !input->frames.count(stamp))throw std::invalid_argument("native_graph_checkpoint_source");
  // Graph-only rows contain observed pixels. Future tracks also need detector
  // pixels that have not yet joined any track; recover those from one complete
  // original frame at a time, without retaining descriptors or rerunning RANSAC.
  for(const auto& [stamp,reference]:input->frames) {
    if(cancelled && cancelled())throw std::runtime_error("mapping_match_cancelled");
    const auto frame=reference.read();mapping_frame_io::shape(*frame);
    if(frame->stamp!=stamp || frame->features.corners.size()!=reference.metadata().corners)
      throw std::invalid_argument("native_graph_checkpoint_source");
    mapper->feature_corners.at(basalt::TimeCamId(stamp,0)).corners=frame->features.corners;
  }
  const size_t policy=document.at("data").value("active_window",size_t(0));
  auto result=std::make_unique<NativeMappingGraph>(matching,0,policy,std::move(row_store));
  result->mapper_.getFramePoses().swap(mapper->getFramePoses());
  result->mapper_.feature_corners.swap(mapper->feature_corners);result->mapper_.feature_tracks.swap(mapper->feature_tracks);
  mapper->lmdb.backup();std::swap(result->mapper_.lmdb,mapper->lmdb);
  result->mapper_.rel_pose_factors.swap(mapper->rel_pose_factors);result->mapper_.roll_pitch_factors.swap(mapper->roll_pitch_factors);
  result->mapper_.lambda=mapper->lambda;result->mapper_.min_lambda=mapper->min_lambda;
  result->mapper_.max_lambda=mapper->max_lambda;result->mapper_.lambda_vee=mapper->lambda_vee;
  validateCheckpoint(result->mapper_,document,input->revision,window);
  result->last_.revision=input->revision;
  result->last_.numeric_bytes=numericBytes(result->mapper_.feature_corners,result->mapper_.feature_tracks,
      input->poses.size(),input->relative.size(),input->roll_pitch.size());
  if(result->last_.numeric_bytes>result->maximum_bytes_)throw std::runtime_error("native_graph_numeric_capacity");
  result->source_identity_=nativeMappingSourceIdentity(result->mapper_);
  result->input_=input;result->next_track_=document.at("data").at("next_track").get<int64_t>();
  if(policy) {
    SubmapInformation::Limits limits;
    limits.numeric_bytes=result->maximum_bytes_;
    result->window_=NativeMappingCheckpoint::restore(result->mapper_,window,limits,cancelled,result->row_store_);
  }
  // All graph allocation and validation precede the matching commit. A failed
  // source read, cancellation or quota check leaves that empty owner unchanged.
  result->matched_=matching.restore(input->revision,input->frames,document.at("data").at("matching"),cancelled);
  return result;
}

NativeMappingGraph::Update NativeMappingGraph::update(const MappingInputStore::Snapshot& input,
    int iterations,const std::function<bool()>& cancelled,int64_t reference_stamp) {
  auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_graph_cancelled");};check();
  if(!input || !input->revision || input->frames.empty() || input->frames.size()!=input->poses.size() ||
      input->frames.size()>(window_size_?matching_.maximum_frames_:size_t(kMaxJointColumns/6)) || iterations<0 || iterations>10 ||
      (matching_.source_identity_ && input->identity!=matching_.source_identity_))
    throw std::invalid_argument("native_graph_input");
  if(reference_stamp<0 || (reference_stamp && !input->poses.count(reference_stamp)) ||
      (window_ && reference_stamp && window_->chart().reference()!=reference_stamp))
    throw std::invalid_argument("native_graph_reference");
  if(input_) {
    if(input->identity!=input_->identity || input->revision<input_->revision)
      throw std::invalid_argument("native_graph_source_changed");
    if(nativeMappingSourceIdentity(mapper_)!=source_identity_)
      throw std::invalid_argument("native_graph_originals_changed");
    if(input->revision==input_->revision) {
      if(input!=input_)throw std::invalid_argument("native_graph_revision");
      Update reused;reused.revision=last_.revision;reused.numeric_bytes=last_.numeric_bytes;reused.reused=true;return reused;
    }
    unchanged(input_->relative,input->relative);unchanged(input_->roll_pitch,input->roll_pitch);
  }
  for(const auto& [stamp,pose]:input->poses)
    if(!input->frames.count(stamp) || !pose.getPose().matrix().allFinite())throw std::invalid_argument("native_graph_pose");
  std::set<std::pair<int64_t,int64_t>> relative_keys;std::set<int64_t> tilt_keys;
  for(const auto& factor:input->relative) {
    if(factor.t_i_ns==factor.t_j_ns || !input->poses.count(factor.t_i_ns) || !input->poses.count(factor.t_j_ns) ||
        !relative_keys.emplace(factor.t_i_ns,factor.t_j_ns).second || !factor.T_i_j.matrix().allFinite() ||
        !factor.cov_inv.allFinite() || !factor.cov_inv.isApprox(factor.cov_inv.transpose(),1e-12) ||
        Eigen::LLT<Eigen::Matrix<double,6,6>>(factor.cov_inv).info()!=Eigen::Success)
      throw std::invalid_argument("native_graph_factor");
  }
  for(const auto& factor:input->roll_pitch) {
    if(!input->poses.count(factor.t_ns) || !tilt_keys.insert(factor.t_ns).second || !factor.R_w_i_meas.matrix().allFinite() ||
        !factor.cov_inv.allFinite() || !factor.cov_inv.isApprox(factor.cov_inv.transpose(),1e-12) ||
        Eigen::LLT<Eigen::Matrix2d>(factor.cov_inv).info()!=Eigen::Success)
      throw std::invalid_argument("native_graph_factor");
  }
  Update committed;
  matching_.advance(input->revision,input->frames,cancelled,[&](const MappingMatchHistory::Snapshot& matched) {
  check();

  // Reconstruct only combinatorial tracks with the pinned native implementation.
  // Temporary swaps keep the one nonlinear mapper, its means and its database.
  basalt::Matches pairs;for(const auto& [key,value]:matched->matches)pairs.emplace(key,*value);
  basalt::FeatureTracks raw_tracks,old_tracks;
  mapper_.feature_matches.swap(pairs);mapper_.feature_tracks.swap(old_tracks);
  try {mapper_.build_tracks();raw_tracks.swap(mapper_.feature_tracks);}
  catch(...) {mapper_.feature_tracks.swap(old_tracks);mapper_.feature_matches.swap(pairs);throw;}
  mapper_.feature_tracks.swap(old_tracks);mapper_.feature_matches.swap(pairs);
  pairs.clear();check();

  Update result;result.revision=input->revision;
  auto next_poses=mapper_.getFramePoses();auto next_corners=mapper_.feature_corners;
  Sophus::SE3d seed_gauge;
  if(!next_poses.empty()) {
    const auto anchor=next_poses.rbegin()->first;
    const auto current=next_poses.at(anchor).getPose(),seed=input->poses.at(anchor).getPose();
    const Eigen::Matrix3d difference=current.rotationMatrix()*seed.rotationMatrix().transpose();
    const auto rotation=Sophus::SO3d::exp(Eigen::Vector3d(0,0,std::atan2(difference(1,0),difference(0,0))));
    seed_gauge=Sophus::SE3d(rotation,current.translation()-rotation*seed.translation());
  }
  for(const auto& [stamp,state]:input->poses)if(!next_poses.count(stamp)) {
    next_poses.emplace(stamp,basalt::PoseStateWithLin<double>(stamp,seed_gauge*state.getPose()));
    next_corners[basalt::TimeCamId(stamp,0)].corners=matched->frames.at(stamp).pixels->corners;++result.new_frames;
  }
  std::map<basalt::ImageFeaturePair,int64_t> previous_pixels;
  for(const auto& [id,track]:mapper_.feature_tracks)for(const auto& pixel:track)
    if(!previous_pixels.emplace(pixel,id).second)throw std::logic_error("native_graph_pixel_reused");
  std::map<basalt::ImageFeaturePair,const basalt::FeatureTrack*> ordered;
  for(const auto& [id,track]:raw_tracks)
    if(!ordered.emplace(first(track),&track).second)throw std::logic_error("native_graph_track_reused");
  basalt::FeatureTracks next_tracks;std::map<int64_t,std::set<int64_t>> ancestors;std::set<int64_t> consumed;
  int64_t next_id=next_track_;
  for(const auto& [first_pixel,track_ptr]:ordered) {
    const auto& track=*track_ptr;std::set<int64_t> old;
    for(const auto& pixel:track) {
      const auto found=previous_pixels.find(pixel);if(found!=previous_pixels.end())old.insert(found->second);
    }
    int64_t id;
    if(old.empty()) {
      if(next_id>INT_MAX)throw std::runtime_error("native_graph_track_capacity");id=next_id++;++result.new_tracks;
    } else {
      id=*std::min_element(old.begin(),old.end(),[&](int64_t a,int64_t b) {
        return first(mapper_.feature_tracks.at(a))<first(mapper_.feature_tracks.at(b));
      });
      for(auto previous:old) {
        if(!consumed.insert(previous).second)throw std::invalid_argument("native_graph_track_split");
        const auto& before=mapper_.feature_tracks.at(previous);
        for(const auto& pixel:before) {
          const auto found=track.find(pixel.first);
          if(found==track.end() || found->second!=pixel.second)throw std::invalid_argument("native_graph_track_replaced");
        }
      }
      if(old.size()>1)++result.merged_components;
      else if(mapper_.feature_tracks.at(id)!=track)++result.extended_tracks;
    }
    ancestors.emplace(id,std::move(old));next_tracks.emplace(id,track);
  }
  result.removed_tracks=mapper_.feature_tracks.size()-consumed.size();
  result.numeric_bytes=numericBytes(next_corners,next_tracks,next_poses.size(),input->relative.size(),input->roll_pitch.size());
  if(result.numeric_bytes>maximum_bytes_)throw std::runtime_error("native_graph_numeric_capacity");

  auto next_landmarks=mapper_.lmdb;
  for(const auto& [id,point]:mapper_.lmdb.getLandmarks())if(!next_tracks.count(id))next_landmarks.removeLandmark(id);
  basalt::FeatureTracks pending;
  for(const auto& [id,track]:next_tracks) {
    if(!next_landmarks.landmarkExists(int(id))) {
      for(auto previous:ancestors.at(id))if(mapper_.lmdb.landmarkExists(int(previous))) {
        auto seed=mapper_.lmdb.getLandmark(previous);seed.obs.clear();next_landmarks.addLandmark(id,seed);break;
      }
    }
    if(!next_landmarks.landmarkExists(int(id))) {pending.emplace(id,track);continue;}
    auto& point=next_landmarks.getLandmark(id);++result.preserved_landmarks;
    if(!track.count(point.host_kf_id))throw std::invalid_argument("native_graph_host_missing");
    for(const auto& [image,feature]:track) {
      const auto& pixel=next_corners.at(image).corners.at(feature);const auto found=point.obs.find(image);
      if(found!=point.obs.end()) {
        if(found->second!=pixel)throw std::invalid_argument("native_graph_pixel_changed");
      } else {
        basalt::KeypointObservation<double> observation;observation.kpt_id=int(id);observation.pos=pixel;
        next_landmarks.addObservation(image,observation);
      }
    }
  }
  auto next_relative=input->relative;auto next_roll_pitch=input->roll_pitch;
  const std::array<double,4> old_solver{mapper_.lambda,mapper_.min_lambda,mapper_.max_lambda,mapper_.lambda_vee};
  auto swap=[&] {
    mapper_.getFramePoses().swap(next_poses);mapper_.feature_corners.swap(next_corners);
    mapper_.feature_tracks.swap(next_tracks);std::swap(mapper_.lmdb,next_landmarks);
    mapper_.rel_pose_factors.swap(next_relative);mapper_.roll_pitch_factors.swap(next_roll_pitch);
  };
  check();swap();
  try {
    std::unique_ptr<NativeMappingWindow> next_window;
    check();
    // Use the original initializer only for tracks without a landmark. Existing
    // parameters, including zero inverse distance, remain free optimizer values.
    mapper_.feature_tracks.swap(pending);
    try {mapper_.setup_opt();}catch(...){mapper_.feature_tracks.swap(pending);throw;}
    mapper_.feature_tracks.swap(pending);
    result.initialized_landmarks=mapper_.lmdb.numLandmarks()-result.preserved_landmarks;
    if(mapper_.lmdb.numLandmarks()>100000)throw std::runtime_error("native_graph_landmark_capacity");
    // Initialize the native backup fields even for a caller's zero-step update.
    mapper_.lmdb.backup();
    // A changed likelihood starts the same native LM trial policy as a fresh
    // solve. This resets optimizer controls, not measurements or point means.
    mapper_.lambda=initial_solver_[0];mapper_.min_lambda=initial_solver_[1];
    mapper_.max_lambda=initial_solver_[2];mapper_.lambda_vee=initial_solver_[3];
    if(window_size_) {
      std::set<int64_t> active;
      for(auto it=mapper_.getFramePoses().rbegin();it!=mapper_.getFramePoses().rend() && active.size()<window_size_;++it)
        active.insert(it->first);
      if(window_)next_window=window_->prepareGrowth(active,cancelled);
      else {
        SubmapInformation::Limits limits;limits.numeric_bytes=maximum_bytes_;
        next_window=std::make_unique<NativeMappingWindow>(mapper_,
            reference_stamp?reference_stamp:mapper_.getFramePoses().begin()->first,active,limits,row_store_);
      }
      for(int i=0;i<iterations;++i)if(!next_window->step(cancelled).accepted)break;
    } else if(iterations)mapper_.optimize(iterations);
    for(const auto& [stamp,pose]:mapper_.getFramePoses())
      if(!pose.getPose().matrix().allFinite())throw std::runtime_error("native_graph_nonfinite_pose");
    for(const auto& [id,point]:mapper_.lmdb.getLandmarks())
      if(!point.direction.allFinite() || !std::isfinite(point.inv_dist) || point.inv_dist<0)
        throw std::runtime_error("native_graph_nonfinite_landmark");
    const auto identity=nativeMappingSourceIdentity(mapper_);check();
    window_.swap(next_window);
    source_identity_=identity;input_=input;matched_=matched;next_track_=next_id;last_=result;
  } catch(...) {
    swap();mapper_.lambda=old_solver[0];mapper_.min_lambda=old_solver[1];
    mapper_.max_lambda=old_solver[2];mapper_.lambda_vee=old_solver[3];throw;
  }
  committed=result;
  });
  return committed;
}
} // namespace posim_slam

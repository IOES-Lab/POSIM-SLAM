// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_input_store.hpp"
#include "mapping_feature_matcher.hpp"
#include "native_mapping_identity.hpp"
#include <array>
#include <functional>

namespace posim_slam {
class NativeMappingWindow;
struct NativeMappingWindowState;
struct NativeMappingGraphState {
  MappingMatchHistory::Snapshot matching;
  MappingInputStore::Snapshot input;
  basalt::Calibration<double> calibration;
  basalt::VioConfig config;
  uint64_t revision=0;int64_t next_track=0;
  InputDigest::Value source{},nonlinear{};
  size_t active_window=0;std::shared_ptr<const NativeMappingWindowState> window;
};
// One thread exclusively owns this graph and its matching history. Input poses
// seed new frames; existing nonlinear means remain optimizer variables. Native
// track conflict rules and triangulation are unchanged. An explicit nonzero
// active_window owns conditional source updates. Checkpoints preserve matching,
// nonlinear state, source anchors and the active-window policy together.
class NativeMappingGraph {
 public:
  struct Update {
    uint64_t revision=0;
    size_t new_frames=0,new_tracks=0,extended_tracks=0,merged_components=0,
      removed_tracks=0,preserved_landmarks=0,initialized_landmarks=0,numeric_bytes=0;
    bool reused=false;
  };
  explicit NativeMappingGraph(MappingMatchHistory& matching,size_t maximum_bytes=0,size_t active_window=0);
  ~NativeMappingGraph();
  basalt::NfrMapper& mapper(){return mapper_;}
  const NativeMappingWindow* window() const {return window_.get();}
  const Update& lastUpdate() const {return last_;}
  MappingMatchHistory::Snapshot matchingSnapshot() const {return matched_;}
  std::shared_ptr<const NativeMappingGraphState> captureState(const MappingInputStore::Snapshot& input);
  static nlohmann::json checkpoint(const NativeMappingGraphState& state);
  static void validateCheckpoint(basalt::NfrMapper& mapper,const nlohmann::json& document,uint64_t revision,
      const nlohmann::json& window={});
  static std::unique_ptr<NativeMappingGraph> restore(MappingMatchHistory& matching,
      const MappingInputStore::Snapshot& input,const nlohmann::json& document,
      std::unique_ptr<basalt::NfrMapper> mapper,const std::function<bool()>& cancelled={},
      const nlohmann::json& window={});
  Update update(const MappingInputStore::Snapshot& input,int iterations=10,
      const std::function<bool()>& cancelled={});
 private:
  static bool same(const basalt::RelPoseFactor& a,const basalt::RelPoseFactor& b) {
    return a.t_i_ns==b.t_i_ns && a.t_j_ns==b.t_j_ns &&
      a.T_i_j.params().isApprox(b.T_i_j.params(),0) && a.cov_inv.isApprox(b.cov_inv,0);
  }
  static bool same(const basalt::RollPitchFactor& a,const basalt::RollPitchFactor& b) {
    return a.t_ns==b.t_ns && a.R_w_i_meas.params().isApprox(b.R_w_i_meas.params(),0) && a.cov_inv.isApprox(b.cov_inv,0);
  }
  template<class Factors> static void unchanged(const Factors& old,const Factors& next) {
    if(old.size()>next.size())throw std::invalid_argument("native_graph_source_changed");
    for(size_t i=0;i<old.size();++i)if(!same(old[i],next[i]))throw std::invalid_argument("native_graph_source_changed");
  }
  static basalt::ImageFeaturePair first(const basalt::FeatureTrack& track) {
    if(track.empty())throw std::invalid_argument("native_graph_empty_track");
    return *track.begin();
  }
  static size_t numericBytes(const basalt::Corners& corners,const basalt::FeatureTracks& tracks,
      size_t poses,size_t relative,size_t roll_pitch) {
    // Reserve the landmark/observation payload for every track before native
    // triangulation. Containers, the transaction copy and solver workspace are
    // additional. The bound includes untriangulated tracks conservatively.
    size_t bytes=poses*sizeof(basalt::PoseStateWithLin<double>)+relative*sizeof(basalt::RelPoseFactor)+
      roll_pitch*sizeof(basalt::RollPitchFactor);
    for(const auto& [image,pixels]:corners)bytes+=sizeof(image)+pixels.corners.size()*sizeof(Eigen::Vector2d);
    for(const auto& [id,track]:tracks)bytes+=sizeof(id)+sizeof(basalt::Keypoint<double>)+
      track.size()*(sizeof(basalt::ImageFeaturePair)+sizeof(basalt::TimeCamId)+sizeof(Eigen::Vector2d)+sizeof(basalt::KeypointId));
    return bytes;
  }
  MappingMatchHistory& matching_;basalt::NfrMapper mapper_;size_t maximum_bytes_;
  std::array<double,4> initial_solver_;
  MappingInputStore::Snapshot input_;MappingMatchHistory::Snapshot matched_;
  InputDigest::Value source_identity_{};int64_t next_track_=0;Update last_;
  size_t window_size_=0;std::unique_ptr<NativeMappingWindow> window_;
};
} // namespace posim_slam

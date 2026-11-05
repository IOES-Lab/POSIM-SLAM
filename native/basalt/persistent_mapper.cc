// SPDX-License-Identifier: Apache-2.0
#include "persistent_mapper.hpp"
#include "mapping_graph_information.hpp"
#include "mapping_feature_matcher.hpp"
#include "native_mapping_graph.hpp"
#include "native_mapping_window.hpp"
#include <Eigen/Cholesky>
#include <chrono>
#include <numeric>

namespace posim_slam {


PersistentMapper::PersistentMapper(const basalt::Calibration<double>& calibration,
    const basalt::VioConfig& config,double three_sigma_m,PersistentMapLimits limits,std::string epoch,std::filesystem::path input_archive,
    size_t active_window,std::shared_ptr<SubmapRowStore> row_store)
    :calibration_(calibration),config_(MappingInputStore::configuration(config)),three_sigma_m_(three_sigma_m),limits_(limits),
     graph_registry_(calibration_,config_,std::move(epoch)),
     latest_(std::make_shared<PersistentMapSnapshot>()) {
  if(!std::isfinite(three_sigma_m) || three_sigma_m<=0 || three_sigma_m>1)
    throw std::invalid_argument("invalid_persistent_mapping_configuration");
  input.set_capacity(2);
  try {
    store_=std::make_unique<MappingInputStore>(calibration_,config_,limits,std::move(input_archive),false,graph_registry_.epoch());
    matching_=store_->makeMatchHistory();
    graph_=std::make_unique<NativeMappingGraph>(*matching_,0,active_window,std::move(row_store));
    retained_.input=store_->snapshot();
  } catch(const std::exception& error) {
    // Keep draining the map handoff queue when storage cannot start. This
    // invalidates the map without blocking the independent navigation owner.
    invalidate(error.what());
  }
  ingest_thread_=std::thread(&PersistentMapper::ingest,this);
  try {mapper_thread_=std::thread(&PersistentMapper::optimize,this);}
  catch(...) {input.push(nullptr);ingest_thread_.join();throw;}
}
PersistentMapper::~PersistentMapper(){finish();}
void PersistentMapper::finish() {
  {std::lock_guard<std::mutex> lock(mutex_);if(finished_)return;finished_=true;}
  input.push(nullptr);
  ingest_thread_.join();mapper_thread_.join();
}
void PersistentMapper::setReference(int64_t stamp,const Sophus::SE3d& pose) {
  if(stamp<=0 || !pose.matrix().allFinite())throw std::invalid_argument("invalid_mapping_reference");
  std::lock_guard<std::mutex> lock(mutex_);
  if(retained_.reference_stamp)throw std::invalid_argument("mapping_reference_already_set");
  retained_.reference_stamp=stamp;retained_.reference_pose=pose;changed_.notify_one();
}
uint64_t PersistentMapper::requestUpdate() {
  std::lock_guard<std::mutex> lock(mutex_);
  if(finished_ || !error_.empty())return 0;
  requested_revision_=std::max(requested_revision_,retained_.revision);
  changed_.notify_all();return requested_revision_;
}
void PersistentMapper::invalidate(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!error_.empty())return;
  error_=error.empty()?"mapping_invalidated":error;
  auto failed=std::make_shared<PersistentMapSnapshot>(*latest_);
  failed->error=error_;failed->points.clear();failed->ids.clear();failed->observed=0;
  for(auto& point:failed->candidates){point.eligible=false;point.observed=false;}
  latest_=std::move(failed);
  changed_.notify_one();
}
PersistentMapper::Snapshot PersistentMapper::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);return latest_;
}
void PersistentMapper::ingest() {
  for(;;) {
    basalt::MargData::Ptr source;input.pop(source);
    if(!source) {
      std::lock_guard<std::mutex> lock(mutex_);input_finished_=true;changed_.notify_one();return;
    }
    try {
      {std::lock_guard<std::mutex> lock(mutex_);if(!error_.empty())continue;}
      const auto committed=store_->append(*source);
      std::lock_guard<std::mutex> lock(mutex_);
      if(!error_.empty())continue;
      retained_.input=committed;retained_.revision=committed->revision;changed_.notify_one();
    } catch(const std::exception& error){invalidate(error.what());}
  }
}
void PersistentMapper::optimize() {
  uint64_t completed=0;
  for(;;) {
    Batch batch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      changed_.wait_for(lock,std::chrono::seconds(5),[&]{return input_finished_ || !error_.empty() ||
        (retained_.reference_stamp && (requested_revision_>completed ||
         retained_.revision>=completed+limits_.update_retirements));});
      if(!error_.empty())return;
      if(retained_.revision==completed){if(input_finished_)return;continue;}
      if(!retained_.reference_stamp) {
        if(!input_finished_)continue;
        lock.unlock();invalidate("mapping_initial_reference_missing");return;
      }
      batch=retained_;
    }
    try {
      auto result=build(batch);
      std::lock_guard<std::mutex> lock(mutex_);
      if(error_.empty())latest_=std::move(result);
      completed=batch.revision;
    } catch(const std::exception& error){invalidate(error.what());return;}
  }
}
std::shared_ptr<PersistentMapSnapshot> PersistentMapper::build(const Batch& batch) {
  const auto started=std::chrono::steady_clock::now();
  const auto updated=graph_->update(batch.input,10,[this] {
    std::lock_guard<std::mutex> lock(mutex_);return !error_.empty();
  },batch.reference_stamp);auto& mapper=graph_->mapper();
  if(!mapper.getFramePoses().count(batch.reference_stamp))
    throw std::runtime_error("mapping_pose_image_or_reference_mismatch");
  const auto window=graph_->window();
  if(window && window->chart().reference()!=batch.reference_stamp)
    throw std::runtime_error("mapping_pose_image_or_reference_mismatch");
  const auto quality=window?window->mapInformation(three_sigma_m_):
    mappingGraphInformation(mapper,calibration_,config_,batch.reference_stamp,three_sigma_m_);
  const auto reference=mapper.getFramePoses().at(batch.reference_stamp).getPose();
  const Eigen::Matrix3d difference=batch.reference_pose.rotationMatrix()*reference.rotationMatrix().transpose();
  const double yaw=std::atan2(difference(1,0),difference(0,0));
  const Sophus::SO3d rotation=Sophus::SO3d::exp(Eigen::Vector3d(0,0,yaw));
  const Sophus::SE3d gauge(rotation,batch.reference_pose.translation()-rotation*reference.translation());
  auto result=std::make_shared<PersistentMapSnapshot>();
  result->revision=batch.revision;result->optimized_through_ns=batch.input->frames.rbegin()->first;
  result->keyframes=batch.input->frames.size();result->retirements=batch.input->revision;
  result->source_image_bytes=batch.input->source_image_bytes;
  result->source_factor_matrix_bytes=batch.input->source_factor_matrix_bytes;
  result->compact_input_bytes=batch.input->compactPayloadBytes();
  const auto matched=graph_->matchingSnapshot();result->matching_new_frames=matched->new_frames;
  result->matching_queried_frames=matched->queried_frames;
  result->matching_new_pairs=matched->new_pairs;result->matching_total_pairs=matched->evaluated_pairs;
  result->matching_numeric_bytes=matched->numeric_bytes;
  result->graph_new_frames=updated.new_frames;result->graph_new_tracks=updated.new_tracks;
  result->graph_extended_tracks=updated.extended_tracks;result->graph_merged_components=updated.merged_components;
  result->graph_removed_tracks=updated.removed_tracks;result->graph_preserved_landmarks=updated.preserved_landmarks;
  result->graph_initialized_landmarks=updated.initialized_landmarks;result->graph_numeric_bytes=updated.numeric_bytes;
  result->graph=graph_registry_.capture(batch.input,mapper,batch.reference_stamp,batch.reference_pose,gauge,window,graph_.get());
  static_cast<MappingOutput&>(*result)=mappingOutput(mapper,gauge,quality,result->graph);
  result->optimize_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  return result;
}
} // namespace posim_slam

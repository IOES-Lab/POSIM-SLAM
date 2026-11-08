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
namespace {
// A scheduling decision, distinct from a malformed measurement or I/O fault.
// The graph and matching owners unwind their staged transactions first.
struct MappingUpdateSuperseded final:std::exception {
  const char* what() const noexcept override {return "mapping_update_superseded";}
};
}

PersistentMapper::PersistentMapper(const basalt::Calibration<double>& calibration,
    const basalt::VioConfig& config,double three_sigma_m,PersistentMapLimits limits,std::string epoch,std::filesystem::path input_archive,
    size_t active_window,std::shared_ptr<SubmapRowStore> row_store)
    :calibration_(calibration),config_(MappingInputStore::configuration(config)),three_sigma_m_(three_sigma_m),limits_(limits),
     row_store_(std::move(row_store)),
     graph_registry_(calibration_,config_,std::move(epoch)),
     latest_(std::make_shared<PersistentMapSnapshot>()) {
  if(!std::isfinite(three_sigma_m) || three_sigma_m<=0 || three_sigma_m>1)
    throw std::invalid_argument("invalid_persistent_mapping_configuration");
  input.set_capacity(2);
  try {
    store_=std::make_unique<MappingInputStore>(calibration_,config_,limits,input_archive,false,graph_registry_.epoch());
    if(active_window && !row_store_ && !input_archive.empty())
      row_store_=std::make_shared<SubmapRowStore>(input_archive.string()+".rows",PagedMappingPolicy::row_cache_bytes);
    matching_=store_->makeMatchHistory();
    graph_=std::make_unique<NativeMappingGraph>(*matching_,0,active_window,row_store_);
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
  if(retained_.revision>requested_revision_) {
    requested_revision_=retained_.revision;requested_input_=retained_.input;
  }
  changed_.notify_all();return requested_revision_;
}
void PersistentMapper::invalidate(const std::string& error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!error_.empty())return;
  error_=error.empty()?"mapping_invalidated":error;
  requested_input_.reset();
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
      if(requested_input_ && requested_input_->revision>completed) {
        batch.input=requested_input_;batch.revision=requested_input_->revision;
        batch.requested=true;
      }
    }
    try {
      auto result=build(batch);
      std::lock_guard<std::mutex> lock(mutex_);
      if(error_.empty())latest_=std::move(result);
      completed=batch.revision;
      if(requested_input_ && requested_input_->revision<=completed)requested_input_.reset();
    } catch(const MappingUpdateSuperseded&) {
      // No measurements were consumed or published by the aborted graph
      // transaction. Keep the completed snapshot and the pinned request.
      ++background_updates_superseded_;
    } catch(const std::exception& error){invalidate(error.what());return;}
  }
}
std::shared_ptr<PersistentMapSnapshot> PersistentMapper::build(const Batch& batch) {
  const auto started=std::chrono::steady_clock::now();
  const auto updated=graph_->update(batch.input,10,[this,&batch] {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!error_.empty())return true;
    if(!batch.requested && requested_input_ && requested_input_->revision>batch.revision)
      throw MappingUpdateSuperseded();
    return false;
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
  result->background_updates_superseded=background_updates_superseded_;
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
  if(row_store_) {
    const auto usage=row_store_->statistics();
    result->row_cache_resident_bytes=usage.resident_bytes;result->row_cache_peak_bytes=usage.peak_resident_bytes;
    result->row_cache_pages=usage.pages;result->row_cache_reads=usage.reads;
    const auto metadata=row_store_->metadata()->statistics();
    result->metadata_cache_entries=metadata.entries;result->metadata_cache_peak_entries=metadata.peak_entries;
    result->metadata_cache_bytes=metadata.cache_bytes;result->metadata_cache_peak_bytes=metadata.peak_cache_bytes;
    result->metadata_directory_bytes=metadata.directory_bytes;result->metadata_directory_peak_bytes=metadata.peak_directory_bytes;
    result->metadata_workspace_peak_bytes=metadata.peak_workspace_bytes;result->metadata_database_bytes=metadata.database_bytes;
    if(window){result->original_identities=window->snapshot()->originalCount();result->assigned_identities=window->snapshot()->assignedCount();}
  }
  result->optimize_seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  return result;
}
} // namespace posim_slam

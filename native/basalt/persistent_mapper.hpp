// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_information.hpp"
#include "mapping_input_store.hpp"
#include "mapping_output.hpp"
#include <condition_variable>
#include <mutex>
#include <thread>
#include <array>
#include <string>

namespace posim_slam {
class NativeMappingGraph;
class SubmapRowStore;
struct PersistentMapSnapshot : MappingOutput {
  uint64_t revision=0;
  int64_t optimized_through_ns=0;
  size_t keyframes=0, retirements=0;
  size_t source_image_bytes=0,source_factor_matrix_bytes=0,compact_input_bytes=0;
  size_t row_cache_resident_bytes=0,row_cache_peak_bytes=0,row_cache_pages=0,row_cache_reads=0;
  size_t matching_new_frames=0,matching_queried_frames=0,matching_new_pairs=0,matching_total_pairs=0,matching_numeric_bytes=0;
  size_t graph_new_frames=0,graph_new_tracks=0,graph_extended_tracks=0,graph_merged_components=0,
    graph_removed_tracks=0,graph_preserved_landmarks=0,graph_initialized_landmarks=0,graph_numeric_bytes=0;
  double optimize_seconds=0;
  std::string error;
  MappingGraphRegistry::Snapshot graph;
};
using PersistentMapLimits=MappingInputLimits;

// The live provider selects this versioned policy explicitly. History identity
// capacity is separate from the unchanged input, archive and retirement quotas.
struct PagedMappingPolicy {
  static constexpr const char* name="conditional_paged_v1";
  static constexpr size_t active_keyframes=12,row_cache_bytes=8*1024*1024;
  static PersistentMapLimits limits() {
    PersistentMapLimits value;value.keyframes=kMaxMappingHistoryFrames;return value;
  }
};

// The ingest thread recovers each retirement's factors and extracts each new
// image's features once. A persistent chronological matching owner evaluates
// only new query frames, preserving old pair results. One bounded nonlinear
// mapper keeps optimized states while source track changes update its graph.
// An explicit active window uses source-owned conditional updates and their
// covariance. With a disk input archive, an omitted row store is created beside
// it with an 8 MiB cache. Existing directories are never reused or removed.
// Input snapshots remain exportable after the derived cache is released.
// Intermediate requests may coalesce;
// input measurements never do. A budget/input failure invalidates map output
// without stopping the caller's independently valid navigation estimator.
class PersistentMapper {
 public:
  using Snapshot=std::shared_ptr<const PersistentMapSnapshot>;
  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> input;
  PersistentMapper(const basalt::Calibration<double>& calibration,
                   const basalt::VioConfig& config,double three_sigma_m,
                   PersistentMapLimits limits={},std::string epoch={},std::filesystem::path input_archive={},
                   size_t active_window=0,std::shared_ptr<SubmapRowStore> row_store={});
  ~PersistentMapper();
  PersistentMapper(const PersistentMapper&)=delete;
  PersistentMapper& operator=(const PersistentMapper&)=delete;
  void setReference(int64_t stamp,const Sophus::SE3d& pose);
  void invalidate(const std::string& error);
  Snapshot snapshot() const;
  // Pin accepted input, not a moving "latest" target. Wake the optimizer
  // without blocking ingestion or waiting for a complete batch.
  uint64_t requestUpdate();
  const std::string& epoch() const {return graph_registry_.epoch();}
  void finish();

 private:
  struct Batch {
    MappingInputStore::Snapshot input;
    int64_t reference_stamp=0;
    Sophus::SE3d reference_pose;
    uint64_t revision=0;
  };
  void ingest();
  void optimize();
  std::shared_ptr<PersistentMapSnapshot> build(const Batch& batch);
  basalt::Calibration<double> calibration_;
  basalt::VioConfig config_;
  double three_sigma_m_;
  PersistentMapLimits limits_;
  std::shared_ptr<SubmapRowStore> row_store_;
  MappingGraphRegistry graph_registry_;
  std::unique_ptr<MappingInputStore> store_;
  std::unique_ptr<MappingMatchHistory> matching_;
  std::unique_ptr<NativeMappingGraph> graph_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  Batch retained_;
  bool input_finished_=false,finished_=false;
  std::string error_;
  Snapshot latest_;
  uint64_t requested_revision_=0;
  std::thread ingest_thread_,mapper_thread_;
};
} // namespace posim_slam

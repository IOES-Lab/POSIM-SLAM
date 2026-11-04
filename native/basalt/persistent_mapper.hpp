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
struct PersistentMapSnapshot : MappingOutput {
  uint64_t revision=0;
  int64_t optimized_through_ns=0;
  size_t keyframes=0, retirements=0;
  size_t source_image_bytes=0,source_factor_matrix_bytes=0,compact_input_bytes=0;
  size_t matching_new_frames=0,matching_queried_frames=0,matching_new_pairs=0,matching_total_pairs=0,matching_numeric_bytes=0;
  double optimize_seconds=0;
  std::string error;
  MappingGraphRegistry::Snapshot graph;
};
using PersistentMapLimits=MappingInputLimits;

// The ingest thread recovers each retirement's factors and extracts each new
// image's features once. A persistent chronological matching owner evaluates
// only new query frames, preserving old pair results. The bounded nonlinear
// graph is still rebuilt from the resulting original associations; incremental
// graph/window ownership is a separate transition. Intermediate requests may coalesce;
// input measurements never do. A budget/input failure invalidates map output
// without stopping the caller's independently valid navigation estimator.
class PersistentMapper {
 public:
  using Snapshot=std::shared_ptr<const PersistentMapSnapshot>;
  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> input;
  PersistentMapper(const basalt::Calibration<double>& calibration,
                   const basalt::VioConfig& config,double three_sigma_m,
                   PersistentMapLimits limits={},std::string epoch={},std::filesystem::path input_archive={});
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
  MappingGraphRegistry graph_registry_;
  std::unique_ptr<MappingInputStore> store_;
  std::unique_ptr<MappingMatchHistory> matching_;
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

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_information.hpp"
#include <basalt/vi_estimator/nfr_mapper.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <array>
#include <string>

namespace posim_slam {
struct PersistentMapCandidate {
  int64_t id;
  Eigen::Vector3d position;
  bool eligible=false, finite=false, observed=false, mature=false;
  double maximum_sigma_m=std::numeric_limits<double>::infinity();
};
struct PersistentMapSnapshot {
  uint64_t revision=0;
  int64_t optimized_through_ns=0;
  size_t keyframes=0, retirements=0, observations=0, factors=0, observed=0;
  size_t nonfinite=0;
  double optimize_seconds=0;
  std::string error;
  std::vector<PersistentMapCandidate> candidates;
  std::vector<std::array<float,3>> points;
  std::vector<int> ids;
};
struct PersistentMapLimits {
  size_t keyframes=kMaxJointColumns/6;
  size_t image_bytes=96*1024*1024;
  size_t retirements=512;
  size_t update_retirements=8;
};

// The ingest thread only validates and retains immutable input. The mapper
// rebuilds a graph from a snapshot of that input, so an old graph's observations
// are never appended again. Intermediate optimization requests may coalesce;
// input measurements never do. A budget/input failure invalidates map output
// without stopping the caller's independently valid navigation estimator.
class PersistentMapper {
 public:
  using Snapshot=std::shared_ptr<const PersistentMapSnapshot>;
  tbb::concurrent_bounded_queue<basalt::MargData::Ptr> input;
  PersistentMapper(const basalt::Calibration<double>& calibration,
                   const basalt::VioConfig& config,double three_sigma_m,
                   PersistentMapLimits limits={});
  ~PersistentMapper();
  PersistentMapper(const PersistentMapper&)=delete;
  PersistentMapper& operator=(const PersistentMapper&)=delete;
  void setReference(int64_t stamp,const Sophus::SE3d& pose);
  void invalidate(const std::string& error);
  Snapshot snapshot() const;
  // Pin accepted input, not a moving "latest" target. Wake the optimizer
  // without blocking ingestion or waiting for a complete batch.
  uint64_t requestUpdate();
  void finish();

 private:
  struct Batch {
    std::vector<basalt::MargData::Ptr> factors;
    std::map<int64_t,basalt::OpticalFlowInput::Ptr> images;
    int64_t reference_stamp=0;
    Sophus::SE3d reference_pose;
    uint64_t revision=0;
  };
  void ingest();
  void optimize();
  std::shared_ptr<PersistentMapSnapshot> build(const Batch& batch) const;
  basalt::Calibration<double> calibration_;
  basalt::VioConfig config_;
  double three_sigma_m_;
  PersistentMapLimits limits_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  Batch retained_;
  std::set<int64_t> retired_;
  size_t image_bytes_=0;
  bool input_finished_=false,finished_=false;
  std::string error_;
  Snapshot latest_;
  uint64_t requested_revision_=0;
  std::thread ingest_thread_,mapper_thread_;
};
} // namespace posim_slam

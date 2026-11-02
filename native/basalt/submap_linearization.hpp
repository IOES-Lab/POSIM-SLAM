// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "submap_information.hpp"
#include <algorithm>

namespace posim_slam {
// Replaceable original likelihoods on a bounded frontier. Sealing commits a
// fixed tangent-space conditional; nonlinear rehydration is the caller's job.
// Every removed original must occur exactly once in the replacement groups.
// No normal-matrix subtraction, added prior or measurement duplication occurs.
class SubmapLinearization {
 public:
  using Information=SubmapInformation;
  using Coordinate=Information::Coordinate;
  struct Rows {
    std::vector<std::string> originals;
    std::vector<Coordinate> columns;
    Eigen::MatrixXd jacobian,constraints;
    Eigen::VectorXd residual,target;
    size_t bytes() const {
      return sizeof(double)*size_t(jacobian.size()+constraints.size()+residual.size()+target.size());
    }
  };
  struct Data {
    uint64_t revision=0;
    Information::Snapshot base,information;
    std::vector<std::shared_ptr<const Rows>> active;
    size_t active_bytes=0;
    size_t numericBytes() const {
      // Conditional nodes are shared by base/information. Count the two
      // distinct frontier matrices and retained replaceable rows once each.
      const size_t extra=base==information?0:base->numericBytes()-base->history_bytes;
      return information->numericBytes()+extra+active_bytes;
    }
   private:
    friend class SubmapLinearization;
    std::shared_ptr<const int> owner;
  };
  using Snapshot=std::shared_ptr<const Data>;
  SubmapLinearization():SubmapLinearization(Information::Limits{}) {}
  explicit SubmapLinearization(Information::Limits limits)
      :core_(std::make_unique<Information>(limits)),limits_(limits) {initialize();}
  // The supplied prior is already owned by the same world-reference lineage.
  SubmapLinearization(const Information& source,const Information::Snapshot& base,
                       Information::Limits limits)
      :core_(source.fork(base,limits)),limits_(limits) {initialize();}
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  Snapshot append(const std::vector<Rows>& rows) {return replace({},rows);}
  Snapshot replace(const std::vector<std::string>& removed,const std::vector<Rows>& rows) {
    std::lock_guard<std::mutex> writer(writer_);
    const auto previous=snapshot();auto next=std::make_shared<Data>(*previous);
    if(rows.empty())throw std::invalid_argument("submap_replacement_empty");
    const auto remove=keys(removed);std::set<std::string> replacing;
    std::vector<std::shared_ptr<const Rows>> retained;
    for(const auto& group:previous->active) {
      bool selected=false,complete=true;
      for(const auto& key:group->originals) {
        selected=selected || remove.count(key);complete=complete && remove.count(key);
      }
      if(selected && !complete)throw std::invalid_argument("submap_replacement_partial_group");
      if(selected)replacing.insert(group->originals.begin(),group->originals.end());
      else retained.push_back(group);
    }
    if(replacing!=remove)throw std::invalid_argument("submap_replacement_requires_active_originals");
    std::set<std::string> incoming;
    size_t source_bytes=0;
    for(const auto& group:retained)source_bytes+=group->bytes();
    for(const auto& group:rows) {
      Information::validateRows(group.columns,group.jacobian,group.residual,group.constraints,group.target);
      if(source_bytes>limits_.numeric_bytes || group.bytes()>limits_.numeric_bytes-source_bytes)
        throw std::runtime_error("submap_linearization_capacity");
      source_bytes+=group.bytes();
      const auto group_keys=keys(group.originals);
      if(group_keys.empty())throw std::invalid_argument("submap_replacement_empty");
      for(const auto& key:group_keys) {
        if(!incoming.insert(key).second ||
            (previous->information->measurements.count(key) && !remove.count(key)))
          throw std::invalid_argument("submap_measurement_reused");
      }
      // Copy caller-owned matrices before validating/committing the transaction.
      retained.push_back(std::make_shared<const Rows>(group));
    }
    if(!std::includes(incoming.begin(),incoming.end(),remove.begin(),remove.end()))
      throw std::invalid_argument("submap_replacement_original_missing");
    next->active=std::move(retained);next->active_bytes=0;
    for(const auto& group:next->active) {
      if(group->bytes()>limits_.numeric_bytes-next->active_bytes)
        throw std::runtime_error("submap_linearization_capacity");
      next->active_bytes+=group->bytes();
    }
    rebuild(*next);
    // Changing association/linearization must not silently erase a previously
    // published coordinate. Preserve it explicitly, even if now unobserved.
    if(!std::includes(next->information->known.begin(),next->information->known.end(),
        previous->information->known.begin(),previous->information->known.end()))
      throw std::invalid_argument("submap_replacement_coordinate_missing");
    return commit(std::move(next));
  }
  Snapshot seal(const std::vector<Coordinate>& kept) {
    std::lock_guard<std::mutex> writer(writer_);
    auto next=std::make_shared<Data>(*snapshot());auto branch=core_->fork(next->information);
    next->base=next->information=branch->seal(kept);
    next->active.clear();next->active_bytes=0;
    return commit(std::move(next));
  }
  // A genuinely new observation touching archived coordinates updates the
  // sealed base first. The active likelihoods are then applied exactly once.
  Snapshot appendLate(const Rows& rows) {
    std::lock_guard<std::mutex> writer(writer_);
    auto next=std::make_shared<Data>(*snapshot());
    for(const auto& key:keys(rows.originals))
      if(next->information->measurements.count(key))throw std::invalid_argument("submap_measurement_reused");
    auto branch=core_->fork(next->base);
    next->base=branch->appendLate(rows.originals,rows.columns,rows.jacobian,rows.residual,rows.constraints,rows.target);
    rebuild(*next);return commit(std::move(next));
  }
  Information::Solution solve(const Snapshot& input) const {
    requireOwner(input);return core_->solve(input->information);
  }
  Information::Query query(const Information::Solution& solution,
      const std::vector<Coordinate>& coordinates,const Eigen::MatrixXd& derivative) const {
    return core_->query(solution,coordinates,derivative);
  }
 private:
  static std::set<std::string> keys(const std::vector<std::string>& input) {
    std::set<std::string> result;
    for(const auto& key:input)
      if(key.empty() || key.size()>256 || !result.insert(key).second)
        throw std::invalid_argument("submap_measurement_reused");
    return result;
  }
  void initialize() {
    auto value=std::make_shared<Data>();value->owner=identity_;
    value->base=value->information=core_->snapshot();data_=value;
  }
  void rebuild(Data& next) const {
    if(next.base->numericBytes()>limits_.numeric_bytes ||
        next.active_bytes>limits_.numeric_bytes-next.base->numericBytes())
      throw std::runtime_error("submap_linearization_capacity");
    auto branch=core_->fork(next.base);
    for(const auto& group:next.active)
      branch->append(group->originals,group->columns,group->jacobian,group->residual,group->constraints,group->target);
    next.information=branch->snapshot();
  }
  void requireOwner(const Snapshot& input) const {
    if(!input || input->owner!=identity_)throw std::invalid_argument("submap_linearization_owner");
  }
  Snapshot commit(std::shared_ptr<Data> next) {
    if(next->information->order.size()>limits_.active_columns ||
        next->information->known.size()>limits_.coordinates ||
        next->information->nodes.size()>limits_.nodes ||
        next->information->measurements.size()>limits_.measurements ||
        next->numericBytes()>limits_.numeric_bytes)
      throw std::runtime_error("submap_linearization_capacity");
    ++next->revision;Snapshot immutable=next;std::atomic_store(&data_,immutable);return immutable;
  }
  std::unique_ptr<Information> core_;
  Information::Limits limits_;
  std::shared_ptr<const int> identity_=std::make_shared<const int>(0);
  mutable std::mutex writer_;Snapshot data_;
};
} // namespace posim_slam

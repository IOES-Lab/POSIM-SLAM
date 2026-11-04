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
      // Late active rows can replace conditional nodes on their branch. Keep
      // both the original base and changed node payloads in the numeric quota.
      size_t extra=base==information?0:base->numericBytes()-base->history_bytes;
      if(base!=information) {
        std::set<const Information::Node*> shared;
        for(const auto& node:information->nodes)shared.insert(node.get());
        for(const auto& node:base->nodes)if(!shared.count(node.get()))extra+=node->bytes();
      }
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
  Snapshot includeConditional(const std::vector<std::string>& originals,const Information::Node& node) {
    std::lock_guard<std::mutex> writer(writer_);auto next=std::make_shared<Data>(*snapshot());
    auto branch=core_->fork(next->base);next->base=branch->includeConditional(originals,node);
    rebuild(*next);return commit(std::move(next));
  }
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
  // Transfer complete selected active groups into the historical base while
  // the other original groups remain replaceable. Eliminate only coordinates
  // outside `kept`; remaining frontier coordinates must fit that separator.
  // Remaining groups may still reference archived conditional coordinates.
  // The same information lineage and original keys survive the transaction.
  Snapshot retire(const std::vector<std::string>& originals,const std::vector<Coordinate>& kept) {
    std::lock_guard<std::mutex> writer(writer_);const auto previous=snapshot();
    const auto retiring=keys(originals);std::set<std::string> found;
    const std::set<Coordinate> wanted(kept.begin(),kept.end());
    const std::set<Coordinate> available(previous->information->order.begin(),previous->information->order.end());
    if(wanted.size()!=kept.size() || !std::includes(available.begin(),available.end(),wanted.begin(),wanted.end()))
      throw std::invalid_argument("submap_retire_coordinates");
    auto next=std::make_shared<Data>(*previous);Data selected=*previous;
    next->active.clear();selected.active.clear();next->active_bytes=selected.active_bytes=0;
    for(const auto& group:previous->active) {
      bool any=false,all=true;
      for(const auto& key:group->originals){any=any || retiring.count(key);all=all && retiring.count(key);}
      if(any && !all)throw std::invalid_argument("submap_replacement_partial_group");
      if(any) {
        found.insert(group->originals.begin(),group->originals.end());
        selected.active.push_back(group);selected.active_bytes+=group->bytes();
      } else {
        for(auto column:group->columns)if(available.count(column) && !wanted.count(column))
          throw std::invalid_argument("submap_retire_active_coordinate");
        next->active.push_back(group);next->active_bytes+=group->bytes();
      }
    }
    if(found!=retiring)throw std::invalid_argument("submap_replacement_requires_active_originals");
    rebuild(selected);auto branch=core_->fork(selected.information);
    std::vector<Coordinate> base_kept;
    for(auto column:branch->snapshot()->order)if(wanted.count(column))base_kept.push_back(column);
    if(base_kept.size()<branch->snapshot()->order.size())branch->seal(base_kept);
    next->base=branch->snapshot();rebuild(*next);
    if(next->information->measurements!=previous->information->measurements ||
        next->information->known!=previous->information->known)
      throw std::logic_error("submap_retire_original_ownership");
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
    core_->releaseContextSnapshot();
  }
  void rebuild(Data& next) const {
    if(next.base->numericBytes()>limits_.numeric_bytes ||
        next.active_bytes>limits_.numeric_bytes-next.base->numericBytes())
      throw std::runtime_error("submap_linearization_capacity");
    auto branch=core_->fork(next.base);
    std::vector<std::shared_ptr<const Rows>> frontier,late;
    const std::set<Coordinate> root(next.base->order.begin(),next.base->order.end());
    for(const auto& group:next.active) {
      const bool archived=std::any_of(group->columns.begin(),group->columns.end(),[&](auto c){return next.base->known.count(c) && !root.count(c);});
      (archived?late:frontier).push_back(group);
    }
    // Preserve original groups for replacement, but factor bounded batches.
    // Refactoring the entire separator after every two-row pixel group makes
    // update work grow with groups times the cube of the separator width.
    std::vector<Coordinate> columns;std::map<Coordinate,size_t> offsets;
    size_t rows=0,exact=0;
    for(const auto& group:frontier) {
      std::set<Coordinate> local;
      for(Coordinate column:group->columns) {
        if(column<0 || !local.insert(column).second)throw std::invalid_argument("submap_coordinates");
        if(offsets.emplace(column,columns.size()).second)columns.push_back(column);
      }
      rows+=group->jacobian.rows();exact+=group->constraints.rows();
    }
    if(columns.size()>limits_.active_columns)throw std::runtime_error("submap_coordinate_capacity");
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(std::min(size_t(4096),rows),columns.size());
    Eigen::MatrixXd c=Eigen::MatrixXd::Zero(std::min(size_t(1024),exact),columns.size());
    Eigen::VectorXd r(j.rows()),target(c.rows());Eigen::Index used=0,used_exact=0;
    std::vector<std::string> originals;
    auto flush=[&] {
      if(originals.empty())return;
      branch->append(originals,columns,j.topRows(used),r.head(used),c.topRows(used_exact),target.head(used_exact));
      originals.clear();used=used_exact=0;j.setZero();c.setZero();
    };
    for(const auto& group:frontier) {
      const auto n=group->jacobian.rows(),m=group->constraints.rows();
      if(used+n>j.rows() || used_exact+m>c.rows())flush();
      for(size_t i=0;i<group->columns.size();++i) {
        const auto column=offsets.at(group->columns[i]);
        j.block(used,column,n,1)=group->jacobian.col(i);
        if(m)c.block(used_exact,column,m,1)=group->constraints.col(i);
      }
      r.segment(used,n)=group->residual;target.segment(used_exact,m)=group->target;
      used+=n;used_exact+=m;
      originals.insert(originals.end(),group->originals.begin(),group->originals.end());
    }
    flush();
    // New-view rows may touch old poses/landmarks. Recondition their historical
    // path from the immutable base on every replacement, applying each active
    // original once. Batches bound temporary columns as well as row counts.
    std::vector<std::shared_ptr<const Rows>> pending;
    std::vector<Coordinate> late_columns;std::map<Coordinate,size_t> late_offsets;
    size_t late_rows=0,late_exact=0;
    auto flush_late=[&] {
      if(pending.empty())return;
      Eigen::MatrixXd j=Eigen::MatrixXd::Zero(late_rows,late_columns.size()),c=Eigen::MatrixXd::Zero(late_exact,late_columns.size());
      Eigen::VectorXd r(late_rows),target(late_exact);Eigen::Index row=0,exact=0;std::vector<std::string> originals;
      for(const auto& group:pending) {
        const auto n=group->jacobian.rows(),m=group->constraints.rows();
        for(size_t i=0;i<group->columns.size();++i) {
          const auto column=late_offsets.at(group->columns[i]);j.block(row,column,n,1)=group->jacobian.col(i);
          if(m)c.block(exact,column,m,1)=group->constraints.col(i);
        }
        r.segment(row,n)=group->residual;target.segment(exact,m)=group->target;row+=n;exact+=m;
        originals.insert(originals.end(),group->originals.begin(),group->originals.end());
      }
      branch->appendLate(originals,late_columns,j,r,c,target);
      pending.clear();late_columns.clear();late_offsets.clear();late_rows=late_exact=0;
    };
    for(const auto& group:late) {
      const auto additional=std::count_if(group->columns.begin(),group->columns.end(),[&](auto c){return !late_offsets.count(c);});
      if(late_rows+group->jacobian.rows()>4096 || late_exact+group->constraints.rows()>1024 ||
          late_columns.size()+additional>Information::temporaryColumns(limits_))flush_late();
      for(auto column:group->columns)if(late_offsets.emplace(column,late_columns.size()).second)late_columns.push_back(column);
      if(late_columns.size()>Information::temporaryColumns(limits_))throw std::runtime_error("submap_coordinate_capacity");
      pending.push_back(group);late_rows+=group->jacobian.rows();late_exact+=group->constraints.rows();
    }
    flush_late();
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

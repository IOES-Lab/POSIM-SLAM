// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "constrained_information.hpp"
#include "mapping_history_budget.hpp"
#include "submap_measurements.hpp"
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace posim_slam {
// A single fixed tangent-space linearization, not a nonlinear SLAM frontend.
// Coordinates and original measurement identities belong to the caller. The
// caller supplies its original world reference once; this class adds no prior.
class SubmapInformation {
  friend class SubmapLinearization;
 public:
  using Coordinate=int64_t;
  struct Limits {
    size_t active_columns=256,nodes=4096,coordinates=kMaxMappingHistoryCoordinates,measurements=65536;
    size_t numeric_bytes=96*1024*1024;
    // Zero uses the retained-frontier quota. A caller may reserve a distinct
    // temporary elimination workspace within the same existing hard ceiling.
    size_t temporary_columns=0;
  };
  struct Node {
    std::vector<Coordinate> kept,removed;
    Eigen::MatrixXd regression,noise,unbounded,deterministic;
    Eigen::VectorXd offset;
    double roundoff=0;
    size_t bytes() const {return sizeof(double)*size_t(regression.size()+noise.size()+unbounded.size()+deterministic.size()+offset.size());}
  };
  struct Data {
    uint64_t revision=0;
    std::vector<Coordinate> order;
    Eigen::MatrixXd jacobian,constraints;
    Eigen::VectorXd residual,target;
    std::vector<std::shared_ptr<const Node>> nodes;
    std::set<Coordinate> known;
    SubmapMeasurements measurements;
    std::set<std::string> conditional_sources;
    size_t history_bytes=0;
    // Numerical precision of the source square root, not measurement noise.
    // It survives compression so a later, smaller solve cannot promote its
    // discarded round-off directions into information.
    double roundoff=0;
    size_t numericBytes() const {
      return history_bytes+sizeof(double)*size_t(jacobian.size()+constraints.size()+residual.size()+target.size());
    }
   private:
    friend class SubmapInformation;
    std::shared_ptr<const int> owner;
  };
  using Snapshot=std::shared_ptr<const Data>;
  class Solution {
    friend class SubmapInformation;
    Snapshot input;
    Eigen::VectorXd mean;
    Eigen::MatrixXd noise,unbounded;
    Solution(Snapshot state,Eigen::VectorXd value,Eigen::MatrixXd finite,Eigen::MatrixXd free)
      :input(std::move(state)),mean(std::move(value)),noise(std::move(finite)),unbounded(std::move(free)) {}
   public:
    Solution(const Solution&)=default;Solution(Solution&&)=default;
  };
  struct Query {
    Eigen::VectorXd mean;
    Eigen::MatrixXd covariance,unbounded_gram;
    bool observed=false;
  };
  SubmapInformation():SubmapInformation(Limits{}) {}
  explicit SubmapInformation(Limits limits):limits_(limits) {
    if(!limits.active_columns || limits.active_columns>size_t(kMaxJointColumns) ||
        !limits.nodes || limits.nodes>4096 || !limits.coordinates || limits.coordinates>65536 ||
        !limits.measurements || limits.measurements>65536 || !limits.numeric_bytes || limits.numeric_bytes>96*1024*1024 ||
        (limits.temporary_columns && (limits.temporary_columns<limits.active_columns || limits.temporary_columns>size_t(kMaxJointColumns))))
      throw std::invalid_argument("submap_information_limits");
    auto empty=std::make_shared<Data>();empty->owner=identity_;empty->jacobian.resize(0,0);empty->constraints.resize(0,0);
    data_=empty;
  }
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  static size_t temporaryColumns(const Limits& limits) {return limits.temporary_columns?limits.temporary_columns:limits.active_columns;}
  // Restore a conditional complementary to an already-owned marginal. The
  // caller supplies the conditional from those same original source rows.
  // No measurement is inserted; this only exposes its eliminated coordinates.
  Snapshot includeConditional(const std::vector<std::string>& originals,const Node& conditional) {
    std::lock_guard<std::mutex> writer(writer_);auto next=std::make_shared<Data>(*snapshot());
    std::set<std::string> keys;
    if(originals.empty())throw std::invalid_argument("submap_conditional_originals");
    for(const auto& key:originals)if(!next->measurements.count(key) ||
        next->conditional_sources.count(key) || !keys.insert(key).second)
      throw std::invalid_argument("submap_conditional_originals");
    const auto kept=index(conditional.kept),removed=index(conditional.removed);
    const auto n=Eigen::Index(removed.size()),k=Eigen::Index(kept.size());
    if(!n || n+k>temporaryColumns(limits_) || conditional.regression.rows()!=n || conditional.regression.cols()!=k ||
        conditional.offset.size()!=n || conditional.noise.rows()!=n || conditional.unbounded.rows()!=n ||
        conditional.deterministic.rows()!=n || conditional.noise.cols()+conditional.unbounded.cols()+conditional.deterministic.cols()!=n ||
        !conditional.regression.allFinite() || !conditional.offset.allFinite() || !conditional.noise.allFinite() ||
        !conditional.unbounded.allFinite() || !conditional.deterministic.allFinite() ||
        !std::isfinite(conditional.roundoff) || conditional.roundoff<0)
      throw std::invalid_argument("submap_conditional_shape");
    for(auto column:conditional.kept)if(!next->known.count(column))throw std::invalid_argument("submap_conditional_parent");
    for(auto column:conditional.removed)if(next->known.count(column))throw std::invalid_argument("submap_conditional_coordinate_reused");
    Eigen::MatrixXd basis(n,n);Eigen::Index offset=0;
    for(Eigen::Index i=0;i<conditional.noise.cols();++i) {
      const double norm=conditional.noise.col(i).stableNorm();
      if(!std::isfinite(norm) || norm<=0)throw std::invalid_argument("submap_conditional_basis");
      basis.col(offset++)=conditional.noise.col(i)/norm;
    }
    basis.middleCols(offset,conditional.unbounded.cols())=conditional.unbounded;offset+=conditional.unbounded.cols();
    basis.rightCols(conditional.deterministic.cols())=conditional.deterministic;
    if(!(basis.transpose()*basis).isApprox(Eigen::MatrixXd::Identity(n,n),1e-10))
      throw std::invalid_argument("submap_conditional_basis");
    if(next->nodes.size()>=limits_.nodes)throw std::runtime_error("submap_node_capacity");
    if(next->known.size()+removed.size()>limits_.coordinates)throw std::runtime_error("submap_coordinate_capacity");
    next->known.insert(conditional.removed.begin(),conditional.removed.end());
    next->conditional_sources.insert(keys.begin(),keys.end());
    next->history_bytes+=conditional.bytes();
    next->nodes.insert(next->nodes.begin(),std::make_shared<const Node>(conditional));
    return commit(std::move(next));
  }
  // Independent writer over the same immutable information lineage. This
  // permits an atomic caller transaction to rebuild only unsealed factors;
  // the published source snapshot and its conditional owners stay unchanged.
  std::unique_ptr<SubmapInformation> fork(const Snapshot& input) const {
    return fork(input,limits_);
  }
  std::unique_ptr<SubmapInformation> fork(const Snapshot& input,Limits limits) const {
    requireOwner(input);
    auto branch=std::make_unique<SubmapInformation>(limits);
    if(input->order.size()>limits.active_columns || input->nodes.size()>limits.nodes ||
        input->known.size()>limits.coordinates || input->measurements.size()>limits.measurements ||
        input->numericBytes()>limits.numeric_bytes)throw std::runtime_error("submap_fork_capacity");
    branch->identity_=identity_;branch->data_=input;
    return branch;
  }
  static void validateRows(const std::vector<Coordinate>& columns,
      const Eigen::MatrixXd& j,const Eigen::VectorXd& r,
      const Eigen::MatrixXd& c,const Eigen::VectorXd& target) {
    if(columns.empty() || columns.size()>size_t(kMaxJointColumns) ||
        j.cols()!=Eigen::Index(columns.size()) || j.rows()>4096 ||
        r.size()!=j.rows() || !j.allFinite() || !r.allFinite() ||
        (c.rows() && (c.cols()!=j.cols() || c.rows()>1024 || target.size()!=c.rows() || !c.allFinite() || !target.allFinite())) ||
        (!c.rows() && target.size()))throw std::invalid_argument("submap_information_rows");
  }
  // Whitened rows J*x+r and optional exact C*x=target. Each original key is
  // consumed once. Use appendLate for factors touching archived coordinates.
  Snapshot append(const std::vector<std::string>& originals,const std::vector<Coordinate>& columns,
      const Eigen::MatrixXd& j,const Eigen::VectorXd& r,
      const Eigen::MatrixXd& c=Eigen::MatrixXd(),const Eigen::VectorXd& target=Eigen::VectorXd(),double roundoff=0) {
    std::lock_guard<std::mutex> writer(writer_);auto next=std::make_shared<Data>(*snapshot());
    if(originals.empty())throw std::invalid_argument("submap_information_rows");
    validateRows(columns,j,r,c,target);
    if(!std::isfinite(roundoff) || roundoff<0)throw std::invalid_argument("submap_information_precision");
    std::set<std::string> keys;
    for(const auto& key:originals)
      if(key.empty() || key.size()>256 || next->measurements.count(key) || !keys.insert(key).second)
        throw std::invalid_argument("submap_measurement_reused");
    if(next->measurements.size()+keys.size()>limits_.measurements)throw std::runtime_error("submap_measurement_capacity");
    auto offsets=index(next->order);std::set<Coordinate> source;
    for(Coordinate coordinate:columns) {
      if(coordinate<0 || !source.insert(coordinate).second)throw std::invalid_argument("submap_coordinates");
      if(!offsets.count(coordinate)) {
        if(next->known.count(coordinate))throw std::invalid_argument("submap_requires_retired_factor_update");
        offsets.emplace(coordinate,next->order.size());next->order.push_back(coordinate);next->known.insert(coordinate);
      }
    }
    if(next->order.size()>limits_.active_columns || next->known.size()>limits_.coordinates)
      throw std::runtime_error("submap_coordinate_capacity");
    // Compact incoming rows before stacking. Work depends on the frontier
    // dimensions, not the total observation count across the mission.
    Eigen::MatrixXd incoming=j;Eigen::VectorXd value=r;
    const bool incoming_compacted=compact(incoming,value,roundoff,roundoff>0);
    next->roundoff=std::hypot(next->roundoff,roundoff);
    const auto previous_rows=next->jacobian.rows(),previous_columns=next->jacobian.cols();
    const auto n=Eigen::Index(next->order.size());
    next->jacobian.conservativeResize(previous_rows+incoming.rows(),n);
    // New coordinates do not occur in the previously committed prior.
    if(n>previous_columns)next->jacobian.topRows(previous_rows).rightCols(n-previous_columns).setZero();
    next->jacobian.bottomRows(incoming.rows()).setZero();
    for(size_t i=0;i<columns.size();++i)next->jacobian.bottomRows(incoming.rows()).col(offsets.at(columns[i]))=incoming.col(i);
    next->residual.conservativeResize(previous_rows+value.size());next->residual.tail(value.size())=value;
    const auto old_constraints=next->constraints.rows(),old_n=next->constraints.cols();
    next->constraints.conservativeResize(old_constraints+c.rows(),n);
    if(n>old_n)next->constraints.topRows(old_constraints).rightCols(n-old_n).setZero();
    next->constraints.bottomRows(c.rows()).setZero();
    for(size_t i=0;c.rows() && i<columns.size();++i)next->constraints.bottomRows(c.rows()).col(offsets.at(columns[i]))=c.col(i);
    next->target.conservativeResize(old_constraints+target.size());next->target.tail(target.size())=target;
    // Scattering the only row block into a larger/permuted coordinate list
    // preserves its independent rows. Stacking with any earlier rows does not.
    const bool compacted=(!previous_rows && incoming_compacted) ||
      compact(next->jacobian,next->residual,next->roundoff,next->roundoff>0);
    compactConstraints(*next);
    next->measurements.insert(keys.begin(),keys.end());return commit(std::move(next),compacted);
  }
  Snapshot seal(const std::vector<Coordinate>& kept) {
    std::lock_guard<std::mutex> writer(writer_);const auto previous=snapshot();
    const bool compacted=compacted_.lock()==previous;
    auto next=std::make_shared<Data>(*previous);
    const auto offsets=index(next->order);const auto selected=index(kept);auto node=std::make_shared<Node>();node->kept=kept;
    for(Coordinate coordinate:kept)if(!offsets.count(coordinate))throw std::invalid_argument("submap_separator_coordinates");
    for(Coordinate coordinate:next->order)if(!selected.count(coordinate))node->removed.push_back(coordinate);
    if(node->removed.empty())throw std::invalid_argument("submap_seal_requires_removed_coordinates");
    if(next->nodes.size()>=limits_.nodes)throw std::runtime_error("submap_node_capacity");
    std::vector<Coordinate> order=kept;order.insert(order.end(),node->removed.begin(),node->removed.end());
    Eigen::MatrixXd j(next->jacobian.rows(),order.size()),c(next->constraints.rows(),order.size());
    for(size_t i=0;i<order.size();++i){j.col(i)=next->jacobian.col(offsets.at(order[i]));c.col(i)=next->constraints.col(offsets.at(order[i]));}
    // Elimination must receive independent likelihood rows and an explicit
    // constant residual. Otherwise QR round-off in redundant rows can become
    // a spurious observable direction when a small marginal is solved alone.
    // Column permutation preserves a previously compacted immutable snapshot.
    // Other snapshots conservatively perform the full rank-aware calculation.
    if(!compacted)compact(j,next->residual,next->roundoff,true);node->roundoff=next->roundoff;
    const auto split=splitConstrainedInformation(j,next->residual,kept.size(),c,next->target,next->roundoff);
    node->regression=split.reduced.backsub_matrix;node->offset=split.reduced.backsub_offset;
    node->noise=split.noise;node->unbounded=split.unbounded;node->deterministic=split.deterministic;
    if(!node->regression.allFinite() || !node->offset.allFinite() || !node->noise.allFinite() ||
        !node->unbounded.allFinite() || !node->deterministic.allFinite())throw std::runtime_error("submap_nonfinite_conditional");
    next->order=kept;next->jacobian=split.reduced.jacobian;next->residual=split.reduced.residual;
    next->constraints=split.constraint_jacobian;next->target=split.constraint_target;
    next->roundoff=split.source_roundoff;
    compact(next->jacobian,next->residual,next->roundoff,true);next->history_bytes+=node->bytes();next->nodes.push_back(std::move(node));
    return commit(std::move(next),true);
  }
  // Recondition affected nodes in elimination order, passing only the new
  // marginal message onward. Original root information enters once, after
  // conditional replacement. Unaffected nodes remain shared and immutable.
  Snapshot appendLate(const std::vector<std::string>& originals,const std::vector<Coordinate>& columns,
      const Eigen::MatrixXd& j,const Eigen::VectorXd& r,
      const Eigen::MatrixXd& c=Eigen::MatrixXd(),const Eigen::VectorXd& target=Eigen::VectorXd(),double roundoff=0) {
    std::lock_guard<std::mutex> writer(writer_);auto next=std::make_shared<Data>(*snapshot());
    // Reuse append's complete row, key, rank and numeric validation in an
    // isolated message owner. Its input keys are not added to history twice.
    auto workspace=limits_;workspace.active_columns=temporaryColumns(limits_);
    SubmapInformation message(workspace);message.append(originals,columns,j,r,c,target,roundoff);
    for(const auto& key:originals)if(next->measurements.count(key))throw std::invalid_argument("submap_measurement_reused");
    if(next->measurements.size()+originals.size()>limits_.measurements)throw std::runtime_error("submap_measurement_capacity");
    for(Coordinate coordinate:columns)if(next->known.insert(coordinate).second)next->order.push_back(coordinate);
    if(next->known.size()>limits_.coordinates || next->order.size()>limits_.active_columns)throw std::runtime_error("submap_coordinate_capacity");
    // Extend the original frontier with new variables, never with archived
    // variables. The old prior supplies no information on those new columns.
    const auto old_n=next->jacobian.cols(),n=Eigen::Index(next->order.size());
    next->jacobian.conservativeResize(Eigen::NoChange,n);next->constraints.conservativeResize(Eigen::NoChange,n);
    if(n>old_n){next->jacobian.rightCols(n-old_n).setZero();next->constraints.rightCols(n-old_n).setZero();}
    auto incoming=message.snapshot();
    for(size_t node_index=0;node_index<next->nodes.size();++node_index) {
      const auto& original=*next->nodes[node_index];bool affected=false;
      const auto local=index(incoming->order);
      for(Coordinate coordinate:original.removed)if(local.count(coordinate)){affected=true;break;}
      if(!affected)continue;
      std::vector<Coordinate> order=original.kept;
      order.insert(order.end(),original.removed.begin(),original.removed.end());
      Eigen::MatrixXd whitened=original.noise.transpose();
      for(Eigen::Index i=0;i<whitened.rows();++i) {
        const double norm=original.noise.col(i).stableNorm();
        if(!std::isfinite(norm) || norm<=0)throw std::runtime_error("submap_nonfinite_conditional");
        whitened.row(i)=(original.noise.col(i)/norm).transpose()/norm;
      }
      Eigen::MatrixXd conditional(whitened.rows(),order.size());
      conditional<<-whitened*original.regression,whitened;
      Eigen::MatrixXd exact(original.deterministic.cols(),order.size());
      exact<<-original.deterministic.transpose()*original.regression,original.deterministic.transpose();
      SubmapInformation combined(workspace);
      combined.append({"original-conditional"},order,conditional,-whitened*original.offset,
        exact,original.deterministic.transpose()*original.offset,original.roundoff);
      combined.append({"new-message"},incoming->order,incoming->jacobian,incoming->residual,incoming->constraints,incoming->target,incoming->roundoff);
      std::set<Coordinate> removed(original.removed.begin(),original.removed.end());
      std::vector<Coordinate> kept;
      for(Coordinate coordinate:combined.snapshot()->order)if(!removed.count(coordinate))kept.push_back(coordinate);
      combined.seal(kept);const auto updated=combined.snapshot();
      const auto& replacement=updated->nodes.back();
      // seal uses the original elimination order for removed coordinates.
      if(replacement->removed!=original.removed)throw std::logic_error("submap_late_removed_ownership");
      next->history_bytes-=original.bytes();next->history_bytes+=replacement->bytes();next->nodes[node_index]=replacement;
      incoming=updated;
    }
    const auto root=index(next->order);
    for(Coordinate coordinate:incoming->order)if(!root.count(coordinate))throw std::logic_error("submap_late_closure");
    if(!next->order.empty()) {
      SubmapInformation frontier(limits_);
      frontier.append({"original-frontier"},next->order,next->jacobian,next->residual,next->constraints,next->target,next->roundoff);
      frontier.append({"new-message"},incoming->order,incoming->jacobian,incoming->residual,incoming->constraints,incoming->target,incoming->roundoff);
      const auto updated=frontier.snapshot();next->order=updated->order;next->jacobian=updated->jacobian;next->residual=updated->residual;
      next->constraints=updated->constraints;next->target=updated->target;next->roundoff=updated->roundoff;
    } else {
      const auto old_rows=next->residual.size();next->residual.conservativeResize(old_rows+incoming->residual.size());
      next->residual.tail(incoming->residual.size())=incoming->residual;next->jacobian.resize(next->residual.size(),0);
      compact(next->jacobian,next->residual,next->roundoff);
    }
    next->measurements.insert(originals.begin(),originals.end());return commit(std::move(next));
  }
  Solution solve(const Snapshot& input) const {
    requireOwner(input);
    const auto split=splitConstrainedInformation(input->jacobian,input->residual,0,input->constraints,input->target,input->roundoff);
    return {input,split.reduced.backsub_offset,split.noise,split.unbounded};
  }
  // Evaluate all outputs jointly. Each conditional noise is counted once, so
  // cross-point covariance and relative-coordinate cancellation are preserved.
  Query query(const Solution& solution,const std::vector<Coordinate>& coordinates,const Eigen::MatrixXd& derivative) const {
    requireOwner(solution.input);index(coordinates);
    const auto rows=derivative.rows(),n=Eigen::Index(solution.input->order.size());
    if(rows<1 || rows>24 || derivative.cols()!=Eigen::Index(coordinates.size()) || !derivative.allFinite() ||
        solution.mean.size()!=n || solution.noise.rows()!=n || solution.unbounded.rows()!=n ||
        !solution.mean.allFinite() || !solution.noise.allFinite() || !solution.unbounded.allFinite())
      throw std::invalid_argument("submap_query_shape");
    std::map<Coordinate,Eigen::VectorXd> coefficients;
    for(size_t i=0;i<coordinates.size();++i) {
      if(!solution.input->known.count(coordinates[i]))throw std::invalid_argument("unknown_submap_query_coordinate");
      // Skip exactly zero support only. A tolerance (or squared norm, which
      // can underflow) would discard valid small coefficients.
      if((derivative.col(i).array()!=0.).any())coefficients.emplace(coordinates[i],derivative.col(i));
    }
    Query result;result.mean=Eigen::VectorXd::Zero(rows);result.covariance=Eigen::MatrixXd::Zero(rows,rows);
    result.unbounded_gram=Eigen::MatrixXd::Zero(rows,rows);
    bool observed=true;
    auto uncertainty=[&](const Eigen::MatrixXd& weights,const Eigen::MatrixXd& noise,const Eigen::MatrixXd& unbounded) {
      const Eigen::MatrixXd finite=weights*noise,free=weights*unbounded;
      result.covariance.noalias()+=finite*finite.transpose();result.unbounded_gram.noalias()+=free*free.transpose();
      if(free.norm()>1e-8*std::max(1.,weights.norm()*unbounded.norm()))observed=false;
    };
    for(const auto& pointer:solution.input->nodes) {
      const auto& node=*pointer;bool used=false;
      for(Coordinate coordinate:node.removed)if(coefficients.count(coordinate)){used=true;break;}
      if(!used)continue;
      Eigen::MatrixXd weights=Eigen::MatrixXd::Zero(rows,node.removed.size());
      for(size_t i=0;i<node.removed.size();++i) {
        auto found=coefficients.find(node.removed[i]);if(found==coefficients.end())continue;
        weights.col(i)=found->second;coefficients.erase(found);
      }
      result.mean.noalias()+=weights*node.offset;uncertainty(weights,node.noise,node.unbounded);
      const Eigen::MatrixXd next=weights*node.regression;
      for(size_t i=0;i<node.kept.size();++i) {
        if(!(next.col(i).array()!=0.).any())continue;
        auto found=coefficients.find(node.kept[i]);
        if(found==coefficients.end())coefficients.emplace(node.kept[i],next.col(i));
        else {
          found->second+=next.col(i);
          if(!(found->second.array()!=0.).any())coefficients.erase(found);
        }
      }
    }
    Eigen::MatrixXd weights=Eigen::MatrixXd::Zero(rows,n);
    for(size_t i=0;i<solution.input->order.size();++i) {
      auto found=coefficients.find(solution.input->order[i]);
      if(found!=coefficients.end()){weights.col(i)=found->second;coefficients.erase(found);}
    }
    if(!coefficients.empty())throw std::logic_error("submap_conditional_closure");
    result.mean.noalias()+=weights*solution.mean;uncertainty(weights,solution.noise,solution.unbounded);
    if(!result.mean.allFinite() || !result.covariance.allFinite() || !result.unbounded_gram.allFinite())
      throw std::runtime_error("submap_nonfinite_query");
    result.observed=observed;return result;
  }
 private:
  // The linearization owner holds the actual base/current snapshots. Its
  // information context needs only the lineage and limits, not an extra old
  // snapshot that would keep retired numeric payload alive outside its quota.
  void releaseContextSnapshot() {
    auto empty=std::make_shared<Data>();empty->owner=identity_;
    Snapshot immutable=empty;std::atomic_store(&data_,immutable);
  }
  static std::map<Coordinate,size_t> index(const std::vector<Coordinate>& order) {
    std::map<Coordinate,size_t> out;
    for(size_t i=0;i<order.size();++i)if(order[i]<0 || !out.emplace(order[i],i).second)throw std::invalid_argument("submap_coordinates");
    return out;
  }
  static bool compact(Eigen::MatrixXd& j,Eigen::VectorXd& r,double& roundoff,bool preserve_rank=false) {
    const auto source_rows=j.rows();
    if(!source_rows)return true;
    if(!preserve_rank && source_rows<=j.cols()+1)return false;
    if(!j.cols()){r=Eigen::VectorXd::Constant(1,r.stableNorm());j.resize(1,0);roundoff=0;return true;}
    if(source_rows>j.cols()+1) {
      Eigen::MatrixXd augmented(source_rows,j.cols()+1);augmented<<j,r;
      Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
      const Eigen::MatrixXd root=qr.matrixQR().topRows(augmented.cols()).triangularView<Eigen::Upper>();
      j=root.leftCols(j.cols());r=root.col(j.cols());
    }
    // Use the same dimension-relative SVD rule as the information split, at
    // the source row count. Shrinking a matrix must not promote QR residuals
    // below its original precision into new measurements.
    Eigen::BDCSVD<Eigen::MatrixXd> svd(j,Eigen::ComputeThinU|Eigen::ComputeThinV);
    if(svd.info()!=Eigen::Success)throw std::runtime_error("submap_compaction_svd_failed");
    const auto& singular=svd.singularValues();
    const double tolerance=std::max(roundoff,std::max(source_rows,j.cols())*std::numeric_limits<double>::epsilon()*singular[0]);
    const Eigen::Index rank=(singular.array()>tolerance).count();
    const Eigen::VectorXd projected=svd.matrixU().leftCols(rank).transpose()*r;
    const double constant=(r-svd.matrixU().leftCols(rank)*projected).stableNorm();
    Eigen::MatrixXd reduced=Eigen::MatrixXd::Zero(rank+1,j.cols());
    reduced.topRows(rank)=singular.head(rank).asDiagonal()*svd.matrixV().leftCols(rank).transpose();
    r.resize(rank+1);r.head(rank)=projected;r[rank]=constant;j=std::move(reduced);
    roundoff=rank?tolerance:0;return true;
  }
  static void compactConstraints(Data& data) {
    if(!data.constraints.rows())return;
    const auto support=affineSupport(data.constraints,data.target,data.order.size());
    data.constraints=support.normal.transpose();data.target=data.constraints*support.point;
  }
  void requireOwner(const Snapshot& input) const {
    if(!input || input->owner!=identity_)throw std::invalid_argument("submap_snapshot_owner");
  }
  Snapshot commit(std::shared_ptr<Data> next,bool compacted=false) {
    if(!next->jacobian.allFinite() || !next->residual.allFinite() || !next->constraints.allFinite() || !next->target.allFinite() ||
        !std::isfinite(next->roundoff) || next->roundoff<0)
      throw std::runtime_error("submap_nonfinite_information");
    if(next->numericBytes()>limits_.numeric_bytes)throw std::runtime_error("submap_numeric_capacity");
    ++next->revision;Snapshot immutable=next;
    compacted_=compacted?immutable:Snapshot{};
    std::atomic_store(&data_,immutable);return immutable;
  }
  Limits limits_;std::shared_ptr<const int> identity_=std::make_shared<const int>(0);
  mutable std::mutex writer_;Snapshot data_;
  // Writer-owned cache of a proven invariant, not additional retained data or
  // serialized information. Forked/restored snapshots recompute conservatively.
  std::weak_ptr<const Data> compacted_;
};
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "submap_linearization.hpp"
#include <functional>

namespace posim_slam {
// Source-backed Gaussian elimination. Original groups remain independently
// replaceable after their coordinates leave the frontier. A bucket owns its
// source factors and incoming child messages; unchanged buckets stay shared.
// Replacements rebuild affected dependencies from source rows, never by
// subtracting information matrices or adding the old marginal a second time.
class SubmapFactorization {
 public:
  using Information=SubmapInformation;
  using Coordinate=Information::Coordinate;
  using Rows=SubmapLinearization::Rows;
  struct Factor {std::string id;Rows rows;};
  struct Layout {
    std::vector<Coordinate> frontier;
    std::vector<std::vector<Coordinate>> eliminated;
  };
  struct Bucket {
    std::vector<std::shared_ptr<const Rows>> sources;
    std::vector<std::shared_ptr<const Bucket>> children;
    std::vector<Coordinate> context;
    Rows message;
    double roundoff=0;
    std::shared_ptr<const Information::Node> conditional;
  };
  struct Data {
    uint64_t revision=0;
    Layout layout;
    std::map<std::string,std::shared_ptr<const Rows>> factors;
    // Original identity survives explicit unassignment. The raw measurement
    // payload remains the responsibility of the caller's input archive.
    std::set<std::string> originals;
    std::map<std::string,std::string> assignments;
    std::vector<std::shared_ptr<const Bucket>> buckets;
    Information::Snapshot information,base;
    std::vector<std::shared_ptr<const Rows>> active;
    std::vector<size_t> rebuilt;
    size_t numericBytes() const {
      size_t bytes=information?information->numericBytes():0;
      if(base && base!=information)bytes+=base->numericBytes()-base->history_bytes;
      for(const auto& [id,rows]:factors)bytes+=rows->bytes();
      for(const auto& bucket:buckets)bytes+=bucket->message.bytes();
      return bytes;
    }
   private:
    friend class SubmapFactorization;
    std::shared_ptr<const int> owner;
  };
  using Snapshot=std::shared_ptr<const Data>;
  explicit SubmapFactorization(Layout layout,Information::Limits limits={})
      :core_(std::make_shared<Information>(limits)),limits_(limits) {
    auto initial=std::make_shared<Data>();initial->owner=identity_;
    initial->layout=std::move(layout);validateLayout(initial->layout,nullptr);
    rebuild(*initial,nullptr,{});data_=initial;
  }
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  // Stage a larger native transaction without publishing to the current owner.
  // Immutable source rows, conditionals and the query lineage remain shared.
  std::unique_ptr<SubmapFactorization> fork(const Snapshot& state) const {
    if(!state || state->owner!=identity_)throw std::invalid_argument("submap_factor_owner");
    return std::unique_ptr<SubmapFactorization>(new SubmapFactorization(*this,state));
  }
  // All originals of removed groups must be reassigned or explicitly marked
  // unassigned. The layout may append elimination blocks and new coordinates;
  // existing blocks and every previously known coordinate remain intact.
  Snapshot replace(const std::vector<std::string>& removed,
      const std::vector<Factor>& incoming,const std::vector<std::string>& unassigned={},
      const Layout* layout=nullptr,const std::function<bool()>& cancelled={}) {
    std::lock_guard<std::mutex> writer(writer_);check(cancelled);
    const auto previous=snapshot();auto next=std::make_shared<Data>(*previous);
    if(layout)next->layout=*layout;
    const auto rank=validateLayout(next->layout,&previous->layout);
    const auto remove=keys(removed),unused=keys(unassigned);std::set<std::string> outgoing;
    for(const auto& id:remove) {
      const auto found=next->factors.find(id);
      if(found==next->factors.end())throw std::invalid_argument("submap_factor_unknown");
      for(const auto& key:found->second->originals) {
        outgoing.insert(key);next->assignments.erase(key);
      }
      next->factors.erase(found);
    }
    if(!std::includes(outgoing.begin(),outgoing.end(),unused.begin(),unused.end()))
      throw std::invalid_argument("submap_factor_unassignment");
    std::set<std::string> assigned;
    size_t source_bytes=0;
    for(const auto& [id,rows]:next->factors)source_bytes+=rows->bytes();
    for(const auto& factor:incoming) {
      check(cancelled);keys({factor.id});
      if(next->factors.count(factor.id))throw std::invalid_argument("submap_factor_reused");
      const auto originals=keys(factor.rows.originals);
      if(originals.empty())throw std::invalid_argument("submap_factor_empty");
      Information::validateRows(factor.rows.columns,factor.rows.jacobian,factor.rows.residual,
                               factor.rows.constraints,factor.rows.target);
      std::set<Coordinate> columns;
      for(auto column:factor.rows.columns)if(!rank.count(column) || !columns.insert(column).second)
        throw std::invalid_argument("submap_factor_coordinates");
      for(const auto& key:originals) {
        if(unused.count(key) || !next->assignments.emplace(key,factor.id).second)
          throw std::invalid_argument("submap_measurement_reused");
        assigned.insert(key);next->originals.insert(key);
      }
      if(next->originals.size()>limits_.measurements)
        throw std::runtime_error("submap_measurement_capacity");
      if(source_bytes>limits_.numeric_bytes || factor.rows.bytes()>limits_.numeric_bytes-source_bytes)
        throw std::runtime_error("submap_factor_numeric_capacity");
      source_bytes+=factor.rows.bytes();
      next->factors.emplace(factor.id,std::make_shared<const Rows>(factor.rows));
    }
    assigned.insert(unused.begin(),unused.end());
    if(!std::includes(assigned.begin(),assigned.end(),outgoing.begin(),outgoing.end()))
      throw std::invalid_argument("submap_factor_original_missing");
    rebuild(*next,previous.get(),cancelled);check(cancelled);
    ++next->revision;Snapshot immutable=next;std::atomic_store(&data_,immutable);return immutable;
  }
  Information::Solution solve(const Snapshot& input) const {
    if(!input || input->owner!=identity_)throw std::invalid_argument("submap_factor_owner");
    return core_->solve(input->information);
  }
  Information::Query query(const Information::Solution& solution,
      const std::vector<Coordinate>& coordinates,const Eigen::MatrixXd& derivative) const {
    return core_->query(solution,coordinates,derivative);
  }
 private:
  SubmapFactorization(const SubmapFactorization& source,Snapshot state)
      :core_(source.core_),limits_(source.limits_),identity_(source.identity_),data_(std::move(state)) {}
  static void check(const std::function<bool()>& cancelled) {
    if(cancelled && cancelled())throw std::runtime_error("submap_factor_cancelled");
  }
  static std::set<std::string> keys(const std::vector<std::string>& values) {
    std::set<std::string> out;
    for(const auto& key:values)if(key.empty() || key.size()>256 || !out.insert(key).second)
      throw std::invalid_argument("submap_factor_identity");
    return out;
  }
  std::map<Coordinate,size_t> validateLayout(const Layout& layout,const Layout* previous) const {
    if(layout.eliminated.size()>limits_.nodes)throw std::runtime_error("submap_node_capacity");
    if(layout.frontier.empty() || layout.frontier.size()>limits_.active_columns)
      throw std::runtime_error("submap_factor_layout_capacity");
    if(previous && (layout.eliminated.size()<previous->eliminated.size() ||
        !std::equal(previous->eliminated.begin(),previous->eliminated.end(),layout.eliminated.begin())))
      throw std::invalid_argument("submap_factor_elimination_changed");
    std::map<Coordinate,size_t> rank;
    auto insert=[&](const std::vector<Coordinate>& columns,size_t bucket) {
      if(columns.empty() || columns.size()>Information::temporaryColumns(limits_))
        throw std::runtime_error("submap_factor_layout_capacity");
      for(auto column:columns)if(column<0 || !rank.emplace(column,bucket).second)
        throw std::invalid_argument("submap_factor_coordinates");
    };
    for(size_t i=0;i<layout.eliminated.size();++i)insert(layout.eliminated[i],i);
    insert(layout.frontier,layout.eliminated.size());
    if(rank.size()>limits_.coordinates)throw std::runtime_error("submap_coordinate_capacity");
    if(previous)for(auto column:previous->frontier)if(!rank.count(column))
      throw std::invalid_argument("submap_factor_coordinate_missing");
    return rank;
  }
  // Compact bounded batches in the existing rank-aware information engine.
  // Internal message labels are local to this workspace. Only source keys are
  // published in Data::information->measurements.
  std::shared_ptr<const Bucket> factorize(std::shared_ptr<Bucket> bucket,bool root,
      const std::function<bool()>& cancelled) const {
    std::vector<const Rows*> rows;
    for(const auto& source:bucket->sources)rows.push_back(source.get());
    for(const auto& child:bucket->children)rows.push_back(&child->message);
    std::vector<Coordinate> order=bucket->context;
    std::set<Coordinate> columns(order.begin(),order.end());
    for(const auto* group:rows)for(auto column:group->columns)
      if(columns.insert(column).second)order.push_back(column);
    if(order.size()>Information::temporaryColumns(limits_))
      throw std::runtime_error("submap_factor_separator_capacity");
    auto workspace=limits_;workspace.active_columns=Information::temporaryColumns(limits_);
    Information combined(workspace);
    double inherited=0;for(const auto& child:bucket->children)inherited=std::hypot(inherited,child->roundoff);
    combined.append({"context"},order,Eigen::MatrixXd::Zero(0,order.size()),Eigen::VectorXd(),Eigen::MatrixXd(),Eigen::VectorXd(),inherited);
    std::map<Coordinate,size_t> offsets;
    for(size_t i=0;i<order.size();++i)offsets.emplace(order[i],i);
    size_t row_count=0,exact_count=0;
    for(const auto* group:rows){row_count+=group->jacobian.rows();exact_count+=group->constraints.rows();}
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(std::min(size_t(4096),row_count),order.size());
    Eigen::MatrixXd c=Eigen::MatrixXd::Zero(std::min(size_t(1024),exact_count),order.size());
    Eigen::VectorXd r(j.rows()),target(c.rows());Eigen::Index used=0,exact=0;size_t batch=0;
    auto flush=[&] {
      if(!used && !exact)return;check(cancelled);
      combined.append({"batch:"+std::to_string(batch++)},order,j.topRows(used),r.head(used),c.topRows(exact),target.head(exact));
      used=exact=0;j.setZero();c.setZero();
    };
    for(const auto* group:rows) {
      const auto n=group->jacobian.rows(),m=group->constraints.rows();
      Eigen::Index row=0,constraint=0;
      while(row<n || constraint<m) {
        const auto take=std::min(n-row,j.rows()-used),take_exact=std::min(m-constraint,c.rows()-exact);
        if(!take && !take_exact){flush();continue;}
        for(size_t i=0;i<group->columns.size();++i) {
          const auto column=offsets.at(group->columns[i]);
          j.block(used,column,take,1)=group->jacobian.block(row,i,take,1);
          if(take_exact)c.block(exact,column,take_exact,1)=group->constraints.block(constraint,i,take_exact,1);
        }
        r.segment(used,take)=group->residual.segment(row,take);
        target.segment(exact,take_exact)=group->target.segment(constraint,take_exact);
        used+=take;exact+=take_exact;row+=take;constraint+=take_exact;
      }
    }
    flush();
    if(!root) {
      std::vector<Coordinate> kept(order.begin()+bucket->context.size(),order.end());
      combined.seal(kept);bucket->conditional=combined.snapshot()->nodes.back();
      if(bucket->conditional->removed!=bucket->context)throw std::logic_error("submap_factor_elimination_order");
    }
    const auto result=combined.snapshot();auto& out=bucket->message;
    bucket->roundoff=result->roundoff;
    out.columns=result->order;out.jacobian=result->jacobian;out.residual=result->residual;
    out.constraints=result->constraints;out.target=result->target;
    return bucket;
  }
  void rebuild(Data& next,const Data* previous,const std::function<bool()>& cancelled) const {
    const auto rank=validateLayout(next.layout,nullptr);const size_t root=next.layout.eliminated.size();
    auto route=[&](const std::vector<Coordinate>& columns) {
      size_t out=root;for(auto column:columns)out=std::min(out,rank.at(column));return out;
    };
    std::vector<std::shared_ptr<Bucket>> pending(root+1);
    for(size_t i=0;i<=root;++i) {
      pending[i]=std::make_shared<Bucket>();
      pending[i]->context=i==root?next.layout.frontier:next.layout.eliminated[i];
    }
    size_t bytes=0;
    for(const auto& [id,rows]:next.factors) {
      pending[route(rows->columns)]->sources.push_back(rows);bytes+=rows->bytes();
    }
    next.buckets.clear();next.rebuilt.clear();
    for(size_t i=0;i<=root;++i) {
      check(cancelled);const auto& incoming=pending[i];std::shared_ptr<const Bucket> bucket;
      // An old root is not an eliminated node when the frontier moves.
      const bool old_kind=previous && ((i==root && i==previous->layout.eliminated.size()) ||
          (i<root && i<previous->layout.eliminated.size()));
      if(old_kind) {
        const auto& old=previous->buckets.at(i);
        if(old->context==incoming->context && old->sources==incoming->sources && old->children==incoming->children)bucket=old;
      }
      if(!bucket){bucket=factorize(incoming,i==root,cancelled);next.rebuilt.push_back(i);}
      bytes+=bucket->message.bytes()+(bucket->conditional?bucket->conditional->bytes():0);
      if(bytes>limits_.numeric_bytes)throw std::runtime_error("submap_factor_numeric_capacity");
      next.buckets.push_back(bucket);
      if(i!=root) {
        const size_t parent=route(bucket->message.columns);
        if(parent<=i)throw std::logic_error("submap_factor_dependency_order");
        pending[parent]->children.push_back(bucket);
      }
    }
    // Reuse the information engine's lineage and query/back-substitution.
    // Its context stays empty; all numeric payload is owned by this snapshot.
    auto information=std::make_shared<Information::Data>(*core_->snapshot());
    const auto& frontier=next.buckets.back()->message;
    information->revision=next.revision+1;information->order=frontier.columns;
    information->jacobian=frontier.jacobian;information->residual=frontier.residual;
    information->constraints=frontier.constraints;information->target=frontier.target;
    information->roundoff=next.buckets.back()->roundoff;
    for(const auto& [column,index]:rank)information->known.insert(column);
    for(const auto& [key,id]:next.assignments)information->measurements.insert(key);
    for(size_t i=0;i<root;++i) {
      information->nodes.push_back(next.buckets[i]->conditional);
      information->history_bytes+=next.buckets[i]->conditional->bytes();
    }
    next.information=information;
    next.active=next.buckets.back()->sources;
    if(next.active.empty())next.base=next.information;
    else if(previous && previous->buckets.back()->context==next.buckets.back()->context &&
        previous->buckets.back()->children==next.buckets.back()->children)next.base=previous->base;
    else {
      // The nonlinear objective combines this historical marginal with the
      // original active residuals. Do not count those active residuals twice.
      auto history=std::make_shared<Bucket>();history->context=next.layout.frontier;
      history->children=next.buckets.back()->children;
      const auto reduced=factorize(history,true,cancelled);
      auto base=std::make_shared<Information::Data>(*information);
      base->order=reduced->message.columns;base->jacobian=reduced->message.jacobian;
      base->residual=reduced->message.residual;base->constraints=reduced->message.constraints;
      base->target=reduced->message.target;base->roundoff=reduced->roundoff;
      for(const auto& rows:next.active)for(const auto& key:rows->originals)base->measurements.erase(key);
      next.base=std::move(base);
    }
    if(next.numericBytes()>limits_.numeric_bytes)throw std::runtime_error("submap_factor_numeric_capacity");
  }
  std::shared_ptr<Information> core_;Information::Limits limits_;
  std::shared_ptr<const int> identity_=std::make_shared<const int>(0);
  std::mutex writer_;Snapshot data_;
};
} // namespace posim_slam

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "submap_row_store.hpp"
#include "submap_ordering.hpp"
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
  using Page=SubmapRowPage::Reference;
  struct Factor {std::string id;Rows rows;};
  struct Layout {
    std::vector<Coordinate> frontier;
    std::vector<std::vector<Coordinate>> eliminated;
    // Optional stable variable partition, independent of numeric supernodes.
    // New variables append to this list as the layout grows.
    std::vector<std::vector<Coordinate>> variables;
  };
  enum class Ordering {fixed,minimum_degree};
  enum class Ownership {resident,paged};
  struct Bucket {
    std::vector<Page> sources;
    std::vector<std::shared_ptr<const Bucket>> children;
    std::vector<Coordinate> context;
    Page message;
    double roundoff=0;
    std::shared_ptr<const Information::Node> conditional;
  };
  struct Data {
    uint64_t revision=0;
    Layout layout;
    std::map<std::string,Page> factors;
    std::vector<std::shared_ptr<const Bucket>> buckets;
    Information::Snapshot information,base;
    std::vector<Page> active;
    std::vector<size_t> rebuilt;
    // These queries cover the complete inventory in either storage mode.
    // Explicitly unassigned identities remain in the caller's input archive
    // and this index. Paged snapshots never copy the resident global sets.
    size_t originalCount() const {return inventory?inventory->count:resident_originals.size();}
    size_t assignedCount() const {return inventory?inventory->assigned:resident_assignments.size();}
    bool hasOriginal(const std::string& key) const {
      return inventory?metadata->lookup(inventory,key).has_value():resident_originals.count(key)>0;
    }
    std::optional<std::string> assignment(const std::string& key) const {
      if(inventory){auto owner=metadata->lookup(inventory,key);return owner && !owner->empty()?owner:std::nullopt;}
      auto owner=resident_assignments.find(key);return owner==resident_assignments.end()?std::nullopt:std::optional<std::string>(owner->second);
    }
    std::string ownerOf(const std::string& key) const {
      auto owner=assignment(key);if(!owner)throw std::out_of_range("submap_original_unassigned");return *owner;
    }
    void eachUnassigned(const std::function<void(const std::string&)>& visitor) const {
      if(inventory)metadata->each(inventory,[&](const auto& row){if(row.owner.empty())visitor(row.key);});
      else for(const auto& key:resident_originals)if(!resident_assignments.count(key))visitor(key);
    }
    void eachOriginal(const std::function<void(const std::string&)>& visitor) const {
      if(inventory)metadata->each(inventory,[&](const auto& row){visitor(row.key);});
      else for(const auto& key:resident_originals)visitor(key);
    }
    bool pagedOwnership() const {return bool(inventory);}
    size_t numericBytes() const {
      size_t bytes=information?information->numericBytes():0;
      if(base && base!=information)bytes+=base->numericBytes()-base->history_bytes;
      for(const auto& [id,rows]:factors)bytes+=rows->bytes();
      for(const auto& bucket:buckets)bytes+=bucket->message->bytes();
      return bytes;
    }
   private:
    friend class SubmapFactorization;
    std::set<std::string> resident_originals;
    std::map<std::string,std::string> resident_assignments;
    std::shared_ptr<SubmapOwnershipStore> metadata;
    SubmapOwnershipStore::Snapshot inventory;
    std::shared_ptr<const int> owner;
  };
  using Snapshot=std::shared_ptr<const Data>;
  explicit SubmapFactorization(Layout layout,Information::Limits limits={},
      const std::vector<std::string>& retained_originals={},std::shared_ptr<SubmapRowStore> row_store={},
      Ownership ownership=Ownership::resident)
      :core_(std::make_shared<Information>(limits)),limits_(limits),row_store_(std::move(row_store)),ownership_(ownership) {
    auto initial=std::make_shared<Data>();initial->owner=identity_;
    if(ownership_==Ownership::paged) {
      if(!row_store_ || row_store_->metadata()->capacity()!=limits.measurements)
        throw std::invalid_argument("submap_metadata_capacity_configuration");
      initial->metadata=row_store_->metadata();initial->inventory=initial->metadata->empty();
      if(!retained_originals.empty())initial->inventory=initial->metadata->stage(initial->inventory,1,
          [&](auto& edit){for(const auto& key:retained_originals)edit.seed(key);});
    } else {
      initial->resident_originals=keys(retained_originals);
      if(initial->resident_originals.size()>limits_.measurements)throw std::runtime_error("submap_measurement_capacity");
    }
    initial->layout=std::move(layout);validateLayout(initial->layout,nullptr);
    rebuild(*initial,nullptr,{});data_=initial;
  }
  Snapshot snapshot() const {return std::atomic_load(&data_);}
  // Stage a larger native transaction without publishing to the current owner.
  // Immutable source rows, conditionals and the query lineage remain shared.
  std::unique_ptr<SubmapFactorization> fork(const Snapshot& state,size_t numeric_bytes=0) const {
    if(!state || state->owner!=identity_)throw std::invalid_argument("submap_factor_owner");
    auto result=std::unique_ptr<SubmapFactorization>(new SubmapFactorization(*this,state));
    // A larger transaction reserves its own current metadata separately. The
    // supplied row budget governs the next replacement, before publication.
    if(numeric_bytes)result->limits_.numeric_bytes=numeric_bytes;
    return result;
  }
  // All originals of removed groups must be reassigned or explicitly marked
  // unassigned. The layout may append elimination blocks and new coordinates;
  // existing blocks and every previously known coordinate remain intact.
  Snapshot replace(const std::vector<std::string>& removed,
      const std::vector<Factor>& incoming,const std::vector<std::string>& unassigned={},
      const Layout* layout=nullptr,const std::function<bool()>& cancelled={},Ordering ordering=Ordering::fixed) {
    std::lock_guard<std::mutex> writer(writer_);check(cancelled);
    // Storage is part of this factor transaction. Its cancellation checkpoints
    // propagate the owning operation's exception and rollback contract.
    const std::function<bool()> source_cancelled=[&]{check(cancelled);return false;};
    const auto previous=snapshot();auto next=std::make_shared<Data>(*previous);
    if(layout)next->layout=*layout;
    const auto rank=validateLayout(next->layout,&previous->layout);
    const auto remove=keys(removed);std::set<std::string> unused,outgoing,assigned;
    if(ownership_==Ownership::resident)unused=keys(unassigned);
    for(const auto& id:remove) {
      const auto found=next->factors.find(id);
      if(found==next->factors.end())throw std::invalid_argument("submap_factor_unknown");
      if(ownership_==Ownership::resident)found->second->eachOriginal([&](const auto& key){
        outgoing.insert(key);next->resident_assignments.erase(key);
      });
      next->factors.erase(found);
    }
    if(ownership_==Ownership::resident && !std::includes(outgoing.begin(),outgoing.end(),unused.begin(),unused.end()))
      throw std::invalid_argument("submap_factor_unassignment");
    size_t source_bytes=0;
    for(const auto& [id,rows]:next->factors)source_bytes+=rows->bytes();
    const auto addIncoming=[&](SubmapOwnershipStore::Editor* edit) {for(const auto& factor:incoming) {
      check(cancelled);keys({factor.id});
      if(next->factors.count(factor.id))throw std::invalid_argument("submap_factor_reused");
      if(factor.rows.originals.empty())throw std::invalid_argument("submap_factor_empty");
      Information::validateRows(factor.rows.columns,factor.rows.jacobian,factor.rows.residual,
                               factor.rows.constraints,factor.rows.target);
      std::set<Coordinate> columns;
      for(auto column:factor.rows.columns)if(!rank.count(column) || !columns.insert(column).second)
        throw std::invalid_argument("submap_factor_coordinates");
      if(ownership_==Ownership::resident) {
        for(const auto& key:keys(factor.rows.originals)) {
          if(unused.count(key) || !next->resident_assignments.emplace(key,factor.id).second)
            throw std::invalid_argument("submap_measurement_reused");
          assigned.insert(key);next->resident_originals.insert(key);
        }
        if(next->resident_originals.size()>limits_.measurements)
          throw std::runtime_error("submap_measurement_capacity");
      }
      if(source_bytes>limits_.numeric_bytes || factor.rows.retainedBytes()>limits_.numeric_bytes-source_bytes)
        throw std::runtime_error("submap_factor_numeric_capacity");
      source_bytes+=factor.rows.retainedBytes();
      next->factors.emplace(factor.id,retain(factor.rows,source_cancelled));
      if(edit){edit->factor(factor.id);for(const auto& key:factor.rows.originals)edit->assign(key);}
    }};
    if(ownership_==Ownership::paged) {
      next->inventory=next->metadata->stage(previous->inventory,previous->inventory->revision+1,[&](auto& edit){
        for(const auto& id:remove)edit.remove(id);
        addIncoming(&edit);
        for(const auto& key:unassigned)edit.unassign(key);
      },source_cancelled);
    } else {
      addIncoming(nullptr);
      assigned.insert(unused.begin(),unused.end());
      if(!std::includes(assigned.begin(),assigned.end(),outgoing.begin(),outgoing.end()))
        throw std::invalid_argument("submap_factor_original_missing");
    }
    if(ordering==Ordering::minimum_degree) {
      expandChanged(*next,*previous,cancelled);
      reorderChanged(*next,*previous,cancelled);
      coalesceChanged(*next,*previous,cancelled);
    }
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
      :core_(source.core_),limits_(source.limits_),row_store_(source.row_store_),ownership_(source.ownership_),
       identity_(source.identity_),data_(std::move(state)) {}
  Page retain(const Rows& rows,const std::function<bool()>& cancelled={}) const {
    return row_store_?row_store_->put(rows,cancelled):SubmapRowPage::resident(rows);
  }
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
    if(previous && !previous->variables.empty() &&
        (layout.variables.size()<previous->variables.size() ||
         !std::equal(previous->variables.begin(),previous->variables.end(),layout.variables.begin())))
      throw std::invalid_argument("submap_factor_variables_changed");
    if(!layout.variables.empty()) {
      std::set<Coordinate> assigned;
      for(const auto& variable:layout.variables) {
        if(variable.empty() || !rank.count(variable.front()))
          throw std::invalid_argument("submap_factor_variables");
        const auto bucket=rank.at(variable.front());
        for(auto column:variable)if(!rank.count(column) || rank.at(column)!=bucket || !assigned.insert(column).second)
          throw std::invalid_argument("submap_factor_variables");
      }
      if(assigned.size()!=rank.size())throw std::invalid_argument("submap_factor_variables");
    }
    return rank;
  }
  struct Symbol {
    std::vector<Page> sources;std::vector<size_t> children;std::vector<Coordinate> columns;
    std::shared_ptr<const Bucket> reused;
  };
  // Source metadata predicts both the outgoing support and exact reuse. The
  // caller reserves any additional symbolic workspace before this allocation.
  std::vector<Symbol> symbolize(const Data& next,const Data& previous,
      const std::function<bool()>& cancelled,size_t reserved=0) const {
    const size_t root=next.layout.eliminated.size();
    const auto rank=validateLayout(next.layout,nullptr);
    size_t workspace=reserved+(root+1)*sizeof(Symbol);
    if(next.factors.size()>limits_.numeric_bytes/sizeof(Page) ||
        workspace>limits_.numeric_bytes-next.factors.size()*sizeof(Page))return {};
    workspace+=next.factors.size()*sizeof(Page);
    const auto route=[&](const std::vector<Coordinate>& columns) {
      size_t out=root;for(auto c:columns)out=std::min(out,rank.at(c));return out;
    };
    std::vector<Symbol> symbols(root+1);
    for(const auto& [id,rows]:next.factors)symbols[route(rows->columns)].sources.push_back(rows);
    std::map<std::vector<Coordinate>,std::shared_ptr<const Bucket>> old;
    for(size_t i=0;i<previous.layout.eliminated.size();++i)old.emplace(previous.buckets[i]->context,previous.buckets[i]);
    for(size_t i=0;i<=root;++i) {
      check(cancelled);auto& symbol=symbols[i];const auto& context=i==root?next.layout.frontier:next.layout.eliminated[i];
      auto found=old.find(context);const auto before=i==root?previous.buckets.back():(found==old.end()?nullptr:found->second);
      bool same=before && before->context==context && before->sources==symbol.sources && before->children.size()==symbol.children.size();
      for(size_t j=0;same && j<symbol.children.size();++j)same=symbols[symbol.children[j]].reused==before->children[j];
      if(same){symbol.reused=before;symbol.columns=before->message->columns;}
      else {
        std::set<Coordinate> columns(context.begin(),context.end());
        for(const auto& page:symbol.sources)columns.insert(page->columns.begin(),page->columns.end());
        for(auto child:symbol.children)columns.insert(symbols[child].columns.begin(),symbols[child].columns.end());
        if(i!=root)for(auto c:context)columns.erase(c);
        symbol.columns.assign(columns.begin(),columns.end());
      }
      const size_t bytes=symbol.columns.size()*sizeof(Coordinate)+symbol.children.size()*sizeof(size_t);
      if(bytes>limits_.numeric_bytes-workspace)return {};workspace+=bytes;
      if(i!=root) {
        const auto parent=route(symbol.columns);
        if(parent<=i)throw std::logic_error("submap_factor_dependency_order");
        symbols[parent].children.push_back(i);
      }
    }
    return symbols;
  }
  // Numeric supernodes are temporary groups for one symbolic graph. New
  // observations can give their variables different separators. Expand only
  // affected groups before ordering; unchanged numeric branches remain shared.
  void expandChanged(Data& next,const Data& previous,const std::function<bool()>& cancelled) const {
    if(next.layout.variables.empty())return;
    const auto symbols=symbolize(next,previous,cancelled);if(symbols.empty())return;
    std::map<Coordinate,size_t> variable;
    for(size_t i=0;i<next.layout.variables.size();++i)
      for(auto c:next.layout.variables[i])variable.emplace(c,i);
    std::vector<std::vector<Coordinate>> expanded;
    for(size_t i=0;i<next.layout.eliminated.size();++i) {
      check(cancelled);const auto& group=next.layout.eliminated[i];
      if(symbols[i].reused)expanded.push_back(group);
      else {
        std::set<size_t> used;
        for(auto c:group)if(used.insert(variable.at(c)).second)
          expanded.push_back(next.layout.variables[variable.at(c)]);
      }
      // Expansion is an ordering optimization, not a new capacity requirement.
      if(expanded.size()>limits_.nodes)return;
    }
    next.layout.eliminated=std::move(expanded);
  }
  // Plan the affected dependency tree using column metadata only. Unchanged
  // descendants stay in their original relative order and retain their exact
  // numeric pages. Reorder only the remaining blocks, from the same source rows.
  void reorderChanged(Data& next,const Data& previous,const std::function<bool()>& cancelled) const {
    const size_t root=next.layout.eliminated.size();if(root<2)return;
    const size_t count=root+next.layout.frontier.size(),words=(count+63)/64;
    if(count>limits_.numeric_bytes/(2*sizeof(uint64_t))/words)return;
    const auto symbols=symbolize(next,previous,cancelled,2*count*words*sizeof(uint64_t)+count*4*sizeof(size_t));
    if(symbols.empty())return;
    std::vector<size_t> fixed,changed;
    for(size_t i=0;i<root;++i)(symbols[i].reused?fixed:changed).push_back(i);
    if(changed.size()<2)return;
    std::vector<size_t> weights;std::map<Coordinate,size_t> vertex;
    for(size_t i=0;i<root;++i){weights.push_back(next.layout.eliminated[i].size());for(auto c:next.layout.eliminated[i])vertex.emplace(c,i);}
    for(auto c:next.layout.frontier){vertex.emplace(c,weights.size());weights.push_back(1);}
    SubmapOrderingGraph graph(weights);
    for(const auto& [id,page]:next.factors) {
      check(cancelled);std::set<size_t> nodes;for(auto c:page->columns)nodes.insert(vertex.at(c));
      for(auto a=nodes.begin();a!=nodes.end();++a)for(auto b=std::next(a);b!=nodes.end();++b)graph.connect(*a,*b);
    }
    const auto cost=[](size_t width){const uint64_t n=width;return n*n*n;};
    uint64_t original_cost=0,candidate_cost=0;size_t original_width=0,candidate_width=0;
    {
      auto original=graph;
      for(size_t i=0;i<root;++i) {
        check(cancelled);const auto width=original.eliminate(i);
        if(!symbols[i].reused){original_cost+=cost(width);original_width=std::max(original_width,width);}
      }
    }
    for(auto node:fixed){check(cancelled);graph.eliminate(node);}
    std::vector<size_t> chosen=fixed;
    while(!changed.empty()) {
      check(cancelled);auto best=std::min_element(changed.begin(),changed.end(),[&](size_t a,size_t b) {
        return graph.degree(a)==graph.degree(b)?a<b:graph.degree(a)<graph.degree(b);
      });
      const auto node=*best;const auto width=graph.eliminate(node);candidate_cost+=cost(width);candidate_width=std::max(candidate_width,width);
      chosen.push_back(node);changed.erase(best);
    }
    if(candidate_cost>=original_cost || candidate_width>original_width)return;
    const auto blocks=next.layout.eliminated;
    for(size_t i=0;i<chosen.size();++i)next.layout.eliminated[i]=blocks[chosen[i]];
  }
  // Strict supernodes share an identical remaining separator. Eliminating
  // them together avoids repeatedly compacting that same dense message. The
  // union has exactly the first block's symbolic width, so it introduces no
  // fill or larger temporary matrix. Unchanged numeric branches stay shared.
  void coalesceChanged(Data& next,const Data& previous,const std::function<bool()>& cancelled) const {
    const size_t root=next.layout.eliminated.size();if(root<2)return;
    const auto symbols=symbolize(next,previous,cancelled);if(symbols.empty())return;
    std::vector<std::vector<Coordinate>> groups;
    for(size_t i=0;i<root;++i) {
      check(cancelled);auto group=next.layout.eliminated[i];
      while(i+1<root && !symbols[i].reused && !symbols[i+1].reused) {
        const auto& child=symbols[i].columns;const auto& parent=symbols[i+1].columns;
        const auto& context=next.layout.eliminated[i+1];
        if(child.size()!=parent.size()+context.size() ||
            !std::includes(child.begin(),child.end(),parent.begin(),parent.end()) ||
            !std::all_of(context.begin(),context.end(),[&](auto c){return std::binary_search(child.begin(),child.end(),c);}))break;
        group.insert(group.end(),context.begin(),context.end());++i;
      }
      groups.push_back(std::move(group));
    }
    next.layout.eliminated=std::move(groups);
  }
  // Compact bounded batches in the existing rank-aware information engine.
  // Internal message labels are local to this workspace. Only source keys are
  // published in Data::information->measurements.
  std::shared_ptr<const Bucket> factorize(std::shared_ptr<Bucket> bucket,bool root,
      const std::function<bool()>& cancelled) const {
    std::vector<Page> rows=bucket->sources;
    for(const auto& child:bucket->children)rows.push_back(child->message);
    std::vector<Coordinate> order=bucket->context;
    std::set<Coordinate> columns(order.begin(),order.end());
    for(const auto& group:rows)for(auto column:group->columns)
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
    for(const auto& group:rows){row_count+=group->rows();exact_count+=group->exactRows();}
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(std::min(size_t(4096),row_count),order.size());
    Eigen::MatrixXd c=Eigen::MatrixXd::Zero(std::min(size_t(1024),exact_count),order.size());
    Eigen::VectorXd r(j.rows()),target(c.rows());Eigen::Index used=0,exact=0;size_t batch=0;
    auto flush=[&] {
      if(!used && !exact)return;check(cancelled);
      combined.append({"batch:"+std::to_string(batch++)},order,j.topRows(used),r.head(used),c.topRows(exact),target.head(exact));
      used=exact=0;j.setZero();c.setZero();
    };
    for(const auto& page:rows) {
      check(cancelled);const auto group=page->pinNumeric();
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
    const auto result=combined.snapshot();Rows out;
    bucket->roundoff=result->roundoff;
    out.columns=result->order;out.jacobian=result->jacobian;out.residual=result->residual;
    out.constraints=result->constraints;out.target=result->target;
    bucket->message=retain(out,cancelled);return bucket;
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
    std::map<std::vector<Coordinate>,std::shared_ptr<const Bucket>> old_buckets;
    if(previous)for(size_t i=0;i<previous->layout.eliminated.size();++i)
      old_buckets.emplace(previous->buckets[i]->context,previous->buckets[i]);
    next.buckets.clear();next.rebuilt.clear();
    for(size_t i=0;i<=root;++i) {
      check(cancelled);const auto& incoming=pending[i];std::shared_ptr<const Bucket> bucket;
      // An old root is not an eliminated node when the frontier moves.
      const auto found=old_buckets.find(incoming->context);
      const auto old=i==root?(previous?previous->buckets.back():nullptr):(found==old_buckets.end()?nullptr:found->second);
      if(old) {
        if(old->context==incoming->context && old->sources==incoming->sources && old->children==incoming->children)bucket=old;
      }
      if(!bucket){bucket=factorize(incoming,i==root,cancelled);next.rebuilt.push_back(i);}
      bytes+=bucket->message->bytes()+(bucket->conditional?bucket->conditional->bytes():0);
      if(bytes>limits_.numeric_bytes)throw std::runtime_error("submap_factor_numeric_capacity");
      next.buckets.push_back(bucket);
      if(i!=root) {
        const size_t parent=route(bucket->message->columns);
        if(parent<=i)throw std::logic_error("submap_factor_dependency_order");
        pending[parent]->children.push_back(bucket);
      }
    }
    // Reuse the information engine's lineage and query/back-substitution.
    // Its context stays empty; all numeric payload is owned by this snapshot.
    auto information=std::make_shared<Information::Data>(*core_->snapshot());
    information->revision=next.revision+1;
    {
      const auto frontier=next.buckets.back()->message->pinNumeric();
      information->order=frontier->columns;information->jacobian=frontier->jacobian;
      information->residual=frontier->residual;information->constraints=frontier->constraints;information->target=frontier->target;
    }
    information->roundoff=next.buckets.back()->roundoff;
    for(const auto& [column,index]:rank)information->known.insert(column);
    if(next.inventory)information->measurements=SubmapMeasurements::view(next.metadata->measurements(next.inventory));
    else for(const auto& [key,id]:next.resident_assignments)information->measurements.insert(key);
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
      const auto message=reduced->message->pinNumeric();
      base->order=message->columns;base->jacobian=message->jacobian;
      base->residual=message->residual;base->constraints=message->constraints;
      base->target=message->target;base->roundoff=reduced->roundoff;
      if(next.inventory) {
        const std::set<Page> active(next.active.begin(),next.active.end());std::vector<std::string> exclude;
        for(const auto& [id,page]:next.factors)if(active.count(page))exclude.push_back(id);
        base->measurements=SubmapMeasurements::view(next.metadata->measurements(next.inventory,exclude));
      } else for(const auto& rows:next.active)rows->eachOriginal([&](const auto& key){base->measurements.erase(key);});
      next.base=std::move(base);
    }
    if(next.numericBytes()>limits_.numeric_bytes)throw std::runtime_error("submap_factor_numeric_capacity");
  }
  std::shared_ptr<Information> core_;Information::Limits limits_;
  std::shared_ptr<SubmapRowStore> row_store_;
  Ownership ownership_;
  std::shared_ptr<const int> identity_=std::make_shared<const int>(0);
  std::mutex writer_;Snapshot data_;
};
} // namespace posim_slam

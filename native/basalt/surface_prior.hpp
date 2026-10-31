// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "range_surface.hpp"
#include "ping_joint.hpp"
#include "constrained_information.hpp"
#include <optional>

// Persistent *joint* Gaussian information. It replaces consumed rows and keeps
// cross terms with unknown surfaces. No geometric association or support-region
// compression is inferred here; no trial damping may enter this prior.
namespace posim_slam::surface_prior {
constexpr Eigen::Index maxColumns=265;
constexpr size_t maxRecentKeys=32768;
enum class Kind {State,Plane,Pressure};
using VariableKey=std::pair<Kind,int64_t>;
using RangeKey=std::tuple<int64_t,int,int>;
struct Variable {
  Kind kind;int64_t id;Eigen::Index size;Eigen::VectorXd reference;
  VariableKey key() const {return {kind,id};}
};
struct Layout {std::map<VariableKey,std::pair<Eigen::Index,size_t>> columns;Eigen::Index count=0;};
inline Layout layout(const std::vector<Variable>& order,bool retirement_source=false) {
  const size_t state_limit=retirement_source ? kMaxLivePoseBlocks : 16;
  if(order.empty() || order.size()>state_limit+9)throw std::invalid_argument("bounded_unique_surface_prior_order_required");
  Layout out;size_t states=0,planes=0,pressure=0;
  for(size_t i=0;i<order.size();++i) {
    const auto& v=order[i];Eigen::Index reference_size=0;
    if(v.kind==Kind::State) {
      if(v.id<=0 || (v.size!=6 && v.size!=15))throw std::invalid_argument("invalid_persistent_surface_variable");
      reference_size=v.size==6 ? 12 : 21;++states;
    } else if(v.kind==Kind::Plane) {
      if(v.id<0 || v.size!=3)throw std::invalid_argument("invalid_persistent_surface_variable");
      reference_size=4;++planes;
    } else if(v.kind==Kind::Pressure) {
      if(v.id<0 || v.size!=1)throw std::invalid_argument("invalid_persistent_surface_variable");
      reference_size=1;++pressure;
    } else throw std::invalid_argument("invalid_persistent_surface_variable");
    if(v.reference.size()!=reference_size || !v.reference.allFinite())
      throw std::invalid_argument("complete_persistent_owner_reference_required");
    if(v.kind==Kind::State) {
      const Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>> r(v.reference.data()+3);
      if(!surface_ranges::rotation(r))throw std::invalid_argument("invalid_rotation");
    } else if(v.kind==Kind::Plane)surface_ranges::normalBasis(v.reference.head<3>());
    if(!out.columns.emplace(v.key(),std::make_pair(out.count,i)).second)
      throw std::invalid_argument("bounded_unique_surface_prior_order_required");
    out.count+=v.size;
  }
  if(states>state_limit || planes>8 || pressure>1 || out.count>(retirement_source ? kMaxStateColumns+1 : maxColumns))
    throw std::invalid_argument("surface_prior_column_budget");
  return out;
}
inline std::set<RangeKey> keys(const std::vector<RangeKey>& input) {
  std::set<RangeKey> out;
  for(const auto& k:input) {
    if(std::get<0>(k)<=0 || std::get<1>(k)<0 || std::get<2>(k)<0)
      throw std::invalid_argument("original_integer_range_key_required");
    if(!out.insert(k).second)throw std::invalid_argument("reused_surface_range_measurement");
  }
  return out;
}
struct Ledger {
  std::set<RangeKey> recent;int64_t closed_through=0;
  void requireValid() const {
    if(closed_through<0 || recent.size()>maxRecentKeys)throw std::invalid_argument("bounded_recent_range_ledger_required");
    for(const auto& k:recent)
      if(std::get<0>(k)<=closed_through || std::get<1>(k)<0 || std::get<2>(k)<0)
        throw std::invalid_argument("bounded_recent_range_ledger_required");
  }
  Ledger admit(const std::vector<RangeKey>& input,std::optional<int64_t> close=std::nullopt) const {
    requireValid();const auto added=keys(input);
    for(const auto& k:added)
      if(std::get<0>(k)<=closed_through || recent.count(k))throw std::invalid_argument("reused_or_closed_surface_range_measurement");
    const int64_t cutoff=close.value_or(closed_through);
    if(cutoff<closed_through)throw std::invalid_argument("monotonic_range_admission_cutoff_required");
    Ledger out;out.closed_through=cutoff;
    for(const auto& k:recent)if(std::get<0>(k)>cutoff)out.recent.insert(k);
    for(const auto& k:added)if(std::get<0>(k)>cutoff)out.recent.insert(k);
    out.requireValid();return out;
  }
};
inline bool label(const std::string& s) {return s.find_first_not_of(" \t\r\n")!=std::string::npos;}
struct Prior {
  std::vector<Variable> order;Eigen::MatrixXd jacobian;Eigen::VectorXd offset;
  Eigen::MatrixXd constraint_jacobian;Eigen::VectorXd constraint_offset;
  Ledger ledger;std::string epoch;int64_t revision=0;
  void requireValid() const {
    const auto shape=layout(order);ledger.requireValid();
    if(constraint_jacobian.rows())affineSupport(constraint_jacobian,-constraint_offset,shape.count);
    else if(constraint_offset.size())throw std::invalid_argument("surface_constraint_shape");
    if(jacobian.cols()!=shape.count || jacobian.rows()<1 || jacobian.rows()>shape.count+1 ||
       offset.size()!=jacobian.rows() || !jacobian.allFinite() || !offset.allFinite() || !label(epoch) || revision<0)
      throw std::invalid_argument("finite_bounded_surface_prior_required");
  }
};
struct Rows {Eigen::MatrixXd jacobian;Eigen::VectorXd residual;Eigen::MatrixXd constraint_jacobian;Eigen::VectorXd constraint_residual;};
inline Rows linearize(const Prior& prior,const std::vector<Variable>& order,const Eigen::VectorXd& delta,const std::string& epoch,
    bool retirement_source=false) {
  prior.requireValid();const auto target=layout(order,retirement_source);
  if(epoch!=prior.epoch)throw std::invalid_argument("surface_prior_epoch_mismatch");
  if(delta.size()!=target.count || !delta.allFinite())throw std::invalid_argument("finite_owner_delta_required");
  Rows out;out.jacobian=Eigen::MatrixXd::Zero(prior.jacobian.rows(),target.count);Eigen::Index source=0;
  out.constraint_jacobian=Eigen::MatrixXd::Zero(prior.constraint_jacobian.rows(),target.count);
  for(const auto& v:prior.order) {
    const auto found=target.columns.find(v.key());
    if(found==target.columns.end())throw std::invalid_argument("changed_or_missing_surface_prior_reference");
    const auto& current=order[found->second.second];
    if(v.size!=current.size || v.reference.size()!=current.reference.size() ||
       !(v.reference.array()==current.reference.array()).all())
      throw std::invalid_argument("changed_or_missing_surface_prior_reference");
    out.jacobian.middleCols(found->second.first,v.size)=prior.jacobian.middleCols(source,v.size);
    if(prior.constraint_jacobian.rows())out.constraint_jacobian.middleCols(found->second.first,v.size)=prior.constraint_jacobian.middleCols(source,v.size);
    source+=v.size;
  }
  out.residual=out.jacobian*delta+prior.offset;
  out.constraint_residual=out.constraint_jacobian*delta+prior.constraint_offset;return out;
}
struct Retain {Kind kind;int64_t id;Eigen::Index size;};
inline Prior marginalize(const std::vector<Variable>& order,const Eigen::VectorXd& delta,
    const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& residual,const std::vector<Retain>& retain,
    const std::string& epoch,const std::vector<RangeKey>& range_keys={},const Prior* previous=nullptr,
    std::optional<int64_t> close_through=std::nullopt,bool retirement_source=false) {
  const auto source=layout(order,retirement_source);const auto originals=keys(range_keys);
  if(delta.size()!=source.count || !delta.allFinite() || jacobian.cols()!=source.count ||
     jacobian.rows()>4096 || residual.size()!=jacobian.rows() || originals.size()>size_t(jacobian.rows()) ||
     !jacobian.allFinite() || !residual.allFinite() || !label(epoch))
    throw std::invalid_argument("finite_new_surface_rows_required");
  for(const auto& k:originals)
    if(!source.columns.count({Kind::State,std::get<0>(k)}))throw std::invalid_argument("new_range_requires_resident_measurement_pose");
  Eigen::MatrixXd j=jacobian,c=Eigen::MatrixXd::Zero(0,source.count);Eigen::VectorXd r=residual,d;Prior out;out.epoch=epoch;
  if(previous) {
    const auto old=linearize(*previous,order,delta,epoch,retirement_source);const auto rows=j.rows();
    c=old.constraint_jacobian;d=old.constraint_residual;
    j.conservativeResize(rows+old.jacobian.rows(),Eigen::NoChange);j.bottomRows(old.jacobian.rows())=old.jacobian;
    r.conservativeResize(rows+old.residual.size());r.tail(old.residual.size())=old.residual;
    out.ledger=previous->ledger;
    if(previous->revision==std::numeric_limits<int64_t>::max())throw std::invalid_argument("surface_prior_revision_overflow");
    out.revision=previous->revision+1;
  }
  out.ledger=out.ledger.admit(range_keys,close_through);
  std::vector<Eigen::Index> keep;std::set<VariableKey> seen;std::vector<bool> used(source.count,false);
  for(const auto& request:retain) {
    const VariableKey key{request.kind,request.id};const auto found=source.columns.find(key);
    if(found==source.columns.end() || !seen.insert(key).second)
      throw std::invalid_argument("unique_known_retained_surface_blocks_required");
    auto v=order[found->second.second];const auto start=found->second.first;
    if(request.size!=v.size && !(v.kind==Kind::State && v.size==15 && request.size==6))
      throw std::invalid_argument("invalid_surface_prior_block_downgrade");
    if(v.size==15 && request.size==6)v.reference=v.reference.head(12).eval();
    v.size=request.size;out.order.push_back(v);
    for(Eigen::Index i=0;i<v.size;++i){keep.push_back(start+i);used[start+i]=true;}
  }
  layout(out.order);
  if(j.rows()<1)throw std::invalid_argument("surface_prior_requires_consumed_information_rows");
  Eigen::MatrixXd ordered(j.rows(),source.count);Eigen::Index column=0;
  for(auto index:keep)ordered.col(column++)=j.col(index);
  for(Eigen::Index i=0;i<source.count;++i)if(!used[i])ordered.col(column++)=j.col(i);
  if(ordered.rows()>source.count+1) {
    Eigen::MatrixXd augmented(ordered.rows(),source.count+1);augmented<<ordered,r;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
    const Eigen::MatrixXd compact=qr.matrixQR().topRows(source.count+1).template triangularView<Eigen::Upper>();
    ordered=compact.leftCols(source.count);r=compact.col(source.count);
  }
  Eigen::MatrixXd reduced_j=ordered;Eigen::VectorXd reduced_r=r;
  if(c.rows()) {
    Eigen::MatrixXd ordered_c(c.rows(),source.count);Eigen::Index col=0;
    for(auto i:keep)ordered_c.col(col++)=c.col(i);
    for(Eigen::Index i=0;i<source.count;++i)if(!used[i])ordered_c.col(col++)=c.col(i);
    const auto split=splitConstrainedInformation(ordered,r,keep.size(),ordered_c,-d);
    reduced_j=split.reduced.jacobian;reduced_r=split.reduced.residual;
    out.constraint_jacobian=split.constraint_jacobian;
    Eigen::VectorXd retained_delta(keep.size());for(size_t i=0;i<keep.size();++i)retained_delta[i]=delta[keep[i]];
    out.constraint_offset=-split.constraint_target-out.constraint_jacobian*retained_delta;
  } else if(keep.size()<size_t(source.count)) {
    const auto reduced=eliminateJointPatch(ordered,r,keep.size());reduced_j=reduced.jacobian;reduced_r=reduced.residual;
  }
  out.jacobian=std::move(reduced_j);Eigen::VectorXd changed(keep.size());
  for(size_t i=0;i<keep.size();++i)changed[i]=delta[keep[i]];
  out.offset=reduced_r-out.jacobian*changed;
  if(out.jacobian.rows()<1){out.jacobian=Eigen::MatrixXd::Zero(1,keep.size());out.offset=Eigen::VectorXd::Zero(1);}
  if(out.jacobian.rows()>Eigen::Index(keep.size()+1)) {
    const Eigen::Index columns=keep.size();Eigen::MatrixXd augmented(out.jacobian.rows(),columns+1);augmented<<out.jacobian,out.offset;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
    const Eigen::MatrixXd compact=qr.matrixQR().topRows(columns+1).template triangularView<Eigen::Upper>();
    out.jacobian=compact.leftCols(columns);out.offset=compact.col(columns);
  }
  out.requireValid();return out;
}
}  // namespace posim_slam::surface_prior

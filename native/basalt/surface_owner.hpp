// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "retired_pose_conditional.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"
#include <memory>

namespace posim_slam {
namespace surface_owner_detail {
namespace sp=surface_prior;
using Coordinate=std::tuple<sp::Kind,int64_t,int>;
inline std::vector<Coordinate> coordinates(const std::vector<sp::Variable>& variables) {
  std::vector<Coordinate> out;
  for(const auto& v:variables)for(int i=0;i<v.size;++i)out.emplace_back(v.kind,v.id,i);
  return out;
}
inline bool equal(const Eigen::MatrixXd& a,const Eigen::MatrixXd& b) {
  return a.rows()==b.rows() && a.cols()==b.cols() && (a.array()==b.array()).all();
}
inline void sameReference(const sp::Variable& a,const sp::Variable& b) {
  const auto* small=&a;const auto* large=&b;if(a.size>b.size)std::swap(small,large);
  if(a.key()!=b.key() || (a.size!=b.size && !(a.kind==sp::Kind::State && small->size==6 && large->size==15)) ||
      !equal(small->reference,large->reference.head(small->reference.size())))
    throw std::invalid_argument("changed_surface_owner_reference");
}
inline void compact(sp::Prior& p) {
  const auto count=sp::layout(p.order).count;
  if(!p.jacobian.rows()){p.jacobian=Eigen::MatrixXd::Zero(1,count);p.offset=Eigen::VectorXd::Zero(1);}
  if(p.jacobian.rows()>count+1) {
    Eigen::MatrixXd augmented(p.jacobian.rows(),count+1);augmented<<p.jacobian,p.offset;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
    const Eigen::MatrixXd rows=qr.matrixQR().topRows(count+1).triangularView<Eigen::Upper>();
    p.jacobian=rows.leftCols(count);p.offset=rows.col(count);
  }
  p.requireValid();
}
using Nodes=std::map<uint64_t,std::shared_ptr<const RetiredPoseConditional>>;
inline void validateArchive(const Nodes& nodes,const std::vector<sp::Variable>& root) {
  if(nodes.size()>256)throw std::invalid_argument("native_conditional_archive_capacity");
  std::map<Coordinate,uint64_t> producers;std::map<sp::VariableKey,sp::Variable> references;
  const auto live=coordinates(root);const std::set<Coordinate> roots(live.begin(),live.end());size_t bytes=0;
  auto reference=[&](const sp::Variable& v) {
    const auto found=references.find(v.key());
    if(found==references.end())references.emplace(v.key(),v);
    else {sameReference(found->second,v);if(found->second.size<v.size)found->second=v;}
  };
  for(const auto& v:root)reference(v);
  for(const auto& [id,pointer]:nodes) {
    if(!pointer || !id || pointer->id!=id)throw std::invalid_argument("native_conditional_identity");
    const auto& n=*pointer;const auto count=sp::layout(n.variables,true).count;
    std::vector<int> partition=n.kept;partition.insert(partition.end(),n.removed.begin(),n.removed.end());
    std::sort(partition.begin(),partition.end());
    if(n.removed.empty() || partition.size()!=size_t(count))throw std::invalid_argument("native_conditional_partition");
    for(int i=0;i<count;++i)if(partition[i]!=i)throw std::invalid_argument("native_conditional_partition");
    const auto rows=Eigen::Index(n.removed.size());
    if(n.regression.rows()!=rows || n.regression.cols()!=Eigen::Index(n.kept.size()) || n.offset.size()!=rows ||
        n.noise.rows()!=rows || n.unbounded.rows()!=rows || n.noise.cols()+n.unbounded.cols()!=rows ||
        !n.regression.allFinite() || !n.offset.allFinite() || !n.noise.allFinite() || !n.unbounded.allFinite())
      throw std::invalid_argument("native_conditional_geometry");
    Eigen::MatrixXd basis(rows,rows);basis.rightCols(n.unbounded.cols())=n.unbounded;
    for(Eigen::Index i=0;i<n.noise.cols();++i) {
      const double norm=n.noise.col(i).norm();if(!(norm>0))throw std::invalid_argument("native_conditional_noise");
      basis.col(i)=n.noise.col(i)/norm;
    }
    if((basis.transpose()*basis-Eigen::MatrixXd::Identity(rows,rows)).cwiseAbs().maxCoeff()>2e-7)
      throw std::invalid_argument("native_conditional_nullspace");
    bytes+=8*size_t(n.regression.size()+n.offset.size()+n.noise.size()+n.unbounded.size());
    if(bytes>32*1024*1024)throw std::invalid_argument("native_conditional_archive_capacity");
    const auto cols=coordinates(n.variables);for(const auto& v:n.variables)reference(v);
    for(int i:n.removed)if(roots.count(cols[i]) || !producers.emplace(cols[i],id).second)
      throw std::invalid_argument("duplicate_native_conditional_owner");
  }
  for(const auto& [id,pointer]:nodes) {
    const auto cols=coordinates(pointer->variables);
    for(int i:pointer->kept)if(!roots.count(cols[i])) {
      const auto found=producers.find(cols[i]);
      if(found==producers.end() || found->second<=id)throw std::invalid_argument("native_conditional_separator");
    }
  }
}
}  // namespace surface_owner_detail

// Complete marginal information, jointly retaining states, unknown planes and
// the pressure datum. Active IMU/DVL/pressure observations remain in their native
// stores and enter exactly once. Prepared increments never replace those rows.
// This component is owner-thread-only; a frontend prepares late likelihoods in
// the same immutable references and installs their conditionals with the rows.
class SurfaceMarginalOwner {
 public:
  using Variable=surface_prior::Variable;using RangeKey=surface_prior::RangeKey;
  struct State {surface_prior::Prior prior;surface_owner_detail::Nodes nodes;uint64_t highest_id=0;};
  struct Snapshot {
    std::shared_ptr<const State> state;
    surface_prior::Coordinates coordinates;
    basalt::MargLinData<double> legacy;
    PressureDatumPrior pressure;
    uint64_t owner_revision;
  };
  class Prepared {
    friend class SurfaceMarginalOwner;
    Snapshot snapshot;std::shared_ptr<const State> next;
    decltype(basalt::BundleAdjustmentBase<double>::frame_states) states;
    decltype(basalt::BundleAdjustmentBase<double>::frame_poses) poses;
    PingPlaneStore planes;basalt::MargLinData<double> zero;
    Prepared(Snapshot s,std::shared_ptr<const State> n,
        decltype(states) owned_states,decltype(poses) owned_poses,PingPlaneStore owned_planes,
        basalt::MargLinData<double> empty):snapshot(std::move(s)),next(std::move(n)),states(std::move(owned_states)),
          poses(std::move(owned_poses)),planes(std::move(owned_planes)),zero(std::move(empty)) {}
   public:
    Prepared(Prepared&&)=default;Prepared& operator=(Prepared&&)=default;
  };
  bool active() const {return bool(state_);}
  const State& state() const {if(!state_)throw std::logic_error("surface_owner_not_adopted");return *state_;}
  static basalt::MargLinData<double> zeroPrior(const basalt::AbsOrderMap& order) {
    requireStateOrder(order);basalt::MargLinData<double> out;out.is_sqrt=true;out.order=order;
    out.H=Eigen::MatrixXd::Zero(1,order.total_size);out.b=Eigen::VectorXd::Zero(1);return out;
  }
  static void requireZero(const basalt::MargLinData<double>& prior,const PressureFactorStore& pressure) {
    if(!prior.is_sqrt || !prior.H.isZero(0) || !prior.b.isZero(0) || pressure.prior().active)
      throw std::invalid_argument("surface_marginal_information_has_duplicate_legacy_owner");
  }
  surface_prior::Coordinates coordinates(const basalt::BundleAdjustmentBase<double>& owner,
      const basalt::AbsOrderMap& order,const PressureFactorStore& pressure) const {
    requireStateOrder(order);std::map<int,int64_t> sorted;
    for(const auto& [stamp,block]:order.abs_order_map)sorted.emplace(block.first,stamp);
    std::vector<Variable> variables;
    for(const auto& [column,stamp]:sorted) {
      const auto size=order.abs_order_map.at(stamp).second;Eigen::VectorXd reference;
      if(size==6)reference=surface_prior::poseReference(owner.frame_poses.at(stamp).getPoseLin(),12);
      else {
        const auto& s=owner.frame_states.at(stamp).getStateLin();reference=surface_prior::poseReference(s.T_w_i,21);
        reference.segment<3>(12)=s.vel_w_i;reference.segment<3>(15)=s.bias_gyro;reference.segment<3>(18)=s.bias_accel;
      }
      variables.push_back({surface_prior::Kind::State,stamp,size,reference});
    }
    variables.push_back({surface_prior::Kind::Pressure,0,1,Eigen::VectorXd::Zero(1)});
    if(state_)for(const auto& v:state_->prior.order)if(v.kind==surface_prior::Kind::Plane)variables.push_back(v);
    return surface_prior::ownedCoordinates(owner,variables,surface_prior::ScalarCoordinate{0,0.,pressure.datum()},true);
  }
  surface_prior::Rows linearize(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const PressureFactorStore& pressure) const {
    const auto current=coordinates(owner,order,pressure);
    return surface_prior::linearize(state().prior,current.order,current.delta,state().prior.epoch,true);
  }
  void adopt(basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      basalt::MargLinData<double>& legacy,PressureFactorStore& pressure,const std::string& epoch) {
    if(active() || owner.posim_ping_trial_active || !surface_prior::label(epoch) ||
        owner.posim_ping_planes.size() || !owner.lmdb.getLandmarks().empty() || !legacy.is_sqrt)
      throw std::invalid_argument("exclusive_camera_free_surface_owner_required");
    const auto current=coordinates(owner,order,pressure);const auto count=current.delta.size();
    if(legacy.H.cols()!=Eigen::Index(legacy.order.total_size) || legacy.H.rows()!=legacy.b.size())
      throw std::invalid_argument("surface_legacy_prior_shape");
    auto next=std::make_shared<State>();auto& prior=next->prior;prior.order=current.order;prior.epoch=epoch;
    prior.jacobian=Eigen::MatrixXd::Zero(legacy.H.rows()+int(pressure.prior().active),count);
    prior.offset=Eigen::VectorXd::Zero(prior.jacobian.rows());prior.offset.head(legacy.b.size())=legacy.b;
    for(const auto& [stamp,block]:legacy.order.abs_order_map) {
      const auto found=order.abs_order_map.at(stamp);
      if(found.second<block.second)throw std::invalid_argument("surface_legacy_prior_state");
      prior.jacobian.block(0,found.first,legacy.H.rows(),block.second)=legacy.H.middleCols(block.first,block.second);
    }
    const auto& datum=pressure.prior();
    if(datum.active) {
      const auto row=legacy.H.rows();
      for(const auto& [stamp,block]:datum.order.abs_order_map) {
        const auto found=order.abs_order_map.at(stamp);
        if(found.second<block.second)throw std::invalid_argument("surface_pressure_prior_state");
        prior.jacobian.block(row,found.first,1,block.second)=datum.states.segment(block.first,block.second);
      }
      prior.jacobian(row,order.total_size)=datum.datum;prior.offset[row]=datum.b-datum.datum*datum.reference;
    }
    surface_owner_detail::compact(prior);
    auto states=owner.frame_states;auto poses=owner.frame_poses;freeze(states,poses);
    auto zero=zeroPrior(order);const auto plan=pressure.planRetirement({});
    pressure.retireWith(plan,{},[&] {
      owner.frame_states.swap(states);owner.frame_poses.swap(poses);swapPrior(legacy,zero);
      state_=std::move(next);++owner.posim_ping_state_revision;
    });
  }
  Snapshot snapshot(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const basalt::MargLinData<double>& legacy,const PressureFactorStore& pressure) const {
    if(owner.posim_ping_trial_active)throw std::logic_error("surface_snapshot_during_trial");
    requireZero(legacy,pressure);state();
    return {state_,coordinates(owner,order,pressure),legacy,pressure.prior(),owner.posim_ping_state_revision};
  }
  Prepared prepareLate(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const basalt::MargLinData<double>& legacy,const PressureFactorStore& pressure,const Snapshot& expected,
      const Eigen::MatrixXd& increment,const Eigen::VectorXd& offset,const std::vector<Variable>& new_planes,
      const std::vector<Variable>& source_order,const std::vector<RetiredPoseConditional>& replacements,
      const std::vector<RangeKey>& keys) const {
    requireSnapshot(owner,order,legacy,pressure,expected);
    if(keys.empty() || keys.size()>surface_prior::maxRecentKeys)throw std::invalid_argument("original_late_range_keys_required");
    auto next=std::make_shared<State>(*state_);auto variables=expected.coordinates.order;
    auto planes=owner.posim_ping_planes;
    for(const auto& v:new_planes) {
      if(v.kind!=surface_prior::Kind::Plane || v.id>std::numeric_limits<int>::max())
        throw std::invalid_argument("new_unknown_surface_planes_required");
      surface_prior::layout({v});planes.set(int(v.id),v.reference.head<3>(),v.reference[3]);variables.push_back(v);
    }
    const auto count=surface_prior::layout(variables).count;
    if(increment.cols()!=count || increment.rows()>count+1 || offset.size()!=increment.rows() ||
        !increment.allFinite() || !offset.allFinite())throw std::invalid_argument("finite_new_live_surface_rows_required");
    // Every original reading belongs to an existing current or retiring pose.
    surface_prior::layout(source_order,true);
    const auto roots=surface_owner_detail::coordinates(variables);const std::set<surface_owner_detail::Coordinate> root(roots.begin(),roots.end());
    std::map<surface_owner_detail::Coordinate,uint64_t> producers;std::map<surface_prior::VariableKey,Variable> references;
    for(const auto& v:variables)references.emplace(v.key(),v);
    for(const auto& [id,node]:state_->nodes) {
      const auto columns=surface_owner_detail::coordinates(node->variables);
      for(int i:node->removed)producers.emplace(columns[i],id);
      for(const auto& v:node->variables) {
        const auto found=references.find(v.key());
        if(found==references.end())references.emplace(v.key(),v);
        else if(found->second.size<v.size)found->second=v;
      }
    }
    std::set<int64_t> stamps;
    for(const auto& v:source_order) {
      const auto found=references.find(v.key());
      if(found==references.end() || found->second.size<v.size)throw std::invalid_argument("late_surface_source_not_owned");
      surface_owner_detail::sameReference(found->second,v);
      if(v.kind==surface_prior::Kind::State)stamps.insert(v.id);
    }
    for(const auto& k:keys)if(!stamps.count(std::get<0>(k)))throw std::invalid_argument("late_range_pose_not_owned");
    next->prior.ledger=state_->prior.ledger.admit(keys);
    std::set<uint64_t> required,replaced;std::vector<uint64_t> pending;
    for(const auto& key:surface_owner_detail::coordinates(source_order))if(!root.count(key)) {
      const auto found=producers.find(key);if(found==producers.end())throw std::invalid_argument("late_surface_coordinate_not_owned");
      pending.push_back(found->second);
    }
    while(!pending.empty()) {
      const auto id=pending.back();pending.pop_back();if(!required.insert(id).second)continue;
      const auto& node=*state_->nodes.at(id);const auto columns=surface_owner_detail::coordinates(node.variables);
      for(int i:node.kept)if(!root.count(columns[i]))pending.push_back(producers.at(columns[i]));
    }
    for(const auto& node:replacements) {
      if(!next->nodes.count(node.id) || !replaced.insert(node.id).second)throw std::invalid_argument("late_conditional_replacement_not_owned");
      const auto& old=*state_->nodes.at(node.id);
      const auto a=surface_owner_detail::coordinates(old.variables),b=surface_owner_detail::coordinates(node.variables);
      std::vector<surface_owner_detail::Coordinate> original,changed;
      for(int i:old.removed)original.push_back(a.at(i));for(int i:node.removed)changed.push_back(b.at(i));
      if(original!=changed)throw std::invalid_argument("late_conditional_removed_ownership_changed");
      next->nodes[node.id]=std::make_shared<RetiredPoseConditional>(node);
    }
    if(replaced!=required)throw std::invalid_argument("complete_late_conditional_closure_required");
    surface_owner_detail::validateArchive(next->nodes,variables);
    Eigen::VectorXd delta=Eigen::VectorXd::Zero(count);delta.head(expected.coordinates.delta.size())=expected.coordinates.delta;
    const auto old=surface_prior::linearize(state_->prior,variables,delta,state_->prior.epoch);
    next->prior.order=variables;next->prior.jacobian.resize(old.jacobian.rows()+increment.rows(),count);
    next->prior.jacobian<<old.jacobian,increment;
    next->prior.offset.resize(old.residual.size()+offset.size());
    next->prior.offset<<old.residual-old.jacobian*delta,offset;
    if(next->prior.revision==std::numeric_limits<int64_t>::max())throw std::invalid_argument("surface_owner_revision_overflow");
    ++next->prior.revision;surface_owner_detail::compact(next->prior);
    Prepared out{expected,std::move(next),owner.frame_states,owner.frame_poses,std::move(planes),zeroPrior(order)};
    freeze(out.states,out.poses);return out;
  }
  void install(basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      basalt::MargLinData<double>& legacy,const PressureFactorStore& pressure,Prepared&& prepared) {
    requireSnapshot(owner,order,legacy,pressure,prepared.snapshot);
    // Prepared is private owner data, not a caller-supplied folded whole-system
    // prior. All allocation and rejection precede these no-allocation swaps.
    owner.frame_states.swap(prepared.states);owner.frame_poses.swap(prepared.poses);
    std::swap(owner.posim_ping_planes,prepared.planes);swapPrior(legacy,prepared.zero);
    state_.swap(prepared.next);++owner.posim_ping_state_revision;
  }
  template<class ImuMap> PingRetirementPlan marginalize(basalt::BundleAdjustmentBase<double>& owner,
      const basalt::AbsOrderMap& order,basalt::MargLinData<double>& legacy,ImuMap& measurements,
      const basalt::ImuLinData<double>& imu,const std::set<int64_t>& removed,const std::set<int64_t>& pose_only,
      DvlFactorStore* dvl,PressureFactorStore& pressure) {
    requireZero(legacy,pressure);
    if(owner.posim_ping_trial_active || !owner.lmdb.getLandmarks().empty() || imu.imu_meas.size()!=measurements.size())
      throw std::invalid_argument("exclusive_native_surface_retirement_required");
    std::vector<std::pair<int64_t,int64_t>> intervals;
    for(const auto& [stamp,m]:measurements) {
      if(!imu.imu_meas.count(stamp) || imu.imu_meas.at(stamp)!=&m || stamp!=m.get_start_t_ns())
        throw std::invalid_argument("changed_native_surface_imu_owner");
      intervals.emplace_back(stamp,stamp+m.get_dt_ns());
    }
    const auto planned=planPingMarginalLayout(owner,removed,pose_only,{},intervals);
    if(order.abs_order_map!=planned.source.abs_order_map)throw std::invalid_argument("changed_native_surface_state_order");
    const auto current=coordinates(owner,order,pressure);
    auto consumed=imu;consumed.imu_meas.clear();
    for(int64_t stamp:planned.consumed_imu_starts)consumed.imu_meas.emplace(stamp,imu.imu_meas.at(stamp));
    PingLinearization::Options options;options.lb_options.huber_parameter=owner.huber_thresh;
    options.lb_options.obs_std_dev=owner.obs_std_dev;
    basalt::LinearizationAbsQR<double,6> rows(&owner,order,options,&legacy,&consumed,&removed);
    bool valid=false;rows.linearizeProblem(&valid);if(!valid)throw std::invalid_argument("invalid_native_surface_consumed_rows");
    rows.performQR();Eigen::MatrixXd j;Eigen::VectorXd r;rows.get_dense_Q2Jp_Q2r(j,r);
    j.conservativeResize(Eigen::NoChange,current.delta.size());j.rightCols(j.cols()-order.total_size).setZero();
    auto append=[&](const Eigen::MatrixXd& a,const Eigen::VectorXd& b) {
      const auto start=j.rows();j.conservativeResize(start+a.rows(),Eigen::NoChange);j.bottomRows(a.rows()).setZero();
      j.bottomLeftCorner(a.rows(),a.cols())=a;r.conservativeResize(start+b.size());r.tail(b.size())=b;
    };
    std::optional<DvlFactorStore::Retirement> velocity;
    if(dvl) {
      velocity=dvl->planRetirement(removed,pose_only);const auto v=dvl->linearize(owner,order,&velocity->stamps);
      append(v.jacobian,v.residual);
    }
    const auto depth_plan=pressure.planRetirement(removed);const auto depth=pressure.linearize(owner,order,&depth_plan.stamps);
    append(depth.jacobian,depth.residual);
    std::map<int,int64_t> by_column;for(const auto& [stamp,block]:planned.retained.abs_order_map)by_column.emplace(block.first,stamp);
    std::vector<surface_prior::Retain> retain;
    for(const auto& [column,stamp]:by_column)retain.push_back({surface_prior::Kind::State,stamp,planned.retained.abs_order_map.at(stamp).second});
    retain.push_back({surface_prior::Kind::Pressure,0,1});
    for(const auto& v:state().prior.order)if(v.kind==surface_prior::Kind::Plane)retain.push_back({v.kind,v.id,v.size});
    auto next=std::make_shared<State>(*state_);
    next->prior=surface_prior::marginalize(current.order,current.delta,j,r,retain,state_->prior.epoch,{},&state_->prior,std::nullopt,true);
    const auto previous=surface_prior::linearize(state_->prior,current.order,current.delta,state_->prior.epoch,true);
    append(previous.jacobian,previous.residual);
    const auto source=surface_prior::layout(current.order,true);std::vector<int> kept;
    for(const auto& v:next->prior.order) {
      const auto start=source.columns.at(v.key()).first;for(int i=0;i<v.size;++i)kept.push_back(start+i);
    }
    auto conditional=captureRetiringConditional(current,kept,j,r,next->prior.jacobian,next->prior.offset);
    if(conditional) {
      if(next->highest_id==std::numeric_limits<uint64_t>::max())throw std::invalid_argument("native_conditional_id_overflow");
      conditional->id=++next->highest_id;
      if(conditional->marginal_information_relative_error>1e-7 || conditional->marginal_gradient_relative_error>1e-7)
        throw std::invalid_argument("native_surface_marginal_conditional_mismatch");
      next->nodes.emplace(conditional->id,std::make_shared<RetiredPoseConditional>(std::move(*conditional)));
    }
    surface_owner_detail::validateArchive(next->nodes,next->prior.order);
    auto zero=zeroPrior(planned.retained);auto groups=std::vector<JointPingGroupSpec>{};
    const auto retirement=planPingRetirement(owner,groups,removed,removed,{});
    std::shared_ptr<const State> publish=std::move(next);
    auto install=[&] {
      installPingMarginalPrior(owner,legacy,groups,measurements,retirement,planned,removed,pose_only,zero);
      state_.swap(publish);
    };
    auto with_velocity=[&] {if(dvl)dvl->retireWith(*velocity,install);else install();};
    pressure.retireWith(depth_plan,{},with_velocity);return retirement;
  }
 private:
  std::shared_ptr<const State> state_;
  template<class States,class Poses> static void freeze(States& states,Poses& poses) {
    for(auto& [stamp,s]:states)if(!s.isLinearized())s.setLinTrue();
    for(auto& [stamp,s]:poses)if(!s.isLinearized())s.setLinTrue();
  }
  static void swapPrior(basalt::MargLinData<double>& a,basalt::MargLinData<double>& b) {
    a.H.swap(b.H);a.b.swap(b.b);a.order.abs_order_map.swap(b.order.abs_order_map);
    std::swap(a.order.total_size,b.order.total_size);std::swap(a.order.items,b.order.items);std::swap(a.is_sqrt,b.is_sqrt);
  }
  void requireSnapshot(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const basalt::MargLinData<double>& legacy,const PressureFactorStore& pressure,const Snapshot& expected) const {
    if(owner.posim_ping_trial_active || state_!=expected.state || owner.posim_ping_state_revision!=expected.owner_revision ||
        !surface_owner_detail::equal(legacy.H,expected.legacy.H) || !surface_owner_detail::equal(legacy.b,expected.legacy.b) ||
        legacy.order.abs_order_map!=expected.legacy.order.abs_order_map)
      throw std::invalid_argument("stale_native_surface_snapshot");
    requireZero(legacy,pressure);const auto current=coordinates(owner,order,pressure);
    if(current.order.size()!=expected.coordinates.order.size() || !surface_owner_detail::equal(current.delta,expected.coordinates.delta))
      throw std::invalid_argument("stale_native_surface_snapshot");
    for(size_t i=0;i<current.order.size();++i) {
      const auto& a=current.order[i];const auto& b=expected.coordinates.order[i];
      if(a.size!=b.size)throw std::invalid_argument("stale_native_surface_snapshot");surface_owner_detail::sameReference(a,b);
    }
  }
};
}  // namespace posim_slam

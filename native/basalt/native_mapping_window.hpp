// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "native_mapping_linearization.hpp"
#include "native_mapping_schedule.hpp"
#include "native_mapping_identity.hpp"
#include "submap_factorization.hpp"
#include "mapping_graph_information.hpp"
#include <functional>
#include <numeric>

namespace posim_slam {
inline constexpr double kNativeMappingPi=3.14159265358979323846;
// Fixed coordinates for a conditional mapping mean. The original reference
// position and heading select the four NFR gauge freedoms, not measurements.
// Its roll/pitch remain variables. World-reference covariance belongs to the
// information model, and must never be inferred from this mean-only chart.
class NativeMappingChart {
  friend class NativeMappingCheckpoint;
 public:
  NativeMappingChart(basalt::NfrMapper& mapper,int64_t reference,
      size_t maximum_coordinates=kMaxNativeMappingPoseCoordinates)
      :reference_(reference),coordinate_capacity_(maximum_coordinates) {
    for(const auto& [stamp,state]:mapper.getFramePoses()) {
      if(state.isLinearized() || !state.getPose().matrix().allFinite())
        throw std::invalid_argument("native_window_current_pose_required");
      anchors_.emplace(stamp,state.getPose());poses_.push_back(stamp);
    }
    offsets_=nativeMappingPoseAddresses(poses_,coordinate_capacity_);
    reference_angles_=angles(anchors_.at(reference_));
    for(const auto& [id,point]:mapper.lmdb.getLandmarks()) {
      points_.emplace(id,Eigen::Vector3d(point.direction.x(),point.direction.y(),point.inv_dist));
      hosts_.emplace(id,point.host_kf_id);
    }
  }
  const std::vector<int64_t>& poses() const {return poses_;}
  int64_t reference() const {return reference_;}
  const std::map<int64_t,int>& offsets() const {return offsets_;}
  const Eigen::Vector3d& point(int64_t id) const {return points_.at(id);}
  size_t numericBytes() const {return sizeof(double)*(7*anchors_.size()+3*points_.size()+3);}
  NativeMappingChart extended(basalt::NfrMapper& mapper) const {
    NativeMappingChart next=*this;
    for(auto stamp:poses_)if(!mapper.getFramePoses().count(stamp))
      throw std::invalid_argument("native_window_pose_removed");
    if(mapper.getFramePoses().size()>coordinate_capacity_/6)
      throw std::runtime_error("native_window_coordinate_capacity");
    for(const auto& [stamp,state]:mapper.getFramePoses())if(!next.offsets_.count(stamp)) {
      if(state.isLinearized() || !state.getPose().matrix().allFinite())
        throw std::invalid_argument("native_window_current_pose_required");
      next.offsets_.emplace(stamp,6*next.poses_.size());next.poses_.push_back(stamp);
      next.anchors_.emplace(stamp,state.getPose());
    }
    // Landmark coordinates are local to their host. A new host starts a new
    // landmark chart; that complete source group must then be relinearized.
    std::set<int64_t> present;
    for(const auto& [id,point]:mapper.lmdb.getLandmarks()) {
      present.insert(id);
      if(!next.points_.count(id) || next.hosts_.at(id)!=point.host_kf_id) {
        next.points_[id]={point.direction.x(),point.direction.y(),point.inv_dist};
        next.hosts_[id]=point.host_kf_id;
      }
    }
    for(auto it=next.points_.begin();it!=next.points_.end();) {
      if(!present.count(it->first)){next.hosts_.erase(it->first);it=next.points_.erase(it);}else ++it;
    }
    next.coordinates(mapper);return next;
  }
  Eigen::VectorXd coordinates(basalt::NfrMapper& mapper) const {
    Eigen::VectorXd out=Eigen::VectorXd::Zero(6*poses_.size());
    if(mapper.getFramePoses().size()!=poses_.size())throw std::invalid_argument("native_window_pose_set_changed");
    for(const auto& [stamp,index]:offsets_) {
      const auto& current=mapper.getFramePoses().at(stamp).getPose();const auto& anchor=anchors_.at(stamp);
      if(stamp==reference_) {
        const auto now=angles(current);
        if((current.translation()-anchor.translation()).norm()>1e-10 ||
            std::abs(std::remainder(now.z()-reference_angles_.z(),2*kNativeMappingPi))>1e-10)
          throw std::invalid_argument("native_window_reference_changed");
        out[index+3]=std::remainder(now.x()-reference_angles_.x(),2*kNativeMappingPi);
        out[index+4]=now.y()-reference_angles_.y();
      } else {
        out.segment<3>(index)=current.translation()-anchor.translation();
        out.segment<3>(index+3)=(current.so3()*anchor.so3().inverse()).log();
        if(out.segment<3>(index+3).norm()>=kNativeMappingPi-1e-6)
          throw std::invalid_argument("native_window_rotation_chart_boundary");
      }
    }
    if(!out.allFinite())throw std::invalid_argument("native_window_coordinates_nonfinite");
    return out;
  }
  Eigen::Matrix<double,6,6> derivative(int64_t stamp,const Eigen::VectorXd& value) const {
    if(value.size()!=6*poses_.size() || !value.allFinite())throw std::invalid_argument("native_window_coordinates_nonfinite");
    const int index=offsets_.at(stamp);Eigen::Matrix<double,6,6> out=Eigen::Matrix<double,6,6>::Zero();
    if(stamp==reference_) {
      const auto yaw=Sophus::SO3d::exp(reference_angles_.z()*Eigen::Vector3d::UnitZ());
      const auto pitch=Sophus::SO3d::exp((reference_angles_.y()+value[index+4])*Eigen::Vector3d::UnitY());
      out.block<3,1>(3,3)=yaw* (pitch*Eigen::Vector3d::UnitX());
      out.block<3,1>(3,4)=yaw*Eigen::Vector3d::UnitY();
    } else {
      out.topLeftCorner<3,3>().setIdentity();
      out.bottomRightCorner<3,3>()=Sophus::SO3d::leftJacobian(value.segment<3>(index+3));
    }
    return out;
  }
  void apply(basalt::NfrMapper& mapper,const Eigen::VectorXd& value) const {
    if(value.size()!=6*poses_.size() || !value.allFinite())throw std::invalid_argument("native_window_coordinates_nonfinite");
    auto next=mapper.getFramePoses();
    for(const auto& [stamp,index]:offsets_) {
      Sophus::SE3d pose=anchors_.at(stamp);
      if(stamp==reference_) {
        const double pitch=reference_angles_.y()+value[index+4];
        if(std::abs(pitch)>=kNativeMappingPi/2-1e-6)throw std::invalid_argument("native_window_reference_heading_undefined");
        pose.so3()=Sophus::SO3d::exp(reference_angles_.z()*Eigen::Vector3d::UnitZ())*
          Sophus::SO3d::exp(pitch*Eigen::Vector3d::UnitY())*
          Sophus::SO3d::exp((reference_angles_.x()+value[index+3])*Eigen::Vector3d::UnitX());
      } else {
        if(value.segment<3>(index+3).norm()>=kNativeMappingPi-1e-6)throw std::invalid_argument("native_window_rotation_chart_boundary");
        pose.translation()+=value.segment<3>(index);
        pose.so3()=Sophus::SO3d::exp(value.segment<3>(index+3))*pose.so3();
      }
      next.at(stamp)=basalt::PoseStateWithLin<double>(stamp,pose);
    }
    mapper.getFramePoses()=std::move(next);
  }
  NativeMappingRows rows(basalt::NfrMapper& mapper,NativeMappingRows source) const {
    const auto value=coordinates(mapper);
    for(auto& factor:source.factors)transform(factor.poses,factor.jacobian,factor.residual,value);
    for(auto& group:source.points)transform(mapper,group,value);
    return source;
  }
  template<class FactorSink,class PointSink>
  NativeMappingRowSummary streamRows(basalt::NfrMapper& mapper,
      FactorSink&& factor_sink,PointSink&& point_sink,
      const std::vector<NativeMappingRowSource>* order=nullptr,
      const NativeMappingSelection* selection=nullptr) const {
    const auto value=coordinates(mapper);
    return streamNativeMappingRows(mapper,
      [&](NativeMappingPoseRows row) {
        transform(row.poses,row.jacobian,row.residual,value);factor_sink(std::move(row));
      },
      [&](NativeMappingPointRows row) {
        transform(mapper,row,value);point_sink(std::move(row));
      },selection,order,coordinate_capacity_);
  }
 private:
  void transform(const std::vector<int64_t>& poses,Eigen::MatrixXd& jacobian,
      Eigen::VectorXd& residual,const Eigen::VectorXd& value) const {
    for(size_t i=0;i<poses.size();++i) {
      jacobian.middleCols<6>(6*i)=(jacobian.middleCols<6>(6*i)*derivative(poses[i],value)).eval();
      residual.noalias()-=jacobian.middleCols<6>(6*i)*value.segment<6>(offsets_.at(poses[i]));
    }
  }
  void transform(basalt::NfrMapper& mapper,NativeMappingPointRows& group,
      const Eigen::VectorXd& value) const {
    const auto& point=mapper.lmdb.getLandmark(group.id);
    if(group.host!=hosts_.at(group.id))throw std::invalid_argument("native_window_landmark_host_changed");
    const Eigen::Vector3d delta=Eigen::Vector3d(point.direction.x(),point.direction.y(),point.inv_dist)-points_.at(group.id);
    transform(group.poses,group.pose,group.residual,value);group.residual.noalias()-=group.landmark*delta;
  }
  static Eigen::Vector3d angles(const Sophus::SE3d& pose) {
    const auto r=pose.rotationMatrix();const double horizontal=r.col(0).head<2>().norm();
    if(horizontal<=1e-6)throw std::invalid_argument("native_window_reference_heading_undefined");
    return {std::atan2(r(2,1),r(2,2)),std::atan2(-r(2,0),horizontal),std::atan2(r(1,0),r(0,0))};
  }
  int64_t reference_;Eigen::Vector3d reference_angles_;
  size_t coordinate_capacity_;
  std::vector<int64_t> poses_;std::map<int64_t,int> offsets_;
  Eigen::aligned_map<int64_t,Sophus::SE3d> anchors_;
  std::map<int64_t,Eigen::Vector3d> points_;std::map<int64_t,basalt::TimeCamId> hosts_;
};

// Last nonlinear anchor of one current source group. Raw measurements stay
// in the graph/input archive; numeric elimination caches need not be serialized.
struct NativeMappingSourceAnchor {
  Eigen::aligned_map<int64_t,Sophus::SE3d> poses;
  Eigen::Vector3d point=Eigen::Vector3d::Zero();bool has_point=false;
  size_t bytes() const {return sizeof(double)*(7*poses.size()+(has_point?3:0));}
};
struct NativeMappingWindowState {
  explicit NativeMappingWindowState(NativeMappingChart value):chart(std::move(value)) {}
  NativeMappingChart chart;
  std::set<int64_t> active;
  SubmapFactorization::Layout layout;
  std::map<std::string,std::shared_ptr<const NativeMappingSourceAnchor>> anchors;
  std::vector<std::string> unassigned;
  InputDigest::Value source{},nonlinear{};
};

// Nonlinear owner for one explicitly selected active window. Original factor
// groups wholly inside it remain replaceable. Groups crossing its boundary
// enter a sealed Gaussian conditional exactly once, retaining back-substitution.
// A window whose separator exceeds the supplied capacity is rejected intact.
// The caller exclusively owns mapper during this object's lifetime. Graph
// growth, track reassociation and rehosting are staged with prepareGrowth,
// which replaces affected source groups before publication. Checkpoints retain
// each group's source anchor and the stable coordinate/elimination layout.
class NativeMappingWindow {
  friend class NativeMappingCheckpoint;
 public:
  struct Step {bool accepted=false;int trials=0;double before=0,after=0;size_t active_originals=0;};
  struct WorldPoints {
    // Current nonlinear positions, with the hybrid model's linearized joint
    // uncertainty. These are not posterior-mean increments or admission flags.
    Eigen::VectorXd positions;
    Eigen::MatrixXd covariance,unbounded_gram;
    bool observed=false;
  };
  class WorldLinearization {
    friend class NativeMappingWindow;
    SubmapFactorization::Snapshot state;
    SubmapInformation::Solution solution;
    Eigen::VectorXd coordinates;
    WorldLinearization(SubmapFactorization::Snapshot snapshot,SubmapInformation::Solution solved,Eigen::VectorXd value)
        :state(std::move(snapshot)),solution(std::move(solved)),coordinates(std::move(value)) {}
   public:
    WorldLinearization(const WorldLinearization&)=default;
    WorldLinearization(WorldLinearization&&)=default;
  };
  NativeMappingWindow(basalt::NfrMapper& mapper,int64_t reference,
      const std::set<int64_t>& active_poses,SubmapInformation::Limits limits={},
      std::shared_ptr<SubmapRowStore> row_store={})
      :NativeMappingWindow(mapper,NativeMappingChart(mapper,reference,limits.coordinates),active_poses,limits,nullptr,{},std::move(row_store)) {
    auto selected=[&](const std::vector<int64_t>& poses) {
      return std::all_of(poses.begin(),poses.end(),[&](auto stamp){return active_poses.count(stamp);});
    };
    std::vector<SubmapFactorization::Factor> factors;
    const auto source=chart_.streamRows(mapper_,
      [&](const NativeMappingPoseRows& group) {
        if(selected(group.poses))selection_.factors.insert(group.original);
        factors.push_back({"factor:"+group.original,nativeMappingFactor(group,chart_.offsets())});
        source_anchors_.emplace("factor:"+group.original,sourceAnchor(group.poses));
      },
      [&](const NativeMappingPointRows& group) {
        if(selected(group.poses))selection_.points.insert(group.id);
        auto reduced=reduceNativeMappingPoint(group,chart_.offsets());
        factors.push_back({"point:"+std::to_string(group.id),std::move(reduced.likelihood)});
        // Keep the conditional's coordinate list when moving its source rows.
        reduced.likelihood.columns=factors.back().rows.columns;
        points_.emplace(group.id,Conditional(std::move(reduced)));
        source_anchors_.emplace("point:"+std::to_string(group.id),sourceAnchor(group.poses,group.id));
      });
    if(selection_.factors.empty() && selection_.points.empty())
      throw std::invalid_argument("native_window_no_active_likelihoods");
    owner_->replace({},factors,{},nullptr,{},SubmapFactorization::Ordering::minimum_degree);
    originals_=source.originals+source.pose_factors;
    if(snapshot()->information->measurements.size()!=originals_)
      throw std::logic_error("native_window_original_ownership");
    linearization_state_=nativeMappingStateIdentity(mapper_);
    group_identities_=nativeMappingGroupIdentities(mapper_);
    configuration_identity_=nativeMappingConfigurationIdentity(mapper_);
  }
  // Prepare against the caller's staged native graph. The caller publishes this
  // owner only when matching, native state and all validation have succeeded.
  std::unique_ptr<NativeMappingWindow> prepareGrowth(const std::set<int64_t>& active,
      const std::function<bool()>& cancelled={}) const {
    auto check=[&]{if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};check();
    if(nativeMappingConfigurationIdentity(mapper_)!=configuration_identity_)
      throw std::invalid_argument("native_window_configuration_changed");
    if(active.empty())throw std::invalid_argument("native_window_empty");
    auto next=std::unique_ptr<NativeMappingWindow>(new NativeMappingWindow(*this,chart_.extended(mapper_),active));
    for(auto stamp:active)if(!next->chart_.offsets().count(stamp))
      throw std::invalid_argument("native_window_unknown_pose");
    auto layout=snapshot()->layout;std::set<int64_t> eliminated;
    for(const auto& block:layout.eliminated)eliminated.insert(block.begin(),block.end());
    layout.frontier.clear();
    for(auto stamp:next->chart_.poses()) {
      const auto columns=nativeMappingColumns({stamp},next->chart_.offsets());
      if(active.count(stamp)) {
        if(eliminated.count(columns.front()))throw std::invalid_argument("native_window_reactivated_pose");
        layout.frontier.insert(layout.frontier.end(),columns.begin(),columns.end());
      } else if(!eliminated.count(columns.front()))layout.eliminated.push_back(columns);
    }
    if(layout.frontier.size()>limits_.active_columns)throw std::runtime_error("native_window_separator_capacity");
    auto groups=nativeMappingGroupIdentities(mapper_);NativeMappingSelection selected,replacement;
    const NativeMappingSchedule schedule(mapper_,active,limits_.coordinates);
    auto add=[](NativeMappingSelection& selection,const std::string& id) {
      if(id.compare(0,7,"factor:")==0)selection.factors.insert(id.substr(7));
      else if(id.compare(0,6,"point:")==0)selection.points.insert(std::stoll(id.substr(6)));
      else throw std::logic_error("native_window_group_identity");
    };
    std::set<std::string> incoming;
    for(const auto& group:schedule.groups)if(group.active) {
      const auto id=groupId(group.source);incoming.insert(id);add(selected,id);
    }
    for(const auto& [id,digest]:groups) {
      const auto old=group_identities_.find(id);
      if(old==group_identities_.end() || old->second!=digest)incoming.insert(id);
    }
    std::vector<std::string> removed;std::set<std::string> outgoing,assigned;
    for(const auto& [id,rows]:snapshot()->factors)if(!groups.count(id) || incoming.count(id)) {
      removed.push_back(id);outgoing.insert(rows->originals.begin(),rows->originals.end());
    }
    for(const auto& id:incoming)add(replacement,id);
    for(auto it=next->points_.begin();it!=next->points_.end();) {
      if(!groups.count("point:"+std::to_string(it->first)))it=next->points_.erase(it);else ++it;
    }
    for(auto it=next->source_anchors_.begin();it!=next->source_anchors_.end();) {
      if(!groups.count(it->first))it=next->source_anchors_.erase(it);else ++it;
    }
    std::vector<SubmapFactorization::Factor> factors;
    next->chart_.streamRows(mapper_,
      [&](const NativeMappingPoseRows& row) {
        factors.push_back({"factor:"+row.original,nativeMappingFactor(row,next->chart_.offsets())});
        next->source_anchors_.insert_or_assign("factor:"+row.original,next->sourceAnchor(row.poses));
      },
      [&](const NativeMappingPointRows& row) {
        auto reduced=reduceNativeMappingPoint(row,next->chart_.offsets());
        factors.push_back({"point:"+std::to_string(row.id),reduced.likelihood});
        next->points_.insert_or_assign(row.id,Conditional(std::move(reduced)));
        next->source_anchors_.insert_or_assign("point:"+std::to_string(row.id),next->sourceAnchor(row.poses,row.id));
      },nullptr,&replacement);
    for(const auto& factor:factors)assigned.insert(factor.rows.originals.begin(),factor.rows.originals.end());
    std::vector<std::string> unassigned;
    std::set_difference(outgoing.begin(),outgoing.end(),assigned.begin(),assigned.end(),std::back_inserter(unassigned));
    check();next->owner_->replace(removed,factors,unassigned,&layout,cancelled,SubmapFactorization::Ordering::minimum_degree);
    next->selection_=std::move(selected);next->active_poses_=active;
    next->group_identities_=std::move(groups);next->source_identity_=nativeMappingSourceIdentity(mapper_);
    next->linearization_state_=nativeMappingStateIdentity(mapper_);
    next->originals_=next->snapshot()->information->measurements.size();
    if(next->numericBytes()>limits_.numeric_bytes)throw std::runtime_error("native_window_numeric_capacity");
    check();return next;
  }
  const NativeMappingChart& chart() const {return chart_;}
  const NativeMappingSelection& selection() const {return selection_;}
  SubmapFactorization::Snapshot snapshot() const {return owner_->snapshot();}
  size_t numericBytes() const {
    size_t out=snapshot()->numericBytes()+chart_.numericBytes();
    for(const auto& [id,anchor]:source_anchors_)out+=anchor->bytes();
    for(const auto& [id,point]:points_)out+=point.bytes();return out;
  }
  // Retire original groups crossing the reduced active set at their last
  // accepted linearization. Mapper variables and original observations are
  // unchanged; archived geometry retains its conditional correlations.
  void retire(const std::set<int64_t>& kept,const std::function<bool()>& cancelled={}) {
    if(nativeMappingSourceIdentity(mapper_)!=source_identity_)
      throw std::invalid_argument("native_window_retirement_source_changed");
    if(nativeMappingStateIdentity(mapper_)!=linearization_state_)
      throw std::invalid_argument("native_window_retirement_state_changed");
    auto plan=retirementPlan(kept);
    if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");
    commitRetirement(std::move(plan));
  }
  // Returned coordinates minimize the current hybrid linearization. No
  // covariance or readiness claim follows from this conditional mean.
  Eigen::VectorXd proposal() const {
    const auto state=snapshot();const auto solution=owner_->solve(state);
    Eigen::VectorXd out(6*chart_.poses().size());
    for(auto stamp:chart_.poses()) {
      const auto columns=nativeMappingColumns({stamp},chart_.offsets());
      out.segment<6>(chart_.offsets().at(stamp))=owner_->query(solution,columns,Eigen::Matrix<double,6,6>::Identity()).mean;
    }
    return out;
  }
  // One original position/world-z-rotation reference supplies four shared
  // latent errors. It is independent of the relative/gravity likelihoods.
  // Query up to eight distinct finite points together to retain cross-point
  // covariance; separate per-point priors would lose relative cancellation.
  // Zero reference weight leaves its gauge unbounded. No reference, damping
  // or measurement is inserted into the conditional mean optimizer.
  WorldLinearization worldLinearization() const {
    const auto state=snapshot();return {state,owner_->solve(state),chart_.coordinates(mapper_)};
  }
  WorldPoints worldInformation(const std::vector<int64_t>& ids) const {
    return worldInformation(worldLinearization(),ids);
  }
  // Share current-image geometry/maturity with full-graph output, but query
  // covariance from this hybrid owner. Rebuilding a full-graph covariance at
  // current poses would discard the retained historical linearization.
  MappingInformationResult mapInformation(double three_sigma_m) const {
    if(!std::isfinite(three_sigma_m) || three_sigma_m<=0)
      throw std::invalid_argument("mapping_world_uncertainty_limit");
    if(nativeMappingSourceIdentity(mapper_)!=source_identity_)
      throw std::invalid_argument("native_window_output_source_changed");
    struct Support {
      MappingInformationResult result;std::set<MappingObservation> observations;
      void addPoseFactor(const std::string&,const std::vector<int>&,const Eigen::MatrixXd&) {}
      void addPoint(const MappingPointRows& rows) {
        MappingPointCertificate point;point.geometry=mappingPointSupport(rows);
        if(!result.points.emplace(rows.id,std::move(point)).second)
          throw std::invalid_argument("mapping_point_rows");
        for(const auto& key:rows.observations)if(!observations.insert(key).second)
          throw std::invalid_argument("mapping_observation_reused");
      }
    } support;
    mappingGraphRows(mapper_,mapper_.calib,mapper_.config,chart_.reference(),support);
    auto& result=support.result;
    // The native likelihood also retains infinite image features. Count its
    // owned originals, not just those with finite export geometry.
    const auto pose_factors=mapper_.rel_pose_factors.size()+mapper_.roll_pitch_factors.size();
    result.observations=originals_-pose_factors;result.pose_factors=pose_factors+(reference_weight_>0);
    result.pose_nullity=-1; // Per-point observability is queried below.
    const auto context=worldLinearization();
    for(auto& [id,point]:result.points) {
      const auto query=worldInformation(context,{id});point.world.covariance=query.covariance;
      point.world.observed=query.observed;
      if(query.observed) {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(point.world.covariance);
        if(eigen.info()!=Eigen::Success || !point.world.covariance.allFinite())
          throw std::runtime_error("invalid_map_point_covariance");
        point.world.maximum_sigma_m=std::sqrt(std::max(0.,eigen.eigenvalues().maxCoeff()));
        point.world.observed=std::isfinite(point.world.maximum_sigma_m);
      }
      certifyMappingPoint(point,three_sigma_m);
    }
    return std::move(result);
  }
  // Reuse one solve across a map export. A changed owner rejects the context;
  // the caller's exclusive mapper ownership also applies while querying it.
  WorldPoints worldInformation(const WorldLinearization& linearization,const std::vector<int64_t>& ids) const {
    if(linearization.state!=snapshot())throw std::invalid_argument("native_window_stale_world_linearization");
    if(ids.empty() || ids.size()>8 || std::set<int64_t>(ids.begin(),ids.end()).size()!=ids.size())
      throw std::invalid_argument("native_window_world_points");
    if(!std::isfinite(reference_weight_) || reference_weight_<0)
      throw std::invalid_argument("native_window_reference_information");
    const auto& value=linearization.coordinates;
    const auto reference=mapper_.getFramePoses().at(chart_.reference()).getPose();
    const auto reference_derivative=chart_.derivative(chart_.reference(),value);
    // Only the selected points' conditional sources, hosts and shared
    // reference contribute. History addresses never size this dense matrix.
    std::set<int64_t> support;
    auto include_pose=[&](int64_t stamp) {
      const auto first=chart_.offsets().at(stamp);
      for(int i=0;i<6;++i)support.insert(first+i);
    };
    include_pose(chart_.reference());
    for(auto id:ids) {
      const auto found=points_.find(id);
      if(found==points_.end())throw std::invalid_argument("native_window_unknown_point");
      support.insert(found->second.columns.begin(),found->second.columns.end());
      include_pose(mapper_.lmdb.getLandmark(id).host_kf_id.frame_id);
    }
    const std::vector<int64_t> order(support.begin(),support.end());
    std::map<int64_t,int> local_column;
    for(size_t i=0;i<order.size();++i)local_column.emplace(order[i],int(i));
    const int reference_column=local_column.at(chart_.offsets().at(chart_.reference()));
    const int rows=3*ids.size(),columns=order.size();
    Eigen::MatrixXd derivative=Eigen::MatrixXd::Zero(rows,columns);
    Eigen::MatrixXd shared=Eigen::MatrixXd::Zero(rows,4);
    WorldPoints out;out.positions.resize(rows);
    out.covariance=Eigen::MatrixXd::Zero(rows,rows);
    out.unbounded_gram=Eigen::MatrixXd::Zero(rows,rows);bool observed=true;
    for(size_t i=0;i<ids.size();++i) {
      const auto found=points_.find(ids[i]);
      if(found==points_.end())throw std::invalid_argument("native_window_unknown_point");
      const auto& conditional=found->second;const auto& point=mapper_.lmdb.getLandmark(ids[i]);
      if(!std::isfinite(point.inv_dist) || point.inv_dist<=0 || !point.direction.allFinite())
        throw std::invalid_argument("native_window_nonfinite_world_point");
      const auto host_pose=mapper_.getFramePoses().at(point.host_kf_id.frame_id).getPose();
      const auto camera=host_pose*mapper_.calib.T_i_c.at(point.host_kf_id.cam_id);
      Eigen::Matrix<double,4,2> direction;
      const Eigen::Vector4d ray=basalt::StereographicParam<double>::unproject(point.direction,&direction);
      const Eigen::Vector3d camera_point=ray.head<3>()/point.inv_dist;
      const Eigen::Vector3d world=camera*camera_point;
      Eigen::Matrix3d local;local.leftCols<2>()=direction.topRows<3>()/point.inv_dist;
      local.col(2)=-camera_point/point.inv_dist;
      const Eigen::Matrix3d landmark=camera.rotationMatrix()*local;
      Eigen::Matrix<double,3,6> host;
      host.leftCols<3>().setIdentity();host.rightCols<3>()=-Sophus::SO3d::hat(world-host_pose.translation());
      const Eigen::MatrixXd propagated=landmark*conditional.backsub;
      for(size_t c=0;c<conditional.columns.size();++c)
        derivative.block<3,1>(3*i,local_column.at(conditional.columns[c]))+=propagated.col(c);
      derivative.block<3,6>(3*i,local_column.at(chart_.offsets().at(point.host_kf_id.frame_id)))+=
        host*chart_.derivative(point.host_kf_id.frame_id,value);
      out.positions.segment<3>(3*i)=world;
      shared.block<3,3>(3*i,0).setIdentity();
      shared.block<3,1>(3*i,3)=Eigen::Vector3d::UnitZ().cross(world-reference.translation());
      // The original VIO prior is on left-rotation z, not Euler heading.
      // At tilted reference attitudes its roll coordinate changes that z
      // component: transfer this shared coupling instead of adding a prior.
      derivative.block<3,6>(3*i,reference_column)-=
        shared.block<3,1>(3*i,3)*reference_derivative.row(5);
      const Eigen::MatrixXd noise=landmark*conditional.noise,free=landmark*conditional.unbounded;
      out.covariance.block<3,3>(3*i,3*i)=noise*noise.transpose();
      out.unbounded_gram.block<3,3>(3*i,3*i)=free*free.transpose();
      if(free.norm()>1e-8*std::max(1.,landmark.norm()*conditional.unbounded.norm()))observed=false;
    }
    const auto query=owner_->query(linearization.solution,order,derivative);
    out.covariance+=query.covariance;out.unbounded_gram+=query.unbounded_gram;
    if(reference_weight_>0)out.covariance.noalias()+=shared*shared.transpose()/reference_weight_;
    else {out.unbounded_gram.noalias()+=shared*shared.transpose();observed=false;}
    if(!out.positions.allFinite() || !out.covariance.allFinite() || !out.unbounded_gram.allFinite())
      throw std::runtime_error("native_window_nonfinite_world_information");
    out.observed=observed && query.observed;return out;
  }
  Step step(const std::function<bool()>& cancelled={}) {
    auto check=[&] {if(cancelled && cancelled())throw std::runtime_error("native_window_cancelled");};
    check();const auto before_rows=nativeMappingRows(mapper_,&selection_,limits_.coordinates);
    const auto current=chart_.coordinates(mapper_);const auto proposed=proposal();
    Step result;result.before=objective(current,before_rows);result.after=result.before;
    result.active_originals=before_rows.originals+before_rows.factors.size();
    Backup backup(mapper_);const auto old_snapshot=snapshot();
    std::map<int64_t,Eigen::Vector3d> point_proposals;
    for(const auto& [id,reduced]:points_)point_proposals.emplace(id,pointValue(id,reduced,proposed));
    for(int trial=0;trial<12;++trial) {
      check();backup.restore();result.trials=trial+1;const double scale=std::ldexp(1.,-trial);
      Eigen::VectorXd value=current+scale*(proposed-current);
      conditionArchived(value,*old_snapshot->base);chart_.apply(mapper_,value);
      for(const auto& [id,reduced]:points_) {
        Eigen::Vector3d next;
        if(selection_.points.count(id))next=backup.points.at(id)+scale*(point_proposals.at(id)-backup.points.at(id));
        else next=pointValue(id,reduced,value);
        auto& point=mapper_.lmdb.getLandmark(id);point.direction=next.head<2>();point.inv_dist=std::max(0.,next.z());
      }
      const auto trial_rows=nativeMappingRows(mapper_,&selection_,limits_.coordinates);
      bool visible=true;
      for(size_t i=0;i<before_rows.points.size();++i)for(size_t k=0;k<before_rows.points[i].valid.size();++k)
        if(before_rows.points[i].valid[k] && !trial_rows.points.at(i).valid.at(k))visible=false;
      const double cost=objective(value,trial_rows);
      if(!visible || !std::isfinite(cost) || cost>result.before)continue;
      replaceActive(trial_rows,check);
      backup.committed=true;result.accepted=true;result.after=cost;return result;
    }
    return result;
  }
 private:
  static size_t rowBudget(basalt::NfrMapper& mapper,const NativeMappingChart& chart,
      const std::set<int64_t>& active,const SubmapInformation::Limits& limits) {
    const NativeMappingSchedule schedule(mapper,active,limits.coordinates);size_t reserved=chart.numericBytes();
    for(const auto& group:schedule.groups) {
      reserved+=sizeof(double)*(7*group.poses.size()+
        (group.source.kind==NativeMappingRowSource::landmark?3:0));
      if(group.source.kind==NativeMappingRowSource::landmark)
        reserved+=sizeof(double)*(3*6*group.poses.size()+3+9);
    }
    if(reserved>=limits.numeric_bytes)throw std::runtime_error("native_window_numeric_capacity");
    return limits.numeric_bytes-reserved;
  }
  NativeMappingWindow(basalt::NfrMapper& mapper,NativeMappingChart chart,
      const std::set<int64_t>& active,SubmapInformation::Limits limits,
      const SubmapFactorization::Layout* saved_layout=nullptr,
      const std::vector<std::string>& unassigned={},std::shared_ptr<SubmapRowStore> row_store={})
      :mapper_(mapper),chart_(std::move(chart)),limits_(limits),active_poses_(active),
       source_identity_(nativeMappingSourceIdentity(mapper)),
       linearization_state_(nativeMappingStateIdentity(mapper)),
       reference_weight_(mapper.config.vio_init_pose_weight) {
    if(active.empty())throw std::invalid_argument("native_window_empty");
    for(auto stamp:active)if(!chart_.offsets().count(stamp))throw std::invalid_argument("native_window_unknown_pose");
    auto owner_limits=limits;owner_limits.numeric_bytes=rowBudget(mapper,chart_,active,limits);
    if(!owner_limits.temporary_columns)owner_limits.temporary_columns=kMaxJointColumns;
    SubmapFactorization::Layout layout;
    if(saved_layout)layout=*saved_layout;
    else for(auto stamp:chart_.poses()) {
      const auto columns=nativeMappingColumns({stamp},chart_.offsets());
      if(active.count(stamp))layout.frontier.insert(layout.frontier.end(),columns.begin(),columns.end());
      else layout.eliminated.push_back(columns);
    }
    if(layout.frontier.size()>limits.active_columns)throw std::runtime_error("native_window_separator_capacity");
    owner_=std::make_unique<SubmapFactorization>(layout,owner_limits,unassigned,std::move(row_store));
    group_identities_=nativeMappingGroupIdentities(mapper);
    configuration_identity_=nativeMappingConfigurationIdentity(mapper);
  }
  NativeMappingWindow(const NativeMappingWindow& source,NativeMappingChart chart,const std::set<int64_t>& active)
      :mapper_(source.mapper_),chart_(std::move(chart)),limits_(source.limits_),active_poses_(source.active_poses_),
       source_identity_(source.source_identity_),linearization_state_(source.linearization_state_),selection_(source.selection_),
       owner_(source.owner_->fork(source.snapshot(),rowBudget(mapper_,chart_,active,limits_))),
       points_(source.points_),originals_(source.originals_),reference_weight_(source.reference_weight_),
       group_identities_(source.group_identities_),configuration_identity_(source.configuration_identity_),
       source_anchors_(source.source_anchors_) {}
  std::shared_ptr<const NativeMappingSourceAnchor> sourceAnchor(const std::vector<int64_t>& poses,int64_t point=-1) const {
    auto result=std::make_shared<NativeMappingSourceAnchor>();
    for(auto stamp:poses)result->poses.emplace(stamp,mapper_.getFramePoses().at(stamp).getPose());
    if(point>=0) {
      const auto& value=mapper_.lmdb.getLandmark(point);result->has_point=true;
      result->point={value.direction.x(),value.direction.y(),value.inv_dist};
    }
    return result;
  }
  std::string groupId(const NativeMappingRowSource& source) const {
    if(source.kind==NativeMappingRowSource::landmark)return "point:"+std::to_string(source.id);
    if(source.kind==NativeMappingRowSource::relative) {
      const auto& f=mapper_.rel_pose_factors.at(source.id);
      return "factor:relative:"+std::to_string(f.t_i_ns)+":"+std::to_string(f.t_j_ns);
    }
    return "factor:roll-pitch:"+std::to_string(mapper_.roll_pitch_factors.at(source.id).t_ns);
  }
  struct Retirement {
    std::set<int64_t> kept;
    Eigen::aligned_map<int64_t,Sophus::SE3d> poses;
    Eigen::aligned_map<int64_t,Eigen::Vector3d> points;
    size_t bytes() const {return sizeof(double)*(7*poses.size()+3*points.size());}
  };
  struct RetirementPlan {
    std::set<int64_t> kept,support;
    NativeMappingSelection selected,next;
    std::vector<int64_t> columns;
    std::vector<std::string> originals;
  };
  RetirementPlan retirementPlan(const std::set<int64_t>& kept) const {
    if(kept.empty() || kept.size()>=active_poses_.size() ||
        !std::includes(active_poses_.begin(),active_poses_.end(),kept.begin(),kept.end()))
      throw std::invalid_argument("native_window_retirement_subset");
    RetirementPlan plan;plan.kept=kept;plan.next=selection_;std::set<int64_t> columns;size_t remaining=0;
    auto outside=[&](const std::vector<int64_t>& columns) {
      return std::any_of(columns.begin(),columns.end(),[&](auto c){return !kept.count(chart_.poses().at(c/6));});
    };
    for(const auto& group:snapshot()->active) {
      if(outside(group->columns)) {
        plan.originals.insert(plan.originals.end(),group->originals.begin(),group->originals.end());
        for(auto c:group->columns)plan.support.insert(chart_.poses().at(c/6));
        for(const auto& key:group->originals)if(plan.next.factors.erase(key))plan.selected.factors.insert(key);
      } else {++remaining;columns.insert(group->columns.begin(),group->columns.end());}
    }
    if(!remaining)throw std::invalid_argument("native_window_no_active_likelihoods");
    for(auto id:selection_.points)if(outside(points_.at(id).columns)) {
      plan.selected.points.insert(id);plan.next.points.erase(id);
    }
    plan.columns.assign(columns.begin(),columns.end());return plan;
  }
  void commitRetirement(RetirementPlan plan) {
    auto layout=snapshot()->layout;
    for(auto stamp:chart_.poses())if(active_poses_.count(stamp) && !plan.kept.count(stamp))
      layout.eliminated.push_back(nativeMappingColumns({stamp},chart_.offsets()));
    layout.frontier=nativeMappingColumns(std::vector<int64_t>(plan.kept.begin(),plan.kept.end()),chart_.offsets());
    owner_->replace({}, {}, {}, &layout);
    // All allocations/validation precede publication of the numeric owner.
    selection_.points.swap(plan.next.points);selection_.factors.swap(plan.next.factors);
    active_poses_.swap(plan.kept);
  }
  struct Conditional {
    std::vector<int64_t> columns;Eigen::MatrixXd backsub,noise,unbounded;Eigen::VectorXd offset;
    explicit Conditional(NativeMappingPointReduction value):columns(std::move(value.likelihood.columns)),
      backsub(std::move(value.backsub)),noise(std::move(value.noise)),
      unbounded(std::move(value.unbounded)),offset(std::move(value.offset)) {}
    size_t bytes() const {return sizeof(double)*(backsub.size()+noise.size()+unbounded.size()+offset.size());}
  };
  struct Backup {
    basalt::NfrMapper& mapper;Eigen::aligned_map<int64_t,basalt::PoseStateWithLin<double>> poses;
    std::map<int64_t,Eigen::Vector3d> points;bool committed=false;
    explicit Backup(basalt::NfrMapper& source):mapper(source),poses(source.getFramePoses()) {
      for(const auto& [id,point]:source.lmdb.getLandmarks())points.emplace(id,Eigen::Vector3d(point.direction.x(),point.direction.y(),point.inv_dist));
    }
    void restore() {mapper.getFramePoses()=poses;for(const auto& [id,value]:points) {
      auto& point=mapper.lmdb.getLandmark(id);point.direction=value.head<2>();point.inv_dist=value.z();
    }}
    ~Backup(){if(!committed)restore();}
  };
  // The same replacement transaction is used after a nonlinear step and after
  // restoring historical anchors. Historical rows are never relinearized at
  // the latest output state merely because the window was saved and reopened.
  void replaceActive(const NativeMappingRows& rows,const std::function<void()>& check) {
    const auto state=nativeMappingStateIdentity(mapper_);std::vector<std::string> remove;
    for(const auto& group:snapshot()->active)remove.insert(remove.end(),group->originals.begin(),group->originals.end());
    replaceSelected(rows,remove,check);linearization_state_=state;
  }
  void replaceSelected(const NativeMappingRows& rows,const std::vector<std::string>& remove,
      const std::function<void()>& check) {
    const auto anchored=chart_.rows(mapper_,rows);
    std::vector<SubmapFactorization::Factor> replacement;std::map<int64_t,Conditional> new_points;
    std::map<std::string,std::shared_ptr<const NativeMappingSourceAnchor>> anchors;
    for(const auto& group:anchored.factors) {
      const auto id="factor:"+group.original;replacement.push_back({id,nativeMappingFactor(group,chart_.offsets())});
      anchors.emplace(id,sourceAnchor(group.poses));
    }
    for(const auto& group:anchored.points) {
      auto reduced=reduceNativeMappingPoint(group,chart_.offsets());replacement.push_back({"point:"+std::to_string(group.id),reduced.likelihood});
      new_points.emplace(group.id,Conditional(std::move(reduced)));
      anchors.emplace("point:"+std::to_string(group.id),sourceAnchor(group.poses,group.id));
    }
    std::set<std::string> ids;
    for(const auto& key:remove)ids.insert(snapshot()->assignments.at(key));
    check();owner_->replace(std::vector<std::string>(ids.begin(),ids.end()),replacement);
    for(auto& [id,reduced]:new_points)points_.at(id)=std::move(reduced);
    for(auto& [id,anchor]:anchors)source_anchors_.at(id)=std::move(anchor);
  }
  static void conditionArchived(Eigen::VectorXd& value,const SubmapInformation::Data& base) {
    for(auto it=base.nodes.rbegin();it!=base.nodes.rend();++it) {
      const auto& node=**it;Eigen::VectorXd kept(node.kept.size());
      for(size_t i=0;i<node.kept.size();++i)kept[i]=value[node.kept[i]];
      const Eigen::VectorXd removed=node.offset+node.regression*kept;
      for(size_t i=0;i<node.removed.size();++i)value[node.removed[i]]=removed[i];
    }
  }
  Eigen::Vector3d pointValue(int64_t id,const Conditional& reduced,const Eigen::VectorXd& value) const {
    Eigen::VectorXd local(reduced.columns.size());
    for(size_t i=0;i<reduced.columns.size();++i)local[i]=value[reduced.columns[i]];
    return chart_.point(id)+reduced.offset+reduced.backsub*local;
  }
  double objective(const Eigen::VectorXd& value,const NativeMappingRows& active) const {
    const auto& base=*snapshot()->base;Eigen::VectorXd local(base.order.size());
    for(size_t i=0;i<base.order.size();++i)local[i]=value[base.order[i]];
    return .5*(base.jacobian*local+base.residual).squaredNorm()+active.vision_cost+.5*active.pose_cost;
  }
  basalt::NfrMapper& mapper_;NativeMappingChart chart_;SubmapInformation::Limits limits_;
  std::set<int64_t> active_poses_;InputDigest::Value source_identity_,linearization_state_;
  NativeMappingSelection selection_;std::unique_ptr<SubmapFactorization> owner_;
  std::map<int64_t,Conditional> points_;size_t originals_=0;double reference_weight_;
  std::map<std::string,InputDigest::Value> group_identities_;InputDigest::Value configuration_identity_{};
  std::map<std::string,std::shared_ptr<const NativeMappingSourceAnchor>> source_anchors_;
};
} // namespace posim_slam

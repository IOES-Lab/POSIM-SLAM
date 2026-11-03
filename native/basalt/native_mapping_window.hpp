// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "native_mapping_linearization.hpp"
#include "native_mapping_identity.hpp"
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
  NativeMappingChart(basalt::NfrMapper& mapper,int64_t reference):reference_(reference) {
    for(const auto& [stamp,state]:mapper.getFramePoses()) {
      if(state.isLinearized() || !state.getPose().matrix().allFinite())
        throw std::invalid_argument("native_window_current_pose_required");
      anchors_.emplace(stamp,state.getPose());poses_.push_back(stamp);
    }
    offsets_=nativeMappingPoseOffsets(poses_);
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
    auto transform=[&](const std::vector<int64_t>& poses,Eigen::MatrixXd& jacobian,Eigen::VectorXd& residual) {
      for(size_t i=0;i<poses.size();++i) {
        jacobian.middleCols<6>(6*i)=(jacobian.middleCols<6>(6*i)*derivative(poses[i],value)).eval();
        residual.noalias()-=jacobian.middleCols<6>(6*i)*value.segment<6>(offsets_.at(poses[i]));
      }
    };
    for(auto& factor:source.factors)transform(factor.poses,factor.jacobian,factor.residual);
    for(auto& group:source.points) {
      const auto& point=mapper.lmdb.getLandmark(group.id);
      if(group.host!=hosts_.at(group.id))throw std::invalid_argument("native_window_landmark_host_changed");
      const Eigen::Vector3d delta=Eigen::Vector3d(point.direction.x(),point.direction.y(),point.inv_dist)-points_.at(group.id);
      transform(group.poses,group.pose,group.residual);group.residual.noalias()-=group.landmark*delta;
    }
    return source;
  }
 private:
  static Eigen::Vector3d angles(const Sophus::SE3d& pose) {
    const auto r=pose.rotationMatrix();const double horizontal=r.col(0).head<2>().norm();
    if(horizontal<=1e-6)throw std::invalid_argument("native_window_reference_heading_undefined");
    return {std::atan2(r(2,1),r(2,2)),std::atan2(-r(2,0),horizontal),std::atan2(r(1,0),r(0,0))};
  }
  int64_t reference_;Eigen::Vector3d reference_angles_;
  std::vector<int64_t> poses_;std::map<int64_t,int> offsets_;
  Eigen::aligned_map<int64_t,Sophus::SE3d> anchors_;
  std::map<int64_t,Eigen::Vector3d> points_;std::map<int64_t,basalt::TimeCamId> hosts_;
};

// Nonlinear owner for one explicitly selected active window. Original factor
// groups wholly inside it remain replaceable. Groups crossing its boundary
// enter a sealed Gaussian conditional exactly once, retaining back-substitution.
// A window whose separator exceeds the supplied capacity is rejected intact.
// The caller exclusively owns mapper during this object's lifetime. Graph
// growth, track reassociation, rehosting and sealed-factor rehydration require
// a new owner built from the original graph; automatic rollover is separate.
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
    SubmapLinearization::Snapshot state;
    SubmapInformation::Solution solution;
    Eigen::VectorXd coordinates;
    WorldLinearization(SubmapLinearization::Snapshot snapshot,SubmapInformation::Solution solved,Eigen::VectorXd value)
        :state(std::move(snapshot)),solution(std::move(solved)),coordinates(std::move(value)) {}
   public:
    WorldLinearization(const WorldLinearization&)=default;
    WorldLinearization(WorldLinearization&&)=default;
  };
  NativeMappingWindow(basalt::NfrMapper& mapper,int64_t reference,
      const std::set<int64_t>& active_poses,SubmapInformation::Limits limits={})
      :mapper_(mapper),chart_(mapper,reference),limits_(limits),active_poses_(active_poses),
       source_identity_(nativeMappingSourceIdentity(mapper)),
       reference_weight_(mapper.config.vio_init_pose_weight) {
    if(active_poses.empty())throw std::invalid_argument("native_window_empty");
    for(auto stamp:active_poses)if(!chart_.offsets().count(stamp))throw std::invalid_argument("native_window_unknown_pose");
    const auto original=nativeMappingRows(mapper_);const auto source=chart_.rows(mapper_,original);
    auto contained=[&](const std::vector<int64_t>& poses) {
      for(auto stamp:poses)if(!active_poses.count(stamp))return false;return true;
    };
    std::vector<SubmapLinearization::Rows> history,active;
    std::set<int64_t> frontier;
    auto retain=[&](SubmapLinearization::Rows row,bool selected) {
      if(selected) {frontier.insert(row.columns.begin(),row.columns.end());active.push_back(std::move(row));}
      else history.push_back(std::move(row));
    };
    for(const auto& group:source.factors) {
      const bool selected=contained(group.poses);if(selected)selection_.factors.insert(group.original);
      retain(nativeMappingFactor(group,chart_.offsets()),selected);
    }
    for(const auto& group:source.points) {
      const bool selected=contained(group.poses);if(selected)selection_.points.insert(group.id);
      auto reduced=reduceNativeMappingPoint(group,chart_.offsets());retain(reduced.likelihood,selected);
      points_.emplace(group.id,Conditional(std::move(reduced)));
    }
    if(active.empty())throw std::invalid_argument("native_window_no_active_likelihoods");
    if(frontier.size()>limits.active_columns)throw std::runtime_error("native_window_separator_capacity");
    // Only the conditional is retained per landmark, not a duplicate copy of
    // every archived likelihood. Finite noise and unbounded directions have
    // three columns in total, including when the conditional rank changes.
    size_t reserved=chart_.numericBytes();
    for(const auto& [id,point]:points_)reserved+=sizeof(double)*(point.backsub.size()+3+9);
    if(reserved>=limits.numeric_bytes)throw std::runtime_error("native_window_numeric_capacity");
    auto owner_limits=limits;owner_limits.numeric_bytes-=reserved;
    auto initial_limits=owner_limits;initial_limits.active_columns=6*source.poses.size();
    SubmapInformation initial(initial_limits);
    appendBatches(initial,history,source.poses.size()*6);
    std::vector<int64_t> kept;
    for(auto column:initial.snapshot()->order)if(frontier.count(column))kept.push_back(column);
    if(kept.size()<initial.snapshot()->order.size())initial.seal(kept);
    owner_=std::make_unique<SubmapLinearization>(initial,initial.snapshot(),owner_limits);
    owner_->append(active);
    if(owner_->snapshot()->information->known.size()!=6*source.poses.size())
      throw std::invalid_argument("native_window_unowned_pose");
    originals_=source.originals+source.factors.size();
    if(owner_->snapshot()->information->measurements.size()!=originals_)
      throw std::logic_error("native_window_original_ownership");
  }
  const NativeMappingChart& chart() const {return chart_;}
  const NativeMappingSelection& selection() const {return selection_;}
  SubmapLinearization::Snapshot snapshot() const {return owner_->snapshot();}
  size_t numericBytes() const {
    size_t out=snapshot()->numericBytes()+chart_.numericBytes();
    for(const auto& [id,point]:points_)out+=point.bytes();return out;
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
    const int reference_column=chart_.offsets().at(chart_.reference());
    const int rows=3*ids.size(),columns=value.size();
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
        derivative.block<3,1>(3*i,conditional.columns[c])+=propagated.col(c);
      derivative.block<3,6>(3*i,chart_.offsets().at(point.host_kf_id.frame_id))+=
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
    std::vector<int64_t> order(columns);std::iota(order.begin(),order.end(),0);
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
    check();const auto before_rows=nativeMappingRows(mapper_,&selection_);
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
      const auto trial_rows=nativeMappingRows(mapper_,&selection_);
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
    const auto anchored=chart_.rows(mapper_,rows);
    std::vector<SubmapLinearization::Rows> replacement;std::map<int64_t,Conditional> new_points;
    for(const auto& group:anchored.factors)replacement.push_back(nativeMappingFactor(group,chart_.offsets()));
    for(const auto& group:anchored.points) {
      auto reduced=reduceNativeMappingPoint(group,chart_.offsets());replacement.push_back(reduced.likelihood);
      new_points.emplace(group.id,Conditional(std::move(reduced)));
    }
    std::vector<std::string> remove;
    for(const auto& group:snapshot()->active)remove.insert(remove.end(),group->originals.begin(),group->originals.end());
    check();owner_->replace(remove,replacement);
    for(auto& [id,reduced]:new_points)points_.at(id)=std::move(reduced);
  }
  static void appendBatches(SubmapInformation& owner,const std::vector<SubmapLinearization::Rows>& groups,int columns) {
    std::vector<int64_t> order(columns);std::iota(order.begin(),order.end(),0);
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(4096,columns);Eigen::VectorXd r(4096);int used=0;
    std::vector<std::string> keys;
    auto flush=[&] {if(!keys.empty()){owner.append(keys,order,j.topRows(used),r.head(used));keys.clear();used=0;j.setZero();}};
    for(const auto& group:groups) {
      if(used+group.jacobian.rows()>j.rows())flush();
      for(size_t c=0;c<group.columns.size();++c)j.block(used,group.columns[c],group.jacobian.rows(),1)=group.jacobian.col(c);
      r.segment(used,group.residual.size())=group.residual;used+=group.residual.size();
      keys.insert(keys.end(),group.originals.begin(),group.originals.end());
    }
    flush();
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
  std::set<int64_t> active_poses_;InputDigest::Value source_identity_;
  NativeMappingSelection selection_;std::unique_ptr<SubmapLinearization> owner_;
  std::map<int64_t,Conditional> points_;size_t originals_=0;double reference_weight_;
};
} // namespace posim_slam

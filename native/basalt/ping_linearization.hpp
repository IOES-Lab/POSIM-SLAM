// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_group.hpp"
#include <basalt/linearization/linearization_abs_qr.hpp>
#include <functional>
#include <map>

namespace posim_slam {
struct JointPingGroupSpec {
  int plane_id;
  std::set<basalt::KeypointId> landmark_ids;
  // Build geometry/rows at the current state with the caller's FEJ convention.
  // Membership and observed measurement identities stay fixed during a trial.
  std::function<NativePingGroup(const std::map<basalt::KeypointId,basalt::LandmarkBlock<double>*>&)> build;
  std::set<int64_t> observation_pose_stamps;
  // Explicit local-planarity hypothesis supported by raw range observations.
  bool range_only=false;

};

struct PingRetirementPlan {
  std::vector<JointPingGroupSpec> groups;
  std::unordered_set<basalt::KeypointId> landmarks;
  std::set<int64_t> required_frames,observations;
  std::set<int> planes;
};

// Read-only closure before constructing the marginal system. Required frames
// include *all* visual targets, including states newer than Basalt's usual
// marginal boundary. The owner must include them in its order and retain them
// in the new prior rather than dropping rows whose columns are missing.
inline PingRetirementPlan planPingRetirement(const basalt::BundleAdjustmentBase<double>& owner,
    const std::vector<JointPingGroupSpec>& groups,const std::set<int64_t>& removed_poses,
    const std::set<int64_t>& removed_hosts,const std::unordered_set<basalt::KeypointId>& lost) {
  if(owner.posim_ping_trial_active)throw std::logic_error("ping_retirement_during_trial");
  if(groups.size()>8)throw std::invalid_argument("invalid_joint_group_count");
  PingRetirementPlan plan;plan.landmarks=lost;
  std::set<int> plane_ids;std::set<basalt::KeypointId> owned;std::set<int64_t> observations;
  for(const auto& group:groups) {
    if(!group.build || (group.range_only ? !group.landmark_ids.empty() : group.landmark_ids.size()<3) || group.landmark_ids.size()>64 ||
       group.observation_pose_stamps.empty() || group.observation_pose_stamps.size()>64 ||
       !plane_ids.insert(group.plane_id).second)
      throw std::invalid_argument("invalid_ping_retirement_group");
    owner.posim_ping_planes.at(group.plane_id);
    bool retire=false;std::set<int64_t> dependencies=group.observation_pose_stamps;
    for(int64_t stamp:group.observation_pose_stamps) {
      if(stamp<=0 || !observations.insert(stamp).second)throw std::invalid_argument("duplicate_joint_ping_owner");
      retire=retire || removed_poses.count(stamp) || removed_hosts.count(stamp);
    }
    for(auto id:group.landmark_ids) {
      if(!owned.insert(id).second || !owner.lmdb.landmarkExists(id))
        throw std::invalid_argument("invalid_joint_landmark_owner");
      const auto& point=owner.lmdb.getLandmark(id);
      dependencies.insert(point.host_kf_id.frame_id);
      retire=retire || lost.count(id) || removed_hosts.count(point.host_kf_id.frame_id) || removed_poses.count(point.host_kf_id.frame_id);
      for(const auto& [target,pixel]:point.obs) {
        dependencies.insert(target.frame_id);
        retire=retire || removed_poses.count(target.frame_id) || removed_hosts.count(target.frame_id);
      }
    }
    for(int64_t stamp:dependencies)
      if(!owner.frame_states.count(stamp) && !owner.frame_poses.count(stamp))
        throw std::invalid_argument("missing_ping_dependency_pose");
    if(retire) {
      plan.groups.push_back(group);plan.planes.insert(group.plane_id);
      plan.landmarks.insert(group.landmark_ids.begin(),group.landmark_ids.end());
      plan.observations.insert(group.observation_pose_stamps.begin(),group.observation_pose_stamps.end());
      plan.required_frames.insert(dependencies.begin(),dependencies.end());
    }
  }
  return plan;
}

// A real Basalt LinearizationBase implementation. Selected visual blocks are
// moved out of independent QR, so their rows enter exactly one coupled group.
// Caller owns group association, nonlinear auxiliary cost, marginalization and
// a JointPingTrial transaction around backSubstitute + state updates.
// Marginalization must pass the parent's retiring-host/landmark selection, with
// every landmark of each retiring coupled group included. Missing group members
// reject construction; retained independent visual rows must not enter a prior.
class PingLinearization final : public basalt::LinearizationAbsQR<double,6> {
  using Parent=basalt::LinearizationAbsQR<double,6>;
  struct Group {
    JointPingGroupSpec spec;
    std::map<basalt::KeypointId,LandmarkBlockPtr> owned;
    NativePingGroup raw;
    JointPatchReduction reduced;
  };
 public:
  PingLinearization(basalt::BundleAdjustmentBase<double>* owner,
      const basalt::AbsOrderMap& order,const Options& options,
      std::vector<JointPingGroupSpec> specs,
      const basalt::MargLinData<double>* prior=nullptr,
      const basalt::ImuLinData<double>* imu=nullptr,
      const std::set<basalt::FrameId>* retiring_hosts=nullptr,
      const std::unordered_set<basalt::KeypointId>* retiring_landmarks=nullptr,
      int64_t last_state_to_marg=std::numeric_limits<int64_t>::max())
      :Parent(owner,order,options,prior,imu,retiring_hosts,retiring_landmarks,last_state_to_marg),owner_(owner) {
    if(specs.empty() || specs.size()>8)throw std::invalid_argument("invalid_joint_group_count");
    std::map<basalt::KeypointId,size_t> membership;std::set<int> planes;
    for(auto& spec:specs) {
      if(!spec.build || (spec.range_only ? !spec.landmark_ids.empty() : spec.landmark_ids.size()<3) || spec.landmark_ids.size()>64 ||
         !planes.insert(spec.plane_id).second)throw std::invalid_argument("invalid_joint_group_membership");
      owner_->posim_ping_planes.at(spec.plane_id);
      for(auto id:spec.landmark_ids)
        if(!membership.emplace(id,groups_.size()).second)throw std::invalid_argument("duplicate_joint_landmark_owner");
      Group group;group.spec=std::move(spec);groups_.push_back(std::move(group));
    }
    std::vector<LandmarkBlockPtr> independent;std::vector<basalt::KeypointId> independent_ids;
    for(size_t i=0;i<landmark_ids.size();++i) {
      auto found=membership.find(landmark_ids[i]);
      if(found==membership.end()) {
        independent_ids.push_back(landmark_ids[i]);independent.push_back(std::move(landmark_blocks[i]));
      }else groups_[found->second].owned.emplace(landmark_ids[i],std::move(landmark_blocks[i]));
    }
    for(const auto& group:groups_)
      if(group.owned.size()!=group.spec.landmark_ids.size())throw std::invalid_argument("missing_joint_landmark");
    landmark_blocks=std::move(independent);landmark_ids=std::move(independent_ids);
    landmark_block_idx.clear();num_rows_Q2r=0;
    for(const auto& block:landmark_blocks) {
      landmark_block_idx.push_back(num_rows_Q2r);num_rows_Q2r+=block->numQ2rows();
    }
  }

  double linearizeProblem(bool* valid=nullptr) override {
    phase_=Phase::Empty;bool all_valid=true;
    double cost=Parent::linearizeProblem(&all_valid);std::set<int64_t> observations;
    if(!std::isfinite(cost))throw std::runtime_error("nonfinite_joint_base_cost");
    for(auto& group:groups_) {
      std::map<basalt::KeypointId,basalt::LandmarkBlock<double>*> blocks;std::set<basalt::LandmarkBlock<double>*> expected;
      for(auto& [id,block]:group.owned) {
        cost+=block->linearizeLandmark();all_valid=all_valid && !block->isNumericalFailure();
        blocks.emplace(id,block.get());expected.insert(block.get());
      }
      group.raw=group.spec.build(blocks);
      const auto& reference=owner_->posim_ping_planes.at(group.spec.plane_id).reference;
      if(!group.raw.reference || (group.raw.reference->normal-reference.normal).norm()>1e-12 ||
         std::abs(group.raw.reference->distance-reference.distance)>1e-12)
        throw std::invalid_argument("joint_plane_reference_mismatch");
      const std::set<basalt::LandmarkBlock<double>*> actual(group.raw.blocks.begin(),group.raw.blocks.end());
      if(actual!=expected || actual.size()!=group.raw.blocks.size() || group.raw.observation_stamps.empty() ||
         group.raw.jacobian.cols()!=Eigen::Index(aom.total_size+3+3*expected.size()))
        throw std::invalid_argument("joint_builder_changed_membership");
      if(!group.spec.observation_pose_stamps.empty() &&
         std::set<int64_t>(group.raw.observation_stamps.begin(),group.raw.observation_stamps.end())!=group.spec.observation_pose_stamps)
        throw std::invalid_argument("joint_builder_changed_observation_dependencies");
      for(int64_t stamp:group.raw.observation_stamps)
        if(!observations.insert(stamp).second)throw std::invalid_argument("duplicate_joint_ping_owner");
      cost+=group.raw.auxiliary_cost;
    }
    if(!std::isfinite(cost))throw std::runtime_error("nonfinite_joint_cost");
    if(valid)*valid=all_valid;
    revision_=owner_->posim_ping_state_revision;
    if(all_valid)phase_=Phase::Raw;
    return cost;
  }

  void performQR() override { performQR(0.); }

  // Optimization reduces the raw group once with its trial damping. The
  // polymorphic no-argument path remains undamped for marginalization.
  void performQR(double nuisance_damping) {
    if(phase_!=Phase::Raw || revision_!=owner_->posim_ping_state_revision)throw std::logic_error("joint_qr_requires_linearization");
    if(!std::isfinite(nuisance_damping) || nuisance_damping<0. || nuisance_damping>1e12)
      throw std::invalid_argument("invalid_joint_nuisance_damping");
    Parent::performQR();
    for(auto& group:groups_)
      group.reduced=eliminateJointPatch(group.raw.jacobian,group.raw.residual,aom.total_size,nuisance_damping);
    phase_=Phase::Reduced;
  }

  void get_dense_H_b(MatX& h,VecX& b) const override {
    requireReduced();Parent::get_dense_H_b(h,b);
    for(const auto& group:groups_) {
      h.noalias()+=group.reduced.jacobian.transpose()*group.reduced.jacobian;
      b.noalias()+=group.reduced.jacobian.transpose()*group.reduced.residual;
    }
  }
  void get_dense_Q2Jp_Q2r(MatX& j,VecX& r) const override {
    requireReduced();Parent::get_dense_Q2Jp_Q2r(j,r);
    for(const auto& group:groups_) {
      const auto rows=group.reduced.jacobian.rows(),start=j.rows();
      j.conservativeResize(start+rows,Eigen::NoChange);r.conservativeResize(start+rows);
      j.bottomRows(rows)=group.reduced.jacobian;r.tail(rows)=group.reduced.residual;
    }
  }

  double backSubstitute(const VecX& pose_delta) override {
    requireReduced();
    if(!owner_->posim_ping_trial_active)throw std::logic_error("joint_trial_required");
    if(pose_delta.size()!=Eigen::Index(aom.total_size) || !pose_delta.allFinite())
      throw std::invalid_argument("invalid_joint_pose_increment");
    std::vector<VecX> nuisance;
    for(const auto& group:groups_) {
      nuisance.push_back(group.reduced.backSubstitute(pose_delta));
      const auto& delta=nuisance.back();
      if(!delta.allFinite())throw std::invalid_argument("invalid_joint_nuisance_increment");
      for(size_t i=0;i<group.raw.blocks.size();++i)
        if(!group.raw.blocks[i]->jointIncrementValid(delta.segment<3>(3+3*i)))
          throw std::invalid_argument("invalid_joint_landmark_increment");
    }
    // Blocks cache the trial's state after applying an increment. A restored
    // owner must relinearize too; stale matrices never survive a trial retry.
    phase_=Phase::Applied;
    double change=Parent::backSubstitute(pose_delta);
    for(size_t i=0;i<groups_.size();++i) {
      auto& group=groups_[i];VecX delta(pose_delta.size()+nuisance[i].size());delta<<pose_delta,nuisance[i];
      change+=.5*(group.raw.residual.squaredNorm()-(group.raw.residual+group.raw.jacobian*delta).squaredNorm());
      owner_->posim_ping_planes.apply(group.spec.plane_id,nuisance[i].head<3>());
      if(!group.raw.applyLandmarks(nuisance[i]))throw std::runtime_error("joint_landmark_apply_failed");
    }
    return change;
  }
 private:
  // The joint route uses unscaled coordinates and external pose damping.
  using Parent::scaleJl_cols;using Parent::scaleJp_cols;using Parent::setLandmarkDamping;
  using Parent::setPoseDamping;using Parent::getJp_diag2;
  enum class Phase {Empty,Raw,Reduced,Applied};Phase phase_=Phase::Empty;
  void requireReduced() const {
    if(phase_!=Phase::Reduced || revision_!=owner_->posim_ping_state_revision)throw std::logic_error("joint_rows_require_fresh_reduction");
  }
  uint64_t revision_=0;
  basalt::BundleAdjustmentBase<double>* owner_;
  std::vector<Group> groups_;
};
}  // namespace posim_slam

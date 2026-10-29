// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_owned_group.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"
#include "sonar_ground_store.hpp"
#include "surface_owner.hpp"

namespace posim_slam {
// One store per estimator stream, used exclusively by its owner thread. Keep
// measurements and associations, never lambdas containing obsolete state columns.
class PingFactorStore {
  struct Definition {
    std::set<basalt::KeypointId> ids;
    std::vector<OwnedPingObservation> observations;
    double surface_sigma,support_radius;
    PingPlaneReference reference;
    bool range_only=false;
  };
 public:
  explicit PingFactorStore(basalt::BundleAdjustmentBase<double>& owner):owner_(owner),ground_(owner) {}
  PingFactorStore(const PingFactorStore&)=delete;
  PingFactorStore& operator=(const PingFactorStore&)=delete;
  size_t size() const {return definitions_.size()+ground_.size();}
  GroundFactorStore& ground() {return ground_;}
  int64_t lastObservation() const {return std::max(last_observation_,ground_.lastObservation());}
  void enablePoseConditionalCapture() {
    if(capture_pose_conditionals_ || surface_ || lastObservation()>0 || owner_.posim_ping_trial_active)
      throw std::logic_error("pose_conditional_capture_must_start_before_observations");
    capture_pose_conditionals_=true;
  }
  void enableSurfaceOwner(const std::string& epoch) {
    if(surface_ || capture_pose_conditionals_ || lastObservation()>0 || owner_.posim_ping_trial_active ||
        !surface_prior::label(epoch) || epoch.size()>128)throw std::invalid_argument("exclusive_native_surface_owner_required");
    auto next=std::make_unique<SurfaceMarginalOwner>();surface_epoch_=epoch;surface_.swap(next);
  }
  SurfaceMarginalOwner* surfaceOwner() {return surface_.get();}
  void adoptSurfaceOwner(const basalt::AbsOrderMap& order,basalt::MargLinData<double>& prior,PressureFactorStore& pressure) {
    if(surface_ && !surface_->active())surface_->adopt(owner_,order,prior,pressure,surface_epoch_);
  }
  std::vector<RetiredPoseConditional> takePoseConditionals() {
    std::vector<RetiredPoseConditional> result;result.swap(pose_conditionals_);return result;
  }

  void add(const basalt::AbsOrderMap& order,int plane_id,const Vec3& normal,double distance,
      const std::set<basalt::KeypointId>& ids,const OwnedPingObservation& observation,
      double surface_sigma,double support_radius,bool range_only=false) {
    requireNewObservation(observation.stamp);
    if(definitions_.size()>=8 || definitions_.count(plane_id))
      throw std::invalid_argument("duplicate_or_excess_ping_factor_group");
    for(const auto& [id,definition]:definitions_)
      for(auto point:ids)if(definition.ids.count(point))
        throw std::invalid_argument("duplicate_ping_factor_landmark");
    Definition definition{ids,{observation},surface_sigma,support_radius,{normal,distance},range_only};
    auto next=definitions_;next.emplace(plane_id,definition);
    auto saved_planes=owner_.posim_ping_planes;
    try {
      owner_.posim_ping_planes.set(plane_id,normal,distance);
      validate(order,plane_id,definition);  // validate before publishing membership
    } catch(...) {std::swap(owner_.posim_ping_planes,saved_planes);throw;}
    definitions_.swap(next);last_observation_=observation.stamp;
  }

  // Association is fixed. Validate the beam against the existing supported
  // surface; adding a measurement never changes its landmark set.
  void append(const basalt::AbsOrderMap& order,int plane_id,const OwnedPingObservation& observation) {
    requireNewObservation(observation.stamp);
    auto next=definitions_.at(plane_id);
    next.observations.push_back(observation);
    validate(order,plane_id,next);
    std::swap(definitions_.at(plane_id),next);last_observation_=observation.stamp;
  }

  // Association uses a bounded neighbourhood and innovation gate; it is an
  // explicit local-planarity hypothesis, not an observed infinite sea floor.
  bool appendRangePatch(const basalt::AbsOrderMap& order,int id,
      const OwnedPingObservation& observation,double radius,double age_s,double innovation_sigma) {
    const auto& d=definitions_.at(id);
    if(!d.range_only || !std::isfinite(radius) || radius<=0 || !std::isfinite(age_s) || age_s<=0 ||
       !std::isfinite(innovation_sigma) || innovation_sigma<=0 || !observation.per_ray_sigma)
      throw std::invalid_argument("invalid_range_patch_association");
    const auto& first=d.observations.front();
    if(observation.stamp-first.stamp>int64_t(age_s*1e9))return false;
    const Vec3 origin=(owner_.getPoseStateWithLin(first.stamp).getPose()*first.calibration.mount).translation();
    const auto pose=owner_.getPoseStateWithLin(observation.stamp).getPose();
    if(((pose*observation.calibration.mount).translation()-origin).norm()>radius)return false;
    const auto& plane=owner_.posim_ping_planes.at(id);
    const auto prediction=expectedPingRange(pose,observation.calibration.mount,observation.calibration.rays,
        plane.normal,plane.distance,observation.bias,*observation.per_ray_sigma,
        observation.calibration.minimum,observation.calibration.maximum).range.prediction;
    if(std::abs(prediction-observation.measured)>innovation_sigma*observation.return_sigma)return false;
    append(order,id,observation);return true;
  }

  std::vector<JointPingGroupSpec> specs(const basalt::AbsOrderMap& order) const {
    if(owner_.posim_ping_trial_active)throw std::logic_error("ping_factors_during_trial");
    std::vector<JointPingGroupSpec> result;
    for(const auto& [id,definition]:definitions_)result.push_back(build(order,id,definition));
    const auto ground=ground_.specs(order);result.insert(result.end(),ground.begin(),ground.end());
    return result;
  }

  // The native estimator still owns keyframe and optical-flow bookkeeping.
  // All allocating/failing work occurs before installation; erasure afterwards
  // cannot fail. The observation watermark survives factor retirement.
  template<class ImuMap> PingRetirementPlan marginalize(const basalt::AbsOrderMap& order,
      basalt::MargLinData<double>& prior,ImuMap& measurements,
      const basalt::ImuLinData<double>& imu,const std::set<int64_t>& removed,
      const std::set<int64_t>& pose_only,const std::unordered_set<basalt::KeypointId>& lost,
      SonarFactorStore* sonar=nullptr,DvlFactorStore* dvl=nullptr,PressureFactorStore* pressure=nullptr) {
    if(surface_) {
      if(!pressure || !definitions_.empty() || ground_.size() || (sonar && sonar->size()) || !lost.empty())
        throw std::invalid_argument("exclusive_native_surface_retirement_required");
      return surface_->marginalize(owner_,order,prior,measurements,imu,removed,pose_only,dvl,*pressure);
    }
    if(imu.imu_meas.size()!=measurements.size())throw std::invalid_argument("changed_ping_factor_imu_owner");
    for(const auto& [stamp,measurement]:measurements)
      if(!imu.imu_meas.count(stamp) || imu.imu_meas.at(stamp)!=&measurement)
        throw std::invalid_argument("changed_ping_factor_imu_owner");
    auto groups=specs(order);
    auto prepared=preparePingPrior(owner_,order,groups,prior,imu,removed,pose_only,lost,sonar,dvl,pressure,capture_pose_conditionals_);
    auto pending=pose_conditionals_;auto next_id=next_pose_conditional_id_;
    if(prepared.pose_conditional) {
      if(pending.size()>=16)throw std::invalid_argument("pose_conditional_capture_backlog");
      prepared.pose_conditional->id=next_id++;pending.push_back(std::move(*prepared.pose_conditional));
    }
    auto install=[&] {
      installPingMarginalPrior(owner_,prior,groups,measurements,prepared.retirement,
        prepared.layout,removed,pose_only,prepared.prior);
    };
    auto with_acoustic=[&] {if(sonar)sonar->retireWith(*prepared.acoustic,install);else install();};
    auto with_velocity=[&] {if(dvl)dvl->retireWith(*prepared.dvl,with_acoustic);else with_acoustic();};
    if(pressure)pressure->retireWith(*prepared.pressure,prepared.pressure_prior,with_velocity);else with_velocity();
    pose_conditionals_.swap(pending);next_pose_conditional_id_=next_id;
    for(int id:prepared.retirement.planes){definitions_.erase(id);ground_.erase(id);}
    return std::move(prepared.retirement);
  }
 private:
  bool capture_pose_conditionals_=false;
  std::unique_ptr<SurfaceMarginalOwner> surface_;std::string surface_epoch_;
  uint64_t next_pose_conditional_id_=1;
  std::vector<RetiredPoseConditional> pose_conditionals_;
  void requireNewObservation(int64_t stamp) const {
    if(surface_)throw std::invalid_argument("native_surface_and_ping_factors_are_exclusive");
    if(owner_.posim_ping_trial_active)throw std::logic_error("ping_factors_during_trial");
    if(stamp<=last_observation_)throw std::invalid_argument("replayed_or_reordered_ping_observation");
  }
  void validate(const basalt::AbsOrderMap& order,int id,const Definition& d) const {
    PingLinearization::Options options;
    options.lb_options.huber_parameter=owner_.huber_thresh;
    options.lb_options.obs_std_dev=owner_.obs_std_dev;
    PingLinearization rows(&owner_,order,options,{build(order,id,d)});
    bool valid=false;rows.linearizeProblem(&valid);
    if(!valid)throw std::invalid_argument("invalid_ping_factor_geometry");
  }
  JointPingGroupSpec build(const basalt::AbsOrderMap& order,int id,const Definition& d) const {
    const auto& reference=owner_.posim_ping_planes.at(id).reference;
    if((reference.normal-d.reference.normal).norm()>1e-12 || std::abs(reference.distance-d.reference.distance)>1e-12)
      throw std::invalid_argument("changed_ping_factor_reference");
    return makeOwnedPingGroup(owner_,order,id,d.ids,d.observations,d.surface_sigma,d.support_radius,d.range_only);
  }
  basalt::BundleAdjustmentBase<double>& owner_;
  std::map<int,Definition> definitions_;
  int64_t last_observation_=0;
  GroundFactorStore ground_;
};
}

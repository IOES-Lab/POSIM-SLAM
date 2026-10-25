// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_owned_group.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"

namespace posim_slam {
// One store per estimator stream, used exclusively by its owner thread. Keep
// measurements and associations, never lambdas containing obsolete state columns.
class PingFactorStore {
  struct Definition {
    std::set<basalt::KeypointId> ids;
    std::vector<OwnedPingObservation> observations;
    double surface_sigma,support_radius;
    PingPlaneReference reference;
  };
 public:
  explicit PingFactorStore(basalt::BundleAdjustmentBase<double>& owner):owner_(owner) {}
  PingFactorStore(const PingFactorStore&)=delete;
  PingFactorStore& operator=(const PingFactorStore&)=delete;
  size_t size() const {return definitions_.size();}
  int64_t lastObservation() const {return last_observation_;}

  void add(const basalt::AbsOrderMap& order,int plane_id,const Vec3& normal,double distance,
      const std::set<basalt::KeypointId>& ids,const OwnedPingObservation& observation,
      double surface_sigma,double support_radius) {
    requireNewObservation(observation.stamp);
    if(definitions_.size()>=8 || definitions_.count(plane_id))
      throw std::invalid_argument("duplicate_or_excess_ping_factor_group");
    for(const auto& [id,definition]:definitions_)
      for(auto point:ids)if(definition.ids.count(point))
        throw std::invalid_argument("duplicate_ping_factor_landmark");
    Definition definition{ids,{observation},surface_sigma,support_radius,{normal,distance}};
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

  std::vector<JointPingGroupSpec> specs(const basalt::AbsOrderMap& order) const {
    if(owner_.posim_ping_trial_active)throw std::logic_error("ping_factors_during_trial");
    std::vector<JointPingGroupSpec> result;
    for(const auto& [id,definition]:definitions_)result.push_back(build(order,id,definition));
    return result;
  }

  // The native estimator still owns keyframe and optical-flow bookkeeping.
  // All allocating/failing work occurs before installation; erasure afterwards
  // cannot fail. The observation watermark survives factor retirement.
  template<class ImuMap> PingRetirementPlan marginalize(const basalt::AbsOrderMap& order,
      basalt::MargLinData<double>& prior,ImuMap& measurements,
      const basalt::ImuLinData<double>& imu,const std::set<int64_t>& removed,
      const std::set<int64_t>& pose_only,const std::unordered_set<basalt::KeypointId>& lost,
      SonarFactorStore* sonar=nullptr,DvlFactorStore* dvl=nullptr) {
    if(imu.imu_meas.size()!=measurements.size())throw std::invalid_argument("changed_ping_factor_imu_owner");
    for(const auto& [stamp,measurement]:measurements)
      if(!imu.imu_meas.count(stamp) || imu.imu_meas.at(stamp)!=&measurement)
        throw std::invalid_argument("changed_ping_factor_imu_owner");
    auto groups=specs(order);
    auto prepared=preparePingPrior(owner_,order,groups,prior,imu,removed,pose_only,lost,sonar,dvl);
    auto install=[&] {
      installPingMarginalPrior(owner_,prior,groups,measurements,prepared.retirement,
        prepared.layout,removed,pose_only,prepared.prior);
    };
    auto with_acoustic=[&] {if(sonar)sonar->retireWith(*prepared.acoustic,install);else install();};
    if(dvl)dvl->retireWith(*prepared.dvl,with_acoustic);else with_acoustic();
    for(int id:prepared.retirement.planes)definitions_.erase(id);
    return std::move(prepared.retirement);
  }
 private:
  void requireNewObservation(int64_t stamp) const {
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
    return makeOwnedPingGroup(owner_,order,id,d.ids,d.observations,d.surface_sigma,d.support_radius);
  }
  basalt::BundleAdjustmentBase<double>& owner_;
  std::map<int,Definition> definitions_;
  int64_t last_observation_=0;
};
}

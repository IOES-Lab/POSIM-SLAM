// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_geometry.hpp"
#include <map>

namespace posim_slam {
struct PingPlaneState {Vec3 normal;double distance;};

class PingPlaneStore {
 public:
  void set(int id,const Vec3& normal,double distance) {
    planeBasis(normal);
    if(id<0 || !std::isfinite(distance) || (!planes_.count(id) && planes_.size()>=8))
      throw std::invalid_argument("invalid_or_excess_ping_plane");
    planes_[id]={normal,distance};
  }
  const PingPlaneState& at(int id) const {return planes_.at(id);}
  size_t size() const {return planes_.size();}
  void apply(int id,const Vec3& delta) {
    if(!delta.allFinite())throw std::invalid_argument("invalid_ping_plane_increment");
    const auto& p=at(id);
    set(id,Sophus::SO3d::exp(planeBasis(p.normal)*delta.head<2>())*p.normal,p.distance+delta[2]);
  }
  void backup() {backup_=planes_;}
  void restore() {planes_=backup_;}
 private:
  std::map<int,PingPlaneState> planes_,backup_;
};

// Use in the owner thread for a fixed association set. Cost must include the
// same visual, IMU, prior and Ping factors before and after the trial. Graph
// topology is frozen during a trial. Rebuild group rows after commit/rollback.
template<class Estimator> class JointPingTrial {
 public:
  explicit JointPingTrial(Estimator& estimator):owner_(estimator) {
    if(owner_.posim_ping_trial_active)throw std::logic_error("nested_ping_trial");
    owner_.posim_ping_trial_active=true;
    try {owner_.backup();}catch(...) {owner_.posim_ping_trial_active=false;throw;}
  }
  JointPingTrial(const JointPingTrial&)=delete;
  JointPingTrial& operator=(const JointPingTrial&)=delete;
  ~JointPingTrial() {if(!finished_)rollback();}
  bool accept(double before,double after,double predicted_decrease) {
    if(finished_)throw std::logic_error("ping_trial_already_finished");
    if(!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(predicted_decrease) ||
       before<0 || after<0 || predicted_decrease<=0 || after>=before) {
      rollback();return false;
    }
    finished_=true;owner_.posim_ping_trial_active=false;return true;
  }
 private:
  void rollback() {owner_.restore();finished_=true;owner_.posim_ping_trial_active=false;}
  Estimator& owner_;bool finished_=false;
};
}  // namespace posim_slam

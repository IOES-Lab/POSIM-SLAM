// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_ground_group.hpp"
#include "ground_predictive.hpp"
#include "ground_surface_noise.hpp"

namespace posim_slam {
struct GroundNoise {double individual,frame_common,time_common;double surface=0.,surface_length=.5;};
struct GroundPoint {int64_t stamp;int beam;Vec3 point;};
inline Eigen::MatrixXd groundCovariance(const std::vector<GroundEntryObservation>& observations,
    const GroundNoise& noise,const std::vector<GroundSurfaceReference>& references={}) {
  for(double sigma:{noise.individual,noise.frame_common,noise.time_common})
    if(!std::isfinite(sigma) || sigma<=0 || sigma>1)throw std::invalid_argument("invalid_ground_noise");
  Eigen::Index count=0;for(const auto& o:observations)count+=o.ranges.size();
  if(count<8 || count>512)throw std::invalid_argument("ground_row_budget");
  Eigen::MatrixXd c=Eigen::MatrixXd::Constant(count,count,noise.time_common*noise.time_common);
  c.diagonal().array()+=noise.individual*noise.individual;
  Eigen::Index start=0;
  for(const auto& o:observations) {
    c.block(start,start,o.ranges.size(),o.ranges.size()).array()+=noise.frame_common*noise.frame_common;
    start+=o.ranges.size();
  }
  if(!std::isfinite(noise.surface) || noise.surface<0 || noise.surface>.2 ||
     !std::isfinite(noise.surface_length) || noise.surface_length<.05 || noise.surface_length>5.)
    throw std::invalid_argument("invalid_ground_surface_noise");
  if(noise.surface>0) {
    if(references.size()!=observations.size())throw std::invalid_argument("missing_ground_surface_references");
    Eigen::MatrixXd points(count,3);Eigen::VectorXd incidence(count);start=0;
    for(size_t i=0;i<observations.size();++i) {
      const auto n=observations[i].ranges.size();const auto& r=references[i];
      if(r.points.rows()!=n || r.points.cols()!=3 || r.incidence.size()!=n)
        throw std::invalid_argument("ground_surface_reference_shape");
      points.middleRows(start,n)=r.points;incidence.segment(start,n)=r.incidence;start+=n;
    }
    c+=surfaceRangeCovariance(points,incidence,noise.surface,noise.surface_length);
  }
  return c;
}

// Owner-thread storage of raw entries. Uses the existing plane state, solver
// and retirement path; it never creates an independent fitted-plane prior.
class GroundFactorStore {
  struct Definition {
    GroundEntryCalibration calibration;
    GroundNoise noise;
    std::vector<GroundEntryObservation> observations;
    Vec3 reference_normal;
    std::vector<GroundSurfaceReference> references;
    Eigen::MatrixXd covariance;
  };
 public:
  explicit GroundFactorStore(basalt::BundleAdjustmentBase<double>& owner):owner_(owner) {}
  size_t size() const {return definitions_.size();}
  int64_t lastObservation() const {return last_observation_;}
  void erase(int id) {definitions_.erase(id);}
  std::vector<int> ids() const {
    std::vector<int> result;for(const auto& [id,d]:definitions_)result.push_back(id);return result;
  }
  std::vector<GroundPoint> points() const {
    if(owner_.posim_ping_trial_active)throw std::logic_error("ground_export_during_trial");
    std::vector<GroundPoint> result;
    for(const auto& [id,d]:definitions_)for(const auto& o:d.observations) {
      const auto p=footprint(o,d.calibration);
      for(Eigen::Index i=0;i<p.rows();++i)result.push_back({o.stamp,o.beam_ids[i],p.row(i).transpose()});
    }
    return result;
  }
  std::vector<JointPingGroupSpec> specs(const basalt::AbsOrderMap& order) const {
    std::vector<JointPingGroupSpec> result;
    for(const auto& [id,d]:definitions_)result.push_back(build(order,id,d));return result;
  }
  void add(const basalt::AbsOrderMap& order,int id,const GroundEntryCalibration& calibration,
           const GroundEntryObservation& observation,const Vec3& normal,double distance,const GroundNoise& noise) {
    requireNew(observation.stamp);
    if(definitions_.size()>=8 || definitions_.count(id))throw std::invalid_argument("ground_group_budget_or_identity");
    Definition d{calibration,noise,{observation},normal,{}, {}};
    if(noise.surface>0)d.references.push_back(surfaceReference(observation,calibration,normal));
    d.covariance=groundCovariance(d.observations,noise,d.references);
    auto next=definitions_;next.emplace(id,d);auto saved=owner_.posim_ping_planes;
    try {owner_.posim_ping_planes.set(id,normal,distance);build(order,id,d).build({});}
    catch(...) {std::swap(owner_.posim_ping_planes,saved);throw;}
    definitions_.swap(next);last_observation_=observation.stamp;
  }
  bool appendPatch(const basalt::AbsOrderMap& order,int id,const GroundEntryObservation& observation,
                   double support_radius,double max_age_s,double max_gap_s,double innovation_sigma,
                   std::optional<GroundPredictiveCheck>* diagnostic=nullptr) {
    if(diagnostic)diagnostic->reset();
    requireNew(observation.stamp);
    for(double value:{support_radius,max_age_s,max_gap_s,innovation_sigma})
      if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid_ground_association");
    const auto& d=definitions_.at(id);const auto& first=d.observations.front();const auto& last=d.observations.back();
    if(d.observations.size()>=8 || observation.stamp-first.stamp>int64_t(max_age_s*1e9) ||
       observation.stamp-last.stamp>int64_t(max_gap_s*1e9) || observation.boundary_sign!=last.boundary_sign)return false;
    Eigen::Index count=observation.ranges.size();for(const auto& o:d.observations)count+=o.ranges.size();
    if(count>512)return false;
    // Overlap is tested on measured, unprojected footprints at current poses.
    // Proximity plus a range innovation is a bounded association hypothesis;
    // it does not establish object identity on a perfectly featureless plane.
    const auto old_points=footprint(last,d.calibration),new_points=footprint(observation,d.calibration);
    size_t supported=0;
    for(Eigen::Index i=0;i<new_points.rows();++i)
      if((old_points.rowwise()-new_points.row(i)).rowwise().squaredNorm().minCoeff()<=support_radius*support_radius)++supported;
    if(supported*2<size_t(new_points.rows()))return false;
    auto next=d;next.observations.push_back(observation);
    if(d.noise.surface>0)next.references.push_back(surfaceReference(observation,d.calibration,d.reference_normal));
    next.covariance=groundCovariance(next.observations,next.noise,next.references);
    const auto joined=build(order,id,next).build({});
    const auto check=groundPredictiveCheck(joined.jacobian.rightCols(3),joined.residual,
                                          count-observation.ranges.size(),innovation_sigma);
    if(diagnostic)*diagnostic=check;
    if(check.score>check.limit)return false;
    std::swap(definitions_.at(id),next);last_observation_=observation.stamp;return true;
  }
 private:
  void requireNew(int64_t stamp) const {
    if(owner_.posim_ping_trial_active)throw std::logic_error("ground_factors_during_trial");
    if(stamp<=last_observation_)throw std::invalid_argument("replayed_or_reordered_ground_observation");
  }
  JointPingGroupSpec build(const basalt::AbsOrderMap& order,int id,const Definition& d) const {
    return makeOwnedGroundGroup(owner_,order,id,d.calibration,d.observations,d.covariance);
  }
  // Measured footprints and incidence at admission, never plane-projected or
  // refreshed during optimization. This conditional covariance is immutable
  // until a new frame is appended, including through group marginalization.
  GroundSurfaceReference surfaceReference(const GroundEntryObservation& o,
      const GroundEntryCalibration& c,const Vec3& normal) const {
    GroundSurfaceReference reference{footprint(o,c),Eigen::VectorXd(o.ranges.size())};
    const auto sensor=owner_.getPoseStateWithLin(o.stamp).getPose()*c.mount;
    const double elevation=o.boundary_sign*c.vertical_fov/2;
    for(Eigen::Index i=0;i<o.ranges.size();++i) {
      const Vec3 ray(std::cos(o.azimuth[i])*std::cos(elevation),
                     std::sin(o.azimuth[i])*std::cos(elevation),std::sin(elevation));
      reference.incidence[i]=normal.dot(sensor.rotationMatrix()*ray);
    }
    return reference;
  }
  Eigen::MatrixXd footprint(const GroundEntryObservation& o,const GroundEntryCalibration& c) const {
    const auto sensor=owner_.getPoseStateWithLin(o.stamp).getPose()*c.mount;
    const double elevation=o.boundary_sign*c.vertical_fov/2;
    Eigen::MatrixXd points(o.ranges.size(),3);
    for(Eigen::Index i=0;i<o.ranges.size();++i)
      points.row(i)=(sensor*(o.ranges[i]*Vec3(std::cos(o.azimuth[i])*std::cos(elevation),
        std::sin(o.azimuth[i])*std::cos(elevation),std::sin(elevation)))).transpose();
    return points;
  }
  basalt::BundleAdjustmentBase<double>& owner_;
  std::map<int,Definition> definitions_;
  int64_t last_observation_=0;
};
}  // namespace posim_slam

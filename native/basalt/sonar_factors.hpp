// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "sonar_track.hpp"
#include <map>
#include <cstdint>
#include <set>

namespace posim_slam {
struct SonarPose {Eigen::Vector3d position;Eigen::Matrix3d attitude;};
struct SonarObservation {int64_t stamp;double range,azimuth;};
struct SonarCalibration {
  Eigen::Vector3d lever;
  Eigen::Matrix3d sonar_to_imu;
  double minimum,maximum,horizontal_fov,vertical_fov;
};
struct SonarTrackInput {
  int64_t id;
  std::vector<SonarObservation> observations;
  Eigen::MatrixXd covariance;
};
using SonarPoses=std::map<int64_t,SonarPose>;

// One Huber loss for the full whitened track preserves its covariance and is
// invariant to an orthogonal change of whitening basis. The transition is
// three RMS standardized residual units, not a calibrated acceptance gate.
struct SonarTrackLoss {double cost,sqrt_weight;};
inline SonarTrackLoss sonarTrackLoss(const Eigen::VectorXd& residual) {
  if(residual.size()<2 || !residual.allFinite())throw std::invalid_argument("invalid_sonar_loss_residual");
  const double norm=residual.norm(),threshold=3.*std::sqrt(double(residual.size()));
  if(norm<=threshold)return {.5*norm*norm,1.};
  return {threshold*(norm-.5*threshold),std::sqrt(threshold/norm)};
}


// Owner-thread storage for independently associated acoustic points. Optical
// points shared with sonar need a joint visual/acoustic block, not this store.
// Pose updates and this store's trial must share the estimator's transaction.
class SonarFactorStore {
  struct Track {
    std::vector<SonarObservation> observations;
    Eigen::MatrixXd covariance;
    SonarCalibration calibration;
    Eigen::Vector3d reference_point;
  };
 public:
  SonarFactorStore()=default;
  SonarFactorStore(const SonarFactorStore&)=delete;
  SonarFactorStore& operator=(const SonarFactorStore&)=delete;
  struct Linearization {
    const SonarFactorStore* owner;
    uint64_t revision;
    std::vector<int64_t> pose_stamps;
    std::map<int64_t,SonarTrackRows> rows;
    std::map<int64_t,SonarReducedRows> reduced;
  };
  void add(int64_t id,const Eigen::Vector3d& point,const SonarCalibration& calibration,
      const std::vector<SonarObservation>& observations,const Eigen::MatrixXd& covariance,
      const SonarPoses& poses) {
    requireIdle();
    if(id<=last_id_ || tracks_.count(id) || tracks_.size()>=16 || !point.allFinite())
      throw std::invalid_argument("invalid_sonar_track_id_or_point");
    Track track{observations,covariance,calibration,point};
    build(track,point,poses);
    auto next_tracks=tracks_;auto next_points=points_;
    next_tracks.emplace(id,std::move(track));next_points.emplace(id,point);
    tracks_.swap(next_tracks);points_.swap(next_points);last_id_=id;++revision_;
  }
  // Supply the complete extended covariance: old temporal correlations cannot
  // be discarded or silently reweighted when a new frame arrives.
  void append(int64_t id,const SonarObservation& observation,
      const Eigen::MatrixXd& covariance,const SonarPoses& poses) {
    requireIdle();auto track=tracks_.at(id);
    if(observation.stamp<=track.observations.back().stamp)
      throw std::invalid_argument("replayed_sonar_track_observation");
    const auto old=track.covariance.rows();
    if(covariance.rows()!=old+2 || covariance.cols()!=old+2 ||
        !(covariance.topLeftCorner(old,old).array()==track.covariance.array()).all())
      throw std::invalid_argument("changed_sonar_track_covariance_history");
    track.observations.push_back(observation);track.covariance=covariance;
    build(track,points_.at(id),poses);
    std::swap(tracks_.at(id),track);++revision_;
  }
  // Validate a complete input batch on a private copy. Existing history and
  // calibration cannot change. New points start on the first measured fan ray;
  // that seed is not an elevation constraint or a qualified map point.
  // Unsupported point geometry rejects only that update; malformed input
  // still rolls back the entire batch. Return the rejected native track IDs.
  std::vector<int64_t> applySnapshot(int64_t stamp,const std::vector<SonarTrackInput>& updates,
      const SonarCalibration& calibration,const SonarPoses& poses) {
    requireIdle();
    if(stamp<=last_snapshot_ || updates.empty() || updates.size()>16)
      throw std::invalid_argument("invalid_sonar_snapshot_order");
    SonarFactorStore staged;
    staged.tracks_=tracks_;staged.points_=points_;staged.last_id_=last_id_;staged.revision_=revision_;
    std::vector<int64_t> rejected;
    int64_t previous=-1;
    for(const auto& update:updates) {
      if(update.id<=previous || update.observations.empty() || update.observations.back().stamp!=stamp)
        throw std::invalid_argument("invalid_sonar_snapshot_membership");
      previous=update.id;
      if(staged.contains(update.id)) {
        const auto& old=staged.tracks_.at(update.id);const auto& c=old.calibration;
        if(update.observations.size()!=old.observations.size()+1 ||
            (c.lever-calibration.lever).norm()!=0 || (c.sonar_to_imu-calibration.sonar_to_imu).norm()!=0 ||
            c.minimum!=calibration.minimum || c.maximum!=calibration.maximum ||
            c.horizontal_fov!=calibration.horizontal_fov || c.vertical_fov!=calibration.vertical_fov)
          throw std::invalid_argument("changed_sonar_snapshot_contract");
        for(size_t i=0;i<old.observations.size();++i) {
          const auto& a=old.observations[i];const auto& b=update.observations[i];
          if(a.stamp!=b.stamp || a.range!=b.range || a.azimuth!=b.azimuth)
            throw std::invalid_argument("changed_sonar_snapshot_history");
        }
        try {staged.append(update.id,update.observations.back(),update.covariance,poses);}
        catch(const SonarPointOutsideSupport&) {rejected.push_back(update.id);}
      }else {
        if(update.observations.size()<3)throw std::invalid_argument("sonar_seed_requires_track_history");
        const auto& first=update.observations.front();const auto& pose=poses.at(first.stamp);
        const Eigen::Vector3d local(first.range*std::cos(first.azimuth),first.range*std::sin(first.azimuth),0.);
        const Eigen::Vector3d seed=pose.position+pose.attitude*(calibration.lever+calibration.sonar_to_imu*local);
        try {staged.add(update.id,seed,calibration,update.observations,update.covariance,poses);}
        catch(const SonarPointOutsideSupport&) {rejected.push_back(update.id);}
      }
    }
    tracks_.swap(staged.tracks_);points_.swap(staged.points_);
    last_id_=staged.last_id_;revision_=staged.revision_;last_snapshot_=stamp;
    return rejected;
  }

  Linearization linearize(const SonarPoses& poses,const SonarPoses* references=nullptr) const {
    requireIdle();Linearization result;result.owner=this;result.revision=revision_;
    for(const auto& [stamp,pose]:poses)result.pose_stamps.push_back(stamp);
    for(const auto& [id,track]:tracks_) {
      auto rows=build(track,points_.at(id),poses);
      if(references) {
        SonarTrackRows fixed;
        try {fixed=build(track,track.reference_point,*references);}
        catch(const SonarPointOutsideSupport& error) {
          throw SonarPointOutsideSupport(std::string("sonar_reference_geometry:")+error.what());
        }
        if(fixed.pose_jacobian.cols()!=rows.pose_jacobian.cols() || references->size()!=poses.size())
          throw std::invalid_argument("changed_sonar_reference_order");
        auto current=poses.begin();
        for(const auto& [stamp,pose]:*references)
          if(stamp!=(current++)->first)throw std::invalid_argument("changed_sonar_reference_order");
        fixed.residual=rows.residual;rows=std::move(fixed);
      }
      const double weight=sonarTrackLoss(rows.residual).sqrt_weight;
      rows.pose_jacobian*=weight;rows.point_jacobian*=weight;rows.residual*=weight;
      result.reduced.emplace(id,eliminateSonarPoint(rows));
      result.rows.emplace(id,std::move(rows));
    }
    return result;
  }
  double cost(const SonarPoses& poses) const {
    double total=0.;
    for(const auto& [id,track]:tracks_)total+=sonarTrackLoss(build(track,points_.at(id),poses).residual).cost;
    if(!std::isfinite(total))throw std::invalid_argument("nonfinite_sonar_cost");
    return total;
  }
  const Eigen::Vector3d& point(int64_t id) const {return points_.at(id);}
  size_t size() const {return tracks_.size();}
  uint64_t revision() const {return revision_;}
  std::vector<int64_t> activeIds() const {
    std::vector<int64_t> result;for(const auto& item:tracks_)result.push_back(item.first);return result;
  }
  bool contains(int64_t id) const {return tracks_.count(id)!=0;}

  struct Retirement {
    const SonarFactorStore* owner;
    uint64_t revision;
    std::set<int64_t> ids,required_poses;
  };
  Retirement planRetirement(const std::set<int64_t>& removed) const {
    requireIdle();Retirement plan{this,revision_,{}, {}};
    for(const auto& [id,track]:tracks_) {
      bool retire=false;
      for(const auto& observation:track.observations)retire=retire || removed.count(observation.stamp);
      if(retire) {
        plan.ids.insert(id);
        for(const auto& observation:track.observations)plan.required_poses.insert(observation.stamp);
      }
    }
    return plan;
  }
  // The caller installs the prior and retires other factors atomically. Until
  // it succeeds, this store is untouched. Erasure afterwards cannot allocate.
  template<class Installer> void retireWith(const Retirement& plan,Installer install) {
    requireIdle();
    if(plan.owner!=this || plan.revision!=revision_)
      throw std::invalid_argument("stale_sonar_retirement");
    for(auto id:plan.ids)if(!tracks_.count(id))throw std::invalid_argument("missing_retired_sonar_track");
    install();
    for(auto id:plan.ids){tracks_.erase(id);points_.erase(id);}
    ++revision_;
  }

  class Trial {
   public:
    explicit Trial(SonarFactorStore& store):store_(store) {
      store_.requireIdle();backup_=store_.points_;store_.trial_=true;
    }
    Trial(const Trial&)=delete;Trial& operator=(const Trial&)=delete;
    ~Trial() {if(!finished_)finish(false);}
    // Accept only after the owner accepts the SAME full visual/IMU/Ping/sonar
    // objective and commits its pose states. Rejection restores all points.
    void finish(bool accepted) {
      if(finished_)throw std::logic_error("sonar_trial_already_finished");
      if(!accepted)store_.points_.swap(backup_);
      ++store_.revision_;store_.trial_=false;finished_=true;
    }
   private:
    SonarFactorStore& store_;std::map<int64_t,Eigen::Vector3d> backup_;bool finished_=false;
  };

  void apply(const Linearization& rows,const Eigen::VectorXd& pose_delta) {
    if(!trial_ || rows.owner!=this || rows.revision!=revision_ || rows.reduced.size()!=tracks_.size() ||
        pose_delta.size()!=6*Eigen::Index(rows.pose_stamps.size()) || !pose_delta.allFinite())
      throw std::logic_error("sonar_increment_requires_current_joint_trial");
    auto next=points_;
    for(const auto& [id,track]:tracks_) {
      next.at(id)+=rows.reduced.at(id).backSubstitute(pose_delta);
      if(!next.at(id).allFinite())throw std::invalid_argument("nonfinite_sonar_point_update");
    }
    points_.swap(next);++revision_;
  }
 private:
  void requireIdle() const {
    if(trial_)throw std::logic_error("sonar_topology_or_linearization_during_trial");
  }
  static SonarTrackRows build(const Track& track,const Eigen::Vector3d& point,const SonarPoses& poses) {
    if(poses.empty() || poses.size()>64 || track.observations.empty() || track.observations.size()>64)
      throw std::invalid_argument("invalid_sonar_window_size");
    std::map<int64_t,int> slots;int slot=0;
    for(const auto& [stamp,pose]:poses) {
      if(stamp<=0)throw std::invalid_argument("invalid_sonar_pose_stamp");
      slots.emplace(stamp,slot++);
    }
    sonarNoise(track.covariance,2*track.observations.size());
    std::string unsupported;
    std::vector<SonarPointRows> rows;std::vector<int> indices;int64_t previous=0;
    const auto& c=track.calibration;
    for(const auto& observation:track.observations) {
      if(observation.stamp<=previous)throw std::invalid_argument("unordered_sonar_track");
      const auto& pose=poses.at(observation.stamp);
      try {rows.push_back(sonarPointRows(pose.position,pose.attitude,c.lever,c.sonar_to_imu,point,
        observation.range,observation.azimuth,c.minimum,c.maximum,c.horizontal_fov,c.vertical_fov));}
      catch(const SonarPointOutsideSupport& error) {unsupported=error.what();}
      indices.push_back(slots.at(observation.stamp));previous=observation.stamp;
    }
    if(!unsupported.empty())throw SonarPointOutsideSupport(unsupported);
    return whitenSonarTrack(rows,indices,poses.size(),track.covariance);
  }
  std::map<int64_t,Track> tracks_;
  std::map<int64_t,Eigen::Vector3d> points_;
  uint64_t revision_=0;int64_t last_id_=-1,last_snapshot_=0;bool trial_=false;
};
}

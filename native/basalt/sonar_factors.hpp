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
using SonarPoses=std::map<int64_t,SonarPose>;

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
  Linearization linearize(const SonarPoses& poses,const SonarPoses* references=nullptr) const {
    requireIdle();Linearization result;result.owner=this;result.revision=revision_;
    for(const auto& [stamp,pose]:poses)result.pose_stamps.push_back(stamp);
    for(const auto& [id,track]:tracks_) {
      auto rows=build(track,points_.at(id),poses);
      if(references) {
        auto fixed=build(track,track.reference_point,*references);
        if(fixed.pose_jacobian.cols()!=rows.pose_jacobian.cols() || references->size()!=poses.size())
          throw std::invalid_argument("changed_sonar_reference_order");
        auto current=poses.begin();
        for(const auto& [stamp,pose]:*references)
          if(stamp!=(current++)->first)throw std::invalid_argument("changed_sonar_reference_order");
        fixed.residual=rows.residual;rows=std::move(fixed);
      }
      result.reduced.emplace(id,eliminateSonarPoint(rows));
      result.rows.emplace(id,std::move(rows));
    }
    return result;
  }
  double cost(const SonarPoses& poses) const {
    double total=0.;
    for(const auto& [id,track]:tracks_)total+=.5*build(track,points_.at(id),poses).residual.squaredNorm();
    if(!std::isfinite(total))throw std::invalid_argument("nonfinite_sonar_cost");
    return total;
  }
  const Eigen::Vector3d& point(int64_t id) const {return points_.at(id);}
  size_t size() const {return tracks_.size();}
  uint64_t revision() const {return revision_;}
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
    std::vector<SonarPointRows> rows;std::vector<int> indices;int64_t previous=0;
    const auto& c=track.calibration;
    for(const auto& observation:track.observations) {
      if(observation.stamp<=previous)throw std::invalid_argument("unordered_sonar_track");
      const auto& pose=poses.at(observation.stamp);
      rows.push_back(sonarPointRows(pose.position,pose.attitude,c.lever,c.sonar_to_imu,point,
        observation.range,observation.azimuth,c.minimum,c.maximum,c.horizontal_fov,c.vertical_fov));
      indices.push_back(slots.at(observation.stamp));previous=observation.stamp;
    }
    return whitenSonarTrack(rows,indices,poses.size(),track.covariance);
  }
  std::map<int64_t,Track> tracks_;
  std::map<int64_t,Eigen::Vector3d> points_;
  uint64_t revision_=0;int64_t last_id_=-1;bool trial_=false;
};
}

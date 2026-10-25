// SPDX-License-Identifier: Apache-2.0
// Bounded live pixel/raw-IMU worker. Reset by replacing this process, never by
// reconnecting a new clock or calibration to an old estimator window.
#include "calibration.hpp"
#include "visual_support.hpp"
#include "landmark_archive.hpp"
#include "ping_input.hpp"
#include "ping_factors.hpp"
#include "sonar_packet.hpp"
#include "sonar_input.hpp"
#include "dvl_input.hpp"
#include "pressure_input.hpp"
#include "sonar_ground_packet.hpp"
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>
#include <tbb/global_control.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <array>
#include <iomanip>
#include <map>

uint32_t u32(const char* p) {
  return uint32_t(uint8_t(p[0])) | uint32_t(uint8_t(p[1])) << 8 |
         uint32_t(uint8_t(p[2])) << 16 | uint32_t(uint8_t(p[3])) << 24;
}
uint64_t u64(const char* p) {
  return uint64_t(u32(p)) | uint64_t(u32(p+4)) << 32;
}
double number(const char* p) {
  uint64_t bits=u64(p); double value;
  std::memcpy(&value,&bits,8);return value;
}
Json vector3(const Vec3& value) {
  return {value.x(),value.y(),value.z()};
}

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  std::ostream output(std::cout.rdbuf());
  // Upstream informational output must never corrupt the JSON protocol.
  std::cout.rdbuf(std::cerr.rdbuf());
  std::mutex mutex;
  auto emit=[&](const Json& value) {
    std::lock_guard<std::mutex> lock(mutex);
    output<<value.dump()<<'\n';output.flush();
  };
  auto fail=[&](const std::string& reason) {
    emit({{"type","error"},{"reason",reason}});
    // Isolated worker exit releases blocked upstream queues/threads. The ROS
    // owner invalidates its whole segment and creates a fresh worker explicitly.
    std::_Exit(2);
  };
  try {
    auto c=input_calibration(argv[1]);
    struct Snapshot {int64_t stamp;std::vector<std::array<float,3>> points;};
    // Exact image/Ping pairs travel atomically. This is observation transport;
    // the joint optimizer must explicitly consume them before claiming fusion.
    std::mutex ping_mutex;
    struct PendingPing {double range;bool owner_received=false;Json candidate=nullptr;bool fused=false;};
    std::map<int64_t,PendingPing> image_pings;
    std::map<int64_t,posim_slam::SonarSnapshot> image_sonar;
    std::map<int64_t,Json> sonar_reports;
    std::map<int64_t,Json> initialization_windows;
    std::map<int64_t,posim_slam::DvlObservation> pending_dvl;
    std::map<int64_t,Json> dvl_reports;
    std::map<int64_t,posim_slam::PressureObservation> pending_pressure;
    std::map<int64_t,Json> pressure_reports;
    std::map<int64_t,posim_slam::GroundPacket> pending_ground;
    std::map<int64_t,Json> ground_reports;
    std::mutex snapshot_mutex;
    std::shared_ptr<const Snapshot> snapshot;
    const fs::path export_file=fs::absolute(argv[1]).parent_path()/"worker-map.ply";
    cv::setNumThreads(1);
    tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
    const auto input_settings=read_json(argv[1]);
    const auto state_clock=input_settings.value("state_clock",std::string("camera"));
    if(state_clock!="camera" && state_clock!="sonar")fail("invalid_state_clock");
    const bool sonar_clock=state_clock=="sonar";
    const int64_t sonar_gap=sonar_clock ? std::llround(positive(input_settings,"max_sonar_gap_s")*1e9) : 0;
    if(sonar_clock && (sonar_gap<=0 || sonar_gap>3000000000LL))fail("invalid_sonar_gap_limit");
    const bool sonar_enabled=input_settings.value("enable_joint_sonar",false);
    const bool auxiliary_enabled=input_settings.value("enable_auxiliary_sonar",false);
    const bool require_gravity_seed=input_settings.value("require_gravity_seed",false);
    if(sonar_clock && (!sonar_enabled || !require_gravity_seed || input_settings.value("enable_joint_ping",false)))
      fail("sonar_clock_configuration");
    if(auxiliary_enabled && !sonar_enabled)fail("auxiliary_sonar_requires_joint_sonar");
    auto config=processing_config(c);
    auto vio=basalt::VioEstimatorFactory::getVioEstimator(
        config,c.cal,Vec3(0,0,-c.gravity),true,true);
    std::optional<posim_slam::PingInputCalibration> ping_calibration;
    const auto ping_file=fs::path(argv[1]).parent_path()/"ping-calibration.json";
    if(fs::exists(ping_file))ping_calibration=posim_slam::pingInputCalibration(read_json(ping_file));
    auto owner=std::dynamic_pointer_cast<basalt::SqrtKeypointVioEstimator<double>>(vio);
    if(!owner)throw std::runtime_error("unsupported_ping_estimator_owner");
    const bool dvl_enabled=input_settings.value("enable_joint_dvl",false);
    const bool pressure_enabled=input_settings.value("enable_joint_pressure",false);
    const bool terrain_enabled=input_settings.value("enable_terrain_ping",false);
    const bool ground_enabled=input_settings.value("enable_sonar_ground",false);
    if(ground_enabled && (!sonar_clock || terrain_enabled || input_settings.value("enable_joint_ping",false)))
      fail("ground_requires_exclusive_sonar_clock");
    if(terrain_enabled && input_settings.value("enable_joint_ping",false))fail("conflicting_ping_surface_modes");
    const bool ordered_auxiliary=dvl_enabled || pressure_enabled || terrain_enabled || ground_enabled;
    posim_slam::GroundNoise ground_noise{};double ground_radius=0.,ground_age=0.,ground_gap=0.,ground_gate=0.;
    if(ground_enabled) {
      ground_noise={positive(input_settings,"ground_individual_sigma_m"),positive(input_settings,"ground_frame_common_sigma_m"),positive(input_settings,"ground_time_common_sigma_m")};
      ground_radius=positive(input_settings,"ground_support_radius_m");ground_age=positive(input_settings,"ground_max_age_s");
      ground_gap=positive(input_settings,"ground_max_gap_s");ground_gate=positive(input_settings,"ground_innovation_sigma");
      if(ground_noise.individual>1 || ground_noise.frame_common>1 || ground_noise.time_common>1 ||
         ground_radius>2 || ground_age>5 || ground_gap>ground_age || ground_gate>10)fail("ground_model_bounds");
    }
    double terrain_sigma=0.,terrain_radius=0.,terrain_age=0.,terrain_gate=0.;
    if(terrain_enabled) {
      terrain_sigma=positive(input_settings,"terrain_return_sigma_m");
      terrain_radius=positive(input_settings,"terrain_patch_radius_m");
      terrain_age=positive(input_settings,"terrain_patch_max_age_s");
      terrain_gate=positive(input_settings,"terrain_innovation_sigma");
      if(terrain_sigma>1 || terrain_radius>5 || terrain_age>5 || terrain_gate>10)fail("terrain_ping_model_bounds");
    }
    std::optional<posim_slam::PressureCalibration> pressure_calibration;
    if(pressure_enabled) {
      pressure_calibration=posim_slam::pressureInputCalibration(read_json(fs::path(argv[1]).parent_path()/"pressure-calibration.json"));
      owner->posim_pressure_factors=std::make_shared<posim_slam::PressureFactorStore>(*pressure_calibration);
    }
    std::optional<posim_slam::DvlCalibration> dvl_calibration;
    if(dvl_enabled) {
      dvl_calibration=posim_slam::dvlInputCalibration(read_json(fs::path(argv[1]).parent_path()/"dvl-calibration.json"));
      owner->posim_dvl_factors=std::make_shared<posim_slam::DvlFactorStore>(*dvl_calibration);
    }
    const bool record_visual_tracks=input_settings.value("record_visual_tracks",false);
    std::ofstream visual_trace;
    if(record_visual_tracks) {
      auto path=fs::path(argv[1]);path.replace_extension("visual-tracks.jsonl");
      visual_trace.open(path);
      if(!visual_trace)throw std::runtime_error("visual_trace_open_failed");
    }
    std::optional<posim_slam::SonarCalibration> sonar_calibration;
    if(sonar_enabled) {
      sonar_calibration=posim_slam::sonarInputCalibration(read_json(fs::path(argv[1]).parent_path()/"sonar-calibration.json"));
      owner->posim_sonar_factors=std::make_shared<posim_slam::SonarFactorStore>();
    }
    const bool joint_enabled=input_settings.value("enable_joint_ping",false);
    std::optional<posim_slam::GroundEntryCalibration> ground_calibration;
    if(ground_enabled)ground_calibration=posim_slam::GroundEntryCalibration{
      Sophus::SE3d(sonar_calibration->sonar_to_imu,sonar_calibration->lever),
      sonar_calibration->vertical_fov,sonar_calibration->minimum,sonar_calibration->maximum};
    const auto loss_name=input_settings.value("ping_range_loss",std::string("cauchy"));
    if(loss_name!="cauchy" && loss_name!="huber")throw std::runtime_error("invalid_ping_range_loss");
    const auto range_loss=loss_name=="cauchy"?posim_slam::PingRangeLoss::Cauchy:posim_slam::PingRangeLoss::Huber;
    std::optional<double> range_scale=3.;
    if(input_settings.contains("ping_range_loss_scale")) {
      if(input_settings.at("ping_range_loss_scale").is_null())range_scale.reset();
      else range_scale=positive(input_settings,"ping_range_loss_scale");
    }
    double ping_bias=0.,ping_ray_sigma=0.;int next_plane_id=0;
    if(joint_enabled || terrain_enabled) {
      if(!ping_calibration)throw std::runtime_error("joint_ping_requires_calibration");
      const auto settings=read_json(ping_file);
      ping_bias=settings.at("range_bias").get<double>();
      ping_ray_sigma=positive(settings,"per_ray_noise_stddev");
      if(!std::isfinite(ping_bias))throw std::runtime_error("invalid_ping_bias");
    }
    if(joint_enabled || terrain_enabled || ground_enabled)
      owner->posim_ping_factors=std::make_shared<posim_slam::PingFactorStore>(static_cast<basalt::BundleAdjustmentBase<double>&>(*owner));
    owner->posim_after_joint=[&](auto& estimator,int64_t stamp) {
      std::lock_guard<std::mutex> lock(ping_mutex);
      // Copy on the estimator thread before marginalization. Output threads
      // never inspect mutable optimizer state. This remains diagnostic only.
      const auto& window=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator).frame_states;
      if(window.size()>128 || (!window.empty() && stamp-window.begin()->first>30'000'000'000LL))
        fail(sonar_clock ? "sonar_initialization_window_exhausted" : "visual_initialization_window_exhausted");
      if(!window.empty()) {
        const auto& oldest=window.begin()->second.getState();
        auto q=oldest.T_w_i.unit_quaternion();
        initialization_windows[stamp]={{"oldest_stamp_ns",oldest.t_ns},
          {"states",window.size()},{"position",vector3(oldest.T_w_i.translation())},
          {"orientation_xyzw",{q.x(),q.y(),q.z(),q.w()}},
          {"velocity",vector3(oldest.vel_w_i)},
          {"gyro_bias",vector3(oldest.bias_gyro)},{"accel_bias",vector3(oldest.bias_accel)}};
        if(initialization_windows.size()>16)fail("initialization_diagnostic_backlog");
      }
      if(sonar_enabled) {
        auto& report=sonar_reports[stamp];if(report.is_null())report=Json::object();
        Json points=Json::array();
        // Copy committed estimates on their owner thread, before retirement.
        // These are acoustic track hypotheses, not classified solid surfaces.
        for(auto id:estimator.posim_sonar_factors->activeIds())
          points.push_back({{"id",id},{"point",vector3(estimator.posim_sonar_factors->point(id))}});
        report["point_estimates"]=std::move(points);
        report["point_update_accepted"]=estimator.posim_joint_last_accepted>0;
      }
      if(dvl_reports.count(stamp)) {
        auto& report=dvl_reports.at(stamp);
        report["optimization_included"]=estimator.posim_joint_started;
        report["accepted_steps"]=estimator.posim_joint_last_accepted;
      }
      if(pressure_reports.count(stamp)) {
        auto& report=pressure_reports.at(stamp);
        report["optimization_included"]=estimator.posim_joint_started;
        report["accepted_steps"]=estimator.posim_joint_last_accepted;
        report["datum_m"]=estimator.posim_pressure_factors->datum();
      }
      if(terrain_enabled && image_pings.count(stamp) && estimator.posim_ping_factors) {
        auto& candidate=image_pings.at(stamp).candidate;
        if(candidate.is_object() && candidate.contains("plane_id")) {
          const auto order=posim_slam::planPingMarginalLayout(static_cast<basalt::BundleAdjustmentBase<double>&>(estimator),{},{},{},{}).source;
          for(const auto& group:estimator.posim_ping_factors->specs(order)) {
            if(group.plane_id!=candidate.at("plane_id").get<int>())continue;
            const auto raw=group.build({});
            const auto reduced=posim_slam::eliminateJointPatch(raw.jacobian,raw.residual,order.total_size);
            const auto& plane=estimator.posim_ping_planes.at(group.plane_id);
            candidate["normal"]=vector3(plane.normal);candidate["distance_m"]=plane.distance;
            candidate["observations"]=raw.observation_stamps.size();
            candidate["conditional_plane_rank"]=reduced.nuisance_rank;
            candidate["retained_constraint_rows"]=reduced.jacobian.rows();
            candidate["map_qualified"]=false;
          }
        }
      }
      if(ground_enabled) {
        auto& report=ground_reports[stamp];if(report.is_null())report={{"owner_received",false},{"factor_registered",false},{"map_qualified",false}};
        report["optimization_included"]=estimator.posim_joint_started;
        report["accepted_steps"]=estimator.posim_joint_last_accepted;
        report["active_groups"]=estimator.posim_ping_factors->ground().size();
        report["point_estimates"]=Json::array();
        for(const auto& p:estimator.posim_ping_factors->ground().points())
          report["point_estimates"].push_back({{"stamp_ns",p.stamp},{"beam_id",p.beam},{"point",vector3(p.point)}});
        report["pose_estimates"]=Json::array();
        const auto& base=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator);
        auto append_pose=[&](int64_t time,const Sophus::SE3d& pose) {
          const auto q=pose.unit_quaternion();
          report["pose_estimates"].push_back({{"stamp_ns",time},{"position",vector3(pose.translation())},
            {"orientation_xyzw",{q.x(),q.y(),q.z(),q.w()}}});
        };
        for(const auto& [time,state]:base.frame_poses)append_pose(time,state.getPose());
        for(const auto& [time,state]:base.frame_states)append_pose(time,state.getState().T_w_i);
        if(ground_reports.size()>16)fail("ground_report_backlog");
        if(report.contains("plane_id")) {
          const auto order=posim_slam::planPingMarginalLayout(static_cast<basalt::BundleAdjustmentBase<double>&>(estimator),{},{},{},{}).source;
          for(const auto& group:estimator.posim_ping_factors->ground().specs(order))if(group.plane_id==report.at("plane_id").get<int>()) {
            const auto raw=group.build({});const auto& plane=estimator.posim_ping_planes.at(group.plane_id);
            report["group_frames"]=raw.observation_stamps.size();report["range_rows"]=raw.residual.size();
            report["normal"]=vector3(plane.normal);report["distance_m"]=plane.distance;
            report["conditional_plane_rank"]=posim_slam::eliminateJointPatch(raw.jacobian,raw.residual,order.total_size).nuisance_rank;
          }
        }
      }
      auto found=image_pings.find(stamp);
      if(found!=image_pings.end() && estimator.posim_ping_factors)
        found->second.fused=estimator.posim_ping_factors->lastObservation()==stamp && estimator.posim_joint_last_accepted>0;
    };
    owner->posim_after_marginalization=[&](auto& estimator,int64_t stamp) {
      if(!sonar_enabled)return;
      std::lock_guard<std::mutex> lock(ping_mutex);
      auto& report=sonar_reports[stamp];if(report.is_null())report=Json::object();
      report["active_track_ids"]=estimator.posim_sonar_factors->activeIds();
      // Report after marginalization: frontend candidates may reference only
      // poses still owned by this estimator, not an assumed frame-count window.
      const auto& base=static_cast<const basalt::BundleAdjustmentBase<double>&>(estimator);
      std::set<int64_t> retained;
      for(const auto& entry:base.frame_poses)retained.insert(entry.first);
      for(const auto& entry:base.frame_states)retained.insert(entry.first);
      report["retained_pose_stamps_ns"]=retained;
      report["visual_projection_rejections_total"]=estimator.posim_projection_rejections;
      if(report.value("owner_received",false))report["accepted_steps"]=estimator.posim_joint_last_accepted;
      report["map_qualified"]=false;
      if(sonar_reports.size()>16)fail("sonar_report_backlog");
    };
    owner->posim_before_marginalization=[&](auto& estimator,int64_t stamp) {
      auto& base=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator);
      {
        std::lock_guard<std::mutex> lock(ping_mutex);
        auto found=pending_dvl.find(stamp);
        if(found!=pending_dvl.end()) {
          estimator.posim_dvl_factors->add(found->second);
          pending_dvl.erase(found);dvl_reports[stamp]={{"owner_received",true}};
        }
        auto depth=pending_pressure.find(stamp);
        if(depth!=pending_pressure.end()) {
          estimator.posim_pressure_factors->add(depth->second,base);
          pending_pressure.erase(depth);pressure_reports[stamp]={{"owner_received",true}};
        }
      }
      std::optional<posim_slam::SonarSnapshot> acoustic;
      std::optional<posim_slam::GroundPacket> ground;
      {
        std::lock_guard<std::mutex> lock(ping_mutex);
        auto found=image_sonar.find(stamp);
        if(found!=image_sonar.end()){acoustic=std::move(found->second);image_sonar.erase(found);}
        auto surface=pending_ground.find(stamp);
        if(surface!=pending_ground.end()){ground=std::move(surface->second);pending_ground.erase(surface);}
      }
      if(ground) {
        const auto order=posim_slam::planPingMarginalLayout(base,{},{},{},{}).source;
        auto& store=estimator.posim_ping_factors->ground();
        Json report={{"owner_received",true},{"factor_registered",false},{"map_qualified",false}};
        int selected=-1;
        for(int id:store.ids())try {
          if(store.appendPatch(order,id,ground->observation,ground_radius,ground_age,ground_gap,ground_gate)){selected=id;break;}
        }catch(const std::invalid_argument& error){report["association_rejection"]=error.what();}
        if(selected<0)try {
          const auto sensor=base.getPoseStateWithLin(stamp).getPose()*ground_calibration->mount;
          const Vec3 normal=sensor.rotationMatrix()*ground->normal;
          const double distance=ground->distance+normal.dot(sensor.translation());
          store.add(order,next_plane_id,*ground_calibration,ground->observation,normal,distance,ground_noise);
          selected=next_plane_id++;
        }catch(const std::invalid_argument& error){report["registration_rejection"]=error.what();}
        if(selected>=0){report["plane_id"]=selected;report["factor_registered"]=true;}
        std::lock_guard<std::mutex> lock(ping_mutex);ground_reports[stamp]=std::move(report);
        if(ground_reports.size()>16)fail("ground_report_backlog");
      }
      if(acoustic) {
        posim_slam::SonarPoses poses;
        for(const auto& [time,state]:base.frame_poses) {
          const auto pose=state.getPose();poses.emplace(time,posim_slam::SonarPose{pose.translation(),pose.rotationMatrix()});
        }
        for(const auto& [time,state]:base.frame_states) {
          const auto pose=state.getState().T_w_i;poses.emplace(time,posim_slam::SonarPose{pose.translation(),pose.rotationMatrix()});
        }
        std::vector<int64_t> rejected;
        std::map<int64_t,std::string> rejection_reasons;
        try{rejected=estimator.posim_sonar_factors->applySnapshot(stamp,acoustic->tracks,*sonar_calibration,poses,&rejection_reasons);}
        catch(const std::exception& error){fail(error.what());}
        std::lock_guard<std::mutex> lock(ping_mutex);
        sonar_reports[stamp]={{"owner_received",true},{"submitted_tracks",acoustic->tracks.size()},
          {"geometry_rejected_track_ids",rejected},{"geometry_rejections",Json::array()}};
        for(const auto& [id,reason]:rejection_reasons)
          sonar_reports[stamp]["geometry_rejections"].push_back({{"id",id},{"reason",reason}});
      }
      {
        std::lock_guard<std::mutex> lock(ping_mutex);
        const auto found=image_pings.find(stamp);
        if(found==image_pings.end())return;
        if(!base.frame_states.count(stamp) || found->second.owner_received)fail("invalid_ping_owner_delivery");
        found->second.owner_received=true;
      }
      double measured;
      {std::lock_guard<std::mutex> lock(ping_mutex);measured=image_pings.at(stamp).range;}
      Json candidate={{"reason","calibration_unavailable"}};
      if(terrain_enabled) {
        const auto& p=*ping_calibration;
        const auto pose=base.getPoseStateWithLin(stamp).getPose();
        const auto order=posim_slam::planPingMarginalLayout(base,{},{},{},{}).source;
        posim_slam::OwnedPingObservation observation{stamp,measured,p,ping_bias,terrain_sigma,ping_ray_sigma,range_scale,range_loss};
        auto& factors=*estimator.posim_ping_factors;
        bool registered=false;int selected=-1;
        candidate={{"reason","terrain_rejected"},{"local_planarity_assumed",true},{"range_sigma_m",terrain_sigma}};
        for(const auto& group:factors.specs(order)) {
          if(!group.range_only)continue;
          try {
            if(factors.appendRangePatch(order,group.plane_id,observation,terrain_radius,terrain_age,terrain_gate)) {
              registered=true;selected=group.plane_id;break;
            }
          }catch(const std::invalid_argument& error){candidate["association_rejection"]=error.what();}
        }
        if(!registered)try {
          const auto sensor=pose*p.mount;
          const Vec3 axis=sensor.rotationMatrix()*p.rays.colwise().mean().transpose();
          if(axis.z()>-.5)throw std::invalid_argument("terrain_ping_requires_downward_view");
          // A numerical seed only. No normal/distance prior is installed.
          const Vec3 normal=-Vec3::UnitZ();
          double cosine=0.;
          for(Eigen::Index i=0;i<p.rays.rows();++i)
            cosine=std::max(cosine,normal.dot(sensor.rotationMatrix()*p.rays.row(i).transpose()));
          double distance=normal.dot(sensor.translation())+(measured-ping_bias)*cosine;
          for(int i=0;i<3;++i) {
            const auto predicted=posim_slam::expectedPingRange(pose,p.mount,p.rays,normal,distance,
                ping_bias,ping_ray_sigma,p.minimum,p.maximum).range;
            distance+=(measured-predicted.prediction)/predicted.plane_jacobian[2];
          }
          factors.add(order,next_plane_id,normal,distance,{},observation,terrain_sigma,terrain_radius,true);
          selected=next_plane_id++;registered=true;
        }catch(const std::invalid_argument& error){candidate["registration_rejection"]=error.what();}
        candidate["reason"]=registered?"terrain_registered":"terrain_rejected";
        if(registered)candidate["plane_id"]=selected;
        std::lock_guard<std::mutex> lock(ping_mutex);image_pings.at(stamp).candidate=std::move(candidate);
        return;
      }

      if(ping_calibration) {
        const auto points=posim_slam::activePingPoints(base,stamp,c.processed_width,c.processed_height);
        const auto& p=*ping_calibration;posim_slam::PingPatchDiagnostics diagnostics;
        const auto patch=posim_slam::selectPingPatch(points,base.getPoseStateWithLin(stamp).getPose(),p.mount,p.rays,p.minimum,p.maximum,.03,&diagnostics);
        candidate={{"reason",patch?"candidate_only":"no_supported_patch"},{"active_mature_points",points.size()},
          {"local_points",diagnostics.local_points},{"max_plane_inliers",diagnostics.max_inliers}};
        if(patch) {
          candidate["landmark_ids"]=patch->ids;candidate["normal"]=vector3(patch->normal);
          candidate["distance_m"]=patch->distance;candidate["predicted_range_m"]=patch->predicted_range;
          candidate["rms_m"]=patch->rms;candidate["support_radius_m"]=patch->support_radius;
          if(joint_enabled) {
            const auto order=posim_slam::planPingMarginalLayout(base,{},{},{},{}).source;
            posim_slam::OwnedPingObservation observation{stamp,measured,p,ping_bias,.02,ping_ray_sigma,range_scale,range_loss};
            auto& factors=*estimator.posim_ping_factors;bool registered=false;
            for(const auto& group:factors.specs(order)) {
              try{factors.append(order,group.plane_id,observation);registered=true;break;}
              catch(const std::invalid_argument&){} // unsupported fixed patch: try another
            }
            if(!registered) {
              try {
                factors.add(order,next_plane_id,patch->normal,patch->distance,
                  std::set<basalt::KeypointId>(patch->ids.begin(),patch->ids.end()),observation,.03,patch->support_radius);
                ++next_plane_id;registered=true;
              } catch(const std::invalid_argument& error){candidate["registration_rejection"]=error.what();}
            }
            candidate["reason"]=registered?"joint_registered":"joint_rejected";
          }
        }
      }
      std::lock_guard<std::mutex> lock(ping_mutex);image_pings.at(stamp).candidate=std::move(candidate);
    };
    // No optical frontend or camera calibration exists in sonar-clock mode.
    basalt::OpticalFlowBase::Ptr flow;
    if(!sonar_clock)flow=basalt::OpticalFlowFactory::getOpticalFlow(config,c.cal);
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> states;
    tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> maps;
    states.set_capacity(4);maps.set_capacity(4);
    if(flow)flow->input_queue.set_capacity(2);
    vio->vision_data_queue.set_capacity(2);
    vio->imu_data_queue.set_capacity(256);
    vio->out_state_queue=&states;vio->out_vis_queue=&maps;
    struct SensorEvent {int64_t stamp;bool image;};
    tbb::concurrent_bounded_queue<SensorEvent> events;
    tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> tracked;
    events.set_capacity(4);tracked.set_capacity(2);
    if(flow)flow->output_queue=&tracked;
    std::thread event_dispatcher([&] {
      for(;;) {
        SensorEvent event;events.pop(event);
        basalt::OpticalFlowResult::Ptr result;
        if(event.image) {
          tracked.pop(result);
          if(!result || result->t_ns!=event.stamp)fail("frontend_event_order");
        }else {
          result=std::make_shared<basalt::OpticalFlowResult>();result->t_ns=event.stamp;
        }
        vio->vision_data_queue.push(result);
      }
    });
    std::thread state_reader([&] {
      for(;;) {
        basalt::PoseVelBiasState<double>::Ptr s;states.pop(s);
        if(!s)fail("estimator_finished");
        if(!s->T_w_i.matrix().allFinite() || !s->vel_w_i.allFinite() ||
           !s->bias_gyro.allFinite() || !s->bias_accel.allFinite())fail("invalid_estimate");
        Json ping=nullptr,candidate=nullptr,initialization=nullptr,acoustic=nullptr,dvl=nullptr,pressure=nullptr,ground=nullptr;bool owner_received=false,fused=false;
        {std::lock_guard<std::mutex> lock(ping_mutex);
         auto surface=ground_reports.find(s->t_ns);if(surface!=ground_reports.end())ground=surface->second;
         ground_reports.erase(ground_reports.begin(),ground_reports.upper_bound(s->t_ns));
         auto found_dvl=dvl_reports.find(s->t_ns);if(found_dvl!=dvl_reports.end())dvl=found_dvl->second;
         dvl_reports.erase(dvl_reports.begin(),dvl_reports.upper_bound(s->t_ns));
         auto depth=pressure_reports.find(s->t_ns);if(depth!=pressure_reports.end())pressure=depth->second;
         pressure_reports.erase(pressure_reports.begin(),pressure_reports.upper_bound(s->t_ns));
         auto sonar_report=sonar_reports.find(s->t_ns);
         if(sonar_report!=sonar_reports.end())acoustic=sonar_report->second;
         sonar_reports.erase(sonar_reports.begin(),sonar_reports.upper_bound(s->t_ns));
         auto diagnostic=initialization_windows.find(s->t_ns);
         if(diagnostic!=initialization_windows.end())initialization=diagnostic->second;
         initialization_windows.erase(initialization_windows.begin(),initialization_windows.upper_bound(s->t_ns));
         auto found=image_pings.find(s->t_ns);
         if(found!=image_pings.end()) {ping={{"stamp_ns",found->first},{"range_m",found->second.range}};owner_received=found->second.owner_received;candidate=found->second.candidate;fused=found->second.fused;}
         image_pings.erase(image_pings.begin(),image_pings.upper_bound(s->t_ns));}
        auto q=s->T_w_i.unit_quaternion();
        emit({{"type","state"},{"stamp_ns",s->t_ns},
              {"state_clock",state_clock},{"map_qualified",false},{"dvl",dvl},{"pressure",pressure},{"ground",ground},
              {"position",vector3(s->T_w_i.translation())},
              {"orientation_xyzw",{q.x(),q.y(),q.z(),q.w()}},
              {"velocity",vector3(s->vel_w_i)},{"initialization_window",initialization},{"sonar",acoustic},
              {"gyro_bias",vector3(s->bias_gyro)},
              {"accel_bias",vector3(s->bias_accel)},{"ping_observation",ping},{"ping_owner_received",owner_received},{"ping_candidate",candidate},{"ping_fused",fused}});
      }
    });
    std::thread map_reader([&] {
      LandmarkArchive archive;
      size_t traced_frames=0;
      for(;;) {
        basalt::VioVisualizationData::Ptr data;maps.pop(data);
        if(!data)fail("estimator_finished");
        // Auxiliary IMU states provide no new visual observation or map age.
        if(data->opt_flow_res && data->opt_flow_res->observations.empty())continue;
        auto full=std::make_shared<Snapshot>();full->stamp=data->t_ns;
        std::vector<LandmarkArchive::Point> active;
        for(const auto& point:data->points)active.push_back({float(point.x()),float(point.y()),float(point.z())});
        try {
          archive.update(data->t_ns,data->point_ids,active,
              mapping_inliers(*data,c.processed_width,c.processed_height,config.vio_obs_huber_thresh));
        } catch(const std::exception& error) {fail(error.what());}
        full->points=archive.points();
        if(record_visual_tracks && traced_frames<=1800) {
          Json trace={{"stamp_ns",data->t_ns},{"width",c.processed_width},{"height",c.processed_height}};
          if(traced_frames==1800)trace["truncated"]=true;
          else {
            std::set<int> mature;
            for(const auto& item:archive.active_mature(std::set<int>(data->point_ids.begin(),data->point_ids.end())))mature.insert(item.first);
            trace["tracks"]=visual_track_trace(*data,c.processed_width,c.processed_height,config.vio_obs_huber_thresh,mature);
          }
          visual_trace<<trace.dump()<<'\n';visual_trace.flush();++traced_frames;
          if(!visual_trace)fail("visual_trace_write_failed");
        }
        {std::lock_guard<std::mutex> lock(snapshot_mutex);snapshot=full;}
        Json points=Json::array();
        const size_t count=full->points.size();
        for(const auto& point:LandmarkArchive::display_sample(full->points))points.push_back(point);
        emit({{"type","map"},{"stamp_ns",data->t_ns},
              {"landmarks_window",data->points.size()},{"landmarks_accumulated",count},
              {"archive_evicted",archive.evicted()},{"archive_rejected",archive.rejected()},{"points",points},
              {"visual_support",visual_support(*data,c.processed_width,c.processed_height,config.vio_obs_huber_thresh)},
              {"observations",data->opt_flow_res && !data->opt_flow_res->observations.empty() ? data->opt_flow_res->observations[0].size() : 0}});
      }
    });
    if(!require_gravity_seed)vio->initialize(Vec3::Zero(),Vec3::Zero());
    int64_t gravity_stamp=0;Vec3 gravity_direction=Vec3::Zero();
    emit({{"type","ready"},{"state_clock",state_clock},{"gravity_seed_transport",sonar_clock ? "first_sensor_direction_v1" : "first_image_direction_v1"},{"gravity_seed_required",require_gravity_seed},{"protocol",1},{"inertial_fused",true},{"map_export",sonar_clock ? Json(nullptr) : Json("segment_landmark_archive_ply_1")},
          {"sonar_clock_transport",sonar_clock ? Json("ordered_sonar_state_v1") : Json(nullptr)},
          {"imu_noise_model","continuous_white_midpoint_acceleration_bridge_v1"},
          {"dvl_measurement_transport",dvl_enabled ? Json("body_velocity_gyro_covariance_v1") : Json(nullptr)},
          {"pressure_measurement_transport",pressure_enabled ? Json("height_unknown_datum_v1") : Json(nullptr)},
          {"sensor_state_transport",ordered_auxiliary ? Json("ordered_camera_auxiliary_states_v1") : Json(nullptr)},
          {"sonar_snapshot_transport",sonar_enabled && !ground_enabled ? Json("exact_image_track_snapshot_v1") : Json(nullptr)},{"joint_sonar_enabled",sonar_enabled && !ground_enabled},{"sonar_auxiliary_transport",auxiliary_enabled ? Json("ordered_imu_state_v1") : Json(nullptr)},
          {"terrain_ping_transport",terrain_enabled ? Json("exact_sensor_range_v1") : Json(nullptr)},
          {"ground_transport",ground_enabled ? Json("exact_beam_entries_v1") : Json(nullptr)},
          {"image_ping_transport","exact_stamp_range_v1"},{"ping_fused",false},{"joint_ping_enabled",joint_enabled},
          {"opencv_threads",1},{"estimation_threads",2},
          {"keyframe_hold_frames",config.vio_min_frames_after_kf},
          {"feature_grid",config.optical_flow_detection_grid_size},{"max_recent_states",config.vio_max_states},
          {"raw_resolution",{c.width,c.height}},
          {"processed_resolution",{c.processed_width,c.processed_height}},
          {"scope",sonar_clock ? (ground_enabled ? "experimental raw sonar ground-plane and inertial estimation; no camera frontend" : terrain_enabled ? "experimental sonar-inertial with local planar range hypotheses; no camera frontend" : "experimental sonar-inertial state estimation; no terrain or camera frontend") : "experimental local visual-inertial odometry with sparse landmark archive"}});
    int64_t imu_stamp=0,image_stamp=0,event_stamp=0,dvl_stamp=0,pressure_stamp=0,range_stamp=0,ground_stamp=0;bool auxiliary_since_image=false;
    // Decode failures must be reported before unwinding joinable reader
    // threads. Otherwise std::thread destruction terminates the process and
    // hides the actual invalid-input reason from the owning segment.
    try { for(;;) {
      char header[8];std::cin.read(header,8);
      if(!std::cin)std::_Exit(0);
      auto type=u32(header),size=u32(header+4);
      if(type==3 && size==0)std::_Exit(0);
      if((type!=1 && type!=2 && type!=4 && type!=5 && type!=6 && type!=7 && type!=8 && type!=9 && type!=10 && type!=11 && type!=12) ||
          (type==7 && !sonar_enabled) || (type==8 && !auxiliary_enabled && !sonar_clock && !ordered_auxiliary) ||
          (type==9 && !dvl_enabled) || (type==10 && !pressure_enabled) || (type==11 && !terrain_enabled) || (type==12 && !ground_enabled) || size>(type==12 ? 2608 : type==7 ? posim_slam::sonarSnapshotLimit : type==9 ? 216 : std::max(uint32_t(56),uint32_t(c.width*c.height+32))))
        fail("packet_size_or_type");
      if(terrain_enabled && type==5)fail("conflicting_ping_transport");
      if(sonar_clock && (type==2 || type==5))fail("camera_packet_in_sonar_clock");
      if(ground_enabled && type==7)fail("ground_and_point_factors_are_exclusive");
      std::vector<char> bytes(size);std::cin.read(bytes.data(),size);
      if(!std::cin)fail("incomplete_packet");
      if(type==12) {
        auto packet=posim_slam::decodeGroundPacket(bytes,*ground_calibration,sonar_calibration->horizontal_fov);
        const auto stamp=packet.observation.stamp;
        if(stamp<=event_stamp || stamp<=ground_stamp)fail("reordered_ground_observation");
        std::lock_guard<std::mutex> lock(ping_mutex);
        if(pending_ground.size()>=8)fail("pending_ground_backlog");
        ground_stamp=stamp;pending_ground.emplace(stamp,std::move(packet));
      } else if(type==11) {
        if(size!=16)fail("terrain_ping_packet_size");
        const int64_t stamp=int64_t(u64(bytes.data()));const double range=number(bytes.data()+8);
        if(stamp<=0 || stamp<=event_stamp || stamp<=range_stamp || !std::isfinite(range) ||
           range<ping_calibration->minimum || range>ping_calibration->maximum)fail("invalid_terrain_ping_observation");
        std::lock_guard<std::mutex> lock(ping_mutex);
        if(image_pings.size()>=8 || !image_pings.emplace(stamp,PendingPing{range,false}).second)fail("terrain_ping_backlog");
        range_stamp=stamp;
      } else if(type==10) {
        auto observation=posim_slam::decodePressureObservation(bytes,*pressure_calibration);
        if(observation.stamp<=event_stamp || observation.stamp<=pressure_stamp)fail("reordered_pressure_observation");
        std::lock_guard<std::mutex> lock(ping_mutex);
        if(pending_pressure.size()>=8)fail("pending_pressure_backlog");
        pressure_stamp=observation.stamp;pending_pressure.emplace(observation.stamp,std::move(observation));
      } else if(type==9) {
        auto observation=posim_slam::decodeDvlObservation(bytes,*dvl_calibration);
        if(observation.stamp<=event_stamp || observation.stamp<=dvl_stamp)fail("reordered_dvl_observation");
        std::lock_guard<std::mutex> lock(ping_mutex);
        if(pending_dvl.size()>=8)fail("pending_dvl_backlog");
        dvl_stamp=observation.stamp;pending_dvl.emplace(observation.stamp,std::move(observation));
      } else if(type==7) {
        auto packet=posim_slam::decodeSonarSnapshot(bytes);
        if(packet.stamp<=event_stamp)fail("sonar_snapshot_after_image");
        std::lock_guard<std::mutex> lock(ping_mutex);
        if(image_sonar.size()>=8 || !image_sonar.emplace(packet.stamp,std::move(packet)).second)
          fail("duplicate_or_excess_sonar_snapshots");
      } else if(type==8) {
        if(size!=8)fail("auxiliary_sonar_packet_size");
        const int64_t stamp=int64_t(u64(bytes.data()));
        if(sonar_clock ? (stamp<=0 || !imu_stamp || stamp<=event_stamp || (event_stamp && stamp-event_stamp>sonar_gap)) :
            (!image_stamp || stamp<=event_stamp || stamp-image_stamp>500000000 || (auxiliary_since_image && !ordered_auxiliary)))
          fail("auxiliary_sonar_clock_or_capacity");
        {
          std::lock_guard<std::mutex> lock(ping_mutex);
          const auto ground=pending_ground.upper_bound(event_stamp);
          if(ground!=pending_ground.end() && ground->first<stamp)fail("ground_observation_missing_state");
          if(terrain_enabled) {
            const auto range=image_pings.upper_bound(event_stamp);
            if(range!=image_pings.end() && range->first<stamp)fail("terrain_ping_missing_state");
          }
          const auto next_dvl=pending_dvl.upper_bound(event_stamp);
          if(next_dvl!=pending_dvl.end() && next_dvl->first<stamp)fail("dvl_observation_missing_state");
          const auto depth=pending_pressure.upper_bound(event_stamp);
          if(depth!=pending_pressure.end() && depth->first<stamp)fail("pressure_observation_missing_state");
          const auto next=image_sonar.upper_bound(event_stamp);
          if(next!=image_sonar.end() && next->first<stamp)fail("sonar_snapshot_missing_state");
        }
        if(sonar_clock && !event_stamp) {
          if(gravity_stamp!=stamp)fail("gravity_seed_sensor_stamp");
          const Eigen::Quaterniond q=Eigen::Quaterniond::FromTwoVectors(gravity_direction,Vec3::UnitZ());
          vio->initialize(stamp,Sophus::SE3d(q,Vec3::Zero()),Vec3::Zero(),Vec3::Zero(),Vec3::Zero());
        }
        if(!events.try_push({stamp,false}))fail("sensor_event_backlog");
        event_stamp=stamp;auxiliary_since_image=true;
      } else if(type==6) {
        if(size!=32 || !require_gravity_seed || event_stamp || gravity_stamp)fail("gravity_seed_contract");
        gravity_stamp=int64_t(u64(bytes.data()));
        for(int k=0;k<3;++k)gravity_direction[k]=number(bytes.data()+8+k*8);
        if(gravity_stamp<=0 || !gravity_direction.allFinite() || std::abs(gravity_direction.norm()-1)>1e-6)
          fail("invalid_gravity_seed");
      } else if(type==4) {
        if(size!=8)fail("snapshot_request_size");
        const uint64_t request=u64(bytes.data());
        if(!request)fail("snapshot_request_id");
        std::shared_ptr<const Snapshot> full;
        {std::lock_guard<std::mutex> lock(snapshot_mutex);full=snapshot;}
        if(!full || full->points.empty()) {
          emit({{"type","export"},{"request_id",request},{"success",false},{"reason","map_unavailable"}});
          continue;
        }
        const fs::path temporary=export_file.string()+".tmp";
        if(fs::is_symlink(export_file) || fs::is_symlink(temporary))fail("snapshot_path");
        std::ofstream file(temporary,std::ios::binary|std::ios::trunc);
        if(!file)fail("snapshot_unwritable");
        file<<"ply\nformat binary_little_endian 1.0\n"
            <<"comment scope accumulated_local_vio_landmarks\n"
            <<"comment request_id "<<request<<"\ncomment stamp_ns "<<full->stamp<<'\n'
            <<"element vertex "<<full->points.size()<<"\nproperty float x\nproperty float y\nproperty float z\nend_header\n";
        // The wire/file format is little endian even on other host byte orders.
        for(const auto& point:full->points)for(float coordinate:point) {
          uint32_t bits;std::memcpy(&bits,&coordinate,4);
          char bytes[4];for(int i=0;i<4;++i)bytes[i]=char((bits>>(8*i))&255);
          file.write(bytes,4);
        }
        file.flush();if(!file)fail("snapshot_write_failed");file.close();
        fs::rename(temporary,export_file);
        emit({{"type","export"},{"request_id",request},{"success",true},
              {"stamp_ns",full->stamp},{"points",full->points.size()}});
      } else if(type==1) {
        if(size!=56)fail("imu_packet_size");
        auto d=std::make_shared<basalt::ImuData<double>>();
        d->t_ns=int64_t(u64(bytes.data()));
        for(int k=0;k<3;++k){d->gyro[k]=number(bytes.data()+8+k*8);d->accel[k]=number(bytes.data()+32+k*8);}
        if(d->t_ns<=0 || !d->gyro.allFinite() || !d->accel.allFinite() ||
           d->gyro.norm()>35 || d->accel.norm()>200)fail("invalid_imu");
        if(imu_stamp && (d->t_ns<=imu_stamp || d->t_ns-imu_stamp>c.max_gap))fail("imu_gap_or_reset");
        imu_stamp=d->t_ns;
        if(!vio->imu_data_queue.try_push(d))fail("imu_backlog");
      } else {
        const int offset=type==5 ? 32 : 16;
        if(size!=uint32_t(c.width*c.height+offset))fail("image_packet_size");
        int64_t stamp=int64_t(u64(bytes.data()));
        if(u32(bytes.data()+8)!=uint32_t(c.width) || u32(bytes.data()+12)!=uint32_t(c.height))fail("image_shape");
        if(stamp<=0 || !imu_stamp || (image_stamp && (stamp<=event_stamp || stamp-image_stamp>500000000)))fail("image_gap_or_reset");
        {
          std::lock_guard<std::mutex> lock(ping_mutex);
          if(terrain_enabled) {
            const auto range=image_pings.upper_bound(event_stamp);
            if(range!=image_pings.end() && range->first<stamp)fail("terrain_ping_missing_state");
          }
          const auto next_dvl=pending_dvl.upper_bound(event_stamp);
          if(next_dvl!=pending_dvl.end() && next_dvl->first<stamp)fail("dvl_observation_missing_state");
          const auto next=image_sonar.upper_bound(event_stamp);
          if(next!=image_sonar.end() && next->first<stamp)fail("sonar_snapshot_missing_image");
          const auto depth=pending_pressure.upper_bound(event_stamp);
          if(depth!=pending_pressure.end() && depth->first<stamp)fail("pressure_observation_missing_state");
        }
        if(type==5) {
          const auto ping_stamp=int64_t(u64(bytes.data()+16));
          const auto range=number(bytes.data()+24);
          if(ping_stamp!=stamp || !std::isfinite(range) || range<=0 || range>1000)
            fail("invalid_exact_image_ping");
          std::lock_guard<std::mutex> lock(ping_mutex);
          if(image_pings.size()>=64)fail("image_ping_backlog");
          image_pings.emplace(stamp,PendingPing{range,false});
        }
        if(!image_stamp && require_gravity_seed) {
          if(gravity_stamp!=stamp)fail("gravity_seed_image_stamp");
          const Eigen::Quaterniond q=Eigen::Quaterniond::FromTwoVectors(gravity_direction,Vec3::UnitZ());
          vio->initialize(stamp,Sophus::SE3d(q,Vec3::Zero()),Vec3::Zero(),Vec3::Zero(),Vec3::Zero());
        }
        image_stamp=stamp;event_stamp=stamp;auxiliary_since_image=false;
        cv::Mat raw(c.height,c.width,CV_8UC1,bytes.data()+offset);
        auto pixels=estimator_image(c,raw);
        auto data=std::make_shared<basalt::OpticalFlowInput>();
        data->t_ns=stamp;data->img_data.resize(1);data->img_data[0].img=pixels;
        if(!flow->input_queue.try_push(data))fail("image_backlog");
        if(!events.try_push({stamp,true}))fail("sensor_event_backlog");
      }
    }} catch(const std::exception& error) {fail(error.what());}
  } catch(const std::exception& error) {fail(error.what());}
}

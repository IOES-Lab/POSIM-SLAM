// SPDX-License-Identifier: Apache-2.0
// Bounded live pixel/raw-IMU worker. Reset by replacing this process, never by
// reconnecting a new clock or calibration to an old estimator window.
#include "calibration.hpp"
#include "visual_support.hpp"
#include "landmark_archive.hpp"
#include "ping_input.hpp"
#include "ping_factors.hpp"
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
    std::map<int64_t,Json> initialization_windows;
    std::mutex snapshot_mutex;
    std::shared_ptr<const Snapshot> snapshot;
    const fs::path export_file=fs::absolute(argv[1]).parent_path()/"worker-map.ply";
    cv::setNumThreads(1);
    tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
    auto config=processing_config(c);
    auto vio=basalt::VioEstimatorFactory::getVioEstimator(
        config,c.cal,Vec3(0,0,-c.gravity),true,true);
    std::optional<posim_slam::PingInputCalibration> ping_calibration;
    const auto ping_file=fs::path(argv[1]).parent_path()/"ping-calibration.json";
    if(fs::exists(ping_file))ping_calibration=posim_slam::pingInputCalibration(read_json(ping_file));
    auto owner=std::dynamic_pointer_cast<basalt::SqrtKeypointVioEstimator<double>>(vio);
    if(!owner)throw std::runtime_error("unsupported_ping_estimator_owner");
    const auto input_settings=read_json(argv[1]);
    const bool record_visual_tracks=input_settings.value("record_visual_tracks",false);
    std::ofstream visual_trace;
    if(record_visual_tracks) {
      auto path=fs::path(argv[1]);path.replace_extension("visual-tracks.jsonl");
      visual_trace.open(path);
      if(!visual_trace)throw std::runtime_error("visual_trace_open_failed");
    }
    const bool joint_enabled=input_settings.value("enable_joint_ping",false);
    const auto loss_name=input_settings.value("ping_range_loss",std::string("cauchy"));
    if(loss_name!="cauchy" && loss_name!="huber")throw std::runtime_error("invalid_ping_range_loss");
    const auto range_loss=loss_name=="cauchy"?posim_slam::PingRangeLoss::Cauchy:posim_slam::PingRangeLoss::Huber;
    std::optional<double> range_scale=3.;
    if(input_settings.contains("ping_range_loss_scale")) {
      if(input_settings.at("ping_range_loss_scale").is_null())range_scale.reset();
      else range_scale=positive(input_settings,"ping_range_loss_scale");
    }
    double ping_bias=0.,ping_ray_sigma=0.;int next_plane_id=0;
    if(joint_enabled) {
      if(!ping_calibration)throw std::runtime_error("joint_ping_requires_calibration");
      const auto settings=read_json(ping_file);
      ping_bias=settings.at("range_bias").get<double>();
      ping_ray_sigma=positive(settings,"per_ray_noise_stddev");
      if(!std::isfinite(ping_bias))throw std::runtime_error("invalid_ping_bias");
      owner->posim_ping_factors=std::make_shared<posim_slam::PingFactorStore>(
        static_cast<basalt::BundleAdjustmentBase<double>&>(*owner));
    }
    owner->posim_after_joint=[&](auto& estimator,int64_t stamp) {
      std::lock_guard<std::mutex> lock(ping_mutex);
      // Copy on the estimator thread before marginalization. Output threads
      // never inspect mutable optimizer state. This remains diagnostic only.
      const auto& window=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator).frame_states;
      if(window.size()>128 || (!window.empty() && stamp-window.begin()->first>30'000'000'000LL))
        fail("visual_initialization_window_exhausted");
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
      auto found=image_pings.find(stamp);
      if(found!=image_pings.end() && estimator.posim_ping_factors)
        found->second.fused=estimator.posim_ping_factors->lastObservation()==stamp && estimator.posim_joint_last_accepted>0;
    };
    owner->posim_before_marginalization=[&](auto& estimator,int64_t stamp) {
      auto& base=static_cast<basalt::BundleAdjustmentBase<double>&>(estimator);
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
    auto flow=basalt::OpticalFlowFactory::getOpticalFlow(config,c.cal);
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> states;
    tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> maps;
    states.set_capacity(4);maps.set_capacity(4);
    flow->input_queue.set_capacity(2);
    vio->vision_data_queue.set_capacity(2);
    vio->imu_data_queue.set_capacity(256);
    vio->out_state_queue=&states;vio->out_vis_queue=&maps;
    flow->output_queue=&vio->vision_data_queue;
    std::thread state_reader([&] {
      for(;;) {
        basalt::PoseVelBiasState<double>::Ptr s;states.pop(s);
        if(!s)fail("estimator_finished");
        if(!s->T_w_i.matrix().allFinite() || !s->vel_w_i.allFinite() ||
           !s->bias_gyro.allFinite() || !s->bias_accel.allFinite())fail("invalid_estimate");
        Json ping=nullptr,candidate=nullptr,initialization=nullptr;bool owner_received=false,fused=false;
        {std::lock_guard<std::mutex> lock(ping_mutex);
         auto diagnostic=initialization_windows.find(s->t_ns);
         if(diagnostic!=initialization_windows.end())initialization=diagnostic->second;
         initialization_windows.erase(initialization_windows.begin(),initialization_windows.upper_bound(s->t_ns));
         auto found=image_pings.find(s->t_ns);
         if(found!=image_pings.end()) {ping={{"stamp_ns",found->first},{"range_m",found->second.range}};owner_received=found->second.owner_received;candidate=found->second.candidate;fused=found->second.fused;}
         image_pings.erase(image_pings.begin(),image_pings.upper_bound(s->t_ns));}
        auto q=s->T_w_i.unit_quaternion();
        emit({{"type","state"},{"stamp_ns",s->t_ns},
              {"position",vector3(s->T_w_i.translation())},
              {"orientation_xyzw",{q.x(),q.y(),q.z(),q.w()}},
              {"velocity",vector3(s->vel_w_i)},{"initialization_window",initialization},
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
    const bool require_gravity_seed=input_settings.value("require_gravity_seed",false);
    if(!require_gravity_seed)vio->initialize(Vec3::Zero(),Vec3::Zero());
    int64_t gravity_stamp=0;Vec3 gravity_direction=Vec3::Zero();
    emit({{"type","ready"},{"gravity_seed_transport","first_image_direction_v1"},{"gravity_seed_required",require_gravity_seed},{"protocol",1},{"inertial_fused",true},{"map_export","segment_landmark_archive_ply_1"},
          {"image_ping_transport","exact_stamp_range_v1"},{"ping_fused",false},{"joint_ping_enabled",joint_enabled},
          {"opencv_threads",1},{"estimation_threads",2},
          {"keyframe_hold_frames",config.vio_min_frames_after_kf},
          {"feature_grid",config.optical_flow_detection_grid_size},
          {"raw_resolution",{c.width,c.height}},
          {"processed_resolution",{c.processed_width,c.processed_height}},
          {"scope","experimental local visual-inertial odometry with sparse landmark archive"}});
    int64_t imu_stamp=0,image_stamp=0;
    for(;;) {
      char header[8];std::cin.read(header,8);
      if(!std::cin)std::_Exit(0);
      auto type=u32(header),size=u32(header+4);
      if(type==3 && size==0)std::_Exit(0);
      if((type!=1 && type!=2 && type!=4 && type!=5 && type!=6) || size>uint32_t(c.width*c.height+32))fail("packet_size_or_type");
      std::vector<char> bytes(size);std::cin.read(bytes.data(),size);
      if(!std::cin)fail("incomplete_packet");
      if(type==6) {
        if(size!=32 || !require_gravity_seed || image_stamp || gravity_stamp)fail("gravity_seed_contract");
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
        if(stamp<=0 || !imu_stamp || (image_stamp && (stamp<=image_stamp || stamp-image_stamp>500000000)))fail("image_gap_or_reset");
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
        image_stamp=stamp;
        cv::Mat raw(c.height,c.width,CV_8UC1,bytes.data()+offset);
        auto pixels=estimator_image(c,raw);
        auto data=std::make_shared<basalt::OpticalFlowInput>();
        data->t_ns=stamp;data->img_data.resize(1);data->img_data[0].img=pixels;
        if(!flow->input_queue.try_push(data))fail("image_backlog");
      }
    }
  } catch(const std::exception& error) {fail(error.what());}
}

// SPDX-License-Identifier: Apache-2.0
// Bounded live pixel/raw-IMU worker. Reset by replacing this process, never by
// reconnecting a new clock or calibration to an old estimator window.
#include "calibration.hpp"
#include "visual_support.hpp"
#include "landmark_archive.hpp"
#include <tbb/global_control.h>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>
#include <array>
#include <iomanip>

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
    std::mutex snapshot_mutex;
    std::shared_ptr<const Snapshot> snapshot;
    const fs::path export_file=fs::absolute(argv[1]).parent_path()/"worker-map.ply";
    cv::setNumThreads(1);
    tbb::global_control workers(tbb::global_control::max_allowed_parallelism,2);
    auto config=processing_config(c);
    auto vio=basalt::VioEstimatorFactory::getVioEstimator(
        config,c.cal,Vec3(0,0,-c.gravity),true,true);
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
        auto q=s->T_w_i.unit_quaternion();
        emit({{"type","state"},{"stamp_ns",s->t_ns},
              {"position",vector3(s->T_w_i.translation())},
              {"orientation_xyzw",{q.x(),q.y(),q.z(),q.w()}},
              {"velocity",vector3(s->vel_w_i)},
              {"gyro_bias",vector3(s->bias_gyro)},
              {"accel_bias",vector3(s->bias_accel)}});
      }
    });
    std::thread map_reader([&] {
      LandmarkArchive archive;
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
    vio->initialize(Vec3::Zero(),Vec3::Zero());
    emit({{"type","ready"},{"protocol",1},{"inertial_fused",true},{"map_export","segment_landmark_archive_ply_1"},
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
      if((type!=1 && type!=2 && type!=4) || size>uint32_t(c.width*c.height+16))fail("packet_size_or_type");
      std::vector<char> bytes(size);std::cin.read(bytes.data(),size);
      if(!std::cin)fail("incomplete_packet");
      if(type==4) {
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
        if(size!=uint32_t(c.width*c.height+16))fail("image_packet_size");
        int64_t stamp=int64_t(u64(bytes.data()));
        if(u32(bytes.data()+8)!=uint32_t(c.width) || u32(bytes.data()+12)!=uint32_t(c.height))fail("image_shape");
        if(stamp<=0 || !imu_stamp || (image_stamp && (stamp<=image_stamp || stamp-image_stamp>500000000)))fail("image_gap_or_reset");
        image_stamp=stamp;
        cv::Mat raw(c.height,c.width,CV_8UC1,bytes.data()+16);
        auto pixels=estimator_image(c,raw);
        auto data=std::make_shared<basalt::OpticalFlowInput>();
        data->t_ns=stamp;data->img_data.resize(1);data->img_data[0].img=pixels;
        if(!flow->input_queue.try_push(data))fail("image_backlog");
      }
    }
  } catch(const std::exception& error) {fail(error.what());}
}

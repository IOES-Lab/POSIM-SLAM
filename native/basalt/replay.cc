#include "calibration.hpp"
#include "ping_input.hpp"
#include "landmark_archive.hpp"
#include "visual_support.hpp"
#include <condition_variable>
#include <mutex>
// SPDX-License-Identifier: Apache-2.0
// Replay pixels and raw IMU only. No simulator truth or orientation input.
#include <basalt/vi_estimator/vio_estimator.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tbb/global_control.h>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

using Json = nlohmann::json;
using Vec3 = Eigen::Vector3d;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc != 3 && (argc != 4 || std::string(argv[3])!="--ping-patches")) return 2;
  try {
    fs::path input = fs::canonical(argv[1]), destination = argv[2];
    if (fs::exists(destination)) throw std::runtime_error("output_exists");
    auto settings = input_calibration(input / "calibration.json");
    auto cal = settings.cal;
    int width=settings.width,height=settings.height;
    double gravity=settings.gravity;
    int64_t max_gap=settings.max_gap;

    // Validate all timing and input files before starting any worker thread.
    std::vector<basalt::ImuData<double>::Ptr> measurements;
    std::ifstream imu(input / "imu.csv");
    std::string line;
    std::getline(imu, line);
    if (line != "stamp_ns,gx,gy,gz,ax,ay,az") throw std::runtime_error("imu_header");
    while (std::getline(imu, line)) {
      if (measurements.size() >= 120000) throw std::runtime_error("imu_input_limit");
      for (char& ch : line) if (ch == ',') ch = ' ';
      std::istringstream row(line);
      auto d = std::make_shared<basalt::ImuData<double>>();
      if (!(row >> d->t_ns >> d->gyro.x() >> d->gyro.y() >> d->gyro.z()
                >> d->accel.x() >> d->accel.y() >> d->accel.z()))
        throw std::runtime_error("invalid_imu_row");
      std::string extra;
      if (row >> extra) throw std::runtime_error("extra_imu_columns");
      if (d->t_ns <= 0 || !d->gyro.allFinite() || !d->accel.allFinite() ||
          d->gyro.norm() > 35 || d->accel.norm() > 200)
        throw std::runtime_error("invalid_imu_measurement");
      if (!measurements.empty()) {
        int64_t dt = d->t_ns - measurements.back()->t_ns;
        if (dt <= 0 || dt > max_gap) throw std::runtime_error("imu_gap_or_reset");
      }
      measurements.push_back(d);
    }
    auto images = read_json(input / "frames.json");
    if (!images.is_array() || images.size() < 5 || images.size() > 900 ||
        measurements.size() < 10) throw std::runtime_error("insufficient_input");
    int64_t previous = 0;
    std::vector<fs::path> paths;
    for (const auto& frame : images) {
      int64_t stamp = frame.at("stamp_ns");
      fs::path relative = frame.at("file").get<std::string>();
      auto file = fs::canonical(input / relative);
      auto inside = fs::relative(file, input).generic_string();
      if (relative.is_absolute() || inside == ".." || inside.rfind("../", 0) == 0 ||
          stamp <= previous || (previous && stamp - previous > 500000000) ||
          stamp < measurements.front()->t_ns || stamp >= measurements.back()->t_ns)
        throw std::runtime_error("image_coverage_path_or_timing");
      auto gray = cv::imread(file.string(), cv::IMREAD_UNCHANGED);
      if (gray.type() != CV_8UC1 || gray.cols != width || gray.rows != height)
        throw std::runtime_error("image_shape_or_encoding");
      previous = stamp;
      paths.push_back(file);
    }
    // The estimator needs one real IMU knot beyond the final exposure. Do not
    // leave a long, unused IMU tail blocking a bounded producer at shutdown.
    auto tail = std::upper_bound(measurements.begin(), measurements.end(), previous,
        [](int64_t t, const auto& d) { return t < d->t_ns; });
    if (tail != measurements.end()) measurements.erase(tail + 1, measurements.end());

    const bool inspect_ping=argc==4;
    Sophus::SE3d ping_mount;Eigen::MatrixXd ping_rays;double ping_min=0,ping_max=0;
    std::map<int64_t,double> ping_measurements;
    std::ofstream patch_file;
    if(inspect_ping) {
      const auto p=read_json(input/"ping-calibration.json");
      const auto calibrated=posim_slam::pingInputCalibration(p);
      ping_mount=calibrated.mount;ping_rays=calibrated.rays;ping_min=calibrated.minimum;ping_max=calibrated.maximum;
      std::ifstream ping(input/"ping.csv");std::getline(ping,line);
      if(line!="stamp_ns,range_m,valid,field_of_view_rad")throw std::runtime_error("ping_header");
      int64_t previous_ping=0;
      while(std::getline(ping,line)) {
        std::stringstream row(line);std::vector<std::string> cells;std::string cell;
        while(std::getline(row,cell,','))cells.push_back(cell);
        if(cells.size()!=4)throw std::runtime_error("ping_columns");
        size_t used=0;const int64_t stamp=std::stoll(cells[0],&used);if(used!=cells[0].size())throw std::runtime_error("ping_stamp");
        const double range=std::stod(cells[1],&used);if(used!=cells[1].size())throw std::runtime_error("ping_range");
        const double fov=std::stod(cells[3],&used);
        if(used!=cells[3].size() || !std::isfinite(fov) || std::abs(fov-p.at("field_of_view").get<double>())>1e-5)
          throw std::runtime_error("ping_field_of_view");
        if(ping_measurements.size()>=120000)throw std::runtime_error("ping_input_limit");
        if(stamp<=previous_ping || (cells[2]!="0" && cells[2]!="1"))throw std::runtime_error("ping_clock_or_validity");
        previous_ping=stamp;
        if(cells[2]=="1") {
          if(!std::isfinite(range) || range<ping_min || range>ping_max)throw std::runtime_error("ping_range");
          ping_measurements.emplace(stamp,range);
        }
      }
      const fs::path patch_path=destination.string()+".patches.jsonl";
      if(fs::exists(patch_path))throw std::runtime_error("patch_output_exists");
      patch_file.open(patch_path);if(!patch_file)throw std::runtime_error("patch_output_unwritable");
    }
    std::ofstream states(destination);
    if (!states) throw std::runtime_error("output_unwritable");

    tbb::global_control workers(tbb::global_control::max_allowed_parallelism, 2);
    auto config=processing_config(settings);
    auto vio = basalt::VioEstimatorFactory::getVioEstimator(
        config, cal, Vec3(0, 0, -gravity), true, true);
    auto flow = basalt::OpticalFlowFactory::getOpticalFlow(config, cal);
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> output;
    flow->output_queue = &vio->vision_data_queue;
    vio->out_state_queue = &output;
    states << std::setprecision(17)
           << "stamp_ns,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
    size_t count = 0,patch_count=0;
    std::mutex pose_mutex;std::condition_variable pose_ready;
    std::map<int64_t,Sophus::SE3d> poses;
    tbb::concurrent_bounded_queue<basalt::VioVisualizationData::Ptr> maps;
    maps.set_capacity(4);std::thread patch_reader;std::exception_ptr patch_failure;
    if(inspect_ping) {
      vio->out_vis_queue=&maps;
      patch_reader=std::thread([&] {
        LandmarkArchive archive;
        try {
          for(;;) {
            basalt::VioVisualizationData::Ptr data;maps.pop(data);if(!data)break;
            Sophus::SE3d pose;
            {std::unique_lock<std::mutex> lock(pose_mutex);
             if(!pose_ready.wait_for(lock,std::chrono::seconds(10),[&]{return poses.count(data->t_ns)>0;}))
               throw std::runtime_error("patch_pose_missing");
             pose=poses.at(data->t_ns);poses.erase(data->t_ns);}
            const auto inliers=mapping_inliers(*data,settings.processed_width,settings.processed_height,config.vio_obs_huber_thresh);
            std::vector<LandmarkArchive::Point> coordinates;
            for(const auto& point:data->points)coordinates.push_back({float(point.x()),float(point.y()),float(point.z())});
            archive.update(data->t_ns,data->point_ids,coordinates,inliers,unqualified_visual_points(*data));
            std::vector<posim_slam::PingPatchPoint> candidates;
            for(const auto& [id,point]:archive.active_mature(inliers))candidates.push_back({size_t(id),Vec3(point[0],point[1],point[2])});
            Json result={{"stamp_ns",data->t_ns},{"active_mature_points",candidates.size()},{"ping_fused",false}};
            const auto measurement=ping_measurements.find(data->t_ns);
            if(measurement==ping_measurements.end())result["reason"]="no_exact_ping";
            else {
              posim_slam::PingPatchDiagnostics diagnostics;
              const auto patch=posim_slam::selectPingPatch(candidates,pose,ping_mount,ping_rays,ping_min,ping_max,.03,&diagnostics);
              result["local_points"]=diagnostics.local_points;result["max_plane_inliers"]=diagnostics.max_inliers;
              result["rejections"]=diagnostics.rejected;
              Json diagnostic_points=Json::array();const size_t display=std::min(size_t(1000),candidates.size());
              for(size_t i=0;i<display;++i) {
                const auto& c=candidates[display<=1?0:i*(candidates.size()-1)/(display-1)];
                diagnostic_points.push_back({c.id,c.point.x(),c.point.y(),c.point.z()});
              }
              result["diagnostic_points"]=diagnostic_points;result["diagnostic_points_truncated"]=display<candidates.size();
              const auto rotation=pose.unit_quaternion();
              result["imu_position"]={pose.translation().x(),pose.translation().y(),pose.translation().z()};
              result["imu_orientation_xyzw"]={rotation.x(),rotation.y(),rotation.z(),rotation.w()};
              result["measured_range_m"]=measurement->second;
              if(!patch)result["reason"]="no_supported_patch";
              else {
                result["reason"]="candidate_only";result["landmark_ids"]=patch->ids;
                result["normal"]={patch->normal.x(),patch->normal.y(),patch->normal.z()};result["distance_m"]=patch->distance;
                result["rms_m"]=patch->rms;result["predicted_range_m"]=patch->predicted_range;result["support_radius_m"]=patch->support_radius;
              }
            }
            patch_file<<result.dump()<<'\n';++patch_count;
          }
        }catch(...) {
          patch_failure=std::current_exception();
          for(;;) {basalt::VioVisualizationData::Ptr data;maps.pop(data);if(!data)break;}
        }
      });
    }
    std::thread reader([&] {
      for (;;) {
        basalt::PoseVelBiasState<double>::Ptr s;
        output.pop(s);
        if (!s) break;
        if(inspect_ping) {std::lock_guard<std::mutex> lock(pose_mutex);poses.emplace(s->t_ns,s->T_w_i);pose_ready.notify_all();}
        const auto& p = s->T_w_i.translation();
        auto q = s->T_w_i.unit_quaternion();
        states << s->t_ns;
        for (double value : {p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w(),
                             s->vel_w_i.x(), s->vel_w_i.y(), s->vel_w_i.z(),
                             s->bias_gyro.x(), s->bias_gyro.y(), s->bias_gyro.z(),
                             s->bias_accel.x(), s->bias_accel.y(), s->bias_accel.z()})
          states << ',' << value;
        states << '\n';
        ++count;
      }
    });
    vio->initialize(Vec3::Zero(), Vec3::Zero());
    std::thread inertial([&] {
      for (const auto& d : measurements) vio->imu_data_queue.push(d);
      vio->imu_data_queue.push(nullptr);
    });
    for (size_t i = 0; i < images.size(); ++i) {
      auto gray = cv::imread(paths[i].string(), cv::IMREAD_GRAYSCALE);
      auto pixels = estimator_image(settings,gray);
      auto data = std::make_shared<basalt::OpticalFlowInput>();
      data->t_ns = images[i].at("stamp_ns");
      data->img_data.resize(1);
      data->img_data[0].img = pixels;
      flow->input_queue.push(data);
    }
    flow->input_queue.push(nullptr);
    vio->maybe_join();
    vio->drain_input_queues();
    inertial.join();
    reader.join();
    if(patch_reader.joinable())patch_reader.join();
    if(patch_failure)std::rethrow_exception(patch_failure);
    if(inspect_ping) {patch_file.flush();if(!patch_file || patch_count!=count)throw std::runtime_error("incomplete_patch_output");}
    states.flush();
    if (!states || count != images.size()) throw std::runtime_error("incomplete_output");
    std::cout << "replayed " << count << " image/IMU states; accuracy not evaluated\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

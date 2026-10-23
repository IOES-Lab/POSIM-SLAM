#include "calibration.hpp"
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
  if (argc != 3) return 2;
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

    std::ofstream states(destination);
    if (!states) throw std::runtime_error("output_unwritable");

    tbb::global_control workers(tbb::global_control::max_allowed_parallelism, 2);
    basalt::VioConfig config;
    auto vio = basalt::VioEstimatorFactory::getVioEstimator(
        config, cal, Vec3(0, 0, -gravity), true, true);
    auto flow = basalt::OpticalFlowFactory::getOpticalFlow(config, cal);
    tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> output;
    flow->output_queue = &vio->vision_data_queue;
    vio->out_state_queue = &output;
    states << std::setprecision(17)
           << "stamp_ns,x,y,z,qx,qy,qz,qw,vx,vy,vz,bgx,bgy,bgz,bax,bay,baz\n";
    size_t count = 0;
    std::thread reader([&] {
      for (;;) {
        basalt::PoseVelBiasState<double>::Ptr s;
        output.pop(s);
        if (!s) break;
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
    states.flush();
    if (!states || count != images.size()) throw std::runtime_error("incomplete_output");
    std::cout << "replayed " << count << " image/IMU states; accuracy not evaluated\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

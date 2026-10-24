// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <basalt/vi_estimator/vio_estimator.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <algorithm>
using Json = nlohmann::json;
using Vec3 = Eigen::Vector3d;
namespace fs = std::filesystem;
inline Json read_json(const fs::path& file) {
  std::ifstream stream(file);
  if (!stream) throw std::runtime_error("input_missing");
  return Json::parse(stream);
}

inline double positive(const Json& config, const char* name) {
  double value = config.at(name).get<double>();
  if (!std::isfinite(value) || value <= 0 || value > 1000)
    throw std::runtime_error(std::string("invalid_calibration:") + name);
  return value;
}

inline int integer_budget(const Json& config,const char* name,int minimum,int maximum,const char* error) {
  const auto& value=config.at(name);
  if(!value.is_number_integer())throw std::runtime_error(error);
  const auto number=value.get<int64_t>();
  if(number<minimum || number>maximum)throw std::runtime_error(error);
  return int(number);
}

struct InputCalibration {
 int width,height,processed_width,processed_height;
 basalt::Calibration<double> cal;
 double gravity;
 int64_t max_gap;
 int feature_grid;
 int max_recent_states;
};
inline InputCalibration input_calibration(const fs::path& file) {
    auto c = read_json(file);
    if (c.at("schema") != "posim-slam-vio-recording-1" ||
        c.at("orientation_used") != false)
      throw std::runtime_error("unsupported_recording");
    int width = c.at("width"), height = c.at("height");
    if (width < 32 || height < 32 || width > 1920 || height > 1080)
      throw std::runtime_error("invalid_resolution");
    int processed_width=width,processed_height=height;
    if(c.contains("max_image_pixels")) {
      int budget=integer_budget(c,"max_image_pixels",1024,1920*1080,"invalid_pixel_budget");
      double scale=std::min(1.,std::sqrt(double(budget)/(width*height)));
      processed_width=std::max(1,int(std::floor(width*scale)));
      processed_height=std::max(1,int(std::floor(height*scale)));
      if(processed_width<32 || processed_height<32)throw std::runtime_error("processing_resolution_too_small");
    }
    Eigen::Vector4d intrinsics(c.at("fx").get<double>(),c.at("fy").get<double>(),
                               c.at("cx").get<double>(),c.at("cy").get<double>());
    if(!intrinsics.allFinite() || intrinsics[0]<=0 || intrinsics[1]<=0)
      throw std::runtime_error("invalid_intrinsics");
    const double sx=double(processed_width)/width,sy=double(processed_height)/height;
    intrinsics[0]*=sx;intrinsics[1]*=sy;
    intrinsics[2]=(intrinsics[2]+.5)*sx-.5;
    intrinsics[3]=(intrinsics[3]+.5)*sy-.5;
    basalt::Calibration<double> cal;
    auto camera = basalt::GenericCamera<double>::fromString("pinhole");
    camera.setFromInit(intrinsics);
    cal.intrinsics.push_back(camera);
    cal.resolution.emplace_back(processed_width, processed_height);
    auto q = c.at("T_i_c_q_xyzw").get<std::vector<double>>();
    auto p = c.at("T_i_c_translation_m").get<std::vector<double>>();
    if (q.size() != 4 || p.size() != 3) throw std::runtime_error("invalid_mount");
    Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
    Vec3 translation(p[0], p[1], p[2]);
    if (!rotation.coeffs().allFinite() || !translation.allFinite() ||
        std::abs(rotation.norm() - 1) > 1e-6 || translation.norm() > 10)
      throw std::runtime_error("invalid_mount");
    cal.T_i_c.emplace_back(rotation, translation);
    cal.imu_update_rate = positive(c, "imu_rate_hz");
    cal.gyro_noise_std.setConstant(positive(c, "gyro_noise_density"));
    cal.accel_noise_std.setConstant(positive(c, "accel_noise_density"));
    cal.gyro_bias_std.setConstant(positive(c, "gyro_bias_random_walk"));
    cal.accel_bias_std.setConstant(positive(c, "accel_bias_random_walk"));
    double gravity = positive(c, "gravity_m_s2");
    int64_t max_gap = std::llround(positive(c, "max_imu_gap_s") * 1e9);
    if (max_gap > 100000000) throw std::runtime_error("invalid_gap_limit");

    int feature_grid=0;
    if(c.contains("max_feature_cells")) {
      int cells=integer_budget(c,"max_feature_cells",32,512,"invalid_feature_budget");
      feature_grid=std::max(16,int(std::ceil(std::sqrt(double(processed_width)*processed_height/cells))));
      if(feature_grid*2>std::min(processed_width,processed_height))
        throw std::runtime_error("feature_budget_aspect_ratio");
    }
    const int max_recent_states=c.contains("max_recent_states")
      ? integer_budget(c,"max_recent_states",3,64,"invalid_recent_state_budget")
      : (c.value("enable_auxiliary_sonar",false) ? 6 : 3);
    return {width,height,processed_width,processed_height,cal,gravity,max_gap,feature_grid,max_recent_states};
}

inline basalt::VioConfig processing_config(const InputCalibration& c) {
  basalt::VioConfig config;
  config.vio_max_states=c.max_recent_states;
  if(c.feature_grid)config.optical_flow_detection_grid_size=c.feature_grid;
  return config;
}

// Pixels and pinhole calibration share one resize contract in replay and live
// paths. Raw wire dimensions always refer to the calibrated source camera.
inline cv::Mat prepare_image(const InputCalibration& c,const cv::Mat& raw) {
  if(raw.type()!=CV_8UC1 || raw.cols!=c.width || raw.rows!=c.height)
    throw std::runtime_error("image_shape_or_encoding");
  if(c.width==c.processed_width && c.height==c.processed_height)return raw;
  cv::Mat resized;
  cv::resize(raw,resized,cv::Size(c.processed_width,c.processed_height),0,0,cv::INTER_AREA);
  return resized;
}
inline std::shared_ptr<basalt::ManagedImage<uint16_t>> estimator_image(const InputCalibration& c,const cv::Mat& raw) {
  auto gray=prepare_image(c,raw);
  auto pixels=std::make_shared<basalt::ManagedImage<uint16_t>>(gray.cols,gray.rows);
  for(int y=0;y<gray.rows;++y)for(int x=0;x<gray.cols;++x)
    (*pixels)(x,y)=uint16_t(gray.at<uint8_t>(y,x))<<8;
  return pixels;
}

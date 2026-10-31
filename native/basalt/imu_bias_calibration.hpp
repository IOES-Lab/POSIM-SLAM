// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "constrained_information.hpp"
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace posim_slam {
struct InitialImuBiasCalibration {
  Eigen::Matrix<double,6,1> mean;
  Eigen::Matrix<double,6,6> covariance;
  Eigen::MatrixXd white,hard;
  std::string source;
  static InitialImuBiasCalibration read(const nlohmann::json& v) {
    if(v.at("schema")!="posim-slam-initial-imu-bias-calibration-1" || v.at("frame")!="input_imu" || v.at("scope")!="first_state")
      throw std::invalid_argument("initial_bias_calibration_frame_scope_required");
    InitialImuBiasCalibration out;out.source=v.at("source").get<std::string>();
    if(out.source.empty() || out.source.size()>128 || out.source.find_first_not_of(" \t\r\n")==std::string::npos ||
        !v.at("mean").is_array() || v.at("mean").size()!=6 || !v.at("covariance").is_array() || v.at("covariance").size()!=6)
      throw std::invalid_argument("explicit_full_initial_bias_calibration_required");
    for(int i=0;i<6;++i) {
      if(!v.at("covariance").at(i).is_array() || v.at("covariance").at(i).size()!=6 || !v.at("mean").at(i).is_number())
        throw std::invalid_argument("explicit_full_initial_bias_calibration_required");
      out.mean[i]=v.at("mean").at(i).get<double>();
      for(int j=0;j<6;++j) {
        if(!v.at("covariance").at(i).at(j).is_number())throw std::invalid_argument("explicit_full_initial_bias_calibration_required");
        out.covariance(i,j)=v.at("covariance").at(i).at(j).get<double>();
      }
    }
    if(!out.mean.allFinite() || !out.covariance.allFinite() || out.mean.cwiseAbs().maxCoeff()>1000 || out.covariance.cwiseAbs().maxCoeff()>1000 ||
       (out.covariance-out.covariance.transpose()).cwiseAbs().maxCoeff()>1e-12)
      throw std::invalid_argument("explicit_full_initial_bias_calibration_required");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> eigen(out.covariance);
    if(eigen.info()!=Eigen::Success)throw std::invalid_argument("bias_calibration_eigensolve_failed");
    const auto values=eigen.eigenvalues();const double tolerance=64*std::numeric_limits<double>::epsilon()*values.cwiseAbs().maxCoeff();
    if(values.minCoeff()<-tolerance)throw std::invalid_argument("positive_semidefinite_bias_covariance_required");
    const auto rank=(values.array()>tolerance).count();out.white.resize(rank,6);out.hard.resize(6-rank,6);
    Eigen::Index soft=0,hard=0;
    for(int i=0;i<6;++i)if(values[i]>tolerance)out.white.row(soft++)=eigen.eigenvectors().col(i).transpose()/std::sqrt(values[i]);
      else out.hard.row(hard++)=eigen.eigenvectors().col(i).transpose();
    return out;
  }
};
}  // namespace posim_slam

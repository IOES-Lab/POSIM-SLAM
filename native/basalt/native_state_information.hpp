// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "map_information.hpp"
#include <array>

namespace posim_slam {
// The finite part alone is not a covariance for an affected free direction.
// Preserve the full selected cross terms and the projected nullspace together.
struct NativeStateInformation {
  Eigen::Matrix<double,15,15> observable_covariance;
  Eigen::MatrixXd unbounded;
  std::array<MapPointInformation,5> blocks;
  Eigen::Index source_columns=0;
};

// Map the pinned state's optimization increment to the current physical
// tangent. A retained rotation delta needs its SO(3) Jacobian, not identity.
// State copies keep this query independent of optimizer mutation/caches.
template<class State>
inline Eigen::Matrix<double,15,15> currentStateDerivative(const State& state) {
  Eigen::Matrix<double,15,15> derivative;
  for(int column=0;column<15;++column) {
    Eigen::Matrix<double,15,1> step=Eigen::Matrix<double,15,1>::Zero();step[column]=1e-6;
    auto plus=state,minus=state;plus.applyInc(step);minus.applyInc(-step);
    const auto a=plus.getState(),b=minus.getState();
    derivative.template block<3,1>(0,column)=(a.T_w_i.translation()-b.T_w_i.translation())/(2e-6);
    derivative.template block<3,1>(3,column)=(a.T_w_i.so3()*b.T_w_i.so3().inverse()).log()/(2e-6);
    derivative.template block<3,1>(6,column)=(a.vel_w_i-b.vel_w_i)/(2e-6);
    derivative.template block<3,1>(9,column)=(a.bias_gyro-b.bias_gyro)/(2e-6);
    derivative.template block<3,1>(12,column)=(a.bias_accel-b.bias_accel)/(2e-6);
  }
  return derivative;
}

inline NativeStateInformation selectedStateInformation(
    const MapPoseInformation& information,Eigen::Index offset,
    const Eigen::Matrix<double,15,15>& current_derivative=Eigen::Matrix<double,15,15>::Identity()) {
  const auto columns=information.covariance_root.rows();
  if(columns<15 || columns>384 || offset<0 || offset>columns-15 ||
      information.covariance_root.cols()>columns || information.nullspace.cols()>columns ||
      information.nullspace.rows()!=columns || !information.covariance_root.allFinite() ||
      !information.nullspace.allFinite() || !current_derivative.allFinite())
    throw std::invalid_argument("invalid_native_state_selection");
  NativeStateInformation result;result.source_columns=columns;
  const Eigen::MatrixXd root=current_derivative*information.covariance_root.middleRows(offset,15);
  result.observable_covariance=root*root.transpose();
  result.unbounded=current_derivative*information.nullspace.middleRows(offset,15);
  const MapNuisanceInformation empty{Eigen::MatrixXd::Zero(0,columns),
      Eigen::MatrixXd::Zero(0,0),Eigen::MatrixXd::Zero(0,0)};
  for(int block=0;block<5;++block) {
    Eigen::MatrixXd derivative=Eigen::MatrixXd::Zero(3,columns);
    derivative.middleCols(offset,15)=current_derivative.middleRows<3>(3*block);
    // Reuse the map's existing free-direction and finite-information rules.
    result.blocks[block]=mapPointInformation(information,empty,derivative,Eigen::MatrixXd::Zero(3,0));
  }
  return result;
}
}  // namespace posim_slam

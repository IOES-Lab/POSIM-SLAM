// SPDX-License-Identifier: Apache-2.0
// Exercise the actual pinned estimator constructor and state increment layout.
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>
#include "native_state_information.hpp"
#include <cmath>
#include <cassert>
#include <sstream>
#include <iostream>

void current_tangent_control() {
  for(bool linearized:{false,true})for(double scale:{0.,1.}) {
    const Sophus::SE3d pose(Sophus::SO3d::exp(Eigen::Vector3d(.2,.3,-.1)),Eigen::Vector3d(1,2,3));
    basalt::PoseVelBiasStateWithLin<double> state(1,pose,Eigen::Vector3d(.2,.3,-.4),
        Eigen::Vector3d(.01,.02,.03),Eigen::Vector3d(.1,.2,.3),linearized);
    Eigen::Matrix<double,15,1> increment=Eigen::Matrix<double,15,1>::Constant(.03);
    increment.segment<3>(3)=scale*Eigen::Vector3d(.4,-.2,.3);state.applyInc(increment);
    const auto before=state.getState();const auto delta=state.getDelta();
    const auto actual=posim_slam::currentStateDerivative(state);
    Eigen::Matrix<double,15,15> expected=Eigen::Matrix<double,15,15>::Identity();
    if(linearized && scale) {
      const auto angle=increment.segment<3>(3);const double norm=angle.norm();Eigen::Matrix3d hat;
      hat<<0,-angle[2],angle[1],angle[2],0,-angle[0],-angle[1],angle[0],0;
      expected.block<3,3>(3,3)=Eigen::Matrix3d::Identity()+
          (1-std::cos(norm))/(norm*norm)*hat+(norm-std::sin(norm))/(norm*norm*norm)*hat*hat;
    }
    assert((actual-expected).cwiseAbs().maxCoeff()<2e-9);
    assert((state.getDelta()-delta).norm()==0);
    assert((state.getState().T_w_i.params()-before.T_w_i.params()).norm()==0);
    assert((state.getState().vel_w_i-before.vel_w_i).norm()==0);
    assert((state.getState().bias_gyro-before.bias_gyro).norm()==0);
    assert((state.getState().bias_accel-before.bias_accel).norm()==0);
  }
}

int main() {
  current_tangent_control();
  basalt::Calibration<double> calibration;
  calibration.imu_update_rate=100;
  calibration.gyro_noise_std.setConstant(.001);
  calibration.accel_noise_std.setConstant(.02);
  calibration.gyro_bias_std.setConstant(.0001);
  calibration.accel_bias_std.setConstant(.001);
  int cases=0;
  for(bool square_root:{false,true})for(bool reversed:{false,true}) {
    basalt::VioConfig config;config.vio_sqrt_marg=square_root;
    config.vio_init_bg_weight=reversed?4.:25.;
    config.vio_init_ba_weight=reversed?25.:4.;
    std::ostringstream captured;auto* saved=std::cout.rdbuf(captured.rdbuf());
    try {
      basalt::SqrtKeypointVioEstimator<double> estimator(Eigen::Vector3d(0,0,-9.80665),calibration,config);
    } catch(...) {std::cout.rdbuf(saved);throw;}
    std::cout.rdbuf(saved);
    // Read the constructor's actual diagnostic matrix, not a duplicate helper.
    std::istringstream input(captured.str());std::string header;std::getline(input,header);
    assert(header.rfind("marg_H (sqrt:",0)==0);
    Eigen::Matrix<double,15,15> matrix;
    for(int i=0;i<15;++i)for(int j=0;j<15;++j)assert(bool(input>>matrix(i,j)));
    const Eigen::Matrix<double,15,15> information=square_root ? (matrix.transpose()*matrix).eval() : matrix;
    for(int index=0;index<15;++index) {
      const double expected=index<3 || index==5 ? config.vio_init_pose_weight :
          index>=9 && index<12 ? config.vio_init_bg_weight : index>=12 ? config.vio_init_ba_weight : 0.;
      assert(std::abs(information(index,index)-expected)<1e-8);
    }
    const Eigen::Matrix<double,15,15> diagonal=information.diagonal().asDiagonal();
    assert((information-diagonal).norm()<1e-12);
    for(int index=9;index<15;++index) {
      basalt::PoseVelBiasState<double> state(1,Sophus::SE3d(),Eigen::Vector3d::Zero(),
          Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
      Eigen::Matrix<double,15,1> increment=Eigen::Matrix<double,15,1>::Zero();increment[index]=.2;
      state.applyInc(increment);
      const double physical=config.vio_init_bg_weight*state.bias_gyro.squaredNorm()+
          config.vio_init_ba_weight*state.bias_accel.squaredNorm();
      assert(std::abs(increment.dot(information*increment)-physical)<1e-12);
    }
    ++cases;
  }
  std::cout<<"PASS "<<cases<<" actual-constructor initial bias prior cases; 24 physical state increments; 4 retained/current tangent and no-mutation controls"<<std::endl;
}

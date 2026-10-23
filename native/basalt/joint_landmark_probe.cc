// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_joint.hpp"
#include <basalt/linearization/landmark_block_abs_dynamic.hpp>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    auto c=input_calibration(argv[1]);
    basalt::Keypoint<double> point;point.host_kf_id={100,0};
    point.direction.setZero();point.inv_dist=.5;
    basalt::AbsOrderMap order;order.total_size=18;
    Eigen::aligned_unordered_map<std::pair<basalt::TimeCamId,basalt::TimeCamId>,
        basalt::RelPoseLin<double>> relative;
    for(int i=0;i<3;++i) {
      int64_t time=100+i;basalt::TimeCamId target(time,0);
      order.abs_order_map[time]={6*i,6};
      auto& rel=relative[{point.host_kf_id,target}];
      rel.T_t_h.setIdentity();rel.T_t_h(0,3)=-.2*i;
      rel.d_rel_d_h.setIdentity();rel.d_rel_d_t=-Eigen::Matrix<double,6,6>::Identity();
      Eigen::Vector2d observation;
      if(!c.cal.intrinsics[0].project(Eigen::Vector3d(-.2*i,0,2.),observation))return 3;
      observation+=Eigen::Vector2d(.1,-.05);point.obs[target]=observation;
    }
    auto other=point;
    basalt::LandmarkBlock<double>::Options options;
    options.huber_parameter=1.;options.obs_std_dev=1.;
    basalt::LandmarkBlockAbsDynamic<double,6> joint,baseline;
    joint.allocateLandmark(point,relative,c.cal,order,options);
    baseline.allocateLandmark(other,relative,c.cal,order,options);
    joint.linearizeLandmark();baseline.linearizeLandmark();
    Eigen::MatrixXd jp,jl;Eigen::VectorXd residual;
    joint.exportJointLinearization(jp,jl,residual);
    Eigen::MatrixXd full(jp.rows(),jp.cols()+3);full<<jp,jl;
    auto reduced=posim_slam::eliminateJointPatch(full,residual,jp.cols());
    baseline.performQR();
    Eigen::MatrixXd qr=Eigen::MatrixXd::Zero(baseline.numQ2rows(),18);
    Eigen::VectorXd qr_r=Eigen::VectorXd::Zero(baseline.numQ2rows());
    static_cast<basalt::LandmarkBlock<double>&>(baseline).get_dense_Q2Jp_Q2r(qr,qr_r,0);
    const double h_error=(qr.transpose()*qr-reduced.jacobian.transpose()*reduced.jacobian).norm();
    const double b_error=(qr.transpose()*qr_r-reduced.jacobian.transpose()*reduced.residual).norm();
    if(h_error>1e-7 || b_error>1e-7)throw std::runtime_error("raw_vs_qr_mismatch");
    Eigen::VectorXd pose_delta=Eigen::VectorXd::Zero(18);
    Eigen::Vector3d delta=reduced.backSubstitute(pose_delta);
    if(!joint.jointIncrementValid(delta))throw std::runtime_error("joint_increment_rejected");
    joint.applyJointIncrement(delta);double cost=0.;baseline.backSubstitute(pose_delta,cost);
    const double update_error=(point.direction-other.direction).norm()+std::abs(point.inv_dist-other.inv_dist);
    if(update_error>1e-9)throw std::runtime_error("joint_landmark_update_mismatch");
    bool stale_rejected=false,scaled_rejected=false;
    try {joint.exportJointLinearization(jp,jl,residual);}catch(const std::logic_error&){stale_rejected=true;}
    joint.linearizeLandmark();
    if(joint.jointIncrementValid(Eigen::Vector3d(0,0,-100)))throw std::runtime_error("negative_depth_accepted");
    joint.scaleJl_cols();
    try {joint.exportJointLinearization(jp,jl,residual);}catch(const std::logic_error&){scaled_rejected=true;}
    if(!stale_rejected || !scaled_rejected)throw std::runtime_error("joint_raw_state_guard_failed");
    std::cout<<Json({{"passed",true},{"raw_qr_h_error",h_error},{"raw_qr_b_error",b_error},
      {"landmark_update_error",update_error},{"stale_rejected",stale_rejected},
      {"scaled_rejected",scaled_rejected}}).dump()<<'\n';
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

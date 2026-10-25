// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_joint.hpp"
#include <basalt/linearization/landmark_block_abs_dynamic.hpp>
#include <iostream>
#include <chrono>

// Compare against unmodified full-width QR rows, including all zero state
// columns. Mixed pose/inertial blocks, non-adjacent observations, scaling and
// damping exercise the same layout that the joint live estimator uses.
template<class Calibration> Json checkVisualNormal(const Calibration& c) {
  double worst_h=0.,worst_b=0.,compact_ms=0.,dense_ms=0.;int cases=0;
  for(const auto [frames,observed]:std::vector<std::pair<int,int>>{{3,3},{24,3},{24,12},{72,16}}) {
    basalt::AbsOrderMap order;order.total_size=0;
    for(int i=0;i<frames;++i) {
      const int size=i%3==0 ? 6 : 15;
      order.abs_order_map[100+i]={order.total_size,size};order.total_size+=size;
    }
    basalt::Keypoint<double> point;point.host_kf_id={100,0};
    point.direction=Eigen::Vector2d(.02,-.01);point.inv_dist=.5;
    Eigen::aligned_unordered_map<std::pair<basalt::TimeCamId,basalt::TimeCamId>,basalt::RelPoseLin<double>> relative;
    for(int i=0;i<observed;++i) {
      const int frame=i*(frames-1)/(observed-1);basalt::TimeCamId target(100+frame,0);
      auto& rel=relative[{point.host_kf_id,target}];
      rel.T_t_h.setIdentity();rel.T_t_h(0,3)=-.03*i;rel.T_t_h(1,3)=.01*i;
      rel.d_rel_d_h.setIdentity();rel.d_rel_d_t=-Eigen::Matrix<double,6,6>::Identity();
      Eigen::Vector2d pixel;
      if(!c.cal.intrinsics[0].project(Eigen::Vector3d(-.03*i,.01*i,2.),pixel))
        throw std::runtime_error("normal_fixture_projection");
      point.obs[target]=pixel+Eigen::Vector2d(.1*std::sin(i),-.05);
    }
    basalt::LandmarkBlock<double>::Options options;options.huber_parameter=1.;options.obs_std_dev=1.;
    basalt::LandmarkBlockAbsDynamic<double,6> block;
    block.allocateLandmark(point,relative,c.cal,order,options);block.linearizeLandmark();
    block.scaleJl_cols();block.performQR();
    Eigen::VectorXd scaling(order.total_size);
    for(int i=0;i<scaling.size();++i)scaling[i]=.5+.01*(i%31);
    block.scaleJp_cols(scaling);
    for(double damping:{0.,.001,1000.}) {
      block.setLandmarkDamping(damping);
      Eigen::MatrixXd j=Eigen::MatrixXd::Zero(block.numQ2rows(),order.total_size);
      Eigen::VectorXd r=Eigen::VectorXd::Zero(block.numQ2rows());
      static_cast<basalt::LandmarkBlock<double>&>(block).get_dense_Q2Jp_Q2r(j,r,0);
      Eigen::MatrixXd h=Eigen::MatrixXd::Constant(order.total_size,order.total_size,.002);
      Eigen::VectorXd b=Eigen::VectorXd::Constant(order.total_size,.003);
      const Eigen::MatrixXd expected_h=h+j.transpose()*j;
      const Eigen::VectorXd expected_b=b+j.transpose()*r;
      static_cast<basalt::LandmarkBlock<double>&>(block).add_dense_H_b(h,b);
      const double h_error=(h-expected_h).norm()/std::max(1.,expected_h.norm());
      const double b_error=(b-expected_b).norm()/std::max(1.,expected_b.norm());
      worst_h=std::max(worst_h,h_error);worst_b=std::max(worst_b,b_error);++cases;
      if(h_error>1e-12 || b_error>1e-12)throw std::runtime_error("compact_visual_normal_mismatch");
      if(frames==24 && observed==12 && damping==.001) {
        auto begin=std::chrono::steady_clock::now();
        for(int n=0;n<100;++n)static_cast<basalt::LandmarkBlock<double>&>(block).add_dense_H_b(h,b);
        auto middle=std::chrono::steady_clock::now();
        for(int n=0;n<100;++n){h.noalias()+=j.transpose()*j;b.noalias()+=j.transpose()*r;}
        auto end=std::chrono::steady_clock::now();
        if(!h.allFinite() || !b.allFinite())throw std::runtime_error("normal_benchmark_nonfinite");
        compact_ms=std::chrono::duration<double,std::milli>(middle-begin).count();
        dense_ms=std::chrono::duration<double,std::milli>(end-middle).count();
      }
    }
  }
  return {{"cases",cases},{"relative_h_error",worst_h},{"relative_b_error",worst_b},
          {"compact_100_ms",compact_ms},{"dense_100_ms",dense_ms},
          {"scope","Equivalent visual normal assembly; microbenchmark, not live throughput"}};
}

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    auto c=input_calibration(argv[1]);
    const auto normal=checkVisualNormal(c);
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
    std::cout<<Json({{"passed",true},{"visual_normal",normal},{"raw_qr_h_error",h_error},{"raw_qr_b_error",b_error},
      {"landmark_update_error",update_error},{"stale_rejected",stale_rejected},
      {"scaled_rejected",scaled_rejected}}).dump()<<'\n';
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

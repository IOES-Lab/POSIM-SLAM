// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_linearization.hpp"
#include <basalt/imu/preintegration.h>
#include <iostream>

class IndependentReference : public basalt::LinearizationAbsQR<double,6> {
 public:
  using basalt::LinearizationAbsQR<double,6>::LinearizationAbsQR;
  double rawVisualCost() const {
    double cost=0.;
    for(const auto& block:landmark_blocks) {
      Eigen::MatrixXd jp,jl;Eigen::VectorXd r;block->exportJointLinearization(jp,jl,r);cost+=.5*r.squaredNorm();
    }
    return cost;
  }
  double reducedVisualCost() const {
    Eigen::MatrixXd j;Eigen::VectorXd r;get_dense_Q2Jp_Q2r(j,r);return .5*r.head(num_rows_Q2r).squaredNorm();
  }
};

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    using namespace posim_slam;
    auto c=input_calibration(argv[1]);c.cal.T_i_c[0]=Sophus::SE3d();
    basalt::BundleAdjustmentBase<double> owner;owner.calib=c.cal;owner.huber_thresh=1.;owner.obs_std_dev=1.;
    basalt::AbsOrderMap order;order.items=3;order.total_size=45;
    std::vector<int64_t> times={1000000000,1100000000,1200000000};
    for(int i=0;i<3;++i) {
      Sophus::SE3d pose;pose.translation()<<.08*i,.02*i,0;
      owner.frame_states.emplace(times[i],basalt::PoseVelBiasStateWithLin<double>(times[i],pose,Vec3(.8,.2,0),Vec3::Zero(),Vec3::Zero(),true));
      order.abs_order_map[times[i]]={15*i,15};
    }
    const Vec3 normal=Vec3(.1,-.15,1.).normalized();const double distance=2.;
    owner.posim_ping_planes.set(1,normal,distance);
    std::set<basalt::KeypointId> ids;
    for(int id=0;id<26;++id) {
      int host=id%3;double x=-.6+.3*(id%5),y=-.6+.3*((id/5)%5);
      Vec3 world(x,y,(distance-normal.x()*x-normal.y()*y)/normal.z());
      const auto pose=owner.getPoseStateWithLin(times[host]).getPose();const Vec3 local=pose.inverse()*world;
      basalt::Keypoint<double> point;point.host_kf_id={times[host],0};
      point.direction=basalt::StereographicParam<double>::project(local.homogeneous());point.inv_dist=1./local.norm();
      owner.lmdb.addLandmark(id,point);
      for(int target=0;target<3;++target) {
        Eigen::Vector2d pixel;owner.calib.intrinsics[0].project(owner.getPoseStateWithLin(times[target]).getPose().inverse()*world,pixel);
        // One independent visual landmark exercises ordinary QR/robust cost.
        if(id==25 && target==2)pixel.x()+=2.;
        owner.lmdb.addObservation({times[target],0},basalt::KeypointObservation<double>{id,pixel});
      }
      if(id<25)ids.insert(id);
    }
    Vec3 gravity(0,0,-9.80665),gyro_weight=Vec3::Constant(100.),accel_weight=Vec3::Constant(100.);
    std::vector<basalt::IntegratedImuMeasurement<double>> preintegrations;
    preintegrations.reserve(2);
    for(int interval=0;interval<2;++interval) {
      preintegrations.emplace_back(times[interval],Vec3::Zero(),Vec3::Zero());
      for(int k=1;k<=20;++k) {
        basalt::ImuData<double> data;data.t_ns=times[interval]+k*5000000LL;data.gyro.setZero();data.accel=Vec3(0,0,9.80665);
        preintegrations.back().integrate(data,Vec3::Constant(.0004),Vec3::Constant(.000001));
      }
    }
    basalt::ImuLinData<double> imu{gravity,gyro_weight,accel_weight,{}};
    for(int i=0;i<2;++i)imu.imu_meas[times[i]]=&preintegrations[i];
    basalt::MargLinData<double> prior;prior.order=order;prior.H=5.*Eigen::MatrixXd::Identity(45,45);prior.b=Eigen::VectorXd::Zero(45);
    PingLinearization::Options options;options.lb_options.huber_parameter=1.;options.lb_options.obs_std_dev=1.;
    Eigen::MatrixXd rays(25,3);int ri=0;
    for(int y=-2;y<=2;++y)for(int x=-2;x<=2;++x) {
      double h=x*.074175,v=y*.074175;rays.row(ri++)=Vec3(-std::cos(v)*std::sin(h),std::sin(v),std::cos(v)*std::cos(h)).transpose();
    }
    Sophus::SE3d mount;mount.translation()<<.02,.03,.1;
    const double measured=pingRange(owner.getPoseStateWithLin(times[2]).getPose(),mount,rays,normal,distance,.1).prediction+.04;
    NativePingGroup raw;
    JointPingGroupSpec spec{1,ids,[&](const auto& blocks) {
      std::vector<PingGroupLandmark> landmarks;
      for(const auto& [id,block]:blocks) {
        const auto& point=owner.lmdb.getLandmark(id);const auto t=point.host_kf_id.frame_id;
        landmarks.push_back({id,block,worldLandmark(owner.getPoseStateWithLin(t).getPose(),owner.calib.T_i_c[0],point.direction,point.inv_dist),order.abs_order_map.at(t).first});
      }
      const auto& plane=owner.posim_ping_planes.at(1);
      PingGroupObservation observation{times[2],times[2],30,owner.getPoseStateWithLin(times[2]).getPose(),mount,rays,measured,.1,.02,.2,50.};
      raw=assemblePingGroup(45,plane.normal,plane.distance,landmarks,{observation},.02,.23);return raw;
    }};
    bool duplicate=false;
    try {PingLinearization invalid(&owner,order,options,{spec,spec},&prior,&imu);}
    catch(const std::invalid_argument&){duplicate=true;}
    PingLinearization joint(&owner,order,options,{spec},&prior,&imu);
    basalt::LinearizationBase<double,6>* api=&joint;bool valid=false;
    const double before=api->linearizeProblem(&valid);api->performQR();
    Eigen::MatrixXd h,j;Eigen::VectorXd b,r;api->get_dense_H_b(h,b);api->get_dense_Q2Jp_Q2r(j,r);
    const double normal_scale=std::max(1.,h.norm()+b.norm());
    const double square_root_error=(h-j.transpose()*j).norm()+(b-j.transpose()*r).norm();
    // Reference: the independent landmark, IMU and prior once, plus the raw
    // coupled group once. This catches accidental duplicate visual information.
    std::unordered_set<basalt::KeypointId> independent_ids={25};
    IndependentReference independent(&owner,order,options,&prior,&imu,nullptr,&independent_ids);
    independent.linearizeProblem();const double initial_visual_cost=independent.rawVisualCost();independent.performQR();
    const double eliminated_visual_constant=initial_visual_cost-independent.reducedVisualCost();
    Eigen::MatrixXd ij;Eigen::VectorXd ir;independent.get_dense_Q2Jp_Q2r(ij,ir);
    Eigen::MatrixXd full=Eigen::MatrixXd::Zero(ij.rows()+raw.jacobian.rows(),raw.jacobian.cols());
    full.topLeftCorner(ij.rows(),45)=ij;full.bottomRows(raw.jacobian.rows())=raw.jacobian;
    Eigen::VectorXd full_r(ir.size()+raw.residual.size());full_r<<ir,raw.residual;
    const auto reduced=eliminateJointPatch(full,full_r,45);
    const double information_error=(h-reduced.jacobian.transpose()*reduced.jacobian).norm();
    const Eigen::VectorXd delta=h.ldlt().solve(-b);
    const Eigen::VectorXd batch_delta=full.colPivHouseholderQr().solve(-full_r);
    const double step_error=(delta-batch_delta.head(45)).norm();
    // Ordinary QR already minimized the independent landmark. Restore that
    // initial Q1 residual cost when comparing the full trial model decrease.
    const double reference_decrease=eliminated_visual_constant+.5*(full_r.squaredNorm()-(full_r+full*batch_delta).squaredNorm());
    bool transaction_required=false;
    try {api->backSubstitute(delta);}catch(const std::logic_error&){transaction_required=true;}
    bool stale=false,rollback_stale=false;double after=0.,predicted=0.,first_prediction=0.;
    {
      JointPingTrial trial(owner);predicted=api->backSubstitute(delta);first_prediction=predicted;
      try {api->get_dense_H_b(h,b);}catch(const std::logic_error&){stale=true;}
      for(int i=0;i<3;++i)owner.frame_states.at(times[i]).applyInc(delta.segment<15>(15*i));
      after=api->linearizeProblem();api->performQR();
      // Even a newly built trial-state reduction is invalid after rollback.
      trial.accept(before,before+1.,predicted);
    }
    try {api->get_dense_H_b(h,b);}catch(const std::logic_error&){rollback_stale=true;}
    const double restored=api->linearizeProblem();api->performQR();
    const bool restored_cost=std::abs(before-restored)<1e-9;
    bool accepted=false;
    {
      JointPingTrial trial(owner);predicted=api->backSubstitute(delta);
      for(int i=0;i<3;++i)owner.frame_states.at(times[i]).applyInc(delta.segment<15>(15*i));
      after=api->linearizeProblem();accepted=trial.accept(before,after,predicted);
    }
    const double prediction_error=std::abs(predicted-reference_decrease);
    bool passed=std::abs(before-3.5)<1e-9 && valid && duplicate && transaction_required && square_root_error/normal_scale<1e-12 && information_error/normal_scale<1e-12 && prediction_error<1e-7 && step_error<1e-7 && stale && rollback_stale && restored_cost && accepted;
    std::cout<<Json({{"passed",passed},{"joint_landmarks",25},{"independent_landmarks",1},{"imu_intervals",2},
      {"square_root_normal_error",square_root_error},{"single_ownership_information_error",information_error},
      {"normal_system_scale",normal_scale},{"relative_square_root_error",square_root_error/normal_scale},
      {"relative_information_error",information_error/normal_scale},{"model_decrease_error",prediction_error},{"reference_decrease",reference_decrease},{"first_prediction",first_prediction},{"reference_initial_cost",.5*full_r.squaredNorm()},{"independent_eliminated_visual_cost",eliminated_visual_constant},
      {"transaction_required",transaction_required},
      {"full_system_pose_step_error",step_error},{"cost_before",before},{"cost_after",after},
      {"predicted_decrease",predicted},{"duplicate_group_rejected",duplicate},{"stale_rows_rejected",stale},
      {"rollback_rows_rejected",rollback_stale},{"rollback_cost_restored",restored_cost},{"trial_accepted",accepted}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

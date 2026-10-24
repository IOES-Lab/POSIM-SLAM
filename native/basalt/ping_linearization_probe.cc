// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_linearization.hpp"
#include "ping_input.hpp"
#include "ping_owned_group.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include "ping_commit.hpp"
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
  if(argc!=2 && !(argc==3 && std::string(argv[2])=="--expected-return"))return 2;
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
    const auto eligible=activePingPoints(owner,times[2],c.processed_width,c.processed_height);
    const auto saved_observation=owner.lmdb.getLandmark(0).obs.at({times[0],0});
    owner.lmdb.removeObservations(0,{{times[0],0}});
    const auto immature=activePingPoints(owner,times[2],c.processed_width,c.processed_height);
    owner.lmdb.addObservation({times[0],0},basalt::KeypointObservation<double>{0,saved_observation});
    const bool owner_support_passed=eligible.size()==25 && immature.size()==24 &&
      std::none_of(eligible.begin(),eligible.end(),[](const auto& p){return p.id==25;});
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
    const double measured=(argc==3 ? expectedPingRange(owner.getPoseStateWithLin(times[2]).getPose(),mount,rays,normal,distance,.1,.005,.2,50.).range.prediction :
      pingRange(owner.getPoseStateWithLin(times[2]).getPose(),mount,rays,normal,distance,.1).prediction)+.04;
    NativePingGroup raw;bool use_reference=true;
    std::map<basalt::KeypointId,basalt::LandmarkBlock<double>*> latest_blocks;
    JointPingGroupSpec spec{1,ids,[&](const auto& blocks) {
      latest_blocks=blocks;
      std::vector<PingGroupLandmark> landmarks;
      for(const auto& [id,block]:blocks) {
        const auto& point=owner.lmdb.getLandmark(id);const auto t=point.host_kf_id.frame_id;
        landmarks.push_back({id,block,worldLandmark(owner.getPoseStateWithLin(t).getPose(),owner.calib.T_i_c[0],point.direction,point.inv_dist),order.abs_order_map.at(t).first,
          use_reference ? std::optional<WorldLandmark>(worldLandmark(owner.getPoseStateWithLin(t).getPoseLin(),owner.calib.T_i_c[0],point.direction,point.inv_dist)) : std::nullopt});
      }
      const auto& plane=owner.posim_ping_planes.at(1);
      PingGroupObservation observation{times[2],times[2],30,owner.getPoseStateWithLin(times[2]).getPose(),mount,rays,measured,.1,.02,.2,50.,use_reference ? std::optional<Sophus::SE3d>(owner.getPoseStateWithLin(times[2]).getPoseLin()) : std::nullopt};
      if(argc==3)observation.per_ray_sigma=.005;
      raw=assemblePingGroup(45,plane.normal,plane.distance,landmarks,{observation},.02,.23,use_reference ? std::optional<PingPlaneReference>(plane.reference) : std::nullopt);return raw;
    }};
    auto reference_builder=spec.build;
    OwnedPingObservation owned_observation{times[2],measured,{mount,rays,.2,50.},.1,.02,std::nullopt};
    if(argc==3)owned_observation.per_ray_sigma=.005;
    const auto owned_spec=makeOwnedPingGroup(owner,order,1,ids,{owned_observation},.02,.23);
    // All valid-path optimization now uses the production owner-backed builder.
    // Retain the deliberately inconsistent builder only as the negative control.
    spec=owned_spec;
    spec.build=[&](const auto& blocks) {
      latest_blocks=blocks;
      raw=use_reference ? owned_spec.build(blocks) : reference_builder(blocks);
      return raw;
    };
    bool invalid_owned_measurement=false;auto bad_owned=owned_observation;bad_owned.return_sigma=0.;
    try{makeOwnedPingGroup(owner,order,1,ids,{bad_owned},.02,.23);}catch(const std::invalid_argument&){invalid_owned_measurement=true;}
    // Mutation of the caller's observation cannot change the copied factor.
    owned_observation.measured+=100.;
    spec.observation_pose_stamps={times[2]};
    const auto untouched=planPingRetirement(owner,{spec},{},{},{});
    const auto lost_plan=planPingRetirement(owner,{spec},{},{},{*ids.begin()});
    auto host_only=spec;host_only.landmark_ids.clear();
    for(auto id:ids)if(owner.lmdb.getLandmark(id).host_kf_id.frame_id==times[0])host_only.landmark_ids.insert(id);
    const auto target_plan=planPingRetirement(owner,{host_only},{times[1]},{},{});
    bool trial_plan_rejected=false;owner.posim_ping_trial_active=true;
    try{planPingRetirement(owner,{spec},{},{},{});}catch(const std::logic_error&){trial_plan_rejected=true;}
    owner.posim_ping_trial_active=false;
    bool missing_metadata_rejected=false;auto missing_metadata=spec;missing_metadata.observation_pose_stamps.clear();
    try{planPingRetirement(owner,{missing_metadata},{},{},{});}catch(const std::invalid_argument&){missing_metadata_rejected=true;}
    const bool retirement_passed=untouched.groups.empty() && untouched.landmarks.empty() &&
      lost_plan.groups.size()==1 && lost_plan.landmarks.size()==25 && lost_plan.required_frames==std::set<int64_t>(times.begin(),times.end()) &&
      lost_plan.observations==spec.observation_pose_stamps && target_plan.groups.size()==1 &&
      target_plan.landmarks.size()==host_only.landmark_ids.size() && trial_plan_rejected && missing_metadata_rejected && owner.lmdb.numLandmarks()==26;
    bool changed_dependencies_rejected=false;auto wrong_dependencies=spec;wrong_dependencies.observation_pose_stamps={times[1]};
    try{PingLinearization wrong(&owner,order,options,{wrong_dependencies},&prior,&imu);wrong.linearizeProblem();}
    catch(const std::invalid_argument&){changed_dependencies_rejected=true;}
    bool duplicate=false;
    try {PingLinearization invalid(&owner,order,options,{spec,spec},&prior,&imu);}
    catch(const std::invalid_argument&){duplicate=true;}
    PingLinearization joint(&owner,order,options,{spec},&prior,&imu);
    basalt::LinearizationBase<double,6>* api=&joint;bool valid=false;
    const double before=api->linearizeProblem(&valid);api->performQR();
    Eigen::MatrixXd h,j;Eigen::VectorXd b,r;api->get_dense_H_b(h,b);api->get_dense_Q2Jp_Q2r(j,r);
    // Retire one group without dropping states: no IMU interval is consumed,
    // and the independent live landmark must not enter the new prior.
    const auto prepared=preparePingPrior(owner,order,{spec},prior,imu,{},{},{0});
    std::set<basalt::FrameId> prior_no_hosts;std::unordered_set<basalt::KeypointId> no_landmarks;
    IndependentReference prior_only(&owner,order,options,&prior,nullptr,&prior_no_hosts,&no_landmarks);
    prior_only.linearizeProblem();prior_only.performQR();
    Eigen::MatrixXd ph;Eigen::VectorXd pb;prior_only.get_dense_H_b(ph,pb);
    const auto reduced_group=eliminateJointPatch(raw.jacobian,raw.residual,45);
    ph+=reduced_group.jacobian.transpose()*reduced_group.jacobian;
    pb+=reduced_group.jacobian.transpose()*reduced_group.residual;
    const double prepared_error=(prepared.prior.H.transpose()*prepared.prior.H-ph).norm()+
      (prepared.prior.H.transpose()*prepared.prior.b-pb).norm();
    const bool prepared_passed=prepared_error/std::max(1.,ph.norm()+pb.norm())<1e-12 &&
      prepared.retirement.landmarks.size()==25 && prepared.layout.consumed_imu_starts.empty() &&
      owner.lmdb.numLandmarks()==26 && owner.posim_ping_planes.size()==1;
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
    // Marginalization selects the complete coupled group, excluding the
    // independent visual track that remains live. IMU/prior appear once.
    const auto saved_raw=raw;
    std::set<basalt::FrameId> no_hosts;
    std::unordered_set<basalt::KeypointId> retiring_ids(ids.begin(),ids.end()),no_ids;
    PingLinearization selected(&owner,order,options,{spec},&prior,&imu,&no_hosts,&retiring_ids);
    selected.linearizeProblem();selected.performQR();Eigen::MatrixXd selected_h;Eigen::VectorXd selected_b;
    selected.get_dense_H_b(selected_h,selected_b);
    IndependentReference inertial_prior(&owner,order,options,&prior,&imu,&no_hosts,&no_ids);
    inertial_prior.linearizeProblem();inertial_prior.performQR();Eigen::MatrixXd base_j;Eigen::VectorXd base_r;
    inertial_prior.get_dense_Q2Jp_Q2r(base_j,base_r);
    const auto group_reduced=eliminateJointPatch(raw.jacobian,raw.residual,45);
    const double selection_error=(selected_h-base_j.transpose()*base_j-group_reduced.jacobian.transpose()*group_reduced.jacobian).norm()+
      (selected_b-base_j.transpose()*base_r-group_reduced.jacobian.transpose()*group_reduced.residual).norm();
    auto incomplete_ids=retiring_ids;incomplete_ids.erase(*ids.begin());bool incomplete_rejected=false;
    try {PingLinearization incomplete(&owner,order,options,{spec},&prior,&imu,&no_hosts,&incomplete_ids);}
    catch(const std::invalid_argument&){incomplete_rejected=true;}
    // Restore callback-owned diagnostic pointers to the optimization instance.
    joint.linearizeProblem();joint.performQR();raw=saved_raw;
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
    Eigen::MatrixXd gauge=Eigen::MatrixXd::Zero(raw.jacobian.cols(),6);
    for(int i=0;i<3;++i) {
      gauge.block<3,3>(15*i,0).setIdentity();
      gauge.block<3,3>(15*i,3)=-crossMatrix(owner.getPoseStateWithLin(times[i]).getPoseLin().translation());
      gauge.block<3,3>(15*i+3,3).setIdentity();
    }
    gauge.block<2,3>(45,3)=planeBasis(normal).transpose();gauge.block<1,3>(47,0)=normal.transpose();
    const double reference_gauge_error=(raw.jacobian*gauge).norm();
    const auto& plane=owner.posim_ping_planes.at(1);
    const bool plane_reference_preserved=(plane.reference.normal-normal).norm()==0 && plane.reference.distance==distance && plane.delta.norm()>0;
    use_reference=false;const auto inconsistent=spec.build(latest_blocks);
    const double inconsistent_gauge_error=(inconsistent.jacobian*gauge).norm();
    bool missing_reference=false;
    try {api->linearizeProblem();}catch(const std::invalid_argument&){missing_reference=true;}
    const double prediction_error=std::abs(predicted-reference_decrease);
    use_reference=true;
    const auto saved_states=owner.frame_states;
    const auto saved_landmarks=owner.lmdb.getLandmarks();
    const auto saved_plane=owner.posim_ping_planes.at(1);
    auto reject_spec=owned_spec;reject_spec.build=[&](const auto& blocks) {
      if(owner.posim_ping_trial_active)throw std::invalid_argument("fixture_trial_support_loss");
      return owned_spec.build(blocks);
    };
    const auto rejected_run=optimizePingGroups(owner,order,{reject_spec},prior,imu,{2,3,.001});
    bool restored_run=rejected_run.accepted==0 && rejected_run.rejected==3 && !owner.posim_ping_trial_active;
    for(const auto& [stamp,state]:owner.frame_states) {
      const auto& original=saved_states.at(stamp).getState();const auto& current=state.getState();
      restored_run=restored_run && (original.T_w_i.matrix()-current.T_w_i.matrix()).norm()==0 &&
        (original.vel_w_i-current.vel_w_i).norm()==0 && (original.bias_gyro-current.bias_gyro).norm()==0 &&
        (original.bias_accel-current.bias_accel).norm()==0;
    }
    for(const auto& [id,point]:owner.lmdb.getLandmarks())restored_run=restored_run &&
      point.inv_dist==saved_landmarks.at(id).inv_dist && (point.direction-saved_landmarks.at(id).direction).norm()==0;
    restored_run=restored_run && (owner.posim_ping_planes.at(1).delta-saved_plane.delta).norm()==0;
    const auto optimized=optimizePingGroups(owner,order,{owned_spec},prior,imu);
    const bool optimizer_passed=restored_run && optimized.accepted>0 && optimized.final_cost<optimized.initial_cost && !owner.posim_ping_trial_active;
    // Exercise the complete preparation/installation path with actual visual,
    // Ping, prior and preintegrated IMU rows after nonlinear optimization.
    const auto outgoing=preparePingPrior(owner,order,{owned_spec},prior,imu,{times[0]},{},{});
    std::map<int64_t,basalt::IntegratedImuMeasurement<double>> remaining_imu;
    for(int i=0;i<2;++i)remaining_imu.emplace(times[i],preintegrations[i]);
    std::vector<JointPingGroupSpec> remaining_groups={owned_spec};auto installed_prior=prior;
    installPingMarginalPrior(owner,installed_prior,remaining_groups,remaining_imu,
      outgoing.retirement,outgoing.layout,{times[0]},{},outgoing.prior);
    const bool prepared_install_passed=outgoing.layout.consumed_imu_starts==std::set<int64_t>{times[0]} &&
      installed_prior.H.cols()==30 && installed_prior.H.allFinite() && owner.frame_states.size()==2 &&
      owner.lmdb.numLandmarks()==1 && owner.lmdb.landmarkExists(25) && owner.posim_ping_planes.size()==0 &&
      remaining_groups.empty() && remaining_imu.size()==1 && remaining_imu.count(times[1]);
    bool passed=prepared_install_passed && prepared_passed && optimizer_passed && invalid_owned_measurement && owner_support_passed && changed_dependencies_rejected && retirement_passed && incomplete_rejected && selection_error/normal_scale<1e-12 && reference_gauge_error<1e-8 && inconsistent_gauge_error>1e-7 && plane_reference_preserved && missing_reference && std::abs(before-3.5)<1e-9 && valid && duplicate && transaction_required && square_root_error/normal_scale<1e-12 && information_error/normal_scale<1e-12 && prediction_error<1e-7 && step_error<1e-7 && stale && rollback_stale && restored_cost && accepted;
    std::cout<<Json({{"prepared_install_passed",prepared_install_passed},{"prepared_prior_passed",prepared_passed},{"prepared_prior_error",prepared_error},{"optimizer_passed",optimizer_passed},{"optimizer_rejection_restored",restored_run},{"optimizer_accepted",optimized.accepted},{"optimizer_initial_cost",optimized.initial_cost},{"optimizer_final_cost",optimized.final_cost},{"expected_return",argc==3},{"invalid_owned_measurement_rejected",invalid_owned_measurement},{"owner_visual_support_passed",owner_support_passed},{"changed_observation_dependencies_rejected",changed_dependencies_rejected},{"retirement_dependency_closure_passed",retirement_passed},{"marginal_selection_error",selection_error},{"incomplete_group_rejected",incomplete_rejected},{"passed",passed},{"joint_landmarks",25},{"independent_landmarks",1},{"imu_intervals",2},
      {"square_root_normal_error",square_root_error},{"single_ownership_information_error",information_error},
      {"normal_system_scale",normal_scale},{"relative_square_root_error",square_root_error/normal_scale},
      {"relative_information_error",information_error/normal_scale},{"model_decrease_error",prediction_error},{"reference_decrease",reference_decrease},{"first_prediction",first_prediction},{"reference_initial_cost",.5*full_r.squaredNorm()},{"independent_eliminated_visual_cost",eliminated_visual_constant},
      {"transaction_required",transaction_required},{"reference_gauge_error",reference_gauge_error},
      {"inconsistent_reference_gauge_error",inconsistent_gauge_error},{"plane_reference_preserved",plane_reference_preserved},
      {"missing_reference_rejected",missing_reference},
      {"full_system_pose_step_error",step_error},{"cost_before",before},{"cost_after",after},
      {"predicted_decrease",predicted},{"duplicate_group_rejected",duplicate},{"stale_rows_rejected",stale},
      {"rollback_rows_rejected",rollback_stale},{"rollback_cost_restored",restored_cost},{"trial_accepted",accepted}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

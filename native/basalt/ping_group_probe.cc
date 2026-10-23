// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_group.hpp"
#include "ping_marginal.hpp"
#include <basalt/vi_estimator/ba_base.h>
#include <basalt/linearization/landmark_block_abs_dynamic.hpp>
#include <memory>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    using namespace posim_slam;
    auto calibration=input_calibration(argv[1]);auto& cal=calibration.cal;
    cal.T_i_c[0]=Sophus::SE3d();
    std::vector<Sophus::SE3d> poses(3);
    basalt::AbsOrderMap order;order.total_size=18;order.items=3;
    for(int i=0;i<3;++i){poses[i].translation()<<.08*i,.02*i,0;order.abs_order_map[100+i]={6*i,6};}
    Eigen::aligned_unordered_map<std::pair<basalt::TimeCamId,basalt::TimeCamId>,basalt::RelPoseLin<double>> relative;
    auto update_relative=[&] {
      for(int h=0;h<3;++h)for(int t=0;t<3;++t) {
        auto& r=relative[{{100+h,0},{100+t,0}}];
        r.T_t_h=basalt::computeRelPose(poses[h],cal.T_i_c[0],poses[t],cal.T_i_c[0],&r.d_rel_d_h,&r.d_rel_d_t).matrix();
      }
    };
    update_relative();
    Vec3 normal=Vec3(.1,-.15,1.).normalized();double distance=2.;
    std::vector<basalt::Keypoint<double>> points;points.reserve(25);
    for(int y=0;y<5;++y)for(int x=0;x<5;++x) {
      const int host=points.size()%3;const double px=-.6+.3*x,py=-.6+.3*y;
      const Vec3 world(px,py,(distance-normal.x()*px-normal.y()*py)/normal.z());
      const Vec3 local=poses[host].inverse()*world;
      basalt::Keypoint<double> point;point.host_kf_id={100+host,0};
      point.direction=basalt::StereographicParam<double>::project(local.homogeneous());point.inv_dist=1./local.norm();
      for(int t=0;t<3;++t) {
        Eigen::Vector2d pixel;
        if(!cal.intrinsics[0].project(poses[t].inverse()*world,pixel))return 3;
        point.obs[{100+t,0}]=pixel;
      }
      points.push_back(point);
    }
    basalt::LandmarkBlock<double>::Options options;options.huber_parameter=1.;options.obs_std_dev=1.;
    std::vector<std::unique_ptr<basalt::LandmarkBlockAbsDynamic<double,6>>> blocks;
    for(auto& point:points) {
      blocks.emplace_back(std::make_unique<basalt::LandmarkBlockAbsDynamic<double,6>>());
      blocks.back()->allocateLandmark(point,relative,cal,order,options);
    }
    auto landmarks=[&] {
      std::vector<PingGroupLandmark> result;
      for(size_t i=0;i<points.size();++i) {
        blocks[i]->linearizeLandmark();const int host=int(points[i].host_kf_id.frame_id-100);
        result.push_back({i,blocks[i].get(),worldLandmark(poses[host],cal.T_i_c[0],points[i].direction,points[i].inv_dist),6*host});
      }
      return result;
    };
    Eigen::MatrixXd rays(25,3);int k=0;
    for(int y=-2;y<=2;++y)for(int x=-2;x<=2;++x) {
      double h=x*.074175,v=y*.074175;
      rays.row(k++)=Vec3(-std::cos(v)*std::sin(h),std::sin(v),std::cos(v)*std::cos(h)).transpose();
    }
    Sophus::SE3d mount;mount.translation()<<.02,.03,.1;
    PingGroupObservation observation{1000000000,1000000000,12,poses[2],mount,rays,0.,.1,.02,.2,50.};
    observation.measured=pingRange(observation.pose,mount,rays,normal,distance,.1).prediction+.04;
    auto inputs=landmarks();auto group=assemblePingGroup(18,normal,distance,inputs,{observation},.02,.23);
    bool duplicate_rejected=false;
    try {assemblePingGroup(18,normal,distance,inputs,{observation,observation},.02,.23);}
    catch(const std::invalid_argument&){duplicate_rejected=true;}
    bool time_mismatch_rejected=false;auto stale=observation;stale.pose_stamp_ns-=100000000;
    try {assemblePingGroup(18,normal,distance,inputs,{stale},.02,.23);}
    catch(const std::invalid_argument&){time_mismatch_rejected=true;}
    Eigen::VectorXd invalid=Eigen::VectorXd::Zero(3+3*points.size());invalid[3]=.01;invalid[invalid.size()-1]=-100.;
    const auto original=points[0].direction;
    const bool atomic_rejection=!group.applyLandmarks(invalid) && (points[0].direction-original).norm()==0;
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(group.jacobian.rows()+18,group.jacobian.cols());
    j.topRows(group.jacobian.rows())=group.jacobian;j.bottomLeftCorner(18,18)=5.*Eigen::MatrixXd::Identity(18,18);
    Eigen::VectorXd residual=Eigen::VectorXd::Zero(j.rows());residual.head(group.residual.size())=group.residual;
    auto reduced=eliminateJointPatch(j,residual,18);
    Eigen::VectorXd delta=reduced.jacobian.colPivHouseholderQr().solve(-reduced.residual);
    Eigen::VectorXd nuisance=reduced.backSubstitute(delta);
    Eigen::VectorXd joined(j.cols());joined<<delta,nuisance;
    const double solve_error=(joined-j.colPivHouseholderQr().solve(-residual)).norm();
    // Retire the middle pose and the entire correlated plane/landmark group.
    basalt::AbsOrderMap keep;keep.total_size=12;keep.items=2;
    keep.abs_order_map[100]={0,6};keep.abs_order_map[102]={6,6};
    basalt::BundleAdjustmentBase<double> estimator;
    for(int i=0;i<3;++i) {
      estimator.frame_poses.emplace(100+i,basalt::PoseStateWithLin<double>(100+i,poses[i],true));
      estimator.frame_poses.at(100+i).applyInc(Eigen::Matrix<double,6,1>::Constant(.003*(i+1)));
    }
    Eigen::VectorXd current_delta;estimator.computeDelta(keep,current_delta);
    const auto prior=marginalizeJointSystem(j,residual,order,keep,current_delta);
    // Marginal information must agree with the corresponding block of the
    // full joint covariance. Future retained-state rows must give the same MAP.
    Eigen::MatrixXd full_h=j.transpose()*j;
    Eigen::MatrixXd full_cov=full_h.ldlt().solve(Eigen::MatrixXd::Identity(j.cols(),j.cols()));
    std::vector<int> selected={0,1,2,3,4,5,12,13,14,15,16,17};
    Eigen::MatrixXd expected_cov(12,12);
    for(int a=0;a<12;++a)for(int b=0;b<12;++b)expected_cov(a,b)=full_cov(selected[a],selected[b]);
    Eigen::MatrixXd marginal_cov=(prior.H.transpose()*prior.H).ldlt().solve(Eigen::MatrixXd::Identity(12,12));
    const double covariance_error=(expected_cov-marginal_cov).norm();
    Eigen::MatrixXd future=Eigen::MatrixXd::Zero(12,j.cols());
    for(int a=0;a<12;++a)future(a,selected[a])=2.;
    Eigen::MatrixXd batch(j.rows()+12,j.cols());batch<<j,future;
    Eigen::VectorXd future_r=Eigen::VectorXd::LinSpaced(12,-.03,.03);
    Eigen::VectorXd batch_r(residual.size()+12);batch_r<<residual,future_r;
    Eigen::VectorXd full_step=batch.colPivHouseholderQr().solve(-batch_r);
    Eigen::MatrixXd condensed(prior.H.rows()+12,12);condensed<<prior.H,2.*Eigen::MatrixXd::Identity(12,12);
    Eigen::VectorXd condensed_r(prior.b.size()+12);condensed_r<<prior.H*current_delta+prior.b,future_r;
    const Eigen::VectorXd step=condensed.colPivHouseholderQr().solve(-condensed_r);
    Eigen::VectorXd expected_step(12);for(int a=0;a<12;++a)expected_step[a]=full_step[selected[a]];
    const double future_error=(step-expected_step).norm();
    double actual_cost;estimator.computeMargPriorError(prior,actual_cost);
    const double rebasing_error=std::abs(actual_cost+.5*prior.b.squaredNorm()-.5*(prior.H*current_delta+prior.b).squaredNorm());
    // Preserve a free common translation rather than inventing a plane prior.
    Eigen::MatrixXd free_j=Eigen::MatrixXd::Zero(2,21);Eigen::VectorXd free_r(2);free_r<<0.,3.;
    free_j(0,0)=-1.;free_j(0,6)=1.;free_j(0,18)=1.;
    const auto free_prior=marginalizeJointSystem(free_j,free_r,order,keep,current_delta);
    const bool gauge_preserved=free_prior.H.norm()<1e-12 && std::abs(free_prior.b.squaredNorm()-9.)<1e-12;
    bool invalid_order=false;auto broken=keep;broken.abs_order_map[102].first=3;
    try {marginalizeJointSystem(j,residual,order,broken,current_delta);}
    catch(const std::invalid_argument&){invalid_order=true;}
    const bool marginal_passed=covariance_error<1e-7 && future_error<1e-8 && rebasing_error<1e-12 && gauge_preserved && invalid_order;
    const double before=group.residual.squaredNorm();
    if(!group.applyLandmarks(nuisance))throw std::runtime_error("joint_trial_rejected");
    normal=Sophus::SO3d::exp(planeBasis(normal)*nuisance.head<2>())*normal;distance+=nuisance[2];
    for(int i=0;i<3;++i)basalt::PoseState<double>::incPose(delta.segment<6>(6*i),poses[i]);
    update_relative();observation.pose=poses[2];
    const auto updated=assemblePingGroup(18,normal,distance,landmarks(),{observation},.02,.23);
    const double after=updated.residual.squaredNorm()+25.*delta.squaredNorm();
    bool passed=marginal_passed && solve_error<1e-8 && after<before && duplicate_rejected && atomic_rejection && time_mismatch_rejected;
    std::cout<<Json({{"passed",passed},{"landmarks",points.size()},{"host_frames",3},
      {"marginal_covariance_error",covariance_error},{"future_factor_step_error",future_error},
      {"basalt_prior_rebasing_error",rebasing_error},{"gauge_preserved",gauge_preserved},{"invalid_order_rejected",invalid_order},
      {"full_vs_reduced_step_error",solve_error},{"objective_before",before},{"objective_after",after},
      {"duplicate_rejected",duplicate_rejected},{"time_mismatch_rejected",time_mismatch_rejected},
      {"invalid_increment_atomic_rejection",atomic_rejection}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

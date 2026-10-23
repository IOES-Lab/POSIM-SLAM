// SPDX-License-Identifier: Apache-2.0
#include "calibration.hpp"
#include "ping_group.hpp"
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
    basalt::AbsOrderMap order;order.total_size=18;
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
        result.push_back({int(i),blocks[i].get(),worldLandmark(poses[host],cal.T_i_c[0],points[i].direction,points[i].inv_dist),6*host});
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
    const double before=group.residual.squaredNorm();
    if(!group.applyLandmarks(nuisance))throw std::runtime_error("joint_trial_rejected");
    normal=Sophus::SO3d::exp(planeBasis(normal)*nuisance.head<2>())*normal;distance+=nuisance[2];
    for(int i=0;i<3;++i)basalt::PoseState<double>::incPose(delta.segment<6>(6*i),poses[i]);
    update_relative();observation.pose=poses[2];
    const auto updated=assemblePingGroup(18,normal,distance,landmarks(),{observation},.02,.23);
    const double after=updated.residual.squaredNorm()+25.*delta.squaredNorm();
    bool passed=solve_error<1e-8 && after<before && duplicate_rejected && atomic_rejection && time_mismatch_rejected;
    std::cout<<Json({{"passed",passed},{"landmarks",points.size()},{"host_frames",3},
      {"full_vs_reduced_step_error",solve_error},{"objective_before",before},{"objective_after",after},
      {"duplicate_rejected",duplicate_rejected},{"time_mismatch_rejected",time_mismatch_rejected},
      {"invalid_increment_atomic_rejection",atomic_rejection}}).dump()<<'\n';
    return passed?0:1;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

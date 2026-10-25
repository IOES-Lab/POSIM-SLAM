// SPDX-License-Identifier: Apache-2.0
#include "ping_factors.hpp"
#include <iostream>

int main() {
  try {
    using namespace posim_slam;
    constexpr int count=7,columns=6*count;
    Eigen::MatrixXd rays(1,3);rays<<0,0,1;
    Sophus::SE3d mount;mount.translation()<<.04,-.03,.1;
    Vec3 normal=Vec3(.1,-.2,1).normalized();double distance=2.;
    std::vector<PingGroupObservation> observations;
    for(int i=0;i<count;++i) {
      Sophus::SE3d pose;pose.translation()<<.1*i,.08*(i%3),.01*(i%2);
      PingGroupObservation o{100+i,100+i,6*i,pose,mount,rays,0.,.01,.03,.1,10.};
      o.per_ray_sigma=.005;o.range_loss_scale.reset();
      o.measured=expectedPingRange(pose,mount,rays,normal,distance,.01,.005,.1,10.).range.prediction+.001*i;
      observations.push_back(o);
    }
    const auto raw=assemblePingGroup(columns,normal,distance,{},observations,.03,.75,std::nullopt,true);
    double jacobian_error=0.;const double epsilon=1e-6;
    for(int col=0;col<columns+3;++col) {
      auto plus=observations,minus=observations;Vec3 np=normal,nm=normal;double dp=distance,dm=distance;
      if(col<columns) {
        const int i=col/6,k=col%6;
        if(k<3){plus[i].pose.translation()[k]+=epsilon;minus[i].pose.translation()[k]-=epsilon;}
        else {
          Vec3 delta=Vec3::Zero();delta[k-3]=epsilon;
          plus[i].pose.so3()=Sophus::SO3d::exp(delta)*plus[i].pose.so3();
          minus[i].pose.so3()=Sophus::SO3d::exp(-delta)*minus[i].pose.so3();
        }
      }else if(col<columns+2) {
        const Vec3 delta=planeBasis(normal).col(col-columns)*epsilon;
        np=Sophus::SO3d::exp(delta)*normal;nm=Sophus::SO3d::exp(-delta)*normal;
      }else {dp+=epsilon;dm-=epsilon;}
      const auto rp=assemblePingGroup(columns,np,dp,{},plus,.03,.75,std::nullopt,true).residual;
      const auto rm=assemblePingGroup(columns,nm,dm,{},minus,.03,.75,std::nullopt,true).residual;
      jacobian_error=std::max(jacobian_error,((rp-rm)/(2*epsilon)-raw.jacobian.col(col)).cwiseAbs().maxCoeff());
    }
    if(jacobian_error>1e-6)throw std::runtime_error("terrain_ping_jacobian");
    auto flat=observations;for(auto& o:flat)o.measured=2.-o.pose.translation().z()-mount.translation().z()+.01;
    const auto flat_raw=assemblePingGroup(columns,Vec3::UnitZ(),2.,{},flat,.03,.75,std::nullopt,true);
    const auto reduced=eliminateJointPatch(flat_raw.jacobian,flat_raw.residual,columns);
    for(int i=0;i<count;++i)for(int axis:{0,1,5})
      if(reduced.jacobian.col(6*i+axis).norm()>1e-10)throw std::runtime_error("invented_flat_horizontal_information");
    Eigen::VectorXd vertical_gauge=Eigen::VectorXd::Zero(columns);
    for(int i=0;i<count;++i)vertical_gauge[6*i+2]=1.;
    if((reduced.jacobian*vertical_gauge).norm()>1e-10)
      throw std::runtime_error("invented_absolute_floor_height");
    if(reduced.nuisance_rank!=3 || reduced.jacobian.rows()!=count-3)throw std::runtime_error("terrain_plane_rank");
    auto line=flat;for(auto& o:line)o.pose.translation().y()=0.;
    const auto line_raw=assemblePingGroup(columns,Vec3::UnitZ(),2.,{},line,.03,.75,std::nullopt,true);
    if(eliminateJointPatch(line_raw.jacobian,line_raw.residual,columns).nuisance_rank!=2)
      throw std::runtime_error("straight_path_cross_slope_not_observable");
    basalt::BundleAdjustmentBase<double> owner;owner.huber_thresh=1.;owner.obs_std_dev=1.;
    basalt::AbsOrderMap order;
    for(const auto& o:observations) {
      owner.frame_states.emplace(o.stamp_ns,basalt::PoseVelBiasStateWithLin<double>(
          o.stamp_ns,o.pose,Vec3::Zero(),Vec3::Zero(),Vec3::Zero(),true));
      order.abs_order_map[o.stamp_ns]={order.total_size,15};order.total_size+=15;++order.items;
    }
    PingFactorStore store(owner);PingInputCalibration calibration{mount,rays,.1,10.};
    for(int i=0;i<count;++i) {
      OwnedPingObservation o{observations[i].stamp_ns,observations[i].measured,calibration,.01,.03,.005,std::nullopt};
      if(i==0)store.add(order,0,normal,distance,{},o,.03,.75,true);
      else store.append(order,0,o);
    }
    auto groups=store.specs(order);
    const auto plan=planPingRetirement(owner,groups,{100},{},{});
    if(plan.planes!=std::set<int>{0} || plan.observations.size()!=count || !plan.landmarks.empty())
      throw std::runtime_error("terrain_retirement_loses_raw_observations");
    PingLinearization::Options options;options.lb_options.huber_parameter=1.;options.lb_options.obs_std_dev=1.;
    PingLinearization lin(&owner,order,options,groups);bool valid=false;lin.linearizeProblem(&valid);
    if(!valid)throw std::runtime_error("range_only_native_group_invalid");lin.performQR();
    Eigen::MatrixXd j;Eigen::VectorXd r;lin.get_dense_Q2Jp_Q2r(j,r);
    const auto full=groups[0].build({});
    // Independent dense Schur complement, retaining all state coordinates.
    const Eigen::MatrixXd a=full.jacobian.leftCols(order.total_size),b=full.jacobian.rightCols(3);
    const Eigen::MatrixXd h=a.transpose()*a-a.transpose()*b*(b.transpose()*b).ldlt().solve(b.transpose()*a);
    const Eigen::VectorXd g=a.transpose()*full.residual-a.transpose()*b*(b.transpose()*b).ldlt().solve(b.transpose()*full.residual);
    if((j.transpose()*j-h).norm()>1e-8 || (j.transpose()*r-g).norm()>1e-8)
      throw std::runtime_error("terrain_joint_schur_mismatch");
    for(double damping:{.001,1.,1000.}) {
      lin.dampNuisance(damping);lin.get_dense_Q2Jp_Q2r(j,r);
      Eigen::Matrix3d hn=b.transpose()*b;
      hn.diagonal().array()+=(b.colwise().squaredNorm().transpose().array()*damping).max(1e-9);
      const Eigen::MatrixXd hd=a.transpose()*a-a.transpose()*b*hn.ldlt().solve(b.transpose()*a);
      const Eigen::VectorXd gd=a.transpose()*full.residual-a.transpose()*b*hn.ldlt().solve(b.transpose()*full.residual);
      if((j.transpose()*j-hd).norm()>1e-8 || (j.transpose()*r-gd).norm()>1e-8)
        throw std::runtime_error("terrain_damped_schur_mismatch");
      const auto damped=eliminateJointPatch(full.jacobian,full.residual,order.total_size,damping);
      const Eigen::VectorXd step=Eigen::VectorXd::Constant(order.total_size,.001);
      if((damped.backSubstitute(step)+hn.ldlt().solve(b.transpose()*(full.residual+a*step))).norm()>1e-8)
        throw std::runtime_error("terrain_damped_backsub_mismatch");
    }
    // A new undamped linearization must drop all trial regularization.
    lin.linearizeProblem();lin.performQR();lin.get_dense_Q2Jp_Q2r(j,r);
    if((j.transpose()*j-h).norm()>1e-8)throw std::runtime_error("terrain_damping_leaked_to_prior_rows");
    bool duplicate=false;try{store.append(order,0,{106,2.,calibration,.01,.03,.005});}
    catch(const std::invalid_argument&){duplicate=true;}
    if(!duplicate)throw std::runtime_error("terrain_duplicate_measurement");
    std::cout<<"{\"passed\":true,\"jacobian_max_error\":"<<jacobian_error
      <<",\"flat_horizontal_information\":0,\"straight_plane_rank\":2,\"retired_observations\":7}\n";
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

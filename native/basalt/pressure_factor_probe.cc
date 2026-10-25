// SPDX-License-Identifier: Apache-2.0
#include "pressure_input.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include <iostream>
using namespace posim_slam;
void require(bool value,const char* reason) {if(!value)throw std::runtime_error(reason);}
template<class F> void rejects(F action,const char* reason) {
  bool rejected=false;try{action();}catch(const std::invalid_argument&){rejected=true;}require(rejected,reason);
}
int main() {try {
  PressureCalibration c;c.imu_to_pressure=Eigen::Vector3d(.2,-.1,.3);
  basalt::PoseVelBiasState<double> s;s.t_ns=100;s.vel_w_i.setZero();s.bias_gyro.setZero();s.bias_accel.setZero();
  s.T_w_i=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.3,-.2,.4)),Eigen::Vector3d(1,2,-3));
  PressureObservation o{100,-1,.04};const double datum=1.5;
  const auto rows=pressureRow(s.T_w_i,s.T_w_i,datum,c,o,false);
  Eigen::Matrix<double,1,6> finite;
  for(int col=0;col<6;++col) {
    auto plus=s,minus=s;Eigen::Matrix<double,15,1> d=Eigen::Matrix<double,15,1>::Zero();d[col]=1e-6;
    plus.applyInc(d);minus.applyInc(-d);
    finite[col]=(pressureRow(plus.T_w_i,plus.T_w_i,datum,c,o,false).residual-
                 pressureRow(minus.T_w_i,minus.T_w_i,datum,c,o,false).residual)/2e-6;
  }
  const double jacobian_error=(rows.pose-finite).norm();require(jacobian_error<1e-8,"pressure_jacobian");
  auto shifted=s;shifted.T_w_i.translation()+=Eigen::Vector3d(4,5,6);
  require(std::abs(pressureRow(shifted.T_w_i,shifted.T_w_i,datum-6,c,o,false).residual-rows.residual)<1e-12,"pressure_translation_datum_gauge");
  require((pressureRow(shifted.T_w_i,s.T_w_i,datum,c,o,false).pose-rows.pose).norm()==0,"pressure_fej");
  auto outlier=o;outlier.height=100;
  require(pressureRow(s.T_w_i,s.T_w_i,datum,c,outlier).cost<pressureRow(s.T_w_i,s.T_w_i,datum,c,outlier,false).cost,"pressure_huber");
  for(double variance:{0.,-1.,std::numeric_limits<double>::quiet_NaN()})rejects([&]{validatePressure(c,{100,0,variance});},"pressure_bad_variance");

  basalt::BundleAdjustmentBase<double> owner;basalt::AbsOrderMap order;PressureFactorStore store(c);
  for(int64_t stamp:{100,200,300}) {
    auto pose=s.T_w_i;pose.translation().z()+=stamp*.001;
    owner.frame_states.emplace(stamp,basalt::PoseVelBiasStateWithLin<double>(stamp,pose,s.vel_w_i,s.bias_gyro,s.bias_accel,true));
    order.abs_order_map[stamp]={order.total_size,15};order.total_size+=15;++order.items;
    store.add({stamp,(pose.translation()+pose.rotationMatrix()*c.imu_to_pressure).z()+7,.04},owner);
  }
  require(std::abs(store.datum()-7)<1e-12,"pressure_seed_is_offset");
  const auto all=store.linearize(owner,order);require(all.residual.norm()<1e-12,"pressure_initial_fit");
  {
    PressureFactorStore::Trial trial(store);trial.apply(10);
    require(store.datum()>16,"pressure_trial_apply");
  }
  require(std::abs(store.datum()-7)<1e-12,"pressure_trial_rollback");
  basalt::MargLinData<double> previous;previous.is_sqrt=true;previous.order=order;
  previous.H=10*Eigen::MatrixXd::Identity(45,45);previous.H.block<3,3>(2,32)=.7*Eigen::Matrix3d::Identity();
  previous.b=Eigen::VectorXd::Constant(45,.01);
  basalt::ImuLinData<double> imu={Eigen::Vector3d(0,0,-9.80665),Eigen::Vector3d::Ones(),Eigen::Vector3d::Ones(),{}};
  const auto prepared=preparePingPrior(owner,order,{},previous,imu,{100},{200},{},nullptr,nullptr,&store);
  require(prepared.pressure->stamps==std::set<int64_t>{100},"pressure_pose_only_retains_observation");
  const auto consumed=store.linearize(owner,order,&prepared.pressure->stamps);
  Eigen::MatrixXd j=Eigen::MatrixXd::Zero(45+consumed.residual.size(),46);Eigen::VectorXd residual(j.rows());
  j.topLeftCorner(45,45)=previous.H;j.bottomRows(consumed.residual.size())=consumed.jacobian;
  residual<<previous.b,consumed.residual;
  std::vector<int> keep,remove;
  for(int i=0;i<46;++i) {
    if((i>=15 && i<21) || i>=30)keep.push_back(i);else remove.push_back(i);
  }
  const Eigen::MatrixXd h=j.transpose()*j;const Eigen::VectorXd g=j.transpose()*residual;
  Eigen::MatrixXd hkk(22,22),hkm(22,24),hmm(24,24);Eigen::VectorXd gk(22),gm(24);
  for(int i=0;i<22;++i) {gk[i]=g[keep[i]];for(int k=0;k<22;++k)hkk(i,k)=h(keep[i],keep[k]);
    for(int k=0;k<24;++k)hkm(i,k)=h(keep[i],remove[k]);}
  for(int i=0;i<24;++i){gm[i]=g[remove[i]];for(int k=0;k<24;++k)hmm(i,k)=h(remove[i],remove[k]);}
  const Eigen::MatrixXd expected=hkk-hkm*hmm.ldlt().solve(hkm.transpose());
  const Eigen::VectorXd expected_g=gk-hkm*hmm.ldlt().solve(gm);
  const auto& state_prior=prepared.prior;const auto& pp=prepared.pressure_prior;
  require(pp.active,"pressure_datum_prior_retained");
  Eigen::MatrixXd actual=Eigen::MatrixXd::Zero(state_prior.H.rows()+1,22);
  actual.topLeftCorner(state_prior.H.rows(),21)=state_prior.H;
  actual.bottomLeftCorner(1,21)=pp.states;actual(actual.rows()-1,21)=pp.datum;
  Eigen::VectorXd actual_r(actual.rows());actual_r<<state_prior.b,pp.b;
  const double prior_error=(actual.transpose()*actual-expected).norm();
  require(prior_error<1e-8,"pressure_joint_prior_information");
  require((actual.transpose()*actual_r-expected_g).norm()<1e-8,"pressure_joint_prior_gradient");
  try{store.retireWith(*prepared.pressure,pp,[]{throw std::runtime_error("injected");});}catch(const std::runtime_error&){}
  require(store.size()==3 && !store.prior().active,"pressure_failed_commit_atomic");
  store.retireWith(*prepared.pressure,pp,[]{});
  require(store.size()==2 && store.prior().active,"pressure_commit");
  require(store.linearize(owner,prepared.layout.retained).residual.size()==3,"pressure_prior_plus_live_only_once");
  rejects([&]{store.retireWith(*prepared.pressure,pp,[]{});},"pressure_stale_commit");
  rejects([&]{store.add(o,owner);},"pressure_replay_after_retirement");

  // A single pressure sample with a free datum supplies no absolute height.
  basalt::AbsOrderMap one;one.items=1;one.total_size=15;one.abs_order_map[100]={0,15};
  Eigen::MatrixXd onej=Eigen::MatrixXd::Zero(1,16);onej(0,2)=5;onej(0,15)=5;
  const auto null=eliminateJointPatch(onej,Eigen::VectorXd::Ones(1),15);
  require(null.jacobian.norm()==0 && null.residual.norm()==0,"pressure_first_sample_not_absolute_anchor");
  std::cout<<nlohmann::json({{"passed",true},{"jacobian_error",jacobian_error},{"prior_information_error",prior_error},
    {"shared_datum_preserved",true},{"first_sample_anchor",false}}).dump()<<'\n';return 0;
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

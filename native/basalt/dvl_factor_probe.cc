// SPDX-License-Identifier: Apache-2.0
#include "dvl_input.hpp"
#include "ping_optimizer.hpp"
#include "ping_prior.hpp"
#include <iostream>

using namespace posim_slam;
using State=basalt::PoseVelBiasState<double>;
void require(bool value,const char* reason) {if(!value)throw std::runtime_error(reason);}
template<class F> void rejects(F action,const char* reason) {
  bool rejected=false;try{action();}catch(const std::invalid_argument&){rejected=true;}require(rejected,reason);
}
DvlObservation sample(int64_t stamp) {
  DvlObservation o;o.stamp=stamp;o.velocity=Eigen::Vector3d(.2,-.1,.05);o.gyro=Eigen::Vector3d(.1,-.2,.3);
  o.covariance<<.04,.003,-.002,.003,.03,.001,-.002,.001,.02;
  o.gyro_covariance=.001*Eigen::Matrix3d::Identity();return o;
}
State state(int64_t stamp) {
  State s;s.t_ns=stamp;s.T_w_i=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.2,-.1,.4)),Eigen::Vector3d(1,2,-3));
  s.vel_w_i=Eigen::Vector3d(.5,.2,-.1);s.bias_gyro=Eigen::Vector3d(.01,-.02,.03);s.bias_accel.setZero();return s;
}
int main() {
 try {
  DvlCalibration c;c.dvl_to_imu=Sophus::SO3d::exp(Eigen::Vector3d(.3,.2,-.4)).matrix();c.imu_to_dvl=Eigen::Vector3d(.2,-.1,.08);
  auto s=state(100);auto o=sample(100);const auto rows=dvlRows(s,s,c,o,false);
  Eigen::Matrix<double,3,15> finite;
  for(int col=0;col<15;++col) {
    auto plus=s,minus=s;Eigen::Matrix<double,15,1> delta=Eigen::Matrix<double,15,1>::Zero();delta[col]=1e-6;
    plus.applyInc(delta);minus.applyInc(-delta);
    finite.col(col)=(dvlRows(plus,plus,c,o,false).residual-dvlRows(minus,minus,c,o,false).residual)/2e-6;
  }
  const double jacobian_error=(rows.jacobian-finite).norm();require(jacobian_error<1e-8,"dvl_jacobian");
  const Eigen::Matrix3d a=c.dvl_to_imu.transpose()*dvlCross(c.imu_to_dvl),w=dvlWhitening(c,o);
  const double whitening_error=(w*(o.covariance+a*o.gyro_covariance*a.transpose())*w.transpose()-Eigen::Matrix3d::Identity()).norm();
  require(whitening_error<1e-12,"dvl_correlated_covariance");
  auto zero=c;zero.imu_to_dvl.setZero();require(dvlRows(s,s,zero,o).jacobian.block<3,3>(0,9).norm()==0,"zero_lever_bias");
  const auto yaw=Sophus::SO3d::exp(Eigen::Vector3d(0,0,.7));auto changed=s;
  changed.T_w_i=Sophus::SE3d(yaw,Eigen::Vector3d(2,-1,3))*s.T_w_i;changed.vel_w_i=yaw*s.vel_w_i;
  require((dvlRows(changed,changed,c,o,false).residual-rows.residual).norm()<1e-12,"dvl_world_gauge");
  changed=s;changed.vel_w_i+=Eigen::Vector3d(.1,.2,.3);
  require((dvlRows(changed,s,c,o,false).jacobian-rows.jacobian).norm()==0,"dvl_fej_reference");
  auto outlier=o;outlier.velocity=Eigen::Vector3d(20,0,0);
  require(dvlRows(s,s,c,outlier).cost<dvlRows(s,s,c,outlier,false).cost,"dvl_huber_cost");
  for(int kind=0;kind<4;++kind) {
    auto bad=o;if(kind==0)bad.covariance(0,0)=-1;if(kind==1)bad.covariance(0,1)+=.1;
    if(kind==2)bad.gyro_covariance(0,0)=-1;if(kind==3)bad.velocity[0]=NAN;
    rejects([&]{dvlWhitening(c,bad);},"invalid_dvl_accepted");
  }
  auto bad_mount=c;bad_mount.dvl_to_imu(0,0)+=.1;
  rejects([&]{dvlWhitening(bad_mount,o);},"invalid_mount_accepted");
  std::vector<char> packet;
  auto integer=[&](uint64_t value,int width){for(int i=0;i<width;++i)packet.push_back(char((value>>(8*i))&255));};
  auto number=[&](double value){uint64_t bits;std::memcpy(&bits,&value,8);integer(bits,8);};
  integer(100,8);integer(100,8);integer(1,4);integer(4,4);
  for(int i=0;i<3;++i)number(o.velocity[i]);for(int i=0;i<3;++i)number(o.gyro[i]);
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)number(o.covariance(i,j));
  for(int i=0;i<3;++i)for(int j=0;j<3;++j)number(o.gyro_covariance(i,j));
  require((decodeDvlObservation(packet,c).covariance-o.covariance).norm()==0,"dvl_packet_roundtrip");
  for(int offset:{8,16,20}) {auto bad=packet;bad[offset]=0;rejects([&]{decodeDvlObservation(bad,c);},"dvl_packet_identity");}
  auto bad=packet;bad.pop_back();rejects([&]{decodeDvlObservation(bad,c);},"dvl_packet_length");

  basalt::BundleAdjustmentBase<double> owner;basalt::AbsOrderMap order;
  DvlFactorStore store(c);
  for(int64_t stamp:{100,200,300}) {
    const auto st=state(stamp);
    owner.frame_states.emplace(stamp,basalt::PoseVelBiasStateWithLin<double>(stamp,st.T_w_i,st.vel_w_i,st.bias_gyro,st.bias_accel,true));
    order.abs_order_map[stamp]={order.total_size,15};order.total_size+=15;++order.items;store.add(sample(stamp));
  }
  const auto plan=store.planRetirement({100},{200});require(plan.stamps==std::set<int64_t>({100,200}),"dvl_pose_only_retirement");
  basalt::MargLinData<double> previous;previous.is_sqrt=true;previous.order=order;
  previous.H=Eigen::MatrixXd::Identity(45,45);previous.H.block<3,3>(6,36)=.7*Eigen::Matrix3d::Identity();previous.b=Eigen::VectorXd::Constant(45,.01);
  basalt::ImuLinData<double> imu={Eigen::Vector3d(0,0,-9.80665),Eigen::Vector3d::Ones(),Eigen::Vector3d::Ones(),{}};
  const auto prepared=preparePingPrior(owner,order,{},previous,imu,{100},{200},{},nullptr,&store);
  const auto consumed=store.linearize(owner,order,&plan.stamps);
  Eigen::MatrixXd j(45+consumed.jacobian.rows(),45);Eigen::VectorXd r(45+consumed.residual.size());
  j<<previous.H,consumed.jacobian;r<<previous.b,consumed.residual;
  const auto expected=marginalizeJointSystem(j,r,order,prepared.layout.retained,Eigen::VectorXd::Zero(21));
  const double prior_error=(prepared.prior.H.transpose()*prepared.prior.H-expected.H.transpose()*expected.H).norm();
  require(prior_error<1e-9,"dvl_prior_information");
  require((prepared.prior.H.transpose()*prepared.prior.b-expected.H.transpose()*expected.b).norm()<1e-9,"dvl_prior_gradient");
  // A failed owner commit must preserve all raw observations and the plan.
  try{store.retireWith(plan,[]{throw std::runtime_error("injected");});}catch(const std::runtime_error&){}
  require(store.size()==3,"dvl_failed_commit_mutated");store.retireWith(plan,[]{});
  require(store.size()==1 && store.lastObservation()==300,"dvl_retirement_watermark");
  rejects([&]{store.retireWith(plan,[]{});},"dvl_stale_retirement");
  rejects([&]{store.add(o);},"dvl_replay_after_retirement");
  require(store.linearize(owner,order).residual.size()==3,"retained_dvl_lost");

  basalt::BundleAdjustmentBase<double> fit;basalt::AbsOrderMap one;one.items=1;one.total_size=15;one.abs_order_map[100]={0,15};
  fit.frame_states.emplace(100,basalt::PoseVelBiasStateWithLin<double>(100,Sophus::SE3d(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),true));
  DvlFactorStore velocity(DvlCalibration{});velocity.add(o);
  auto anchor=previous;anchor.order=one;anchor.H=10*Eigen::MatrixXd::Identity(15,15);anchor.H.block<3,3>(6,6).setZero();anchor.b=Eigen::VectorXd::Zero(15);
  const auto report=optimizePingGroups(fit,one,{},anchor,imu,{},nullptr,&velocity);
  const double velocity_error=(fit.frame_states.at(100).getState().vel_w_i-o.velocity).norm();
  require(report.accepted>0 && report.final_cost<report.initial_cost && velocity_error<1e-5,"dvl_optimizer_decrease");
  std::cout<<nlohmann::json({{"passed",true},{"jacobian_error",jacobian_error},{"whitening_error",whitening_error},{"prior_information_error",prior_error},{"velocity_error",velocity_error},{"accepted_steps",report.accepted}}).dump()<<'\n';
  return 0;
 } catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}

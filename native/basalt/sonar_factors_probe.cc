// SPDX-License-Identifier: Apache-2.0
#include "sonar_factors.hpp"
#include <cassert>
#include <iostream>
#include <functional>

void reject(const std::function<void()>& operation) {
  bool rejected=false;try {operation();}catch(const std::exception&){rejected=true;}
  assert(rejected);
}
int main() {
  using namespace posim_slam;
  const Eigen::Vector3d truth(4,.5,.3),seed(4.02,.51,.32);
  SonarCalibration c{Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.2,10.,2.1,.35};
  SonarPoses poses;
  std::vector<SonarObservation> observations;
  for(int i=0;i<4;++i) {
    const int64_t stamp=1000*(i+1);
    const Eigen::Vector3d position(0,.2*i,.05*i*i);
    poses.emplace(stamp,SonarPose{position,Eigen::Matrix3d::Identity()});
    const Eigen::Vector3d local=truth-position;
    observations.push_back({stamp,local.norm(),std::atan2(local.y(),local.x())});
  }
  Eigen::MatrixXd covariance=Eigen::MatrixXd::Identity(6,6)*1e-4;
  covariance(0,2)=covariance(2,0)=2e-5;
  SonarFactorStore store;
  store.add(7,seed,c,{observations[0],observations[1],observations[2]},covariance,poses);
  const double before=store.cost(poses);
  const auto rows=store.linearize(poses);
  const auto delta=Eigen::VectorXd::Zero(24).eval();
  reject([&]{store.apply(rows,delta);});
  {
    SonarFactorStore::Trial trial(store);
    reject([&]{SonarFactorStore::Trial nested(store);});
    reject([&]{store.linearize(poses);});
    reject([&]{store.append(7,observations[3],Eigen::MatrixXd::Identity(8,8),poses);});
    store.apply(rows,delta);
    assert(store.cost(poses)<before);
    reject([&]{store.apply(rows,delta);});
    // Scope exit models rejection, including an exception in nonlinear cost.
  }
  assert((store.point(7)-seed).norm()==0);
  assert(store.cost(poses)==before);
  {
    SonarFactorStore::Trial trial(store);
    reject([&]{store.apply(rows,delta);}); // rejected trial invalidates rows
  }
  const auto fresh=store.linearize(poses);
  {
    SonarFactorStore::Trial trial(store);
    store.apply(fresh,delta);
    assert(store.cost(poses)<before);
    trial.finish(true);
  }
  assert((store.point(7)-seed).norm()>0);
  const auto accepted=store.point(7);
  Eigen::MatrixXd extended=Eigen::MatrixXd::Identity(8,8)*1e-4;
  extended.topLeftCorner(6,6)=covariance;
  extended(0,6)=extended(6,0)=1e-5;
  auto changed=extended;changed(0,0)+=1e-5;
  reject([&]{store.append(7,observations[3],changed,poses);});
  assert((store.point(7)-accepted).norm()==0);
  store.append(7,observations[3],extended,poses);
  assert(store.linearize(poses).rows.at(7).residual.size()==8);
  reject([&]{store.append(7,observations[3],extended,poses);});
  reject([&]{auto missing=poses;missing.erase(1000);store.cost(missing);});
  reject([&]{store.add(7,seed,c,{observations[0]},Eigen::MatrixXd::Identity(2,2),poses);});
  // Nonfinite steps must not partially change the store.
  const auto last=store.linearize(poses);
  {
    SonarFactorStore::Trial trial(store);
    auto invalid=delta;invalid[0]=std::numeric_limits<double>::quiet_NaN();
    reject([&]{store.apply(last,invalid);});
    assert((store.point(7)-accepted).norm()==0);
  }
  std::cout<<"PASS native sonar point update, trial rollback, observation/covariance ownership and stale-row rejection\n";
}

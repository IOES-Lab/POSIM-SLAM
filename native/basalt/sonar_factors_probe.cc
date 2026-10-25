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
  {
    // At (4,0,0), elevation's only local derivative is dz/4. Compare against
    // the known diagonal covariance, including weak-but-nonzero information.
    Eigen::Matrix3d j=Eigen::Vector3d(100.,50.,2.).asDiagonal();
    const Eigen::Vector3d p(4.,0.,0.);
    assert(std::abs(sonarConditionalElevationSigma(j,p,Eigen::Matrix3d::Identity())-.125)<1e-12);
    assert(std::abs(sonarConditionalElevationSigma(j/2.,p,Eigen::Matrix3d::Identity())-.25)<1e-12);
    const Eigen::Matrix3d r=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    assert(std::abs(sonarConditionalElevationSigma(j*r.transpose(),p,r.transpose())-.125)<1e-12);
    j(2,2)=1e-5;
    assert(sonarConditionalElevationSigma(j,p,Eigen::Matrix3d::Identity())>10000.);
    j(2,2)=0.;
    assert(std::isinf(sonarConditionalElevationSigma(j,p,Eigen::Matrix3d::Identity())));
    reject([&]{sonarConditionalElevationSigma(j,Eigen::Vector3d::Zero(),r);});
  }
  {
    const Eigen::VectorXd small=Eigen::VectorXd::Constant(6,.1);
    assert(std::abs(sonarTrackLoss(small).cost-.5*small.squaredNorm())<1e-12);
    assert(sonarTrackLoss(small).sqrt_weight==1.);
    Eigen::VectorXd outlier(6);outlier<<12.,-8.,5.,3.,-2.,1.;
    const auto robust=sonarTrackLoss(outlier);
    assert(robust.sqrt_weight<1. && robust.cost<.5*outlier.squaredNorm());
    // The IRLS gradient must be the derivative of the actual trial objective.
    for(int i=0;i<6;++i) {
      auto plus=outlier,minus=outlier;plus[i]+=1e-5;minus[i]-=1e-5;
      const double numerical=(sonarTrackLoss(plus).cost-sonarTrackLoss(minus).cost)/2e-5;
      assert(std::abs(numerical-robust.sqrt_weight*robust.sqrt_weight*outlier[i])<1e-7);
    }
    Eigen::MatrixXd rotation=Eigen::MatrixXd::Identity(6,6);
    rotation(0,0)=rotation(1,1)=std::cos(.6);rotation(0,1)=-std::sin(.6);rotation(1,0)=std::sin(.6);
    assert(std::abs(sonarTrackLoss(rotation*outlier).cost-robust.cost)<1e-12);
  }
  const Eigen::Vector3d truth(4,.5,.3),seed(4.02,.51,.32);
  SonarCalibration c{Eigen::Vector3d::Zero(),Eigen::Matrix3d::Identity(),.2,10.,2.1,.35};
  SonarPoses poses;
  std::vector<SonarObservation> observations;
  for(int i=0;i<4;++i) {
    const int64_t stamp=1000*(i+1);
    const Eigen::Vector3d position(0,.2*i,.08*i*i);
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
  auto outside_references=poses;
  for(auto& [stamp,pose]:outside_references)pose.position.z()-=2.;
  reject([&]{store.cost(outside_references);});
  const auto reference_rows=store.linearize(poses,&outside_references);
  assert(reference_rows.rows.at(7).pose_jacobian.allFinite());
  assert((reference_rows.rows.at(7).residual-rows.rows.at(7).residual).norm()<1e-12);
  // Disabling reference support cannot hide an undefined range/azimuth derivative.
  auto singular_references=poses;singular_references.begin()->second.position=seed;
  reject([&]{store.linearize(poses,&singular_references);});
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
  const auto fixed=store.linearize(poses,&poses);
  auto moved=poses;for(auto& [stamp,pose]:moved)pose.position.x()+=.01;
  const auto changed_pose=store.linearize(moved,&poses);
  assert((fixed.rows.at(7).pose_jacobian-changed_pose.rows.at(7).pose_jacobian).norm()==0);
  assert((fixed.rows.at(7).residual-changed_pose.rows.at(7).residual).norm()>0);
  auto cross=[](const Eigen::Vector3d& v){Eigen::Matrix3d m;m<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return m;};
  Eigen::MatrixXd gauge=Eigen::MatrixXd::Zero(27,6);int offset=0;
  for(const auto& [stamp,pose]:poses) {
    gauge.block<3,3>(offset,0).setIdentity();gauge.block<3,3>(offset,3)=-cross(pose.position);
    gauge.block<3,3>(offset+3,3).setIdentity();offset+=6;
  }
  gauge.block<3,3>(24,0).setIdentity();gauge.block<3,3>(24,3)=-cross(seed);
  Eigen::MatrixXd joint(6,27);joint<<fixed.rows.at(7).pose_jacobian,fixed.rows.at(7).point_jacobian;
  assert((joint*gauge).norm()<1e-9);
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
  const auto retirement=store.planRetirement({1000});
  assert(retirement.ids==std::set<int64_t>{7} && retirement.required_poses.size()==4);
  reject([&]{store.retireWith(retirement,[]{throw std::runtime_error("install_failure");});});
  assert(store.size()==1);
  store.retireWith(retirement,[]{});
  assert(store.size()==0);
  reject([&]{store.retireWith(retirement,[]{});});
  reject([&]{store.add(7,seed,c,{observations[0]},Eigen::MatrixXd::Identity(2,2),poses);});
  SonarFactorStore batch;
  std::vector<SonarTrackInput> updates{{10,{observations[0],observations[1],observations[2]},covariance},
                                     {11,{observations[0],observations[1],observations[2]},covariance}};
  batch.applySnapshot(3000,updates,c,poses);
  const auto batch_revision=batch.revision();const auto batch_point=batch.point(10);
  assert((batch_point-truth).norm()<1e-5);
  {
    // Exact data, unique multi-view solution and full rank do not imply useful
    // elevation precision. With the same declared noise, a longer baseline
    // resolves it. The deferred source can be retried with another native ID.
    SonarPoses weak_poses;std::vector<SonarObservation> weak_observations;
    for(int i=0;i<4;++i) {
      const Eigen::Vector3d position(0,.2*i,.05*i*i),local=truth-position;
      weak_poses.emplace(1000*(i+1),SonarPose{position,Eigen::Matrix3d::Identity()});
      weak_observations.push_back({1000*(i+1),local.norm(),std::atan2(local.y(),local.x())});
    }
    SonarFactorStore deferred;std::map<int64_t,std::string> reasons;
    assert(deferred.applySnapshot(3000,{{10,{weak_observations[0],weak_observations[1],weak_observations[2]},covariance}},
      c,weak_poses,&reasons)==std::vector<int64_t>{10});
    assert(reasons.at(10)=="sonar_initialization_elevation_underconstrained" && deferred.size()==0);
    assert(deferred.applySnapshot(4000,{{11,weak_observations,extended}},c,weak_poses,&reasons).empty());
    assert(reasons.empty() && (deferred.point(11)-truth).norm()<1e-5);
  }
  // A different map origin/orientation must not alter admission or the fit.
  {
    const Eigen::Matrix3d rotation=Eigen::AngleAxisd(.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
    const Eigen::Vector3d translation(1e6,-2e6,3e6);
    auto transformed=poses;
    for(auto& [stamp,pose]:transformed) {
      pose.position=rotation*pose.position+translation;
      pose.attitude=rotation*pose.attitude;
    }
    SonarFactorStore moved_store;
    assert(moved_store.applySnapshot(3000,updates,c,transformed).empty());
    assert((rotation.transpose()*(moved_store.point(10)-translation)-truth).norm()<1e-5);
  }
  {
    auto inconsistent=updates;inconsistent[0].observations[1].range+=2.;
    inconsistent.resize(1);SonarFactorStore rejected_store;
    assert(rejected_store.applySnapshot(3000,inconsistent,c,poses)==std::vector<int64_t>{10});
    assert(rejected_store.size()==0);
  }
  // Three equal-height views leave two mirror elevations; stationary views
  // leave a continuous elevation ambiguity. Neither may seed a fixed 3D point.
  for(bool stationary:{false,true}) {
    SonarFactorStore ambiguous;SonarPoses flat;
    std::vector<SonarObservation> track;
    for(int i=0;i<3;++i) {
      const Eigen::Vector3d position(0.,stationary?0.:.2*i,0.);
      flat.emplace(1000*(i+1),SonarPose{position,Eigen::Matrix3d::Identity()});
      const Eigen::Vector3d local=truth-position;
      track.push_back({1000*(i+1),local.norm(),std::atan2(local.y(),local.x())});
    }
    assert(ambiguous.applySnapshot(3000,{{10,track,covariance}},c,flat)==std::vector<int64_t>{10});
    assert(ambiguous.size()==0);
  }
  for(auto& update:updates){update.observations.push_back(observations[3]);update.covariance=extended;}
  updates[1].observations[0].range+=.1;
  reject([&]{batch.applySnapshot(4000,updates,c,poses);});
  assert(batch.revision()==batch_revision && (batch.point(10)-batch_point).norm()==0);
  updates[1].observations[0].range-=.1;
  // Restore the exact original binary value rather than floating roundoff.
  updates[1].observations[0]=observations[0];
  auto changed_calibration=c;changed_calibration.lever.x()=.01;
  reject([&]{batch.applySnapshot(4000,updates,changed_calibration,poses);});
  batch.applySnapshot(4000,updates,c,poses);
  assert(batch.linearize(poses).rows.at(10).residual.size()==8 && batch.linearize(poses).rows.at(11).residual.size()==8);
  reject([&]{batch.applySnapshot(4000,updates,c,poses);});
  // Geometrically unsupported associations are rejected without killing VIO.
  // Malformed covariance/measurements still abort the atomic input batch.
  SonarFactorStore admission;
  std::vector<SonarTrackInput> candidates{{10,{observations[0],observations[1],observations[2]},covariance},
                                        {11,{observations[0],observations[1],observations[2]},covariance}};
  candidates[1].observations[0].range=.4;
  auto invalid_candidates=candidates;invalid_candidates[1].covariance(0,0)=-1;
  reject([&]{admission.applySnapshot(3000,invalid_candidates,c,poses);});
  assert(admission.size()==0);
  invalid_candidates=candidates;invalid_candidates[1].observations[2].range=-1;
  reject([&]{admission.applySnapshot(3000,invalid_candidates,c,poses);});
  assert(admission.size()==0);
  const auto rejected=admission.applySnapshot(3000,candidates,c,poses);
  assert(rejected==std::vector<int64_t>{11} && admission.activeIds()==std::vector<int64_t>{10});
  const auto original_point=admission.point(10);const auto original_revision=admission.revision();
  auto unsupported=poses;unsupported.at(4000).position.z()=4.;
  SonarTrackInput extension{10,observations,extended};
  const auto rejected_extension=admission.applySnapshot(4000,{extension},c,unsupported);
  assert(rejected_extension==std::vector<int64_t>{10});
  assert(admission.revision()==original_revision && (admission.point(10)-original_point).norm()==0);
  assert(admission.linearize(poses).rows.at(10).residual.size()==6);
  std::cout<<"PASS native sonar point update, trial rollback, observation/covariance ownership and stale-row rejection\n";
}

// SPDX-License-Identifier: Apache-2.0
#include "submap_information.hpp"
#include "submap_linearization.hpp"
#include <cassert>
#include <iostream>
#include <numeric>

namespace {
using Store=posim_slam::SubmapInformation;
std::vector<int64_t> coordinates(int start,int count) {
  std::vector<int64_t> out(count);std::iota(out.begin(),out.end(),start);return out;
}
template<class F> void rejects(F action,const std::string& reason) {
  bool rejected=false;try{action();}catch(const std::exception& error){assert(error.what()==reason);rejected=true;}
  assert(rejected);
}
template<class A,class B> void close(const A& a,const B& b,double tolerance=1e-9) {
  assert(a.rows()==b.rows() && a.cols()==b.cols());
  assert((a-b).norm()<=tolerance*std::max(1.,b.norm()));
}
void denseAndLateAdjustment() {
  const int n=12;Eigen::MatrixXd j(23,n);Eigen::VectorXd r(23);
  for(int row=0;row<j.rows();++row) {
    r[row]=std::sin(.17*row);
    for(int col=0;col<n;++col)j(row,col)=std::cos(.23*row+.31*col)+(row==col?2.:0.);
  }
  Store store;store.append({"original-world-and-observation-rows"},coordinates(0,n),j,r);
  store.seal({9,6,8,7});store.seal({8,9});
  const auto old_snapshot=store.snapshot();const auto old_solution=store.solve(old_snapshot);
  const auto baseline=store.query(old_solution,coordinates(0,n),Eigen::MatrixXd::Identity(n,n));
  const Eigen::MatrixXd h=j.transpose()*j;const Eigen::MatrixXd covariance=h.ldlt().solve(Eigen::MatrixXd::Identity(n,n));
  const Eigen::VectorXd mean=-h.ldlt().solve(j.transpose()*r);
  assert(baseline.observed);close(baseline.mean,mean);close(baseline.covariance,covariance);
  // New separator information must move old geometry and its shared covariance.
  Eigen::MatrixXd late(2,n);late.setZero();late(0,8)=4;late(1,9)=3;
  Eigen::Vector2d residual(1.2,-.7);
  Eigen::Matrix2d local=Eigen::Matrix2d::Zero();local(0,0)=4;local(1,1)=3;
  store.append({"new-separator-observation"},{8,9},local,residual);
  const auto revised=store.query(store.solve(store.snapshot()),coordinates(0,n),Eigen::MatrixXd::Identity(n,n));
  const Eigen::MatrixXd updated_h=h+late.transpose()*late;
  close(revised.mean,-updated_h.ldlt().solve(j.transpose()*r+late.transpose()*residual));
  close(revised.covariance,updated_h.ldlt().solve(Eigen::MatrixXd::Identity(n,n)));
  assert((revised.mean.head(8)-baseline.mean.head(8)).norm()>1e-3);
  close(store.query(old_solution,coordinates(0,n),Eigen::MatrixXd::Identity(n,n)).covariance,baseline.covariance);
  const auto before=store.snapshot();
  rejects([&]{store.append({"new-separator-observation"},{8},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1));},"submap_measurement_reused");
  rejects([&]{store.append({"late-old-coordinate"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1));},"submap_requires_retired_factor_update");
  assert(store.snapshot()==before);
  Eigen::Matrix<double,2,3> archived;archived<<2,.5,-1,-.3,1,.2;Eigen::Vector2d archived_r(.2,-.4);
  store.appendLate({"new-observation-of-archived-coordinates"},{0,4,9},archived,archived_r);
  Eigen::MatrixXd scatter=Eigen::MatrixXd::Zero(2,n);scatter.col(0)=archived.col(0);scatter.col(4)=archived.col(1);scatter.col(9)=archived.col(2);
  const Eigen::MatrixXd late_h=updated_h+scatter.transpose()*scatter;
  const auto late_result=store.query(store.solve(store.snapshot()),coordinates(0,n),Eigen::MatrixXd::Identity(n,n));
  close(late_result.mean,-late_h.ldlt().solve(j.transpose()*r+late.transpose()*residual+scatter.transpose()*archived_r));
  close(late_result.covariance,late_h.ldlt().solve(Eigen::MatrixXd::Identity(n,n)));
  const auto complete=store.snapshot();
  rejects([&]{store.appendLate({"new-observation-of-archived-coordinates"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1));},"submap_measurement_reused");
  assert(store.snapshot()==complete);
  Store other;rejects([&]{other.solve(before);},"submap_snapshot_owner");
  rejects([&]{other.query(old_solution,{8},Eigen::MatrixXd::Identity(1,1));},"submap_snapshot_owner");
}
void unobservedReference() {
  Store store;Eigen::MatrixXd j=Eigen::MatrixXd::Zero(6,12);j.leftCols(6)=-Eigen::MatrixXd::Identity(6,6);j.rightCols(6).setIdentity();
  const Eigen::VectorXd move=Eigen::VectorXd::LinSpaced(6,.1,.6);
  store.append({"relative-pose"},coordinates(0,12),j,-move);store.seal(coordinates(6,6));
  const auto solution=store.solve(store.snapshot());
  assert(!store.query(solution,coordinates(0,6),Eigen::MatrixXd::Identity(6,6)).observed);
  assert(!store.query(solution,coordinates(6,6),Eigen::MatrixXd::Identity(6,6)).observed);
  const auto relative=store.query(solution,coordinates(0,12),j);
  assert(relative.observed);close(relative.mean,move);close(relative.covariance,Eigen::MatrixXd::Identity(6,6));
}
void exactZeroQuerySupport() {
  Store store;store.append({"two-measurements"},{0,1},Eigen::Matrix2d::Identity(),Eigen::Vector2d(-1,-2));
  store.seal({1});const auto solution=store.solve(store.snapshot());
  const auto zero=store.query(solution,{0,1},Eigen::MatrixXd::Zero(1,2));
  assert(zero.observed && zero.mean[0]==0 && zero.covariance(0,0)==0);
  // A squared-norm zero check would incorrectly discard this coefficient.
  const auto tiny=store.query(solution,{0},Eigen::MatrixXd::Constant(1,1,1e-200));
  assert(tiny.mean[0]>0 && std::abs(tiny.mean[0]/1e-200-1)<1e-12);
  rejects([&]{store.query(solution,{2},Eigen::MatrixXd::Zero(1,1));},"unknown_submap_query_coordinate");
}
void exactConstraintsAndCapacity() {
  Store store;Eigen::Matrix3d j;j<<2,1,.2,.3,3,.4,.1,.2,4;Eigen::Vector3d residual(.1,.2,.3);
  Eigen::Matrix<double,2,3> c;c<<1,0,0,0,1,-1;Eigen::Vector2d target(2,3);
  store.append({"rows-and-exact-support"},{0,1,2},j,residual,c,target);store.seal({2});
  const auto result=store.query(store.solve(store.snapshot()),{0,1,2},Eigen::Matrix3d::Identity());
  const Eigen::Vector3d point(2,3,0),direction(0,1,1);const Eigen::Vector3d derivative=j*direction;
  const double variance=1/derivative.squaredNorm(),mean=-derivative.dot(j*point+residual)*variance;
  assert(result.observed);close(result.mean,point+direction*mean);close(result.covariance,variance*direction*direction.transpose());
  const auto before=store.snapshot();
  rejects([&]{store.append({"inconsistent-support"},{2},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd(),Eigen::MatrixXd::Ones(2,1),Eigen::Vector2d(0,1));},"inconsistent_affine_constraints");
  assert(store.snapshot()==before);
  rejects([&]{store.appendLate({"inconsistent-archived-support"},{0},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd(),Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Constant(1,3));},"inconsistent_affine_constraints");
  assert(store.snapshot()==before);
  Store::Limits limits;limits.numeric_bytes=1;Store tiny(limits);const auto empty=tiny.snapshot();
  rejects([&]{tiny.append({"over-budget"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1));},"submap_numeric_capacity");assert(tiny.snapshot()==empty);
  limits=Store::Limits{};limits.active_columns=2;Store bounded(limits);const auto initial=bounded.snapshot();
  rejects([&]{bounded.append({"too-many-coordinates"},{0,1,2},Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());},"submap_coordinate_capacity");assert(bounded.snapshot()==initial);
  limits=Store::Limits{};limits.nodes=1;Store one_node(limits);
  one_node.append({"source"},{0,1,2},Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());one_node.seal({1,2});const auto first=one_node.snapshot();
  rejects([&]{one_node.seal({2});},"submap_node_capacity");assert(one_node.snapshot()==first);
  Store extreme;extreme.append({"overflowing-covariance"},{0},Eigen::MatrixXd::Constant(1,1,1e-160),Eigen::VectorXd::Zero(1));
  const auto solution=extreme.solve(extreme.snapshot());
  rejects([&]{extreme.query(solution,{0},Eigen::MatrixXd::Identity(1,1));},"submap_nonfinite_query");
}
void pressureDatumAndClosedFrontier() {
  Store store;Eigen::Matrix<double,2,3> j;j<<1,0,-1,0,1,-1;Eigen::Vector2d residual(2,3);
  store.append({"two-pressure-readings"},{0,1,2},j,residual);store.seal({1,2});
  const auto unreferenced=store.solve(store.snapshot());
  assert(!store.query(unreferenced,{0},Eigen::MatrixXd::Identity(1,1)).observed);
  Eigen::Matrix<double,1,2> difference;difference<<-1,1;
  const auto relative=store.query(unreferenced,{0,1},difference);
  assert(relative.observed);close(relative.mean,Eigen::VectorXd::Constant(1,-1));close(relative.covariance,Eigen::MatrixXd::Constant(1,1,2));
  store.appendLate({"single-original-height-reference"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Constant(1,-10));
  Eigen::Matrix3d all;all.topRows(2)=j;all.bottomRows(1)<<1,0,0;Eigen::Vector3d r(2,3,-10);
  const auto updated=store.query(store.solve(store.snapshot()),{0,1,2},Eigen::Matrix3d::Identity());
  assert(updated.observed);close(updated.mean,-(all.transpose()*all).ldlt().solve(all.transpose()*r));
  close(updated.covariance,(all.transpose()*all).ldlt().solve(Eigen::Matrix3d::Identity()));
  Store closed;closed.append({"full-source"},{0,1},Eigen::Matrix2d::Identity(),Eigen::Vector2d::Zero());closed.seal({});
  closed.appendLate({"past-measurement"},{0},Eigen::MatrixXd::Constant(1,1,2),Eigen::VectorXd::Constant(1,-3));
  const auto past=closed.query(closed.solve(closed.snapshot()),{0,1},Eigen::Matrix2d::Identity());
  close(past.mean,Eigen::Vector2d(1.2,0));close(past.covariance,Eigen::Vector2d(.2,1).asDiagonal().toDenseMatrix());
}
void longChain() {
  constexpr int count=2000;Store::Limits limits;limits.active_columns=12;Store store(limits);
  const Eigen::Matrix<double,6,1> q=(Eigen::Matrix<double,6,1>()<<.0004,.0009,.0016,.0001,.0002,.0003).finished();
  const Eigen::Matrix<double,6,1> initial=q*.2,step=(Eigen::Matrix<double,6,1>()<<.001,-.002,.003,.0001,0,-.0001).finished();
  store.append({"single-initial-reference"},coordinates(0,6),initial.cwiseSqrt().cwiseInverse().asDiagonal(),Eigen::VectorXd::Zero(6));
  size_t maximum_active=6;
  for(int i=1;i<=count;++i) {
    Eigen::MatrixXd j=Eigen::MatrixXd::Zero(6,12);j.leftCols(6)=(-q.cwiseSqrt().cwiseInverse()).asDiagonal();j.rightCols(6)=q.cwiseSqrt().cwiseInverse().asDiagonal();
    store.append({"relative:"+std::to_string(i)},coordinates(6*(i-1),12),j,-q.cwiseSqrt().cwiseInverse().cwiseProduct(step));
    maximum_active=std::max(maximum_active,store.snapshot()->order.size());store.seal(coordinates(6*i,6));
  }
  const auto old_solution=store.solve(store.snapshot());
  const auto prior=store.query(old_solution,coordinates(6000,6),Eigen::MatrixXd::Identity(6,6));
  close(prior.mean,1000*step);close(prior.covariance,(initial+1000*q).asDiagonal().toDenseMatrix());
  const Eigen::Matrix<double,6,1> observation_variance=2*q,innovation=Eigen::Matrix<double,6,1>::Constant(.5);
  store.append({"new-endpoint-measurement"},coordinates(6*count,6),observation_variance.cwiseSqrt().cwiseInverse().asDiagonal(),
    -observation_variance.cwiseSqrt().cwiseInverse().cwiseProduct(count*step+innovation));
  const auto solution=store.solve(store.snapshot());double maximum_error=0;
  for(int first:{0,1,100,1000,1999}) {
    const int second=count;auto ids=coordinates(6*first,6);const auto tail=coordinates(6*second,6);ids.insert(ids.end(),tail.begin(),tail.end());
    const auto actual=store.query(solution,ids,Eigen::MatrixXd::Identity(12,12));
    Eigen::MatrixXd expected=Eigen::MatrixXd::Zero(12,12);Eigen::VectorXd mean(12);
    for(int axis=0;axis<6;++axis) {
      const double total=initial[axis]+count*q[axis]+observation_variance[axis];
      for(int a=0;a<2;++a) {
        const int i=a?second:first;mean[6*a+axis]=i*step[axis]+(initial[axis]+i*q[axis])/total*innovation[axis];
        for(int b=0;b<2;++b) {
          const int k=b?second:first;
          expected(6*a+axis,6*b+axis)=initial[axis]+std::min(i,k)*q[axis]-(initial[axis]+i*q[axis])*(initial[axis]+k*q[axis])/total;
        }
      }
    }
    assert(actual.observed);close(actual.mean,mean,1e-8);close(actual.covariance,expected,1e-8);
    maximum_error=std::max(maximum_error,(actual.covariance-expected).norm());
  }
  assert((store.query(solution,coordinates(6000,6),Eigen::MatrixXd::Identity(6,6)).mean-prior.mean).norm()>.1);
  assert(store.snapshot()->nodes.size()==count && maximum_active==12 && store.snapshot()->measurements.size()==count+2);
  const auto before=store.snapshot();Eigen::MatrixXd wide=Eigen::MatrixXd::Zero(6,12);wide.leftCols(6).setIdentity();wide.rightCols(6).setIdentity();
  auto far=coordinates(0,6);const auto endpoint=coordinates(6*count,6);far.insert(far.end(),endpoint.begin(),endpoint.end());
  rejects([&]{store.appendLate({"wide-late-factor"},far,wide,Eigen::VectorXd::Zero(6));},"submap_coordinate_capacity");assert(store.snapshot()==before);
  const Eigen::Matrix<double,6,1> old_variance=3*q,old_observation=Eigen::Matrix<double,6,1>::Constant(-.2);
  store.appendLate({"new-past-pose-observation"},coordinates(0,6),old_variance.cwiseSqrt().cwiseInverse().asDiagonal(),
    -old_variance.cwiseSqrt().cwiseInverse().cwiseProduct(old_observation));
  const auto reconditioned=store.solve(store.snapshot());
  for(int time:{0,1,100,1000,1999,count}) {
    Eigen::VectorXd expected_mean(6),expected_variance(6);
    for(int axis=0;axis<6;++axis) {
      const double total=initial[axis]+count*q[axis]+observation_variance[axis];
      const double c00=initial[axis]-initial[axis]*initial[axis]/total;
      const double ci0=initial[axis]-(initial[axis]+time*q[axis])*initial[axis]/total;
      const double cii=initial[axis]+time*q[axis]-std::pow(initial[axis]+time*q[axis],2)/total;
      const double old_mean=initial[axis]/total*innovation[axis],mean=time*step[axis]+(initial[axis]+time*q[axis])/total*innovation[axis];
      expected_mean[axis]=mean+ci0/(c00+old_variance[axis])*(old_observation[axis]-old_mean);
      expected_variance[axis]=cii-ci0*ci0/(c00+old_variance[axis]);
    }
    const auto actual=store.query(reconditioned,coordinates(6*time,6),Eigen::MatrixXd::Identity(6,6));
    assert(actual.observed);close(actual.mean,expected_mean,1e-8);close(actual.covariance,expected_variance.asDiagonal().toDenseMatrix(),1e-8);
  }
  assert(store.snapshot()->measurements.size()==count+3);
  std::cout<<"SUBMAP_CHAIN poses="<<count+1<<" maximum_active_columns="<<maximum_active<<" numeric_bytes="<<store.snapshot()->numericBytes()<<" maximum_covariance_error="<<maximum_error<<'\n';
}
using Active=posim_slam::SubmapLinearization;
Active::Rows likelihood(std::vector<std::string> keys,std::vector<int64_t> ids,
    const Eigen::MatrixXd& j,const Eigen::VectorXd& r) {
  Active::Rows out;out.originals=std::move(keys);out.columns=std::move(ids);
  out.jacobian=j;out.residual=r;return out;
}
void replaceOriginalLikelihoods() {
  Store base;Eigen::Matrix2d j;j<<1,0,-1,1;Eigen::Vector2d r(-.2,-1.5);
  base.append({"world","motion"},{0,1},j,r);base.seal({1});
  const auto original=base.snapshot();Active active(base,original,Store::Limits{});
  auto first=likelihood({"pixel-a","pixel-b"},{1},Eigen::Vector2d(2,3),Eigen::Vector2d(-4,-6));
  active.append({first});const auto before=active.snapshot();const auto before_solution=active.solve(before);
  const auto old=active.query(before_solution,{0,1},Eigen::Matrix2d::Identity());
  auto replacement=likelihood(first.originals,{1},Eigen::Vector2d(.5,1),Eigen::Vector2d(-.3,-.7));
  active.replace(first.originals,{replacement});
  auto check=[&](double late_weight=0.,double late_value=0.) {
    Eigen::Matrix<double,5,2> all;all.topRows(2)=j;all.block<2,2>(2,0).setZero();
    all.block<2,1>(2,1)=replacement.jacobian;all.row(4)<<late_weight,0;
    Eigen::Matrix<double,5,1> residual;residual<<r,replacement.residual,late_value;
    const Eigen::Matrix2d normal=all.transpose()*all;
    const auto actual=active.query(active.solve(active.snapshot()),{0,1},Eigen::Matrix2d::Identity());
    assert(actual.observed);close(actual.mean,-normal.ldlt().solve(all.transpose()*residual));
    close(actual.covariance,normal.ldlt().solve(Eigen::Matrix2d::Identity()));return actual;
  };
  const auto weaker=check();assert(weaker.covariance(1,1)>old.covariance(1,1));
  close(active.query(before_solution,{0,1},Eigen::Matrix2d::Identity()).covariance,old.covariance);
  assert(base.snapshot()==original && active.snapshot()->information->measurements.size()==4);
  const auto stable=active.snapshot();
  rejects([&]{active.replace({"pixel-a"},{replacement});},"submap_replacement_partial_group");
  auto missing=replacement;missing.originals={"pixel-a"};
  rejects([&]{active.replace(first.originals,{missing});},"submap_replacement_original_missing");
  rejects([&]{active.replace({"world"},{replacement});},"submap_replacement_requires_active_originals");
  rejects([&]{active.append({replacement});},"submap_measurement_reused");
  auto invalid=replacement;invalid.jacobian(0,0)=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{active.replace(first.originals,{invalid});},"submap_information_rows");
  assert(active.snapshot()==stable);
  // Split/recombine source groups atomically without reusing or losing a pixel.
  active.replace(first.originals,{
    likelihood({"pixel-a"},{1},replacement.jacobian.topRows(1),replacement.residual.head(1)),
    likelihood({"pixel-b"},{1},replacement.jacobian.bottomRows(1),replacement.residual.tail(1))});check();
  active.appendLate(likelihood({"new-past-reading"},{0},Eigen::MatrixXd::Constant(1,1,4),Eigen::VectorXd::Constant(1,-2)));
  check(4,-2);
  auto revised=replacement;revised.residual*=2;replacement=revised;
  active.replace(first.originals,{revised});check(4,-2);
  active.seal({});check(4,-2);
  const auto sealed=active.snapshot();
  rejects([&]{active.replace(first.originals,{replacement});},"submap_replacement_requires_active_originals");
  assert(active.snapshot()==sealed && sealed->active.empty());
  Active other;rejects([&]{other.solve(before);},"submap_linearization_owner");
  rejects([&]{other.query(before_solution,{1},Eigen::MatrixXd::Identity(1,1));},"submap_snapshot_owner");
  auto fork=base.fork(original);fork->append({"fork-only"},{1},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1));
  assert(base.snapshot()==original && !original->measurements.count("fork-only"));
}
void replaceRankAndExactSupport() {
  Store base;Eigen::Matrix<double,1,2> relative;relative<<-1,1;
  base.append({"relative"},{0,1},relative,Eigen::VectorXd::Constant(1,-2));base.seal({1});
  Active active(base,base.snapshot(),Store::Limits{});
  auto exact=likelihood({"support"},{1},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd());
  exact.constraints=Eigen::MatrixXd::Ones(1,1);exact.target=Eigen::VectorXd::Constant(1,3);
  active.append({exact});const auto observed=active.query(active.solve(active.snapshot()),{0,1},Eigen::Matrix2d::Identity());
  assert(observed.observed);close(observed.mean,Eigen::Vector2d(1,3));
  Eigen::Matrix2d expected=Eigen::Matrix2d::Zero();expected(0,0)=1;close(observed.covariance,expected);
  auto free=likelihood({"support"},{1},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd());
  active.replace({"support"},{free});
  const auto no_reference=active.solve(active.snapshot());
  assert(!active.query(no_reference,{1},Eigen::MatrixXd::Identity(1,1)).observed);
  const auto difference=active.query(no_reference,{0,1},relative);
  assert(difference.observed);close(difference.mean,Eigen::VectorXd::Constant(1,2));
  close(difference.covariance,Eigen::MatrixXd::Identity(1,1));
  exact.target[0]=5;active.replace({"support"},{exact});
  close(active.query(active.solve(active.snapshot()),{0,1},Eigen::Matrix2d::Identity()).mean,Eigen::Vector2d(3,5));
  // Caller mutation cannot alter the retained source likelihood.
  exact.target[0]=100;
  close(active.query(active.solve(active.snapshot()),{0,1},Eigen::Matrix2d::Identity()).mean,Eigen::Vector2d(3,5));
  Store::Limits tiny;tiny.numeric_bytes=64;Active bounded(tiny);
  const auto empty=bounded.snapshot();
  rejects([&]{bounded.append({likelihood({"too-large"},{0,1},Eigen::Matrix2d::Identity(),Eigen::Vector2d::Zero())});},"submap_linearization_capacity");
  assert(bounded.snapshot()==empty);
  Active coordinates_kept;coordinates_kept.append({likelihood({"original"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1))});
  const auto stable=coordinates_kept.snapshot();
  rejects([&]{coordinates_kept.replace({"original"},{likelihood({"original"},{1},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1))});},"submap_replacement_coordinate_missing");
  assert(coordinates_kept.snapshot()==stable);
}
void boundedReplacements() {
  Store::Limits limits;limits.active_columns=2;Active active(limits);
  active.append({likelihood({"world"},{0},Eigen::MatrixXd::Constant(1,1,2),Eigen::VectorXd::Zero(1))});
  std::vector<double> means{0},variances{.25};size_t maximum_active=1,maximum_groups=1;
  constexpr int count=1000;
  for(int i=1;i<=count;++i) {
    const double increment=.01+.001*std::sin(i),variance=.02+.001*std::cos(i);
    Eigen::Matrix<double,1,2> derivative;derivative<<-1,1;
    const std::string key="motion:"+std::to_string(i);
    auto rows=likelihood({key},{i-1,i},derivative/std::sqrt(variance*2),Eigen::VectorXd::Constant(1,-increment/std::sqrt(variance*2)));
    active.append({rows});
    rows.jacobian=derivative/std::sqrt(variance);rows.residual=Eigen::VectorXd::Constant(1,-increment/std::sqrt(variance));
    active.replace({key},{rows});active.replace({key},{rows});
    maximum_active=std::max(maximum_active,active.snapshot()->information->order.size());
    maximum_groups=std::max(maximum_groups,active.snapshot()->active.size());
    active.seal({i});means.push_back(means.back()+increment);variances.push_back(variances.back()+variance);
  }
  double maximum_error=0;
  const auto solution=active.solve(active.snapshot());
  for(int first:{0,1,100,500,999}) {
    const auto result=active.query(solution,{first,count},Eigen::Matrix2d::Identity());
    Eigen::Matrix2d covariance;covariance<<variances[first],variances[first],variances[first],variances[count];
    assert(result.observed);close(result.mean,Eigen::Vector2d(means[first],means[count]),1e-8);
    close(result.covariance,covariance,1e-8);maximum_error=std::max(maximum_error,(result.covariance-covariance).norm());
  }
  assert(maximum_active==2 && maximum_groups==2);
  assert(active.snapshot()->information->measurements.size()==count+1);
  std::cout<<"SUBMAP_REPLACEMENTS poses="<<count+1<<" maximum_active_columns="<<maximum_active
    <<" numeric_bytes="<<active.snapshot()->numericBytes()<<" maximum_covariance_error="<<maximum_error<<'\n';
}
void selectiveRetirement() {
  // Keep the newest original replaceable while the older edge becomes a
  // conditional. Query the original gauge and cross-time covariance after
  // repeated movement; retiring a group must not add a second observation.
  constexpr int count=300;Eigen::Matrix<double,1,2> relative;relative<<-1,1;
  for(bool referenced:{true,false}) {
    Store::Limits limits;limits.active_columns=3;Active active(limits);
    std::vector<double> means{0},variances{referenced?.04:0.};
    if(referenced)active.append({likelihood({"world"},{0},Eigen::MatrixXd::Constant(1,1,5),Eigen::VectorXd::Zero(1))});
    size_t maximum_active=0;double maximum_covariance_error=0;
    for(int i=1;i<=count;++i) {
      const std::string key="edge:"+std::to_string(i);
      const double increment=.2+.01*std::sin(i),variance=.01+.001*std::cos(i);
      auto rows=likelihood({key},{i-1,i},relative/std::sqrt(2*variance),
        Eigen::VectorXd::Constant(1,-2*increment/std::sqrt(2*variance)));
      active.append({rows});maximum_active=std::max(maximum_active,active.snapshot()->information->order.size());
      rows.jacobian=relative/std::sqrt(variance);rows.residual=Eigen::VectorXd::Constant(1,-increment/std::sqrt(variance));
      active.replace({key},{rows});
      means.push_back(means.back()+increment);variances.push_back(variances.back()+variance);
      const auto immutable=active.snapshot();const auto old_solution=active.solve(immutable);
      std::vector<std::string> retired;
      if(i==1 && referenced)retired={"world"};
      if(i>1)retired={"edge:"+std::to_string(i-1)};
      const auto old=active.query(old_solution,{0,i},relative);
      active.retire(retired,{i-1,i});
      assert(active.snapshot()->active.size()==1);
      assert(active.snapshot()->information->measurements.size()==size_t(i+referenced));
      assert(active.snapshot()->information->known.size()==size_t(i+1));
      // Replacing the remaining group still works after retirement. Repeating
      // the same replacement changes neither the likelihood nor its identity.
      active.replace({key},{rows});
      const auto solved=active.solve(active.snapshot());
      const auto difference=active.query(solved,{0,i},relative);
      assert(difference.observed);close(difference.mean,Eigen::VectorXd::Constant(1,means.back()),1e-8);
      const double expected=variances.back()-variances.front();
      close(difference.covariance,Eigen::MatrixXd::Constant(1,1,expected),1e-8);
      maximum_covariance_error=std::max(maximum_covariance_error,std::abs(difference.covariance(0,0)-expected));
      close(difference.mean,old.mean);close(difference.covariance,old.covariance);
      close(active.query(old_solution,{0,i},relative).covariance,old.covariance);
      const auto joint=active.query(solved,{0,i},Eigen::Matrix2d::Identity());
      assert(joint.observed==referenced);
      if(referenced) {
        Eigen::Matrix2d expected_cov;expected_cov<<.04,.04,.04,variances.back();
        close(joint.mean,Eigen::Vector2d(0,means.back()),1e-8);close(joint.covariance,expected_cov,1e-8);
      }
    }
    assert(maximum_active==3 && active.snapshot()->information->order.size()==2);
    std::cout<<"SUBMAP_SELECTIVE_RETIREMENT referenced="<<referenced<<" poses="<<count+1
      <<" maximum_active_columns="<<maximum_active<<" maximum_covariance_error="<<maximum_covariance_error<<'\n';
  }
  Active active;
  const auto pair=likelihood({"a","b"},{0,1},Eigen::Matrix2d::Identity(),Eigen::Vector2d(-1,-2));
  const auto edge=likelihood({"c"},{1,2},relative,Eigen::VectorXd::Constant(1,-1));
  active.append({pair,edge});const auto before=active.snapshot();
  rejects([&]{active.retire({"a"},{1,2});},"submap_replacement_partial_group");
  rejects([&]{active.retire({"missing"},{0,1,2});},"submap_replacement_requires_active_originals");
  rejects([&]{active.retire({"a","b"},{2});},"submap_retire_active_coordinate");
  rejects([&]{active.retire({"a","b"},{1,1,2});},"submap_retire_coordinates");
  rejects([&]{active.retire({"a","b"},{1,2,3});},"submap_retire_coordinates");
  assert(active.snapshot()==before);
  active.retire({"a","b"},{1,2});const auto retired=active.snapshot();
  rejects([&]{active.retire({"a","b"},{1,2});},"submap_replacement_requires_active_originals");
  rejects([&]{active.retire({"c"},{0,2});},"submap_retire_coordinates");
  assert(active.snapshot()==retired);
  // An independent exact support has no finite noise. Its deterministic
  // conditional must still constrain the remaining active original.
  Active exact;auto support=likelihood({"support"},{0},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd());
  support.constraints=Eigen::MatrixXd::Ones(1,1);support.target=Eigen::VectorXd::Constant(1,2);
  exact.append({support,likelihood({"step"},{0,1},relative,Eigen::VectorXd::Constant(1,-3))});
  exact.retire({"support"},{0,1});exact.retire({"step"},{1});
  const auto result=exact.query(exact.solve(exact.snapshot()),{0,1},Eigen::Matrix2d::Identity());
  assert(result.observed);close(result.mean,Eigen::Vector2d(2,5));
  Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();covariance(1,1)=1;close(result.covariance,covariance);
  // A capacity failure after constructing the candidate cannot publish it.
  Store::Limits limited;limited.nodes=1;Active bounded(limited);
  bounded.append({likelihood({"origin"},{0},Eigen::MatrixXd::Identity(1,1),Eigen::VectorXd::Zero(1)),
    likelihood({"one"},{0,1},relative,Eigen::VectorXd::Constant(1,-1)),edge});
  bounded.retire({"origin","one"},{1,2});const auto stable=bounded.snapshot();
  rejects([&]{bounded.retire({"c"},{2});},"submap_node_capacity");
  assert(bounded.snapshot()==stable);
}
void correlatedReobservation() {
  Store::Limits limits;limits.active_columns=2;limits.temporary_columns=4;
  Store base(limits);Eigen::Matrix2d pose;pose<<5,0,-1/.3,1/.3;
  base.append({"world","motion"},{0,1},pose,Eigen::Vector2d(0,-1/.3));
  // The marginalized landmark row has no marginal pose information. Its
  // complementary conditional is y = x0 + 2 + N(0,.16), from that same row.
  base.append({"landmark"},{0},Eigen::MatrixXd::Zero(0,1),Eigen::VectorXd());base.seal({1});
  const auto before=base.snapshot();Store::Node point;point.kept={0};point.removed={2};
  point.regression=Eigen::MatrixXd::Ones(1,1);point.offset=Eigen::VectorXd::Constant(1,2);
  point.noise=Eigen::MatrixXd::Constant(1,1,.4);point.unbounded.resize(1,0);point.deterministic.resize(1,0);
  base.includeConditional({"landmark"},point);const auto complete=base.snapshot();
  assert(complete->measurements==before->measurements && complete->order==before->order);
  assert(complete->conditional_sources==std::set<std::string>{"landmark"});
  Active active(base,complete,limits);
  Eigen::Matrix<double,1,2> direction;direction<<1,-1;
  auto view=likelihood({"view"},{2,1},direction/.15,Eigen::VectorXd::Constant(1,-1.2/.15));
  Eigen::MatrixXd originals=Eigen::MatrixXd::Zero(3,3);originals.topLeftCorner<2,2>()=pose;originals.row(2)<<-1/.4,0,1/.4;
  Eigen::Vector3d residual(0,-1/.3,-2/.4);
  auto check=[&](const Eigen::MatrixXd& rows,const Eigen::VectorXd& values) {
    const auto h=(rows.transpose()*rows).eval();const auto result=active.query(active.solve(active.snapshot()),coordinates(0,rows.cols()),Eigen::MatrixXd::Identity(rows.cols(),rows.cols()));
    assert(result.observed);close(result.mean,-h.ldlt().solve(rows.transpose()*values),1e-9);
    close(result.covariance,h.ldlt().solve(Eigen::MatrixXd::Identity(rows.cols(),rows.cols())),1e-9);return result;
  };
  check(originals,residual);active.append({view});
  Eigen::MatrixXd all=Eigen::MatrixXd::Zero(4,3);all.topRows(3)=originals;all(3,2)=view.jacobian(0,0);all(3,1)=view.jacobian(0,1);
  Eigen::VectorXd values(4);values<<residual,view.residual;const auto old=check(all,values);const auto immutable=active.snapshot();const auto old_solution=active.solve(immutable);
  // Account for both the untouched base and every reconditioned node.
  std::set<const Store::Node*> nodes;size_t numeric=immutable->active_bytes;
  for(const auto& snapshot:{immutable->base,immutable->information}) {
    numeric+=snapshot->numericBytes()-snapshot->history_bytes;
    for(const auto& node:snapshot->nodes)if(nodes.insert(node.get()).second)numeric+=node->bytes();
  }
  assert(numeric==immutable->numericBytes() && nodes.size()>immutable->base->nodes.size());
  auto small=limits;small.numeric_bytes=numeric-1;Active bounded(base,complete,small);const auto stable=bounded.snapshot();
  rejects([&]{bounded.append({view});},"submap_linearization_capacity");assert(bounded.snapshot()==stable);
  view.jacobian=direction/.5;view.residual=Eigen::VectorXd::Constant(1,-1.4/.5);active.replace({"view"},{view});
  all(3,2)=view.jacobian(0,0);all(3,1)=view.jacobian(0,1);values[3]=view.residual[0];check(all,values);
  close(active.query(old_solution,{0,1,2},Eigen::Matrix3d::Identity()).covariance,old.covariance);
  auto motion=likelihood({"new-motion"},{1,3},-direction/.2,Eigen::VectorXd::Constant(1,-1/.2));
  auto latest=likelihood({"new-view"},{2,3},direction/.3,Eigen::VectorXd::Constant(1,-.1/.3));
  active.append({motion,latest});Eigen::MatrixXd expanded=Eigen::MatrixXd::Zero(6,4);expanded.topLeftCorner(4,3)=all;
  expanded(4,1)=motion.jacobian(0,0);expanded(4,3)=motion.jacobian(0,1);
  expanded(5,2)=latest.jacobian(0,0);expanded(5,3)=latest.jacobian(0,1);
  Eigen::VectorXd combined(6);combined<<values,motion.residual,latest.residual;check(expanded,combined);
  active.retire({"view","new-motion"},{3});check(expanded,combined);
  latest.residual[0]*=2;combined[5]=latest.residual[0];active.replace({"new-view"},{latest});check(expanded,combined);
  assert(active.snapshot()->information->measurements.size()==6 && active.snapshot()->information->known.size()==4);
  assert(active.snapshot()->information->order==std::vector<int64_t>{3});
  const auto unchanged=base.snapshot();auto bad=point;bad.removed={7};
  rejects([&]{base.includeConditional({"landmark"},bad);},"submap_conditional_originals");
  rejects([&]{base.includeConditional({"unknown"},bad);},"submap_conditional_originals");
  rejects([&]{base.includeConditional({"world"},point);},"submap_conditional_coordinate_reused");
  bad.kept={9};rejects([&]{base.includeConditional({"world"},bad);},"submap_conditional_parent");
  bad=point;bad.removed={7};bad.noise.setZero();rejects([&]{base.includeConditional({"world"},bad);},"submap_conditional_basis");
  assert(base.snapshot()==unchanged);
  // An unbounded conditional does not invent a point prior. A later original
  // relative reading can make it observable through the existing world frame.
  Store free(limits);free.append({"pose","unseen"},{0},Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Zero(1));
  point.kept={0};point.removed={1};point.noise.resize(1,0);point.unbounded=Eigen::MatrixXd::Ones(1,1);
  free.includeConditional({"unseen"},point);Active unbounded(free,free.snapshot(),limits);
  assert(!unbounded.query(unbounded.solve(unbounded.snapshot()),{1},Eigen::MatrixXd::Identity(1,1)).observed);
  unbounded.append({likelihood({"observed"},{1,0},direction/.2,Eigen::VectorXd::Constant(1,-3/.2))});
  const auto known=unbounded.query(unbounded.solve(unbounded.snapshot()),{0,1},Eigen::Matrix2d::Identity());
  assert(known.observed);close(known.mean,Eigen::Vector2d(0,3));Eigen::Matrix2d covariance;covariance<<1,1,1,1.04;close(known.covariance,covariance);
  Store exact(limits);exact.append({"pose","exact"},{0},Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Zero(1));
  Active constrained(exact,exact.snapshot(),limits);point.regression=Eigen::MatrixXd::Constant(1,1,2);point.offset=Eigen::VectorXd::Ones(1);
  point.unbounded.resize(1,0);point.deterministic=Eigen::MatrixXd::Ones(1,1);constrained.includeConditional({"exact"},point);
  constrained.append({likelihood({"reading"},{1},Eigen::MatrixXd::Constant(1,1,2),Eigen::VectorXd::Constant(1,-6))});
  const auto deterministic=constrained.query(constrained.solve(constrained.snapshot()),{0,1},Eigen::Matrix2d::Identity());
  assert(deterministic.observed);close(deterministic.mean,Eigen::Vector2d(16./17,49./17));covariance<<1,2,2,4;close(deterministic.covariance,covariance/17);
  // The lineage context must not retain the initial base after replacement.
  // Only caller-held snapshots may keep that older numeric payload alive.
  std::weak_ptr<const Store::Data> released;std::weak_ptr<const Store::Node> released_node;
  auto make=[&] {
    Store initial(limits);initial.append({"world","motion"},{0,1},pose,Eigen::Vector2d(0,-1/.3));initial.seal({1});
    released=initial.snapshot();released_node=initial.snapshot()->nodes.front();return std::make_unique<Active>(initial,initial.snapshot(),limits);
  };
  auto owner=make();assert(!released.expired() && !released_node.expired());
  owner->appendLate(likelihood({"late"},{0},Eigen::MatrixXd::Ones(1,1),Eigen::VectorXd::Zero(1)));
  assert(released.expired() && released_node.expired());
  std::cout<<"Correlated reobservation: complementary source conditional, new/replaced pixels, new pose, retirement, shared covariance, free directions, immutable ownership and exact numeric accounting pass.\n";
}
void batchedOriginals() {
  // Cross both the 4096 likelihood-row and 1024 exact-row batch boundaries.
  for(bool exact:{false,true}) {
    Active active;std::vector<Active::Rows> groups;std::vector<std::string> keys;
    constexpr int count=700;Eigen::Matrix<double,8,2> j;
    j<<1,0,0,1,1,1,2,-1,0,2,3,1,1,-2,2,2;
    const Eigen::Vector2d expected(2,-1);
    for(int i=0;i<count;++i) {
      auto group=likelihood({"reading:"+std::to_string(i)},{8,3},j,-j*expected);
      if(exact){group.constraints=Eigen::MatrixXd::Zero(3,2);group.constraints.col(0).setOnes();group.target=Eigen::VectorXd::Constant(3,2);}
      keys.push_back(group.originals[0]);groups.push_back(std::move(group));
    }
    active.append(groups);const auto snapshot=active.snapshot();
    const auto result=active.query(active.solve(snapshot),{8,3},Eigen::Matrix2d::Identity());
    assert(result.observed && snapshot->information->measurements.size()==count);close(result.mean,expected);
    Eigen::Matrix2d covariance=Eigen::Matrix2d::Zero();
    if(exact)covariance(1,1)=1./(count*j.col(1).squaredNorm());
    else covariance=(count*j.transpose()*j).inverse();
    close(result.covariance,covariance,1e-10);
    for(auto& group:groups){group.jacobian*=.5;group.residual*=.5;}
    active.replace(keys,groups);
    const auto weaker=active.query(active.solve(active.snapshot()),{8,3},Eigen::Matrix2d::Identity());
    close(weaker.mean,expected);close(weaker.covariance,4*covariance,1e-10);
    const auto stable=active.snapshot();auto duplicate=groups.front();duplicate.columns={8,8};
    rejects([&]{active.replace(duplicate.originals,{duplicate});},"submap_coordinates");
    assert(active.snapshot()==stable);duplicate.columns={-1,3};
    rejects([&]{active.replace(duplicate.originals,{duplicate});},"submap_coordinates");
    assert(active.snapshot()==stable);
  }
}
void batchedRelativeGauge() {
  for(int batch_size:{7,2100}) {
    Store store;const int count=6300;
    for(int start=0;start<count;start+=batch_size) {
      const int count_here=std::min(batch_size,count-start);
      Eigen::MatrixXd j=Eigen::MatrixXd::Ones(count_here,2);j.col(1).setConstant(-1);
      Eigen::VectorXd residual(count_here);
      for(int i=0;i<count_here;++i)residual[i]=-.1*((start+i)/2100);
      store.append({"source:"+std::to_string(start)},{0,1},j,residual);
    }
    assert(!store.query(store.solve(store.snapshot()),{0,1},Eigen::Matrix2d::Identity()).observed);
    store.seal({1});const auto solved=store.solve(store.snapshot());
    assert(!store.query(solved,{1},Eigen::MatrixXd::Identity(1,1)).observed);
    Eigen::Matrix<double,1,2> difference;difference<<1,-1;
    const auto relative=store.query(solved,{0,1},difference);assert(relative.observed);
    close(relative.mean,Eigen::VectorXd::Constant(1,.1),1e-10);
    close(relative.covariance,Eigen::MatrixXd::Constant(1,1,1./count),1e-10);
  }
  // A genuinely weak, independent direction stays finite. Uniform rescaling
  // changes covariance, not observability or the recovered mean.
  for(double scale:{1e-4,1.,1e4}) {
    Store store;Eigen::MatrixXd j=Eigen::MatrixXd::Zero(2100,2);Eigen::Vector2d mean(.2,-.3);
    for(int i=0;i<2100;++i)j(i,i%2)=scale*(i%2?1e-8:1.);
    store.append({"weak-independent-measurement"},{0,1},j,-j*mean);
    store.seal({1});const auto result=store.query(store.solve(store.snapshot()),{0,1},Eigen::Matrix2d::Identity());
    assert(result.observed);close(result.mean,mean,1e-8);
    Eigen::Matrix2d expected=Eigen::Matrix2d::Zero();expected(0,0)=1./(1050*scale*scale);expected(1,1)=expected(0,0)*1e16;
    close(result.covariance,expected,1e-10);
  }
}
} // namespace
int main() {
  denseAndLateAdjustment();unobservedReference();exactZeroQuerySupport();exactConstraintsAndCapacity();pressureDatumAndClosedFrontier();longChain();
  replaceOriginalLikelihoods();replaceRankAndExactSupport();boundedReplacements();selectiveRetirement();correlatedReobservation();batchedOriginals();
  batchedRelativeGauge();
  std::cout<<"Submap information: batch mean/covariance, shared noise, unobserved gauge, exact support, immutable ownership and bounded-frontier chain pass.\n";
}

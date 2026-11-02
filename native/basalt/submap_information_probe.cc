// SPDX-License-Identifier: Apache-2.0
#include "submap_information.hpp"
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
} // namespace
int main() {
  denseAndLateAdjustment();unobservedReference();exactConstraintsAndCapacity();pressureDatumAndClosedFrontier();longChain();
  std::cout<<"Submap information: batch mean/covariance, shared noise, unobserved gauge, exact support, immutable ownership and bounded-frontier chain pass.\n";
}

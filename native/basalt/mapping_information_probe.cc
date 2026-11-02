// SPDX-License-Identifier: Apache-2.0
#include "mapping_information.hpp"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <numeric>

using namespace posim_slam;
using Eigen::MatrixXd;
template<class F> void rejected(F operation) {
  bool failed=false;
  try { operation(); } catch(const std::invalid_argument&) { failed=true; }
  assert(failed);
}
MappingPointRows measurement(int64_t id, int count=3) {
  MappingPointRows rows;
  rows.id=id; rows.world=Eigen::Vector3d(0,0,2); rows.host_origin.setZero();
  rows.pose_columns={0,1,2}; rows.point=MatrixXd::Random(2*count,3)*100;
  rows.pose=-rows.point;
  for(int i=0;i<count;++i) {
    rows.observations.emplace_back(i+1,0,id);
    rows.full_weight.push_back(true);
  }
  return rows;
}
void ownership_and_gauge() {
  const auto rows=measurement(1);
  MappingInformation graph(3); graph.addPoint(rows);
  const auto unknown=graph.evaluate(.1);
  assert(unknown.pose_nullity==3 && !unknown.points.at(1).world.observed);
  rejected([&]{graph.addPoint(rows);});
  auto alias=rows; alias.id=2; rejected([&]{graph.addPoint(alias);});
  alias.observations[1]=alias.observations[0];
  MappingInformation separate(3); rejected([&]{separate.addPoint(alias);});
  // A 20 cm common translation uncertainty cannot pass a 10 cm world gate.
  graph.addPoseFactor("world-reference",{0,1,2},5*MatrixXd::Identity(3,3));
  rejected([&]{graph.addPoseFactor("world-reference",{0,1,2},MatrixXd::Identity(3,3));});
  const auto uncertain=graph.evaluate(.1);
  assert(uncertain.points.at(1).world.observed && !uncertain.points.at(1).eligible);
  const MatrixXd expected=(rows.point.transpose()*rows.point).inverse()+.04*MatrixXd::Identity(3,3);
  assert((uncertain.points.at(1).world.covariance-expected).norm()<1e-11);
  const auto again=graph.evaluate(.1);
  assert((again.points.at(1).world.covariance-expected).norm()<1e-11);
  assert(again.observations==3 && again.pose_factors==1);
}
void maturity_and_geometry() {
  for(int count:{2,3}) {
    auto rows=measurement(7,count);
    MappingInformation graph(3);
    graph.addPoseFactor("reference",{0,1,2},1000*MatrixXd::Identity(3,3));
    graph.addPoint(rows);
    const auto point=graph.evaluate(.1).points.at(7);
    assert(point.geometry.eligible && point.world.observed);
    assert(point.mature==(count>=3) && point.eligible==(count>=3));
  }
  auto degenerate=measurement(8);degenerate.point.col(2).setZero();
  degenerate.pose=-degenerate.point;
  MappingInformation graph(3);
  graph.addPoseFactor("reference",{0,1,2},1000*MatrixXd::Identity(3,3));
  graph.addPoint(degenerate);
  const auto result=graph.evaluate(.1).points.at(8);
  assert(!result.geometry.eligible && !result.world.observed && !result.eligible);
  auto outlier=measurement(9);outlier.full_weight[2]=false;
  MappingInformation robust(3);
  robust.addPoseFactor("reference",{0,1,2},1000*MatrixXd::Identity(3,3));
  robust.addPoint(outlier);
  assert(!robust.evaluate(.1).points.at(9).mature);
}
void dense_joint_and_units() {
  const int n=12, landmarks=4, dim=n+3*landmarks;
  MatrixXd full=MatrixXd::Zero(dim,dim);
  MappingInformation graph(n), rescaled(n);
  Eigen::VectorXd units=Eigen::VectorXd::LinSpaced(n,.001,100.);
  MatrixXd prior=MatrixXd::Random(n+5,n); prior.topRows(n)+=3*MatrixXd::Identity(n,n);
  std::vector<int> columns(n); std::iota(columns.begin(),columns.end(),0);
  graph.addPoseFactor("prior",columns,prior);
  rescaled.addPoseFactor("prior",columns,prior*units.asDiagonal());
  full.topLeftCorner(n,n)=prior.transpose()*prior;
  for(int k=0;k<landmarks;++k) {
    auto rows=measurement(k+20,6); rows.pose_columns={k,k+1,k+4,k+7};
    rows.pose=MatrixXd::Random(12,4); rows.point=MatrixXd::Random(12,3)*10;
    MatrixXd assembled=MatrixXd::Zero(12,dim);
    for(int i=0;i<4;++i)assembled.col(rows.pose_columns[i])=rows.pose.col(i);
    assembled.middleCols(n+3*k,3)=rows.point;
    full+=assembled.transpose()*assembled;
    graph.addPoint(rows);
    for(int i=0;i<4;++i)rows.pose.col(i)*=units[rows.pose_columns[i]];
    rescaled.addPoint(rows);
  }
  const MatrixXd oracle=full.ldlt().solve(MatrixXd::Identity(dim,dim));
  const auto original=graph.evaluate(.1),changed=rescaled.evaluate(.1);
  assert(original.pose_nullity==0 && changed.pose_nullity==0);
  for(int k=0;k<landmarks;++k) {
    const auto a=original.points.at(k+20).world,b=changed.points.at(k+20).world;
    assert(a.observed && b.observed);
    assert((a.covariance-oracle.block<3,3>(n+3*k,n+3*k)).norm()<1e-10);
    assert((a.covariance-b.covariance).norm()<1e-10);
  }
}
int main() {
  std::srand(29);
  ownership_and_gauge();maturity_and_geometry();dense_joint_and_units();
  rejected([]{MappingInformation graph(0);});
  std::cout<<"mapping observation ownership, maturity, gauge, dense covariance and units passed\n";
}

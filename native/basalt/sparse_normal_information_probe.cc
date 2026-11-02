// SPDX-License-Identifier: Apache-2.0
#include "sparse_normal_information.hpp"
#include "map_information.hpp"
#include <cassert>
#include <iostream>
#include <numeric>
using namespace posim_slam;
using Eigen::MatrixXd;
void dense_and_units() {
  for(int n:{3,12,60})for(int gauge:{0,2}) {
    MatrixXd a=MatrixXd::Random(n+8,n);a.topRows(n)+=3*MatrixXd::Identity(n,n);
    if(gauge)a.rightCols(gauge).setZero();
    Eigen::VectorXd units=Eigen::VectorXd::LinSpaced(n,.001,100.);
    for(bool rescale:{false,true}) {
      MatrixXd j=rescale?MatrixXd(a*units.asDiagonal()):a;
      Eigen::VectorXd scale=(j.colwise().squaredNorm().array()>0).select(
        j.colwise().norm().array().max(1e-200).inverse(),1).transpose();
      SparseNormalInformation selected(n);std::vector<int> columns(n);std::iota(columns.begin(),columns.end(),0);
      selected.add(columns,j);const bool ready=selected.factorize(scale);
      const auto dense=mapPoseInformation(j.transpose()*j,scale);
      assert(ready==(gauge==0));if(gauge)continue;
      MatrixXd b=MatrixXd::Random(3,n);if(gauge)b.rightCols(gauge).setZero();
      const auto result=selected.covariance(columns,b);assert(result.observed);
      const MatrixXd root=b*dense.covariance_root;
      assert((result.covariance-root*root.transpose()).norm()<1e-6*std::max(1.,result.covariance.norm()));
      if(gauge) {b.setZero();b(0,n-1)=1;assert(!selected.covariance(columns,b).observed);}
    }
  }
}
void chain(int nodes,bool anchored) {
  const int n=3*nodes;SparseNormalInformation selected(n);Eigen::VectorXd diag=Eigen::VectorXd::Zero(n);
  if(anchored){selected.add({0,1,2},100*MatrixXd::Identity(3,3));diag.head<3>().array()+=10000;}
  MatrixXd relative(3,6);relative<<-10*MatrixXd::Identity(3,3),10*MatrixXd::Identity(3,3);
  for(int i=1;i<nodes;++i) {
    selected.add({3*i-3,3*i-2,3*i-1,3*i,3*i+1,3*i+2},relative);
    diag.segment<6>(3*i-3).array()+=100;
  }
  selected.factorize(diag.array().sqrt().inverse().matrix());
  const auto last=selected.covariance({n-3,n-2,n-1},MatrixXd::Identity(3,3));
  assert(last.observed==anchored);
  if(anchored)assert((last.covariance-(.0001+.01*(nodes-1))*MatrixXd::Identity(3,3)).norm()<1e-7*nodes);
  if(!anchored)return;
  const auto difference=selected.covariance({n-6,n-5,n-4,n-3,n-2,n-1},relative/10);
  assert(difference.observed && (difference.covariance-.01*MatrixXd::Identity(3,3)).norm()<1e-8);
  assert(selected.factorNonZeros()<size_t(30*nodes));
  if(nodes==20) {
    for(int i=0;i<nodes;++i)for(int j=i+1;j<nodes;++j) {
      const auto pair=selected.covariance({3*i,3*i+1,3*i+2,3*j,3*j+1,3*j+2},relative/10);
      assert(pair.observed && (pair.covariance-.01*(j-i)*MatrixXd::Identity(3,3)).norm()<1e-8);
    }
    assert(selected.solveQueries()>0); // Missing inverse entries require actual solves.
  }
  std::cout<<"chain nodes="<<nodes<<" anchored="<<anchored<<" columns="<<n<<" rank="<<(selected.ready()?n:0)
    <<" input_nnz="<<selected.inputNonZeros()<<" factor_nnz="<<selected.factorNonZeros()<<'\n';
}
int main() {
  std::srand(43);dense_and_units();chain(20,true);chain(20,false);chain(2000,true);chain(2000,false);
  std::cout<<"sparse selected covariance, unanchored refusal, cross-correlation and units passed\n";
}

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

void structured_covariance() {
  using Matrix=Eigen::MatrixXd;
  using LongMatrix=Eigen::Matrix<long double,Eigen::Dynamic,Eigen::Dynamic>;
  for(int n:{3,12,60,120})for(int unit_mode:{-8,0,8})for(int groups:{1,2})for(int epsilon_power:{0,-24}) {
    const double epsilon=std::ldexp(1.,epsilon_power);
    Eigen::VectorXd units(n),v(n);
    for(int i=0;i<n;++i){units[i]=std::ldexp(1.,unit_mode*(i%3-1));v[i]=std::ldexp(double(i%7+1),-3);}
    const int split=groups==1?n:3*n/4;
    Matrix h=epsilon*Matrix::Identity(n,n);LongMatrix inverse=LongMatrix::Zero(n,n);
    // Exact binary source coefficients; each block has a closed-form inverse.
    for(auto bounds:std::vector<std::pair<int,int>>{{0,split},{split,n}}) {
      const auto [first,last]=bounds;if(first==last)continue;
      long double vv=0;for(int i=first;i<last;++i)vv+=static_cast<long double>(v[i])*v[i];
      for(int i=first;i<last;++i)for(int j=first;j<last;++j) {
        h(i,j)+=v[i]*v[j];
        inverse(i,j)=((i==j?1.L:0.L)-static_cast<long double>(v[i])*v[j]/(epsilon+vv))/epsilon/units[i]/units[j];
      }
    }
    h=units.asDiagonal()*h*units.asDiagonal();
    std::vector<int> columns(n);std::iota(columns.begin(),columns.end(),0);
    std::reverse(columns.begin(),columns.end());std::rotate(columns.begin(),columns.begin()+n/3,columns.end());
    Matrix reordered(n,n);for(int i=0;i<n;++i)for(int j=0;j<n;++j)reordered(i,j)=h(columns[i],columns[j]);
    SparseNormalInformation selected(n);selected.addNormal(columns,reordered);
    const Eigen::VectorXd scale=units.cwiseInverse();assert(selected.factorize(scale));
    // Include the old numeric arrays, column indices and component metadata.
    const size_t nonzeros=selected.factorNonZeros();
    const size_t old_storage=sizeof(double)*(2*nonzeros+size_t(n))+
      sizeof(SparseNormalInformation::Sparse::StorageIndex)*(nonzeros+size_t(n)+1)+sizeof(int)*size_t(n);
    assert(selected.covarianceStorageBytes()<=old_storage);
    for(int rows:{1,3,24}) {
      Matrix derivative(rows,n);LongMatrix global=LongMatrix::Zero(rows,n);
      for(int i=0;i<rows;++i)for(int j=0;j<n;++j) {
        derivative(i,j)=std::ldexp(double((i*7+j*3)%11-5),-4);global(i,columns[j])=derivative(i,j);
      }
      const Matrix expected=(global*inverse*global.transpose()).cast<double>();
      const auto result=selected.covariance(columns,derivative);assert(result.observed);
      assert((result.covariance-expected).norm()<1e-6*std::max(1.,expected.norm()));
    }
    selected.addNormal({0},Matrix::Constant(1,1,units[0]*units[0]));assert(selected.factorize(scale));
    const auto updated=selected.covariance({0},Matrix::Identity(1,1));
    const long double expected=inverse(0,0)/(1+units[0]*units[0]*inverse(0,0));
    assert(updated.observed && std::abs(updated.covariance(0,0)-double(expected))<1e-6*std::max(1.,double(expected)));
    Matrix free=h;free.row(n-1).setZero();free.col(n-1).setZero();
    SparseNormalInformation unobserved(n);std::vector<int> natural(n);std::iota(natural.begin(),natural.end(),0);
    unobserved.addNormal(natural,free);assert(!unobserved.factorize(scale));
    assert(!unobserved.covariance({n-1},Matrix::Identity(1,1)).observed);
  }
}

int main() {
  std::srand(43);dense_and_units();structured_covariance();chain(20,true);chain(20,false);chain(2000,true);chain(2000,false);
  std::cout<<"sparse selected covariance, unanchored refusal, cross-correlation and units passed\n";
}

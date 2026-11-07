// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>
namespace posim_slam {
// Selected covariance for an anchored, positive-definite pose normal matrix.
// A rank-deficient system is explicitly refused, never regularized. Its caller
// must use a rank-aware backend; no fixed pose or diagonal damping is inserted.
class SparseNormalInformation {
 public:
  using Sparse=Eigen::SparseMatrix<double>;
  explicit SparseNormalInformation(int columns):columns_(columns) {
    if(columns<1 || columns>24576)throw std::invalid_argument("sparse_pose_columns");
  }
  void addNormal(const std::vector<int>& columns,const Eigen::MatrixXd& normal) {
    if(columns.empty() || normal.rows()!=Eigen::Index(columns.size()) ||
        normal.cols()!=normal.rows() || !normal.allFinite() ||
        (normal-normal.transpose()).norm()>1e-10*std::max(1.,normal.norm()))
      throw std::invalid_argument("sparse_pose_normal");
    std::set<int> used;
    for(int c:columns)if(c<0 || c>=columns_ || !used.insert(c).second)
      throw std::invalid_argument("sparse_pose_indices");
    factorized_=false;ready_=false;
    for(size_t j=0;j<columns.size();++j)for(size_t i=0;i<columns.size();++i) {
      if(columns[i]<columns[j])continue;
      const double value=.5*(normal(i,j)+normal(j,i));if(value==0)continue;
      const uint64_t key=(uint64_t(columns[j])<<32)|uint64_t(columns[i]);
      values_[key]+=value;
      if(values_.size()>2000000)throw std::runtime_error("sparse_pose_storage_budget");
    }
  }
  void add(const std::vector<int>& columns,const Eigen::MatrixXd& rows) {
    if(rows.rows()<1 || rows.rows()>4096 || rows.cols()!=Eigen::Index(columns.size()) || !rows.allFinite())
      throw std::invalid_argument("sparse_pose_rows");
    addNormal(columns,rows.transpose()*rows);
  }
  bool factorize(const Eigen::VectorXd& scale) {
    if(scale.size()!=columns_ || !scale.allFinite() || scale.minCoeff()<=0)
      throw std::invalid_argument("sparse_pose_scale");
    if(factorized_ && scale_.size()==scale.size() && (scale_-scale).squaredNorm()==0)return ready_;
    cached_queries_=0;solve_queries_=0;
    scale_=scale;std::vector<Eigen::Triplet<double>> entries;entries.reserve(2*values_.size());
    for(const auto& [key,value]:values_) {
      const int row=int(key&0xffffffff),col=int(key>>32);
      const double scaled=value*scale[row]*scale[col];
      entries.emplace_back(row,col,scaled);if(row!=col)entries.emplace_back(col,row,scaled);
    }
    normal_.resize(columns_,columns_);normal_.setFromTriplets(entries.begin(),entries.end());normal_.makeCompressed();
    if(!normal_.coeffs().allFinite())throw std::runtime_error("sparse_pose_overflow");
    solver_.compute(normal_);factorized_=true;
    const double cutoff=64*columns_*std::numeric_limits<double>::epsilon()*std::max(1.,normal_.norm());
    ready_=solver_.info()==Eigen::Success && solver_.vectorD().allFinite() && solver_.vectorD().minCoeff()>cutoff;
    if(ready_ && solver_.matrixL().nestedExpression().nonZeros()>4000000)
      throw std::runtime_error("sparse_pose_factor_budget");
    if(ready_)recoverSelected();
    return ready_;
  }
  struct Result {bool observed=false;Eigen::MatrixXd covariance;};
  Result covariance(const std::vector<int>& columns,const Eigen::MatrixXd& derivative) const {
    if(!factorized_ || derivative.rows()<1 || derivative.rows()>24 ||
        derivative.cols()!=Eigen::Index(columns.size()) || !derivative.allFinite())
      throw std::invalid_argument("sparse_pose_function");
    if(!ready_)return {};
    std::set<int> checked;
    for(int c:columns)if(c<0 || c>=columns_ || !checked.insert(c).second)
      throw std::invalid_argument("sparse_pose_indices");
    Eigen::MatrixXd mapped=Eigen::MatrixXd::Zero(derivative.rows(),columns.size());bool cached=true;
    const auto& permutation=solver_.permutationP().indices();
    for(size_t j=0;j<columns.size() && cached;++j)for(size_t i=0;i<columns.size();++i) {
      const int row=columns[i],col=columns[j];
      if(row<0 || col<0 || row>=columns_ || col>=columns_)throw std::invalid_argument("sparse_pose_indices");
      auto value=selectedAt(permutation.size()?permutation[row]:row,permutation.size()?permutation[col]:col);
      if(!value){cached=false;break;}
      mapped.col(j).noalias()+=scale_[row]*scale_[col]*(*value)*derivative.col(i);
    }
    if(cached) {
      ++cached_queries_;
      const Eigen::MatrixXd result=mapped*derivative.transpose();
      return {true,.5*(result+result.transpose())};
    }
    ++solve_queries_;
    Eigen::MatrixXd rhs=Eigen::MatrixXd::Zero(columns_,derivative.rows());std::set<int> used;
    for(size_t i=0;i<columns.size();++i) {
      int c=columns[i];if(c<0 || c>=columns_ || !used.insert(c).second)
        throw std::invalid_argument("sparse_pose_indices");
      rhs.row(c)=scale_[c]*derivative.col(i).transpose();
    }
    const Eigen::MatrixXd solution=solver_.solve(rhs);
    if(solver_.info()!=Eigen::Success || !solution.allFinite() ||
        (normal_*solution-rhs).norm()>1e-9*std::max(1.,rhs.norm()))
      throw std::runtime_error("sparse_pose_solve_residual");
    const Eigen::MatrixXd covariance=rhs.transpose()*solution;
    return {true,.5*(covariance+covariance.transpose())};
  }
  Sparse unscaledNormal() const {
    std::vector<Eigen::Triplet<double>> entries;entries.reserve(2*values_.size());
    for(const auto& [key,value]:values_) {
      int row=int(key&0xffffffff),col=int(key>>32);
      entries.emplace_back(row,col,value);if(row!=col)entries.emplace_back(col,row,value);
    }
    Sparse result(columns_,columns_);result.setFromTriplets(entries.begin(),entries.end());return result;
  }
  bool ready() const {return ready_;}
  int rank() const {return ready_?columns_:-1;}
  int rows() const {return columns_;}
  size_t inputNonZeros() const {return normal_.nonZeros();}
  size_t factorNonZeros() const {return ready_?solver_.matrixL().nestedExpression().nonZeros():0;}
  size_t cachedQueries() const {return cached_queries_;}
  size_t solveQueries() const {return solve_queries_;}
  // Logical selected-inverse arrays and fill metadata; excludes the solver and
  // allocator/container overhead. The direct layout fits the old cache budget.
  size_t covarianceStorageBytes() const {
    size_t bytes=sizeof(double)*(size_t(lower_.nonZeros())+inverse_lower_.size()+inverse_diagonal_.size()+inverse_packed_.size())+
      sizeof(uint64_t)*inverse_pattern_.size()+sizeof(int)*component_.size();
    if(lower_.outerIndexPtr())bytes+=sizeof(Sparse::StorageIndex)*(size_t(lower_.nonZeros())+size_t(lower_.outerSize())+1);
    return bytes;
  }
 private:
  // Takahashi recursion on the filled LDLT pattern. Missing requested entries
  // use an exact solve; absent entries are never assumed to be zero unless
  // they belong to disconnected components of the factor graph.
  std::optional<double> selectedAt(int row,int col) const {
    if(!inverse_packed_.empty()) {
      if(component_[row]!=component_[col])return 0.;
      if(row<col)std::swap(row,col);const size_t index=size_t(row)*(size_t(row)+1)/2+size_t(col);
      if(!(inverse_pattern_[index/64]&(uint64_t(1)<<(index%64))))return {};
      return inverse_packed_[index];
    }
    if(row==col)return inverse_diagonal_[row];
    if(component_[row]!=component_[col])return 0.;
    if(row<col)std::swap(row,col);
    const int begin=lower_.outerIndexPtr()[col],end=lower_.outerIndexPtr()[col+1];
    const int* found=std::lower_bound(lower_.innerIndexPtr()+begin,lower_.innerIndexPtr()+end,row);
    if(found==lower_.innerIndexPtr()+end || *found!=row)return {};
    return inverse_lower_[found-lower_.innerIndexPtr()];
  }
  void recoverSelected() {
    const auto& factor=solver_.matrixL().nestedExpression();
    const size_t n=columns_,entries=n*(n+1)/2,words=(entries+63)/64;
    const size_t old_numeric_values=2*size_t(factor.nonZeros())+n;
    const bool packed=factor.isCompressed() && entries<=old_numeric_values && words<=old_numeric_values-entries;
    if(packed) {
      lower_.resize(0,0);lower_.data().squeeze();
      std::vector<double>().swap(inverse_lower_);std::vector<double>().swap(inverse_diagonal_);
      inverse_packed_.assign(entries,0.);inverse_pattern_.assign(words,0);
      const auto mark=[&](size_t row,size_t col) {
        if(row<col)std::swap(row,col);const size_t index=row*(row+1)/2+col;
        inverse_pattern_[index/64]|=uint64_t(1)<<(index%64);
      };
      for(int col=0;col<columns_;++col) {
        mark(col,col);for(Sparse::InnerIterator it(factor,col);it;++it)mark(it.row(),col);
      }
    } else {
      std::vector<double>().swap(inverse_packed_);std::vector<uint64_t>().swap(inverse_pattern_);
      lower_=factor;lower_.makeCompressed();
      inverse_lower_.assign(lower_.nonZeros(),0.);inverse_diagonal_.assign(columns_,0.);
    }
    const auto& lower=packed?factor:lower_;
    component_.resize(columns_);for(int i=0;i<columns_;++i)component_[i]=i;
    auto root=[&](int i){while(component_[i]!=i){component_[i]=component_[component_[i]];i=component_[i];}return i;};
    for(int col=0;col<columns_;++col)for(Sparse::InnerIterator it(lower,col);it;++it)
      if(it.row()!=col && it.value()!=0)component_[root(it.row())]=root(col);
    for(int i=0;i<columns_;++i)component_[i]=root(i);
    for(int col=columns_-1;col>=0;--col) {
      const int begin=lower.outerIndexPtr()[col],end=lower.outerIndexPtr()[col+1];
      for(int target=begin;target<end;++target) {
        int row=lower.innerIndexPtr()[target];if(row<=col)continue;
        double sum=0;
        for(int k=begin;k<end;++k) {
          int other=lower.innerIndexPtr()[k];if(other<=col || lower.valuePtr()[k]==0)continue;
          const auto value=selectedAt(other,row);
          if(!value)throw std::runtime_error("sparse_inverse_pattern");
          sum+=lower.valuePtr()[k]*(*value);
        }
        if(packed)inverse_packed_[size_t(row)*(size_t(row)+1)/2+size_t(col)]=-sum;
        else inverse_lower_[target]=-sum;
      }
      double diagonal=1/solver_.vectorD()[col];
      for(int k=begin;k<end;++k)if(lower.innerIndexPtr()[k]>col) {
        const double value=packed?inverse_packed_[size_t(lower.innerIndexPtr()[k])*(size_t(lower.innerIndexPtr()[k])+1)/2+size_t(col)]:inverse_lower_[k];
        diagonal-=lower.valuePtr()[k]*value;
      }
      if(!std::isfinite(diagonal) || diagonal<=0)throw std::runtime_error("sparse_inverse_diagonal");
      if(packed)inverse_packed_[size_t(col)*(size_t(col)+1)/2+size_t(col)]=diagonal;
      else inverse_diagonal_[col]=diagonal;
    }
  }

  int columns_;bool factorized_=false,ready_=false;
  std::unordered_map<uint64_t,double> values_;
  Eigen::VectorXd scale_;Sparse normal_;
  Sparse lower_;std::vector<double> inverse_lower_,inverse_diagonal_,inverse_packed_;
  std::vector<uint64_t> inverse_pattern_;std::vector<int> component_;
  mutable size_t cached_queries_=0,solve_queries_=0;
  Eigen::SimplicialLDLT<Sparse,Eigen::Lower,Eigen::AMDOrdering<int>> solver_;
};
} // namespace posim_slam

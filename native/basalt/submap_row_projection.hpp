// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "estimator_budget.hpp"
#include "input_digest.hpp"
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace posim_slam {
// Lossless representation of a projected Gaussian source group. Retain the
// sparse original rows and the exact Householder transform used by its owner;
// reconstruct the same dense marginal only while a reader holds its lease.
// This is process-local derived storage, not a portable checkpoint format.
class SubmapRowProjection {
 public:
  using Ptr=std::shared_ptr<const SubmapRowProjection>;
  struct Decoded {Eigen::MatrixXd jacobian;Eigen::VectorXd residual;};
  static InputDigest::Value identity(const Eigen::MatrixXd& j,const Eigen::VectorXd& r) {
    const std::array<int64_t,3> shape={j.rows(),j.cols(),r.size()};InputDigest hash;
    hash.update(shape.data(),sizeof(shape));hash.update(j.data(),sizeof(double)*j.size());
    hash.update(r.data(),sizeof(double)*r.size());return hash.finish();
  }
  static Ptr create(const Eigen::MatrixXd& source,const Eigen::VectorXd& residual,
      const Eigen::HouseholderQR<Eigen::MatrixXd>& transform,Eigen::Index removed,
      const Eigen::MatrixXd& reduced,const Eigen::VectorXd& reduced_residual) {
    const auto m=source.rows(),n=source.cols();
    if(m<0 || m>4096 || n<0 || n>kMaxJointColumns || removed<0 || removed>m ||
        residual.size()!=m || reduced.rows()!=m-removed || reduced.cols()!=n ||
        reduced_residual.size()!=m-removed || !source.allFinite() || !residual.allFinite() ||
        !reduced.allFinite() || !reduced_residual.allFinite())
      throw std::invalid_argument("submap_projection_shape");
    const size_t dense_bytes=sizeof(double)*(reduced.size()+reduced_residual.size());
    if(!dense_bytes)return {};
    if(removed && (transform.matrixQR().rows()!=m || transform.matrixQR().cols()!=removed ||
        transform.hCoeffs().size()!=removed || !transform.matrixQR().allFinite() || !transform.hCoeffs().allFinite()))
      throw std::invalid_argument("submap_projection_transform");
    size_t count=0;
    for(Eigen::Index i=0;i<source.size();++i) {
      uint64_t bits;std::memcpy(&bits,source.data()+i,sizeof(bits));count+=bits!=0;
    }
    const size_t size=4*sizeof(int64_t)+count*(sizeof(uint32_t)+sizeof(double))+
        sizeof(double)*size_t(m+m*removed+removed);
    if(size>=dense_bytes)return {};
    auto out=std::shared_ptr<SubmapRowProjection>(new SubmapRowProjection);
    out->payload_.resize(size);out->expected_=identity(reduced,reduced_residual);
    size_t offset=0;
    const auto append=[&](const void* data,size_t bytes) {
      if(bytes)std::memcpy(out->payload_.data()+offset,data,bytes);offset+=bytes;
    };
    const std::array<int64_t,4> shape={m,n,removed,int64_t(count)};append(shape.data(),sizeof(shape));
    // Original column-major bit patterns, including signed zero, are preserved.
    for(Eigen::Index i=0;i<source.size();++i) {
      uint64_t bits;std::memcpy(&bits,source.data()+i,sizeof(bits));
      if(bits){const uint32_t index=uint32_t(i);append(&index,sizeof(index));append(source.data()+i,sizeof(double));}
    }
    append(residual.data(),sizeof(double)*m);
    if(removed) {
      append(transform.matrixQR().data(),sizeof(double)*m*removed);
      append(transform.hCoeffs().data(),sizeof(double)*removed);
    }
    if(offset!=size)throw std::logic_error("submap_projection_encoding_size");
    return out;
  }
  size_t bytes() const {return payload_.size();}
  const std::vector<uint8_t>& payload() const {return payload_;}
  const InputDigest::Value& expected() const {return expected_;}
  bool matches(const Eigen::MatrixXd& j,const Eigen::VectorXd& r) const {return identity(j,r)==expected_;}
  static Decoded decode(const void* data,size_t bytes,Eigen::Index rows,Eigen::Index columns,
      const InputDigest::Value& expected) {
    if(!data || bytes<4*sizeof(int64_t))throw std::runtime_error("submap_projection_shape");
    const auto input=static_cast<const uint8_t*>(data);size_t offset=0;
    const auto read=[&](void* target,size_t size) {
      if(size>bytes-offset)throw std::runtime_error("submap_projection_shape");
      if(size)std::memcpy(target,input+offset,size);offset+=size;
    };
    std::array<int64_t,4> shape;read(shape.data(),sizeof(shape));
    const auto m=shape[0],n=shape[1],removed=shape[2],count=shape[3];
    if(m<1 || m>4096 || n<1 || n>kMaxJointColumns || removed<0 || removed>=m ||
        removed>kMaxJointColumns || m-removed!=rows || n!=columns || count<0 || count>m*n ||
        bytes!=4*sizeof(int64_t)+size_t(count)*(sizeof(uint32_t)+sizeof(double))+
               sizeof(double)*size_t(m+m*removed+removed))
      throw std::runtime_error("submap_projection_shape");
    Eigen::MatrixXd combined=Eigen::MatrixXd::Zero(m,n+1);uint32_t previous=0;
    for(int64_t i=0;i<count;++i) {
      uint32_t index;double value;read(&index,sizeof(index));read(&value,sizeof(value));
      if(index>=m*n || (i && index<=previous) || !std::isfinite(value))
        throw std::runtime_error("submap_projection_shape");
      combined.data()[index]=value;previous=index;
    }
    read(combined.col(n).data(),sizeof(double)*m);
    if(removed) {
      Eigen::MatrixXd vectors(m,removed);Eigen::VectorXd coefficients(removed);
      read(vectors.data(),sizeof(double)*vectors.size());read(coefficients.data(),sizeof(double)*coefficients.size());
      if(!vectors.allFinite() || !coefficients.allFinite())throw std::runtime_error("submap_projection_shape");
      Eigen::HouseholderSequence<Eigen::MatrixXd,Eigen::VectorXd> transform(vectors,coefficients);
      combined=(transform.adjoint()*combined).eval();
    }
    Decoded result{combined.bottomRows(rows).leftCols(n),combined.col(n).tail(rows)};
    if(offset!=bytes || identity(result.jacobian,result.residual)!=expected)
      throw std::runtime_error("submap_projection_digest");
    return result;
  }
 private:
  std::vector<uint8_t> payload_;InputDigest::Value expected_{};
};
}

// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_joint.hpp"
#include <basalt/utils/imu_types.h>
#include <vector>

namespace posim_slam {
inline void requireStateOrder(const basalt::AbsOrderMap& order) {
  if(order.total_size<6 || order.total_size>512 || order.items!=order.abs_order_map.size())
    throw std::invalid_argument("invalid_joint_state_order");
  std::vector<bool> occupied(order.total_size,false);
  for(const auto& [stamp,block]:order.abs_order_map) {
    const auto [offset,size]=block;
    if(stamp<=0 || offset<0 || (size!=6 && size!=15) || (size_t(offset)+size)>order.total_size)
      throw std::invalid_argument("invalid_joint_state_block");
    for(int i=offset;i<offset+size;++i) {
      if(occupied[i])throw std::invalid_argument("overlapping_joint_state_blocks");
      occupied[i]=true;
    }
  }
  if(std::find(occupied.begin(),occupied.end(),false)!=occupied.end())
    throw std::invalid_argument("incomplete_joint_state_order");
}

// Input columns: source pose/inertial states, then ALL shared plane/landmark
// coordinates being retired. Include each raw factor exactly once, including
// the existing prior and any relevant inertial rows. Do not append a second
// independently eliminated copy of the same visual rows.
// The caller owns factor retirement and FEJ Jacobians. delta is the retained
// state's current displacement from its existing Basalt linearization point.
inline basalt::MargLinData<double> marginalizeJointSystem(
    const Eigen::MatrixXd& jacobian,const Eigen::VectorXd& residual,
    const basalt::AbsOrderMap& source,const basalt::AbsOrderMap& retained,
    const Eigen::VectorXd& delta) {
  requireStateOrder(source);requireStateOrder(retained);
  if(jacobian.rows()<1 || jacobian.rows()>4096 || jacobian.cols()>1024 ||
     jacobian.cols()<=Eigen::Index(source.total_size) ||
     delta.size()!=Eigen::Index(retained.total_size) || !delta.allFinite())
    throw std::invalid_argument("invalid_joint_marginal_layout");
  std::vector<int> keep(retained.total_size);std::vector<bool> used(jacobian.cols(),false);
  for(const auto& [stamp,target]:retained.abs_order_map) {
    const auto found=source.abs_order_map.find(stamp);
    if(found==source.abs_order_map.end() || target.second>found->second.second)
      throw std::invalid_argument("unknown_joint_retained_state");
    // A 15-state inertial block may retain only its first six pose coordinates.
    for(int i=0;i<target.second;++i) {
      const int column=found->second.first+i;
      keep[target.first+i]=column;used[column]=true;
    }
  }
  Eigen::MatrixXd ordered(jacobian.rows(),jacobian.cols());int column=0;
  for(int index:keep)ordered.col(column++)=jacobian.col(index);
  for(Eigen::Index index=0;index<jacobian.cols();++index)
    if(!used[index])ordered.col(column++)=jacobian.col(index);
  const auto reduced=eliminateJointPatch(ordered,residual,retained.total_size);
  basalt::MargLinData<double> prior;prior.is_sqrt=true;prior.order=retained;
  prior.H=reduced.jacobian;
  // Basalt evaluates H*(delta + increment)+b, not H*increment+b.
  prior.b=reduced.residual-prior.H*delta;
  if(!prior.H.allFinite() || !prior.b.allFinite())throw std::runtime_error("nonfinite_joint_marginal_prior");
  return prior;
}
}  // namespace posim_slam

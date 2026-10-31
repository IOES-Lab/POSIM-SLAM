// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_linearization.hpp"
#include "surface_owner.hpp"
#include "dvl_factors.hpp"

namespace posim_slam {
struct NativeSurfaceInformation {
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  Eigen::MatrixXd constraint_jacobian;Eigen::VectorXd constraint_residual;
  size_t imu_intervals=0,dvl_observations=0,pressure_observations=0;
};

// Owner-thread, camera-free local Gauss-Newton information. The marginal owner
// and active raw stores contribute once, in exactly the optimizer's coordinates
// and frozen robust weights. Numerical trial damping is never included.
inline NativeSurfaceInformation nativeSurfaceInformation(
    basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
    const basalt::MargLinData<double>& legacy,const basalt::ImuLinData<double>& imu,
    const DvlFactorStore* dvl,const PressureFactorStore& pressure,const SurfaceMarginalOwner& surface) {
  if(!surface.active() || owner.posim_ping_trial_active || !owner.lmdb.getLandmarks().empty())
    throw std::invalid_argument("exclusive_surface_information_required");
  SurfaceMarginalOwner::requireZero(legacy,pressure);
  const auto current=surface.coordinates(owner,order,pressure);
  // This is a persistent snapshot, not the transient seventeenth-state source.
  surface_prior::layout(current.order);
  const auto marginal=surface.linearize(owner,order,pressure);
  const auto depth=pressure.linearize(owner,order);
  DvlFactorStore::Rows velocity;
  if(dvl)velocity=dvl->linearize(owner,order);
  else {velocity.jacobian.resize(0,order.total_size);velocity.residual.resize(0);}
  PingLinearization::Options options;
  options.lb_options.huber_parameter=owner.huber_thresh;options.lb_options.obs_std_dev=owner.obs_std_dev;
  basalt::LinearizationAbsQR<double,6> active(&owner,order,options,&legacy,&imu);
  bool valid=false;active.linearizeProblem(&valid);
  if(!valid)throw std::invalid_argument("invalid_surface_information_linearization");
  active.performQR();Eigen::MatrixXd ji;Eigen::VectorXd ri;active.get_dense_Q2Jp_Q2r(ji,ri);
  NativeSurfaceInformation out;
  const auto columns=current.delta.size();
  Eigen::MatrixXd augmented=Eigen::MatrixXd::Zero(marginal.jacobian.rows()+depth.jacobian.rows()+velocity.jacobian.rows()+ji.rows(),columns+1);
  Eigen::Index offset=0;
  auto append=[&](const Eigen::MatrixXd& j,const Eigen::VectorXd& r) {
    if(j.cols()>columns || j.rows()!=r.size() || !j.allFinite() || !r.allFinite())
      throw std::invalid_argument("invalid_surface_information_rows");
    augmented.block(offset,0,j.rows(),j.cols())=j;augmented.col(columns).segment(offset,r.size())=r;offset+=j.rows();
  };
  append(marginal.jacobian,marginal.residual);append(depth.jacobian,depth.residual);
  append(velocity.jacobian,velocity.residual);append(ji,ri);
  if(augmented.rows()>columns+1) {
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(augmented);
    augmented=qr.matrixQR().topRows(columns+1).triangularView<Eigen::Upper>();
  }
  out.jacobian=augmented.leftCols(columns);out.residual=augmented.col(columns);
  out.constraint_jacobian=marginal.constraint_jacobian;out.constraint_residual=marginal.constraint_residual;
  out.imu_intervals=imu.imu_meas.size();out.dvl_observations=dvl ? dvl->size() : 0;
  out.pressure_observations=pressure.size();return out;
}
}  // namespace posim_slam

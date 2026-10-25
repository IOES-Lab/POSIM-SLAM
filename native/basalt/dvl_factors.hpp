// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_marginal.hpp"
#include <basalt/vi_estimator/ba_base.h>

namespace posim_slam {
struct DvlCalibration {
  Eigen::Matrix3d dvl_to_imu=Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_to_dvl=Eigen::Vector3d::Zero();
};
struct DvlObservation {
  int64_t stamp;
  Eigen::Vector3d velocity,gyro;
  Eigen::Matrix3d covariance,gyro_covariance;
};
inline Eigen::Matrix3d dvlCross(const Eigen::Vector3d& v) {
  Eigen::Matrix3d a;a<<0,-v.z(),v.y(),v.z(),0,-v.x(),-v.y(),v.x(),0;return a;
}
inline void validateDvlCalibration(const DvlCalibration& c) {
  if(!c.dvl_to_imu.allFinite() || !c.imu_to_dvl.allFinite() || c.imu_to_dvl.norm()>10 ||
     (c.dvl_to_imu.transpose()*c.dvl_to_imu-Eigen::Matrix3d::Identity()).norm()>1e-8 ||
     std::abs(c.dvl_to_imu.determinant()-1)>1e-8)
    throw std::invalid_argument("invalid_dvl_mount");
}
inline Eigen::Matrix3d dvlWhitening(const DvlCalibration& c,const DvlObservation& o) {
  validateDvlCalibration(c);
  if(o.stamp<=0 || !o.velocity.allFinite() || o.velocity.norm()>50 ||
     !o.gyro.allFinite() || o.gyro.norm()>35 || !o.covariance.allFinite() ||
     !o.gyro_covariance.allFinite() ||
     !o.covariance.isApprox(o.covariance.transpose(),1e-10) ||
     !o.gyro_covariance.isApprox(o.gyro_covariance.transpose(),1e-10))
    throw std::invalid_argument("invalid_dvl_observation");
  Eigen::LLT<Eigen::Matrix3d> sensor(o.covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> gyro(o.gyro_covariance);
  if(sensor.info()!=Eigen::Success || gyro.info()!=Eigen::Success || gyro.eigenvalues().minCoeff()<0)
    throw std::invalid_argument("invalid_dvl_covariance");
  const Eigen::Matrix3d lever=c.dvl_to_imu.transpose()*dvlCross(c.imu_to_dvl);
  const Eigen::Matrix3d covariance=o.covariance+lever*o.gyro_covariance*lever.transpose();
  Eigen::LLT<Eigen::Matrix3d> factor(covariance);
  if(factor.info()!=Eigen::Success)throw std::invalid_argument("invalid_dvl_effective_covariance");
  return factor.matrixL().solve(Eigen::Matrix3d::Identity());
}
struct DvlRows {
  Eigen::Matrix<double,3,15> jacobian;
  Eigen::Vector3d residual;
  double cost;
};
// Basalt increments are world translation, left/world rotation, world velocity,
// gyro bias, accel bias. Jacobians may use a frozen FEJ reference; residuals and
// the Huber weight always use the current state. Raw gyro is a conditional input:
// its lever-arm variance is propagated, but correlation with IMU preintegration
// noise is not represented by this approximation.
inline DvlRows dvlRows(const basalt::PoseVelBiasState<double>& current,
    const basalt::PoseVelBiasState<double>& reference,const DvlCalibration& c,
    const DvlObservation& o,bool robust=true) {
  if(current.t_ns!=o.stamp || reference.t_ns!=o.stamp)throw std::invalid_argument("dvl_state_stamp_mismatch");
  const Eigen::Matrix3d whiten=dvlWhitening(c,o),rotation=current.T_w_i.rotationMatrix();
  const Eigen::Vector3d prediction=c.dvl_to_imu.transpose()*(rotation.transpose()*current.vel_w_i+
      (o.gyro-current.bias_gyro).cross(c.imu_to_dvl));
  DvlRows result;result.residual=whiten*(prediction-o.velocity);result.jacobian.setZero();
  const Eigen::Matrix3d frame=c.dvl_to_imu.transpose()*reference.T_w_i.rotationMatrix().transpose();
  result.jacobian.block<3,3>(0,3)=whiten*frame*dvlCross(reference.vel_w_i);
  result.jacobian.block<3,3>(0,6)=whiten*frame;
  result.jacobian.block<3,3>(0,9)=whiten*c.dvl_to_imu.transpose()*dvlCross(c.imu_to_dvl);
  const double norm=result.residual.norm(),threshold=3.*std::sqrt(3.);
  const double weight=robust && norm>threshold ? threshold/norm : 1.;
  result.cost=weight==1. ? .5*norm*norm : threshold*(norm-.5*threshold);
  result.jacobian*=std::sqrt(weight);result.residual*=std::sqrt(weight);
  if(!result.jacobian.allFinite() || !result.residual.allFinite() || !std::isfinite(result.cost))
    throw std::invalid_argument("nonfinite_dvl_linearization");
  return result;
}

class DvlFactorStore {
 public:
  struct Rows {Eigen::MatrixXd jacobian;Eigen::VectorXd residual;std::vector<int> offsets;double cost=0.;};
  struct Retirement {uint64_t revision;std::set<int64_t> stamps;};
  explicit DvlFactorStore(DvlCalibration calibration):calibration_(calibration) {validateDvlCalibration(calibration_);}
  void add(const DvlObservation& observation) {
    if(observation.stamp<=last_stamp_ || observations_.size()>=kMaxLivePoseBlocks)
      throw std::invalid_argument("replayed_or_excess_dvl_observation");
    dvlWhitening(calibration_,observation);
    observations_.emplace(observation.stamp,observation);last_stamp_=observation.stamp;++revision_;
  }
  int64_t lastObservation() const {return last_stamp_;}
  size_t size() const {return observations_.size();}
  Rows linearize(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const std::set<int64_t>* selected=nullptr) const {
    requireStateOrder(order);
    Rows rows;const auto count=selected ? selected->size() : observations_.size();
    rows.jacobian=Eigen::MatrixXd::Zero(3*count,order.total_size);rows.residual.resize(3*count);
    size_t offset=0;
    for(const auto& [stamp,observation]:observations_) {
      if(selected && !selected->count(stamp))continue;
      const auto block=order.abs_order_map.find(stamp);
      if(block==order.abs_order_map.end() || block->second.second!=15 || !owner.frame_states.count(stamp))
        throw std::invalid_argument("missing_dvl_inertial_state");
      const auto& state=owner.frame_states.at(stamp);
      const auto local=dvlRows(state.getState(),state.getStateLin(),calibration_,observation);
      rows.jacobian.block<3,15>(offset,block->second.first)=local.jacobian;
      rows.offsets.push_back(block->second.first);
      rows.residual.segment<3>(offset)=local.residual;rows.cost+=local.cost;offset+=3;
    }
    if(offset!=3*count)throw std::invalid_argument("unknown_dvl_retirement_stamp");
    return rows;
  }
  Retirement planRetirement(const std::set<int64_t>& removed,const std::set<int64_t>& pose_only) const {
    Retirement plan{revision_,{}};
    for(const auto& [stamp,observation]:observations_)
      if(removed.count(stamp) || pose_only.count(stamp))plan.stamps.insert(stamp);
    return plan;
  }
  template<class Install> void retireWith(const Retirement& plan,Install install) {
    if(plan.revision!=revision_)throw std::invalid_argument("stale_dvl_retirement");
    for(auto stamp:plan.stamps)if(!observations_.count(stamp))throw std::invalid_argument("unknown_dvl_retirement_stamp");
    install();
    for(auto stamp:plan.stamps)observations_.erase(stamp);
    ++revision_;
  }
 private:
  DvlCalibration calibration_;
  std::map<int64_t,DvlObservation> observations_;
  int64_t last_stamp_=0;uint64_t revision_=0;
};
}

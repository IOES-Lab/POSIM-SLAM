// SPDX-License-Identifier: Apache-2.0
#include <basalt/imu/preintegration.h>
#include <Eigen/Eigenvalues>
#include <cassert>
#include <iostream>

int main() {
  using Measurement=basalt::IntegratedImuMeasurement<double>;
  const Eigen::Vector3d qa(.0004,.0009,.0016),qg(.000001,.000004,.000009);
  double maximum_whitening=0.,maximum_partition=0.;
  for(int64_t interval:{1000000LL,10000000LL,37000000LL}) {
    const int64_t start=1000000000;const double dt=interval*1e-9;
    Measurement one(start,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    basalt::ImuData<double> sample;sample.t_ns=start+interval;sample.accel.setZero();sample.gyro.setZero();
    one.integrateContinuous(sample,qa,qg);
    Measurement legacy(start,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    legacy.integrate(sample,qa/dt,qg/dt);
    Eigen::FullPivLU<Eigen::Matrix<double,9,9>> legacy_rank(legacy.get_cov());
    assert(legacy_rank.rank()==6); // Reproduce the lost within-interval modes.
    Eigen::Matrix<double,9,9> expected=Eigen::Matrix<double,9,9>::Zero();
    expected.block<3,3>(0,0)=(qa*(dt*dt*dt/3)).asDiagonal();
    expected.block<3,3>(0,6)=(qa*(dt*dt/2)).asDiagonal();
    expected.block<3,3>(6,0)=expected.block<3,3>(0,6);
    expected.block<3,3>(6,6)=(qa*dt).asDiagonal();
    expected.block<3,3>(3,3)=(qg*dt).asDiagonal();
    assert((one.get_cov()-expected).norm()<expected.norm()*1e-12);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9>> eigen(one.get_cov());
    assert(eigen.eigenvalues().minCoeff()>0);
    const auto& whitening=one.get_sqrt_cov_inv();
    maximum_whitening=std::max(maximum_whitening,
        (whitening*one.get_cov()*whitening.transpose()-Eigen::Matrix<double,9,9>::Identity()).norm());
    // Independent analytic partition test, including nonuniform subintervals.
    Measurement split(start,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
    for(int64_t elapsed:{interval/7,interval*3/5,interval}) {
      sample.t_ns=start+elapsed;split.integrateContinuous(sample,qa,qg);
    }
    maximum_partition=std::max(maximum_partition,(split.get_cov()-expected).norm()/expected.norm());
    assert((split.getDeltaState().T_w_i.matrix()-one.getDeltaState().T_w_i.matrix()).norm()<1e-12);
    for(int axis=0;axis<3;++axis) {
      Eigen::Matrix<double,9,1> shifted=Eigen::Matrix<double,9,1>::Zero();shifted[axis]=.001;
      assert((whitening*shifted).squaredNorm()>1.); // No free position axis.
    }
    bool rejected=false;try{split.integrateContinuous(sample,qa,qg);}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  assert(maximum_whitening<1e-10);assert(maximum_partition<1e-12);
  std::cout << "PASS continuous IMU single/irregular intervals; whitening " << maximum_whitening
    << "; partition relative error " << maximum_partition << std::endl;
}

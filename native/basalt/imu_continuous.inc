  // POSIM extension. Inputs are continuous white-noise power spectral
  // densities, in the same units as Calibration::{accel,gyro}_noise_std^2.
  // The midpoint attitude/gyro propagation remains Basalt's approximation.
  // Acceleration's integrated position/velocity covariance is [dt^3/3,
  // dt^2/2; dt^2/2, dt], including the within-interval Brownian bridge.
  // Dropping that bridge makes a one-sample interval rank deficient; a
  // covariance pseudoinverse then incorrectly removes a position constraint.
  void integrateContinuous(const ImuData<Scalar>& data, const Vec3& accel_psd,
                           const Vec3& gyro_psd) {
    const Scalar dt = (data.t_ns - start_t_ns_ - delta_state_.t_ns) * Scalar(1e-9);
    if (!(dt > 0) || !std::isfinite(dt) || !accel_psd.allFinite() ||
        !gyro_psd.allFinite() || (accel_psd.array() <= 0).any() ||
        (gyro_psd.array() <= 0).any())
      throw std::invalid_argument("invalid_continuous_imu_interval");
    const Mat3 rotation = (delta_state_.T_w_i.so3() *
        SO3::exp(Scalar(.5) * dt * (data.gyro - bias_gyro_lin_))).matrix();
    integrate(data, accel_psd / dt, gyro_psd / dt);
    cov_.template topLeftCorner<3, 3>() += rotation * accel_psd.asDiagonal() *
        rotation.transpose() * (dt * dt * dt / Scalar(12));
    sqrt_cov_inv_computed_ = false;
  }


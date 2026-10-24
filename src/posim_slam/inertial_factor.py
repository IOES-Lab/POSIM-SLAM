"""Whitened raw-IMU preintegration factor for a joint state optimizer.

Uses ImuWindow's measured preintegration and explicit noise calibration. Endpoint
states and gravity are optimization variables, not external pose measurements.
Large bias changes require fresh preintegration; this factor has no pose prior.
"""
import math
import numpy as np
from .dvl_scale import rotation,vector
from .inertial import exp_so3
from .inertial_alignment import log_so3
from .inertial_filter import covariance


def pair_residual(preintegrated,first,second,*,gravity):
    """Whiten p/v/right-rotation/bg/ba residuals, keeping their correlations.

    States contain stamp, position, velocity, attitude, gyro_bias and accel_bias.
    Gravity is an explicit map-frame vector. Bias corrections are first-order
    around the preintegration's stored biases. The caller must re-integrate
    after nonlinear steps instead of treating this approximation as exact.
    """
    pre=preintegrated;dt=pre['dt']
    if (not math.isfinite(dt) or not 0<dt<=3 or pre.get('gravity_included') is not False or
        not all(math.isfinite(t) for t in (first['stamp'],second['stamp'],pre['start'],pre['end'])) or
        abs(pre['end']-pre['start']-dt)>1e-9 or
        abs(first['stamp']-pre['start'])>1e-9 or abs(second['stamp']-pre['end'])>1e-9):
        raise ValueError('preintegration_state_interval_mismatch')
    r0=rotation(first['attitude']);r1=rotation(second['attitude']);g=vector(gravity)
    p0,p1=vector(first['position']),vector(second['position'])
    v0,v1=vector(first['velocity']),vector(second['velocity'])
    bg0,bg1=vector(first['gyro_bias']),vector(second['gyro_bias'])
    ba0,ba1=vector(first['accel_bias']),vector(second['accel_bias'])
    bias=np.r_[bg0-vector(pre['gyro_bias']),ba0-vector(pre['accel_bias'])]
    j=np.asarray(pre['bias_jacobian'],dtype=float)
    if j.shape!=(9,6) or not np.isfinite(j).all():raise ValueError('invalid_preintegration_bias_jacobian')
    correction=j@bias
    dp=vector(pre['delta_position'])+correction[:3]
    dv=vector(pre['delta_velocity'])+correction[3:6]
    dr=rotation(pre['rotation'])@exp_so3(correction[6:9])
    raw=np.r_[r0.T@(p1-p0-v0*dt-.5*g*dt*dt)-dp,
              r0.T@(v1-v0-g*dt)-dv,log_so3(dr.T@r0.T@r1),bg1-bg0,ba1-ba0]
    noise=covariance(pre['covariance'],15)
    return np.linalg.solve(np.linalg.cholesky(noise),raw)

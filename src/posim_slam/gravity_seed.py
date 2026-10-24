"""Rotation-compensated multi-sample gravity seed; not dynamic initialization.

The mean calibrated specific force is transported into the final IMU frame.
Translational acceleration and bias uncertainty remain confounded with gravity.
This is a numerical starting direction, never an observability certificate.
"""
import numpy as np
from .inertial import exp_so3
from .dvl_scale import vector


def gravity_seed(window,start,end,*,gyro_bias,accel_bias):
    if not .2-1e-9<=end-start<=3:raise ValueError('gravity_seed_duration')
    bg,ba=vector(gyro_bias),vector(accel_bias)
    knots,raw=window.interval(start,end)
    transport=np.eye(3);total=np.zeros(3)
    for i in range(len(knots)-2,-1,-1):
        dt=knots[i+1]-knots[i]
        previous=transport@exp_so3(-(.5*(raw[i,:3]+raw[i+1,:3])-bg)*dt)
        total+=.5*(previous@(raw[i,3:]-ba)+transport@(raw[i+1,3:]-ba))*dt
        transport=previous
    force=total/(end-start);magnitude=float(np.linalg.norm(force))
    if not np.isfinite(magnitude) or magnitude<1:raise ValueError('gravity_seed_degenerate')
    return dict(stamp=end,direction_imu=force/magnitude,mean_specific_force_m_s2=force,
                samples=len(knots),duration_s=end-start,gravity_magnitude_estimated=False,
                velocity_estimated=False,bias_estimated=False,observability_qualified=False)

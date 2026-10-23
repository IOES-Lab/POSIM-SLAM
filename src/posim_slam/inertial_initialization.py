"""Stationary raw-IMU/DVL initialization in a gravity-aligned local gauge.

Stationarity is a model assumption supported by independent bottom-locked DVL
and bounded IMU dynamics, not a proof against every constant motion. Yaw and
position are local coordinate gauges, not geographic heading or global pose.
Accelerometer bias remains a supplied calibration prior; gravity cannot identify
all its components from rest. No ideal orientation or simulator pose is accepted.
"""
import math
import numpy as np
from .dvl_scale import rotation,vector
from .inertial import skew
from .inertial_filter import InertialFilter,covariance

ALGORITHM_VERSION='0.1.0'


def stationary_initialize(window,start,end,dvl_samples,*,dvl_to_imu,imu_to_dvl,
                          accel_bias,accel_bias_covariance,noise_density,
                          angular_stationarity_std,linear_stationarity_std,
                          gravity_m_s2=9.80665):
    """Return filter + diagnostics after a covered two-to-three second window.

    DVL samples: (simulation seconds, measured 3D velocity, 3x3 covariance,
    bottom_locked bool). At least three real samples bracket the whole interval,
    with <=1.5 s gaps. Zero/unknown covariance is rejected. Noise densities follow
    InertialFilter order; explicit stationarity standard deviations represent
    residual motion uncertainty, not estimated sensor noise.
    """
    if (any(type(x) not in (int,float) or not math.isfinite(x) for x in (start,end))
            or not 2<=end-start<=3):raise ValueError('stationary_duration_required')
    duration=end-start
    for value in (angular_stationarity_std,linear_stationarity_std,gravity_m_s2):
        if type(value) not in (int,float) or not math.isfinite(value) or value<=0:
            raise ValueError('stationary_calibration_required')
    if angular_stationarity_std>.01 or linear_stationarity_std>.02 or not 9<gravity_m_s2<11:
        raise ValueError('stationary_calibration_out_of_range')
    mount=rotation(dvl_to_imu);lever=vector(imu_to_dvl);ba=vector(accel_bias)
    ba_cov=covariance(accel_bias_covariance,3)
    noise=np.asarray(noise_density,dtype=float)
    if noise.shape!=(4,) or not np.isfinite(noise).all() or np.any(noise<=0):
        raise ValueError('imu_noise_calibration_required')
    knots,raw=window.interval(start,end);dt=np.diff(knots)
    mean=np.sum((raw[:-1]+raw[1:])*.5*dt[:,None],axis=0)/duration
    residual=raw-mean
    variance=np.sum((residual[:-1]**2+residual[1:]**2)*.5*dt[:,None],axis=0)/duration
    gyro_rms=float(np.sqrt(variance[:3].sum()));accel_rms=float(np.sqrt(variance[3:].sum()))
    force=mean[3:]-ba;magnitude=float(np.linalg.norm(force))
    if (np.linalg.norm(mean[:3])>.03 or gyro_rms>.02 or accel_rms>.12
            or abs(magnitude-gravity_m_s2)>.25):raise ValueError('imu_not_stationary')
    samples=list(dvl_samples)
    if not 3<=len(samples)<=32:raise ValueError('stationary_dvl_samples_required')
    times=[];velocities=[];covariances=[]
    for stamp,velocity,uncertainty,locked in samples:
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:
            raise ValueError('invalid_dvl_timestamp')
        if type(locked) is not bool or not locked:raise ValueError('dvl_bottom_lock_required')
        if times and stamp<=times[-1]:raise ValueError('dvl_time_reversal')
        times.append(float(stamp));velocities.append(vector(velocity));covariances.append(covariance(uncertainty,3))
    times=np.array(times)
    if times[0]>start or times[-1]<end:raise ValueError('stationary_dvl_not_covered')
    lo=max(0,int(np.searchsorted(times,start,side='right'))-1)
    hi=min(len(times),int(np.searchsorted(times,end,side='left'))+1)
    selected=times[lo:hi]
    if len(selected)<3 or np.max(np.diff(selected))>1.5:raise ValueError('stationary_dvl_gap')
    velocities=np.asarray(velocities)[lo:hi];covariances=np.asarray(covariances)[lo:hi]
    margins=[float(np.linalg.norm(v)+3*math.sqrt(np.linalg.eigvalsh(c).max())) for v,c in zip(velocities,covariances)]
    if max(margins)>.06:raise ValueError('dvl_not_stationary')
    up=force/magnitude
    # Define local yaw by the first physical IMU axis with a stable horizontal
    # projection. This handles inverted FRD mounts without an antipodal formula.
    axis=np.array([1.,0.,0.]) if abs(up[0])<.95 else np.array([0.,1.,0.])
    horizontal=axis-up*np.dot(axis,up);horizontal/=np.linalg.norm(horizontal)
    attitude=np.vstack((horizontal,np.cross(up,horizontal),up))
    j=skew(up)/magnitude
    tilt_noise=noise[1]**2/duration+(gravity_m_s2*angular_stationarity_std)**2
    p=np.zeros((15,15));p[:3,:3]=np.eye(3)*1e-8 # local origin gauge
    gyro_cov=np.eye(3)*(noise[0]**2/duration+angular_stationarity_std**2+noise[2]**2*duration/3)
    # DVL mean uncertainty plus residual motion and rotational lever-arm noise.
    sensor_cov=np.sum(covariances,axis=0)/len(covariances)**2
    velocity_cov=mount@sensor_cov@mount.T+skew(lever)@gyro_cov@skew(lever).T
    p[3:6,3:6]=attitude@velocity_cov@attitude.T+np.eye(3)*linear_stationarity_std**2
    p[6:9,6:9]=j@(ba_cov+np.eye(3)*tilt_noise)@j.T+np.outer(up,up)*1e-8
    p[9:12,9:12]=gyro_cov;p[12:15,12:15]=ba_cov
    p[6:9,12:15]=j@ba_cov;p[12:15,6:9]=p[6:9,12:15].T
    result=InertialFilter(stamp=end,position=[0,0,0],velocity=[0,0,0],attitude=attitude,
        gyro_bias=mean[:3],accel_bias=ba,state_covariance=p,noise_density=noise,gravity=[0,0,-gravity_m_s2])
    diagnostics=dict(algorithm_version=ALGORITHM_VERSION,start=start,end=end,imu_samples=len(knots),
        dvl_samples=len(selected),gyro_rms_rad_s=gyro_rms,accel_rms_m_s2=accel_rms,
        gravity_norm_m_s2=magnitude,max_dvl_speed_3sigma_m_s=max(margins),
        initial_yaw='local_gauge',initial_position='local_origin',accel_bias_estimated=False,
        stationary_assumption=True,orientation_measurement_used=False,navigation_ready=False)
    return result,diagnostics

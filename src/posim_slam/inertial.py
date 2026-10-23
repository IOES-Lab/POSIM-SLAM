"""Bounded raw-IMU preintegration in the interval's initial IMU frame.

This is estimator input, not a navigation solution. Deltas exclude gravity and
initial velocity. No simulator orientation/pose is accepted. The caller supplies
calibrated gyro/accelerometer biases and continuous noise densities, and must
jointly estimate gravity, velocity and biases using independent observations.
Error/covariance ordering is position, velocity, right attitude error, gyro bias,
accelerometer bias. Units are SI and timestamps are simulation seconds.
"""
from collections import deque
import math
import numpy as np
from .dvl_scale import vector


def skew(value):
    x,y,z=value
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def exp_so3(value):
    value=vector(value);angle=float(np.linalg.norm(value));cross=skew(value)
    if angle<1e-7:
        a=1-angle**2/6;b=.5-angle**2/24
    else:a=math.sin(angle)/angle;b=(1-math.cos(angle))/angle**2
    return np.eye(3)+a*cross+b*cross@cross


def error_transition(mid,omega,accel,dt,noise):
    """Shared midpoint right-error dynamics and integrated process noise.

    Position/velocity errors use the integration frame; attitude error uses the
    current IMU frame. Three-point positive quadrature preserves PSD noise and
    position/velocity/bias correlations at sensor cadence.
    """
    f=np.zeros((15,15));f[:3,3:6]=np.eye(3)
    f[3:6,6:9]=-mid@skew(accel);f[3:6,12:15]=-mid
    f[6:9,6:9]=-skew(omega);f[6:9,9:12]=-np.eye(3)
    squared=f@f;identity=np.eye(15)
    def transition(t):return identity+f*t+.5*squared*t*t
    q=np.zeros((15,15))
    for a,b,density in ((3,6,noise[1]),(6,9,noise[0]),(9,12,noise[2]),(12,15,noise[3])):
        q[a:b,a:b]=np.eye(3)*density**2
    discrete=np.zeros((15,15))
    for node,weight in ((-math.sqrt(3/5),5/9),(0.,8/9),(math.sqrt(3/5),5/9)):
        transport=transition(.5*dt*(node+1))
        discrete+=.5*dt*weight*(transport@q@transport.T)
    return transition(dt),discrete


class ImuWindow:
    """Up to 8192 samples (8.19 seconds at 1 kHz), with strict coverage.

    Sensor timestamps must increase. Invalid input clears the entire window;
    callers must also invalidate their fused segment. Endpoint interpolation
    requires real bracketing samples and never extrapolates through a gap.
    """
    def __init__(self,*,max_gap=.02):
        if type(max_gap) not in (int,float) or not math.isfinite(max_gap) or not 0<max_gap<=.1:
            raise ValueError('invalid_imu_max_gap')
        self.max_gap=float(max_gap)
        self.samples=deque(maxlen=8192)

    def clear(self):self.samples.clear()

    def add(self,stamp,angular_velocity,specific_force):
        try:
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:
                raise ValueError('invalid_imu_stamp')
            gyro,accel=vector(angular_velocity).copy(),vector(specific_force).copy()
            if np.linalg.norm(gyro)>35 or np.linalg.norm(accel)>200:
                raise ValueError('imu_saturated')
            if self.samples and stamp<=self.samples[-1][0]:raise ValueError('imu_time_reversal')
            if self.samples and stamp-self.samples[-1][0]>self.max_gap+1e-9:
                raise ValueError('imu_time_gap')
            self.samples.append((float(stamp),gyro,accel))
        except (ValueError,TypeError):
            self.clear()
            raise

    def interval(self,start,end):
        if (any(type(t) not in (int,float) or not math.isfinite(t) for t in (start,end))
                or start<0 or not 0<end-start<=3):raise ValueError('invalid_imu_interval')
        if not self.samples or self.samples[0][0]>start or self.samples[-1][0]<end:
            raise ValueError('imu_interval_not_covered')
        times=np.array([s[0] for s in self.samples])
        # Take only bracketing samples and interior knots, keeping work bounded
        # by the requested interval rather than the full retained window.
        low=max(0,int(np.searchsorted(times,start,side='right'))-1)
        high=min(len(times),int(np.searchsorted(times,end,side='left'))+1)
        subset=list(self.samples)[low:high];times=times[low:high]
        knots=np.array([start]+[float(t) for t in times if start<t<end]+[end])
        values=np.array([np.r_[s[1],s[2]] for s in subset])
        return knots,np.column_stack([np.interp(knots,times,values[:,axis]) for axis in range(6)])

    def preintegrate(self,start,end,*,gyro_bias,accel_bias,noise_density):
        """Midpoint deltas, 15-state covariance and 9x6 bias Jacobian.

        noise_density = gyro(rad/s/sqrt(Hz)), accel(m/s²/sqrt(Hz)), gyro
        bias random walk(rad/s²/sqrt(Hz)), accel bias random walk(m/s³/sqrt(Hz)).
        No default noise is invented when Gazebo publishes zero covariance.
        Bias corrections are first order; re-integrate for large bias updates.
        """
        bg,ba=vector(gyro_bias),vector(accel_bias)
        noise=np.asarray(noise_density,dtype=float)
        if noise.shape!=(4,) or not np.isfinite(noise).all() or np.any(noise<=0):
            raise ValueError('imu_noise_calibration_required')
        knots,values=self.interval(start,end)
        attitude=np.eye(3);velocity=np.zeros(3);position=np.zeros(3)
        covariance=np.zeros((15,15));jacobian=np.zeros((9,6))
        for i,dt in enumerate(np.diff(knots)):
            omega=(values[i,:3]+values[i+1,:3])*.5-bg
            accel=(values[i,3:]+values[i+1,3:])*.5-ba
            mid=attitude@exp_so3(omega*dt*.5)
            specific=mid@accel
            position+=velocity*dt+.5*specific*dt**2
            velocity+=specific*dt
            phi,discrete=error_transition(mid,omega,accel,dt,noise)
            covariance=phi@covariance@phi.T+discrete
            jacobian=phi[:9,:9]@jacobian+phi[:9,9:15]
            attitude=attitude@exp_so3(omega*dt)
        return {'start':start,'end':end,'dt':end-start,'rotation':attitude,
                'delta_velocity':velocity,'delta_position':position,
                'covariance':(covariance+covariance.T)*.5,'bias_jacobian':jacobian,
                'gyro_bias':bg.copy(),'accel_bias':ba.copy(),'gravity_included':False}

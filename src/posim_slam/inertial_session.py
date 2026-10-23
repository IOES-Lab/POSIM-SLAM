"""Bounded streaming IMU/DVL owner; independent of ROS and simulator state."""
from collections import deque
import math
import numpy as np
from .inertial import ImuWindow
from .inertial_alignment import moving_initialize
from .inertial_filter import covariance
from .dvl_scale import vector


class InertialDvlSession:
    """12-second moving initialization followed by timestamped DVL/IMU fusion.

    Output is an IMU pose at a measured DVL time, not extrapolated display time.
    No stationary test, orientation input, map or navigation-ready claim.
    The transport owner validates session/source identities before submitting.
    """
    def __init__(self,calibration,*,attitude_limit_deg=3.,velocity_limit_m_s=.1):
        self.calibration=calibration
        if (not math.isfinite(attitude_limit_deg) or not 0<attitude_limit_deg<=30 or
                not math.isfinite(velocity_limit_m_s) or not 0<velocity_limit_m_s<=1):
            raise ValueError('invalid_initialization_quality_limits')
        self.limits=(attitude_limit_deg,velocity_limit_m_s)
        self.window=ImuWindow(max_samples=32768)
        self.dvl_samples=deque(maxlen=2048)
        self.segment=0;self.state='initializing';self.reason='waiting_inputs'
        self.filter=None;self.diagnostics=None;self.last_attempt=None
        self.last_imu_wall=None;self.last_dvl_wall=None;self.last_dvl_stamp=None
        self.accepted=0;self.last_update=None

    def invalidate(self,reason):
        if self.state!='lost' or self.filter is not None:self.segment+=1
        self.state='lost';self.reason=reason;self.filter=None;self.diagnostics=None
        self.window.clear();self.dvl_samples.clear();self.last_attempt=None
        self.last_imu_wall=None;self.last_dvl_wall=None;self.last_dvl_stamp=None
        self.accepted=0;self.last_update=None

    def imu(self,stamp,gyro,force,now):
        try:
            self.window.add(stamp,gyro,force);self.last_imu_wall=float(now)
            self.flush()
        except (ValueError,TypeError,np.linalg.LinAlgError) as error:self.invalidate(str(error))

    def dvl(self,stamp,velocity,uncertainty,locked,now):
        try:
            if locked is not True:raise ValueError('dvl_bottom_lock_lost')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:
                raise ValueError('invalid_dvl_stamp')
            if self.last_dvl_stamp is not None and not 0<stamp-self.last_dvl_stamp<=1.5:
                raise ValueError('dvl_time_gap_or_reversal')
            sample=(float(stamp),vector(velocity).copy(),covariance(uncertainty,3),True)
            self.last_dvl_stamp=stamp;self.last_dvl_wall=float(now)
            self.dvl_samples.append(sample);self.flush()
        except (ValueError,TypeError,np.linalg.LinAlgError) as error:self.invalidate(str(error))

    def flush(self):
        if not self.window.samples or not self.dvl_samples:return
        first,last=self.window.samples[0][0],self.window.samples[-1][0]
        if self.filter is None:
            candidates=[s for s in self.dvl_samples if first<=s[0]<=last]
            if not candidates:return
            end=candidates[-1][0]
            # Select actual timestamps spanning >=12 s; never synthesize DVL.
            candidates=[s for s in candidates if end-s[0]<=13.5]
            if len(candidates)<13 or end-candidates[0][0]<12-1e-9:
                self.state='initializing';self.reason='collecting_moving_window';return
            if self.last_attempt==end:return
            self.last_attempt=end
            # Bound graph size independently of a faster DVL publishing rate.
            indices=np.linspace(0,len(candidates)-1,min(24,len(candidates)),dtype=int)
            selected=[candidates[i] for i in indices]
            try:
                estimator,diagnostics=moving_initialize(self.window,selected,**self.calibration)
            except ValueError as error:
                self.state='initializing';self.reason=str(error);return
            self.diagnostics=diagnostics
            if (diagnostics['attitude_three_sigma_deg']>self.limits[0] or
                    diagnostics['velocity_three_sigma_m_s']>self.limits[1]):
                self.state='initializing';self.reason='initial_uncertainty_too_large';return
            self.filter=estimator;self.state='tracking';self.reason='moving_initialized'
        for stamp,velocity,uncertainty,_ in self.dvl_samples:
            if stamp<=self.filter.stamp:continue
            if stamp>last:break
            self.filter.propagate(self.window,stamp)
            knots,raw=self.window.interval(max(first,stamp-.01),stamp)
            noise=self.calibration['noise_density'][0]**2/float(np.min(np.diff(knots)))
            result=self.filter.update_dvl(velocity,uncertainty,stamp=stamp,gyro=raw[-1,:3],
                dvl_to_imu=self.calibration['dvl_to_imu'],imu_to_dvl=self.calibration['imu_to_dvl'],
                gyro_covariance=np.eye(3)*noise)
            if not self.filter.valid:raise ValueError(self.filter.invalid_reason)
            self.last_update=result;self.accepted+=int(result['accepted'])
            self.reason='fused' if result['accepted'] else 'dvl_innovation_rejected'

    def tick(self,now):
        if self.last_imu_wall is not None and now-self.last_imu_wall>2:
            self.invalidate('imu_stale')
        elif self.last_dvl_wall is not None and now-self.last_dvl_wall>3:
            self.invalidate('dvl_stale')

    def snapshot(self):
        f=self.filter
        return dict(state=self.state,reason=self.reason,segment=self.segment,
            stamp=f.stamp if f else None,initialization=self.diagnostics,
            accepted_dvl_updates=self.accepted,inertial_fused=f is not None,
            navigation_ready=False,orientation_measurement_used=False,
            position=f.position.tolist() if f else None,
            attitude=f.attitude.tolist() if f else None,
            covariance=f.covariance.tolist() if f else None)

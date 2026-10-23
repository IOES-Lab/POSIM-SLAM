"""Raw-IMU error-state filter with velocity and cloned relative-pose updates.

State errors are world position/velocity, right IMU attitude, gyro bias and
accelerometer bias. A cloned pose retains correlation for relative observations.
Initialization and all uncertainty values must be supplied explicitly; this
module accepts neither simulator messages nor ideal IMU orientation.
"""
import math

ALGORITHM_VERSION="0.1.0"
import numpy as np
from .inertial import exp_so3,skew,error_transition
from .dvl_scale import rotation,vector


def covariance(value,size,*,allow_singular=False):
    result=np.asarray(value,dtype=float)
    if (result.shape!=(size,size) or not np.isfinite(result).all() or
        not np.allclose(result,result.T,atol=1e-10,rtol=0) or (np.linalg.eigvalsh(result).min() < -1e-12*np.linalg.norm(result,2)
            if allow_singular else np.linalg.eigvalsh(result).min()<=0)):
        raise ValueError('positive_covariance_required')
    return result.copy()


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


class InertialFilter:
    """One local segment; the caller invalidates it on sensor/clock loss."""
    def __init__(self,*,stamp,position,velocity,attitude,gyro_bias,accel_bias,
                 state_covariance,noise_density,gravity=(0.,0.,-9.80665)):
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:raise ValueError('invalid_state_stamp')
        self.stamp=float(stamp);self.position=vector(position).copy();self.velocity=vector(velocity).copy()
        self.attitude=rotation(attitude).copy();self.gyro_bias=vector(gyro_bias).copy();self.accel_bias=vector(accel_bias).copy()
        self.covariance=covariance(state_covariance,15,allow_singular=True)
        # Calibrated bias directions may be exact; pose/velocity uncertainty
        # must still be supplied rather than invented as an all-zero state.
        covariance(self.covariance[:9,:9],9)
        self.gravity=vector(gravity).copy()
        self.noise=np.array(noise_density,dtype=float,copy=True)
        if self.noise.shape!=(4,) or not np.isfinite(self.noise).all() or np.any(self.noise<=0):
            raise ValueError('imu_noise_calibration_required')
        self.anchor=None;self.updates=dict(dvl=0,relative=0,depth=0);self.last_innovation=None
        self.observed={};self.rejected={};self.valid=True;self.invalid_reason=None

    def _usable(self):
        if not self.valid:raise ValueError('inertial_segment_invalid:'+str(self.invalid_reason))

    def invalidate(self,reason):
        self.valid=False;self.invalid_reason=str(reason);self.anchor=None

    def propagate(self,window,end):
        self._usable()
        if end==self.stamp:return
        try:knots,values=window.interval(self.stamp,end)
        except (ValueError,TypeError) as error:
            self.invalidate(str(error));raise
        for index,dt in enumerate(np.diff(knots)):
            gyro=(values[index,:3]+values[index+1,:3])*.5-self.gyro_bias
            accel=(values[index,3:]+values[index+1,3:])*.5-self.accel_bias
            mid=self.attitude@exp_so3(gyro*dt*.5)
            acceleration=mid@accel+self.gravity
            self.position+=self.velocity*dt+.5*acceleration*dt*dt
            self.velocity+=acceleration*dt
            phi,discrete=error_transition(mid,gyro,accel,dt,self.noise)
            transition=np.eye(len(self.covariance));transition[:15,:15]=phi
            self.covariance=transition@self.covariance@transition.T
            self.covariance[:15,:15]+=discrete
            self.covariance=(self.covariance+self.covariance.T)*.5
            self.attitude=self.attitude@exp_so3(gyro*dt)
        self.stamp=float(end)

    def clone_pose(self):
        self._usable()
        p=self.covariance[:15,:15].copy();j=np.zeros((6,15));j[:3,:3]=np.eye(3);j[3:,6:9]=np.eye(3)
        self.covariance=np.block([[p,p@j.T],[j@p,j@p@j.T]])
        self.anchor=(self.stamp,self.position.copy(),self.attitude.copy())

    def _observation(self,kind,stamp):
        self._usable()
        # Measurements must already be synchronized to the propagated state.
        # Delayed data require replay by the owner, never an arrival-time update.
        if (type(stamp) not in (int,float) or not math.isfinite(stamp) or
                abs(stamp-self.stamp)>1e-9):raise ValueError('measurement_state_time_mismatch')
        if stamp<=self.observed.get(kind,-1):raise ValueError('measurement_already_consumed')
        self.observed[kind]=stamp

    def _correct(self,residual,h,measurement_covariance,*,gate,kind):
        r=np.asarray(residual,dtype=float)
        if r.ndim!=1 or not np.isfinite(r).all():raise ValueError('invalid_residual')
        noise=covariance(measurement_covariance,len(r))
        innovation=h@self.covariance@h.T+noise
        nis=float(r@np.linalg.solve(innovation,r))
        if not math.isfinite(nis) or not math.isfinite(gate) or gate<=0:raise ValueError('invalid_innovation')
        self.last_innovation=nis
        if nis>gate:
            self.rejected[kind]=self.rejected.get(kind,0)+1
            if self.rejected[kind]>=3:self.invalidate(kind+'_innovation_rejected')
            return dict(accepted=False,nis=nis,segment_valid=self.valid)
        self.rejected[kind]=0
        gain=np.linalg.solve(innovation,h@self.covariance).T;delta=gain@r
        identity=np.eye(len(self.covariance));a=identity-gain@h
        self.covariance=a@self.covariance@a.T+gain@noise@gain.T
        self.position+=delta[:3];self.velocity+=delta[3:6]
        self.attitude=self.attitude@exp_so3(delta[6:9]);self.gyro_bias+=delta[9:12];self.accel_bias+=delta[12:15]
        reset=identity.copy();reset[6:9,6:9]=np.eye(3)-.5*skew(delta[6:9])
        if self.anchor is not None:
            t,p,r=self.anchor;self.anchor=(t,p+delta[15:18],r@exp_so3(delta[18:21]))
            reset[18:21,18:21]=np.eye(3)-.5*skew(delta[18:21])
        self.covariance=reset@self.covariance@reset.T
        self.covariance=(self.covariance+self.covariance.T)*.5
        return dict(accepted=True,nis=nis,segment_valid=self.valid)

    def update_dvl(self,velocity,measurement_covariance,*,stamp,gyro,dvl_to_imu,imu_to_dvl,gyro_covariance,gate=11.34486673):
        r=rotation(dvl_to_imu);lever=vector(imu_to_dvl);omega=vector(gyro)-self.gyro_bias
        body_velocity=self.attitude.T@self.velocity
        predicted=r.T@(body_velocity+np.cross(omega,lever))
        h=np.zeros((3,len(self.covariance)))
        h[:,3:6]=r.T@self.attitude.T;h[:,6:9]=r.T@skew(body_velocity);h[:,9:12]=r.T@skew(lever)
        gyro_noise=covariance(gyro_covariance,3)
        noise=covariance(measurement_covariance,3)+r.T@skew(lever)@gyro_noise@skew(lever).T@r
        residual=vector(velocity)-predicted
        self._observation('dvl',stamp)
        result=self._correct(residual,h,noise,gate=gate,kind='dvl')
        if result['accepted']:self.updates['dvl']+=1
        return result

    def relative_prediction(self,sonar_to_imu,imu_to_sonar,error=None):
        if self.anchor is None:raise ValueError('relative_anchor_required')
        r=rotation(sonar_to_imu);lever=vector(imu_to_sonar)
        error=np.zeros(21) if error is None else error
        _,p0,r0=self.anchor
        p=self.position+error[:3];current=self.attitude@exp_so3(error[6:9])
        p0=p0+error[15:18];r0=r0@exp_so3(error[18:21])
        anchor=r0@r
        translation=anchor.T@(p+current@lever-p0-r0@lever)
        attitude=anchor.T@current@r
        return translation,attitude

    def update_relative_planar(self,translation,yaw,measurement_covariance,*,stamp,sonar_to_imu,imu_to_sonar,
                               max_normal_translation=.15,max_tilt=.1,gate=11.34486673):
        if (not math.isfinite(max_normal_translation) or max_normal_translation<=0 or
                not math.isfinite(max_tilt) or not 0<max_tilt<math.pi/2):
            raise ValueError('invalid_planar_limits')
        measured=np.asarray(translation,dtype=float)
        if measured.shape!=(2,) or not np.isfinite(measured).all() or not math.isfinite(yaw):raise ValueError('invalid_planar_measurement')
        def predict(error=None):
            p,r=self.relative_prediction(sonar_to_imu,imu_to_sonar,error)
            return np.array([p[0],p[1],math.atan2(r[1,0],r[0,0])])
        p,r=self.relative_prediction(sonar_to_imu,imu_to_sonar)
        if abs(p[2])>max_normal_translation or math.acos(float(np.clip(r[2,2],-1,1)))>max_tilt:
            raise ValueError('planar_motion_not_supported')
        base=predict();h=np.empty((3,21));epsilon=1e-6
        for i in range(21):
            error=np.zeros(21);error[i]=epsilon
            plus=predict(error);minus=predict(-error);difference=plus-minus;difference[2]=wrap(difference[2])
            h[:,i]=difference/(2*epsilon)
        residual=np.r_[measured-base[:2],wrap(yaw-base[2])]
        self._observation('relative',stamp)
        result=self._correct(residual,h,measurement_covariance,gate=gate,kind='relative')
        if result['accepted']:self.updates['relative']+=1
        return result

    def update_depth(self,sensor_z,variance,*,stamp,imu_to_pressure,gate=6.634896601):
        """Calibrated pressure sensor ENU height, including its physical offset.

        The owner supplies pressure-to-height calibration and its uncertainty.
        This is not a Ping range observation or automatic sea-level calibration.
        """
        if not math.isfinite(sensor_z) or not math.isfinite(variance) or variance<=0:
            raise ValueError('invalid_depth_observation')
        lever=vector(imu_to_pressure);vertical=np.array([0.,0.,1.])
        predicted=float((self.position+self.attitude@lever)[2])
        h=np.zeros((1,len(self.covariance)));h[0,2]=1
        h[0,6:9]=-vertical@self.attitude@skew(lever)
        self._observation('depth',stamp)
        result=self._correct([sensor_z-predicted],h,[[variance]],gate=gate,kind='depth')
        if result['accepted']:self.updates['depth']+=1
        return result

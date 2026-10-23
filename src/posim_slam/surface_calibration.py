"""Estimate a metric-map water plane from synchronized pressure and attitude.

IMU orientation is sensor-to-ENU. Inputs are measurements, not simulator poses.
Camera-to-pressure lever arm and IMU-to-camera rotation are fixed calibration.
"""
from collections import deque
import math
import numpy as np


class SurfaceCalibration:
    def __init__(self, imu_to_camera, camera_to_pressure, reference_pa=101325., density=1025.):
        self.rotation=np.asarray(imu_to_camera,dtype=float)
        self.offset=np.asarray(camera_to_pressure,dtype=float)
        if (self.rotation.shape!=(3,3) or self.offset.shape!=(3,) or
            not np.isfinite(self.rotation).all() or not np.isfinite(self.offset).all() or
            not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=1e-6) or
            not math.isclose(np.linalg.det(self.rotation),1.,abs_tol=1e-6)):
            raise ValueError('invalid_surface_extrinsics')
        if not 50000<reference_pa<150000 or not 900<density<1200:
            raise ValueError('invalid_surface_pressure_reference')
        self.reference_pa,self.density=reference_pa,density
        self.reset()

    def reset(self):
        self.samples=deque(maxlen=8);self.last_stamp=None;self.plane=None

    def add(self,t,origin,rotation,pressure_t,pressure_pa,pressure_variance,imu_t,quaternion):
        origin=np.asarray(origin,dtype=float);rotation=np.asarray(rotation,dtype=float)
        q=np.asarray(quaternion,dtype=float)
        values=[t,pressure_t,imu_t,pressure_pa,pressure_variance]
        if (not np.isfinite(values).all() or min(t,pressure_t,imu_t)<0 or
            abs(t-pressure_t)>.1 or abs(t-imu_t)>.1 or
            pressure_variance<0 or pressure_variance>10000 or
            origin.shape!=(3,) or rotation.shape!=(3,3) or q.shape!=(4,) or
            not np.isfinite(origin).all() or not np.isfinite(rotation).all() or not np.isfinite(q).all() or
            abs(np.linalg.norm(q)-1)>.001 or
            not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or
            not math.isclose(np.linalg.det(rotation),1.,abs_tol=1e-6)):
            self.reset();raise ValueError('invalid_surface_measurement')
        if self.last_stamp is not None and (t<=self.last_stamp or t-self.last_stamp>1.):
            self.reset()
        self.last_stamp=t
        depth=(pressure_pa-self.reference_pa)/(self.density*9.80665)
        if not 0<=depth<=300:
            self.reset();raise ValueError('invalid_surface_depth')
        x,y,z,w=q/np.linalg.norm(q)
        # Third row of R(ENU <- IMU) gives world up in the IMU frame.
        up_imu=np.array([2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)])
        normal=rotation@self.rotation@up_imu
        normal/=np.linalg.norm(normal)
        pressure_position=origin+rotation@self.offset
        plane=np.r_[normal,-depth-normal@pressure_position]
        self.samples.append(plane)
        if len(self.samples)<8:self.plane=None;return None
        samples=np.asarray(self.samples)
        mean_normal=samples[:,:3].mean(axis=0);mean_normal/=np.linalg.norm(mean_normal)
        # Require stable orientation and height; waves/outliers restart acquisition.
        if (np.min(samples[:,:3]@mean_normal)<math.cos(math.radians(3)) or
            np.ptp(samples[:,3])>.1):
            self.reset();self.samples.append(plane);self.last_stamp=t;return None
        candidate=np.r_[mean_normal,np.median(samples[:,3])]
        if self.plane is not None:
            if (self.plane[:3]@candidate[:3]<math.cos(math.radians(3)) or
                abs(self.plane[3]-candidate[3])>.1):
                self.reset();self.samples.append(plane);self.last_stamp=t;return None
        else:self.plane=candidate
        # Hold a calibrated plane through ordinary sensor noise so full-map
        # exports use the same plane as the live map. Drift triggers reacquisition.
        return self.plane.copy()

"""Metric scale from synchronized visual camera poses and bottom-locked DVL.

This estimator uses no simulator pose. Poses are camera-to-map transforms with
arbitrary translation scale. DVL velocities are measured at the sensor origin.
The caller must reset on session/map replacement and tracking loss.
"""
from collections import deque
import math
import numpy as np

# Pinned Orca camera: Ry(pi/2), sensor translation (0,0,-.1).
# Gazebo camera axes FLU -> optical RDF. DVL axes match base_link.
BODY_TO_OPTICAL = np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
CAMERA_TO_DVL = BODY_TO_OPTICAL @ np.array([.1,0.,-.12])


def rotation(value):
    value=np.asarray(value,dtype=float)
    if (value.shape!=(3,3) or not np.isfinite(value).all()
        or not np.allclose(value.T@value,np.eye(3),atol=1e-5)
        or abs(np.linalg.det(value)-1)>1e-5):raise ValueError('invalid_rotation')
    return value


def vector(value):
    value=np.asarray(value,dtype=float)
    if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError('invalid_vector')
    return value


class DvlScaleWindow:
    """Bounded 1 Hz integration; require eight consistent moving intervals."""
    def __init__(self,velocity_to_camera=BODY_TO_OPTICAL,camera_to_dvl=CAMERA_TO_DVL):
        self.velocity_to_camera=rotation(velocity_to_camera).copy()
        self.camera_to_dvl=vector(camera_to_dvl).copy()
        self.values=deque(maxlen=15)
        self.previous=None
    def reset(self):
        self.values.clear();self.previous=None
    def add(self, stamp, position, camera_to_map, dvl_stamp, velocity, covariance, *, valid=True):
        try:return self._add(stamp,position,camera_to_map,dvl_stamp,velocity,covariance,valid)
        except (ValueError,TypeError):
            self.reset()
            raise
    def _add(self,stamp,position,camera_to_map,dvl_stamp,velocity,covariance,valid):
        if not valid:raise ValueError('dvl_tracking_lost')
        if not math.isfinite(stamp) or not math.isfinite(dvl_stamp) or min(stamp,dvl_stamp)<0:raise ValueError('invalid_timestamp')
        if abs(stamp-dvl_stamp)>.1+1e-6:raise ValueError('dvl_not_synchronized')
        p=vector(position);r=rotation(camera_to_map);v=vector(velocity)
        cov=np.asarray(covariance,dtype=float)
        if cov.size!=9:raise ValueError('invalid_covariance')
        cov=cov.reshape(3,3)
        if (not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-10)
            or np.linalg.eigvalsh(cov).min()<-1e-10 or np.linalg.eigvalsh(cov).max()>.01):raise ValueError('invalid_covariance')
        current=(stamp,dvl_stamp,p.copy(),r.copy(),r@self.velocity_to_camera@v)
        previous=self.previous
        self.previous=current
        if previous is None:return {'ready':False,'observations':0,'scale':None,'reason':'waiting_motion'}
        t0,d0,p0,r0,v0=previous
        dt=dvl_stamp-d0
        if stamp<=t0 or not .5<=dt<=1.5 or abs((stamp-t0)-dt)>.02:raise ValueError('dvl_time_gap')
        angle=math.acos(float(np.clip((np.trace(r0.T@r)-1)/2,-1,1)))
        if angle>math.radians(15):raise ValueError('dvl_rotation_too_fast')
        # Integral of sensor velocity minus change in the rotating camera/DVL
        # baseline gives camera displacement. The baseline is already in metres.
        metric=.5*(v0+current[4])*dt-(r-r0)@self.camera_to_dvl
        visual=p-p0
        length=float(np.linalg.norm(metric));denominator=float(visual@visual)
        if length<.05 or denominator<.005**2:
            self.values.clear()
            return {'ready':False,'observations':0,'scale':None,'reason':'insufficient_motion'}
        scale=float(visual@metric/denominator)
        if not 1e-4<scale<1e4:raise ValueError('invalid_scale')
        residual=float(np.linalg.norm(scale*visual-metric))
        if residual>max(.01,.1*length):raise ValueError('dvl_visual_disagreement')
        self.values.append(scale)
        values=np.asarray(self.values);median=float(np.median(values))
        spread=float(np.max(np.abs(values-median))/median)
        ready=len(values)>=8 and spread<=.1
        return {'ready':ready,'observations':len(values),'scale':median,'relative_spread':spread,
                'residual_m':residual,'reason':'ready' if ready else 'waiting_scale'}

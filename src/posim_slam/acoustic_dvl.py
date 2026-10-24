"""Experimental interval translation fusion in the acoustic sensor fan plane.

DVL velocities are measured at the DVL origin. Acoustic registration supplies
rotation; linear interpolation of that rotation is a planar motion assumption.
No simulator pose, elevation, probabilistic confidence or obstacle identity is
inferred. The caller must break its map segment on rejected/lost input and bound
how long it waits for a DVL sample covering an acoustic interval.
"""
from collections import deque
import math
import numpy as np
from .dvl_scale import rotation, vector


def planar_rotation(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s],[s,c]])


class AcousticDvlIntervals:
    """32 measured samples; DVL translation with acoustic rotation and matching.

    Covariance is a quality gate, not an advertised fused pose covariance.
    Future adapters must call clear() on DVL lock loss/session replacement.
    """
    def __init__(self,velocity_to_sonar,sonar_to_dvl,*,max_normal_excursion_m=.05):
        if not math.isfinite(max_normal_excursion_m) or max_normal_excursion_m<=0:
            raise ValueError('invalid_normal_excursion_limit')
        self.max_normal_excursion_m=float(max_normal_excursion_m)
        self.rotation=rotation(velocity_to_sonar).copy()
        self.baseline=vector(sonar_to_dvl).copy()[:2]
        self.samples=deque(maxlen=32)

    def clear(self):
        self.samples.clear()

    def add(self,stamp,velocity,covariance,*,valid=True):
        try:
            if not valid:raise ValueError('dvl_tracking_lost')
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<0:
                raise ValueError('invalid_dvl_timestamp')
            v=vector(velocity)
            cov=np.asarray(covariance,dtype=float)
            if cov.size!=9:raise ValueError('invalid_dvl_covariance')
            cov=cov.reshape(3,3)
            if (not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-10)
                    or np.linalg.eigvalsh(cov).min()<-1e-10 or np.linalg.eigvalsh(cov).max()>.01):
                raise ValueError('invalid_dvl_covariance')
            if self.samples and stamp<=self.samples[-1][0]:raise ValueError('dvl_time_reversal')
            coverage_reset=bool(self.samples and stamp-self.samples[-1][0]>1.5)
            if coverage_reset:
                # Keep the new sample as the start of a fresh coverage window.
                self.clear()
            self.samples.append((float(stamp),(self.rotation@v).copy()))
            return {'coverage_reset':coverage_reset}
        except (ValueError,TypeError):
            self.clear()
            raise

    def fuse(self,start,end,translation,yaw):
        """Return an interval update, or waiting without inventing DVL motion.

        Input/output translations are current-to-anchor in metres. All DVL
        vectors are rotated into that anchor's fan plane before integration.
        """
        if (any(type(t) not in (int,float) or not math.isfinite(t) for t in (start,end,yaw))
                or start<0 or not 0<end-start<=3):raise ValueError('invalid_acoustic_interval')
        acoustic=np.asarray(translation,dtype=float)
        if acoustic.shape!=(2,) or not np.isfinite(acoustic).all():raise ValueError('invalid_acoustic_translation')
        if abs(yaw)>math.radians(15):raise ValueError('dvl_rotation_too_fast')
        if not self.samples or self.samples[-1][0]<end:
            return {'state':'waiting_dvl'}
        if self.samples[0][0]>start:raise ValueError('dvl_interval_not_covered')
        times=np.array([s[0] for s in self.samples])
        velocities=np.array([s[1] for s in self.samples])
        # Insert both acoustic boundaries; np.interp cannot extrapolate here.
        knots=np.array([start]+[t for t in times if start<t<end]+[end])
        values=np.column_stack([np.interp(knots,times,velocities[:,axis]) for axis in (0,1,2)])
        # A planar yaw leaves the fan normal invariant. Keep the measured normal
        # motion instead of silently dropping it during the 2-D projection.
        normal=0.;excursion=0.
        for dt,v0,v1 in zip(np.diff(knots),values[:-1,2],values[1:,2]):
            if v0*v1<0:
                # A reversal can leave zero net motion while leaving the plane.
                tau=-v0*dt/(v1-v0)
                excursion=max(excursion,abs(normal+v0*tau+.5*(v1-v0)/dt*tau*tau))
            normal+=.5*(v0+v1)*dt
            excursion=max(excursion,abs(normal))
        if excursion>self.max_normal_excursion_m:
            raise ValueError('dvl_motion_outside_sonar_plane')
        angles=yaw*(knots-start)/(end-start)
        rotated=np.array([planar_rotation(a)@v[:2] for a,v in zip(angles,values)])
        dvl=np.sum((rotated[:-1]+rotated[1:])*.5*np.diff(knots)[:,None],axis=0)
        # DVL-origin translation = sonar translation + change of lever arm.
        dvl+=self.baseline-planar_rotation(yaw)@self.baseline
        residual=float(np.linalg.norm(acoustic-dvl))
        if not np.isfinite(dvl).all() or not math.isfinite(residual):
            raise ValueError('invalid_integrated_motion')
        # Explicit experimental consistency threshold, not a confidence bound.
        if residual>max(.15,.2*max(np.linalg.norm(acoustic),np.linalg.norm(dvl))):
            raise ValueError('acoustic_dvl_disagreement')
        return {'state':'tracking','anchor_stamp':start,'stamp':end,
                'translation':dvl.tolist(),'yaw_rad':yaw,
                'acoustic_translation_m':acoustic.tolist(),'dvl_translation_m':dvl.tolist(),
                'normal_displacement_m':float(normal),'normal_excursion_m':float(excursion),
                'max_normal_excursion_m':self.max_normal_excursion_m,
                'residual_m':residual,'dvl_weight':1.,'translation_source':'bottom_locked_dvl','elevation_resolved':False}

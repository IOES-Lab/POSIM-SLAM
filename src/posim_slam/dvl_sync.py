"""Bounded interpolation of visual camera poses at DVL simulation timestamps."""
import math
import numpy as np
from .dvl_scale import rotation, vector


class PoseBuffer:
    def __init__(self):self.values={};self.last=None
    def clear(self):self.values.clear();self.last=None
    def add(self,stamp,position,orientation):
        if not math.isfinite(stamp) or stamp<0:raise ValueError('invalid_timestamp')
        if self.last is not None and stamp<=self.last:
            self.clear();raise ValueError('visual_time_reversal')
        self.values[stamp]=(vector(position).copy(),rotation(orientation).copy());self.last=stamp
        while len(self.values)>30:del self.values[min(self.values)]
    def at(self,stamp):
        if stamp in self.values:return self.values[stamp]
        # Native sensors can differ by one 1 ms physics step. Match an observed
        # pose within two steps before requiring interpolation; never predict
        # motion beyond an observed pose or bridge a larger missing interval.
        nearest=min(self.values,key=lambda t:abs(t-stamp)) if self.values else None
        if nearest is not None and abs(nearest-stamp)<=.002+1e-9:
            return self.values[nearest]
        before=[t for t in self.values if t<stamp];after=[t for t in self.values if t>stamp]
        if not before or not after:return None
        a,b=max(before),min(after)
        if b-a>.3 or stamp-a>.2 or b-stamp>.2:raise ValueError('visual_interpolation_gap')
        p0,r0=self.values[a];p1,r1=self.values[b];f=(stamp-a)/(b-a)
        delta=r0.T@r1;angle=math.acos(float(np.clip((np.trace(delta)-1)/2,-1,1)))
        if angle>math.radians(15):raise ValueError('visual_rotation_too_fast')
        if angle<1e-8:r=r0
        else:
            skew=(delta-delta.T)/(2*math.sin(angle))
            r=r0@(np.eye(3)+math.sin(f*angle)*skew+(1-math.cos(f*angle))*(skew@skew))
        return p0+f*(p1-p0),r

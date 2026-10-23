"""Bounded streaming planar odometry; segment breaks never invent continuity.

This core does not publish a map or infer elevation. Only two acoustic frames
are needed per update. A caller must explicitly handle segment changes.
"""
import numpy as np
from .acoustic_flow import estimate,validate_frame


class AcousticStream:
    def __init__(self, *, frame_step=4, max_gap_seconds=1.25):
        if type(frame_step) is not int or frame_step not in (1,2,4):raise ValueError('invalid_frame_step')
        if not np.isfinite(max_gap_seconds) or max_gap_seconds<=0:raise ValueError('invalid_max_gap')
        self.frame_step=frame_step;self.max_gap=max_gap_seconds;self.segment=0
        self.anchor=None;self.last_stamp=None;self.since_anchor=0
        self.position=np.zeros(2);self.yaw=0.

    def reset(self,reason='reset'):
        self.segment+=1;self.anchor=None;self.last_stamp=None;self.since_anchor=0
        self.position=np.zeros(2);self.yaw=0.
        return {'state':'lost','reason':reason,'segment':self.segment}

    def push(self,frame):
        try:validate_frame(frame)
        except (ValueError,KeyError,TypeError):return self.reset('invalid_acoustic_frame')
        stamp=float(frame['stamp'][0]+frame['stamp'][1]*1e-9)
        if self.last_stamp is not None:
            if stamp<=self.last_stamp:return self.reset('nonincreasing_timestamp')
            if stamp-self.last_stamp>self.max_gap:return self.reset('frame_gap')
        if self.anchor is not None and (not np.array_equal(frame['ranges'],self.anchor['ranges']) or
                not np.array_equal(frame['beam_directions'],self.anchor['beam_directions'])):
            return self.reset('calibration_changed')
        self.last_stamp=stamp
        if self.anchor is None:
            self.anchor={k:np.array(frame[k],copy=True) for k in ('intensity_db','ranges','beam_directions','stamp')}
            return {'state':'initializing','segment':self.segment,'stamp':stamp}
        self.since_anchor+=1
        if self.since_anchor<self.frame_step:
            return {'state':'waiting','segment':self.segment,'stamp':stamp}
        anchor_stamp=float(self.anchor['stamp'][0]+self.anchor['stamp'][1]*1e-9)
        result=estimate(self.anchor,frame,'raw')
        if not result['fitted']:return self.reset(result.get('reason','registration_failed'))
        delta=np.asarray(result['translation']);angle=result['yaw_rad']
        if delta.shape!=(2,) or not np.isfinite(delta).all() or not np.isfinite(angle):
            return self.reset('invalid_registration')
        c,s=np.cos(self.yaw),np.sin(self.yaw)
        self.position+=np.array([[c,-s],[s,c]])@delta;self.yaw+=angle
        self.anchor={k:np.array(frame[k],copy=True) for k in ('intensity_db','ranges','beam_directions','stamp')}
        self.since_anchor=0
        return {'state':'tracking','segment':self.segment,'stamp':stamp,'anchor_stamp':anchor_stamp,
                'position_fan_m':self.position.tolist(),'yaw_fan_rad':self.yaw,
                'registration':result}

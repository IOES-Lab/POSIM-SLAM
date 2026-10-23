"""Session-bound acoustic/DVL fusion lifecycle, independent of ROS transport."""
import copy
import math
import numpy as np
from .acoustic_dvl import AcousticDvlIntervals, planar_rotation
from .acoustic_map import AcousticMap


class AcousticDvlSession:
    def __init__(self,nonce,geometry,*,source='/vehicle/sensors/dvl',frame='vehicle/dvl'):
        self.nonce,self.source,self.frame=nonce,source,frame
        self.intervals=AcousticDvlIntervals(**geometry)
        self.feature_map=AcousticMap()
        self.enabled=False;self.segment=0;self.state='disabled';self.reason='disabled'
        self.position=np.zeros(2);self.yaw=0.;self.last_end=None
        self.pending=[];self.acoustic_segment=None
        self.last_acoustic=None;self.last_dvl=None;self.last_stamp=0.;self.not_before=0.

    def reset(self,reason,*,clear_dvl=True):
        self.segment+=1;self.position[:]=0;self.yaw=0.;self.last_end=None;self.pending.clear()
        self.not_before=max(self.not_before,self.last_stamp)
        if clear_dvl:self.intervals.clear();self.last_dvl=None
        self.state='lost' if self.enabled else 'disabled';self.reason=reason
        result={'state':self.state,'reason':reason,'segment':self.segment}
        self.feature_map.push(dict(result,state='lost'))
        return [result]

    def set_enabled(self,value):
        self.enabled=bool(value)
        return self.reset('waiting_inputs' if self.enabled else 'disabled')

    def dvl(self,value,now):
        if not self.enabled or value.get('session_nonce')!=self.nonce:return []
        try:
            if (value.get('source')!=self.source or value.get('frame_id')!=self.frame
                    or value.get('version')!='0.1.0'):raise ValueError('invalid_dvl_source')
            beams=value.get('locked_beams')
            if (value.get('valid') is not True or value.get('reason')!='bottom_locked'
                    or type(beams) is not int or not 3<=beams<=4):raise ValueError('dvl_tracking_lost')
            sec,ns=value['stamp']['sec'],value['stamp']['nanosec']
            if type(sec) is not int or type(ns) is not int or sec<0 or not 0<=ns<1_000_000_000:
                raise ValueError('invalid_dvl_stamp')
            t=sec+ns*1e-9
            info=self.intervals.add(t,value['velocity_m_s'],value['covariance'])
            self.last_dvl=now
            if info['coverage_reset']:
                self.not_before=max(self.not_before,t)
                return self.reset('dvl_time_gap',clear_dvl=False)
            return self.flush(now)
        except (ValueError,KeyError,TypeError) as error:return self.reset(str(error))

    def acoustic(self,value,now):
        if not self.enabled:return []
        self.last_acoustic=now
        segment=value.get('segment')
        if type(segment) is not int or segment<0:return self.reset('invalid_acoustic_segment')
        events=[]
        if self.acoustic_segment is not None and self.acoustic_segment!=segment:
            events=self.reset('acoustic_segment_changed',clear_dvl=False)
        self.acoustic_segment=segment
        stamp=value.get('stamp')
        if stamp is not None:
            if type(stamp) not in (int,float) or not math.isfinite(stamp) or stamp<self.last_stamp:
                return self.reset('invalid_acoustic_stamp')
            self.last_stamp=stamp
        state=value.get('state')
        if state in ('lost','error'):return self.reset('acoustic_tracking_lost',clear_dvl=False)
        if state in ('initializing','waiting'):return events
        if state!='tracking' or stamp is None:return self.reset('invalid_acoustic_state')
        try:
            start=value['anchor_stamp']
            if type(start) not in (int,float) or not math.isfinite(start) or not 0<=start<stamp:
                raise ValueError('invalid_acoustic_interval')
            if start<self.not_before-1e-6:return events
            if self.pending and start!=self.pending[-1][0]['stamp']:
                raise ValueError('acoustic_interval_gap')
            if len(self.pending)>=3:raise ValueError('dvl_pairing_overflow')
            self.pending.append((copy.deepcopy(value),now))
            return events+self.flush(now)
        except (ValueError,KeyError,TypeError) as error:return self.reset(str(error),clear_dvl=False)

    def flush(self,now):
        events=[]
        try:
            while self.pending:
                value,received=self.pending[0]
                if now-received>3:raise ValueError('dvl_pairing_timeout')
                fit=value['registration']
                result=self.intervals.fuse(value['anchor_stamp'],value['stamp'],fit['translation'],fit['yaw_rad'])
                if result['state']=='waiting_dvl':
                    self.state='waiting_dvl';self.reason='waiting_interval_coverage';break
                if self.last_end is not None and abs(value['anchor_stamp']-self.last_end)>1e-6:
                    raise ValueError('acoustic_interval_gap')
                position=self.position+planar_rotation(self.yaw)@np.array(result['translation'])
                yaw=self.yaw+result['yaw_rad']
                update=dict(value,position_fan_m=position.tolist(),yaw_fan_rad=yaw,
                            segment=self.segment,fusion=result)
                self.feature_map.push(update)
                self.position=position;self.yaw=yaw;self.last_end=value['stamp'];self.pending.pop(0)
                self.state='tracking';self.reason='fused';events.append(update)
        except (ValueError,KeyError,TypeError) as error:
            # Never emit accepted updates followed by a reset using a cleared map.
            return self.reset(str(error),clear_dvl=False)
        return events

    def tick(self,now):
        if not self.enabled:return []
        if self.last_acoustic is not None and now-self.last_acoustic>3:
            if self.reason!='acoustic_stale':return self.reset('acoustic_stale')
            return []
        if self.last_dvl is not None and now-self.last_dvl>3:
            return self.reset('dvl_stale')
        return self.flush(now)

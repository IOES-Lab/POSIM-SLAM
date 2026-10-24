"""Bounded persistent intensity tracks, without planar motion assumptions.

Track IDs identify optical-flow chains within a segment, not proven 3D objects.
The consumer must reject ambiguous returns and retain measurement correlation.
"""
import math
import numpy as np
from .acoustic_flow import validate_frame,corner_pixels,correspondences,polar_observations


class AcousticTracks:
    def __init__(self,*,max_tracks=300,max_gap_seconds=1.25):
        if type(max_tracks) is not int or not 1<=max_tracks<=300:
            raise ValueError('invalid_track_capacity')
        if not math.isfinite(max_gap_seconds) or max_gap_seconds<=0:
            raise ValueError('invalid_track_gap')
        self.capacity=max_tracks;self.max_gap_ns=int(max_gap_seconds*1e9)
        self.segment=0;self.next_id=0;self.previous=None;self.stamp=None
        self.pixels=np.empty((0,2),dtype=np.float32);self.ids=[];self.lengths=[]

    def reset(self,reason='reset'):
        self.segment+=1;self.next_id=0;self.previous=None;self.stamp=None
        self.pixels=np.empty((0,2),dtype=np.float32);self.ids=[];self.lengths=[]
        return dict(state='lost',reason=reason,segment=self.segment,observations=[])

    def push(self,frame):
        try:
            validate_frame(frame)
            stamp=int(frame['stamp'][0])*1_000_000_000+int(frame['stamp'][1])
            if self.stamp is not None:
                if stamp<=self.stamp:return self.reset('nonincreasing_timestamp')
                if stamp-self.stamp>self.max_gap_ns:return self.reset('frame_gap')
            retained=0;errors={}
            if self.previous is not None and len(self.pixels):
                tracks=correspondences(self.previous,frame,seed_pixels=self.pixels)
                if not tracks['matched']:return self.reset('acoustic_tracks_lost')
                moved=np.array(tracks['current_pixels'],dtype=np.float32)
                # A shared one-pixel footprint cannot carry two independent IDs.
                distance=np.linalg.norm(moved[:,None,:]-moved[None,:,:],axis=2)
                np.fill_diagonal(distance,np.inf)
                unique=np.all(distance>=1.,axis=1)
                if not unique.any():return self.reset('ambiguous_acoustic_tracks')
                indices=np.asarray(tracks['previous_indices'])[unique].tolist()
                self.ids=[self.ids[i] for i in indices]
                self.lengths=[self.lengths[i]+1 for i in indices]
                self.pixels=moved[unique]
                errors=dict(zip(self.ids,np.asarray(tracks['forward_backward_error_pixels'])[unique].tolist()))
                retained=len(self.ids)
            elif self.previous is not None:
                # Even a featureless interval cannot silently change calibration.
                if (not np.array_equal(frame['ranges'],self.previous['ranges']) or
                        not np.array_equal(frame['beam_directions'],self.previous['beam_directions'])):
                    return self.reset('acoustic_pair_calibration_changed')
            points=list(self.pixels)
            for pixel in corner_pixels(frame):
                if len(points)>=self.capacity:break
                height,width=frame['intensity_db'].shape
                if not np.isfinite(pixel).all() or np.any(pixel<0) or np.any(pixel>=[width-1,height-1]):continue
                if points and np.min(np.linalg.norm(np.asarray(points)-pixel,axis=1))<5.:continue
                points.append(pixel.copy());self.ids.append(self.next_id);self.lengths.append(1);self.next_id+=1
            self.pixels=np.asarray(points,dtype=np.float32).reshape(-1,2)
            polar=polar_observations(self.pixels,frame)
            observations=[dict(track_id=ident,length=length,pixel=pixel.tolist(),range_m=float(value[0]),
                azimuth_rad=float(value[1]),forward_backward_error_pixels=errors.get(ident))
                for ident,length,pixel,value in zip(self.ids,self.lengths,self.pixels,polar)]
            self.previous={k:np.array(frame[k],copy=True) for k in ('intensity_db','ranges','beam_directions','stamp')}
            self.stamp=stamp
            return dict(state='tracking' if retained else 'initializing',segment=self.segment,
                        stamp_ns=stamp,retained=retained,observations=observations)
        except (ValueError,KeyError,TypeError) as error:
            return self.reset(str(error))

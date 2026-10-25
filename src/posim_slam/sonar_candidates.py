"""Bounded causal measured-track histories with explicit diagnostic noise.

Independent range/azimuth errors are an experiment assumption, not calibration.
No pose, simulator truth or manually seeded 3D point is accepted here.
"""
import math
import numpy as np


class SonarCandidates:
    def __init__(self,range_sigma,azimuth_sigma):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in (range_sigma,azimuth_sigma)):
            raise ValueError('positive_explicit_sonar_sigmas_required')
        self.variance=[range_sigma**2,azimuth_sigma**2]
        self.stamp=0;self.segment=None;self.history={};self.next_id=0

    def push(self,frame,*,retained_stamps=None):
        """Keep source identity while discarding poses the estimator retired.

        The caller passes the preceding native state's retained pose stamps.
        Active factors only reference retained poses, so their measurement values
        and covariance remain unchanged. Omission preserves full-history replay.
        """
        stamp=frame['stamp_ns'];segment=frame['segment'];rows=frame['observations']
        if type(stamp) is not int or stamp<=self.stamp or type(segment) is not int or segment<0:
            raise ValueError('ordered_measured_sonar_frame_required')
        if self.segment is not None and segment<self.segment:raise ValueError('sonar_segment_regressed')
        if retained_stamps is not None:
            retained_stamps=set(retained_stamps)
            if any(type(t) is not int or not 0<t<stamp for t in retained_stamps):
                raise ValueError('invalid_retained_sonar_pose_stamps')
        if len(rows)>300:raise ValueError('sonar_candidate_capacity')
        prior=self.history if segment==self.segment else {};updated={};next_id=self.next_id
        for row in rows:
            ident=row['track_id'];r=row['range_m'];az=row['azimuth_rad']
            if type(ident) is not int or ident<0 or ident in updated:raise ValueError('invalid_measured_sonar_id')
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in (r,az)) or r<=0 or abs(az)>math.pi:
                raise ValueError('invalid_measured_sonar_observation')
            old=prior.get(ident)
            observations=[] if old is None else list(old['observations'])
            if retained_stamps is not None:
                observations=[o for o in observations if o['stamp_ns'] in retained_stamps]
            if len(observations)>=64:raise ValueError('measured_sonar_history_capacity')
            observations.append(dict(stamp_ns=stamp,range_m=r,azimuth_rad=az))
            source_id=next_id if old is None else old['id']
            if old is None:next_id+=1
            updated[ident]=dict(id=source_id,observations=observations)
        self.history=updated;self.next_id=next_id;self.stamp=stamp;self.segment=segment
        return [dict(**track,covariance=np.diag(self.variance*len(track['observations'])).tolist())
                for track in updated.values() if len(track['observations'])>=3]

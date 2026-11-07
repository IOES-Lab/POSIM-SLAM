# SPDX-License-Identifier: Apache-2.0
"""Conditional local-plane shadow candidates, using only public NumPy code.

Contrast is an empirical diagnostic, never a calibrated probability. Dark
regions do not prove occlusion, free space, physical point identity or height.
"""
import math
import numpy as np
from posim_slam.sonar_input import validate_frame,log_image_values
from posim_slam.sonar_surface import _median,_runs

PROTOCOL=dict(median_width=5,normalized_range_samples=128,aperture_guard_samples=8,
              dark_drop_db=8.,bright_excess_db=3.,background_fit_dynamic_range_db=25.,
              minimum_beam_span=3,minimum_sample_span=3,minimum_area=12,
              flank_length_m=.15,background='one positive common gain; uniform supplied plane')


def plane_background(model,normal,distance):
    """Uniform supplied plane through the existing conditional power operator."""
    from posim_slam.sonar_surface_power import TriangleSurface
    n=np.asarray(normal,dtype=float)
    if (n.shape!=(3,) or not np.isfinite(n).all()
            or not np.isclose(np.linalg.norm(n),1.,atol=1e-9,rtol=0)
            or not np.isfinite(distance) or distance<=0):
        raise ValueError('positive_unit_shadow_plane_required')
    helper=np.eye(3)[np.argmin(abs(n))]
    first=np.cross(n,helper);first/=np.linalg.norm(first);second=np.cross(n,first)
    extent=max(100.,2*model.maximum_range_m)
    corners=np.array([distance*n+extent*(a*first+b*second)
                       for a,b in [(-1,-1),(-1,1),(1,1),(1,-1)]])
    surface=TriangleSurface(corners[[[0,1,2],[0,2,3]]])
    return model.observation(surface,np.zeros(3),np.eye(3)).forward(np.ones(2))


def components(mask):
    """Deterministic 8-connected run components; retain every observed run."""
    mask=np.asarray(mask,dtype=bool)
    parent=[];runs=[];previous=[]
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    for beam in range(mask.shape[1]):
        current=[]
        for start,stop in _runs(mask[:,beam]):
            index=len(parent);parent.append(index)
            runs.append((beam,int(start),int(stop)));current.append(index)
            for old in previous:
                _,a,b=runs[old]
                if min(stop,b)>=max(start,a):parent[root(index)]=root(old)
        previous=current
    groups={}
    for i,row in enumerate(runs):groups.setdefault(root(i),[]).append(row)
    result=[]
    for rows in groups.values():
        bb,lo,hi=np.array(rows).T
        result.append(dict(beam_min=int(bb.min()),beam_max=int(bb.max()),
            sample_min=int(lo.min()),sample_max=int(hi.max()-1),
            area=int((hi-lo).sum()),runs=rows))
    result.sort(key=lambda r:(r['sample_min'],r['beam_min'],r['sample_max'],r['beam_max']))
    return result


def plane_aperture(azimuth,normal,distance,vertical_fov):
    n=np.asarray(normal,dtype=float);a=np.asarray(azimuth,dtype=float)
    if (n.shape!=(3,) or not np.isfinite(n).all()
            or not np.isclose(np.linalg.norm(n),1.,atol=1e-9,rtol=0)
            or a.ndim!=1 or not np.isfinite(a).all()
            or not np.isfinite([distance,vertical_fov]).all()
            or distance<=0 or not 0<vertical_fov<math.pi):
        raise ValueError('invalid_shadow_aperture')
    v=n[0]*np.cos(a)+n[1]*np.sin(a);half=vertical_fov/2
    values=np.column_stack((v*math.cos(half)-n[2]*math.sin(half),
                            v*math.cos(half)+n[2]*math.sin(half)))
    maximum=values.max(axis=1);minimum=values.min(axis=1)
    # Both stationary extrema, including a tangent inside the aperture.
    phase=np.arctan2(n[2],v)
    for shift in (0.,math.pi,-math.pi):
        e=phase+shift
        value=v*np.cos(e)+n[2]*np.sin(e)
        inside=abs(e)<=half
        maximum=np.where(inside,np.maximum(maximum,value),maximum)
        minimum=np.where(inside,np.minimum(minimum,value),minimum)
    near=np.divide(distance,maximum,out=np.full_like(v,np.inf),where=maximum>0)
    far=np.divide(distance,minimum,out=np.full_like(v,np.inf),where=minimum>0)
    return near,far


def extract(frame, *, normal, distance, vertical_fov, range_max, expected_power=None):
    """Compare aligned median background with a supplied physical plane mean.

    Expected power may depend on calibrated rays and the supplied DVL plane,
    but must not contain the assessment scene or RGB. It is a conditional
    uniform-ground model. Unknown reflectivity and plane extrapolation remain.
    """
    validate_frame(frame)
    ranges=np.asarray(frame['ranges'],dtype=float)
    a=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
    if not np.isfinite(range_max) or range_max<=0:raise ValueError('invalid_shadow_range_clip')
    near,far=plane_aperture(a,normal,distance,vertical_fov)
    valid=(np.isfinite(near)&np.isfinite(far)&(near>ranges[0])
           &(far<range_max)&(far-near>.25))
    raw,support=log_image_values(frame['intensity_db'])
    width=PROTOCOL['median_width']
    smooth=_median(_median(raw,width,0),width,1)
    smooth_support=np.all(np.lib.stride_tricks.sliding_window_view(
        np.pad(support,width//2,constant_values=False),(width,width)),axis=(-1,-2))
    u=np.linspace(0,1,PROTOCOL['normalized_range_samples'])
    samples=np.full((len(u),len(a)),np.nan)
    samples[:,valid]=near[valid]+u[:,None]*(far-near)[valid]
    aligned=np.full_like(samples,np.nan)
    predicted_aligned=None
    if expected_power is not None:
        expected_power=np.asarray(expected_power,dtype=float)
        if (expected_power.shape!=smooth.shape or not np.isfinite(expected_power).all()
                or np.any(expected_power<0) or not np.any(expected_power>0)):
            raise ValueError('finite_positive_shadow_background_required')
        expected=10*np.log10(np.maximum(expected_power,1e-100))
        expected=_median(_median(expected,width,0),width,1)
        predicted_aligned=np.full_like(samples,np.nan)
    for beam in np.flatnonzero(valid):
        row=np.interp(samples[:,beam],ranges,np.arange(len(ranges)))
        lo=np.floor(row).astype(int);hi=np.minimum(lo+1,len(ranges)-1)
        observed=smooth_support[lo,beam]&smooth_support[hi,beam]
        aligned[observed,beam]=np.interp(samples[observed,beam],ranges,smooth[:,beam])
        if predicted_aligned is not None:
            predicted_aligned[observed,beam]=np.interp(samples[observed,beam],ranges,expected[:,beam])
    gain=None
    if predicted_aligned is None:
        # Explicit control: no physics-based angular/range normalization.
        background=np.full(len(u),np.nan)
        observed=np.any(np.isfinite(aligned),axis=1)
        background[observed]=np.nanmedian(aligned[observed],axis=1)
        contrast=aligned-background[:,None]
    else:
        observed=np.isfinite(predicted_aligned)&np.isfinite(aligned)
        eligible=np.zeros_like(observed)
        if observed.any():
            eligible=observed&(predicted_aligned>=predicted_aligned[observed].max()
                       -PROTOCOL['background_fit_dynamic_range_db'])
        if eligible.sum()<32:
            return dict(available=False,reason='insufficient_conditional_background_support',components=[])
        gain=float(np.median((aligned-predicted_aligned)[eligible]))
        contrast=aligned-predicted_aligned-gain
    mask=np.isfinite(contrast)&(contrast<=-PROTOCOL['dark_drop_db'])
    guard=PROTOCOL['aperture_guard_samples']
    mask[:guard]=False;mask[-guard:]=False
    raw_components=components(mask)
    candidates=[]
    for index,c in enumerate(raw_components):
        if (c['beam_max']-c['beam_min']+1<PROTOCOL['minimum_beam_span']
                or c['sample_max']-c['sample_min']+1<PROTOCOL['minimum_sample_span']
                or c['area']<PROTOCOL['minimum_area']):continue
        c=dict(c,component=index)
        c['aperture_truncated']=bool(c['sample_min']<=guard or c['sample_max']>=len(u)-guard-1)
        c['angular_truncated']=bool(c['beam_min']<=2 or c['beam_max']>=len(a)-3
            or not valid[max(0,c['beam_min']-1):min(len(a),c['beam_max']+2)].all())
        observations=[]
        for beam,start,stop in c['runs']:
            first=float(samples[start,beam]);last=float(samples[stop-1,beam])
            before=np.flatnonzero((samples[:,beam]>=first-PROTOCOL['flank_length_m'])&(np.arange(len(u))<start))
            after=np.flatnonzero((samples[:,beam]<=last+PROTOCOL['flank_length_m'])&(np.arange(len(u))>=stop))
            values=contrast[before,beam]
            paired=bool(np.any(np.isfinite(values)&(values>=PROTOCOL['bright_excess_db'])))
            recovered=bool(np.any(np.isfinite(contrast[after,beam])&(contrast[after,beam]>=-PROTOCOL['bright_excess_db'])))
            observations.append(dict(beam=int(beam),start=int(start),stop=int(stop),
                edge_range_m=first,shadow_end_range_m=last,
                boundary_ranges_m=[float(samples[max(start-1,0),beam]),first,last,
                                   float(samples[min(stop,len(u)-1),beam])],
                azimuth_rad=float(a[beam]),bright_pair_observed=paired,recovery_observed=recovered))
        c.pop('runs');c['observations']=observations;candidates.append(c)
    return dict(available=True,components=candidates,raw_component_count=len(raw_components),
          angles=a,sample_ranges=samples,near=near,far=far,valid=valid,
          aligned=aligned,contrast=contrast,mask=mask,gain_db=gain,
          background_model='uniform_plane_power' if expected_power is not None else 'aligned_angular_median',
          map_qualified=False,physical_identity_verified=False)

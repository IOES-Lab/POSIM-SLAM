"""Intensity-region candidates for imaging sonar; no invented elevations.

Ground-entry bounds are an explicit scene hypothesis supplied by the caller.
Dark deficits are possible shadows, not measured empty space or landmarks.
Only NumPy is required. No ground truth, optical image or simulator scene input.
"""
from dataclasses import dataclass
import math
import numpy as np
from .sonar_input import validate_frame


@dataclass(frozen=True)
class SurfaceConfig:
    noise_range_max_m: float = 1.0
    return_margin_db: float = 12.0
    echo_excess_db: float = 8.0
    shadow_drop_db: float = 10.0
    min_shadow_length_m: float = .30
    min_echo_length_m: float = .075
    recovery_length_m: float = .15
    min_region_beams: int = 3


def ping_plane_in_sonar(rotation_world_imu, rotation_imu_ping, imu_to_ping,
                        rotation_imu_sonar, imu_to_sonar, ping_rays, range_m, *, bias_m=0.):
    """Conditional horizontal plane from a minimum-ray Ping and IMU attitude.

    This is a caller-selected flat-ground hypothesis. One range does not measure
    slope, and a Ping return from an intervening object can invalidate it.
    Returns a unit normal towards the sensor and positive sonar altitude.
    """
    r,rp,rs=(np.asarray(v,dtype=float) for v in (rotation_world_imu,rotation_imu_ping,rotation_imu_sonar))
    lp,ls=(np.asarray(v,dtype=float) for v in (imu_to_ping,imu_to_sonar))
    rays=np.asarray(ping_rays,dtype=float)
    if (any(v.shape!=(3,3) or not np.isfinite(v).all() or
            not np.allclose(v.T@v,np.eye(3),atol=1e-8,rtol=0) or np.linalg.det(v)<0 for v in (r,rp,rs))
            or any(v.shape!=(3,) or not np.isfinite(v).all() for v in (lp,ls))
            or rays.ndim!=2 or rays.shape[1]!=3 or not 1<=len(rays)<=4096 or not np.isfinite(rays).all()
            or not np.allclose(np.linalg.norm(rays,axis=1),1,atol=1e-8,rtol=0)
            or not np.isfinite([range_m,bias_m]).all() or range_m-bias_m<=0):
        raise ValueError('invalid_ping_plane_inputs')
    up=np.array([0.,0.,1.]);down=np.max(-(rays@(r@rp).T)@up)
    if down<=0:raise ValueError('ping_has_no_downward_ray')
    altitude=float((range_m-bias_m)*down+up@r@(ls-lp))
    if altitude<=0:raise ValueError('nonpositive_sonar_altitude')
    return (r@rs).T@up,altitude


def ground_entry_band(azimuth_rad, vertical_fov_rad, normal_sonar, plane_distance_m,
                      *, distance_bound_m=0., normal_bound_rad=0.):
    """Bound first plane intersections over the entire elevation fan.

    Plane: n dot x = d in sonar coordinates, unit n pointing to the sensor,
    d < 0. Bounds are deterministic caller assumptions, not a covariance.
    Infinite limits mean the relevant intersection is not bounded/visible.
    Uses analytic extrema, so tilted planes do not depend on ray sample count.
    """
    a=np.asarray(azimuth_rad,dtype=float);n=np.asarray(normal_sonar,dtype=float)
    if (a.ndim!=1 or not 1<=len(a)<=4096 or not np.isfinite(a).all()
            or np.any(abs(a)>=math.pi/2) or n.shape!=(3,) or not np.isfinite(n).all()
            or not np.isclose(np.linalg.norm(n),1.,atol=1e-8,rtol=0)
            or not np.isfinite([vertical_fov_rad,plane_distance_m,distance_bound_m,normal_bound_rad]).all()
            or not 0<vertical_fov_rad<math.pi or plane_distance_m>=0
            or not 0<=distance_bound_m<-plane_distance_m or not 0<=normal_bound_rad<math.pi/2):
        raise ValueError('invalid_ground_entry_geometry')
    half=vertical_fov_rad/2
    b=n[0]*np.cos(a)+n[1]*np.sin(a)
    extreme=(np.arctan2(n[2],b)+2*math.pi)%(2*math.pi)-math.pi
    values=np.column_stack([b*np.cos(half)-n[2]*np.sin(half),
                            b*np.cos(half)+n[2]*np.sin(half),
                            np.where(abs(extreme)<=half,b*np.cos(extreme)+n[2]*np.sin(extreme),np.inf)])
    angle=np.arccos(np.clip(values.min(1),-1,1))
    lower_dot=np.cos(np.minimum(math.pi,angle+normal_bound_rad))
    upper_dot=np.cos(np.maximum(0.,angle-normal_bound_rad))
    result=np.full((len(a),2),np.inf)
    for column,(dot,height) in enumerate(((lower_dot,-plane_distance_m-distance_bound_m),
                                         (upper_dot,-plane_distance_m+distance_bound_m))):
        valid=dot < -1e-9
        result[valid,column]=height/-dot[valid]
    return result


def _runs(mask):
    changes=np.diff(np.r_[False,mask,False].astype(np.int8))
    return zip(np.flatnonzero(changes==1),np.flatnonzero(changes==-1))


def _median(a, width, axis):
    pad=[(0,0)]*a.ndim;pad[axis]=(width//2,width//2)
    windows=np.lib.stride_tricks.sliding_window_view(np.pad(a,pad,mode='edge'),width,axis=axis)
    return np.median(windows,axis=-1)


def _regions(intervals, minimum):
    """Join overlapping runs only across adjacent beams; preserve each run."""
    parent=list(range(len(intervals)))
    def root(i):
        while parent[i]!=i:
            parent[i]=parent[parent[i]];i=parent[i]
        return i
    by_beam={}
    for i,row in enumerate(intervals):
        for j in by_beam.get(row['beam']-1,[]):
            other=intervals[j]
            if min(row['stop_bin'],other['stop_bin'])>max(row['start_bin'],other['start_bin']):
                parent[root(i)]=root(j)
        by_beam.setdefault(row['beam'],[]).append(i)
    groups={}
    for i,row in enumerate(intervals):groups.setdefault(root(i),[]).append(row)
    return [dict(beam_first=min(x['beam'] for x in rows),beam_last=max(x['beam'] for x in rows),
                 intervals=rows) for rows in groups.values() if len({x['beam'] for x in rows})>=minimum]


def segment_surface(frame, *, ground_entry_band_m, config=SurfaceConfig()):
    """Return echo / dark-deficit regions and ground-onset candidates.

    A same-range upper quantile across eligible beams estimates background.
    The caller must test beam-pattern/reflectivity effects; dark deficits alone
    cannot distinguish shadow from low reflectivity. Unknown background ends a
    run without declaring a shadow endpoint. No 3D points are emitted here.
    """
    validate_frame(frame)
    floats=[getattr(config,k) for k in config.__dataclass_fields__ if k!='min_region_beams']
    if (not np.isfinite(floats).all() or min(floats)<=0 or type(config.min_region_beams) is not int
            or not 2<=config.min_region_beams<=31):raise ValueError('invalid_surface_config')
    raw=np.asarray(frame['intensity_db'],dtype=float);ranges=np.asarray(frame['ranges'])
    rows,beams=raw.shape;band=np.asarray(ground_entry_band_m,dtype=float)
    if (band.shape!=(beams,2) or np.isnan(band).any() or np.any(band<=0) or np.any(band[:,0]>band[:,1])):
        raise ValueError('invalid_ground_entry_band')
    noise_rows=(ranges>.2)&(ranges<config.noise_range_max_m)
    if noise_rows.sum()<4:raise ValueError('insufficient_noise_window')
    # Use raw dB directly; do not clip away weak echoes with display contrast.
    smooth=_median(_median(raw,3,1),3,0)
    noise=float(np.median(smooth[noise_rows]))
    floor=noise+config.return_margin_db
    # Candidate ground may begin anywhere inside the bound. Waiting until the
    # upper bound would discard the very start of each shadow. Keep censoring
    # flags instead of silently equating a mask boundary with an observed edge.
    after_ground=ranges[:,None]>=band[None,:,0]
    background=np.full(rows,np.nan)
    for i in range(rows):
        candidates=smooth[i,after_ground[i]]
        if len(candidates)>=max(8,beams//4):background[i]=np.quantile(candidates,.7)
    reliable=np.isfinite(background)&(background>floor)
    # Echo contrast uses the whole fan even before ground entry. This avoids
    # promoting broad range sidelobes merely because they exceed near noise.
    base=np.quantile(smooth,.7,axis=1)
    echo=(smooth>np.maximum(floor,base[:,None]+config.echo_excess_db))&(ranges[:,None]>.4)
    dark=(smooth<background[:,None]-config.shadow_drop_db)&reliable[:,None]&after_ground
    onsets=np.full(beams,np.nan)
    echoes=[];shadows=[]
    for beam in range(beams):
        for lo,hi in _runs((smooth[:,beam]>floor)&(ranges>=band[beam,0])&(ranges<=band[beam,1])):
            if ranges[hi-1]-ranges[lo]>=config.recovery_length_m:
                # A prior-mask boundary is not an observed intensity transition.
                if lo>0 and smooth[lo-1,beam]<=floor:onsets[beam]=ranges[lo]
                break
        for lo,hi in _runs(echo[:,beam]):
            if ranges[hi-1]-ranges[lo]>=config.min_echo_length_m:
                echoes.append(dict(beam=beam,start_bin=int(lo),stop_bin=int(hi),
                                   range_start_m=float(ranges[lo]),range_last_m=float(ranges[hi-1])))
        for lo,hi in _runs(dark[:,beam]):
            if ranges[hi-1]-ranges[lo]<config.min_shadow_length_m:continue
            end=int(np.searchsorted(ranges,(ranges[hi] if hi<rows else ranges[-1])+config.recovery_length_m))
            # A gap / weak far background / final range bin never establishes
            # return to the seabed intensity. Require a sustained visible end.
            recovered=(hi<rows and end<rows and end>hi and reliable[hi:end+1].all()
                       and np.all(smooth[hi:end+1,beam]>=background[hi:end+1]-config.shadow_drop_db))
            shadows.append(dict(beam=beam,start_bin=int(lo),stop_bin=int(hi),
                                range_start_m=float(ranges[lo]),range_last_m=float(ranges[hi-1]),
                                start_observed=bool(lo>0 and reliable[lo-1] and after_ground[lo-1,beam]
                                                    and not dark[lo-1,beam]),
                                end_observed=bool(recovered),
                                range_end_m=float(ranges[hi]) if recovered else None))
    groups=_regions(shadows,config.min_region_beams)
    echo_groups=_regions(echoes,config.min_region_beams)
    return dict(noise_db=noise,background_db=background,smoothed_db=smooth,
                ground_onset_m=onsets,echo_regions=echo_groups,shadow_candidates=groups,
                scope='conditional intensity regions; no physical association or 3D landmarks',
                map_qualified=False)

"""Experimental ground-entry curves from FLS intensity, without point tracking.

An entry is a range transition, not a persistent scatterer. A local plane can
explain such transitions at the elevation aperture boundary. The hypothesis is
explicit: mostly visible, locally planar bottom within a slope bound about
estimated gravity. No prior terrain, camera, pose position or truth is accepted.
Thresholds and conditional fit information are diagnostic, not calibrated noise.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from .sonar_input import validate_frame
from .sonar_surface import _median, _runs, ground_entry_band


@dataclass(frozen=True)
class GroundEntryConfig:
    noise_range_max_m: float = 1.
    return_margin_db: float = 12.
    beam_relative_margin_db: float | None = 6.
    sustained_return_m: float = .15
    angular_spacing_rad: float = math.radians(2.)
    minimum_span_rad: float = math.radians(45.)
    maximum_slope_rad: float = math.radians(30.)
    range_sigma_m: float = .03
    minimum_consensus: float = .6
    ransac_trials: int = 256


def ground_entries(frame, *, config=GroundEntryConfig(), source_bounds=None, amplitude_error=None):
    """First sustained, observed noise-to-return transition in each beam.

    Near-range noise and absent returns remain censored. Smoothing rejects
    isolated samples but does not establish seabed identity. Angular thinning
    bounds computation and avoids counting every oversampled beam as a new fit
    constraint; it is not a claim that the remaining errors are independent.
    Optional source_bounds requires an explicit complex response and amplitude
    error interval. It only censors existing angularly thinned samples whose
    source amplitude may be zero; it never searches a later, stronger onset.
    """
    validate_frame(frame)
    values=[getattr(config,k) for k in config.__dataclass_fields__ if k not in ('ransac_trials','beam_relative_margin_db')]
    if (not np.isfinite(values).all() or min(values)<=0
            or config.noise_range_max_m<=.2 or config.minimum_consensus>1
            or not 0<config.maximum_slope_rad<math.pi/2
            or not 0<config.angular_spacing_rad<config.minimum_span_rad<math.pi
            or type(config.ransac_trials) is not int or not 1<=config.ransac_trials<=1024):
        raise ValueError('invalid_ground_entry_config')
    if config.beam_relative_margin_db is not None and (not np.isfinite(config.beam_relative_margin_db)
            or config.beam_relative_margin_db<=0):raise ValueError('invalid_ground_relative_margin')
    if source_bounds is None:
        if amplitude_error is not None:raise ValueError('amplitude_error_requires_beam_response')
    else:
        from .sonar_phase_bounds import BeamPhaseBounds
        if (not isinstance(source_bounds,BeamPhaseBounds) or source_bounds.beams!=frame['intensity_db'].shape[1]
                or not isinstance(amplitude_error,(tuple,list)) or len(amplitude_error)!=2):
            raise ValueError('explicit_ground_beam_response_and_error_required')
        # Validate even when no onset is visible; errors cannot bypass the contract.
        source_bounds.query(np.zeros((1,source_bounds.beams)),[0],
                            relative_error=amplitude_error[0],absolute_error=amplitude_error[1])
    ranges=np.asarray(frame['ranges'],dtype=float)
    angles=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
    if not np.all(np.diff(angles)>0) or np.any(abs(angles)>=math.pi/2):
        raise ValueError('ordered_forward_ground_beams_required')
    near=(ranges>.2)&(ranges<config.noise_range_max_m)
    if near.sum()<4:raise ValueError('insufficient_ground_noise_window')
    smooth=_median(_median(np.asarray(frame['intensity_db'],dtype=float),3,1),3,0)
    noise=float(np.median(smooth[near]));floor=np.full(len(angles),noise+config.return_margin_db)
    if config.beam_relative_margin_db is not None:
        # Weak angular leakage from another reflector can exceed near-range
        # noise in every beam. Require support relative to each beam's strong
        # returns as well. Q95 avoids treating one isolated peak as its scale.
        # This may censor a weak bottom under a stronger object; it does not
        # prove that the resulting edge is a physical first arrival.
        floor=np.maximum(floor,np.quantile(smooth[ranges>config.noise_range_max_m],.95,axis=0)
                         -config.beam_relative_margin_db)
    onsets=np.full(len(angles),np.nan)
    for beam in range(len(angles)):
        for lo,hi in _runs((smooth[:,beam]>floor[beam])&(ranges>config.noise_range_max_m)):
            if (ranges[hi-1]-ranges[lo]>=config.sustained_return_m and lo>0
                    and smooth[lo-1,beam]<=floor[beam]):
                onsets[beam]=ranges[lo];break
    selected=[]
    # Select by angle before testing visibility, so missing returns cannot
    # increase the apparent independent angular support of another region.
    for i,angle in enumerate(angles):
        if not selected or angle-angles[selected[-1]]>=config.angular_spacing_rad:selected.append(i)
    if len(selected)>128:raise ValueError('ground_entry_angular_budget')
    bound_report={}
    if source_bounds is not None:
        indices=np.asarray(selected,dtype=int);indices=indices[np.isfinite(onsets[indices])]
        excluded=0
        if len(indices):
            bins=np.searchsorted(ranges,onsets[indices])
            with np.errstate(over='ignore'):
                magnitudes=10.**(np.asarray(frame['intensity_db'][bins],dtype=float)/20.)
            interval=source_bounds.query(magnitudes,indices,
                relative_error=amplitude_error[0],absolute_error=amplitude_error[1])
            rejected=indices[interval['lower']<=0];excluded=len(rejected);onsets[rejected]=np.nan
        bound_report=dict(source_bound_tested=len(indices),source_bound_excluded=excluded)
    return dict(azimuth_rad=angles,range_m=onsets,selected_beams=np.asarray(selected,dtype=int),
                noise_db=noise,return_floor_db=floor,**bound_report)


def ground_entry_curve(frame, *, vertical_fov_rad, up_sonar, config=GroundEntryConfig(),
                       source_bounds=None, amplitude_error=None):
    """Measured onset curve under an explicit downward aperture-boundary assumption.

    No whole-fan plane is required. The returned elevation is assumed, not
    observed; curved/occluding surfaces may violate this hypothesis. Retain the
    raw range/beam identifiers so downstream estimators can model that ambiguity.
    """
    entries=ground_entries(frame,config=config,source_bounds=source_bounds,amplitude_error=amplitude_error);up=np.asarray(up_sonar,dtype=float)
    if (up.shape!=(3,) or not np.isfinite(up).all() or not np.isclose(np.linalg.norm(up),1.,atol=1e-8,rtol=0)
            or not np.isfinite(vertical_fov_rad) or not 0<vertical_fov_rad<math.pi):
        raise ValueError('invalid_ground_entry_geometry')
    base=dict(accepted=False,map_qualified=False,navigation_ready=False,entries=entries,
              scope='conditional aperture-boundary curve; elevation not measured')
    if abs(up[2])<=math.sin(config.maximum_slope_rad):
        return dict(base,reason='ground_aperture_branch_unresolved')
    indices=entries['selected_beams'];indices=indices[np.isfinite(entries['range_m'][indices])]
    angles=entries['azimuth_rad'][indices];ranges=entries['range_m'][indices]
    if len(indices)<8 or np.ptp(angles)<config.minimum_span_rad:
        return dict(base,reason='insufficient_ground_entry_support')
    elevation=-math.copysign(vertical_fov_rad/2,up[2])
    rays=np.column_stack([np.cos(angles)*math.cos(elevation),np.sin(angles)*math.cos(elevation),
                          np.full(len(angles),math.sin(elevation))])
    return dict(base,accepted=True,reason='conditional_ground_curve',beam_ids=indices,
                rays=rays,points_sonar=ranges[:,None]*rays,boundary_elevation_rad=elevation)


def fit_ground_entry(frame, *, vertical_fov_rad, up_sonar, config=GroundEntryConfig(),
                       source_bounds=None, amplitude_error=None):
    """Fit a supported aperture-boundary plane; emit measured inliers only.

    A plane n·x=-h is represented by q=n/h. For boundary ray u and measured
    range r, u·q=-1/r. RANSAC initializes this linear inverse-range model;
    refinement minimizes metric range residuals. Every accepted plane is also
    checked against the full elevation fan's analytic first intersection.
    Output points depend on the boundary hypothesis, not measured elevation.
    No position correction, loop closure or navigation readiness is claimed.
    """
    curve=ground_entry_curve(frame,vertical_fov_rad=vertical_fov_rad,up_sonar=up_sonar,config=config,
                             source_bounds=source_bounds,amplitude_error=amplitude_error)
    entries=curve['entries'];up=np.asarray(up_sonar,dtype=float)
    base=dict(accepted=False,map_qualified=False,navigation_ready=False,entries=entries,
              scope='conditional local ground-entry surface; no fixed point correspondence')
    if not curve['accepted']:
        return dict(base,reason=curve['reason'])
    indices=curve['beam_ids']
    angles=entries['azimuth_rad'][indices];ranges=entries['range_m'][indices]
    elevation=curve['boundary_elevation_rad'];rays=curve['rays']
    def supported(q):
        length=np.linalg.norm(q)
        return (np.isfinite(q).all() and length>1e-9
                and q@up/length>=math.cos(config.maximum_slope_rad)
                and np.all(rays@q < -1e-9))
    gate=3*config.range_sigma_m;rng=np.random.default_rng(0);best=None
    for _ in range(config.ransac_trials):
        chosen=rng.choice(len(ranges),3,replace=False)
        if np.linalg.cond(rays[chosen])>1e4:continue
        q=np.linalg.solve(rays[chosen],-1/ranges[chosen])
        if not supported(q):continue
        error=abs(-1/(rays@q)-ranges);mask=error<=gate
        score=(int(mask.sum()),-float(np.minimum(error,gate).sum()))
        if best is None or score>best[0]:best=(score,q,mask)
    if best is None:return dict(base,reason='no_supported_ground_plane')
    _,q,mask=best
    for _ in range(8):
        if mask.sum()<8:break
        u=rays[mask];r=ranges[mask];dot=u@q
        residual=-1/dot-r;j=u/dot[:,None]**2
        delta=np.linalg.lstsq(j,-residual,rcond=1e-10)[0]
        improved=False
        for scale in (1.,.5,.25,.125):
            candidate=q+scale*delta
            if supported(candidate) and np.sum((-1/(u@candidate)-r)**2)<=residual@residual:
                q=candidate;improved=True;break
        mask=abs(-1/(rays@q)-ranges)<=gate
        if not improved or np.linalg.norm(delta)<1e-10:break
    if (mask.sum()<8 or mask.sum()/len(indices)<config.minimum_consensus
            or np.ptp(angles[mask])<config.minimum_span_rad):
        return dict(base,reason='ground_plane_consensus_failed',visible_beams=len(indices),inliers=int(mask.sum()))
    height=1/np.linalg.norm(q);normal=q*height
    predicted=ground_entry_band(angles[mask],vertical_fov_rad,normal,-height)[:,0]
    boundary=-1/(rays[mask]@q)
    if not np.allclose(predicted,boundary,atol=1e-7,rtol=0):
        return dict(base,reason='ground_first_return_not_at_aperture_boundary')
    error=predicted-ranges[mask];j=rays[mask]/(rays[mask]@q)[:,None]**2/config.range_sigma_m
    singular=np.linalg.svd(j,compute_uv=False)
    if singular[-1]<=1e-8*singular[0]:return dict(base,reason='ground_plane_rank_deficient')
    return dict(base,accepted=True,reason='conditional_ground_plane',normal_sonar=normal,
        distance_m=-height,visible_beams=len(indices),inlier_beams=indices[mask],
        points_sonar=ranges[mask,None]*rays[mask],range_rmse_m=float(np.sqrt(np.mean(error**2))),
        range_information_singular_values=singular,boundary_elevation_rad=elevation,
        uncertainty_scope='fit information conditional on correct entries, aperture model and attitude; no calibrated covariance')

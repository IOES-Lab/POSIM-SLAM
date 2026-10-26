"""Pose-conditioned search geometry for imaging-sonar correspondence.

An observed range/azimuth defines an elevation arc, not a 3D landmark. Project
the entire sampled arc into the next sensor frame. Caller-supplied poses must
share one metric map frame. Pose and measurement uncertainty are not estimated
here; this nominal locus must not be used as a calibrated rejection gate.
"""
import math
import numpy as np
from .dvl_scale import rotation, vector
from .sonar_factor import within_fov


def project_return_arc(*, measured_range, measured_azimuth, previous_position,
                       previous_attitude, current_position, current_attitude,
                       imu_to_sonar, sonar_to_imu, range_min, range_max,
                       horizontal_fov, vertical_fov, samples=65):
    """Return sampled elevation and predicted range/azimuth with support flags.

    Both poses are IMU-to-map transforms; lever and mount use the same contract
    as point_return_factor. No seabed plane, camera image or true pose is read.
    Sampling preserves both elevation branches but does not bound the continuous
    curve between samples. Invalid target support is explicit, never clamped.
    """
    values = (measured_range, measured_azimuth, range_min, range_max,
              horizontal_fov, vertical_fov)
    if (not all(math.isfinite(v) for v in values) or not 0 < range_min < range_max
            or not range_min <= measured_range <= range_max
            or not 0 < horizontal_fov <= math.pi or not 0 < vertical_fov < math.pi
            or not within_fov(measured_azimuth, horizontal_fov)):
        raise ValueError('invalid_sonar_measurement')
    if type(samples) is not int or not 3 <= samples <= 257 or samples % 2 != 1:
        raise ValueError('bounded_odd_arc_samples_required')
    p0, p1 = vector(previous_position), vector(current_position)
    r0, r1 = rotation(previous_attitude), rotation(current_attitude)
    lever, mount = vector(imu_to_sonar), rotation(sonar_to_imu)
    elevations = np.linspace(-vertical_fov/2, vertical_fov/2, samples)
    rays = measured_range * np.column_stack((np.cos(elevations)*math.cos(measured_azimuth),
        np.cos(elevations)*math.sin(measured_azimuth), np.sin(elevations)))
    # Current sonar coordinates, including rotating sensor lever arms.
    relative_rotation = mount.T @ r1.T @ r0 @ mount
    relative_translation = mount.T @ (r1.T @ (p0-p1+r0@lever)-lever)
    local = rays @ relative_rotation.T + relative_translation
    ranges = np.linalg.norm(local, axis=1)
    horizontal = np.linalg.norm(local[:, :2], axis=1)
    azimuths = np.arctan2(local[:, 1], local[:, 0])
    target_elevations = np.arctan2(local[:, 2], horizontal)
    tolerance = 4*float(np.finfo(np.float32).eps)
    supported = ((local[:, 0] > 0) & (horizontal > 1e-6)
        & (ranges >= range_min) & (ranges <= range_max)
        & (abs(azimuths) <= horizontal_fov/2+tolerance)
        & (abs(target_elevations) <= vertical_fov/2+tolerance))
    return dict(source_elevation_rad=elevations, range_azimuth=np.column_stack((ranges, azimuths)),
        target_elevation_rad=target_elevations, supported=supported,
        scope='nominal fixed-pose search locus; not an association or uncertainty gate',
        elevation_resolved=False)


def rank_return_patches(previous, current, source_pixel, candidate_pixels, *,
                        patch_radius=3, search_radius=2, maximum_modes=8):
    """Rank distinct integer-pixel patch modes around a caller-provided locus.

    Zero-mean normalized correlation tolerates positive gain and offset. It is
    not a probability or an association acceptance test. The caller must inspect
    competing modes and validate future views. No elevation or point is returned.
    Search margin is explicit pixels, not a pose-covariance approximation.
    """
    images=[np.asarray(v,dtype=float) for v in (previous,current)]
    if any(v.ndim!=2 or min(v.shape)<3 or max(v.shape)>4096 or v.size>2_000_000
           or not np.isfinite(v).all() for v in images):
        raise ValueError('invalid_sonar_patch_image')
    if (type(patch_radius) is not int or not 1<=patch_radius<=10
            or type(search_radius) is not int or not 0<=search_radius<=4
            or type(maximum_modes) is not int or not 1<=maximum_modes<=16):
        raise ValueError('invalid_sonar_patch_budget')
    source=np.asarray(source_pixel,dtype=float);candidates=np.asarray(candidate_pixels,dtype=float)
    if (source.shape!=(2,) or not np.isfinite(source).all() or candidates.ndim!=2
            or candidates.shape[1]!=2 or len(candidates)>257 or not np.isfinite(candidates).all()):
        raise ValueError('invalid_sonar_patch_pixels')
    def patch(image,pixel):
        x,y=pixel;r=patch_radius
        if x-r<0 or y-r<0 or x+r>=image.shape[1] or y+r>=image.shape[0]:return None
        values=image[y-r:y+r+1,x-r:x+r+1].ravel().copy()
        values-=values.mean();norm=float(np.linalg.norm(values))
        return values/norm if norm>1e-10 else None
    result=dict(modes=[],evaluated_centres=0,scope='appearance ranking only; physical identity unresolved')
    if np.any(source<0) or np.any(source>=[images[0].shape[1],images[0].shape[0]]):
        return dict(result,reason='source_patch_unavailable')
    anchor=patch(images[0],tuple(np.rint(source).astype(int)))
    if anchor is None:return dict(result,reason='source_patch_unavailable')
    candidates=candidates[np.all(candidates>=-search_radius-1,axis=1)
        & np.all(candidates<np.array([images[1].shape[1],images[1].shape[0]])+search_radius+1,axis=1)]
    centres=set()
    for x,y in np.rint(candidates).astype(int):
        for dy in range(-search_radius,search_radius+1):
            for dx in range(-search_radius,search_radius+1):centres.add((int(x+dx),int(y+dy)))
    scored=[]
    for centre in sorted(centres):
        target=patch(images[1],centre)
        if target is not None:scored.append((float(np.clip(anchor@target,-1,1)),centre))
    result['evaluated_centres']=len(scored)
    modes=[]
    for score,centre in sorted(scored,key=lambda item:(-item[0],item[1])):
        if any(max(abs(centre[i]-m['pixel'][i]) for i in (0,1))<=patch_radius for m in modes):continue
        modes.append(dict(pixel=list(centre),correlation=score))
        if len(modes)==maximum_modes:break
    return dict(result,modes=modes,reason='ranked' if modes else 'target_patch_unavailable')

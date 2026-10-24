"""Acoustic intensity-only planar correspondence core. No ground truth or file I/O.

OpenCV 4.10 reference implementation; planar estimates do not resolve elevation.
"""
from .acoustic_backend import cv as cv2
import numpy as np

FILTERS = ('raw', 'smooth', 'contrast', 'whiten', 'range_whiten')


def validate_frame(frame):
    power = np.asarray(frame['intensity_db'])
    ranges = np.asarray(frame['ranges'])
    directions = np.asarray(frame['beam_directions'])
    stamp = np.asarray(frame['stamp'])
    if (power.ndim != 2 or power.size > 2_000_000 or not 16 <= min(power.shape) or max(power.shape) > 4096
            or ranges.shape != (power.shape[0],) or directions.shape != (power.shape[1], 3)
            or stamp.shape != (2,) or not all(np.isfinite(v).all() for v in (power, ranges, directions, stamp))
            or not (np.diff(ranges) > 0).all() or ranges[0] < 0
            or stamp[0] < 0 or not np.equal(stamp,np.floor(stamp)).all()
            or not 0 <= stamp[1] < 1e9):
        raise ValueError('invalid_acoustic_frame')
    angles = np.arctan2(directions[:, 1], directions[:, 0])
    if (not (np.diff(angles) > 0).all() or np.max(np.abs(directions[:, 2])) > 1e-5
            or not np.allclose(np.linalg.norm(directions, axis=1), 1, atol=1e-5)):
        raise ValueError('invalid_acoustic_fan')


def image(frame, variant):
    power = np.asarray(frame['intensity_db'], dtype=np.float32)
    if variant == 'raw':
        return cv2.GaussianBlur(np.uint8(np.clip((power - 20) * 255 / 70, 0, 255)), (5, 5), 0)
    power = cv2.GaussianBlur(power, (0, 0), sigmaX=1, sigmaY=2.5)
    if variant == 'contrast':
        power = (power - cv2.GaussianBlur(power, (0, 0), sigmaX=8, sigmaY=8) + 8) * 10
    elif variant == 'range_whiten':
        # Remove same-range broad returns using acoustic intensity alone.
        # This is an experimental common-mode filter, not a seabed label.
        power = (power - np.median(power, axis=1)[:, None] + 10) * 8
    elif variant == 'whiten':
        power = (power - np.median(power, axis=0)[None, :] + 15) * 5
    else:
        power = (power - 20) * 255 / 70
    return np.uint8(np.clip(power, 0, 255))


def polar_observations(pixels, frame):
    pixels = pixels.reshape(-1, 2)
    height, width = frame['intensity_db'].shape
    if not np.isfinite(pixels).all() or np.any(pixels < 0) or np.any(pixels >= [width - 1, height - 1]):
        raise ValueError('acoustic_track_outside_image')
    ranges = np.interp(pixels[:, 1], np.arange(height), frame['ranges'])
    angles = np.interp(pixels[:, 0], np.arange(width),
                       np.arctan2(frame['beam_directions'][:, 1], frame['beam_directions'][:, 0]))
    return np.column_stack((ranges, angles))


def fan_points(pixels, frame):
    polar=polar_observations(pixels,frame)
    return np.column_stack((polar[:,0]*np.cos(polar[:,1]),polar[:,0]*np.sin(polar[:,1])))


def fit(source, target):
    a, b = source.mean(0), target.mean(0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    return rotation, b - rotation @ a


def correspondences(previous, current, variant='raw'):
    """Return bounded two-frame intensity tracks before any planar fit.

    Range/azimuth observations retain unresolved elevation. A forward/backward
    pixel test is only an association candidate gate, not calibrated covariance
    or proof of a persistent physical scatterer. Indices belong to this pair;
    they are not persistent landmark IDs across subsequent calls.
    """
    validate_frame(previous);validate_frame(current)
    stamps=[int(f['stamp'][0])*1_000_000_000+int(f['stamp'][1]) for f in (previous,current)]
    if stamps[1]<=stamps[0]:raise ValueError('nonincreasing_acoustic_pair')
    if (not np.array_equal(previous['ranges'],current['ranges']) or
            not np.array_equal(previous['beam_directions'],current['beam_directions'])):
        raise ValueError('acoustic_pair_calibration_changed')
    result=dict(previous_stamp_ns=stamps[0],current_stamp_ns=stamps[1],corners=0,matched=0,
                previous_pixels=[],current_pixels=[],previous_range_azimuth=[],current_range_azimuth=[],
                forward_backward_error_pixels=[])
    a, b = image(previous, variant), image(current, variant)
    mask = np.uint8(previous['ranges'][:, None] > .4) * np.ones_like(a)
    pixels = cv2.goodFeaturesToTrack(a, 300, .04, 5, mask=mask, blockSize=7)
    if pixels is None:
        return dict(result,reason='no_acoustic_corners')
    result['corners']=len(pixels)
    moved, forward, _ = cv2.calcOpticalFlowPyrLK(a, b, pixels, None, winSize=(21, 21), maxLevel=2)
    if moved is None or forward is None:
        return dict(result,reason='optical_flow_unavailable')
    back, backward, _ = cv2.calcOpticalFlowPyrLK(b, a, moved, None, winSize=(21, 21), maxLevel=2)
    if back is None or backward is None:
        return dict(result,reason='reverse_flow_unavailable')
    height, width = a.shape
    inside = lambda p: (np.isfinite(p).all(axis=(1, 2)) & (p >= 0).all(axis=(1, 2))
                        & (p < [width - 1, height - 1]).all(axis=(1, 2)))
    good = (forward.ravel().astype(bool) & backward.ravel().astype(bool)
            & inside(pixels) & inside(moved) & inside(back)
            & (np.linalg.norm(back - pixels, axis=2).ravel() < .5))
    return dict(result,matched=int(good.sum()),
        previous_pixels=pixels[good].reshape(-1,2).tolist(),current_pixels=moved[good].reshape(-1,2).tolist(),
        previous_range_azimuth=polar_observations(pixels[good],previous).tolist(),
        current_range_azimuth=polar_observations(moved[good],current).tolist(),
        forward_backward_error_pixels=np.linalg.norm(back-pixels,axis=2).ravel()[good].tolist())


def estimate(previous, current, variant, *, include_correspondences=False):
    """Fit current-to-previous planar motion using acoustic frames alone."""
    tracks=correspondences(previous,current,variant)
    result=dict(fitted=False,corners=tracks['corners'],matched=tracks['matched'])
    if include_correspondences:result['correspondences']=tracks
    if tracks.get('reason'):return dict(result,reason=tracks['reason'])
    target=fan_points(np.asarray(tracks['previous_pixels']),previous)
    source=fan_points(np.asarray(tracks['current_pixels']),current)
    if len(source) < 10:
        return dict(result, reason='insufficient_acoustic_tracks')
    rng, best = np.random.default_rng(0), np.zeros(len(source), dtype=bool)
    for _ in range(300):
        ids = rng.choice(len(source), 2, replace=False)
        if np.linalg.norm(source[ids[0]] - source[ids[1]]) < .5:
            continue
        rotation, translation = fit(source[ids], target[ids])
        inliers = np.linalg.norm(source @ rotation.T + translation - target, axis=1) < .04
        if inliers.sum() > best.sum():
            best = inliers
    result['inliers'] = int(best.sum())
    if best.sum() < 5:
        return dict(result, reason='insufficient_acoustic_consensus')
    rotation, translation = fit(source[best], target[best])
    residual = np.linalg.norm(source[best] @ rotation.T + translation - target[best], axis=1)
    return dict(result, fitted=True, translation=translation.tolist(),
                inlier_points_current_fan_m=source[best].tolist(),
                yaw_rad=float(np.arctan2(rotation[1, 0], rotation[0, 0])),
                rmse_m=float(np.sqrt(np.mean(residual ** 2))))

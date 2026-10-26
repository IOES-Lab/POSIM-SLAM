"""Experimental bearing/IMU linear startup seed, in the first IMU frame.

Eliminate each unknown landmark from bearing constraints, then solve initial
velocity and gravity using measured preintegrated motion. No scene depth,
simulator pose, stationary assumption or guessed metric scale is supplied.
Biases and relative rotations are conditioned inputs, not estimated here.
This seed requires nonlinear refinement and uncertainty qualification before
use in a live estimator. It does not claim robust outlier rejection.
"""
import numpy as np
from .dvl_scale import rotation, vector


def bearing_imu_seed(times, rotations, delta_positions, tracks, *, camera_to_imu,
                     camera_origin_imu):
    """Solve v(0), g from 4–32 frames and 8–256 multi-view bearing tracks.

    times are seconds relative to the first frame. rotations and delta_positions
    are cumulative raw-IMU preintegrals from that frame, excluding gravity and
    initial velocity. tracks map feature IDs to {frame_index: unit camera ray}.
    Camera extrinsics map camera coordinates into IMU coordinates. All rays
    must already be undistorted. Returned gravity points down; do not pass it
    directly to a specific-force/up-direction seed protocol.
    """
    t = np.asarray(times, dtype=float)
    if (t.ndim != 1 or not 4 <= len(t) <= 32 or not np.isfinite(t).all()
            or t[0] != 0 or np.any(np.diff(t) <= 0) or not .2 <= t[-1] <= 10):
        raise ValueError('visual_seed_times')
    r = np.asarray([rotation(value) for value in rotations])
    alpha = np.asarray(delta_positions, dtype=float)
    if (r.shape != (len(t), 3, 3) or alpha.shape != (len(t), 3)
            or not np.isfinite(alpha).all() or not np.allclose(r[0], np.eye(3), atol=1e-9)
            or not np.allclose(alpha[0], 0, atol=1e-9)):
        raise ValueError('visual_seed_preintegrals')
    mount, lever = rotation(camera_to_imu), vector(camera_origin_imu)
    if not 8 <= len(tracks) <= 256:
        raise ValueError('visual_seed_track_count')
    blocks, targets, landmarks = [], [], []
    for observations in tracks.values():
        if not 3 <= len(observations) <= len(t):
            raise ValueError('visual_seed_track_length')
        h, a, b = [], [], []
        for index, bearing in sorted(observations.items()):
            if type(index) is not int or not 0 <= index < len(t):
                raise ValueError('visual_seed_frame_index')
            ray = vector(bearing)
            if abs(np.linalg.norm(ray)-1) > 1e-6:
                raise ValueError('visual_seed_unit_bearing')
            direction = r[index] @ mount @ ray
            # Two orthonormal tangent rows avoid a redundant third equation.
            _, _, vt = np.linalg.svd(direction.reshape(1, 3), full_matrices=True)
            tangent = vt[1:]
            h.append(tangent)
            a.append(np.hstack((-t[index]*tangent, -.5*t[index]**2*tangent)))
            b.append(tangent @ (alpha[index]+r[index] @ lever))
        h, a, b = np.vstack(h), np.vstack(a), np.concatenate(b)
        u, singular, _ = np.linalg.svd(h, full_matrices=True)
        if singular[-1] <= singular[0]*1e-8:
            raise ValueError('visual_seed_no_parallax')
        null = u[:, 3:].T
        blocks.append(null @ a); targets.append(null @ b)
        landmarks.append((observations, h, a, b))
    a, b = np.vstack(blocks), np.concatenate(targets)
    scales = np.linalg.norm(a, axis=0)
    if np.any(scales < 1e-12):
        raise ValueError('visual_seed_unobservable')
    normalized = a/scales
    u, singular, vt = np.linalg.svd(normalized, full_matrices=False)
    if singular[-1] <= singular[0]*1e-6:
        raise ValueError('visual_seed_unobservable')
    solution = (vt.T @ ((u.T @ b)/singular))/scales
    residual = a @ solution-b
    positive, total = 0, 0
    for observations, h, state, rhs in landmarks:
        point = np.linalg.lstsq(h, rhs-state @ solution, rcond=None)[0]
        for index, bearing in observations.items():
            origin = t[index]*solution[:3]+.5*t[index]**2*solution[3:]+alpha[index]+r[index] @ lever
            positive += int((r[index] @ mount @ bearing) @ (point-origin) > 0)
            total += 1
    return dict(velocity_initial=solution[:3], gravity_initial=solution[3:],
                normalized_singular_values=singular,
                reduced_residual_rms_m=float(np.sqrt(np.mean(residual**2))),
                positive_depth_observations=positive, observations=total,
                bias_estimated=False, covariance_estimated=False,
                observability_qualified=False, scope='linear_seed_requires_refinement')

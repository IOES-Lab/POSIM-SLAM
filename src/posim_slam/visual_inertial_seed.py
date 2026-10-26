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


def tracked_imu_seed(frames, window, *, intrinsics, camera_to_imu,
                     camera_origin_imu, gyro_bias, accel_bias, noise_density):
    """Form an endpoint seed from bounded live frontend replies and raw IMU.

    Intrinsics (fx, fy, cx, cy) describe the processed pinhole image, including
    pixel-centre resizing. No previous VIO states or map coordinates are used.
    The caller waits for sufficient motion and treats this as a provisional
    numerical seed, not an uncertainty-qualified initializer.
    """
    if not 4 <= len(frames) <= 32:
        raise ValueError('visual_seed_frame_count')
    k = np.asarray(intrinsics, dtype=float)
    if k.shape != (4,) or not np.isfinite(k).all() or np.any(k[:2] <= 0):
        raise ValueError('visual_seed_intrinsics')
    stamps, tracks, shape = [], {}, None
    for i, frame in enumerate(frames):
        stamp = frame.get('stamp_ns')
        if (frame.get('type') != 'startup_tracks' or type(stamp) is not int or not 0 < stamp < 2**63
                or (stamps and not 0 < stamp-stamps[-1] <= 500_000_000)):
            raise ValueError('visual_seed_track_clock')
        dimensions = (frame.get('width'), frame.get('height'))
        if (any(type(v) is not int or not 32 <= v <= 1920 for v in dimensions)
                or (shape is not None and dimensions != shape)):
            raise ValueError('visual_seed_track_shape')
        shape = dimensions; stamps.append(stamp)
        observations = frame.get('tracks')
        if not isinstance(observations, list) or len(observations) > 4096:
            raise ValueError('visual_seed_track_capacity')
        seen = set()
        for observation in observations:
            ident = observation.get('id'); pixel = np.asarray(observation.get('pixel'), dtype=float)
            if (type(ident) is not int or ident < 0 or ident in seen or pixel.shape != (2,)
                    or not np.isfinite(pixel).all() or np.any(pixel < 0)
                    or pixel[0] >= shape[0] or pixel[1] >= shape[1]):
                raise ValueError('visual_seed_pixel_identity')
            seen.add(ident)
            ray = np.r_[(pixel-k[2:])/k[:2], 1.]
            tracks.setdefault(ident, {})[i] = ray/np.linalg.norm(ray)
    times = (np.array(stamps, dtype=np.int64)-stamps[0])*1e-9
    if times[-1] > 10:
        raise ValueError('visual_seed_times')
    selected = sorted(tracks.items(), key=lambda item: (-len(item[1]), item[0]))
    tracks = dict([(ident, values) for ident, values in selected
                   if len(values) >= max(3, len(frames)//2)][:256])
    rotations, alpha, dv = [np.eye(3)], [np.zeros(3)], np.zeros(3)
    for start, end in zip(stamps[:-1], stamps[1:]):
        delta = window.preintegrate(start/1e9, end/1e9, gyro_bias=gyro_bias,
                                   accel_bias=accel_bias, noise_density=noise_density)
        alpha.append(alpha[-1]+dv*delta['dt']+rotations[-1] @ delta['delta_position'])
        dv += rotations[-1] @ delta['delta_velocity']
        rotations.append(rotations[-1] @ delta['rotation'])
    result = bearing_imu_seed(times, rotations, alpha, tracks,
        camera_to_imu=camera_to_imu, camera_origin_imu=camera_origin_imu)
    gravity = result['gravity_initial']; magnitude = np.linalg.norm(gravity)
    if not np.isfinite(magnitude) or magnitude < 1:
        raise ValueError('visual_seed_gravity_degenerate')
    result.update(stamp_ns=stamps[-1], first_stamp_ns=stamps[0], last_measurement_ns=stamps[-1],
        up_direction_imu=-rotations[-1].T @ gravity/magnitude,
        velocity_imu=rotations[-1].T @ (result['velocity_initial']+gravity*times[-1]+dv),
        gyro_bias=vector(gyro_bias).copy(), accel_bias=vector(accel_bias).copy(),
        selected_tracks=len(tracks))
    return result


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

"""Bounded bearing/IMU seed consensus; NumPy only, no pose or depth oracle.

Known gravity magnitude is refined on its two-dimensional tangent plane, as in
VINS-Mono section V-B-3 (Qin et al., arXiv:1708.03852). This implementation uses
our bearing-eliminated system and contains no VINS-Mono source code.
"""
import numpy as np


def _track_groups(parts, rotations):
    """Batch equal-length tracks without padding or changing their observations."""
    lengths = {}
    for i, part in enumerate(parts):
        lengths.setdefault(len(part[3]), []).append(i)
    groups = []
    for ids in lengths.values():
        pinv, state, rhs, indices, pixels = map(np.stack, zip(*(parts[i] for i in ids)))
        groups.append((np.array(ids), pinv, state, rhs, indices, pixels, rotations[indices]))
    return groups


def _reprojection_errors(groups, origins, solution, mount, focal, radius, count):
    errors = np.full(count, radius)
    for ids, pinv, state, rhs, indices, pixels, rotations in groups:
        point = (pinv @ (rhs-(state @ solution))[..., None])[..., 0]
        camera = np.einsum('nlji,nlj->nli', rotations, point[:, None, :]-origins[indices]) @ mount
        front = np.all(camera[:, :, 2] > 0, axis=1)
        # Back-facing tracks keep the same capped error as the scalar rule.
        valid = np.flatnonzero(front)
        projected = camera[valid]
        residual = (projected[:, :, :2]/projected[:, :, 2, None]-pixels[valid])*focal
        finite = np.isfinite(residual).all(axis=(1, 2))
        errors[ids[valid[finite]]] = np.median(np.linalg.norm(residual[finite], axis=2), axis=1)
    return errors


def linear_solution(a, b):
    scales = np.linalg.norm(a, axis=0)
    if np.any(scales < 1e-12):
        raise ValueError('visual_seed_unobservable')
    u, singular, vt = np.linalg.svd(a/scales, full_matrices=False)
    if len(singular) != a.shape[1] or singular[-1] <= singular[0]*1e-6:
        raise ValueError('visual_seed_unobservable')
    return (vt.T @ ((u.T @ b)/singular))/scales, singular


def fixed_gravity_solution(a, b, magnitude):
    solution, _ = linear_solution(a, b)
    norm = np.linalg.norm(solution[3:])
    if norm < 1e-12:
        raise ValueError('visual_seed_gravity_degenerate')
    gravity = solution[3:]*magnitude/norm
    for _ in range(64):
        _, _, vt = np.linalg.svd(gravity.reshape(1, 3), full_matrices=True)
        tangent = vt[1:].T
        design = np.column_stack((a[:, :3], a[:, 3:] @ tangent))
        step, _ = linear_solution(design, b-a[:, 3:] @ gravity)
        updated = gravity+tangent @ step[3:]
        updated *= magnitude/np.linalg.norm(updated)
        change = np.linalg.norm(updated-gravity)
        gravity = updated
        if change <= magnitude*1e-10:
            velocity = np.linalg.lstsq(a[:, :3], b-a[:, 3:] @ gravity, rcond=None)[0]
            return np.r_[velocity, gravity]
    raise ValueError('visual_seed_refinement_failed')


def consensus_solution(t, rotations, alpha, mount, lever, blocks, targets, landmarks,
                       *, gravity_magnitude, focal_lengths, pixel_sigma):
    """RANSAC over track groups, then refit the agreeing multi-view tracks.

    Each group has at least three views. Three sampled groups supply at least
    nine reduced rows for the six linear unknowns; rank is checked separately.
    A track agrees when its median reprojection is within three pixel sigmas
    and its triangulated point lies in front of every observing camera.
    Consensus estimates a numerical seed, never an uncertainty certificate.
    """
    if (type(gravity_magnitude) not in (int, float) or not np.isfinite(gravity_magnitude)
            or gravity_magnitude <= 0 or type(pixel_sigma) not in (int, float)
            or not np.isfinite(pixel_sigma) or pixel_sigma <= 0):
        raise ValueError('visual_seed_robust_calibration')
    focal = np.asarray(focal_lengths, dtype=float)
    if focal.shape != (2,) or not np.isfinite(focal).all() or np.any(focal <= 0):
        raise ValueError('visual_seed_robust_calibration')
    parts = []
    for observations, h, state, rhs in landmarks:
        indices = np.array(sorted(observations))
        rays = np.array([observations[i] for i in indices])
        if np.any(rays[:, 2] <= 0):
            raise ValueError('visual_seed_forward_bearing')
        parts.append((np.linalg.pinv(h), state, rhs, indices, rays[:, :2]/rays[:, 2, None]))
    groups = _track_groups(parts, rotations)
    radius = 3*pixel_sigma

    def solve(indices):
        a = np.vstack([blocks[i] for i in indices])
        b = np.concatenate([targets[i] for i in indices])
        return fixed_gravity_solution(a, b, gravity_magnitude)

    def score(solution):
        origins = (t[:, None]*solution[:3]+.5*t[:, None]**2*solution[3:]+alpha
                   + np.einsum('nij,j->ni', rotations, lever))
        errors = _reprojection_errors(groups, origins, solution, mount, focal, radius, len(parts))
        mask = errors < radius
        return (int(mask.sum()), -float(np.minimum(errors, radius).sum())), mask, errors

    # Fixed generator makes identical sensor windows reproducible. All work is
    # bounded independently of scene identity, simulator time or accepted truth.
    rng = np.random.default_rng(0)
    best = None
    for _ in range(256):
        try:
            solution = solve(rng.choice(len(parts), 3, replace=False))
        except ValueError as error:
            if str(error) not in ('visual_seed_unobservable', 'visual_seed_gravity_degenerate',
                                  'visual_seed_refinement_failed'):
                raise
            continue
        rank, mask, errors = score(solution)
        if best is None or rank > best[0]:
            best = rank, solution, mask, errors
    if best is None or best[0][0] < 8:
        raise ValueError('visual_seed_no_consensus')
    for _ in range(8):
        try:
            solution = solve(np.flatnonzero(best[2]))
        except ValueError:
            break
        rank, mask, errors = score(solution)
        if rank <= best[0]:
            break
        best = rank, solution, mask, errors
    selected = np.flatnonzero(best[2])
    _, singular = linear_solution(np.vstack([blocks[i] for i in selected]),
                                  np.concatenate([targets[i] for i in selected]))
    return best[1], singular, dict(
        seed_method='bearing_imu_ransac_fixed_gravity', inlier_tracks=int(len(selected)),
        rejected_tracks=int(len(parts)-len(selected)), ransac_hypotheses=256,
        gravity_magnitude_m_s2=float(gravity_magnitude), pixel_sigma=float(pixel_sigma),
        inlier_radius_px=float(radius),
        median_inlier_reprojection_px=float(np.median(best[3][best[2]])))

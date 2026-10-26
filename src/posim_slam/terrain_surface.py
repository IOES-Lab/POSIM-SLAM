"""Bounded, supported point-to-plane registration for measured terrain samples.

Inputs must already be XYZ observations with an explicit sensor model. This
module does not resolve an imaging sonar's unmeasured elevation. Reference
points come only from earlier frames; local planes require support from several
frames and interpolation inside their tangent-plane convex hull. The returned
information spectrum is conditional on those correspondences, not a calibrated
pose covariance or a navigation readiness decision.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class SurfaceConfig:
    neighborhood_m: float = .75
    surface_sigma_m: float = .05
    minimum_spread_m: float = .05
    maximum_neighbors: int = 24
    maximum_per_frame: int = 4
    minimum_frames: int = 3
    maximum_reference: int = 4096
    maximum_query: int = 128
    iterations: int = 10


def _points(value, maximum):
    points = np.asarray(value, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) > maximum
            or not np.isfinite(points).all()):
        raise ValueError('invalid_or_excessive_surface_points')
    return points


def _config(config):
    if (not np.isfinite([config.neighborhood_m, config.surface_sigma_m,
                         config.minimum_spread_m]).all()
            or not 0 < config.surface_sigma_m < config.neighborhood_m
            or not 0 < config.minimum_spread_m < config.neighborhood_m
            or any(type(getattr(config, k)) is not int for k in (
                'maximum_neighbors', 'maximum_per_frame', 'minimum_frames',
                'maximum_reference', 'maximum_query', 'iterations'))
            or not 6 <= config.maximum_neighbors <= 64
            or not 1 <= config.maximum_per_frame <= config.maximum_neighbors
            or not 3 <= config.minimum_frames <= config.maximum_neighbors
            or not 6 <= config.maximum_reference <= 8192
            or not 3 <= config.maximum_query <= 256
            or not 1 <= config.iterations <= 20):
        raise ValueError('invalid_surface_config')


def _contains_origin(points):
    """Convex hull containment in 2D; no bounding-box extrapolation."""
    values = sorted(set(map(tuple, points)))
    if len(values) < 3:
        return False
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower = []
    upper = []
    for sequence, hull in ((values, lower), (values[::-1], upper)):
        for value in sequence:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], value) <= 0:
                hull.pop()
            hull.append(value)
    hull = lower[:-1]+upper[:-1]
    return len(hull) >= 3 and all(
        cross(a, b, (0., 0.)) >= -1e-12
        for a, b in zip(hull, hull[1:]+hull[:1]))


def surface_matches(query, reference, reference_frames, *, query_frame,
                    config=SurfaceConfig()):
    """Return local plane residuals supported by strictly earlier frames.

    A frame has bounded influence on each plane. Same-frame and future samples
    are errors, not silently accepted reference support. No persistent point
    identity is assigned to successive sonar aperture boundaries.
    """
    _config(config)
    query = _points(query, config.maximum_query)
    reference = _points(reference, config.maximum_reference)
    frames = np.asarray(reference_frames)
    if (frames.shape != (len(reference),) or not np.issubdtype(frames.dtype, np.integer)
            or not isinstance(query_frame, (int, np.integer))
            or np.any(frames >= query_frame)):
        raise ValueError('strictly_prior_surface_frames_required')
    selected = []
    normals = []
    centers = []
    support = []
    reasons = dict(no_neighbors=0, insufficient_frames=0, line_support=0,
                   nonplanar=0, outside_support=0, distance=0)
    for index, point in enumerate(query):
        squared = np.sum((reference-point)**2, axis=1)
        order = np.flatnonzero(squared <= config.neighborhood_m**2)
        order = order[np.argsort(squared[order], kind='stable')]
        counts = {}
        ids = []
        for candidate in order:
            frame = int(frames[candidate])
            if counts.get(frame, 0) >= config.maximum_per_frame:
                continue
            counts[frame] = counts.get(frame, 0)+1
            ids.append(candidate)
            if len(ids) == config.maximum_neighbors:
                break
        if len(ids) < 6:
            reasons['no_neighbors'] += 1
            continue
        if len(counts) < config.minimum_frames:
            reasons['insufficient_frames'] += 1
            continue
        neighborhood = reference[ids]
        center = neighborhood.mean(axis=0)
        eigenvalues, axes = np.linalg.eigh((neighborhood-center).T @ (neighborhood-center)/len(ids))
        if eigenvalues[1] < config.minimum_spread_m**2:
            reasons['line_support'] += 1
            continue
        if eigenvalues[0] > config.surface_sigma_m**2:
            reasons['nonplanar'] += 1
            continue
        normal = axes[:, 0]
        if not _contains_origin((neighborhood-point) @ axes[:, 1:]):
            reasons['outside_support'] += 1
            continue
        if abs((point-center) @ normal) > 3*config.surface_sigma_m:
            reasons['distance'] += 1
            continue
        selected.append(index)
        centers.append(center)
        normals.append(normal)
        support.append(len(counts))
    ids = np.asarray(selected, dtype=int)
    normals = np.asarray(normals).reshape(-1, 3)
    centers = np.asarray(centers).reshape(-1, 3)
    return dict(indices=ids, normals=normals, centers=centers,
                residuals=np.sum((query[ids]-centers)*normals, axis=1),
                support_frames=np.asarray(support, dtype=int), rejections=reasons)


def _rotation(vector):
    angle = np.linalg.norm(vector)
    if angle < 1e-14:
        return np.eye(3)
    x, y, z = vector/angle
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


def register_surface(query, reference, reference_frames, *, query_frame, origin,
                     config=SurfaceConfig()):
    """Estimate a bounded local rigid correction; leave numerical null modes zero.

    Point-to-plane ICP uses Huber loss and rematches at each iteration. Rotation
    is normalized by RMS sensor range before SVD so meters/radians do not define
    the rank test. The 1e-6 cutoff is numerical only; noisy apparent modes are
    not evidence of physical observability. No odometry prior is invented.
    """
    _config(config)
    query = _points(query, config.maximum_query)
    origin = np.asarray(origin, dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('invalid_surface_origin')
    identity = dict(rotation=np.eye(3), translation=np.zeros(3), accepted=False,
                    map_qualified=False, navigation_ready=False)
    if len(query) < 6:
        return dict(identity, reason='insufficient_query', matches=0)
    lever = query-origin
    scale = max(config.neighborhood_m, float(np.sqrt(np.mean(np.sum(lever**2, axis=1)))))
    rotation = np.eye(3)
    translation = np.zeros(3)
    history = []
    converged = False
    sigma = config.surface_sigma_m
    def loss(residual):
        absolute = abs(residual)
        return float(np.sum(np.where(absolute <= sigma, .5*residual**2,
                                     sigma*(absolute-.5*sigma))))
    for iteration in range(config.iterations):
        moved = lever@rotation.T+origin+translation
        match = surface_matches(moved, reference, reference_frames,
                                query_frame=query_frame, config=config)
        ids, normal, residual = match['indices'], match['normals'], match['residuals']
        if len(ids) < 6:
            return dict(identity, reason='insufficient_supported_overlap', matches=len(ids),
                        rejections=match['rejections'], history=history)
        weight = np.sqrt(np.minimum(1., sigma/np.maximum(abs(residual), 1e-15)))
        jacobian = np.column_stack([normal, np.cross(lever[ids]@rotation.T, normal)/scale])
        u, singular, vt = np.linalg.svd(jacobian*weight[:, None], full_matrices=False)
        keep = singular > 1e-6*singular[0]
        delta = -vt[keep].T@((u[:, keep].T@(residual*weight))/singular[keep])
        before = loss(residual)
        improved = False
        for fraction in (1., .5, .25, .125):
            dr = _rotation(fraction*delta[3:]/scale)
            candidate_rotation = dr@rotation
            candidate_translation = translation+fraction*delta[:3]
            angle = math.acos(float(np.clip((np.trace(candidate_rotation)-1)/2, -1, 1)))
            if (np.linalg.norm(candidate_translation) > config.neighborhood_m/2
                    or angle > math.radians(10)):
                continue
            candidate = lever[ids]@candidate_rotation.T+origin+candidate_translation
            after = loss(np.sum((candidate-match['centers'])*normal, axis=1))
            if after <= before+1e-15:
                rotation, translation = candidate_rotation, candidate_translation
                improved = True
                break
        history.append(dict(iteration=iteration, matches=len(ids), rank=int(keep.sum()),
                            singular_values=singular.tolist(), loss_before=before,
                            loss_after=after if improved else before,
                            rejections=match['rejections']))
        if not improved:
            return dict(identity, reason='surface_step_outside_trust_region',
                        matches=len(ids), history=history)
        if np.linalg.norm(fraction*delta) < 1e-6:
            converged = True
            break
    if not converged:
        return dict(identity, reason='surface_iteration_limit', matches=len(ids), history=history)
    # A result must retain valid support after the last correction as well.
    moved = lever@rotation.T+origin+translation
    final = surface_matches(moved, reference, reference_frames,
                            query_frame=query_frame, config=config)
    if len(final['indices']) < 6:
        return dict(identity, reason='lost_surface_support', matches=len(final['indices']), history=history)
    return dict(rotation=rotation, translation=translation, accepted=True,
                reason='conditional_surface_registration', matches=len(final['indices']),
                residual_rmse_m=float(np.sqrt(np.mean(final['residuals']**2))),
                history=history, map_qualified=False, navigation_ready=False)

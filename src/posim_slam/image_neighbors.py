"""Select overlapping images from observed tracks and estimated 3D geometry.

This is the conventional OpenSfM shared-track/parallax rule, independently
implemented with NumPy. Inputs use one consistent estimated frame. Every input
view appears in the result, including views without sufficient support. Viewer
point limits and fixed temporal strides do not select reconstruction inputs.
"""
from dataclasses import dataclass
import math
from numbers import Integral
from typing import Mapping, Iterable

import numpy as np


@dataclass(frozen=True)
class ImageNeighbor:
    view_id: int
    shared_tracks: int
    baseline_m: float


def _identifier(value):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError('nonnegative_integer_image_or_track_id_required')
    return int(value)


def _point(value):
    value = np.asarray(value, dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError('finite_three_dimensional_geometry_required')
    return value


def image_neighbors(
    camera_centers: Mapping[int, Iterable[float]],
    observed_tracks: Mapping[int, Iterable[int]],
    landmark_positions: Mapping[int, Iterable[float]],
    *,
    max_neighbors: int = 7,
    minimum_shared_tracks: int = 20,
    minimum_parallax_deg: float = 3.,
    maximum_parallax_deg: float = 30.,
) -> dict[int, tuple[ImageNeighbor, ...]]:
    """Rank views by common observed tracks with strictly bounded parallax.

    More than ``minimum_shared_tracks`` must have known estimated positions and
    lie strictly between the two angle limits. Unknown landmarks contribute no
    score. IDs resolve score ties deterministically. The caller supplies actual
    inlier observations; a low reprojection residual alone is not observability.
    This graph does not certify depth, map completeness or estimator accuracy.
    """
    if (type(max_neighbors) is not int or max_neighbors < 1
            or type(minimum_shared_tracks) is not int or minimum_shared_tracks < 0):
        raise ValueError('positive_neighbor_limit_and_nonnegative_track_threshold_required')
    angles = (minimum_parallax_deg, maximum_parallax_deg)
    if (any(isinstance(v, bool) or not isinstance(v, (float, int))
            or not math.isfinite(v) for v in angles)
            or not 0 <= minimum_parallax_deg < maximum_parallax_deg <= 180):
        raise ValueError('ordered_finite_parallax_limits_required')
    centers = {_identifier(k): _point(v) for k, v in camera_centers.items()}
    tracks = {_identifier(k): {_identifier(t) for t in v}
              for k, v in observed_tracks.items()}
    if set(centers) != set(tracks):
        raise ValueError('exact_image_time_observations_required')
    points = {_identifier(k): _point(v) for k, v in landmark_positions.items()}
    lower, upper = np.deg2rad(angles)
    result = {}
    for view in sorted(centers):
        origin = centers[view]
        ranked = []
        for other in sorted(centers):
            if other == view:
                continue
            baseline = float(np.linalg.norm(centers[other] - origin))
            if baseline == 0:
                continue
            shared = sorted((tracks[view] & tracks[other]) & points.keys())
            if len(shared) <= minimum_shared_tracks:
                continue
            positions = np.stack([points[t] for t in shared])
            a, b = positions - origin, positions - centers[other]
            length = np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)
            valid = length > 1e-12
            cosine = np.full(len(shared), np.nan)
            cosine[valid] = (a[valid] * b[valid]).sum(axis=1) / length[valid]
            parallax = np.arccos(np.clip(cosine, -1, 1))
            count = int(((parallax > lower) & (parallax < upper)).sum())
            if count > minimum_shared_tracks:
                ranked.append(ImageNeighbor(other, count, baseline))
        ranked.sort(key=lambda n: (-n.shared_tracks, n.view_id))
        result[view] = tuple(ranked[:max_neighbors])
    return result

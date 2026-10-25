"""Conditional spatial surface-error covariance for raw ground-entry ranges.

A zero-mean stationary Gaussian surface offset is shared by nearby measured
footprints. Linearizing the ray/plane intersection maps surface error to range
with 1/(normal dot ray). Reference geometry must stay fixed throughout a group;
this model is not a terrain measurement, pose prior or calibrated sensor model.
"""
import numpy as np


def surface_range_covariance(points, incidence, *, sigma_m, length_m):
    p = np.asarray(points, dtype=float)
    a = np.asarray(incidence, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 3 or not 8 <= len(p) <= 512
            or a.shape != (len(p),) or not np.isfinite(p).all()
            or not np.isfinite(a).all() or np.any(abs(a) < 1e-4)
            or np.any(abs(a) > 1 + 1e-8)
            or not np.isfinite([sigma_m, length_m]).all()
            or not 0 <= sigma_m <= .2 or not .05 <= length_m <= 5):
        raise ValueError('invalid_ground_surface_covariance')
    difference = p[:, None, :] - p[None, :, :]
    kernel = np.exp(-np.sum(difference * difference, axis=2) / (2 * length_m**2))
    scale = sigma_m / a
    return kernel * scale[:, None] * scale[None, :]

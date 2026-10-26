"""Conditional source-magnitude bounds from mixed complex beam magnitudes.

For y = Hx, x_i is a sum of terms (H^-1)_ij y_j with unknown phases.
Polygon inequalities bound its magnitude without inventing those phases. These
are deterministic intervals conditional on H and measurement-error intervals,
not confidence intervals. Bounds for different source beams are not independent.
A zero lower bound means unresolved contribution, never observed free space.
"""
from __future__ import annotations
import numpy as np


class BeamPhaseBounds:
    """Bound selected source beams; retain the calibration outside the estimator.

    H must describe the complex-amplitude mixing at the supplied range samples.
    Clipped images, an unknown device response or nonlinear preprocessing do not
    satisfy this contract. Range sidelobes, elevation and object identity remain
    unresolved. A positive lower bound alone is not a geometric landmark.
    """
    def __init__(self, mixing):
        matrix = np.asarray(mixing, dtype=np.complex128)
        if (matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]
                or not 1 <= matrix.shape[0] <= 512 or not np.isfinite(matrix).all()):
            raise ValueError('invalid_complex_beam_mixing')
        condition = float(np.linalg.cond(matrix))
        if not np.isfinite(condition) or condition > 1e8:
            raise ValueError('ill_conditioned_complex_beam_mixing')
        inverse = np.linalg.inv(matrix)
        if np.linalg.norm(inverse@matrix-np.eye(len(matrix)), ord=np.inf) > 1e-8:
            raise ValueError('inaccurate_complex_beam_inverse')
        self._weights = np.abs(inverse)
        self._weights.setflags(write=False)
        self.beams = len(matrix)
        self.condition = condition

    def query(self, magnitudes, source_beams, *, relative_error=0., absolute_error=0.):
        """One selected source beam per row of measured output magnitudes.

        Each actual output magnitude lies in max(0, m*(1-relative)-absolute)
        through m*(1+relative)+absolute. Limits: 4,096 rows and 512 beams.
        No threshold is applied to the returned lower/upper arrays.
        """
        values = np.asarray(magnitudes, dtype=float)
        indices = np.asarray(source_beams)
        if (values.ndim != 2 or values.shape[1] != self.beams
                or not 1 <= len(values) <= 4096 or not np.isfinite(values).all()
                or np.any(values < 0) or indices.shape != (len(values),)
                or indices.dtype.kind not in 'iu' or np.any(indices < 0)
                or np.any(indices >= self.beams)):
            raise ValueError('invalid_beam_magnitude_query')
        if (not np.isscalar(relative_error) or not np.isscalar(absolute_error)
                or not np.isfinite([relative_error, absolute_error]).all()
                or not 0 <= relative_error < 1 or absolute_error < 0):
            raise ValueError('invalid_beam_magnitude_error_interval')
        weights = self._weights[indices]
        with np.errstate(over='ignore', invalid='ignore'):
            lo = weights*np.maximum(0, values*(1-relative_error)-absolute_error)
            hi = weights*(values*(1+relative_error)+absolute_error)
            upper = hi.sum(axis=1)
            # One phasor can exceed the largest possible opposing sum. If none
            # does, the polygon can close and zero remains a possible source.
            lower = np.maximum(0, (lo+hi).max(axis=1)-upper)
        if not np.isfinite(lower).all() or not np.isfinite(upper).all():
            raise ValueError('beam_magnitude_interval_overflow')
        return dict(lower=lower, upper=upper)

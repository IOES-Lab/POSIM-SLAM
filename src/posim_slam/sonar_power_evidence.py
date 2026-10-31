# SPDX-License-Identifier: Apache-2.0
"""Profile common reflectivity with correlated acoustic power scores.

The exponential marginal mean score avoids inverting a nearly singular pixel
covariance. Its sensitivity and full score covariance are kept separately;
the conditional sandwich covariance retains correlations and unobservable
directions. This is source-model evidence, not a calibrated image likelihood,
native factor, physical correspondence or joint navigation/map covariance.
"""
import numpy as np


def profile_power_evidence(observed, mean, covariance, *, log_mean_jacobian=None):
    """Profile one positive common power gain, with supplied unit-gain moments.

    Inputs describe the same statistic: for an average of independent fixed-
    geometry frames, supply its mean and single-frame covariance divided by the
    actual frame count. Gain scales the covariance quadratically. Geometry,
    material grouping and any log-mean Jacobian are caller hypotheses. Returned
    score information conditions on these inputs and excludes their uncertainty.
    No pixel-independence assumption is used for the score covariance.
    """
    p, m, c = (np.asarray(x) for x in (observed, mean, covariance))
    if (p.ndim != 1 or not 1 <= len(p) <= 256 or m.shape != p.shape
            or c.shape != (len(p), len(p))
            or any(x.dtype.kind not in 'fiu' or not np.isfinite(x).all() for x in (p, m, c))
            or np.any(p < 0) or np.any(m < 0)):
        raise ValueError('bounded_nonnegative_power_moments_required')
    p, m, c = (x.astype(float, copy=False) for x in (p, m, c))
    positive = m > 0
    if (np.any(p[~positive] != 0) or np.any(c[~positive] != 0)
            or np.any(c[:, ~positive] != 0)):
        raise ValueError('power_outside_model_support')
    if not positive.any():
        raise ValueError('no_model_power_support')
    selected = c[np.ix_(positive, positive)]
    with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
        relative = selected/m[positive, None]/m[None, positive]
        ratios = p[positive]/m[positive]
    if not np.isfinite(relative).all() or not np.isfinite(ratios).all():
        raise ValueError('power_scaling_overflow')
    scale = np.max(abs(relative))
    tolerance = 64*np.finfo(float).eps*len(relative)*scale
    if (np.any(np.diag(relative) <= 0)
            or np.max(abs(relative-relative.T)) > tolerance
            or np.linalg.eigvalsh((relative+relative.T)/2).min() < -tolerance):
        raise ValueError('positive_semidefinite_power_covariance_required')
    relative = (relative+relative.T)/2
    gain = float(ratios.mean())
    if gain <= 0 or not np.isfinite(gain):
        raise ValueError('positive_common_gain_unavailable')
    residual = np.zeros(len(p))
    residual[positive] = ratios/gain-1
    result = dict(gain=gain, normalized_residual=residual,
                  supported_pixels=int(positive.sum()), zero_support_pixels=int((~positive).sum()),
                  composite_log_score=float(np.log(m[positive]).sum()
                                             + positive.sum()*(np.log(gain)+1)))
    if log_mean_jacobian is None:
        return result
    j = np.asarray(log_mean_jacobian)
    if (j.ndim != 2 or j.shape[0] != len(p) or not 1 <= j.shape[1] <= 32
            or j.dtype.kind not in 'fiu' or not np.isfinite(j).all()):
        raise ValueError('bounded_log_mean_jacobian_required')
    j = j[positive].astype(float, copy=False)
    # Remove a common offset before the mean to retain exactly constant gain
    # directions even when their decimal value is not exactly representable.
    difference = j-j[0]
    centered = difference-difference.mean(axis=0)
    _, singular, vt = np.linalg.svd(centered, full_matrices=True)
    cutoff = (64*np.finfo(float).eps*max(centered.shape)*singular[0]
              if len(singular) else 0.)
    rank = int(np.count_nonzero(singular > cutoff))
    basis, null = vt[:rank].T, vt[rank:].T
    sensitivity = centered.T@centered
    score_covariance = centered.T@relative@centered
    if rank:
        inverse = 1/singular[:rank]**2
        reduced = basis.T@score_covariance@basis
        conditional = inverse[:, None]*reduced*inverse[None, :]
    else:
        conditional = np.empty((0, 0))
    result.update(shape_rank=rank, identifiable_basis=basis, nullspace_basis=null,
                  score=centered.T@residual[positive], sensitivity=sensitivity,
                  score_covariance=score_covariance,
                  conditional_identifiable_covariance=conditional)
    if not all(np.isfinite(v).all() for v in result.values()):
        raise ValueError('power_score_overflow')
    return result

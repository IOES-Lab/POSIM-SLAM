# SPDX-License-Identifier: Apache-2.0
"""Conditional four-cone mean fit; unknown range uncertainty remains unknown."""
import numpy as np
from scipy.optimize import least_squares

def plane_from_ranges(packet, calibration):
    """Unweighted finite-cone fit. Supplied DVL range variances are unknown."""
    model = calibration['bottom_ranges']
    axes = {row['id']: row for row in model['beams']}
    rows = [row for row in packet['bottom_ranges']['beams'] if row['locked']]
    if len(rows) < 4:
        return dict(available=False, reason='four_locked_ranges_required')
    rays = np.array([axes[row['id']]['axis'] for row in rows])
    half = np.array([axes[row['id']]['aperture_rad']/2 for row in rows])
    observed = np.array([row['range_m'] for row in rows])

    def prediction(q):
        norm = np.linalg.norm(q)
        cosine = np.clip(rays @ q / norm, -1, 1)
        maximum = np.where(cosine >= np.cos(half), 1,
                           cosine*np.cos(half) + np.sqrt(1-cosine**2)*np.sin(half))
        return 1 / (norm*maximum)

    start = np.mean(rays, axis=0)
    start /= np.linalg.norm(start)
    start /= np.median(observed) * .87
    fit = least_squares(lambda q: prediction(q)-observed, start,
                        xtol=1e-12, ftol=1e-12, gtol=1e-12, max_nfev=100)
    q = fit.x
    residual = prediction(q)-observed
    bounded = bool(np.all((rays @ q / np.linalg.norm(q)) > np.sin(half)))
    return dict(available=bool(fit.success and bounded), q=q, observed=observed,
                predicted=prediction(q), residual=residual,
                range_variances_available=False, local_plane_extrapolated=True)

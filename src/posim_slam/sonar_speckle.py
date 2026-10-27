# SPDX-License-Identifier: Apache-2.0
"""Conditional intensity moments for proper complex-Gaussian scattering.

For z=H xi, E[xi xi*]=diag(q), K=H diag(q) H*, the intensity mean is
diag(K), and its covariance is |K|**2. Nonnegative weighted pixel means
retain these correlations. These are source-model moments, not an independent
pixel likelihood, pose covariance or hardware noise calibration.
"""
import numpy as np


def power_moments(response, source_power, weights=None):
    h, q = np.asarray(response), np.asarray(source_power)
    if (h.ndim != 2 or h.dtype.kind not in 'fciu' or not np.isfinite(h).all()
            or not 1 <= h.shape[0] <= 256 or not 1 <= h.shape[1] <= 65536
            or h.size > 4_194_304 or q.shape != (h.shape[1],)
            or q.dtype.kind not in 'fiu' or not np.isfinite(q).all() or np.any(q < 0)):
        raise ValueError('bounded_complex_response_and_nonnegative_power_required')
    w = np.eye(h.shape[0]) if weights is None else np.asarray(weights)
    if (w.ndim != 2 or not 1 <= w.shape[0] <= 256 or w.shape[1] != h.shape[0]
            or w.dtype.kind not in 'fiu' or not np.isfinite(w).all() or np.any(w < 0)
            or not np.allclose(w.sum(axis=1), 1., rtol=0, atol=1e-12)):
        raise ValueError('normalized_nonnegative_pixel_averages_required')
    # Native float32 inputs can have finite means but overflow fourth moments.
    # Accumulate both complex amplitudes and power moments in double precision.
    w = w.astype(float, copy=False)
    with np.errstate(over='ignore', invalid='ignore'):
        a = h.astype(np.complex128, copy=False) * np.sqrt(q.astype(float, copy=False))
        k = a @ a.conj().T
        mean = w @ np.diag(k).real
        covariance = w @ (abs(k) ** 2) @ w.T
    if not all(np.isfinite(v).all() for v in (k, mean, covariance)):
        raise ValueError('finite_conditional_power_moments_required')
    variance = np.diag(covariance)
    looks = np.divide(mean ** 2, variance, out=np.zeros_like(mean), where=variance > 0)
    return dict(mean=mean, covariance=covariance, equivalent_looks=looks,
                complex_covariance=k)


def periodic_hamming_amplitude(delay_bins, bins):
    """Exact finite Fourier sum of the source's shifted periodic Hamming window.

    Dividing by sqrt(bins) matches SurfacePowerModel's power convention. The
    common range-dependent phase omitted here cancels from H diag(q) H*.
    """
    d = np.asarray(delay_bins)
    if (type(bins) is not int or not 2 <= bins <= 1024
            or d.dtype.kind not in 'fiu' or not np.isfinite(d).all()
            or d.size > 4_194_304):
        raise ValueError('bounded_finite_source_delays_required')

    def dirichlet(x):
        x = (x + bins/2) % bins - bins/2
        return bins * np.sinc(x) / np.sinc(x/bins) * np.exp(1j*np.pi*(bins-1)*x/bins)

    f = np.arange(bins)
    norm = np.linalg.norm(.54-.46*np.cos(2*np.pi*(f+1)/bins))
    return (.54*dirichlet(d) - .23*(np.exp(2j*np.pi/bins)*dirichlet(d+1)
                                  + np.exp(-2j*np.pi/bins)*dirichlet(d-1))) / (norm*np.sqrt(bins))


def surface_pixel_moments(observation, reflectivity, pixels, weights=None):
    """Moments of selected native (range, beam) pixels for frozen surfaces.

    Uses exact finite Fourier amplitudes, with full complex pixel correlation.
    Unlike the mean renderer, no sub-bin power interpolation is needed. Pixels
    are selected by the diagnostic caller; input geometry remains conditional.
    """
    m = observation.model
    mu, pixels = np.asarray(reflectivity), np.asarray(pixels)
    if (mu.shape != (observation.size,) or mu.dtype.kind not in 'fiu'
            or not np.isfinite(mu).all() or np.any(mu < 0)
            or pixels.ndim != 2 or pixels.shape[1] != 2 or pixels.dtype.kind not in 'iu'
            or not 1 <= len(pixels) <= 256 or np.any(pixels < 0)
            or np.any(pixels[:, 0] >= len(m.native_ranges_m))
            or np.any(pixels[:, 1] >= m.nbeams)):
        raise ValueError('bounded_native_pixels_and_surface_reflectivity_required')
    if not len(observation.selected_ranges):
        # One zero-power source represents an empty ray set without allocating H.
        return power_moments(np.zeros((len(pixels), 1)), np.zeros(1), weights)
    if len(pixels) * len(observation.selected_ranges) > 4_194_304:
        raise MemoryError('conditional_pixel_response_budget')
    az = m.azimuth_rad
    beam = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:, None]-az)))
    beam /= np.linalg.norm(beam)
    angular = beam[pixels[:, 1, None], observation.beam_ids]
    step = m.native_ranges_m[1]-m.native_ranges_m[0]
    delay = observation.selected_ranges[None, :]/step-pixels[:, 0, None]
    h = angular * periodic_hamming_amplitude(delay, len(m.native_ranges_m))
    q = mu[observation.triangle_ids] * observation.unit_power
    return power_moments(h, q, weights)

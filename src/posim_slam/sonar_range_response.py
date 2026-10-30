"""POSIM's sampled frequency aperture and conditional diffuse-return moments.

These source-specific diagnostics require supplied scatterer ranges/variances.
They do not recover geometry, physical correspondence or calibrated uncertainty
from an image. Independent circular complex Gaussian scatterers give correlated
range/beam powers; the actual GPU also has finite-precision accumulation.
"""
import numpy as np


class PosimRangeResponse:
    """Original-length FFT of the producer's normalized periodic Hamming window.

    Complex outputs use [query, range]. Distances are not relabelled as bins.
    The periodic transfer alone has no visibility cutoff; mean_power and
    power_moments apply the explicit source maximum to supplied scatterers.
    """
    def __init__(self, samples, bandwidth_hz, sound_speed_m_s):
        if (type(samples) is not int or not 16 <= samples <= 4096
                or not np.isfinite([bandwidth_hz, sound_speed_m_s]).all()
                or bandwidth_hz <= 0 or sound_speed_m_s <= 0):
            raise ValueError('invalid_range_response_calibration')
        self.samples = samples
        self.bandwidth_hz = float(bandwidth_hz)
        self.sound_speed_m_s = float(sound_speed_m_s)
        f = np.arange(samples, dtype=float)
        self._window = .54 - .46 * np.cos(2*np.pi*(f+1)/samples)
        self._window /= np.linalg.norm(self._window)
        offset = -samples/2+1 if samples % 2 == 0 else -(samples-1)/2+1
        self._frequency = (f+offset)*bandwidth_hz/samples

    def complex_response(self, distances_m):
        distances = np.asarray(distances_m, dtype=float)
        if (distances.ndim != 1 or not len(distances) or distances.size*self.samples > 2_000_000
                or not np.isfinite(distances).all() or np.any(distances < 0)):
            raise ValueError('invalid_range_response_query')
        try:
            with np.errstate(over='raise', invalid='raise'):
                phase = 4*np.pi*distances[:, None]*self._frequency/self.sound_speed_m_s
                result = np.fft.fft(self._window*np.exp(1j*phase), axis=1)*self.bandwidth_hz/self.samples
        except FloatingPointError as error:
            raise ValueError('range_response_overflow') from error
        if not np.isfinite(result).all():
            raise ValueError('range_response_overflow')
        return result

    def _inputs(self, distances_m, variances, mixing, range_max_m):
        depths = np.asarray(distances_m, dtype=float)
        variance = np.asarray(variances, dtype=float)
        matrix = np.asarray(mixing, dtype=complex)
        if (depths.ndim != 2 or variance.shape != depths.shape or not depths.size
                or depths.size > 2_000_000 or matrix.ndim != 2
                or matrix.shape[1] != len(depths) or not 1 <= matrix.shape[0] <= 512
                or not 1 <= len(depths) <= 512 or matrix.shape[0]*self.samples > 2_000_000
                or not all(np.isfinite(v).all() for v in (depths, variance, matrix))
                or np.any(depths < 0) or np.any(variance < 0)
                or np.any((depths == 0) & (variance > 0))
                or not np.isfinite(range_max_m) or range_max_m <= 0):
            raise ValueError('invalid_diffuse_return_model')
        return depths, variance, matrix, (variance > 0) & (depths <= range_max_m)

    def mean_power(self, distances_m, variances, mixing, *, range_max_m):
        """E[|y|²] in [output beam, range], conditional on supplied geometry.

        Scatterer variance includes source level, propagation, incidence,
        reflectivity and area. Beams/ranges in the output are not independent.
        At most 32 scatterer kernels are formed in each calculation block.
        """
        depths, variance, matrix, valid = self._inputs(distances_m, variances, mixing, range_max_m)
        source = np.zeros((len(depths), self.samples))
        for beam in range(len(depths)):
            indices = np.flatnonzero(valid[beam])
            for start in range(0, len(indices), 32):
                selected = indices[start:start+32]
                response = self.complex_response(depths[beam, selected])
                source[beam] += variance[beam, selected] @ (np.abs(response)**2)
        with np.errstate(over='ignore', invalid='ignore'):
            result = np.abs(matrix)**2 @ source
        if not np.isfinite(result).all():
            raise ValueError('diffuse_return_moment_overflow')
        return result

    def power_moments(self, distances_m, variances, mixing, queries, *, range_max_m):
        """Mean/covariance of <=64 selected [beam, range] powers.

        For independent circular complex Gaussian sources, C=H diag(v) H*,
        E[P_i]=C_ii and Cov(P_i,P_j)=|C_ij|². This conditional covariance is
        not a pose/landmark covariance or a field-calibrated noise certificate.
        """
        depths, variance, matrix, valid = self._inputs(distances_m, variances, mixing, range_max_m)
        selected = np.asarray(queries)
        if (selected.ndim != 2 or selected.shape[1] != 2 or not 1 <= len(selected) <= 64
                or selected.dtype.kind not in 'iu' or np.any(selected < 0)
                or np.any(selected[:, 0] >= len(matrix)) or np.any(selected[:, 1] >= self.samples)):
            raise ValueError('invalid_power_moment_queries')
        covariance = np.zeros((len(selected), len(selected)), dtype=complex)
        for beam in range(len(depths)):
            indices = np.flatnonzero(valid[beam])
            for start in range(0, len(indices), 32):
                rows = indices[start:start+32]
                response = self.complex_response(depths[beam, rows])[:, selected[:, 1]].T
                transfer = matrix[selected[:, 0], beam, None]*response*np.sqrt(variance[beam, rows])
                covariance += transfer @ transfer.conj().T
        with np.errstate(over='ignore', invalid='ignore'):
            powers = np.abs(covariance)**2
        if not np.isfinite(powers).all():
            raise ValueError('diffuse_return_moment_overflow')
        return dict(mean=np.real(np.diag(covariance)), covariance=powers)

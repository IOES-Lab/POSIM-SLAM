import unittest
import numpy as np
from posim_slam.sonar_range_response import PosimRangeResponse


def direct_kernel(distance, n, bandwidth, speed):
    # Independent closed-form Dirichlet sum, not an FFT implementation.
    def dirichlet(t):
        t = (t+.5) % 1-.5
        return n*np.sinc(n*t)/np.sinc(t)*np.exp(1j*np.pi*(n-1)*t)
    t = 2*distance*bandwidth/(n*speed)-np.arange(n)/n
    phase = np.exp(4j*np.pi*distance*bandwidth/n*(-n/2+1 if n%2 == 0 else -(n-1)/2+1)/speed)
    w = (.54*dirichlet(t)-.23*np.exp(2j*np.pi/n)*dirichlet(t+1/n)
         -.23*np.exp(-2j*np.pi/n)*dirichlet(t-1/n))
    return phase*w/np.sqrt(n*(.54**2+.46**2/2))*bandwidth/n


class RangeResponseTests(unittest.TestCase):
    def test_matches_closed_form_for_odd_even_fractional_and_endpoint_ranges(self):
        for n in (16, 17, 400, 599):
            model = PosimRangeResponse(n, 29900., 1500.)
            for distance in (0., .2, 4.013, 14.999, 15.):
                np.testing.assert_allclose(model.complex_response([distance])[0],
                    direct_kernel(distance, n, 29900., 1500.), atol=2e-9, rtol=2e-10)

    def test_transfer_has_source_period_without_relabelling_distance(self):
        model = PosimRangeResponse(599, 29900., 1500.)
        period = 599*1500/(2*29900)
        np.testing.assert_allclose(model.complex_response([2.123]),
            model.complex_response([2.123+period]), atol=2e-9)

    def test_mixed_independent_sources_add_power_not_amplitude(self):
        model = PosimRangeResponse(17, 29900., 1500.)
        depths = np.array([[1., 1.4], [2., 0.]])
        variance = np.array([[2., 3.], [4., 0.]])
        mixing = np.array([[1., .3j], [.2, 1.]])
        before = [v.copy() for v in (depths, variance, mixing)]
        expected = np.zeros((2, 17))
        for b in range(2):
            for r in range(2):
                expected += np.abs(mixing[:, b, None]*direct_kernel(depths[b, r], 17, 29900., 1500.))**2*variance[b, r]
        np.testing.assert_allclose(model.mean_power(depths, variance, mixing, range_max_m=10), expected, atol=1e-5, rtol=1e-12)
        for actual, saved in zip((depths, variance, mixing), before):
            np.testing.assert_array_equal(actual, saved)

    def test_clip_and_zero_variance_produce_no_echo(self):
        model = PosimRangeResponse(16, 29900., 1500.)
        result = model.mean_power([[0., 10.01]], [[0., 1.]], [[1.]], range_max_m=10.)
        self.assertFalse(result.any())

    def test_joint_power_moments_match_gaussian_control_and_keep_correlation(self):
        model = PosimRangeResponse(17, 29900., 1500.)
        depths = np.array([[.21, .24]])
        variance = np.array([[2., 3.]])
        mixing = np.array([[1.], [.5j]])
        queries = np.array([[0, 8], [0, 9], [1, 8]])
        result = model.power_moments(depths, variance, mixing, queries, range_max_m=10.)
        transfer = np.array([[mixing[b, 0]*direct_kernel(d, 17, 29900., 1500.)[r]*np.sqrt(v)
            for d, v in zip(depths[0], variance[0])] for b, r in queries])
        rng = np.random.default_rng(421)
        sources = (rng.normal(size=(100000, 2))+1j*rng.normal(size=(100000, 2)))/np.sqrt(2)
        power = np.abs(sources @ transfer.T)**2
        np.testing.assert_allclose(power.mean(axis=0), result['mean'], rtol=.02)
        np.testing.assert_allclose(np.cov(power, rowvar=False), result['covariance'], rtol=.04)
        self.assertGreater(result['covariance'][0, 1], 0)
        np.testing.assert_allclose(result['covariance'][0, 2], result['mean'][0]*result['mean'][2], rtol=1e-12)

    def test_invalid_shapes_values_and_query_limits_remain_errors(self):
        for args in ((15, 29900, 1500), (4097, 29900, 1500), (16, 0, 1500), (16, 29900, np.inf)):
            with self.assertRaises(ValueError):
                PosimRangeResponse(*args)
        model = PosimRangeResponse(16, 29900, 1500)
        for distances in ([], [-1.], [np.nan], [np.inf], [1e308]):
            with self.assertRaises(ValueError):
                model.complex_response(distances)
        for depths, variance in (([[0.]], [[1.]]), ([[1.]], [[-1.]]), ([[np.nan]], [[1.]])):
            with self.assertRaises(ValueError):
                model.mean_power(depths, variance, [[1.]], range_max_m=10)
        for queries in ([], [[0, 16]], [[1, 0]], [[0., 1.]], [[0, 1]]*65):
            with self.assertRaises(ValueError):
                model.power_moments([[1.]], [[1.]], [[1.]], queries, range_max_m=10)


if __name__ == '__main__':
    unittest.main()

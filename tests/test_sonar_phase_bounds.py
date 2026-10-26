import unittest
import numpy as np
from posim_slam.sonar_phase_bounds import BeamPhaseBounds


class BeamPhaseBoundsTests(unittest.TestCase):
    def test_identity_keeps_measurement_intervals(self):
        model = BeamPhaseBounds(np.eye(3))
        result = model.query([[1, 2, 3], [4, 5, 6]], [1, 2], relative_error=.1, absolute_error=.2)
        np.testing.assert_allclose(result['lower'], [1.6, 5.2])
        np.testing.assert_allclose(result['upper'], [2.4, 6.8])

    def test_two_phasor_extremes_are_attainable(self):
        mixing = np.array([[1, -1], [0, 1]])
        result = BeamPhaseBounds(mixing).query([[3, 1]], [0])
        np.testing.assert_allclose(result['lower'], [2])
        np.testing.assert_allclose(result['upper'], [4])
        for sign, bound in [(1, 'upper'), (-1, 'lower')]:
            source = np.linalg.solve(mixing, [3, sign])
            np.testing.assert_allclose(abs(source[0]), result[bound][0])

    def test_zero_lower_bound_does_not_imply_zero_source(self):
        inverse = np.array([[1, 1, 1], [0, 1, 0], [0, 0, 1]])
        mixing = np.linalg.inv(inverse)
        result = BeamPhaseBounds(mixing).query([[3, 2, 2]], [0])
        np.testing.assert_allclose(result['lower'], [0])
        np.testing.assert_allclose(result['upper'], [7])
        angle = np.arccos(3/4)
        cancelled = inverse@np.array([3, 2*np.exp(1j*(np.pi+angle)), 2*np.exp(1j*(np.pi-angle))])
        self.assertLess(abs(cancelled[0]), 1e-14)
        self.assertEqual((inverse@np.array([3, 2, 2]))[0], 7)

    def test_arbitrary_complex_phases_and_calibrated_noise_stay_inside_bounds(self):
        rng = np.random.default_rng(439)
        n = 13
        mixing = np.eye(n)+.04*(rng.normal(size=(n,n))+1j*rng.normal(size=(n,n)))
        model = BeamPhaseBounds(mixing)
        measured = rng.uniform(0, 4, size=(200, n))
        actual = np.maximum(0, measured*rng.uniform(.9, 1.1, size=measured.shape)
                            +rng.uniform(-.02, .02, size=measured.shape))
        output = actual*np.exp(1j*rng.uniform(-np.pi, np.pi, size=actual.shape))
        source = np.linalg.solve(mixing, output.T).T
        indices = np.arange(len(measured)) % n
        bounds = model.query(measured, indices, relative_error=.1, absolute_error=.02)
        truth = abs(source[np.arange(len(source)), indices])
        self.assertTrue(np.all(truth >= bounds['lower']-1e-12))
        self.assertTrue(np.all(truth <= bounds['upper']+1e-12))

    def test_larger_uncertainty_only_widens_interval(self):
        model = BeamPhaseBounds([[1, .2], [.3, 1]])
        a = model.query([[1, .1]], [0])
        b = model.query([[1, .1]], [0], relative_error=.2, absolute_error=.5)
        self.assertLessEqual(b['lower'][0], a['lower'][0])
        self.assertGreaterEqual(b['upper'][0], a['upper'][0])

    def test_singular_unbounded_and_malformed_inputs_are_rejected(self):
        for matrix in ([[1, 1], [1, 1]], np.eye(513), [[float('nan')]], [[1, 2]], np.diag([1, 1e-12])):
            with self.subTest(shape=np.shape(matrix)), self.assertRaises(ValueError):
                BeamPhaseBounds(matrix)
        model = BeamPhaseBounds(np.eye(2))
        for values, beams in [([[1, -1]], [0]), ([[1, 2]], [2]), ([[1, 2]], [True]),
                               ([[1, 2]], [0.5]), (np.ones((4097, 2)), np.zeros(4097, dtype=int))]:
            with self.assertRaises(ValueError): model.query(values, beams)
        for interval in [dict(relative_error=1), dict(absolute_error=-1), dict(relative_error=float('nan'))]:
            with self.assertRaises(ValueError): model.query([[1, 2]], [0], **interval)


if __name__ == '__main__': unittest.main()

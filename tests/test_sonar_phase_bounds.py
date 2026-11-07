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

    def test_dense_image_matches_every_selected_source_across_blocks(self):
        rng = np.random.default_rng(541)
        mixing = np.eye(64)+.015*(rng.normal(size=(64, 64))+1j*rng.normal(size=(64, 64)))
        model = BeamPhaseBounds(mixing)
        measured = rng.uniform(0, 3, (300, 64))
        dense = model.query_image(measured, relative_error=.1, absolute_error=.02)
        for source in range(64):
            selected = model.query(measured, np.full(len(measured), source), relative_error=.1, absolute_error=.02)
            for key in ('lower', 'upper'):
                np.testing.assert_allclose(dense[key][:, source], selected[key], atol=2e-12, rtol=2e-13)
        self.assertLessEqual(dense['maximum_product_temporary_bytes'], 8*1024*1024)
        self.assertFalse(dense['phase_estimated'] or dense['range_response_included'])

    def test_dense_bounds_cover_actual_complex_sources_and_unknown_phases(self):
        rng = np.random.default_rng(542)
        n = 16
        mixing = np.eye(n)+.04*(rng.normal(size=(n, n))+1j*rng.normal(size=(n, n)))
        model = BeamPhaseBounds(mixing)
        measured = rng.uniform(0, 4, (257, n))
        actual = np.maximum(0, measured*rng.uniform(.9, 1.1, measured.shape)+rng.uniform(-.02, .02, measured.shape))
        output = actual*np.exp(1j*rng.uniform(-np.pi, np.pi, actual.shape))
        source = np.linalg.solve(mixing, output.T).T
        bounds = model.query_image(measured, relative_error=.1, absolute_error=.02)
        self.assertTrue(np.all(abs(source) >= bounds['lower']-1e-12))
        self.assertTrue(np.all(abs(source) <= bounds['upper']+1e-12))

    def test_full_beam_image_keeps_blank_returns_and_bounds_its_temporary(self):
        model = BeamPhaseBounds(np.eye(256))
        blank = model.query_image(np.zeros((70, 256)))
        np.testing.assert_array_equal(blank['lower'], 0)
        np.testing.assert_array_equal(blank['upper'], 0)
        self.assertEqual(blank['maximum_product_temporary_bytes'], 8*1024*1024)
        source = np.arange(2*256, dtype=float).reshape(2, 256)
        bounds = model.query_image(source, relative_error=.1, absolute_error=.2)
        np.testing.assert_allclose(bounds['lower'], np.maximum(0, source*.9-.2))
        np.testing.assert_allclose(bounds['upper'], source*1.1+.2)

    def test_dense_invalid_or_overflowing_measurements_are_rejected(self):
        model = BeamPhaseBounds(np.eye(2))
        for values in (np.ones(2), np.ones((0, 2)), np.ones((1, 3)), [[1, -1]], [[1, np.inf]], np.ones((4097, 2))):
            with self.assertRaises(ValueError):
                model.query_image(values)
        for interval in (dict(relative_error=1), dict(absolute_error=-1), dict(relative_error=np.nan)):
            with self.assertRaises(ValueError):
                model.query_image([[1, 2]], **interval)
        with self.assertRaisesRegex(ValueError, 'invalid_beam_magnitude_image'):
            BeamPhaseBounds(np.eye(512)).query_image(np.ones((4096, 512)))
        with self.assertRaisesRegex(ValueError, 'beam_magnitude_interval_overflow'):
            BeamPhaseBounds([[1e-300]]).query_image([[1e10]])

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

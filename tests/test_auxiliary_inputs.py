import unittest
import numpy as np
from posim_slam.auxiliary_inputs import bracketed_gyro, hydrostatic_height


class AuxiliaryInputTests(unittest.TestCase):
    def test_interpolation_preserves_clock_and_propagates_independent_variance(self):
        samples = [(10**9, [0, 2, 4], (np.eye(3)*.04).ravel()),
                   (1_020_000_000, [4, 6, 8], (np.eye(3)*.16).ravel())]
        value = bracketed_gyro(samples, 1_005_000_000, model_sigma=.1)
        np.testing.assert_allclose(value['gyro'], [1, 3, 5])
        np.testing.assert_allclose(value['covariance'], np.eye(3)*(.75**2*.04+.25**2*.16+.01))
        self.assertTrue(value['interpolated'])
        self.assertEqual(value['unknown_covariance_inputs'], 0)

    def test_unknown_covariance_requires_explicit_model_and_never_extrapolates(self):
        samples = [(10**9, [0, 0, 0], [-1]+[0]*8),
                   (1_010_000_000, [0, 0, 0], [0]*9)]
        value = bracketed_gyro(samples, 1_005_000_000, model_sigma=.01)
        self.assertEqual(value['unknown_covariance_inputs'], 2)
        np.testing.assert_allclose(value['covariance'], np.eye(3)*.0001)
        for stamp, sigma in ((999_999_999, .01), (1_010_000_001, .01), (10**9, 0)):
            with self.subTest(stamp=stamp, sigma=sigma), self.assertRaises(ValueError):
                bracketed_gyro(samples, stamp, model_sigma=sigma)
        with self.assertRaisesRegex(ValueError, 'clock'):
            bracketed_gyro(samples[::-1], 10**9, model_sigma=.01)

    def test_pressure_does_not_invent_surface_height_or_accept_unknown_variance(self):
        calibration = dict(reference_pa=101325, density_kg_m3=1025, gravity_m_s2=9.80665)
        scale = 1025*9.80665
        value = hydrostatic_height(123, 101325+2*scale, 900, **calibration)
        self.assertEqual(value['packet']['stamp_ns'], 123)
        self.assertAlmostEqual(value['packet']['height_m'], -2)
        self.assertAlmostEqual(value['packet']['variance_m2'], 900/scale**2)
        surface = hydrostatic_height(124, 101325, 900, **calibration)
        self.assertFalse(surface['valid'])
        self.assertNotIn('packet', surface)
        with self.assertRaises(ValueError):
            hydrostatic_height(125, 101325+scale, 0, **calibration)


if __name__ == '__main__': unittest.main()

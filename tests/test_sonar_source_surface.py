import unittest
from pathlib import Path
import numpy as np
from posim_slam.sonar_source_normals import ray_surface_normals
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface
from posim_slam.sonar_range_response import PosimRangeResponse


def plane(x):
    a, b, c, d = np.array([[x, -10, -10], [x, 10, -10],
                           [x, 10, 10], [x, -10, 10]])
    return np.array([[a, b, c], [a, c, d]])


class SourceSurfaceTests(unittest.TestCase):
    def test_known_plane_uses_ray_incidence_and_ray_basis(self):
        az = np.linspace(-.8, .8, 31).astype(np.float32)
        el = np.linspace(-.2, .2, 25).astype(np.float32)
        # Construct the plane on the producer's float32 trigonometric grid.
        ca, sa, ce, se = (v.astype(np.float32).astype(float) for v in
                          (np.cos(az.astype(float)), np.sin(az.astype(float)),
                           np.cos(el.astype(float)), np.sin(el.astype(float))))
        rr = (3/(ce[:, None]*ca)).astype(np.float32)
        normal = ray_surface_normals(rr, az, el, 15.)
        expected = np.stack(np.broadcast_arrays(-sa, -se[:, None]*ca,
                                                ce[:, None]*ca), axis=-1)
        np.testing.assert_allclose(normal, expected, atol=5e-6, rtol=0)
        self.assertGreater(np.ptp(normal[..., 2]), .2)

    def test_missing_returns_do_not_create_an_isolated_normal(self):
        rr = np.zeros((5, 7), dtype=np.float32)
        rr[2, 3] = 3.
        rr[0, :4] = [np.nan, np.inf, -1, 16]
        normal = ray_surface_normals(rr, np.linspace(-.3, .3, 7),
                                     np.linspace(-.2, .2, 5), 15.)
        np.testing.assert_array_equal(normal, 0.)

    def test_complete_native_reference_cases_include_discontinuities_and_dense_fan(self):
        root = Path(__file__).resolve().parents[1]
        fixture = root/'docs/diagnostics/sonar-source-local-power-20261031/native-normals.npz'
        with np.load(fixture, allow_pickle=False) as z:
            names = sorted(k[:-7] for k in z.files if k.endswith('_ranges'))
            self.assertEqual(len(names), 10)
            for name in names:
                actual = ray_surface_normals(z[name+'_ranges'], z[name+'_azimuth'],
                                             z[name+'_elevation'], 15.)
                expected = z[name+'_normals']
                np.testing.assert_array_equal(np.linalg.norm(actual, axis=-1) > 0,
                                               np.linalg.norm(expected, axis=-1) > 0)
                np.testing.assert_allclose(actual, expected, atol=2e-5, rtol=0)

    def test_invalid_grid_and_float32_angle_collapse_are_rejected(self):
        for rr, az, el, maximum in (([[True]], [0.], [0.], 15.),
                                    ([[1., 1.]], [0., 0.], [0.], 15.),
                                    ([[1., 1.]], [1., 1.+1e-10], [0.], 15.),
                                    ([[1.]], [0.], [0.], np.nan),
                                    ([[1.]], [0.], [0.], 201.),
                                    ([[1.]], [0.], [0.], True),
                                    ([[1.]], [0.], [0.], [15.])):
            with self.assertRaises(ValueError):
                ray_surface_normals(rr, az, el, maximum)

    def test_exact_power_moments_match_separate_full_fft_operator(self):
        az = np.linspace(-.25, .25, 7)
        el = np.linspace(-.1, .1, 5)
        rr = np.arange(80)*.1
        model = SurfacePowerModel(az, el, rr, maximum_range_m=7.9)
        observation = SourceSurfaceObservation(model, TriangleSurface(plane(3.15)))
        pixels = np.array([[30, 2], [31, 3], [32, 4]])
        result = observation.moments([.001, .001], pixels)
        depths = np.zeros((len(az), len(el)))
        variance = np.zeros_like(depths)
        # Recover each source's producer amplitude variance from the mean-model
        # convention, without using the selected analytic Fourier response.
        factor = len(rr)*(7500/len(rr))**2
        for b in range(len(az)):
            selected = observation.beam_ids == b
            depths[b] = observation.selected_ranges[selected]
            variance[b] = observation.unit_power[selected]*.001/factor
        mixing = abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:, None]-az)))
        mixing /= np.linalg.norm(mixing)
        expected = PosimRangeResponse(80, 7500., 1500.).power_moments(
            depths, variance, mixing, pixels[:, ::-1], range_max_m=7.9)
        np.testing.assert_allclose(result['mean'], expected['mean'], rtol=1e-11)
        np.testing.assert_allclose(result['covariance'], expected['covariance'], rtol=1e-11)
        self.assertGreater(result['covariance'][0, 1], 0.)

    def test_opaque_zero_reflectivity_still_occludes_far_plane(self):
        model = SurfacePowerModel(np.linspace(-.2, .2, 7), np.linspace(-.1, .1, 5),
                                   np.arange(80)*.1, maximum_range_m=7.9)
        observation = SourceSurfaceObservation(model, TriangleSurface(
            np.concatenate((plane(2.), plane(4.)))))
        result = observation.moments([0., 0., 1., 1.], np.array([[40, 3]]))
        np.testing.assert_array_equal(result['mean'], 0.)
        np.testing.assert_array_equal(result['covariance'], 0.)
        self.assertTrue(np.all(observation.triangle_ids < 2))

    def test_near_clip_geometry_is_explicitly_unsupported(self):
        model = SurfacePowerModel(np.linspace(-.2, .2, 7), np.linspace(-.1, .1, 5),
                                   np.arange(80)*.1, maximum_range_m=7.9)
        with self.assertRaisesRegex(ValueError, 'near_clip'):
            SourceSurfaceObservation(model, TriangleSurface(plane(.1)))


if __name__ == '__main__':
    unittest.main()

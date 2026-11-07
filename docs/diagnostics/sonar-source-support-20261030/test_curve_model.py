import copy
import math
import unittest

import numpy as np

from posim_slam.range_surface_joint import vertical_fan_plane_factor
from curve_model import EchoCurveConfig, echo_curves, fan_plane_curve, fit_fan_plane_curve


def image():
    ranges = np.linspace(.1, 8., 256)
    angles = np.linspace(-.6, .6, 64)
    power = np.full((len(ranges), len(angles)), -80.)
    for beam in [*range(6, 24), *range(34, 54)]:
        first = 2.+.01*beam
        power[(ranges >= first) & (ranges <= first+.3), beam] = 20.
        if beam < 24:
            power[(ranges >= 5.) & (ranges <= 5.3), beam] = 15.
    return dict(ranges=ranges, intensity_db=power,
        beam_directions=np.column_stack([np.cos(angles), np.sin(angles), np.zeros(len(angles))]),
        stamp=np.array([42, 17_000_000], dtype=np.int64))


class EchoCurveTests(unittest.TestCase):
    def test_source_pixels_later_echoes_and_missing_beams_are_preserved(self):
        frame = image(); before = copy.deepcopy(frame)
        result = echo_curves(frame, sensor_id=3, vertical_fov_rad=.35)
        self.assertTrue(result['curves'])
        self.assertEqual(result['stamp_ns'], 42_017_000_000)
        self.assertGreater(len(result['edge_bin_ids']), np.sum(result['first_range_bin_ids'] >= 0))
        edges = set(map(tuple, result['edge_bin_ids']))
        used = []
        for curve in result['curves']:
            ids = curve['beam_ids']; bins = curve['range_bin_ids']
            np.testing.assert_array_equal(np.diff(ids), 1)
            np.testing.assert_array_equal(curve['ranges_m'], frame['ranges'][bins])
            self.assertTrue(all((int(k), int(b)) in edges for b, k in zip(ids, bins)))
            self.assertEqual(curve['source_keys'], tuple((42_017_000_000, 3, int(b), int(k)) for b, k in zip(ids, bins)))
            self.assertFalse('points' in curve or 'elevation_rad' in curve)
            used.extend(ids)
        self.assertEqual(len(used), len(set(used)))
        self.assertFalse(any(24 <= b < 34 for b in used))
        for key in frame:
            np.testing.assert_array_equal(frame[key], before[key])
        self.assertFalse(result['map_qualified'] or result['physical_identity_verified'] or result['observed_elevation'])

    def test_uniform_gain_keeps_observations_and_blank_frame_has_no_invented_return(self):
        frame = image(); shifted = copy.deepcopy(frame); shifted['intensity_db'] += 37.
        a = echo_curves(frame, sensor_id=0, vertical_fov_rad=.35)
        b = echo_curves(shifted, sensor_id=0, vertical_fov_rad=.35)
        np.testing.assert_array_equal(a['edge_bin_ids'], b['edge_bin_ids'])
        np.testing.assert_array_equal(a['first_range_bin_ids'], b['first_range_bin_ids'])
        frame['intensity_db'].fill(-80.)
        blank = echo_curves(frame, sensor_id=0, vertical_fov_rad=.35)
        self.assertEqual(blank['curves'], [])
        self.assertTrue(np.all(blank['first_range_bin_ids'] == -1))
        self.assertEqual(len(blank['edge_bin_ids']), 0)

    def test_capacity_rejects_whole_frame_and_invalid_calibration_does_not_pass(self):
        with self.assertRaisesRegex(ValueError, 'echo_curve_edge_capacity'):
            echo_curves(image(), sensor_id=0, vertical_fov_rad=.35, config=EchoCurveConfig(maximum_edges=1))
        for sensor, aperture in ((-1, .35), (True, .35), (0, 0.), (0, math.pi), (0, math.nan)):
            with self.assertRaises(ValueError):
                echo_curves(image(), sensor_id=sensor, vertical_fov_rad=aperture)
        with self.assertRaises(ValueError):
            echo_curves(image(), sensor_id=0, vertical_fov_rad=.35, config=EchoCurveConfig(edge=None))

    def test_curve_predictions_match_independent_dense_rays_and_existing_primitive(self):
        angles = np.linspace(-.6, .6, 17)
        for q in (np.array([.3, .08, 0.]), np.array([.3, -.06, .23]), np.array([.3, .04, -.03])):
            result = fan_plane_curve(q, angles, vertical_fov_rad=.35)
            elevation = np.linspace(-.175, .175, 65537)
            for i, angle in enumerate(angles):
                rays = np.column_stack([np.cos(elevation)*np.cos(angle), np.cos(elevation)*np.sin(angle), np.sin(elevation)])
                expected = 1/np.max(rays@q)
                self.assertAlmostEqual(result['ranges_m'][i], expected, places=9)
                primitive = vertical_fan_plane_factor(position=[0, 0, 0], attitude=np.eye(3),
                    imu_to_sensor=[0, 0, 0], sensor_to_imu=np.eye(3), azimuth_rad=angle,
                    vertical_fov_rad=.35, plane_normal=q/np.linalg.norm(q), plane_distance=1/np.linalg.norm(q),
                    measured_range=3., range_min=.001, range_max=100.)
                self.assertAlmostEqual(result['ranges_m'][i], primitive['predicted_range'], places=12)

    def test_prediction_jacobian_and_elevation_reflection_are_consistent(self):
        angles = np.linspace(-.5, .5, 23); q = np.array([.3, -.04, .05]); epsilon = 1e-7
        result = fan_plane_curve(q, angles, vertical_fov_rad=.4)
        numeric = np.column_stack([(fan_plane_curve(q+epsilon*np.eye(3)[i], angles, vertical_fov_rad=.4)['ranges_m']-
            fan_plane_curve(q-epsilon*np.eye(3)[i], angles, vertical_fov_rad=.4)['ranges_m'])/(2*epsilon) for i in range(3)])
        np.testing.assert_allclose(result['jacobian'], numeric, atol=3e-8, rtol=1e-7)
        mirror = fan_plane_curve(q*[1, 1, -1], angles, vertical_fov_rad=.4)
        np.testing.assert_allclose(result['ranges_m'], mirror['ranges_m'], atol=1e-12)
        np.testing.assert_allclose(result['predicted_elevation_rad'], -mirror['predicted_elevation_rad'], atol=1e-12)

    def test_other_view_retains_nonintersecting_beams_without_finite_information(self):
        result = fan_plane_curve([0., 1., 0.], [-.4, 0., .4], vertical_fov_rad=.35, allow_missing=True)
        np.testing.assert_array_equal(result['visible'], [False, False, True])
        self.assertTrue(np.isposinf(result['ranges_m'][:2]).all())
        self.assertTrue(np.isnan(result['jacobian'][:2]).all())
        self.assertTrue(np.isnan(result['predicted_elevation_rad'][:2]).all())
        self.assertAlmostEqual(result['ranges_m'][2], 1/math.sin(.4))

    def test_tilted_curve_retains_both_modes_without_a_prior_or_confidence_claim(self):
        angles = np.linspace(-.6, .6, 65); q = np.array([.3, .04, .2])
        ranges = fan_plane_curve(q, angles, vertical_fov_rad=.35)['ranges_m']
        result = fit_fan_plane_curve(ranges, angles, vertical_fov_rad=.35)
        self.assertEqual(result['plane_modes_inverse_m'].shape, (2, 3))
        for candidate in result['plane_modes_inverse_m']:
            np.testing.assert_allclose(fan_plane_curve(candidate, angles, vertical_fov_rad=.35)['ranges_m'], ranges, atol=2e-7)
        self.assertLess(result['rms_m'], 2e-7)
        self.assertEqual(result['source_rows'], len(ranges))
        self.assertFalse(result['added_prior'] or result['covariance_available'] or result['physical_identity_verified'] or result['map_qualified'])

    def test_vertical_plane_keeps_its_unobservable_local_direction(self):
        angles = np.linspace(-.6, .6, 65); q = np.array([.3, .04, 0.])
        ranges = fan_plane_curve(q, angles, vertical_fov_rad=.35)['ranges_m']
        result = fit_fan_plane_curve(ranges, angles, vertical_fov_rad=.35)
        self.assertEqual(result['rank'], 2)
        self.assertEqual(result['unbounded_basis'].shape, (3, 1))
        self.assertAlmostEqual(abs(result['unbounded_basis'][2, 0]), 1.)

    def test_bad_curve_keeps_every_row_and_reports_its_large_residual(self):
        angles = np.linspace(-.6, .6, 65); q = np.array([.3, .04, .2])
        ranges = fan_plane_curve(q, angles, vertical_fov_rad=.35)['ranges_m']
        ranges[20:40] += .8
        result = fit_fan_plane_curve(ranges, angles, vertical_fov_rad=.35)
        self.assertEqual(result['source_rows'], 65)
        self.assertEqual(len(result['residual_m']), 65)
        self.assertGreater(result['rms_m'], .1)

    def test_invalid_curve_inputs_and_no_forward_plane_are_rejected(self):
        for ranges, angles in (([1, 2, 3], [0, .1, .2]), ([1, 2, 0, 3], [0, .1, .2, .3]),
                                ([1]*4, [0, .1, .1, .3]), ([1]*4, [0, .1, .2, math.nan])):
            with self.assertRaises(ValueError):
                fit_fan_plane_curve(ranges, angles, vertical_fov_rad=.35)
        with self.assertRaises(ValueError):
            fan_plane_curve([-1, 0, 0], [0], vertical_fov_rad=.35)


if __name__ == '__main__':
    unittest.main()

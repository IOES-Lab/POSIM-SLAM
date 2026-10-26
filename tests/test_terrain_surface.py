import unittest
import numpy as np

from posim_slam.terrain_surface import surface_matches, register_surface, SurfaceConfig


class TerrainSurfaceTests(unittest.TestCase):
    def grid(self):
        x, y = np.meshgrid(np.linspace(-2, 2, 41), np.linspace(-2, 2, 41))
        points = np.c_[x.ravel(), y.ravel(), np.zeros(x.size)]
        frames = np.tile(np.arange(41) % 7, 41)
        return points, frames

    def test_flat_plane_corrects_height_without_inventing_horizontal_information(self):
        reference, frames = self.grid()
        query = reference.reshape(41, 41, 3)[12:29:2, 12:29:2].reshape(-1, 3)+[0, 0, .04]
        result = register_surface(query, reference, frames, query_frame=10, origin=[0, 0, 2])
        self.assertTrue(result['accepted'])
        np.testing.assert_allclose(result['translation'], [0, 0, -.04], atol=1e-10)
        np.testing.assert_allclose(result['rotation'], np.eye(3), atol=1e-10)
        self.assertEqual(result['history'][-1]['rank'], 3)

    def test_same_or_future_frame_cannot_fit_itself(self):
        reference, frames = self.grid()
        query = reference[:12]
        for invalid in (np.full(len(reference), 10), np.full(len(reference), 11)):
            with self.assertRaisesRegex(ValueError, 'strictly_prior'):
                surface_matches(query, reference, invalid, query_frame=10)
        match = surface_matches(query, reference, np.zeros(len(reference), dtype=int), query_frame=10)
        self.assertEqual(len(match['indices']), 0)

    def test_scan_boundary_outside_observed_surface_is_not_a_match(self):
        x = np.tile(np.linspace(-1, 1, 21), 4)
        y = np.repeat([-.3, -.2, -.1, 0], 21)
        reference = np.c_[x, y, np.zeros(len(x))]
        query = np.c_[np.linspace(-.6, .6, 13), np.full(13, .1), np.zeros(13)]
        result = register_surface(query, reference, np.repeat(np.arange(4), 21),
                                  query_frame=5, origin=[0, 0, 2])
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'insufficient_supported_overlap')
        self.assertGreater(result['rejections']['outside_support'], 0)

    def test_collinear_support_cannot_supply_a_surface_normal(self):
        reference = np.c_[np.linspace(-1, 1, 60), np.zeros((60, 2))]
        match = surface_matches(reference[::6], reference, np.arange(60) % 5,
                                query_frame=6)
        self.assertEqual(len(match['indices']), 0)
        self.assertGreater(match['rejections']['line_support'], 0)

    def test_convex_support_is_stricter_than_axis_aligned_bounds(self):
        reference = np.array([[-.4+.08*i, -.4+.08*j, 0]
                              for i in range(11) for j in range(11-i)])
        match = surface_matches([[.25, .25, .01]], reference,
                                np.arange(len(reference)) % 7, query_frame=8)
        self.assertEqual(len(match['indices']), 0)
        self.assertEqual(match['rejections']['outside_support'], 1)

    def test_known_relief_recovers_translation(self):
        reference, frames = self.grid()
        def height(points):
            x, y = points[:, 0], points[:, 1]
            return .2*np.sin(2*x)+.18*np.cos(2.5*y)+.13*x*y
        reference[:, 2] = height(reference)
        original = reference.reshape(41, 41, 3)[8:33:3, 8:33:3].reshape(-1, 3)
        delta = np.array([.03, -.02, .025])
        query = original+delta
        result = register_surface(query, reference, frames, query_frame=10, origin=[0, 0, 2])
        self.assertTrue(result['accepted'])
        corrected = (query-[0, 0, 2])@result['rotation'].T+[0, 0, 2]+result['translation']
        self.assertLess(np.sqrt(np.mean(np.sum((corrected-original)**2, axis=1))), .012)
        self.assertEqual(result['history'][-1]['rank'], 6)

    def test_known_relief_rotation_about_sensor_origin(self):
        reference, frames = self.grid()
        x, y = reference[:, 0], reference[:, 1]
        reference[:, 2] = .2*np.sin(2*x)+.18*np.cos(2.5*y)+.13*x*y
        original = reference.reshape(41, 41, 3)[8:33:3, 8:33:3].reshape(-1, 3)
        angle = .025
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1.]])
        origin = np.array([0, 0, 2.])
        query = (original-origin)@rotation.T+origin+[.02, -.01, .015]
        result = register_surface(query, reference, frames, query_frame=10, origin=origin)
        self.assertTrue(result['accepted'])
        corrected = (query-origin)@result['rotation'].T+origin+result['translation']
        self.assertLess(np.sqrt(np.mean(np.sum((corrected-original)**2, axis=1))), .012)

    def test_resource_and_numeric_limits(self):
        reference, frames = self.grid()
        with self.assertRaises(ValueError):
            surface_matches(np.zeros((129, 3)), reference, frames, query_frame=10)
        with self.assertRaises(ValueError):
            surface_matches([[0, 0, np.nan]], reference, frames, query_frame=10)
        with self.assertRaises(ValueError):
            surface_matches([[0, 0, 0]], reference, frames, query_frame=10,
                            config=SurfaceConfig(neighborhood_m=-1))

    def test_iteration_budget_is_not_success(self):
        reference, frames = self.grid()
        query = reference.reshape(41, 41, 3)[12:29:2, 12:29:2].reshape(-1, 3)+[0, 0, .04]
        result = register_surface(query, reference, frames, query_frame=10, origin=[0, 0, 2],
                                  config=SurfaceConfig(iterations=1))
        self.assertFalse(result['accepted'])
        self.assertEqual(result['reason'], 'surface_iteration_limit')
        np.testing.assert_array_equal(result['translation'], np.zeros(3))


if __name__ == '__main__':
    unittest.main()

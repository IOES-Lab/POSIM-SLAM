"""Geometry and observation-identity controls for multiview input selection."""
import unittest
import numpy as np
from posim_slam.image_neighbors import image_neighbors


class ImageNeighborTests(unittest.TestCase):
    def setUp(self):
        self.points = {i: [i*.001, 0., 3.] for i in range(30)}
        self.centers = {10: [0, 0, 0], 20: [.3, 0, 0],
                        30: [.6, 0, 0], 40: [0, 0, 0], 50: [10, 0, 0]}
        self.observations = {k: set(self.points) for k in self.centers}

    def test_parallax_rejects_identical_and_excessive_baselines(self):
        graph = image_neighbors(self.centers, self.observations, self.points)
        self.assertEqual([n.view_id for n in graph[10]], [20, 30])
        self.assertEqual(graph[50], ())
        self.assertEqual(graph[10][0].shared_tracks, 30)
        self.assertAlmostEqual(graph[10][0].baseline_m, .3)
        zero_limit = image_neighbors(self.centers, self.observations, self.points,
                                     minimum_parallax_deg=0)
        self.assertNotIn(40, [n.view_id for n in zero_limit[10]])

    def test_shared_ids_and_known_points_are_required_not_counts(self):
        self.observations[20] = set(range(100, 130))
        self.observations[30] = set(range(20)) | set(range(100, 200))
        graph = image_neighbors(self.centers, self.observations, self.points)
        self.assertEqual(graph[10], ())

    def test_every_input_view_survives_and_order_is_deterministic(self):
        forward = image_neighbors(self.centers, self.observations, self.points,
                                  max_neighbors=1)
        reverse = image_neighbors(dict(reversed(list(self.centers.items()))),
                                  self.observations, self.points, max_neighbors=1)
        self.assertEqual(forward, reverse)
        self.assertEqual(set(forward), set(self.centers))
        self.assertEqual(forward[10][0].view_id, 20)

    def test_missing_observation_timestamp_is_not_silently_decimated(self):
        del self.observations[20]
        with self.assertRaisesRegex(ValueError, 'exact_image_time'):
            image_neighbors(self.centers, self.observations, self.points)

    def test_invalid_geometry_and_policy_fail_before_mapping(self):
        for value in ([0, 0], [0, np.nan, 1]):
            with self.subTest(point=value), self.assertRaises(ValueError):
                image_neighbors({1: value}, {1: []}, {})
        for policy in ({'max_neighbors': 0}, {'max_neighbors': True},
                       {'minimum_shared_tracks': -1},
                       {'minimum_parallax_deg': 31},
                       {'maximum_parallax_deg': float('nan')}):
            with self.subTest(policy=policy), self.assertRaises(ValueError):
                image_neighbors(self.centers, self.observations, self.points, **policy)
        for invalid in (-1, True, '1'):
            with self.subTest(identifier=invalid), self.assertRaises(ValueError):
                image_neighbors({invalid: [0, 0, 0]}, {invalid: []}, {})


if __name__ == '__main__':
    unittest.main()

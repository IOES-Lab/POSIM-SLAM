import sys
import unittest
from pathlib import Path
import numpy as np
from posim_slam.acoustic_map import AcousticMap


def update(points, segment=0, yaw=0., position=(0., 0.)):
    return dict(state='tracking', segment=segment, yaw_fan_rad=yaw,
                position_fan_m=position,
                registration={'inlier_points_current_fan_m': points})


class MapTests(unittest.TestCase):
    def test_rotation_translation_and_export_semantics(self):
        m = AcousticMap()
        m.push(update([[1., 0.]], yaw=np.pi/2, position=(2., 3.)))
        np.testing.assert_allclose(m.snapshot()['points_fan_m'], [[2., 4.]])
        self.assertIn('elevation_resolved false', m.ply())
        self.assertIn('element vertex 1', m.ply())
        self.assertTrue(m.ply().endswith('2 4 0\n'))

    def test_segment_loss_and_waiting(self):
        m = AcousticMap()
        m.push(update([[1., 0.]]))
        m.push(dict(state='waiting', segment=0))
        self.assertEqual(len(m.cells), 1)
        m.push(dict(state='lost', segment=1))
        self.assertEqual(len(m.cells), 0)
        m.push(update([[2., 0.]], segment=1))
        m.push(update([[3., 0.]], segment=2))
        self.assertEqual(m.snapshot()['points_fan_m'], [[3., 0.]])

    def test_bounded_deduplication_and_snapshot_ownership(self):
        m = AcousticMap(max_points=2)
        m.push(update([[1., 0.], [1.01, 0.], [2., 0.], [3., 0.]]))
        self.assertEqual(m.snapshot()['points_fan_m'], [[2., 0.], [3., 0.]])
        m.snapshot()['points_fan_m'][0][0] = 999
        self.assertEqual(m.snapshot()['points_fan_m'][0][0], 2.)

    def test_invalid_update_cannot_pollute_map(self):
        m = AcousticMap()
        m.push(update([[1., 0.]]))
        for points in ([[float('nan'), 0.]], [[1., 2., 3.]]):
            with self.assertRaises(ValueError):
                m.push(update(points))
        self.assertEqual(m.snapshot()['points_fan_m'], [[1., 0.]])


if __name__ == '__main__':
    unittest.main()

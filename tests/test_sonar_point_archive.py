import unittest
from posim_slam.sonar_point_archive import SonarPointArchive
from posim_slam.sonar_snapshots import SonarSnapshots


def report(ident, point, accepted=True):
    return dict(point_estimates=[dict(id=ident, point=point)], point_update_accepted=accepted)


class SonarPointArchiveTests(unittest.TestCase):
    def test_rebinding_updates_one_source_and_rejected_step_cannot_publish(self):
        archive = SonarPointArchive()
        archive.update(1, report(10, [1., 2., 3.], False), {10: 2})
        self.assertEqual(archive.snapshot()['points'], [])
        archive.update(2, report(10, [1., 2., 3.]), {10: 2})
        archive.update(3, report(11, [2., 3., 4.]), {11: 2})
        archive.update(4, report(11, [99., 99., 99.], False), {11: 2})
        rows = archive.snapshot()['points']
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0], dict(source_id=2, native_id=11, stamp_ns=3, point=[2., 3., 4.]))
        self.assertIn('element vertex 1', archive.ply())
        self.assertTrue(archive.ply().endswith('2 3 4\n'))
        self.assertFalse(archive.snapshot()['map_qualified'])

    def test_invalid_batch_is_atomic_and_reset_is_a_new_archive(self):
        archive = SonarPointArchive(2)
        archive.update(1, report(1, [0., 0., 1.]), {1: 1})
        before = archive.snapshot()
        bad = dict(point_estimates=[dict(id=2, point=[2., 2., 2.]),
                                   dict(id=3, point=[float('nan'), 0., 0.])],
                   point_update_accepted=True)
        with self.assertRaises(ValueError):
            archive.update(2, bad, {2: 2, 3: 3})
        self.assertEqual(archive.snapshot(), before)
        with self.assertRaises(ValueError):
            archive.update(2, report(3, [0., 0., 1.]), {})
        self.assertEqual(archive.snapshot(), before)
        archive.update(2, report(2, [2., 2., 2.]), {2: 2})
        archive.update(3, report(3, [3., 3., 3.]), {3: 3})
        self.assertEqual([p['source_id'] for p in archive.snapshot()['points']], [2, 3])
        self.assertEqual(archive.evicted, 1)
        self.assertEqual(SonarPointArchive().snapshot()['points'], [])

    def test_pending_native_ids_are_mapped_before_retirement_ack(self):
        owner = SonarSnapshots()
        owner.bindings = {42: {'id': 7}}
        self.assertEqual(owner.point_sources(), {7: 42})
        owner.pending = (9, {42: {'id': 7}, 43: {'id': 8}}, 9, {8})
        self.assertEqual(owner.point_sources(), {7: 42, 8: 43})
        owner.observe(dict(stamp_ns=9, sonar=dict(retained_pose_stamps_ns=[9],
                          active_track_ids=[], owner_received=True)))
        self.assertEqual(owner.point_sources(), {})


if __name__ == '__main__':
    unittest.main()

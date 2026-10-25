import unittest
from posim_slam.sonar_candidates import SonarCandidates as RecordedSonarCandidates


def frame(stamp,segment=0,ids=(8,)):
    return dict(stamp_ns=stamp,segment=segment,observations=[dict(track_id=i,range_m=4.,azimuth_rad=.1) for i in ids])


class CandidateTests(unittest.TestCase):
    def test_only_causal_measured_history_and_explicit_variance(self):
        c=RecordedSonarCandidates(.02,.01)
        self.assertEqual(c.push(frame(1)),[]);self.assertEqual(c.push(frame(2)),[])
        track=c.push(frame(3))[0]
        self.assertEqual([o['stamp_ns'] for o in track['observations']],[1,2,3])
        self.assertEqual([track['covariance'][i][i] for i in range(6)],[.0004,.0001]*3)
        self.assertEqual(track['covariance'][0][2],0)
        self.assertNotIn('point',track)

    def test_loss_and_segment_reset_do_not_join_associations(self):
        c=RecordedSonarCandidates(.02,.01)
        for i in range(1,4):old=c.push(frame(i))
        c.push(frame(4,ids=()))
        for i in range(5,8):new=c.push(frame(i))
        self.assertNotEqual(old[0]['id'],new[0]['id'])
        self.assertEqual(c.push(frame(8,segment=1)),[])
        for i in (9,10):reset=c.push(frame(i,segment=1))
        self.assertNotEqual(new[0]['id'],reset[0]['id'])

    def test_rejection_is_atomic(self):
        c=RecordedSonarCandidates(.02,.01);c.push(frame(1))
        with self.assertRaises(ValueError):c.push(frame(2,ids=(8,8)))
        self.assertEqual(c.stamp,1);c.push(frame(2));self.assertEqual(len(c.push(frame(3))),1)
        with self.assertRaises(ValueError):c.push(frame(3))
        with self.assertRaises(ValueError):RecordedSonarCandidates(0,.01)

    def test_long_track_keeps_keyframe_and_recent_poses_with_same_identity(self):
        c=RecordedSonarCandidates(.02,.01);source=None
        for stamp in range(1,201):
            retained={1}|set(range(max(1,stamp-4),stamp)) if stamp>1 else set()
            tracks=c.push(frame(stamp),retained_stamps=retained)
            if stamp<3:continue
            self.assertEqual(len(tracks),1);track=tracks[0]
            if source is None:source=track['id']
            self.assertEqual(track['id'],source)
            self.assertEqual([o['stamp_ns'] for o in track['observations']],sorted(retained|{stamp}))
            self.assertLessEqual(len(track['observations']),6)
            self.assertEqual(track['observations'][0],dict(stamp_ns=1,range_m=4.,azimuth_rad=.1))
        self.assertEqual(c.next_id,1)

    def test_owned_history_survives_pruning_and_appends_without_reweighting(self):
        from posim_slam.sonar_snapshots import SonarSnapshots
        owner=SonarSnapshots();builder=RecordedSonarCandidates(.02,.01)
        def observe(stamp,retained,active=(),received=False):
            owner.observe(dict(stamp_ns=stamp,sonar=dict(retained_pose_stamps_ns=retained,
                active_track_ids=list(active),owner_received=received)))
        for stamp in (1,2):
            builder.push(frame(stamp),retained_stamps=owner.retained)
            observe(stamp,list(range(1,stamp+1)))
        tracks=builder.push(frame(3),retained_stamps=owner.retained)
        first=owner.prepare(3,tracks)[0];observe(3,[1,2,3],[first['id']],True)
        tracks=builder.push(frame(4),retained_stamps=owner.retained)
        second=owner.prepare(4,tracks)[0]
        self.assertEqual(second['observations'][:-1],first['observations'])
        self.assertEqual([row[:-2] for row in second['covariance'][:-2]],first['covariance'])
        # Native retirement removes the factor when pose 2 is marginalized.
        observe(4,[1,3,4],[],True)
        for stamp in (5,6):
            tracks=builder.push(frame(stamp),retained_stamps=owner.retained)
            self.assertEqual(owner.prepare(stamp,tracks),[])
            observe(stamp,[1,*range(3,stamp+1)])
        tracks=builder.push(frame(7),retained_stamps=owner.retained)
        rebound=owner.prepare(7,tracks)[0]
        self.assertNotEqual(rebound['id'],first['id'])
        self.assertEqual([o['stamp_ns'] for o in rebound['observations']],[5,6,7])
        self.assertEqual(owner.point_sources()[rebound['id']],tracks[0]['id'])

    def test_invalid_retention_does_not_change_state(self):
        c=RecordedSonarCandidates(.02,.01);c.push(frame(1))
        for bad in ({0},{2},{True},{1.5}):
            with self.assertRaisesRegex(ValueError,'invalid_retained_sonar_pose_stamps'):
                c.push(frame(2),retained_stamps=bad)
            self.assertEqual(c.stamp,1)


if __name__=='__main__':unittest.main()

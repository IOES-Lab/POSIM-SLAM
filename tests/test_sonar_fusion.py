import unittest
from copy import deepcopy
from posim_slam.sonar_fusion import SonarFusion


def frame(stamp, segment=0):
    return dict(stamp_ns=stamp, segment=segment,
                observations=[dict(track_id=8, range_m=4., azimuth_rad=.1)])


def state(stamp, *, received=False, active=(), points=(), accepted=False, retained=None):
    return dict(type='state', stamp_ns=stamp, sonar=dict(
        retained_pose_stamps_ns=list(range(1, stamp+1)) if retained is None else retained,
        active_track_ids=list(active), owner_received=received,
        point_estimates=list(points), point_update_accepted=accepted))


class FusionTests(unittest.TestCase):
    def seed(self):
        fusion=SonarFusion(.02,.01)
        for t in (1,2):
            self.assertEqual(fusion.prepare(t,frame(t)),[])
            fusion.observe(state(t))
        updates=fusion.prepare(3,frame(3))
        self.assertEqual(len(updates),1)
        return fusion,updates[0]['id']

    def test_image_and_auxiliary_states_share_one_clock_and_archive(self):
        f,ident=self.seed()
        point=dict(id=ident,point=[1.,2.,3.])
        f.observe(state(3,received=True,active=[ident],points=[point],accepted=True))
        # A camera event without sonar still updates the joint estimate.
        self.assertEqual(f.prepare(4),[])
        point['point']=[2.,2.,3.]
        f.observe(state(4,active=[ident],points=[point],accepted=True))
        self.assertEqual(len(f.snapshot()['points']),1)
        self.assertEqual(f.snapshot()['points'][0]['point'],[2.,2.,3.])
        self.assertEqual(f.states,4)
        self.assertFalse(f.snapshot()['map_qualified'])
        self.assertIn('element vertex 1',f.ply())

    def test_retirement_response_keeps_pending_identity_and_rebind_does_not_duplicate(self):
        f,ident=self.seed()
        f.observe(state(3,received=True,points=[dict(id=ident,point=[1.,0.,0.])],accepted=True))
        update=f.prepare(4,frame(4))[0]
        self.assertNotEqual(update['id'],ident)
        f.observe(state(4,received=True,active=[update['id']],
                        points=[dict(id=update['id'],point=[2.,0.,0.])],accepted=True))
        self.assertEqual(len(f.snapshot()['points']),1)
        self.assertEqual(f.snapshot()['points'][0]['source_id'],0)
        self.assertEqual(f.snapshot()['points'][0]['point'],[2.,0.,0.])

    def test_invalid_ack_cannot_publish_points_and_fails_segment(self):
        f,ident=self.seed()
        before=f.archive.snapshot()
        with self.assertRaisesRegex(ValueError,'snapshot_ack_mismatch'):
            f.observe(state(3,points=[dict(id=ident,point=[99.,0.,0.])],accepted=True))
        self.assertEqual(f.archive.snapshot(),before)
        self.assertEqual(f.owner.stamp,2)
        for call in (lambda:f.prepare(4),lambda:f.observe(state(3)),f.snapshot,f.ply):
            with self.assertRaisesRegex(RuntimeError,'segment_failed'):call()

    def test_bad_point_does_not_commit_owner_or_map(self):
        f,ident=self.seed();before=f.archive.snapshot();owner=deepcopy(f.owner.__dict__)
        with self.assertRaisesRegex(ValueError,'invalid_acoustic_point'):
            f.observe(state(3,received=True,active=[ident],
                            points=[dict(id=ident,point=[float('nan'),0.,0.])],accepted=True))
        self.assertEqual(f.archive.snapshot(),before)
        self.assertEqual(f.owner.__dict__,owner)

    def test_every_event_requires_exact_ack_even_without_factors(self):
        f=SonarFusion(.02,.01);f.prepare(1)
        with self.assertRaisesRegex(ValueError,'state_ack_pending'):f.prepare(2)
        f=SonarFusion(.02,.01);f.prepare(1)
        with self.assertRaisesRegex(ValueError,'unexpected_state'):f.observe(state(2))
        f=SonarFusion(.02,.01)
        with self.assertRaisesRegex(ValueError,'exact_frame'):f.prepare(2,frame(1))

    def test_native_omits_snapshot_ack_on_camera_only_state(self):
        f=SonarFusion(.02,.01);f.prepare(1)
        event=state(1);del event['sonar']['owner_received']
        f.observe(event)
        self.assertEqual(f.states,1)
        f.prepare(2)
        event=state(2);event['sonar']['owner_received']=None
        with self.assertRaisesRegex(ValueError,'snapshot_ack_mismatch'):f.observe(event)

    def test_new_segment_cannot_export_previous_points(self):
        f,ident=self.seed()
        f.observe(state(3,received=True,active=[ident],points=[dict(id=ident,point=[1.,2.,3.])],accepted=True))
        self.assertEqual(len(f.snapshot()['points']),1)
        replacement=SonarFusion(.02,.01)
        self.assertEqual(replacement.snapshot()['points'],[])
        self.assertIsNone(replacement.awaiting)

    def test_frontend_reset_does_not_reuse_native_or_source_ids(self):
        f,ident=self.seed()
        f.observe(state(3,received=True,active=[ident],
                        points=[dict(id=ident,point=[1.,2.,3.])],accepted=True))
        for stamp in (4,5):
            self.assertEqual(f.prepare(stamp,frame(stamp,segment=1)),[])
            f.observe(state(stamp,active=[ident]))
        update=f.prepare(6,frame(6,segment=1))[0]
        self.assertNotEqual(update['id'],ident)
        self.assertEqual([o['stamp_ns'] for o in update['observations']],[4,5,6])
        f.observe(state(6,received=True,active=[ident,update['id']],
                        points=[dict(id=update['id'],point=[3.,2.,1.])],accepted=True))
        self.assertEqual([p['source_id'] for p in f.snapshot()['points']],[0,1])


if __name__=='__main__':unittest.main()

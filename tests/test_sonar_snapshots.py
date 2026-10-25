import unittest
import numpy as np
from posim_slam.sonar_snapshots import SonarSnapshots


def state(stamp, retained, active=(), received=False):
    return dict(stamp_ns=stamp, sonar=dict(retained_pose_stamps_ns=retained,
                active_track_ids=list(active), owner_received=received))


def track(stamps, ident=90, correlated=True):
    n=2*len(stamps)
    covariance=np.eye(n)*.01+(np.ones((n,n))*.001 if correlated else 0)
    return dict(id=ident,observations=[dict(stamp_ns=t,range_m=4.,azimuth_rad=.1) for t in stamps],
                covariance=covariance.tolist())


class SnapshotsTest(unittest.TestCase):
    def test_expired_seed_and_correlations(self):
        manager=SonarSnapshots();manager.observe(state(3,[2,3]))
        source=track([1,2,3,4]);updates=manager.prepare(4,[source])
        self.assertEqual([o['stamp_ns'] for o in updates[0]['observations']],[2,3,4])
        self.assertTrue(np.array_equal(updates[0]['covariance'],np.asarray(source['covariance'])[2:,2:]))
        with self.assertRaisesRegex(ValueError,'ack_pending'):manager.prepare(5,[])
        manager.observe(state(4,[2,3,4],[0],True))
        updates=manager.prepare(5,[track([1,2,3,4,5])])
        self.assertEqual(updates[0]['id'],0)
        self.assertEqual(len(updates[0]['observations']),4)

    def test_noncontiguous_retained_poses_do_not_reintroduce_removed_rows(self):
        manager=SonarSnapshots();manager.observe(state(4,[1,3,4]))
        source=track([1,2,3,4,5]);updates=manager.prepare(5,[source])
        self.assertEqual([o['stamp_ns'] for o in updates[0]['observations']],[1,3,4,5])
        manager.observe(state(5,[1,3,4,5],[0],True))
        source=track([1,2,3,4,5,6]);updates=manager.prepare(6,[source])
        self.assertEqual([o['stamp_ns'] for o in updates[0]['observations']],[1,3,4,5,6])
        axes=[0,1,4,5,6,7,8,9,10,11]
        np.testing.assert_array_equal(updates[0]['covariance'],np.asarray(source['covariance'])[np.ix_(axes,axes)])

    def test_owned_history_edits_and_multiple_appends_still_rejected(self):
        manager=SonarSnapshots();manager.observe(state(4,[1,3,4]))
        manager.prepare(5,[track([1,2,3,4,5])]);manager.observe(state(5,[1,3,4,5],[0],True))
        for source in [track([1,2,4,5,6]),track([1,2,3,4,5,6,7])]:
            with self.assertRaisesRegex(ValueError,'changed_sonar_candidate_history'):
                manager.prepare(source['observations'][-1]['stamp_ns'],[source])
            self.assertIsNone(manager.pending)
        changed=track([1,2,3,4,5,6]);changed['observations'][2]['range_m']=4.1
        with self.assertRaisesRegex(ValueError,'changed_sonar_candidate_history'):
            manager.prepare(6,[changed])
        changed=track([1,2,3,4,5,6]);changed['covariance'][4][4]+=.01
        with self.assertRaisesRegex(ValueError,'changed_sonar_candidate_covariance'):
            manager.prepare(6,[changed])
        self.assertIsNone(manager.pending)

    def test_rejected_extension_freezes_only_that_association_until_retirement(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])]);manager.observe(state(3,[1,2,3],[0],True))
        manager.prepare(4,[track([1,2,3,4])])
        reply=state(4,[1,2,3,4],[0],True);reply['sonar']['geometry_rejected_track_ids']=[0]
        manager.observe(reply)
        self.assertEqual(len(manager.bindings[90]['observations']),3)
        self.assertEqual(manager.prepare(5,[track([1,2,3,4,5])]),[])
        manager.observe(state(5,[3,4,5]))
        self.assertEqual(manager.blocked,set())
        self.assertEqual(manager.prepare(6,[track([1,2,3,4,5,6],correlated=False)])[0]['id'],1)

    def test_rejected_new_candidate_and_unknown_rejection(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])])
        reply=state(3,[1,2,3],[],True);reply['sonar']['geometry_rejected_track_ids']=[99]
        with self.assertRaisesRegex(ValueError,'unknown_rejected'):manager.observe(reply)
        self.assertIsNotNone(manager.pending)
        reply['sonar']['geometry_rejected_track_ids']=[0];manager.observe(reply)
        self.assertEqual(manager.bindings,{})
        self.assertEqual(manager.consumed,{})
        self.assertEqual(manager.prepare(4,[track([1,2,3,4])])[0]['id'],1)

    def test_retirement_uses_only_fresh_observations_even_when_absent(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3],correlated=False)])
        manager.observe(state(3,[2,3],[],True))
        self.assertEqual(manager.consumed,{2:{90},3:{90}})
        self.assertEqual(manager.prepare(4,[]),[])
        manager.observe(state(4,[2,3,4]))
        self.assertEqual(manager.prepare(5,[track([2,3,4,5],correlated=False)]),[])
        manager.observe(state(5,[2,3,4,5]))
        update=manager.prepare(6,[track([2,3,4,5,6],correlated=False)])[0]
        self.assertEqual(update['id'],1)
        self.assertEqual([o['stamp_ns'] for o in update['observations']],[4,5,6])
        manager.observe(state(6,[2,4,5,6],[1],True))
        self.assertEqual(manager.consumed,{2:{90},4:{90},5:{90},6:{90}})

    def test_correlated_rebinding_fails_atomically(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])]);manager.observe(state(3,[2,3],[],True))
        manager.observe(state(4,[2,3,4]));manager.observe(state(5,[2,3,4,5]))
        with self.assertRaisesRegex(ValueError,'correlated_sonar_rebinding'):
            manager.prepare(6,[track([2,3,4,5,6])])
        self.assertIsNone(manager.pending)
        self.assertEqual(manager.next_id,1)
        self.assertEqual(manager.consumed,{2:{90},3:{90}})

    def test_ownership_expires_only_with_pose_and_cannot_be_resurrected(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])]);manager.observe(state(3,[2,3],[],True))
        manager.observe(state(4,[3,4]));self.assertEqual(manager.consumed,{3:{90}})
        with self.assertRaisesRegex(ValueError,'retained_pose_lifecycle'):
            manager.observe(state(5,[2,3,4,5]))
        self.assertEqual(manager.stamp,4)
        manager.observe(state(5,[4,5]));self.assertEqual(manager.consumed,{})

    def test_retained_pose_budget_matches_native_state_and_keyframe_capacity(self):
        manager=SonarSnapshots();manager.observe(state(73,list(range(1,74))))
        with self.assertRaisesRegex(ValueError,'retained_pose_lifecycle'):
            manager.observe(state(74,list(range(1,75))))
        self.assertEqual(manager.stamp,73)

    def test_ownership_budget_fails_without_forgetting_consumed_measurements(self):
        manager=SonarSnapshots();manager.max_sources_per_pose=1
        manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3],1),track([1,2,3],2)])
        with self.assertRaisesRegex(ValueError,'ownership_budget'):
            manager.observe(state(3,[1,2,3],[],True))
        self.assertEqual(manager.consumed,{})
        self.assertEqual(manager.stamp,2)
        self.assertIsNotNone(manager.pending)

    def test_short_horizon_waits(self):
        manager=SonarSnapshots();manager.observe(state(3,[3]))
        self.assertEqual(manager.prepare(4,[track([1,2,4])]),[])
        self.assertEqual(manager.next_id,0)

    def test_changed_covariance_and_failed_batch_are_atomic(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])]);manager.observe(state(3,[1,2,3],[0],True))
        changed=track([1,2,3,4]);changed['covariance'][0][0]*=2
        with self.assertRaisesRegex(ValueError,'changed_sonar_candidate_covariance'):
            manager.prepare(4,[track([1,2,3,4],1),changed])
        self.assertIsNone(manager.pending)
        self.assertEqual(manager.next_id,1)
        self.assertEqual(manager.prepare(4,[track([1,2,3,4])])[0]['id'],0)

    def test_ack_failure_preserves_pending(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3])])
        with self.assertRaisesRegex(ValueError,'missing_sonar_snapshot_ack'):
            manager.observe(state(3,[1,2,3],[0]))
        self.assertEqual(manager.stamp,2)
        self.assertEqual(manager.consumed,{})
        manager.observe(state(3,[1,2,3],[0],True))
        self.assertIsNone(manager.pending)

    def test_rejected_candidates_do_not_starve_other_eligible_tracks(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]));tried=[]
        for stamp in (3,4,5):
            updates=manager.prepare(stamp,[track(list(range(1,stamp+1)),i) for i in range(48)])
            self.assertEqual(len(updates),16)
            sources=manager.point_sources();tried.extend(sources[u['id']] for u in updates)
            reply=state(stamp,list(range(1,stamp+1)),[],True)
            reply['sonar']['geometry_rejected_track_ids']=[u['id'] for u in updates]
            manager.observe(reply)
        self.assertEqual(tried,list(range(48)))
        self.assertEqual(len(manager.candidate_order),48)
        manager.prepare(6,[])
        self.assertEqual(manager.candidate_order,{})

    def test_active_history_has_priority_and_waiting_candidates_rotate(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        manager.prepare(3,[track([1,2,3],199)]);manager.observe(state(3,[1,2,3],[0],True))
        seen=[]
        for stamp in (4,5):
            inputs=[track(list(range(1,stamp+1)),i) for i in [*range(40),199]]
            updates=manager.prepare(stamp,inputs)
            self.assertEqual(len(updates),16)
            owned=next(u for u in updates if u['id']==0)
            self.assertEqual(len(owned['observations']),stamp)
            sources=manager.point_sources();seen.extend(sources[u['id']] for u in updates if u['id']!=0)
            reply=state(stamp,list(range(1,stamp+1)),[0],True)
            reply['sonar']['geometry_rejected_track_ids']=[u['id'] for u in updates if u['id']!=0]
            manager.observe(reply)
        self.assertEqual(seen,list(range(30)))

    def test_failed_batch_and_ack_do_not_commit_scheduler_state(self):
        manager=SonarSnapshots();manager.observe(state(2,[1,2]))
        invalid=track([1,2,3],20);invalid['covariance'][0][0]=-1
        with self.assertRaises(ValueError):manager.prepare(3,[track([1,2,3],0),invalid])
        self.assertEqual(manager.candidate_order,{})
        manager.prepare(3,[track([1,2,3],0)])
        with self.assertRaisesRegex(ValueError,'missing_sonar_snapshot_ack'):manager.observe(state(3,[1,2,3],[0]))
        self.assertEqual(manager.candidate_order,{})
        manager.observe(state(3,[1,2,3],[0],True))
        self.assertEqual(manager.candidate_order,{0:(3,1)})


if __name__=='__main__':unittest.main()

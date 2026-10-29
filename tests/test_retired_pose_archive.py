"""Correlated delayed-state recovery; no independent pose observations."""
import copy
import unittest
import numpy as np
from posim_slam.ping_joint import split_patch_information
from posim_slam.retired_pose_archive import RetiredPoseArchive


def state(stamp,size=6):
    return dict(kind='state',ident=stamp,size=size,
                reference=[0.,0.,0.,*np.eye(3).ravel(),*([0.]*9 if size==15 else [])])


def conditional(ident,variables,j,r,kept,removed):
    split=split_patch_information(j[:,kept+removed],r,retained_columns=len(kept))
    return dict(id=ident,variables=variables,kept_indices=kept,removed_indices=removed,
                regression=split['backsub_matrix'].tolist(),offset=split['backsub_offset'].tolist(),
                noise_factor=split['observable_noise_factor'].tolist(),unbounded_basis=split['unobservable_basis'].tolist())


def packet(stamp,variables,delta,nodes=()):
    return dict(epoch='control',stamp_ns=stamp,variables=variables,delta=list(delta),conditionals=list(nodes))


class RetiredPoseArchiveTests(unittest.TestCase):
    def sequential(self):
        rng=np.random.default_rng(615);j=rng.normal(size=(42,18));r=rng.normal(size=42)
        v=[state(t) for t in (1,2,3)]
        n1=conditional(1,v,j,r,list(range(6,18)),list(range(6)))
        first=split_patch_information(j[:,list(range(6,18))+list(range(6))],r,retained_columns=12)
        n2=conditional(2,v[1:],first['jacobian'],first['residual'],list(range(6,12)),list(range(6)))
        archive=RetiredPoseArchive('control');live=rng.normal(size=6)
        archive.ingest(packet(4,[v[2]],live,[n1,n2]))
        return archive,j,r,live,n1,n2

    def test_shared_ancestor_mean_and_cross_covariance_equal_dense_conditioning(self):
        archive,j,r,live,_,_=self.sequential();got=archive.recover([1,2])
        h=j[:,:12].T@j[:,:12];cov=np.linalg.inv(h)
        mean=-np.linalg.solve(h,j[:,:12].T@(j[:,12:]@live+r))
        np.testing.assert_allclose(got['deltas'],mean,atol=3e-14)
        np.testing.assert_allclose(got['observable_conditional_covariance'],cov,atol=3e-14)
        self.assertGreater(np.linalg.norm(cov[:6,6:]),.01)
        self.assertEqual(got['unbounded_basis'].shape,(12,0))
        self.assertTrue(got['live_coordinates_treated_as_known'])
        self.assertFalse(got['independent_pose_observations']);self.assertFalse(got['map_qualified'])
        archive.ingest(packet(5,[state(3)],live+.3))
        changed=archive.recover([1,2]);self.assertGreater(np.linalg.norm(changed['deltas']-mean),.01)

    def test_state_downgrade_preserves_pose_and_free_bias_directions(self):
        variables=[state(1,15),state(2)];j=np.eye(21);r=np.arange(21)/100
        # Only the last nine coordinates of state 1 retire; its pose stays live.
        removed=list(range(6,15));kept=list(range(6))+list(range(15,21))
        node=conditional(1,variables,j,r,kept,removed)
        a=RetiredPoseArchive('control');a.ingest(packet(3,[state(1),state(2)],np.arange(12)/10,[node]))
        got=a.recover([1]);np.testing.assert_array_equal(got['deltas'],np.arange(6)/10)
        np.testing.assert_array_equal(got['observable_conditional_covariance'],np.zeros((6,6)))
        node=conditional(1,[state(1),state(2)],np.zeros((2,12)),np.zeros(2),list(range(6,12)),list(range(6)))
        a=RetiredPoseArchive('control');a.ingest(packet(3,[state(2)],np.zeros(6),[node]))
        got=a.recover([1]);self.assertEqual(np.linalg.matrix_rank(got['unbounded_basis']),6)

    def test_epoch_reference_missing_separator_and_replay_rejections_are_atomic(self):
        archive,_,_,live,n1,n2=self.sequential();before=archive.recover([1,2])['deltas'].copy()
        cases=[]
        p=packet(5,[state(3)],live);p['epoch']='other';cases.append(p)
        p=packet(5,[state(3)],live);p['variables'][0]['reference'][0]=1.;cases.append(p)
        cases.append(packet(5,[state(4)],live))
        cases.append(packet(5,[state(3)],live,[n2]))
        p=packet(5,[state(3)],live,[copy.deepcopy(n1)]);p['conditionals'][0]['id']=3;cases.append(p)
        for bad in cases:
            with self.assertRaises(ValueError):archive.ingest(bad)
            self.assertEqual(archive.stamp_ns,4)
            np.testing.assert_array_equal(archive.recover([1,2])['deltas'],before)

    def test_expiry_keeps_ancestor_closure_and_requires_explicit_old_pose_protection(self):
        a,_,_,_,_,_=self.sequential();a.expire_through(2,protect=[('state',1)])
        self.assertEqual(set(a.nodes),{1,2});a.recover([1])
        with self.assertRaises(ValueError):a.recover([2])
        a.expire_through(2);self.assertFalse(a.nodes)
        with self.assertRaises(ValueError):a.recover([1])

    def test_capacity_rejects_without_replacing_current_snapshot(self):
        _,_,_,live,n1,n2=self.sequential();a=RetiredPoseArchive('control',max_nodes=1)
        with self.assertRaisesRegex(ValueError,'capacity'):a.ingest(packet(4,[state(3)],live,[n1,n2]))
        self.assertEqual(a.stamp_ns,0);self.assertFalse(a.nodes)
        a,_,_,_,_,_=self.sequential();a.max_recovery_bytes=1024
        with self.assertRaisesRegex(ValueError,'capacity'):a.recover([1,2])
        self.assertFalse(a._memo)
        a=RetiredPoseArchive('control',max_recovery_bytes=1024)
        a.ingest(packet(4,[state(1),state(2)],np.zeros(12)))
        with self.assertRaisesRegex(ValueError,'capacity'):a.recover([1,2])


if __name__=='__main__':unittest.main()

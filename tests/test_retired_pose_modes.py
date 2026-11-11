"""Bounded conditional modes remain distinct from correlated uncertainty."""
import copy
import unittest
from unittest.mock import patch

import numpy as np

from posim_slam.inertial import exp_so3
from posim_slam.retired_pose_archive import RetiredPoseArchive
import test_retired_pose_archive as fixtures
from test_retired_pose_archive import conditional,packet,state


class RetiredPoseModeTests(unittest.TestCase):
    def test_modes_equal_dense_conditional_solution_and_left_pose_chart(self):
        archive,j,r,live,_,_=fixtures.RetiredPoseArchiveTests().sequential()
        reference=exp_so3(np.array([.3,-.2,.1]))
        variable=archive.references[('state',1)]
        archive.references[('state',1)]=type(variable)('state',1,6,tuple([.2,.4,-.3,*reference.ravel()]))
        before=archive.recover([1,2]);memo=archive._memo
        with patch.object(archive,'_resolve',side_effect=AssertionError('covariance expansion')):
            result=archive.recover_modes([1,2])
        mean=-np.linalg.solve(j[:,:12].T@j[:,:12],j[:,:12].T@(j[:,12:]@live+r))
        np.testing.assert_allclose(result['deltas'],mean,atol=3e-14)
        np.testing.assert_allclose(result['poses'][0]['attitude'],exp_so3(mean[3:6])@reference,atol=3e-14)
        np.testing.assert_allclose(result['poses'][0]['position'],[.2,.4,-.3]+mean[:3],atol=3e-14)
        self.assertTrue(result['conditional_modes_only']);self.assertFalse(result['uncertainty_computed'])
        self.assertFalse(result['independent_pose_observations']);self.assertFalse(result['map_qualified'])
        self.assertNotIn('observable_conditional_covariance',result)
        self.assertIs(archive._memo,memo)
        np.testing.assert_array_equal(archive.recover([1,2])['observable_conditional_covariance'],before['observable_conditional_covariance'])

    def test_long_shared_ancestry_fits_original_budget_without_covariance_expansion(self):
        nodes=[];count=24
        for ident in range(1,count+1):
            nodes.append(dict(id=ident,variables=[state(ident),state(ident+1)],
                kept_indices=list(range(6,12)),removed_indices=list(range(6)),
                regression=np.eye(6).tolist(),offset=(np.arange(6)*ident/1000).tolist(),
                noise_factor=np.eye(6).tolist(),unbounded_basis=np.zeros((6,0)).tolist()))
        archive=RetiredPoseArchive('control',max_recovery_bytes=4096)
        live=np.arange(6)/10
        archive.ingest(packet(count+2,[state(count+1)],live,nodes))
        with self.assertRaisesRegex(ValueError,'workspace_capacity'):archive.recover([1,4])
        result=archive.recover_modes([1,4])
        for i,stamp in enumerate([1,4]):
            expected=live+np.arange(6)*sum(range(stamp,count+1))/1000
            np.testing.assert_allclose(result['deltas'][6*i:6*i+6],expected,atol=2e-15)
        self.assertLessEqual(result['workspace_bound_bytes'],4096)
        self.assertFalse(archive._memo)

    def test_expiry_and_query_budget_reject_without_mutating_snapshot(self):
        archive,_,_,_,_,_=fixtures.RetiredPoseArchiveTests().sequential()
        archive.expire_through(2,protect=[('state',1)])
        before=copy.deepcopy(archive.root)
        archive.recover_modes([1])
        for query in ([2],[1,1],[],[True],[99]):
            with self.assertRaises(ValueError):archive.recover_modes(query)
        archive.max_recovery_bytes=1024
        with self.assertRaisesRegex(ValueError,'workspace_capacity'):archive.recover_modes([1,3])
        self.assertEqual(archive.root,before);self.assertFalse(archive._memo)

    def test_unbounded_archive_does_not_turn_modes_into_exact_observations(self):
        node=conditional(1,[state(1),state(2)],np.zeros((2,12)),np.zeros(2),list(range(6,12)),list(range(6)))
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,[state(2)],np.zeros(6),[node]))
        modes=archive.recover_modes([1])
        self.assertFalse(modes['uncertainty_computed'])
        self.assertNotIn('unbounded_basis',modes)
        self.assertEqual(np.linalg.matrix_rank(archive.recover([1])['unbounded_basis']),6)

    def test_existing_covariance_cache_counts_against_the_same_budget(self):
        archive,_,_,_,_,_=fixtures.RetiredPoseArchiveTests().sequential()
        self.assertLess(archive.recover_modes([1])['workspace_bound_bytes'],1536)
        archive.recover([1]);memo=archive._memo;archive.max_recovery_bytes=1536
        with self.assertRaisesRegex(ValueError,'workspace_capacity'):archive.recover_modes([1])
        self.assertIs(archive._memo,memo)


if __name__=='__main__':unittest.main()

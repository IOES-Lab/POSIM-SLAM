"""Full joint marginals against independent dense systems and gauge controls."""
import copy
import unittest
import numpy as np

from posim_slam.delayed_surface_update import prepare_delayed_ranges
from posim_slam.native_surface import NativeSurfaceSnapshot
from posim_slam.native_surface_information import NativeSurfaceInformation
from posim_slam.ping_joint import split_patch_information
from posim_slam.retired_pose_archive import RetiredPoseArchive,coordinates
from posim_slam.surface_prior import Variable
from test_delayed_surface_update import physical_case,descriptor
from test_native_surface import snapshot
from test_retired_pose_archive import conditional,packet,state


def information(j,r):
    if len(j)>j.shape[1]+1:
        _,a=np.linalg.qr(np.column_stack((j,r)));j=a[:,:-1];r=a[:,-1]
    return dict(schema='posim-slam-native-surface-information-1',scope='local_frozen_weight_gauss_newton',
        jacobian=j.tolist(),residual=r.tolist(),imu_intervals=0,dvl_observations=0,pressure_observations=0)


class JointNativeInformationTests(unittest.TestCase):
    def case(self):
        prior,archive,kwargs,order,j,b=physical_case()
        updated=prepare_delayed_ranges(prior,archive,**kwargs)
        delta=np.array([updated.archive.root[k] for k in coordinates(updated.prior.order)])
        value=snapshot(updated.archive,ledger=updated.prior.ledger.recent)
        value['information']=information(updated.prior.jacobian,updated.prior.jacobian@delta+updated.prior.offset)
        return NativeSurfaceSnapshot.read(value),order,j,b,value

    def test_retired_live_plane_and_pressure_cross_covariance_equal_full_dense_batch(self):
        native,order,j,b,_=self.case();before=native.recover([1,2])['deltas'].copy()
        got=native.joint_marginal(order)
        np.testing.assert_allclose(got['observable_covariance'],np.linalg.inv(j.T@j),atol=3e-10,rtol=3e-9)
        np.testing.assert_allclose(got['linearized_mode'],-np.linalg.solve(j.T@j,j.T@b),atol=2e-10)
        self.assertEqual(got['unbounded_basis'].shape[1],0)
        self.assertGreater(np.linalg.norm(got['observable_covariance'][:6,-6:]),.01)
        self.assertFalse(got['live_coordinates_treated_as_known']);self.assertFalse(got['map_qualified'])
        np.testing.assert_array_equal(native.recover([1,2])['deltas'],before)
        self.assertFalse(native._information.jacobian.flags.writeable)

    def test_joint_uncertainty_exceeds_known_root_conditioning_and_reorders_exactly(self):
        native,order,j,_,_=self.case();got=native.joint_marginal(order[:2])
        conditioned=native.recover([1,2])['observable_conditional_covariance']
        added=got['observable_covariance']-conditioned
        self.assertGreater(np.linalg.norm(added),.01)
        self.assertGreaterEqual(np.linalg.eigvalsh(added).min(),-1e-10)
        permuted=(order[-1],order[0],order[-3]);keys=coordinates(order)
        indices=[keys.index(k) for k in coordinates(permuted)]
        np.testing.assert_allclose(native.joint_marginal(permuted)['observable_covariance'],
            np.linalg.inv(j.T@j)[np.ix_(indices,indices)],atol=2e-10)

    def test_shared_free_root_and_retired_null_directions_remain_explicit(self):
        variables=[state(1),state(2),dict(kind='pressure',ident=0,size=1,reference=[0.])]
        # Relative position constrains x1-x2; absolute x and all rotations stay free.
        j=np.zeros((4,13));j[:3,:3]=np.eye(3);j[:3,6:9]=-np.eye(3);j[3,8]=1;j[3,12]=1
        node=conditional(1,variables,j,np.zeros(4),list(range(6,13)),list(range(6)))
        split=split_patch_information(j[:,list(range(6,13))+list(range(6))],np.zeros(4),retained_columns=7)
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,variables[1:],np.zeros(7),[node]))
        value=snapshot(archive);value['information']=information(split['jacobian'],split['residual'])
        native=NativeSurfaceSnapshot.read(value);order=tuple(Variable(**v) for v in variables)
        got=native.joint_marginal(order);self.assertLess(got['root_rank'],got['root_columns'])
        free=got['unbounded_basis'];self.assertEqual(np.linalg.matrix_rank(free),9)
        np.testing.assert_allclose(j@free,0,atol=2e-12)
        np.testing.assert_allclose(j@got['observable_covariance']@j.T,np.eye(4),atol=2e-12)
        self.assertGreater(np.linalg.norm(free[:3]),.1)

    def test_full_state_downgrade_recovers_bias_pose_correlation(self):
        old=state(1,15);live=state(1);pressure=dict(kind='pressure',ident=0,size=1,reference=[0.])
        rng=np.random.default_rng(790);j=rng.normal(size=(40,16));r=rng.normal(size=40)
        keep=list(range(6))+[15];gone=list(range(6,15))
        node=conditional(1,[old,pressure],j,r,keep,gone)
        split=split_patch_information(j[:,keep+gone],r,retained_columns=7)
        archive=RetiredPoseArchive('control');archive.ingest(packet(2,[live,pressure],np.zeros(7),[node]))
        value=snapshot(archive);value['information']=information(split['jacobian'],split['residual'])
        native=NativeSurfaceSnapshot.read(value)
        got=native.joint_marginal((Variable(**old),Variable(**pressure)))
        np.testing.assert_allclose(got['observable_covariance'],np.linalg.inv(j.T@j),atol=1e-13)

    def test_old_snapshots_remain_preparable_but_cannot_claim_complete_uncertainty(self):
        native,_,_,_,value=self.case();value.pop('information')
        old=NativeSurfaceSnapshot.read(value)
        with self.assertRaisesRegex(ValueError,'complete_native_joint_information'):old.joint_marginal(old.order)
        self.assertIsNone(old._information)

    def test_schema_dimensions_nonfinite_and_counts_are_rejected(self):
        good=information(np.eye(7),np.zeros(7))
        for change in [dict(schema='other'),dict(scope='conditional'),dict(jacobian=np.zeros((9,7)).tolist()),
                       dict(jacobian=np.zeros((7,8)).tolist()),dict(residual=[float('nan')]*7),
                       dict(imu_intervals=True),dict(dvl_observations=-1),dict(pressure_observations=74)]:
            with self.subTest(change=change),self.assertRaises(ValueError):NativeSurfaceInformation.read(dict(good,**change),7)

    def test_changed_references_expiry_and_workspace_fail_without_owner_mutation(self):
        native,order,_,_,_=self.case();original=native.recover([1,2])['deltas'].copy()
        changed=list(order[0].reference);changed[0]+=.1
        with self.assertRaisesRegex(ValueError,'reference'):
            native.joint_marginal((Variable('state',1,6,tuple(changed)),))
        for tolerance in (True,0,-1,1e-4,float('nan')):
            with self.assertRaises(ValueError):native.joint_marginal(order,relative_tolerance=tolerance)
        native._archive.max_recovery_bytes=1024
        with self.assertRaisesRegex(ValueError,'capacity'):native.joint_marginal(order)
        native._archive.max_recovery_bytes=32*1024*1024
        np.testing.assert_array_equal(native.recover([1,2])['deltas'],original)
        native._archive.expire_through(2,protect=[('state',1)])
        native.joint_marginal((order[0],))
        with self.assertRaisesRegex(ValueError,'expired'):native.joint_marginal((order[1],))


if __name__=='__main__':unittest.main()

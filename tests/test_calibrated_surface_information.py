"""Root and retired exact bias support survive finite delayed likelihoods."""
import unittest
import numpy as np
from posim_slam.constrained_information import split_constrained_information
from posim_slam.native_surface import NativeSurfaceSnapshot,prepare_native_ranges
from posim_slam.retired_pose_archive import RetiredPoseArchive,coordinates
from posim_slam.surface_prior import Variable
from test_retired_pose_archive import state,packet
from test_native_surface import snapshot
from test_native_surface_information import information


def batch_covariance(j,c):
    n=j.shape[1];kkt=np.block([[j.T@j,c.T],[c,np.zeros((len(c),len(c)))]])
    return np.linalg.inv(kkt)[:n,:n]


class CalibratedSurfaceInformationTests(unittest.TestCase):
    def test_live_exact_bias_support_is_not_unbounded_or_a_finite_weight(self):
        variables=[state(1,15),dict(kind='pressure',ident=0,size=1,reference=[0.])]
        rng=np.random.default_rng(987);j=rng.normal(size=(35,16));r=rng.normal(0,.01,35)
        c=np.zeros((3,16));c[:,9:12]=np.eye(3)
        a=RetiredPoseArchive('control');a.ingest(packet(2,variables,np.zeros(16)))
        p=snapshot(a);p['information']=information(j,r)
        p['information'].update(schema='posim-slam-native-surface-information-2',constraint_jacobian=c.tolist(),constraint_residual=[0.]*3)
        native=NativeSurfaceSnapshot.read(p);got=native.joint_marginal(tuple(Variable(**v) for v in variables))
        np.testing.assert_allclose(got['observable_covariance'],batch_covariance(j,c),atol=2e-13)
        np.testing.assert_allclose(c@got['observable_covariance'],0,atol=1e-13)
        self.assertEqual(got['root_constraint_rank'],3);self.assertEqual(got['unbounded_basis'].shape[1],0)
        p['information']['schema']='posim-slam-native-surface-information-1'
        with self.assertRaisesRegex(ValueError,'exact_support_schema'):NativeSurfaceSnapshot.read(p)

    def test_retired_exact_bias_support_and_correlations_survive_delayed_pose_rows(self):
        v=[state(1,15),state(2,15),dict(kind='pressure',ident=0,size=1,reference=[0.])]
        full=tuple(Variable(**x) for x in v);rng=np.random.default_rng(988)
        j=rng.normal(size=(55,31));b=rng.normal(0,.01,55);c=np.zeros((6,31));c[:,9:15]=np.eye(6)
        kept=list(range(15,31));gone=list(range(15))
        split=split_constrained_information(j[:,kept+gone],b,retained_columns=16,
            constraint_jacobian=c[:,kept+gone],constraint_target=np.zeros(6))
        node=dict(id=1,variables=v,kept_indices=kept,removed_indices=gone,
            regression=split['backsub_matrix'],offset=split['backsub_offset'],noise_factor=split['observable_noise_factor'],
            unbounded_basis=split['unobservable_basis'],deterministic_basis=split['deterministic_basis'])
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,v[1:],np.zeros(16),[node]))
        p=snapshot(archive);p['information']=information(split['jacobian'],split['residual'])
        native=NativeSurfaceSnapshot.read(p);recovered=native.joint_marginal(full)
        np.testing.assert_allclose(recovered['observable_covariance'],batch_covariance(j,c),atol=2e-12)
        source=(Variable('state',1,6,full[0].reference[:12]),Variable('plane',0,3,(0,0,1,3)))
        delta=np.r_[native.recover([1])['deltas'],np.zeros(3)]
        extra=rng.normal(size=(10,9));residual=rng.normal(0,.01,10)
        update=prepare_native_ranges(native,order=source,delta=delta,jacobian=extra,residual=residual,
            range_keys=[(1,2,i) for i in range(10)])
        out=update._likelihood;root_j=np.vstack((np.pad(split['jacobian'],((0,0),(0,3))),out.new_live_rows.jacobian))
        root_b=np.r_[split['residual'],out.new_live_rows.offset]
        p=snapshot(out.archive);p['information']=information(root_j,root_b)
        after=NativeSurfaceSnapshot.read(p);all_order=full+(source[-1],)
        extra_full=np.zeros((10,34));keys=coordinates(all_order)
        extra_full[:,[keys.index(k) for k in coordinates(source)]]=extra
        dense=np.vstack((np.pad(j,((0,0),(0,3))),extra_full));hard=np.pad(c,((0,0),(0,3)))
        np.testing.assert_allclose(after.joint_marginal(all_order)['observable_covariance'],batch_covariance(dense,hard),atol=2e-11)
        self.assertEqual(out.archive.nodes[1].deterministic.shape,(15,6))
        self.assertIn('deterministic_basis',update.as_packet()['conditionals'][0])


if __name__=='__main__':unittest.main()

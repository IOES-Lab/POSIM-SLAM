"""Native calibrated-prior ownership compared with dense constrained batches."""
import copy
import gzip
import json
import os
from pathlib import Path
import unittest
import numpy as np
from posim_slam.imu_bias_calibration import InitialImuBiasCalibration
from posim_slam.native_surface import NativeSurfaceSnapshot,prepare_native_ranges
from posim_slam.surface_prior import Variable
from posim_slam.retired_pose_archive import coordinates
from test_surface_owner import Owner,fixture,variables,encode


def batch(j,b,c,d):
    n=j.shape[1];kkt=np.block([[j.T@j,c.T],[c,np.zeros((len(c),len(c)))]])
    return np.linalg.solve(kkt,np.r_[-j.T@b,d])[:n],np.linalg.inv(kkt)[:n,:n]


def snapshot(state):
    return NativeSurfaceSnapshot.read(dict(schema='posim-slam-native-surface-snapshot-1',snapshot_id=1,
        owner_revision=state['owner_revision'],prior_revision=state['prior']['revision'],epoch=state['prior']['epoch'],
        stamp_ns=3*10**9,variables=state['order'],delta=state['delta'],conditionals=state['conditionals'],
        range_ledger=state['prior']['ledger'],archive_closed_through_ns=0,protected_states=[],information=state['information']))


@unittest.skipUnless(os.environ.get('POSIM_SLAM_SURFACE_OWNER_PROBE'),'compiled native owner required')
class NativeBiasOwnerTests(unittest.TestCase):
    def owner(self,args):
        owner=Owner(self._testMethodName);self.addCleanup(owner.close)
        if os.environ.get('POSIM_SLAM_BIAS_OWNER_TRACE'):
            self._trace_count=getattr(self,'_trace_count',0)+1
            path=Path(os.environ['POSIM_SLAM_BIAS_OWNER_TRACE'])/(self._testMethodName+'-'+str(self._trace_count)+'.json.gz')
            def retain():
                path.parent.mkdir(parents=True,exist_ok=True)
                path.write_bytes(gzip.compress(json.dumps(owner.trace,default=encode).encode(),mtime=0))
            self.addCleanup(retain)
        return owner,owner.call(args)['state']

    def test_finite_and_singular_calibration_optimization_retirement_and_covariance_match_KKT(self):
        rng=np.random.default_rng(985);q=np.linalg.qr(rng.normal(size=(6,6)))[0]
        for eigenvalues in ([.01,.02,.04,.1,.2,.3],[0,0,.01,.02,.1,.2],[0]*6):
            with self.subTest(eigenvalues=eigenvalues):
                args=fixture(size=15);cov=(q*np.array(eigenvalues))@q.T
                calibration=InitialImuBiasCalibration(q@(np.array(eigenvalues)*.01),cov,'independent-dense-control')
                baseline_owner,baseline=self.owner(copy.deepcopy(args))
                args['initial_bias']=calibration.packet();owner,before=self.owner(args)
                rows=calibration.rows();j=np.array(before['joint_jacobian']);r=np.array(before['joint_residual'])
                c=np.zeros((len(rows['constraint_jacobian']),j.shape[1]));c[:,9:15]=rows['constraint_jacobian']
                target=-rows['constraint_offset'];start=np.array(before['delta'])
                increment,expected_cov=batch(j,r,c,target-c@start);expected_mode=start+increment
                augmented=np.zeros((len(rows['jacobian']),j.shape[1]));augmented[:,9:15]=rows['jacobian']
                original=np.array(baseline['joint_jacobian']);original_r=np.array(baseline['joint_residual'])
                np.testing.assert_allclose(j.T@j,original.T@original+augmented.T@augmented,atol=2e-9)
                np.testing.assert_allclose(j.T@r,original.T@original_r+augmented.T@rows['offset'],atol=2e-9)
                joint=snapshot(before).joint_marginal(variables(before))
                np.testing.assert_allclose(joint['observable_covariance'],expected_cov,atol=2e-9)
                optimized=owner.call(dict(op='optimize'))['state']
                np.testing.assert_allclose(optimized['delta'],expected_mode,atol=3e-7)
                np.testing.assert_allclose(c@optimized['delta'],target,atol=2e-9)
                after=owner.call(dict(op='retire',removed=[10**9]))['state']
                full=variables(before);keys=coordinates(full);keep=[keys.index(k) for k in coordinates(variables(after))]
                recovered=snapshot(after).joint_marginal(full)
                np.testing.assert_allclose(recovered['observable_covariance'],expected_cov,atol=2e-8)
                root=snapshot(after).joint_marginal(variables(after))
                np.testing.assert_allclose(root['observable_covariance'],expected_cov[np.ix_(keep,keep)],atol=2e-8)
                expected_exact=6-np.linalg.matrix_rank(cov,tol=1e-10)
                self.assertEqual(sum(len(n.get('deterministic_basis',[[]])[0]) for n in after['conditionals']),expected_exact)

    def test_delayed_rows_preserve_retired_exact_support_and_reject_corrupt_replacement(self):
        args=fixture(size=15);args['initial_bias']=InitialImuBiasCalibration(np.zeros(6),np.zeros((6,6)),'exact-initial').packet()
        owner,before=self.owner(args);old_order=variables(before)
        after=owner.call(dict(op='retire',removed=[10**9]))['state'];native=snapshot(after)
        old=native.reference('state',10**9);current=next(v for v in native.order if v.kind=='state')
        order=(Variable('state',old.ident,6,old.reference[:12]),Variable('state',current.ident,6,current.reference[:12]),Variable('plane',0,3,(0,0,1,3)))
        delta=np.r_[native.recover([old.ident,current.ident])['deltas'],np.zeros(3)]
        rng=np.random.default_rng(986);j=rng.normal(0,.2,(12,15));r=rng.normal(0,.02,12)
        update=prepare_native_ranges(native,order=order,delta=delta,jacobian=j,residual=r,range_keys=[(old.ident,4,i) for i in range(12)])
        packet=update.as_packet();request=dict(op='late',jacobian=packet['jacobian'],offset=packet['offset'],new_planes=packet['new_planes'],
            source_order=packet['source_order'],conditionals=packet['conditionals'],keys=packet['range_keys'])
        corrupt=copy.deepcopy(request);corrupt['conditionals'][0].pop('deterministic_basis')
        rejected=owner.call(corrupt,error=True);self.assertIn('exact_support',rejected['error']);self.assertEqual(rejected['state'],after)
        installed=owner.call(request)['state'];full=old_order+(order[-1],)
        keys=coordinates(full);where=[keys.index(k) for k in coordinates(order)]
        baseline_j=np.pad(np.array(before['joint_jacobian']),((0,0),(0,3)))
        extra=np.zeros((12,len(keys)));extra[:,where]=j
        matrix=np.vstack((baseline_j,extra));offset=np.r_[before['joint_residual'],r-j@delta]
        hard=np.zeros((6,len(keys)));hard[:,9:15]=np.eye(6)
        _,cov=batch(matrix,offset,hard,np.zeros(6));got=snapshot(installed).joint_marginal(full)
        np.testing.assert_allclose(got['observable_covariance'],cov,atol=2e-8)
        np.testing.assert_allclose(hard@got['observable_covariance'],0,atol=2e-10)

    def test_unrelated_retirement_and_inertial_downgrade_keep_calibration_owned_once(self):
        args=fixture(size=15);args['initial_bias']=InitialImuBiasCalibration(np.zeros(6),np.zeros((6,6)),'exact-initial').packet()
        owner,before=self.owner(args);full=variables(before);j=np.array(before['joint_jacobian']);r=np.array(before['joint_residual'])
        hard=np.zeros((6,j.shape[1]));hard[:,9:15]=np.eye(6);_,cov=batch(j,r,hard,np.zeros(6))
        unrelated=owner.call(dict(op='retire',removed=[2*10**9]))['state']
        self.assertEqual(len(unrelated['information']['constraint_jacobian']),6)
        np.testing.assert_allclose(snapshot(unrelated).joint_marginal(full)['observable_covariance'],cov,atol=2e-8)
        downgraded=owner.call(dict(op='retire',removed=[],pose_only=[10**9]))['state']
        self.assertNotIn('constraint_jacobian',downgraded['information'])
        np.testing.assert_allclose(snapshot(downgraded).joint_marginal(full)['observable_covariance'],cov,atol=2e-8)
        self.assertEqual(sum(len(n.get('deterministic_basis',[[]])[0]) for n in downgraded['conditionals']),6)


if __name__=='__main__':unittest.main()

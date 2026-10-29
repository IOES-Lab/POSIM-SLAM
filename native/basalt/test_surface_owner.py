"""Actual Basalt owner lifecycle with persistent delayed Gaussian information.

Synthetic likelihoods exercise ownership and algebra, not physical sonar
correspondence, a calibrated range model or live-worker map acceptance.
"""
from dataclasses import asdict
import copy
import gzip
import json
import os
from pathlib import Path
import select
import subprocess
import tempfile
import unittest

import numpy as np

from posim_slam.delayed_surface_update import prepare_delayed_ranges
from posim_slam.ping_joint import split_patch_information
from posim_slam.retired_pose_archive import RetiredPoseArchive,coordinates
from posim_slam.surface_prior import Variable,marginalize_surface,linearize_prior


def encode(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,Variable):return asdict(value)
    return value.item()


def state(stamp,size=6):
    return Variable('state',stamp,size,tuple(np.r_[[stamp/1e9,0,-2],np.eye(3).ravel(),np.zeros(size-6)]))


def fixture(size=6,count=3,*,free=False,depth=True):
    states=[state((i+1)*10**9,size) for i in range(count)];n=size*count;rng=np.random.default_rng(812)
    j=np.zeros((1,n)) if free else rng.normal(0,.2,(n+5,n))+np.r_[np.eye(n),np.zeros((5,n))]
    b=np.zeros(len(j)) if free else rng.normal(0,.01,len(j))
    return dict(op='init',states=states,legacy_j=j,legacy_b=b,
                pressure=[[v.ident,-1.9+i*.01,.04] for i,v in enumerate(states)] if depth else [])


class Owner:
    def __init__(self,name):
        self.name=name;self.trace=[]
        self.stderr=tempfile.TemporaryFile(mode='w+')
        self.process=subprocess.Popen([os.environ['POSIM_SLAM_SURFACE_OWNER_PROBE']],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr,text=True,bufsize=1)

    def close(self):
        self.process.stdin.close()
        try:self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=10)
        self.process.stdout.close();self.stderr.close()
        target=os.environ.get('POSIM_SLAM_SURFACE_OWNER_TRACE')
        if target and self.name in ('test_delayed_native_install_optimization_and_second_retirement_match_dense_system',
                                    'test_dvl_and_pressure_retire_only_their_consumed_readings'):
            root=Path(target);root.mkdir(parents=True,exist_ok=True)
            (root/(self.name+'.json.gz')).write_bytes(gzip.compress(json.dumps(self.trace,default=encode).encode(),mtime=0))

    def call(self,data,*,error=False):
        self.process.stdin.write(json.dumps(data,default=encode,allow_nan=False)+'\n');self.process.stdin.flush()
        if not select.select([self.process.stdout],[],[],30)[0]:raise AssertionError('Native owner response timeout')
        line=self.process.stdout.readline()
        if not line:
            self.stderr.seek(0);raise AssertionError(self.stderr.read()[-4000:])
        value=json.loads(line)
        self.trace.append(dict(request=data,response=value))
        if not error and ('error' in value or 'state_error' in value):raise AssertionError(value)
        if error and 'error' not in value:raise AssertionError('Expected owner rejection')
        return value


def variables(value):return tuple(Variable(**v) for v in value['order'])


def moments(value):
    j=np.array(value['joint_jacobian']);r=np.array(value['joint_residual'])
    return j.T@j,j.T@r,r@r


def original_keys(value):return value['prior']['ledger']['recent']


def conditional(node):
    return dict(id=node.ident,variables=node.variables,kept_indices=node.kept,removed_indices=node.removed,
                regression=node.regression,offset=node.offset,noise_factor=node.noise,unbounded_basis=node.unbounded)


def late_request(value):
    """Run the independent reference on a complete actual-owner snapshot."""
    order=variables(value);delta=np.array(value['delta']);j=np.array(value['joint_jacobian']);r=np.array(value['joint_residual'])
    prior=marginalize_surface(order=order,delta=delta,jacobian=j,residual=r,
        retain=[(v.kind,v.ident,v.size) for v in order],epoch='native-surface-owner')
    archive=RetiredPoseArchive(prior.epoch)
    archive.ingest(dict(epoch=prior.epoch,stamp_ns=3*10**9,variables=value['order'],delta=delta,conditionals=value['conditionals']))
    retired=archive.references[('state',10**9)];resident=next(v for v in order if v.kind=='state')
    plane=Variable('plane',0,3,(0,0,1,3));pressure=next(v for v in order if v.kind=='pressure')
    new_order=(retired,resident,pressure,plane);rng=np.random.default_rng(724)
    current=np.r_[archive.recover([retired.ident])['deltas'],delta[:resident.size],delta[sum(v.size for v in order if v.kind=='state')],np.zeros(3)]
    new_j=rng.normal(0,.4,(12,len(current)));new_r=rng.normal(0,.05,12)
    keys=[(retired.ident,7,i) for i in range(12)]
    updated=prepare_delayed_ranges(prior,archive,order=new_order,delta=current,jacobian=new_j,residual=new_r,
        range_keys=keys,source_stamp_ns=archive.stamp_ns,expected_revision=prior.revision)
    request=dict(op='late',jacobian=updated.new_live_rows.jacobian,offset=updated.new_live_rows.offset,
        new_planes=[plane],source_order=new_order,
        conditionals=[conditional(updated.archive.nodes[i]) for i in updated.changed_conditionals],keys=keys)
    return request,updated


@unittest.skipUnless(os.environ.get('POSIM_SLAM_SURFACE_OWNER_PROBE'),'compiled native persistent owner probe required')
class NativeSurfaceOwnerTests(unittest.TestCase):
    metrics=dict(max_information_error=0.,max_gradient_error=0.,max_squared_residual_error=0.,max_covariance_error=0.,max_solution_error=0.)

    @classmethod
    def tearDownClass(cls):
        print(json.dumps(dict(scope='Synthetic real-owner algebra and lifecycle; no physical sonar map',**cls.metrics),sort_keys=True))

    def owner(self,args=None):
        owner=Owner(self._testMethodName);self.addCleanup(owner.close)
        first=owner.call(args or fixture())['state'];return owner,first

    def assertMoments(self,actual,j,r):
        for key,left,right in zip(('max_information_error','max_gradient_error','max_squared_residual_error'),
                                 moments(actual),(j.T@j,j.T@r,r@r)):
            self.metrics[key]=max(self.metrics[key],float(np.max(np.abs(left-right))))
            np.testing.assert_allclose(left,right,atol=3e-8,rtol=2e-9)

    def assertMarginal(self,before,after):
        old_columns=coordinates(variables(before));keep=[old_columns.index(k) for k in coordinates(variables(after))]
        removed=[i for i in range(len(old_columns)) if i not in keep]
        split=split_patch_information(np.array(before['joint_jacobian'])[:,keep+removed],
            np.array(before['joint_residual']),retained_columns=len(keep))
        self.assertMoments(after,split['jacobian'],split['residual'])

    def retired(self,args=None):
        owner,before=self.owner(args)
        after=owner.call(dict(op='retire',removed=[10**9]))['state']
        self.assertMarginal(before,after)
        return owner,after

    def test_adoption_preserves_legacy_pressure_cross_terms_and_active_readings(self):
        args=fixture();n=len(args['states'])*6
        args['pressure_prior']=dict(states=np.linspace(-.1,.1,n),datum=.8,b=.02,reference=.3)
        owner,actual=self.owner(args);j=np.zeros((len(args['legacy_j'])+1+3,n+1));r=np.zeros(len(j))
        j[:len(args['legacy_j']),:n]=args['legacy_j'];r[:len(args['legacy_b'])]=args['legacy_b']
        j[len(args['legacy_j'])]=np.r_[args['pressure_prior']['states'],.8]
        datum=actual['delta'][n];r[len(args['legacy_j'])]=.02+.8*(datum-.3)
        for i in range(3):
            j[-3+i,6*i+2]=5;j[-3+i,n]=5;r[-3+i]=(datum-.1-i*.01)*5
        self.assertMoments(actual,j,r)
        self.assertTrue(actual['legacy_zero']);self.assertEqual(actual['pressure_count'],3)

    def test_delayed_native_install_optimization_and_second_retirement_match_dense_system(self):
        owner,before=self.retired();request,expected=late_request(before)
        actual=owner.call(request)['state'];delta=np.r_[before['delta'],[0,0,0]]
        rows=linearize_prior(expected.prior,order=expected.prior.order,delta=delta,epoch=expected.prior.epoch)
        self.assertMoments(actual,rows['jacobian'],rows['residual'])
        covariance=np.linalg.inv(moments(actual)[0]);expected_covariance=np.linalg.inv(rows['jacobian'].T@rows['jacobian'])
        self.metrics['max_covariance_error']=float(np.max(np.abs(covariance-expected_covariance)))
        np.testing.assert_allclose(covariance,expected_covariance,atol=1e-10,rtol=1e-9)
        self.assertEqual(len(original_keys(actual)),12);self.assertEqual(actual['pressure_count'],2)
        j=np.array(actual['joint_jacobian']);r=np.array(actual['joint_residual'])
        solution=delta-np.linalg.lstsq(j,r,rcond=1e-12)[0]
        optimized=owner.call(dict(op='optimize'))
        self.assertGreater(optimized['optimization']['accepted'],0)
        self.assertLess(optimized['optimization']['final_cost'],optimized['optimization']['initial_cost'])
        np.testing.assert_allclose(optimized['state']['delta'],solution,atol=2e-7,rtol=1e-7)
        self.metrics['max_solution_error']=float(np.max(np.abs(np.array(optimized['state']['delta'])-solution)))
        after=owner.call(dict(op='retire',removed=[2*10**9]))['state']
        self.assertMarginal(optimized['state'],after)
        self.assertEqual([v['kind'] for v in after['order']],['state','pressure','plane'])
        self.assertEqual(len(after['conditionals']),2);self.assertEqual(after['pressure_count'],1)
        self.assertEqual(original_keys(actual),original_keys(after));self.assertTrue(after['legacy_zero'])

    def test_reuse_and_invalid_archive_leave_the_complete_owner_unchanged(self):
        owner,before=self.retired();request,_=late_request(before)
        incomplete=dict(request,conditionals=[])
        rejected=owner.call(incomplete,error=True);self.assertIn('closure',rejected['error']);self.assertEqual(rejected['state'],before)
        bad=copy.deepcopy(request);bad['conditionals'][0]['noise_factor']=np.zeros_like(bad['conditionals'][0]['noise_factor'])
        rejected=owner.call(bad,error=True);self.assertEqual(rejected['state'],before)
        actual=owner.call(request)['state'];again=dict(request,new_planes=[])
        again['jacobian']=np.zeros((1,len(actual['delta'])));again['offset']=[0.]
        rejected=owner.call(again,error=True);self.assertIn('reused',rejected['error']);self.assertEqual(rejected['state'],actual)

    def test_intervening_optimizer_state_invalidates_a_prepared_snapshot(self):
        owner,before=self.retired();request,_=late_request(before)
        owner.call(dict(op='snapshot'));current=owner.call(dict(op='optimize'))['state']
        self.assertNotEqual(current['delta'],before['delta'])
        rejected=owner.call(request,error=True);self.assertIn('stale',rejected['error']);self.assertEqual(rejected['state'],current)

    def test_trial_blocks_installation_and_restores_pose_plane_and_pressure(self):
        owner,before=self.retired();request,_=late_request(before);installed=owner.call(request)['state']
        trial=dict(op='trial_reject',source_order=installed['order'],jacobian=np.zeros((1,len(installed['delta']))),offset=[0.],keys=[[3*10**9,7,20]])
        out=owner.call(trial);self.assertTrue(out['installation_during_trial_rejected'])
        current=out['state'];self.assertEqual(current['delta'],installed['delta'])
        self.assertEqual(current['prior'],installed['prior']);self.assertEqual(current['conditionals'],installed['conditionals'])
        self.assertEqual(current['pressure_count'],installed['pressure_count'])
        self.assertGreater(current['owner_revision'],installed['owner_revision'])

    def test_transient_seventeenth_state_retires_without_enlarging_live_prior_budget(self):
        owner,before=self.owner(fixture(size=15,count=16,depth=False))
        owner.call(dict(op='append',states=[state(17*10**9,15)]))
        actual=owner.call(dict(op='retire',removed=[10**9]))['state']
        self.assertEqual(sum(v['kind']=='state' for v in actual['order']),16)
        self.assertEqual(len(actual['delta']),241);self.assertEqual(len(actual['conditionals']),1)

    def test_dvl_and_pressure_retire_only_their_consumed_readings(self):
        args=fixture(size=15);args['dvl']=[dict(stamp=v.ident,velocity=[0,0,0]) for v in args['states']]
        args['imu_intervals']=[[10**9,2*10**9],[2*10**9,3*10**9]]
        owner,before=self.owner(args)
        actual=owner.call(dict(op='retire',removed=[10**9],pose_only=[2*10**9]))['state']
        self.assertMarginal(before,actual)
        self.assertEqual(actual['dvl_count'],1);self.assertEqual(actual['pressure_count'],2)
        self.assertEqual(actual['imu_count'],0)
        self.assertEqual([v['size'] for v in actual['order']],[6,15,1])

    def test_fully_retained_imu_interval_stays_active_without_entering_the_prior_twice(self):
        args=fixture(size=15);args['imu_intervals']=[[10**9,2*10**9],[2*10**9,3*10**9]]
        owner,before=self.owner(args)
        actual=owner.call(dict(op='retire',removed=[10**9]))['state']
        self.assertMarginal(before,actual);self.assertEqual(actual['imu_count'],1)

    def test_free_pressure_datum_does_not_create_an_absolute_height_anchor(self):
        owner,actual=self.owner(fixture(free=True));h,_,_=moments(actual)
        gauge=np.zeros(len(h));gauge[2::6][:3]=1;gauge[-1]=-1
        np.testing.assert_allclose(h@gauge,0,atol=1e-12)
        after=owner.call(dict(op='retire',removed=[10**9]))['state'];self.assertMarginal(actual,after)
        gauge=np.zeros(len(after['delta']));gauge[2]=gauge[8]=1;gauge[-1]=-1
        np.testing.assert_allclose(moments(after)[0]@gauge,0,atol=1e-12)

    def test_duplicate_legacy_prior_is_rejected_before_optimization(self):
        owner,_=self.owner();current=owner.call(dict(op='duplicate_prior'))['state']
        rejected=owner.call(dict(op='optimize'),error=True)
        self.assertIn('duplicate_legacy_owner',rejected['error']);self.assertEqual(rejected['state'],current)


if __name__=='__main__':unittest.main()

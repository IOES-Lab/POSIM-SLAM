"""Late raw ranges match a dense batch, including uncertainty and constants."""
import unittest

import numpy as np

from posim_slam.delayed_surface_update import prepare_delayed_ranges
from posim_slam.ping_joint import split_patch_information
from posim_slam.range_surface_joint import linearize_surface_ranges
from posim_slam.retired_pose_archive import RetiredPoseArchive,coordinates
from posim_slam.surface_prior import Variable,SurfacePrior,RangeLedger
from test_range_surface_joint import fixture
from test_retired_pose_archive import conditional,packet,state
from test_surface_prior import variables


def descriptor(v):return dict(kind=v.kind,ident=v.ident,size=v.size,reference=list(v.reference))


def prior_rows(order,j,b):
    # A real marginal owner bounds [J b] by columns+1 before publishing it.
    # Preserve the residual-only constant rather than discarding surplus rows.
    if len(j)>j.shape[1]+1:
        _,compact=np.linalg.qr(np.column_stack((j,b)),mode='reduced')
        j=compact[:,:j.shape[1]];b=compact[:,-1]
    return SurfacePrior(order,j,b,RangeLedger(),'control')


def physical_case():
    args=fixture();order,delta=variables(args);raw=linearize_surface_ranges(**args)
    pressure=Variable('pressure',0,1,(0.,))
    nav_order=order[:3]+(pressure,)
    rng=np.random.default_rng(843);j=rng.normal(size=(42,19));b=rng.normal(0,.02,42)
    source=[descriptor(v) for v in nav_order]
    n1=conditional(1,source,j,b,list(range(6,19)),list(range(6)))
    first=split_patch_information(j[:,list(range(6,19))+list(range(6))],b,retained_columns=13)
    n2=conditional(2,source[1:],first['jacobian'],first['residual'],list(range(6,13)),list(range(6)))
    second=split_patch_information(first['jacobian'][:,list(range(6,13))+list(range(6))],first['residual'],retained_columns=7)
    live_order=nav_order[2:];live=np.r_[delta[12:18],.07]
    archive=RetiredPoseArchive('control');archive.ingest(packet(4,[descriptor(v) for v in live_order],live,[n1,n2]))
    prior=prior_rows(live_order,second['jacobian'],second['residual'])
    kwargs=dict(order=order,delta=delta,jacobian=raw['jacobian'],residual=raw['residual'],
                range_keys=raw['factor_keys'],source_stamp_ns=4,expected_revision=0)
    full_order=nav_order+order[3:];full_j=np.zeros((len(j),25));full_j[:,:19]=j
    new_j=np.zeros((len(raw['jacobian']),25));new_j[:,:18]=raw['jacobian'][:,:18];new_j[:,19:]=raw['jacobian'][:,18:]
    return prior,archive,kwargs,full_order,np.vstack((full_j,new_j)),np.r_[b,raw['residual']-raw['jacobian']@delta]


def full_cost(prior,archive,values):
    total=float(np.linalg.norm(prior.jacobian@np.array([values[k] for k in coordinates(prior.order)])+prior.offset)**2)
    for node in archive.nodes.values():
        keys=node.source_coordinates
        error=np.array([values[keys[i]] for i in node.removed])-node.regression@np.array([values[keys[i]] for i in node.kept])-node.offset
        # Independent least-squares whitening, including rank-deficient noise.
        white=np.linalg.lstsq(node.noise,error,rcond=None)[0]
        total+=float(white@white)
    return total


class DelayedSurfaceTests(unittest.TestCase):
    def assert_batch(self,update,order,j,b):
        keys=coordinates(order);where={key:i for i,key in enumerate(keys)}
        keep=[where[key] for key in coordinates(update.prior.order)];gone=[i for i in range(len(keys)) if i not in keep]
        h=j.T@j;g=j.T@b;oo=h[np.ix_(gone,gone)];ok=h[np.ix_(gone,keep)]
        expected=h[np.ix_(keep,keep)]-ok.T@np.linalg.solve(oo,ok)
        gradient=g[keep]-ok.T@np.linalg.solve(oo,g[gone])
        a=update.prior.jacobian
        np.testing.assert_allclose(a.T@a,expected,atol=2e-9,rtol=2e-10)
        np.testing.assert_allclose(a.T@update.prior.offset,gradient,atol=2e-9,rtol=2e-10)
        np.testing.assert_allclose(np.linalg.inv(a.T@a),np.linalg.inv(h)[np.ix_(keep,keep)],atol=1e-9,rtol=2e-9)
        rng=np.random.default_rng(844)
        for _ in range(10):
            x=rng.normal(0,.03,len(keys));values=dict(zip(keys,x))
            np.testing.assert_allclose(full_cost(update.prior,update.archive,values),np.linalg.norm(j@x+b)**2,atol=2e-8,rtol=2e-11)

    def test_correlated_finite_cone_and_fan_rows_equal_dense_live_posterior(self):
        prior,archive,kwargs,order,j,b=physical_case();before=archive.recover([1,2])['deltas'].copy()
        result=prepare_delayed_ranges(prior,archive,**kwargs)
        self.assert_batch(result,order,j,b)
        self.assertEqual(result.changed_conditionals,(1,2));self.assertEqual(result.prior.revision,1)
        self.assertEqual(result.prior.ledger.recent,frozenset(kwargs['range_keys']))
        self.assertEqual([v.kind for v in result.prior.order],['state','pressure','plane','plane'])
        self.assertGreater(np.linalg.norm(result.prior.jacobian[:,-7:-6]),.01)
        # The native owner can keep its active nonlinear rows. Its new root
        # likelihood contains no second copy of the existing live information.
        old=np.pad(prior.jacobian,((0,0),(0,6)));increment=result.new_live_rows
        np.testing.assert_allclose(result.prior.jacobian.T@result.prior.jacobian,
            old.T@old+increment.jacobian.T@increment.jacobian,atol=2e-9)
        np.testing.assert_allclose(result.prior.jacobian.T@result.prior.offset,
            old.T@prior.offset+increment.jacobian.T@increment.offset,atol=2e-9)
        self.assertFalse(increment.jacobian.flags.writeable);self.assertFalse(increment.offset.flags.writeable)
        np.testing.assert_array_equal(archive.recover([1,2])['deltas'],before)
        self.assertEqual(prior.revision,0);self.assertFalse(prior.ledger.recent)

    def test_updated_conditionals_recover_exact_joint_old_pose_mean_and_covariance(self):
        prior,archive,kwargs,order,j,b=physical_case();out=prepare_delayed_ranges(prior,archive,**kwargs)
        recovered=out.archive.recover([1,2]);old=j[:,:12];root=j[:,12:]
        values=np.array([out.archive.root[k] for k in coordinates(out.prior.order)])
        h=old.T@old;expected=-np.linalg.solve(h,old.T@(root@values+b))
        np.testing.assert_allclose(recovered['deltas'],expected,atol=1e-10)
        conditional_cov=np.linalg.inv(h)
        np.testing.assert_allclose(recovered['observable_conditional_covariance'],conditional_cov,atol=1e-11)
        live_cov=np.linalg.inv(out.prior.jacobian.T@out.prior.jacobian)
        regression=-np.linalg.solve(h,old.T@root)
        joint_cov=conditional_cov+regression@live_cov@regression.T
        np.testing.assert_allclose(joint_cov,np.linalg.inv(j.T@j)[:12,:12],atol=1e-9)
        self.assertGreater(np.linalg.norm(joint_cov-conditional_cov),.01)
        self.assertTrue(recovered['live_coordinates_treated_as_known'])

    def test_second_update_requires_the_updated_prior_and_all_updated_conditionals(self):
        prior,archive,kwargs,order,j,b=physical_case();first=prepare_delayed_ranges(prior,archive,**kwargs)
        new=dict(kwargs,jacobian=kwargs['jacobian']*1.1,residual=kwargs['residual']*1.1+.03,
                 range_keys=[(t,s+20,k) for t,s,k in kwargs['range_keys']],expected_revision=1)
        second=prepare_delayed_ranges(first.prior,first.archive,**new)
        extra=np.zeros((len(new['jacobian']),25));extra[:,:18]=new['jacobian'][:,:18];extra[:,19:]=new['jacobian'][:,18:]
        self.assert_batch(second,order,np.vstack((j,extra)),np.r_[b,new['residual']-new['jacobian']@new['delta']])
        self.assertEqual(second.prior.revision,2)
        self.assertEqual(len(second.prior.ledger.recent),42)

    def test_shared_children_outside_the_changed_ancestor_closure_remain_valid(self):
        rng=np.random.default_rng(845);order=tuple(Variable(**state(t)) for t in (1,2,3,4))
        j=rng.normal(size=(45,24));b=rng.normal(0,.02,45);a=j;offset=b;nodes=[]
        for i in range(3):
            variables=[descriptor(v) for v in order[i:]];n=a.shape[1]
            nodes.append(conditional(i+1,variables,a,offset,list(range(6,n)),list(range(6))))
            split=split_patch_information(a[:,list(range(6,n))+list(range(6))],offset,retained_columns=n-6)
            a=split['jacobian'];offset=split['residual']
        archive=RetiredPoseArchive('control');archive.ingest(packet(5,[descriptor(order[-1])],np.zeros(6),nodes))
        prior=prior_rows(order[-1:],a,offset)
        rows=rng.normal(size=(3,12));r=rng.normal(size=3)
        out=prepare_delayed_ranges(prior,archive,order=(order[1],order[-1]),delta=np.zeros(12),jacobian=rows,residual=r,
                                  range_keys=[(2,9,i) for i in range(3)],source_stamp_ns=5,expected_revision=0)
        self.assertEqual(out.changed_conditionals,(2,3));self.assertIs(out.archive.nodes[1],archive.nodes[1])
        full=np.zeros((3,24));full[:,6:12]=rows[:,:6];full[:,18:]=rows[:,6:]
        self.assert_batch(out,order,np.vstack((j,full)),np.r_[b,r])

    def test_inertial_tail_outside_pose_closure_keeps_its_correlations(self):
        order=(Variable(**state(1,15)),Variable(**state(2)))
        rng=np.random.default_rng(846);j=rng.normal(size=(40,21));b=rng.normal(0,.02,40)
        kept=list(range(6))+list(range(15,21));removed=list(range(6,15))
        n1=conditional(1,[descriptor(v) for v in order],j,b,kept,removed)
        split=split_patch_information(j[:,kept+removed],b,retained_columns=12)
        downgraded=(Variable(**state(1)),order[1])
        n2=conditional(2,[descriptor(v) for v in downgraded],split['jacobian'],split['residual'],list(range(6,12)),list(range(6)))
        last=split_patch_information(split['jacobian'][:,list(range(6,12))+list(range(6))],split['residual'],retained_columns=6)
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,[descriptor(order[1])],np.zeros(6),[n1,n2]))
        prior=prior_rows(order[1:],last['jacobian'],last['residual'])
        rows=rng.normal(size=(2,12));r=rng.normal(size=2)
        out=prepare_delayed_ranges(prior,archive,order=downgraded,delta=np.zeros(12),jacobian=rows,residual=r,
                                  range_keys=[(1,9,i) for i in range(2)],source_stamp_ns=3,expected_revision=0)
        self.assertEqual(out.changed_conditionals,(2,));self.assertEqual(out.peak_front_columns,12)
        full=np.zeros((2,21));full[:,:6]=rows[:,:6];full[:,15:]=rows[:,6:]
        self.assert_batch(out,order,np.vstack((j,full)),np.r_[b,r])

    def test_new_information_changes_free_old_modes_without_filling_live_gauge(self):
        order=(Variable(**state(1)),Variable(**state(2)));j=np.zeros((1,12));b=np.zeros(1)
        node=conditional(1,[descriptor(v) for v in order],j,b,list(range(6,12)),list(range(6)))
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,[descriptor(order[1])],np.zeros(6),[node]))
        prior=SurfacePrior(order[1:],np.zeros((1,6)),np.zeros(1),RangeLedger(),'control')
        rows=np.zeros((3,12));rows[:,:3]=np.eye(3)*10
        out=prepare_delayed_ranges(prior,archive,order=order,delta=np.zeros(12),jacobian=rows,residual=[1.,2.,3.],
            range_keys=[(1,9,i) for i in range(3)],source_stamp_ns=3,expected_revision=0)
        np.testing.assert_allclose(out.prior.jacobian,0,atol=1e-12)
        result=out.archive.recover([1]);self.assertEqual(np.linalg.matrix_rank(result['unbounded_basis']),3)
        np.testing.assert_allclose(result['deltas'][:3],[-.1,-.2,-.3],atol=1e-13)
        np.testing.assert_allclose(result['observable_conditional_covariance'][:3,:3],np.eye(3)*.01,atol=1e-14)
        for x in np.random.default_rng(847).normal(size=(4,12)):
            self.assertAlmostEqual(full_cost(out.prior,out.archive,dict(zip(coordinates(order),x))),np.linalg.norm(rows@x+[1.,2.,3.])**2,places=9)

    def test_reuse_stale_nonfinite_reference_and_capacity_rejections_leave_inputs_exact(self):
        prior,archive,kwargs,_,_,_=physical_case();out=prepare_delayed_ranges(prior,archive,**kwargs)
        prior,archive=out.prior,out.archive
        base=dict(kwargs,expected_revision=1,range_keys=[(t,s+30,k) for t,s,k in kwargs['range_keys']])
        bad=[]
        bad.append(dict(base,expected_revision=0));bad.append(dict(base,source_stamp_ns=5))
        bad.append(dict(base,range_keys=kwargs['range_keys']))
        bad.append(dict(base,max_workspace_bytes=1024))
        changed=base['delta'].copy();changed[12]+=.01;bad.append(dict(base,delta=changed))
        changed=base['jacobian'].copy();changed[0,0]=np.nan;bad.append(dict(base,jacobian=changed))
        variables=list(base['order']);v=variables[0];ref=list(v.reference);ref[0]+=.1
        variables[0]=Variable(v.kind,v.ident,v.size,tuple(ref));bad.append(dict(base,order=variables))
        before=(prior.jacobian.copy(),prior.offset.copy(),archive.recover([1,2])['deltas'].copy(),dict(archive.nodes),dict(archive.root))
        for case in bad:
            with self.assertRaises(ValueError):prepare_delayed_ranges(prior,archive,**case)
            np.testing.assert_array_equal(prior.jacobian,before[0]);np.testing.assert_array_equal(prior.offset,before[1])
            np.testing.assert_array_equal(archive.recover([1,2])['deltas'],before[2])
            self.assertTrue(all(archive.nodes[k] is v for k,v in before[3].items()));self.assertEqual(archive.root,before[4])

    def test_live_only_rows_update_the_prior_without_rewriting_any_conditionals(self):
        prior,archive,_,_,_,_=physical_case();j=np.zeros((1,7));j[0,2]=1.;j[0,6]=-.7
        delta=np.array(list(archive.root.values()))
        out=prepare_delayed_ranges(prior,archive,order=prior.order,delta=delta,jacobian=j,residual=[.2],
                                  range_keys=[(3,9,0)],source_stamp_ns=4,expected_revision=0)
        self.assertEqual(out.changed_conditionals,());self.assertTrue(all(out.archive.nodes[k] is v for k,v in archive.nodes.items()))
        np.testing.assert_allclose(out.prior.jacobian.T@out.prior.jacobian,prior.jacobian.T@prior.jacobian+j.T@j,atol=1e-12)
        kwargs=dict(order=prior.order,delta=delta,jacobian=j,residual=[.3],range_keys=[(3,9,1)],source_stamp_ns=4,expected_revision=1)
        with self.assertRaisesRegex(ValueError,'published_together'):
            prepare_delayed_ranges(out.prior,archive,**kwargs)
        changed=SurfacePrior(out.prior.order,out.prior.jacobian*1.01,out.prior.offset,
                             out.prior.ledger,out.prior.epoch,out.prior.revision)
        with self.assertRaisesRegex(ValueError,'published_together'):
            prepare_delayed_ranges(changed,out.archive,**kwargs)

    def test_long_history_uses_small_fronts_and_matches_independent_gaussian_update(self):
        # 1,200 historical coordinates exceed the dense split's 1,024 columns.
        # The live estimator still owns one six-coordinate pose.
        count=200;nodes=[]
        for i in range(1,count):
            nodes.append(dict(id=i,variables=[state(i),state(i+1)],kept_indices=list(range(6,12)),
                removed_indices=list(range(6)),regression=np.eye(6),offset=np.zeros(6),
                noise_factor=np.eye(6)*.1,unbounded_basis=np.zeros((6,0))))
        archive=RetiredPoseArchive('control');archive.ingest(packet(count+1,[state(count)],np.zeros(6),nodes))
        live=(Variable(**state(count)),);prior=SurfacePrior(live,np.eye(6)*5,np.zeros(6),RangeLedger(),'control')
        sigma=.05;measurement=np.array([.3,-.2,.4]);rows=np.zeros((3,6));rows[:,:3]=np.eye(3)/sigma
        out=prepare_delayed_ranges(prior,archive,order=(Variable(**state(1)),),delta=np.zeros(6),jacobian=rows,
            residual=-measurement/sigma,range_keys=[(1,9,i) for i in range(3)],source_stamp_ns=count+1,expected_revision=0)
        self.assertEqual(len(out.changed_conditionals),199);self.assertEqual(out.peak_front_columns,12)
        root_variance=.2**2;old_conditional_variance=(count-1)*.1**2
        denominator=root_variance+old_conditional_variance+sigma**2
        covariance=np.eye(6)*root_variance
        covariance[:3,:3]-=np.eye(3)*root_variance**2/denominator
        mean=np.r_[root_variance/denominator*measurement,np.zeros(3)]
        actual=-np.linalg.lstsq(out.prior.jacobian,out.prior.offset,rcond=None)[0]
        np.testing.assert_allclose(actual,mean,atol=2e-13)
        np.testing.assert_allclose(np.linalg.inv(out.prior.jacobian.T@out.prior.jacobian),covariance,atol=2e-13)
        for x in np.random.default_rng(848).normal(0,.03,size=(2,count,6)):
            values={('state',i+1,k):x[i,k] for i in range(count) for k in range(6)}
            expected=np.linalg.norm(np.diff(x,axis=0)/.1)**2+np.linalg.norm(x[-1]*5)**2+np.linalg.norm((x[0,:3]-measurement)/sigma)**2
            np.testing.assert_allclose(full_cost(out.prior,out.archive,values),expected,atol=2e-8,rtol=2e-11)

    def test_zero_new_information_never_creates_a_gauge_prior_at_extreme_old_precision(self):
        node=dict(id=1,variables=[state(1),state(2)],kept_indices=list(range(6,12)),removed_indices=list(range(6)),
            regression=np.eye(6),offset=np.arange(6)*.01,noise_factor=np.eye(6)*1e-9,unbounded_basis=np.zeros((6,0)))
        archive=RetiredPoseArchive('control');archive.ingest(packet(3,[state(2)],np.zeros(6),[node]))
        prior=SurfacePrior((Variable(**state(2)),),np.zeros((1,6)),np.zeros(1),RangeLedger(),'control')
        out=prepare_delayed_ranges(prior,archive,order=(Variable(**state(1)),),delta=np.zeros(6),
            jacobian=np.zeros((1,6)),residual=[.3],range_keys=[(1,9,0)],source_stamp_ns=3,expected_revision=0)
        np.testing.assert_array_equal(out.new_live_rows.jacobian,np.zeros_like(out.new_live_rows.jacobian))
        np.testing.assert_array_equal(out.prior.jacobian,np.zeros_like(out.prior.jacobian))
        self.assertAlmostEqual(float(out.prior.offset@out.prior.offset),.09,places=14)


if __name__=='__main__':unittest.main()

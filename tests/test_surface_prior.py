import copy
import unittest

import numpy as np

from posim_slam.inertial import exp_so3,skew,left_jacobian_so3
from posim_slam.range_surface_joint import linearize_surface_ranges,normal_basis
from posim_slam.surface_prior import (Variable,RangeLedger,SurfacePrior,layout,
    linearize_prior,marginalize_surface,MAX_RECENT_KEYS)
from test_range_surface_joint import fixture


def variables(args):
    order=[];delta=[]
    for stamp,state in sorted(args['states'].items()):
        order.append(Variable('state',stamp,6,tuple(np.r_[state['position'],state['rotation_reference'].ravel()])))
        delta.extend(np.r_[np.zeros(3),state['rotation_delta']])
    for ident,plane in sorted(args['planes'].items()):
        order.append(Variable('plane',ident,3,tuple(np.r_[plane['normal_reference'],plane['distance']])))
        delta.extend(np.r_[plane['normal_delta'],0.])
    return tuple(order),np.array(delta)


def embed(j,source,target):
    _,to,count=layout(target);result=np.zeros((len(j),count));column=0
    for v in source:
        start=to[v.key][0];result[:,start:start+v.size]=j[:,column:column+v.size];column+=v.size
    return result


def delayed_case():
    args=fixture();old=linearize_surface_ranges(**args);order,delta=variables(args)
    # The declared synthetic pose prior anchors gauge and has full cross terms.
    rng=np.random.default_rng(717);a=rng.normal(size=(18,18))*.03+np.eye(18)*8.
    initial=np.zeros((18,24));initial[:,:18]=a
    j=np.vstack((old['jacobian'],initial));r=np.r_[old['residual'],initial@delta]
    retained=[('state',t,6) for t in (2,3)]+[('plane',p,3) for p in (0,1)]
    first=marginalize_surface(order=order,delta=delta,jacobian=j,residual=r,retain=retained,
        range_keys=old['factor_keys'],epoch='synthetic-session')
    later=copy.deepcopy(args);later['states']={4:dict(position=np.array([.7,.23,.32]),
        rotation_reference=exp_so3([.03,-.02,.04]),rotation_delta=np.array([.13,-.08,.11]))}
    later['observations']=[dict(row,stamp_ns=4) for row in args['observations'] if row['stamp_ns']==3]
    later['measurement_covariance']=np.eye(7)*.02**2+np.ones((7,7))*.005**2
    new=linearize_surface_ranges(**later);new_order,new_delta=variables(later)
    full_order=order[:3]+new_order[:1]+order[3:]
    full_delta=np.r_[delta[:18],new_delta[:6],delta[18:]]
    current_order=first.order[:2]+new_order[:1]+first.order[2:]
    current_delta=np.r_[delta[6:18],new_delta[:6],delta[18:]]
    # Six conditional motion rows link old/new poses without a new map prior.
    between=np.zeros((6,30));between[:,12:18]=-np.eye(6)*6;between[:,18:24]=np.eye(6)*6
    new_full=np.vstack((embed(new['jacobian'],new_order,full_order),between))
    new_r=np.r_[new['residual'],between@full_delta]
    keep_columns=[*range(6,30)]
    new_current=new_full[:,keep_columns]
    retain=[(v.kind,v.ident,v.size) for v in current_order]
    second=marginalize_surface(order=current_order,delta=current_delta,jacobian=new_current,residual=new_r,
        retain=retain,range_keys=new['factor_keys'],previous=first,epoch='synthetic-session')
    batch_j=np.vstack((embed(j,order,full_order),new_full));batch_r=np.r_[r,new_r]
    batch=marginalize_surface(order=full_order,delta=full_delta,jacobian=batch_j,residual=batch_r,
        retain=retain,range_keys=old['factor_keys']+new['factor_keys'],epoch='synthetic-session')
    return dict(args=args,old=old,order=order,delta=delta,first=first,second=second,batch=batch,
        initial_j=j,initial_r=r,first_retain=retained,
        current_order=current_order,current_delta=current_delta,batch_j=batch_j,batch_r=batch_r,
        full_order=full_order,full_delta=full_delta,new_j=new_current,new_r=new_r,new_keys=new['factor_keys'])


class SurfacePriorTests(unittest.TestCase):
    def test_delayed_new_ranges_preserve_batch_information_gradient_cost_and_covariance(self):
        case=delayed_case();sequential=case['second'];batch=case['batch'];d=case['current_delta']
        a=linearize_prior(sequential,order=case['current_order'],delta=d,epoch='synthetic-session')
        b=linearize_prior(batch,order=case['current_order'],delta=d,epoch='synthetic-session')
        np.testing.assert_allclose(a['jacobian'].T@a['jacobian'],b['jacobian'].T@b['jacobian'],atol=1e-9)
        np.testing.assert_allclose(a['jacobian'].T@a['residual'],b['jacobian'].T@b['residual'],atol=1e-9)
        rng=np.random.default_rng(719)
        for _ in range(5):
            step=rng.normal(0,.003,len(d))
            self.assertAlmostEqual(np.linalg.norm(a['residual']+a['jacobian']@step)**2,
                np.linalg.norm(b['residual']+b['jacobian']@step)**2,places=8)
        h=case['batch_j'].T@case['batch_j'];direct=np.linalg.inv(h)[6:,6:]
        np.testing.assert_allclose(np.linalg.inv(a['jacobian'].T@a['jacobian']),direct,atol=1e-9)
        self.assertEqual(sequential.ledger.recent,batch.ledger.recent)
        self.assertEqual(sequential.revision,1)
        # Plane estimates are correlated with poses; conditioning them at their
        # fitted value is falsely more confident than the joint posterior.
        pose=18;true_pose=np.linalg.inv(a['jacobian'].T@a['jacobian'])[:pose,:pose]
        fixed_plane=np.linalg.inv((a['jacobian'].T@a['jacobian'])[:pose,:pose])
        self.assertGreater(np.linalg.norm(true_pose-fixed_plane),.001)
        self.assertGreaterEqual(np.linalg.eigvalsh(true_pose-fixed_plane).min(),-1e-10)

    def test_global_gauge_is_not_filled_during_surface_retention(self):
        args=fixture();raw=linearize_surface_ranges(**args);order,delta=variables(args)
        keep=[(v.kind,v.ident,v.size) for v in order if v.key!=('state',1)]
        prior=marginalize_surface(order=order,delta=delta,jacobian=raw['jacobian'],residual=raw['residual'],
            retain=keep,range_keys=raw['factor_keys'],epoch='unanchored')
        gauge=np.zeros((24,6))
        for i,(stamp,state) in enumerate(sorted(args['states'].items())):
            gauge[6*i:6*i+3,:3]=np.eye(3);gauge[6*i:6*i+3,3:]=-skew(state['position'])
            gauge[6*i+3:6*i+6,3:]=np.linalg.inv(left_jacobian_so3(state['rotation_delta']))
        for i,p in args['planes'].items():
            basis=normal_basis(p['normal_reference']);u=basis@p['normal_delta'];n=exp_so3(u)@p['normal_reference']
            coordinates=left_jacobian_so3(u)@basis
            gauge[18+3*i:20+3*i,3:]=np.linalg.lstsq(-skew(n)@coordinates,-skew(n),rcond=None)[0]
            gauge[20+3*i,:3]=n
        np.testing.assert_allclose(prior.jacobian@gauge[6:],0,atol=2e-11)

    def test_unobserved_plane_stays_unobservable_and_no_epsilon_prior_is_added(self):
        state=Variable('state',1,6,tuple(np.r_[np.zeros(3),np.eye(3).ravel()]))
        plane=Variable('plane',0,3,(0.,0.,1.,3.));j=np.zeros((1,9));j[0,2]=-1;j[0,8]=1
        prior=marginalize_surface(order=(state,plane),delta=np.zeros(9),jacobian=j,residual=[.7],
            retain=[('plane',0,3)],range_keys=[(1,0,0)],epoch='free')
        np.testing.assert_array_equal(prior.jacobian,np.zeros((1,3)))
        np.testing.assert_array_equal(prior.offset,np.zeros(1))

    def test_pose_only_downgrade_marginalizes_velocity_and_bias_without_changing_reference(self):
        reference=tuple(np.r_[np.zeros(3),np.eye(3).ravel(),np.zeros(9)])
        state=Variable('state',1,15,reference);plane=Variable('plane',2,3,(0.,0.,1.,3.))
        rng=np.random.default_rng(41);j=rng.normal(size=(30,18));r=rng.normal(size=30);delta=rng.normal(0,.03,18)
        prior=marginalize_surface(order=(state,plane),delta=delta,jacobian=j,residual=r,
            retain=[('plane',2,3),('state',1,6)],epoch='downcast')
        self.assertEqual(prior.order[1],Variable('state',1,6,reference[:12]))
        # Compare to independent normal-equation Schur complement.
        keep=[15,16,17,*range(6)];remove=list(range(6,15));h=j.T@j;g=j.T@r
        expected=h[np.ix_(keep,keep)]-h[np.ix_(keep,remove)]@np.linalg.solve(h[np.ix_(remove,remove)],h[np.ix_(remove,keep)])
        grad=g[keep]-h[np.ix_(keep,remove)]@np.linalg.solve(h[np.ix_(remove,remove)],g[remove])
        np.testing.assert_allclose(prior.jacobian.T@prior.jacobian,expected,atol=1e-12)
        np.testing.assert_allclose(prior.jacobian.T@(prior.jacobian@delta[keep]+prior.offset),grad,atol=1e-12)

    def test_original_readings_cannot_be_reused_after_cutoff_compaction(self):
        ledger=RangeLedger().admit([(1,0,1),(2,0,1)],close_through_ns=1)
        self.assertEqual(ledger.recent,frozenset({(2,0,1)}))
        for keys in ([(1,0,1)],[(1,0,99)],[(2,0,1)],[(3,0,1),(3,0,1)]):
            with self.assertRaisesRegex(ValueError,'reused'):ledger.admit(keys)
        with self.assertRaisesRegex(ValueError,'monotonic'):ledger.admit([],close_through_ns=0)
        with self.assertRaisesRegex(ValueError,'bounded'):
            RangeLedger(frozenset((i+1,0,0) for i in range(MAX_RECENT_KEYS+1)))

    def test_pressure_datum_cross_information_is_retained_with_unknown_plane(self):
        state=Variable('state',1,6,tuple(np.r_[np.zeros(3),np.eye(3).ravel()]))
        plane=Variable('plane',0,3,(0,0,1,3));datum=Variable('pressure',0,1,(.2,))
        rng=np.random.default_rng(341);j=rng.normal(size=(25,10));r=rng.normal(size=25);delta=rng.normal(0,.01,10)
        prior=marginalize_surface(order=(state,plane,datum),delta=delta,jacobian=j,residual=r,
            retain=[('plane',0,3),('pressure',0,1)],epoch='depth-surface')
        h=j.T@j;g=j.T@r
        expected=h[6:,6:]-h[6:,:6]@np.linalg.solve(h[:6,:6],h[:6,6:])
        gradient=g[6:]-h[6:,:6]@np.linalg.solve(h[:6,:6],g[:6])
        np.testing.assert_allclose(prior.jacobian.T@prior.jacobian,expected,atol=1e-12)
        np.testing.assert_allclose(prior.jacobian.T@(prior.jacobian@delta[6:]+prior.offset),gradient,atol=1e-12)
        self.assertGreater(np.linalg.norm(expected[:3,3]),.1)
        scalar=marginalize_surface(order=prior.order,delta=delta[6:],jacobian=np.zeros((0,4)),residual=np.zeros(0),
            retain=[('pressure',0,1)],previous=prior,epoch='depth-surface')
        unchanged=marginalize_surface(order=scalar.order,delta=[delta[-1]],jacobian=np.zeros((0,1)),residual=np.zeros(0),
            retain=[('pressure',0,1)],previous=scalar,epoch='depth-surface')
        np.testing.assert_allclose(scalar.jacobian.T@scalar.jacobian,unchanged.jacobian.T@unchanged.jacobian,atol=1e-12)

    def test_reused_factor_missing_state_changed_reference_and_session_are_rejected(self):
        case=delayed_case();prior=case['first'];order=case['current_order'];delta=case['current_delta']
        kwargs=dict(order=order,delta=delta,jacobian=case['new_j'],residual=case['new_r'],
            retain=[(v.kind,v.ident,v.size) for v in order],range_keys=case['new_keys'],epoch='synthetic-session')
        with self.assertRaisesRegex(ValueError,'reused'):
            marginalize_surface(**kwargs,previous=case['second'])
        with self.assertRaisesRegex(ValueError,'resident'):
            marginalize_surface(**dict(kwargs,range_keys=[(1,4,5)]),previous=prior)
        moved=list(order);v=moved[0];ref=list(v.reference);ref[0]+=.1
        moved[0]=Variable(v.kind,v.ident,v.size,tuple(ref))
        with self.assertRaisesRegex(ValueError,'reference'):
            linearize_prior(prior,order=moved,delta=delta,epoch='synthetic-session')
        with self.assertRaisesRegex(ValueError,'epoch'):
            linearize_prior(prior,order=order,delta=delta,epoch='different-session')


if __name__=='__main__':unittest.main()

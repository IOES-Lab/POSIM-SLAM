"""Cross-language persistent unknown-plane and deduplication controls."""
from dataclasses import asdict,is_dataclass
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
from test_surface_prior import delayed_case
from posim_slam.surface_prior import (Variable,SurfacePrior,RangeLedger,
    linearize_prior,marginalize_surface)


def encode(value):
    if is_dataclass(value):return asdict(value)
    if isinstance(value,frozenset):return sorted(value)
    return value.tolist()


def decode(value):
    return SurfacePrior(tuple(Variable(**v) for v in value['order']),np.array(value['jacobian']),
        np.array(value['offset']),RangeLedger(value['ledger']['recent'],value['ledger']['closed_through_ns']),
        value['epoch'],value['revision'])


def first_input(case):
    return dict(order=case['order'],delta=case['delta'],jacobian=case['initial_j'],residual=case['initial_r'],
        retain=case['first_retain'],range_keys=case['old']['factor_keys'],epoch='synthetic-session')


def next_input(case,previous):
    return dict(order=case['current_order'],delta=case['current_delta'],jacobian=case['new_j'],
        residual=case['new_r'],retain=[(v.kind,v.ident,v.size) for v in case['current_order']],
        range_keys=case['new_keys'],previous=previous,epoch='synthetic-session')


@unittest.skipUnless(os.environ.get('POSIM_SLAM_SURFACE_PRIOR_PROBE'),'compiled persistent surface prior probe required')
class NativeSurfacePriorTests(unittest.TestCase):
    def probe(self,args):
        return subprocess.run([os.environ['POSIM_SLAM_SURFACE_PRIOR_PROBE']],input=json.dumps(args,default=encode,allow_nan=False),
            text=True,capture_output=True,timeout=30)

    def compare(self,args,reference=None):
        if reference is None:reference=marginalize_surface(**args)
        result=self.probe(args);self.assertEqual(result.returncode,0,result.stderr);data=json.loads(result.stdout);actual=decode(data)
        self.assertEqual(actual.order,reference.order);self.assertEqual(actual.ledger,reference.ledger)
        self.assertEqual((actual.epoch,actual.revision),(reference.epoch,reference.revision))
        for left,right in ((actual.jacobian.T@actual.jacobian,reference.jacobian.T@reference.jacobian),
                           (actual.jacobian.T@actual.offset,reference.jacobian.T@reference.offset),
                           (actual.offset@actual.offset,reference.offset@reference.offset)):
            np.testing.assert_allclose(left,right,atol=2e-8,rtol=2e-10)
        return data,actual

    def test_delayed_surface_prior_roundtrip_equals_single_batch_system(self):
        case=delayed_case();first_data,first=self.compare(first_input(case),case['first'])
        args=next_input(case,first_data);second_data,second=self.compare(args,case['second'])
        np.testing.assert_allclose(second.jacobian.T@second.jacobian,
            case['batch'].jacobian.T@case['batch'].jacobian,atol=2e-8,rtol=2e-10)
        self.assertNotIn(('state',1),[v.key for v in second.order])
        self.assertIn(('plane',0),[v.key for v in second.order])

    def test_zero_information_and_inertial_pose_downgrade(self):
        state=Variable('state',1,15,tuple(np.r_[np.zeros(3),np.eye(3).ravel(),np.zeros(9)]))
        plane=Variable('plane',0,3,(0,0,1,3));rng=np.random.default_rng(88)
        args=dict(order=(state,plane),delta=rng.normal(0,.03,18),jacobian=rng.normal(size=(30,18)),
            residual=rng.normal(size=30),retain=[('plane',0,3),('state',1,6)],epoch='downcast')
        self.compare(args)
        j=np.zeros((1,18));j[0,2]=-1;j[0,17]=1
        self.compare(dict(args,delta=np.zeros(18),jacobian=j,residual=np.array([.7]),retain=[('plane',0,3)]))
        datum=Variable('pressure',0,1,(.2,));rng=np.random.default_rng(341)
        self.compare(dict(order=(state,plane,datum),delta=rng.normal(0,.01,19),jacobian=rng.normal(size=(30,19)),
            residual=rng.normal(size=30),retain=[('plane',0,3),('pressure',0,1)],epoch='depth-surface'))
        args=dict(order=(datum,),delta=np.array([.03]),jacobian=np.array([[1.],[2.],[3.]]),
            residual=np.array([.1,.2,.4]),retain=[('pressure',0,1)],epoch='scalar-only')
        data,prior=self.compare(args)
        self.compare(dict(args,jacobian=np.zeros((0,1)),residual=np.zeros(0),previous=prior))

    def test_mapping_into_permuted_order_keeps_reference_and_accumulated_delta(self):
        case=delayed_case();data,prior=self.compare(first_input(case),case['first'])
        order=case['current_order'][::-1];columns={};offset=0
        for v in case['current_order']:columns[v.key]=slice(offset,offset+v.size);offset+=v.size
        delta=np.concatenate([case['current_delta'][columns[v.key]] for v in order])
        args=dict(operation='linearize',previous=data,order=order,delta=delta,epoch='synthetic-session')
        result=self.probe(args);self.assertEqual(result.returncode,0,result.stderr);actual=json.loads(result.stdout)
        reference=linearize_prior(prior,order=order,delta=delta,epoch='synthetic-session')
        j=np.array(actual['jacobian']);r=np.array(actual['residual'])
        np.testing.assert_allclose(j.T@j,reference['jacobian'].T@reference['jacobian'],atol=2e-8)
        np.testing.assert_allclose(j.T@r,reference['jacobian'].T@reference['residual'],atol=2e-8)

    def test_closed_ledger_rejects_seen_and_unseen_old_ranges_after_pruning(self):
        case=delayed_case();data,prior=self.compare(first_input(case),case['first'])
        kwargs=dict(order=prior.order,delta=np.r_[case['delta'][6:18],case['delta'][18:]],
            jacobian=np.zeros((0,18)),residual=np.zeros(0),retain=[(v.kind,v.ident,v.size) for v in prior.order],
            epoch='synthetic-session',previous=prior,close_through_ns=3)
        closed_data,closed=self.compare(kwargs)
        self.assertEqual(closed.ledger.recent,frozenset())
        for key in ((3,0,0),(3,99,99)):
            args=dict(kwargs,previous=closed_data,jacobian=np.zeros((1,18)),residual=np.zeros(1),range_keys=[key])
            self.assertEqual(self.probe(args).returncode,2)

    def test_new_rows_reject_reuse_changed_reference_session_and_nonresident_pose(self):
        case=delayed_case();data,first=self.compare(first_input(case),case['first'])
        second_data,second=self.compare(next_input(case,data),case['second'])
        self.assertEqual(self.probe(next_input(case,second_data)).returncode,2)
        changed=list(case['current_order']);v=changed[0];ref=list(v.reference);ref[0]+=.1
        changed[0]=Variable(v.kind,v.ident,v.size,tuple(ref))
        args=next_input(case,data)
        for altered in (dict(args,order=changed),dict(args,epoch='other'),dict(args,range_keys=[(1,0,99)])):
            self.assertEqual(self.probe(altered).returncode,2)


if __name__=='__main__':unittest.main()

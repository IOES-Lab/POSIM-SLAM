"""Real-owner accumulated coordinates after source-pose marginalization."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
from test_surface_prior import delayed_case
from test_surface_prior_native import encode,decode,first_input,next_input
from posim_slam.surface_prior import Variable,linearize_prior,marginalize_surface


@unittest.skipUnless(os.environ.get('POSIM_SLAM_SURFACE_PRIOR_OWNER_PROBE'),'compiled Basalt persistent-prior owner probe required')
class PersistentPriorOwnerTests(unittest.TestCase):
    def probe(self,args):
        return subprocess.run([os.environ['POSIM_SLAM_SURFACE_PRIOR_OWNER_PROBE']],
            input=json.dumps(args,default=encode,allow_nan=False),text=True,capture_output=True,timeout=30)

    def compare(self,args,reference):
        result=self.probe(args);self.assertEqual(result.returncode,0,result.stderr);actual=json.loads(result.stdout);prior=decode(actual['prior'])
        self.assertEqual([(v.kind,v.ident,v.size) for v in prior.order],[(v.kind,v.ident,v.size) for v in reference.order])
        # Sophus canonicalizes the authored matrix through a unit quaternion.
        # Capture those actual owner references; cross-language authored matrices
        # can differ by roundoff. Runtime prior/reference matching stays exact.
        for actual_reference,authored in zip(prior.order,reference.order):
            np.testing.assert_allclose(actual_reference.reference,authored.reference,atol=4*np.finfo(float).eps,rtol=0)
        self.assertEqual(prior.ledger,reference.ledger)
        np.testing.assert_allclose(prior.jacobian.T@prior.jacobian,reference.jacobian.T@reference.jacobian,atol=2e-8,rtol=2e-10)
        np.testing.assert_allclose(prior.jacobian.T@prior.offset,reference.jacobian.T@reference.offset,atol=2e-8,rtol=2e-10)
        if 'maximum_owner_derivative_error' in actual:self.assertLess(actual['maximum_owner_derivative_error'],2e-6)
        self.assertFalse(actual['live_worker_updated']);return actual

    def test_delayed_plane_information_uses_actual_remaining_owner_poses(self):
        case=delayed_case();first=self.compare(first_input(case),case['first'])
        second=self.compare(next_input(case,first['prior']),case['second'])
        self.assertEqual(second['resident_pose_ids'],[2,3,4]);self.assertNotIn(1,second['resident_pose_ids'])
        current=linearize_prior(case['first'],order=case['current_order'],delta=case['current_delta'],epoch='synthetic-session')
        j=np.array(second['prior_jacobian']);r=np.array(second['prior_residual'])
        np.testing.assert_allclose(j.T@r,current['jacobian'].T@current['residual'],atol=2e-8)

    def test_full_inertial_plane_pressure_coordinates_and_reference_guards(self):
        state=Variable('state',1,15,tuple(np.r_[np.zeros(3),np.eye(3).ravel(),[.1,.2,.3],[.01,.02,.03],[.04,.05,.06]]))
        plane=Variable('plane',0,3,(0,0,1,3));datum=Variable('pressure',0,1,(.2,));rng=np.random.default_rng(891)
        order=(state,plane,datum);delta=rng.normal(0,.03,19);keep=[(v.kind,v.ident,v.size) for v in order]
        args=dict(order=order,delta=delta,jacobian=rng.normal(size=(30,19)),residual=rng.normal(size=30),retain=keep,epoch='inertial-depth')
        reference=marginalize_surface(**args);first=self.compare(args,reference)
        next_args=dict(args,jacobian=np.zeros((0,19)),residual=np.zeros(0),previous=first['prior'])
        expected=marginalize_surface(**dict(next_args,previous=reference));self.compare(next_args,expected)
        self.assertEqual(self.probe(dict(next_args,remove_state=1)).returncode,2)
        case=delayed_case();first=self.compare(first_input(case),case['first']);args=next_input(case,first['prior'])
        self.assertEqual(self.probe(dict(args,change_pose_reference=2)).returncode,2)


if __name__=='__main__':unittest.main()

import json
import os
import subprocess
import unittest
import numpy as np
from posim_slam.ground_surface_noise import surface_range_covariance
from posim_slam.inertial import exp_so3
from posim_slam.ping_joint import eliminate_patch


class SurfaceNoiseTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.default_rng(7)
        self.points=rng.normal(size=(12,3));self.incidence=-rng.uniform(.2,.8,12)

    def covariance(self,points=None,incidence=None,sigma=.04,length=.7):
        return surface_range_covariance(self.points if points is None else points,
            self.incidence if incidence is None else incidence,sigma_m=sigma,length_m=length)

    def test_metric_invariance_incidence_scaling_and_zero_model(self):
        c=self.covariance();rotation=exp_so3([.3,-.2,.1]);order=np.arange(12)[::-1]
        np.testing.assert_allclose(c,self.covariance(self.points@rotation.T+[12,-8,3]),atol=1e-16)
        np.testing.assert_allclose(c[np.ix_(order,order)],self.covariance(self.points[order],self.incidence[order]),atol=1e-16)
        np.testing.assert_allclose(np.diag(c),(.04/self.incidence)**2,atol=1e-16)
        np.testing.assert_allclose(self.covariance(incidence=self.incidence/2),4*c,atol=1e-16)
        np.testing.assert_array_equal(self.covariance(sigma=0),np.zeros((12,12)))
        self.assertGreater(np.linalg.eigvalsh(c).min(),0)

    def test_matches_eliminating_explicit_shared_surface_variables(self):
        rng=np.random.default_rng(5);j=rng.normal(size=(12,6));r=rng.normal(size=12)*.03
        k=self.covariance(incidence=np.ones(12))
        field=np.linalg.solve(np.linalg.cholesky(k),np.eye(12))
        observation=np.diag(1/self.incidence)
        whole=np.vstack((np.column_stack((j,observation))/.03,np.column_stack((np.zeros((12,6)),field))))
        residual=np.r_[r/.03,np.zeros(12)]
        reduced=eliminate_patch(whole,residual,retained_columns=6)
        covariance=np.eye(12)*.03**2+self.covariance()
        np.testing.assert_allclose(reduced['jacobian'].T@reduced['jacobian'],j.T@np.linalg.solve(covariance,j),atol=2e-8)
        np.testing.assert_allclose(reduced['jacobian'].T@reduced['residual'],j.T@np.linalg.solve(covariance,r),atol=2e-9)

    def test_rejects_unbounded_and_grazing_inputs(self):
        for kwargs in ({'sigma':-.01},{'sigma':.3},{'length':0},{'incidence':np.zeros(12)},
                       {'points':np.zeros((513,3))},{'points':np.full((12,3),np.nan)}):
            with self.subTest(kwargs=list(kwargs)),self.assertRaises(ValueError):self.covariance(**kwargs)

    @unittest.skipUnless(os.environ.get('POSIM_SLAM_GROUND_PROBE'),'native ground probe required')
    def test_native_matches_reference_covariance(self):
        payload=dict(surface_points=self.points.tolist(),surface_incidence=self.incidence[:,None].tolist(),
                     surface_sigma_m=.04,surface_length_m=.7)
        result=subprocess.run([os.environ['POSIM_SLAM_GROUND_PROBE']],input=json.dumps(payload),text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr)
        np.testing.assert_allclose(json.loads(result.stdout)['surface_covariance'],self.covariance(),atol=1e-16)

"""Actual Basalt owner derivative, optimization and retirement controls."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
from test_range_surface_joint import fixture,supported_fixture
from posim_slam.ping_joint import eliminate_patch
from posim_slam.range_surface_joint import linearize_surface_ranges


@unittest.skipUnless(os.environ.get('POSIM_SLAM_RANGE_SURFACE_OWNER_PROBE'),'compiled Basalt surface owner probe required')
class RangeSurfaceOwnerTests(unittest.TestCase):
    def compare(self,args,prior=False):
        reference=linearize_surface_ranges(**args);n=6*len(args['states'])
        if prior:args=dict(args,prior_sqrt=np.eye(n)*10.)
        result=subprocess.run([os.environ['POSIM_SLAM_RANGE_SURFACE_OWNER_PROBE']],
            input=json.dumps(args,default=lambda value:value.tolist(),allow_nan=False),
            text=True,capture_output=True,timeout=30)
        self.assertEqual(result.returncode,0,result.stderr);actual=json.loads(result.stdout)
        np.testing.assert_allclose(actual['jacobian'],reference['jacobian'],atol=2e-10,rtol=1e-10)
        np.testing.assert_allclose(actual['residual'],reference['residual'],atol=2e-10,rtol=1e-10)
        reduced=eliminate_patch(reference['jacobian'],reference['residual'],retained_columns=n)
        np.testing.assert_allclose(actual['reduced_H'],reduced['jacobian'].T@reduced['jacobian'],atol=2e-8,rtol=1e-9)
        np.testing.assert_allclose(actual['reduced_g'],reduced['jacobian'].T@reduced['residual'],atol=2e-8,rtol=1e-9)
        self.assertEqual(actual['observation_stamps'],sorted(args['states']))
        self.assertEqual(actual['nuisance_rank'],reduced['nuisance_rank'])
        self.assertLess(actual['maximum_derivative_error'],2e-6)
        if prior:
            # Compare the real retirement result to one dense joint elimination
            # of the first pose and every shared surface, retaining all others.
            j=reference['jacobian'];r=reference['residual']
            extra=np.zeros((n,j.shape[1]));extra[:,:n]=args['prior_sqrt']
            all_j=np.vstack([j,extra]);all_r=np.r_[r,np.zeros(n)]
            order=[*range(6,n),*range(6),*range(n,j.shape[1])]
            retired=eliminate_patch(all_j[:,order],all_r,retained_columns=n-6)
            np.testing.assert_allclose(actual['marginal_H'],retired['jacobian'].T@retired['jacobian'],atol=2e-8,rtol=1e-9)
            self.assertEqual(actual['retired_stamps'],sorted(args['states']))
            self.assertEqual(actual['retired_planes'],sorted(args['planes']))
            self.assertGreater(actual['optimization']['accepted'],0)
            self.assertLess(actual['optimization']['final_cost'],actual['optimization']['initial_cost'])
            self.assertEqual(actual['installation'],dict(remaining_planes=0,remaining_groups=0,
                remaining_poses=len(args['states'])-1,replay_rejected=True,retired_source_rejected=True))
        return actual

    def test_accumulated_coordinates_dense_prior_optimization_and_retirement(self):
        self.compare(fixture(),prior=True)

    def test_raw_support_sources_are_owner_dependencies(self):
        self.compare(supported_fixture())


if __name__=='__main__':unittest.main()

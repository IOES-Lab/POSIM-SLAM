"""Native/source controls for conditional multi-surface raw range blocks."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tests'))
from test_range_surface_joint import fixture,supported_fixture
from posim_slam.inertial import exp_so3
from posim_slam.range_surface_joint import linearize_surface_ranges


@unittest.skipUnless(os.environ.get('POSIM_SLAM_RANGE_SURFACE_PROBE'),'compiled native surface-range probe required')
class NativeSurfaceRangeTests(unittest.TestCase):
    def probe(self,args):
        text=json.dumps(args,default=lambda value:value.tolist(),allow_nan=False)
        return subprocess.run([os.environ['POSIM_SLAM_RANGE_SURFACE_PROBE']],input=text,
            text=True,capture_output=True,timeout=15)

    def compare(self,args):
        reference=linearize_surface_ranges(**args);result=self.probe(args)
        self.assertEqual(result.returncode,0,result.stderr);actual=json.loads(result.stdout)
        for name in ('jacobian','residual','unwhitened_jacobian','unwhitened_residual',
                     'measurement_covariance','predicted_ranges','conditional_hits'):
            np.testing.assert_allclose(actual[name],reference[name],rtol=1e-10,atol=2e-10,err_msg=name)
        for name in ('state_stamps_ns','plane_ids','factor_indices','support_only_indices'):
            self.assertEqual(actual[name],reference[name])
        self.assertEqual(actual['source_keys'],[list(key) for key in reference['source_keys']])
        for ident,points in reference['support_seeds'].items():
            np.testing.assert_allclose(actual['support_seeds'][str(ident)],points,atol=1e-12)
        self.assertFalse(actual['map_qualified']);self.assertFalse(actual['physical_identity_verified'])
        return actual

    def test_accumulated_rotations_multiple_surfaces_and_full_covariance(self):
        args=fixture();self.compare(args);rng=np.random.default_rng(571)
        for _ in range(6):
            other=copy.deepcopy(args)
            for state in other['states'].values():state['rotation_delta']+=rng.normal(0,.025,3)
            for plane in other['planes'].values():plane['normal_delta']+=rng.normal(0,.015,2)
            order=rng.permutation(len(args['observations']))
            other['observations']=[other['observations'][i] for i in order]
            other['measurement_covariance']=args['measurement_covariance'][np.ix_(order,order)]
            self.compare(other)

    def test_current_raw_support_and_global_gauge(self):
        args=supported_fixture();original=self.compare(args)
        q=exp_so3([.2,-.15,.3]);shift=np.array([2.,-1.,.7]);moved=copy.deepcopy(args)
        for state in moved['states'].values():
            state['position']=q@state['position']+shift;state['attitude']=q@state['attitude']
        for plane in moved['planes'].values():
            plane['normal']=q@plane['normal'];plane['distance']+=plane['normal']@shift
        actual=self.compare(moved)
        np.testing.assert_allclose(actual['residual'],original['residual'],atol=1e-12)
        # Invert the signed plane without changing its physical geometry.
        for plane in moved['planes'].values():plane['normal']=-plane['normal'];plane['distance']=-plane['distance']
        self.compare(moved)

    def test_native_derivatives_at_nonzero_accumulated_coordinates(self):
        args=fixture();original=self.compare(args);epsilon=1e-6
        for column in range(len(original['jacobian'][0])):
            plus=copy.deepcopy(args);minus=copy.deepcopy(args)
            if column<18:
                stamp=column//6+1;axis=column%6;field='position' if axis<3 else 'rotation_delta';axis%=3
                plus['states'][stamp][field][axis]+=epsilon;minus['states'][stamp][field][axis]-=epsilon
            else:
                ident=(column-18)//3;axis=(column-18)%3
                if axis<2:
                    plus['planes'][ident]['normal_delta'][axis]+=epsilon;minus['planes'][ident]['normal_delta'][axis]-=epsilon
                else:
                    plus['planes'][ident]['distance']+=epsilon;minus['planes'][ident]['distance']-=epsilon
            positive=self.probe(plus);negative=self.probe(minus)
            self.assertEqual(positive.returncode,0,positive.stderr);self.assertEqual(negative.returncode,0,negative.stderr)
            numeric=(np.array(json.loads(positive.stdout)['residual'])-json.loads(negative.stdout)['residual'])/(2*epsilon)
            np.testing.assert_allclose(np.array(original['jacobian'])[:,column],numeric,atol=2e-6,rtol=2e-7)

    def test_native_rejects_reuse_retired_support_unknown_noise_and_unsupported_footprint(self):
        cases=[];args=fixture();duplicate=copy.deepcopy(args['observations'][0]);duplicate['plane_id']=1
        args['observations'].append(duplicate);args['measurement_covariance']=np.eye(22);cases.append(args)
        args=supported_fixture();args['planes'][0]['support_keys'][0]=(99,0,0);cases.append(args)
        args=fixture();args['uncertainty_source']='';cases.append(args)
        args=fixture();args['measurement_covariance']=np.zeros((21,21));cases.append(args)
        args=supported_fixture();args['observations'][0]['aperture_rad']=1.;cases.append(args)
        args=fixture();args['observations'][0]['association']='unresolved';cases.append(args)
        args=supported_fixture();args['planes'][0]['support_points']=[[0,0,-3]];cases.append(args)
        args=supported_fixture();args['states'][1]['attitude'][0,1]=1.5e-5;cases.append(args)
        for index,args in enumerate(cases):
            with self.subTest(case=index):
                with self.assertRaises(ValueError):linearize_surface_ranges(**args)
                self.assertEqual(self.probe(args).returncode,2)


if __name__=='__main__':unittest.main()

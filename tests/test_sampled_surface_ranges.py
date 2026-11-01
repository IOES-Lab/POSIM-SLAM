import copy
import unittest
import numpy as np

from posim_slam.inertial import exp_so3,left_jacobian_so3,skew
from posim_slam.range_surface_joint import linearize_surface_ranges,sampled_ray_plane_factor,normal_basis
from posim_slam.sonar_surface_power import TriangleSurface
from test_range_surface_joint import fixture,supported_fixture


def sampled_fixture():
    args=fixture()
    for row in args['observations']:
        if row['model']!='cone':continue
        axis=np.asarray(row['beam_axis'])
        row.update(model='sampled',ray_directions=np.array([exp_so3(v)@axis for v in
            [[.013,-.009,.004],[-.007,.011,-.003],[.002,.004,.005]]]))
        row.pop('aperture_rad')
    return args


def sampled_supported_fixture():
    args=supported_fixture()
    for row in args['observations']:
        if row['model']=='cone':
            row.update(model='sampled',ray_directions=np.array([[0.,0.,-1.],[.02,0.,-np.sqrt(1-.02**2)]]))
            row.pop('aperture_rad')
    return args


class SampledSurfaceTests(unittest.TestCase):
    def test_prediction_against_independent_triangle_intersections_and_plane_sign(self):
        rng=np.random.default_rng(713)
        for _ in range(12):
            position=rng.normal(0,.1,3);attitude=exp_so3(rng.normal(0,.1,3));mount=exp_so3(rng.normal(0,.1,3))
            lever=[.12,-.03,.07];normal=exp_so3(rng.normal(0,.1,3))@np.array([0.,0.,-1.])
            rays=np.column_stack((rng.uniform(-.3,.3,65),rng.uniform(-.3,.3,65),-np.ones(65)))
            rays/=np.linalg.norm(rays,axis=1)[:,None]
            distance=3.;basis=normal_basis(normal);center=distance*normal
            corners=center+np.array([[-10,-10],[10,-10],[10,10],[-10,10]])@basis.T
            scene=TriangleSurface(corners[[[0,1,2],[0,2,3]]])
            hit,_=scene.first_hits(position+attitude@lever,rays@(attitude@mount).T,15.)
            common=dict(position=position,attitude=attitude,imu_to_sensor=lever,sensor_to_imu=mount,
                ray_directions=rays,plane_normal=normal,plane_distance=distance,measured_range=3.,range_min=.1,range_max=15.)
            actual=sampled_ray_plane_factor(**common)
            np.testing.assert_allclose(actual['predicted_range'],hit.min(),atol=1e-12)
            inverse=sampled_ray_plane_factor(**dict(common,plane_normal=-normal,plane_distance=-distance))
            np.testing.assert_allclose(actual['pose_jacobian'],inverse['pose_jacobian'],atol=1e-12)

    def test_all_pose_plane_columns_at_noncommuting_accumulated_rotations(self):
        args=sampled_fixture();out=linearize_surface_ranges(**args);epsilon=1e-6
        for column in range(out['jacobian'].shape[1]):
            plus=copy.deepcopy(args);minus=copy.deepcopy(args)
            if column<18:
                stamp=column//6+1;axis=column%6;name='position' if axis<3 else 'rotation_delta';axis%=3
                plus['states'][stamp][name][axis]+=epsilon;minus['states'][stamp][name][axis]-=epsilon
            else:
                ident=(column-18)//3;axis=(column-18)%3
                if axis<2:
                    plus['planes'][ident]['normal_delta'][axis]+=epsilon;minus['planes'][ident]['normal_delta'][axis]-=epsilon
                else:
                    plus['planes'][ident]['distance']+=epsilon;minus['planes'][ident]['distance']-=epsilon
            numeric=(linearize_surface_ranges(**plus)['residual']-linearize_surface_ranges(**minus)['residual'])/(2*epsilon)
            np.testing.assert_allclose(out['jacobian'][:,column],numeric,atol=2e-6,rtol=2e-7)

    def test_all_six_global_gauge_directions_remain_null(self):
        args=sampled_fixture();out=linearize_surface_ranges(**args)
        gauge=np.zeros((out['jacobian'].shape[1],6))
        for i,stamp in enumerate(out['state_stamps_ns']):
            state=args['states'][stamp];p=state['position']
            gauge[6*i:6*i+3,:3]=np.eye(3);gauge[6*i:6*i+3,3:]=-skew(p)
            gauge[6*i+3:6*i+6,3:]=np.linalg.inv(left_jacobian_so3(state['rotation_delta']))
        for i,ident in enumerate(out['plane_ids']):
            plane=args['planes'][ident];basis=normal_basis(plane['normal_reference']);tangent=basis@plane['normal_delta']
            n=exp_so3(tangent)@plane['normal_reference'];coordinates=left_jacobian_so3(tangent)@basis
            column=6*len(args['states'])+3*i
            gauge[column:column+2,3:]=np.linalg.lstsq(-skew(n)@coordinates,-skew(n),rcond=None)[0]
            gauge[column+2,:3]=n
        np.testing.assert_allclose(out['jacobian']@gauge,0.,atol=2e-12)

    def test_all_calibrated_rays_need_support_even_when_nearest_hit_is_inside(self):
        args=sampled_supported_fixture();out=linearize_surface_ranges(**args)
        self.assertTrue(out['all_support_checked']);self.assertEqual(out['factor_indices'],[0,1])
        np.testing.assert_array_equal(out['measurement_covariance'],args['measurement_covariance'][:2,:2])
        # The selected vertical return remains inside; the alternative can hit outside.
        args['observations'][0]['ray_directions']=np.array([[0.,0.,-1.],[.3,0.,-np.sqrt(1-.3**2)]])
        with self.assertRaises(ValueError):linearize_surface_ranges(**args)
        self.assertEqual(len(linearize_surface_ranges(**dict(args,require_support=False))['residual']),2)

    def test_missing_or_blind_range_is_not_replaced_with_zero_error(self):
        args=sampled_supported_fixture();args['observations'][0]['ray_directions']=np.array([[0.,0.,1.],[0.,0.,-1.]])
        with self.assertRaisesRegex(ValueError,'incomplete_sampled_surface_support'):linearize_surface_ranges(**args)
        args['observations'][0]['ray_directions']=np.array([[0.,0.,1.]])
        with self.assertRaisesRegex(ValueError,'out_of_range'):linearize_surface_ranges(**dict(args,require_support=False))

    def test_maximum_sample_count_keeps_every_support_query(self):
        args=sampled_supported_fixture()
        rays=np.tile([0.,0.,-1.],(512,1));args['observations'][0]['ray_directions']=rays
        self.assertTrue(linearize_surface_ranges(**args)['all_support_checked'])
        rays[-1]=[.3,0.,-np.sqrt(1-.3**2)]
        with self.assertRaisesRegex(ValueError,'does_not_cover'):linearize_surface_ranges(**args)

    def test_duplicate_readings_unknown_noise_and_bad_calibration_are_rejected(self):
        args=sampled_fixture();args['observations'][0]['ray_directions'][0]*=1.01
        with self.assertRaisesRegex(ValueError,'calibrated_surface_rays'):linearize_surface_ranges(**args)
        args=sampled_fixture();args['measurement_covariance']*=0
        with self.assertRaises(ValueError):linearize_surface_ranges(**args)
        args=sampled_fixture();args['observations'][1]=copy.deepcopy(args['observations'][0])
        with self.assertRaisesRegex(ValueError,'duplicate'):linearize_surface_ranges(**args)


if __name__=='__main__':unittest.main()

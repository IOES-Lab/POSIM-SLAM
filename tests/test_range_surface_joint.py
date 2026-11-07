import copy
import math
import unittest

import numpy as np

from posim_slam.inertial import exp_so3,left_jacobian_so3,skew
from posim_slam.range_surface_joint import (linearize_surface_ranges,
    normal_basis,vertical_fan_plane_factor)
from posim_slam.ping_joint import eliminate_patch


def fixture():
    states={i+1:dict(position=np.array([.2*i,.07*i,.1+.13*i]),
        rotation_reference=exp_so3([.03,-.02,.04]),rotation_delta=np.array([.12,-.07,.08])*(i+1)/3)
        for i in range(3)}
    planes={0:dict(normal_reference=np.array([0.,0.,-1.]),normal_delta=np.array([.07,-.04]),distance=3.),
            1:dict(normal_reference=np.array([1.,0.,0.]),normal_delta=np.array([.025,.035]),distance=4.)}
    observations=[]
    for stamp in states:
        for beam in range(4):
            az=(beam+.5)*math.pi/2
            observations.append(dict(model='cone',stamp_ns=stamp,sensor_id=0,beam_id=beam,plane_id=0,
                association='surface_hypothesis',measured_range=3.5,range_min=.1,range_max=15.,
                imu_to_sensor=[.1,-.02,-.12],sensor_to_imu=exp_so3([.02,-.03,.04]),
                beam_axis=[.5*math.cos(az),.5*math.sin(az),-math.sqrt(.75)],aperture_rad=.035))
        for beam,az in enumerate((-.3,0.,.3)):
            observations.append(dict(model='fan',stamp_ns=stamp,sensor_id=1,beam_id=beam,plane_id=1,
                association='surface_hypothesis',measured_range=4.2,range_min=.1,range_max=15.,
                imu_to_sensor=[.24,0.,0.],sensor_to_imu=np.eye(3),azimuth_rad=az,vertical_fov_rad=.35))
    n=len(observations);noise=np.eye(n)*.02**2+np.ones((n,n))*.005**2
    for i,a in enumerate(observations):
        for j,b in enumerate(observations):
            if a['stamp_ns']==b['stamp_ns']:noise[i,j]+=.01**2
    return dict(states=states,planes=planes,observations=observations,
        measurement_covariance=noise,uncertainty_source='explicit synthetic correlated model',require_support=False)


def supported_fixture():
    states={1:dict(position=np.zeros(3),attitude=np.eye(3))}
    observations=[]
    common=dict(measured_range=3.,range_min=.1,range_max=15.,imu_to_sensor=[0,0,0],
                sensor_to_imu=np.eye(3),model='cone',beam_axis=[0.,0.,-1.],aperture_rad=.02,sensor_id=0)
    observations.append(dict(common,stamp_ns=1,beam_id=0,plane_id=0,association='surface_hypothesis'))
    observations.append(dict(model='fan',stamp_ns=1,sensor_id=1,beam_id=0,plane_id=0,
        association='surface_hypothesis',measured_range=3.,range_min=.1,range_max=15.,imu_to_sensor=[0,0,0],
        sensor_to_imu=exp_so3([0,math.pi/2,0]),azimuth_rad=0.,vertical_fov_rad=.35))
    keys=[]
    for i,(x,y) in enumerate(((-.5,-.5),(-.5,.5),(.5,.5),(.5,-.5)),2):
        states[i]=dict(position=np.array([x,y,0.]),attitude=np.eye(3))
        observations.append(dict(common,stamp_ns=i,beam_id=0,role='support_only'))
        keys.append((i,0,0))
    planes={0:dict(normal=np.array([0.,0.,-1.]),distance=3.,support_keys=keys,
        max_plane_deviation=.05,max_support_distance=1.)}
    n=len(observations)
    return dict(states=states,planes=planes,observations=observations,measurement_covariance=np.eye(n)*.0004+np.ones((n,n))*.0001,
        uncertainty_source='synthetic support-only sources with correlated errors')


class RangeSurfaceJointTests(unittest.TestCase):
    def test_continuous_fan_against_independent_dense_intersections(self):
        rng=np.random.default_rng(603)
        for _ in range(16):
            n=rng.normal(size=3);n/=np.linalg.norm(n)
            # Put the plane forward with an interior or boundary extremum.
            n[0]=abs(n[0])+.8;n/=np.linalg.norm(n);az=float(rng.uniform(-.5,.5))
            kw=dict(position=[0,0,0],attitude=np.eye(3),imu_to_sensor=[0,0,0],sensor_to_imu=np.eye(3),
                azimuth_rad=az,vertical_fov_rad=.6,plane_normal=n,plane_distance=2.,measured_range=3.,range_min=.1,range_max=15.)
            out=vertical_fan_plane_factor(**kw)
            elevations=np.linspace(-.3,.3,65537)
            rays=np.column_stack((np.cos(az)*np.cos(elevations),np.sin(az)*np.cos(elevations),np.sin(elevations)))
            distances=2/(rays@n);sampled=distances[distances>0].min()
            self.assertGreaterEqual(sampled+1e-12,out['predicted_range'])
            self.assertLess(sampled-out['predicted_range'],1e-9)
            inverse=vertical_fan_plane_factor(**dict(kw,plane_normal=-n,plane_distance=-2.))
            np.testing.assert_allclose(out['pose_jacobian'],inverse['pose_jacobian'],atol=1e-12)

    def test_all_joint_columns_at_accumulated_pose_and_plane_rotations(self):
        args=fixture();out=linearize_surface_ranges(**args);epsilon=1e-6
        for column in range(out['jacobian'].shape[1]):
            plus=copy.deepcopy(args);minus=copy.deepcopy(args)
            if column<18:
                stamp=column//6+1;axis=column%6
                field='position' if axis<3 else 'rotation_delta';axis%=3
                plus['states'][stamp][field][axis]+=epsilon;minus['states'][stamp][field][axis]-=epsilon
            else:
                ident=(column-18)//3;axis=(column-18)%3
                if axis<2:
                    plus['planes'][ident]['normal_delta'][axis]+=epsilon;minus['planes'][ident]['normal_delta'][axis]-=epsilon
                else:
                    plus['planes'][ident]['distance']+=epsilon;minus['planes'][ident]['distance']-=epsilon
            numeric=(linearize_surface_ranges(**plus)['residual']-linearize_surface_ranges(**minus)['residual'])/(2*epsilon)
            np.testing.assert_allclose(out['jacobian'][:,column],numeric,atol=2e-6,rtol=2e-7)

    def test_covariance_permutation_and_shared_plane_elimination(self):
        args=fixture();out=linearize_surface_ranges(**args);rng=np.random.default_rng(304)
        order=rng.permutation(len(args['observations']));permuted=copy.deepcopy(args)
        permuted['observations']=[args['observations'][i] for i in order]
        permuted['measurement_covariance']=args['measurement_covariance'][np.ix_(order,order)]
        other=linearize_surface_ranges(**permuted)
        np.testing.assert_allclose(out['jacobian'].T@out['jacobian'],other['jacobian'].T@other['jacobian'],rtol=1e-11,atol=1e-9)
        np.testing.assert_allclose(out['jacobian'].T@out['residual'],other['jacobian'].T@other['residual'],atol=1e-9)
        j=out['jacobian'];a=rng.normal(size=(18,18))*.005;prior=a@a.T+np.eye(18)*.0004
        full=j.T@j;full[:18,:18]+=np.linalg.inv(prior)
        direct=np.linalg.inv(full)[:18,:18]
        reduced=eliminate_patch(j,out['residual'],retained_columns=18)
        actual=np.linalg.inv(reduced['jacobian'].T@reduced['jacobian']+np.linalg.inv(prior))
        np.testing.assert_allclose(actual,direct,atol=1e-10,rtol=1e-8)
        fixed=np.linalg.inv(j[:,:18].T@j[:,:18]+np.linalg.inv(prior))
        self.assertGreaterEqual(np.linalg.eigvalsh(actual-fixed).min(),-1e-10)
        diagonal=linearize_surface_ranges(**dict(args,measurement_covariance=np.diag(np.diag(args['measurement_covariance']))))
        self.assertGreater(np.linalg.norm(j.T@j-diagonal['jacobian'].T@diagonal['jacobian']),1.)

    def test_nominal_support_relinearizes_and_preserves_global_gauge(self):
        args=supported_fixture();out=linearize_surface_ranges(**args)
        self.assertEqual(out['factor_indices'],[0,1]);self.assertEqual(len(out['support_only_indices']),4)
        q=exp_so3([.2,-.15,.3]);shift=np.array([2.,-1.,.7]);moved=copy.deepcopy(args)
        for state in moved['states'].values():
            state['position']=q@state['position']+shift;state['attitude']=q@state['attitude']
        for plane in moved['planes'].values():
            plane['normal']=q@plane['normal'];plane['distance']+=plane['normal']@shift
        actual=linearize_surface_ranges(**moved)
        np.testing.assert_allclose(actual['residual'],out['residual'],atol=1e-12)
        np.testing.assert_allclose(actual['conditional_hits'],out['conditional_hits']@q.T+shift,atol=1e-12)
        np.testing.assert_allclose(actual['support_seeds'][0],out['support_seeds'][0]@q.T+shift,atol=1e-12)
        self.assertFalse(actual['map_qualified']);self.assertTrue(actual['all_support_checked'])

    def test_support_only_rows_use_marginal_not_conditioned_noise(self):
        args=supported_fixture();out=linearize_surface_ranges(**args);noise=args['measurement_covariance']
        np.testing.assert_array_equal(out['measurement_covariance'],noise[:2,:2])
        conditional=noise[:2,:2]-noise[:2,2:]@np.linalg.solve(noise[2:,2:],noise[2:,:2])
        self.assertGreater(np.linalg.norm(out['measurement_covariance']-conditional),1e-5)
        self.assertEqual(len(out['source_keys']),6)

    def test_six_global_gauge_directions_remain_null_at_accumulated_coordinates(self):
        args=fixture();out=linearize_surface_ranges(**args)
        gauge=np.zeros((out['jacobian'].shape[1],6))
        for i,stamp in enumerate(out['state_stamps_ns']):
            state=args['states'][stamp];p=state['position']
            gauge[6*i:6*i+3,:3]=np.eye(3)
            gauge[6*i:6*i+3,3:]=-skew(p)
            gauge[6*i+3:6*i+6,3:]=np.linalg.inv(left_jacobian_so3(state['rotation_delta']))
        for i,ident in enumerate(out['plane_ids']):
            plane=args['planes'][ident];reference=plane['normal_reference'];basis=normal_basis(reference)
            tangent=basis@plane['normal_delta'];n=exp_so3(tangent)@reference
            coordinates=left_jacobian_so3(tangent)@basis;column=6*len(args['states'])+3*i
            gauge[column:column+2,3:]=np.linalg.lstsq(-skew(n)@coordinates,-skew(n),rcond=None)[0]
            gauge[column+2,:3]=n
        np.testing.assert_allclose(out['jacobian']@gauge,0.,atol=2e-12)

    def test_rejects_reused_measurements_frozen_support_and_unknown_noise(self):
        args=fixture();duplicate=copy.deepcopy(args['observations'][0]);duplicate['plane_id']=1
        with self.assertRaisesRegex(ValueError,'duplicate'):
            linearize_surface_ranges(**dict(args,observations=args['observations']+[duplicate],measurement_covariance=np.eye(22)))
        with self.assertRaises(ValueError):linearize_surface_ranges(**dict(args,uncertainty_source=''))
        with self.assertRaises(ValueError):linearize_surface_ranges(**dict(args,measurement_covariance=np.zeros((21,21))))
        args=supported_fixture();args['planes'][0]['support_points']=[[0,0,-3]];del args['planes'][0]['support_keys']
        with self.assertRaisesRegex(ValueError,'frozen_surface_support'):linearize_surface_ranges(**args)
        args=supported_fixture();del args['states'][2]
        with self.assertRaisesRegex(ValueError,'missing_pose'):linearize_surface_ranges(**args)
        args=fixture();args['observations'][0]['association']='unresolved'
        with self.assertRaisesRegex(ValueError,'unresolved'):linearize_surface_ranges(**args)


if __name__=='__main__':unittest.main()

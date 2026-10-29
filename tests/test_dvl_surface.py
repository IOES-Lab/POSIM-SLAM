import copy
import math
import unittest

import numpy as np

from posim_slam.dvl_surface import (cone_plane_range_factor,
    require_cone_surface_support,whiten_plane_ranges)
from posim_slam.inertial import skew


def exp(v):
    angle=np.linalg.norm(v)
    if not angle:return np.eye(3)
    k=skew(v/angle)
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*k@k


def geometry(**kwargs):
    result=dict(position=np.array([.2,-.1,.3]),attitude=exp(np.array([.12,-.16,.24])),
        imu_to_dvl=np.array([.1,.04,-.12]),dvl_to_imu=exp(np.array([-.1,.03,.12])),
        beam_axis=np.array([.3,-.4,-math.sqrt(.75)]),aperture_rad=math.radians(2),
        plane_normal=np.array([.08,-.15,-math.sqrt(1-.08**2-.15**2)]),
        plane_distance=2.5,measured_range=3.,range_min=.1,range_max=20.)
    result.update(kwargs)
    return result


class DvlSurfaceTests(unittest.TestCase):
    def test_continuous_return_matches_independent_dense_cone_rays(self):
        rng=np.random.default_rng(510)
        worst=0.
        for _ in range(24):
            axis=np.array([0.,0.,-1.]);beta=rng.uniform(0,1.1);half=rng.uniform(.01,.12)
            normal=np.array([math.sin(beta),0,-math.cos(beta)])
            rotation=exp(rng.normal(size=3))
            axis=rotation@axis;normal=rotation@normal
            kw=geometry(position=np.zeros(3),attitude=np.eye(3),imu_to_dvl=np.zeros(3),
                dvl_to_imu=np.eye(3),beam_axis=axis,plane_normal=normal,
                aperture_rad=2*half,plane_distance=float(rng.uniform(.5,5)))
            result=cone_plane_range_factor(**kw)
            u=np.cross(axis,np.eye(3)[np.argmin(abs(axis))]);u/=np.linalg.norm(u);v=np.cross(axis,u)
            # Independent ray-plane intersections over the cone, not the formula.
            phi=np.arange(8192)*2*np.pi/8192
            angles=np.linspace(0,half,9)
            rays=np.concatenate([math.cos(a)*axis+math.sin(a)*(np.cos(phi)[:,None]*u+np.sin(phi)[:,None]*v) for a in angles])
            if axis@normal>=math.cos(half):rays=np.vstack((rays,normal))
            distances=kw['plane_distance']/(rays@normal)
            sampled=float(distances[distances>0].min())
            self.assertGreaterEqual(sampled+1e-12,result['predicted_range'])
            worst=max(worst,sampled-result['predicted_range'])
            self.assertAlmostEqual(np.linalg.norm(result['conditional_hit']),result['predicted_range'])
        self.assertLess(worst,2e-7)

    def test_analytic_pose_and_plane_jacobians_both_plane_signs(self):
        epsilon=1e-6
        for sign in (1,-1):
            kw=geometry();kw['plane_normal']*=sign;kw['plane_distance']*=sign
            result=cone_plane_range_factor(**kw)
            for i in range(10):
                plus=copy.deepcopy(kw);minus=copy.deepcopy(kw)
                if i<3:
                    plus['position'][i]+=epsilon;minus['position'][i]-=epsilon
                elif i<6:
                    axis=np.eye(3)[i-3]*epsilon
                    plus['attitude']=kw['attitude']@exp(axis);minus['attitude']=kw['attitude']@exp(-axis)
                elif i<9:
                    axis=np.eye(3)[i-6]*epsilon
                    plus['plane_normal']=exp(axis)@kw['plane_normal'];minus['plane_normal']=exp(-axis)@kw['plane_normal']
                else:
                    plus['plane_distance']+=epsilon;minus['plane_distance']-=epsilon
                numerical=(cone_plane_range_factor(**plus)['predicted_range']-cone_plane_range_factor(**minus)['predicted_range'])/(2*epsilon)
                expected=np.r_[result['pose_jacobian'],result['plane_jacobian']][i]
                self.assertAlmostEqual(numerical,expected,places=7)
        original=cone_plane_range_factor(**geometry())
        np.testing.assert_allclose(original['pose_jacobian'],result['pose_jacobian'])
        np.testing.assert_allclose(original['conditional_hit'],result['conditional_hit'])

    def test_footprint_ellipse_bounds_every_sampled_cone_boundary(self):
        kw=geometry();result=cone_plane_range_factor(**kw)
        axis=kw['attitude']@kw['dvl_to_imu']@kw['beam_axis'];origin=kw['position']+kw['attitude']@kw['imu_to_dvl']
        u=np.cross(axis,[0.,1.,0.]);u/=np.linalg.norm(u);v=np.cross(axis,u)
        phi=np.linspace(0,2*np.pi,2049)
        half=kw['aperture_rad']/2
        rays=math.cos(half)*axis+math.sin(half)*(np.cos(phi)[:,None]*u+np.sin(phi)[:,None]*v)
        distances=(kw['plane_distance']-kw['plane_normal']@origin)/(rays@kw['plane_normal'])
        points=origin+distances[:,None]*rays
        local=(points-result['footprint_center'])@result['footprint_basis']
        quadratic=np.einsum('ni,ij,nj->n',local,np.linalg.inv(result['footprint_shape2']),local)
        np.testing.assert_allclose(quadratic,1,atol=1e-10)

    def test_support_covers_full_ellipse_and_rejects_unobserved_holes(self):
        result=cone_plane_range_factor(**geometry())
        center=result['footprint_center'];basis=result['footprint_basis']
        square=np.array([[-.3,-.3],[-.3,.3],[.3,.3],[.3,-.3],[0,0]])
        points=center+square@basis.T
        supported=require_cone_surface_support(result,points,max_plane_deviation=.01,max_support_distance=.1)
        self.assertTrue(supported['supported']);self.assertFalse(result['supported'])
        with self.assertRaisesRegex(ValueError,'gap'):
            require_cone_surface_support(result,points[:4],max_plane_deviation=.01,max_support_distance=.1)
        # The minimum-return point is inside this strip, but the cone is not.
        hit=(result['conditional_hit']-center)@basis
        narrow=center+(hit+np.array([[-.001,-.3],[-.001,.3],[.001,.3],[.001,-.3]]))@basis.T
        with self.assertRaisesRegex(ValueError,'outside_surface'):
            require_cone_surface_support(result,narrow,max_plane_deviation=.01,max_support_distance=.5)
        displaced=points+np.array(geometry()['plane_normal'])*.02
        with self.assertRaisesRegex(ValueError,'not_planar'):
            require_cone_surface_support(result,displaced,max_plane_deviation=.01,max_support_distance=.5)

    def test_correlated_ranges_preserve_flat_ground_null_directions(self):
        factors=[]
        for angle in (45,135,-45,-135):
            a=math.radians(angle);tilt=math.pi/6
            axis=np.array([-math.sin(tilt)*math.cos(a),math.sin(tilt)*math.sin(a),-math.cos(tilt)])
            factors.append(cone_plane_range_factor(**geometry(position=np.zeros(3),attitude=np.eye(3),
                imu_to_dvl=np.array([.1,.02,-.1]),dvl_to_imu=np.eye(3),beam_axis=axis,
                plane_normal=np.array([0.,0.,-1.]),plane_distance=3.)))
        covariance=.02**2*np.eye(4)+.05**2*np.ones((4,4))
        result=whiten_plane_ranges(factors,covariance,uncertainty_source='declared diagnostic correlated model')
        j=np.array([f['pose_jacobian'] for f in factors])
        np.testing.assert_allclose(result['pose_jacobian'].T@result['pose_jacobian'],j.T@np.linalg.solve(covariance,j),atol=1e-10)
        np.testing.assert_allclose(result['pose_jacobian'][:,[0,1,5]],0,atol=1e-13)
        self.assertEqual(np.linalg.matrix_rank(result['pose_jacobian']),3)
        self.assertFalse(result['supported']);self.assertFalse(result['map_qualified'])
        diagonal=whiten_plane_ranges(factors,np.diag(np.diag(covariance)),uncertainty_source='independence comparison')
        self.assertLess(np.linalg.norm(result['pose_jacobian'][:,2]),np.linalg.norm(diagonal['pose_jacobian'][:,2]))
        for invalid in (None,np.zeros((4,4)),np.eye(3),np.eye(4)-np.ones((4,4)),np.eye(4)+np.triu(np.ones((4,4)),1)):
            with self.assertRaises(ValueError):whiten_plane_ranges(factors,invalid,uncertainty_source='invalid control')
        with self.assertRaises(ValueError):whiten_plane_ranges(factors,covariance,uncertainty_source='')
        different=copy.deepcopy(factors);different[-1]['state_reference'][0]+=.01
        with self.assertRaisesRegex(ValueError,'state'):whiten_plane_ranges(different,covariance,uncertainty_source='wrong state')

    def test_axis_inside_cone_zero_width_and_invalid_first_returns(self):
        kw=geometry(position=np.zeros(3),attitude=np.eye(3),imu_to_dvl=np.zeros(3),dvl_to_imu=np.eye(3),
            beam_axis=np.array([0.,0.,-1.]),plane_normal=np.array([0.,0.,-1.]),plane_distance=3.)
        for aperture in (0.,.1):
            result=cone_plane_range_factor(**dict(kw,aperture_rad=aperture))
            self.assertEqual(result['predicted_range'],3.)
            np.testing.assert_allclose(result['pose_jacobian'][3:],0)
        for edit in (dict(plane_distance=.01),dict(plane_distance=21),dict(plane_distance=0),
                     dict(plane_normal=np.array([1.,0.,0.])),dict(beam_axis=np.zeros(3)),dict(aperture_rad=float('nan'))):
            with self.assertRaises(ValueError):cone_plane_range_factor(**dict(kw,**edit))


if __name__=='__main__':unittest.main()

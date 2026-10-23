"""Independent ray geometry, numerical derivatives and gauge invariance."""
import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.ping_factor import plane_range_factor
from posim_slam.ping_scale import PING_DIRECTIONS


class PingFactorTests(unittest.TestCase):
    def setUp(self):
        normal=np.array([.23,-.16,1.]);normal/=np.linalg.norm(normal)
        self.args=dict(position=np.array([.3,-.2,.1]),attitude=exp_so3([.03,.04,-.02]),
            imu_to_ping=np.array([.05,-.02,.095]),ping_to_imu=exp_so3([.02,-.01,.01]),
            rays=PING_DIRECTIONS,plane_normal=normal,plane_distance=2.,
            measured_range=2.,range_bias=.1,range_min=.2,range_max=50.)

    def test_intersections_and_nearest_return(self):
        a=self.args;f=plane_range_factor(**a)
        np.testing.assert_allclose(f['intersections']@a['plane_normal'],a['plane_distance'])
        origin=a['position']+a['attitude']@a['imu_to_ping']
        lengths=np.linalg.norm(f['intersections']-origin,axis=1)
        self.assertAlmostEqual(f['predicted_range'],min(lengths)+.1)
        self.assertAlmostEqual(f['residual'],2.-f['predicted_range'])

    def test_pose_and_plane_derivatives(self):
        a=self.args;f=plane_range_factor(**a);self.assertTrue(f['differentiable'])
        numerical=[];epsilon=1e-6
        for k in range(10):
            predictions=[]
            for sign in (-1,1):
                b=dict(a);delta=np.eye(3)[k%3]*epsilon*sign
                if k<3:b['position']=a['position']+delta
                elif k<6:b['attitude']=a['attitude']@exp_so3(delta)
                elif k<9:b['plane_normal']=exp_so3(delta)@a['plane_normal']
                else:b['plane_distance']=a['plane_distance']+epsilon*sign
                predictions.append(plane_range_factor(**b)['predicted_range'])
            numerical.append((predictions[1]-predictions[0])/(2*epsilon))
        np.testing.assert_allclose(numerical,np.r_[f['pose_jacobians'][0],f['plane_jacobians'][0]],atol=1e-8)

    def test_global_gauge_does_not_change_range(self):
        a=self.args;f=plane_range_factor(**a);q=exp_so3([.4,-.7,.2]);shift=np.array([3.,-2.,4.])
        n=q@a['plane_normal']
        b=dict(a,position=q@a['position']+shift,attitude=q@a['attitude'],
               plane_normal=n,plane_distance=a['plane_distance']+n@shift)
        self.assertAlmostEqual(plane_range_factor(**b)['predicted_range'],f['predicted_range'])
        b=dict(a,plane_normal=-a['plane_normal'],plane_distance=-a['plane_distance'])
        self.assertAlmostEqual(plane_range_factor(**b)['predicted_range'],f['predicted_range'])

    def test_tied_rays_are_not_reported_as_smooth(self):
        a=dict(self.args,rays=np.array([[.1,0,1.],[-.1,0,1.]])/np.sqrt(1.01),
               position=np.zeros(3),attitude=np.eye(3),imu_to_ping=np.zeros(3),
               ping_to_imu=np.eye(3),plane_normal=np.array([0.,0.,1.]))
        f=plane_range_factor(**a)
        self.assertFalse(f['differentiable']);self.assertEqual(len(f['pose_jacobians']),2)

    def test_rejects_invalid_ranges_and_unsupported_planes(self):
        for changes in (dict(measured_range=float('nan')),dict(measured_range=0.),
                        dict(plane_distance=-2.),dict(plane_normal=[1.,0.,0.]),
                        dict(rays=[[0.,0.,2.]]),dict(plane_distance=100.)):
            with self.assertRaises(ValueError):plane_range_factor(**dict(self.args,**changes))


if __name__=='__main__':unittest.main()

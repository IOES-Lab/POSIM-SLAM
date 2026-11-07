import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.sonar_factor import point_return_factor


class SonarFactorTests(unittest.TestCase):
    def setUp(self):
        self.args=dict(position=np.array([.3,-.2,.1]),attitude=exp_so3([.03,.04,-.02]),
            imu_to_sonar=np.array([.2,-.1,.05]),sonar_to_imu=exp_so3([.02,-.01,.1]),
            landmark=np.array([4.,1.,.4]),measured_range=4.,measured_azimuth=.2,
            range_min=.1,range_max=10.,horizontal_fov=np.deg2rad(120.),vertical_fov=np.deg2rad(40.))

    def test_calibrated_point_geometry(self):
        a=self.args;factor=point_return_factor(**a)
        q=a['sonar_to_imu'].T@(a['attitude'].T@(a['landmark']-a['position'])-a['imu_to_sonar'])
        expected=np.array([np.linalg.norm(q),np.arctan2(q[1],q[0])])
        np.testing.assert_allclose(factor['prediction'],expected)
        np.testing.assert_allclose(factor['residual'],np.array([4.,.2])-expected)

    def test_pose_and_landmark_derivatives(self):
        a=self.args;factor=point_return_factor(**a);numerical=[];epsilon=1e-6
        for k in range(9):
            values=[]
            for sign in (-1,1):
                b=dict(a);delta=np.eye(3)[k%3]*epsilon*sign
                if k<3:b['position']=a['position']+delta
                elif k<6:b['attitude']=a['attitude']@exp_so3(delta)
                else:b['landmark']=a['landmark']+delta
                values.append(point_return_factor(**b)['prediction'])
            numerical.append((values[1]-values[0])/(2*epsilon))
        np.testing.assert_allclose(np.array(numerical).T,
            np.column_stack((factor['pose_jacobian'],factor['landmark_jacobian'])),atol=1e-8)

    def test_elevation_is_unobserved_not_zero(self):
        a=dict(self.args,position=np.zeros(3),attitude=np.eye(3),imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3))
        predictions=[]
        for elevation in (-.2,0.,.2):
            point=4*np.array([np.cos(elevation)*np.cos(.2),np.cos(elevation)*np.sin(.2),np.sin(elevation)])
            f=point_return_factor(**dict(a,landmark=point));predictions.append(f['prediction'])
            tangent=4*np.array([-np.sin(elevation)*np.cos(.2),-np.sin(elevation)*np.sin(.2),np.cos(elevation)])
            np.testing.assert_allclose(f['landmark_jacobian']@tangent,0.,atol=1e-12)
            self.assertEqual(np.linalg.matrix_rank(f['landmark_jacobian']),2)
        np.testing.assert_allclose(predictions,[[4.,.2]]*3,atol=1e-12)

    def test_global_rigid_gauge(self):
        a=self.args;q=exp_so3([.4,-.3,.7]);shift=np.array([2.,-1.,3.])
        transformed=dict(a,position=q@a['position']+shift,attitude=q@a['attitude'],landmark=q@a['landmark']+shift)
        np.testing.assert_allclose(point_return_factor(**a)['prediction'],point_return_factor(**transformed)['prediction'],atol=1e-12)

    def test_rejects_invalid_or_unseen_geometry(self):
        for change in ({'measured_range':float('nan')},{'measured_range':float('inf')},{'measured_range':0.},
                       {'measured_azimuth':2.},{'vertical_fov':0.},
                       {'landmark':[-3.,0.,0.]},{'landmark':[4.,0.,5.]},
                       {'landmark':[1.,8.,0.]},{'landmark':self.args['position']}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                point_return_factor(**dict(self.args,**change))

    def test_range_limits_constrain_physical_point_instead_of_noisy_sample(self):
        for minimum,actual,measured in ((.2,9.999,10.008360862731934),(.51,.510001,.5016722679138184)):
            args=dict(position=np.zeros(3),attitude=np.eye(3),imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),
                landmark=[actual,0,0],measured_range=measured,measured_azimuth=0.,range_min=minimum,
                range_max=10.,horizontal_fov=2.1,vertical_fov=.35)
            result=point_return_factor(**args)
            np.testing.assert_allclose(result['residual'],[measured-actual,0.],atol=1e-14)
            for unsupported in (minimum-.001,10.001):
                with self.assertRaisesRegex(ValueError,'unsupported_sonar_point'):
                    point_return_factor(**dict(args,landmark=[unsupported,0,0]))


if __name__=='__main__':unittest.main()

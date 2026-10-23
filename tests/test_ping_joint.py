"""Joint derivatives, correlated elimination and unobservable-plane behavior."""
import unittest
import numpy as np
from posim_slam.inertial import exp_so3
from posim_slam.ping_scale import PING_DIRECTIONS
from posim_slam.ping_joint import linearize_ping_patch,eliminate_patch


class JointPingTests(unittest.TestCase):
    def setUp(self):
        n=np.array([.1,.2,1.]);n/=np.linalg.norm(n)
        xy=np.array([(x,y) for x in np.linspace(-.6,.6,8) for y in np.linspace(-.6,.6,8)])
        self.points=np.column_stack([xy,(2.-xy@n[:2])/n[2]])
        self.args=dict(position=np.zeros(3),attitude=np.eye(3),imu_to_ping=np.array([.02,.03,.1]),
            ping_to_imu=np.eye(3),rays=PING_DIRECTIONS,plane_normal=n,plane_distance=2.,
            measured_range=2.,range_bias=.1,range_min=.2,range_max=50.,
            range_sigma=.02,surface_sigma=.01,max_support_distance=.15)

    def test_joint_jacobian_matches_finite_difference(self):
        block=linearize_ping_patch(landmarks=self.points,**self.args)
        numerical=np.empty_like(block['jacobian']);epsilon=1e-6
        for k in range(numerical.shape[1]):
            values=[]
            for sign in (-1,1):
                a=dict(self.args);points=self.points.copy();d=epsilon*sign
                if k<3:a['position']=a['position']+np.eye(3)[k]*d
                elif k<6:a['attitude']=a['attitude']@exp_so3(np.eye(3)[k-3]*d)
                elif k<8:a['plane_normal']=exp_so3(block['normal_basis'][:,k-6]*d)@a['plane_normal']
                elif k==8:a['plane_distance']+=d
                else:points[(k-9)//3,(k-9)%3]+=d
                values.append(linearize_ping_patch(landmarks=points,**a)['residual'])
            numerical[:,k]=(values[1]-values[0])/(2*epsilon)
        np.testing.assert_allclose(block['jacobian'],numerical,atol=1e-7)

    def test_reduced_solution_and_covariance_equal_full_joint_system(self):
        block=linearize_ping_patch(landmarks=self.points,**self.args)
        size=block['jacobian'].shape[1]
        # Independent synthetic linear pose/landmark observations. The plane
        # has no prior; all its information comes through shared landmarks.
        indices=list(range(6))+list(range(9,size))
        prior=np.eye(size)[indices]*5.
        j=np.vstack([prior,block['jacobian']]);r=np.r_[np.zeros(len(indices)),block['residual']]
        reduced=eliminate_patch(j,r,retained_columns=6)
        delta=np.linalg.lstsq(reduced['jacobian'],-reduced['residual'],rcond=None)[0]
        nuisance=reduced['backsub_offset']+reduced['backsub_matrix']@delta
        expected=np.linalg.lstsq(j,-r,rcond=None)[0]
        np.testing.assert_allclose(np.r_[delta,nuisance],expected,atol=1e-9)
        full_cov=np.linalg.inv(j.T@j)[:6,:6]
        np.testing.assert_allclose(np.linalg.inv(reduced['jacobian'].T@reduced['jacobian']),full_cov,atol=1e-9)
        fixed_cov=np.linalg.inv(j[:,:6].T@j[:,:6])
        self.assertGreater(np.trace(full_cov),np.trace(fixed_cov)+1e-4)

    def test_unobserved_plane_does_not_invent_pose_information(self):
        block=linearize_ping_patch(landmarks=self.points,**self.args)
        reduced=eliminate_patch(block['jacobian'],block['residual'],retained_columns=6)
        self.assertEqual(reduced['jacobian'].shape,(0,6))
        self.assertGreater(reduced['nuisance_nullity'],0)

    def test_invalid_noise_and_system_rejected(self):
        for field in ('range_sigma','surface_sigma'):
            for value in (0.,-1.,float('nan')):
                with self.assertRaises(ValueError):
                    linearize_ping_patch(landmarks=self.points,**dict(self.args,**{field:value}))
        for retained in (0,3,True):
            with self.assertRaises(ValueError):eliminate_patch(np.eye(3),np.zeros(3),retained_columns=retained)


if __name__=='__main__':unittest.main()

import unittest
import numpy as np
from posim_slam.sonar_joint import linearize_track
from posim_slam.ping_joint import eliminate_patch
from posim_slam.inertial import exp_so3


class SonarJointTests(unittest.TestCase):
    def setUp(self):
        self.args=dict(observations=[dict(stamp_ns=i+1,range_m=4.,azimuth_rad=.1) for i in range(3)],
            states={i+1:dict(position=np.array([i*.2,i*.05,-i*.1]),attitude=exp_so3([.01*i,.02*i,0.])) for i in range(3)},
            landmark=np.array([4.,.7,.4]),measurement_covariance=np.diag([.04,.0004]*3),
            imu_to_sonar=np.array([.1,0.,.05]),sonar_to_imu=exp_so3([.01,.02,.03]),
            range_min=.1,range_max=10.,horizontal_fov=2.,vertical_fov=.8)

    def test_correlated_whitening_matches_full_quadratic(self):
        a=self.args;noise=a['measurement_covariance'].copy()
        noise[0,2]=noise[2,0]=.01;noise[1,5]=noise[5,1]=.0001
        plain=linearize_track(**dict(a,measurement_covariance=np.eye(6)))
        white=linearize_track(**dict(a,measurement_covariance=noise))
        np.testing.assert_allclose(white['jacobian'].T@white['jacobian'],plain['jacobian'].T@np.linalg.solve(noise,plain['jacobian']),atol=1e-10)
        self.assertAlmostEqual(white['residual']@white['residual'],plain['residual']@np.linalg.solve(noise,plain['residual']))

    def test_pose_and_shared_point_derivatives(self):
        a=self.args;base=linearize_track(**a);derivatives=[];epsilon=1e-6
        for k in range(21):
            results=[]
            for sign in (-1,1):
                states={t:dict(s) for t,s in a['states'].items()};b=dict(a,states=states)
                delta=np.eye(3)[k%3]*epsilon*sign
                if k>=18:b['landmark']=a['landmark']+delta
                else:
                    stamp=k//6+1
                    if k%6<3:states[stamp]['position']=states[stamp]['position']+delta
                    else:states[stamp]['attitude']=states[stamp]['attitude']@exp_so3(delta)
                results.append(linearize_track(**b)['residual'])
            derivatives.append((results[1]-results[0])/(2*epsilon))
        np.testing.assert_allclose(np.array(derivatives).T,base['jacobian'],atol=1e-7)

    def test_elimination_retains_correlations_and_no_absolute_pose_prior(self):
        f=linearize_track(**self.args);n=f['retained_columns'];j=f['jacobian'];r=f['residual']
        reduced=eliminate_patch(j,r,retained_columns=n)
        delta=np.linspace(-.02,.03,n)
        point_step=np.linalg.lstsq(j[:,n:],-(r+j[:,:n]@delta),rcond=None)[0]
        expected=r+j[:,:n]@delta+j[:,n:]@point_step
        actual=reduced['residual']+reduced['jacobian']@delta
        self.assertAlmostEqual(expected@expected,actual@actual,places=9)
        for axis in range(3):
            translation=np.zeros(n);translation[axis::6]=1
            np.testing.assert_allclose(reduced['jacobian']@translation,0.,atol=1e-11)
            omega=np.eye(3)[axis];turn=np.zeros(n)
            for i,state in enumerate(self.args['states'].values()):
                turn[6*i:6*i+3]=np.cross(omega,state['position'])
                turn[6*i+3:6*i+6]=state['attitude'].T@omega
            np.testing.assert_allclose(reduced['jacobian']@turn,0.,atol=1e-10)
        # A single range/azimuth observation with a free point gives no pose prior.
        a=dict(self.args,observations=self.args['observations'][:1],measurement_covariance=np.eye(2))
        f=linearize_track(**a);one=eliminate_patch(f['jacobian'],f['residual'],retained_columns=6)
        self.assertEqual(one['jacobian'].shape,(0,6));self.assertEqual(one['nuisance_nullity'],1)

    def test_missing_states_duplicate_times_and_unknown_noise_rejected(self):
        a=self.args
        for change in (dict(states={}),dict(observations=[a['observations'][0]]*3),
                       dict(measurement_covariance=np.zeros((6,6))),dict(observations=[])):
            with self.subTest(change=change),self.assertRaises(ValueError):linearize_track(**dict(a,**change))


if __name__=='__main__':unittest.main()

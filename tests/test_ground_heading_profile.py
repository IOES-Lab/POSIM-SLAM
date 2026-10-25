import unittest
import numpy as np
from posim_slam.ground_heading_profile import heading_profile


def fixture(slope=(0.,0.), correction=0.):
    az=np.linspace(-.8,.8,12);el=-.35
    rays=np.column_stack((np.cos(az)*np.cos(el),np.sin(az)*np.cos(el),np.full(len(az),np.sin(el))))
    normal=np.r_[slope,1.];normal/=np.linalg.norm(normal);distance=-2.
    observations=[];states={}
    for i in range(4):
        stamp=1_000_000_000+i*500_000_000;t=.5*i
        position=np.array([.2*t,.03*t,0.]);angle=-correction*t
        c,s=np.cos(angle),np.sin(angle);attitude=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        ranges=(distance-normal@position)/(rays@normal)
        observations.append(dict(stamp_ns=stamp,azimuth_rad=az,range_m=ranges,boundary_sign=-1))
        states[stamp]=dict(position=position,attitude=attitude)
    return dict(observations=observations,states=states,plane_normal=normal,plane_distance=distance,
                measurement_covariance=np.eye(48)*.03**2,imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),
                vertical_fov=.7,range_min=.2,range_max=10.,rates_rad_s=np.deg2rad(np.linspace(-10,10,41)))


class GroundHeadingProfileTests(unittest.TestCase):
    def test_flat_plane_has_no_relative_yaw_information(self):
        result=heading_profile(**fixture())
        self.assertTrue(all(r['valid'] and r['converged'] for r in result['profile']))
        self.assertLess(max(r['cost'] for r in result['profile']),1e-20)

    def test_distinct_slope_recovers_injected_relative_turn_without_pose_prior(self):
        a=fixture((.12,-.07),np.deg2rad(3.));result=heading_profile(**a)
        best=min(result['profile'],key=lambda r:r.get('cost',float('inf')))
        self.assertTrue(best['converged']);self.assertAlmostEqual(best['rate_rad_s'],np.deg2rad(3.),places=10)
        self.assertLess(best['cost'],1e-16)
        self.assertGreater(result['profile'][20]['cost'],.5)

    def test_global_yaw_and_translation_leave_profile_unchanged(self):
        a=fixture((.08,-.1),np.deg2rad(-2.));reference=heading_profile(**a)
        angle=.6;c,s=np.cos(angle),np.sin(angle);rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        translation=np.array([3.,-2.,.7]);a['plane_normal']=rotation@a['plane_normal'];a['plane_distance']+=a['plane_normal']@translation
        for state in a['states'].values():
            state['position']=rotation@state['position']+translation;state['attitude']=rotation@state['attitude']
        changed=heading_profile(**a)
        np.testing.assert_allclose([r['cost'] for r in reference['profile']],[r['cost'] for r in changed['profile']],atol=2e-9)

    def test_correlated_noise_and_invalid_geometry(self):
        a=fixture((.12,.04));a['measurement_covariance']+=.02**2
        result=heading_profile(**a);self.assertLess(result['profile'][20]['cost'],1e-16)
        for change in ({'rates_rad_s':[0.,0.]},{'measurement_covariance':np.zeros((48,48))},
                       {'plane_normal':[0.,1.,0.]},{'range_max':float('inf')},{'observations':a['observations'][:1]}):
            with self.assertRaises((ValueError,np.linalg.LinAlgError)):heading_profile(**dict(a,**change))


if __name__=='__main__':unittest.main()

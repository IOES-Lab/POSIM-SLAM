import unittest
import numpy as np
from posim_slam.sonar_correspondence import project_return_arc,rank_return_patches


class SonarCorrespondenceTests(unittest.TestCase):
    def arguments(self):
        return dict(measured_range=5., measured_azimuth=.2,
            previous_position=np.zeros(3), previous_attitude=np.eye(3),
            current_position=np.zeros(3), current_attitude=np.eye(3),
            imu_to_sonar=np.zeros(3), sonar_to_imu=np.eye(3),
            range_min=.2, range_max=10., horizontal_fov=2.1, vertical_fov=.35)

    def test_stationary_observation_keeps_elevation_ambiguity(self):
        result=project_return_arc(**self.arguments())
        np.testing.assert_allclose(result['range_azimuth'],np.tile([5.,.2],(65,1)),atol=1e-14)
        self.assertTrue(result['supported'].all())
        self.assertFalse(result['elevation_resolved'])
        self.assertLess(result['source_elevation_rad'][0],0)
        self.assertGreater(result['source_elevation_rad'][-1],0)

    def test_vertical_baseline_separates_two_unmeasured_branches(self):
        args=self.arguments(); args.update(measured_azimuth=0.,current_position=np.array([0.,0.,.2]))
        result=project_return_arc(**args)
        elevations=result['source_elevation_rad']
        expected=np.sqrt(25.+.04-2*5*.2*np.sin(elevations))
        np.testing.assert_allclose(result['range_azimuth'][:,0],expected,atol=1e-14)
        self.assertGreater(result['range_azimuth'][0,0],result['range_azimuth'][-1,0])
        self.assertFalse(result['supported'][0])
        self.assertTrue(result['supported'][-1])

    def test_rotating_lever_and_global_frame_invariance(self):
        args=self.arguments(); args.update(measured_range=4.,measured_azimuth=0.,
            imu_to_sonar=np.array([1.,0.,0.]),horizontal_fov=np.pi)
        rz=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        args['current_attitude']=rz
        result=project_return_arc(**args)
        # Source central point is (5,0,0); target sensor origin is (0,1,0).
        np.testing.assert_allclose(result['range_azimuth'][32],[np.sqrt(26),np.arctan2(-5,-1)])
        self.assertFalse(result['supported'][32])
        offset=np.array([2.,3.,4.])
        for prefix in ('previous','current'):
            args[prefix+'_position']=rz@args[prefix+'_position']+offset
            args[prefix+'_attitude']=rz@args[prefix+'_attitude']
        shifted=project_return_arc(**args)
        np.testing.assert_allclose(shifted['range_azimuth'],result['range_azimuth'],atol=1e-14)
        np.testing.assert_array_equal(shifted['supported'],result['supported'])

    def test_invalid_geometry_and_unbounded_sampling_rejected(self):
        for changes in (dict(samples=1000),dict(samples=4),dict(samples=True),
                        dict(measured_range=float('nan')),dict(measured_azimuth=2.),
                        dict(sonar_to_imu=np.zeros((3,3)))):
            with self.assertRaises(ValueError):project_return_arc(**dict(self.arguments(),**changes))

    def test_patch_translation_survives_gain_and_offset(self):
        rng=np.random.default_rng(7);a=rng.normal(size=(40,50));b=rng.normal(size=a.shape)
        b[20:27,30:37]=a[10:17,10:17]*3+12
        result=rank_return_patches(a,b,[13,13],[[32,22],[20,15]],search_radius=2)
        self.assertEqual(result['modes'][0]['pixel'],[33,23])
        self.assertAlmostEqual(result['modes'][0]['correlation'],1.)

    def test_repeated_patches_preserve_competing_modes(self):
        rng=np.random.default_rng(9);a=rng.normal(size=(50,50));b=np.zeros_like(a)
        for y,x in ((10,10),(30,30)):b[y-3:y+4,x-3:x+4]=a[17:24,17:24]
        result=rank_return_patches(a,b,[20,20],[[10,10],[30,30]],search_radius=0)
        self.assertEqual(len(result['modes']),2)
        self.assertTrue(all(abs(m['correlation']-1)<1e-12 for m in result['modes']))

    def test_uninformative_border_and_excessive_search(self):
        a=np.ones((20,20))
        self.assertEqual(rank_return_patches(a,a,[10,10],[[10,10]])['modes'],[])
        self.assertEqual(rank_return_patches(a,a,[0,0],[[10,10]])['modes'],[])
        with self.assertRaises(ValueError):rank_return_patches(a,a,[10,10],np.zeros((258,2)))


if __name__=='__main__':unittest.main()

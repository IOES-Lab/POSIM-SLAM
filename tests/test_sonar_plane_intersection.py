import math
import unittest
import numpy as np

from posim_slam.sonar_plane_intersection import range_azimuth_plane


def solve(**changes):
    arguments=dict(ranges=[3.],azimuths=[0.],origin=np.zeros(3),
        sonar_to_map=np.eye(3),plane_normal=[0.,0.,1.],
        plane_distance=-.3,vertical_fov_rad=math.radians(20))
    arguments.update(changes)
    return range_azimuth_plane(**arguments)


class SonarPlaneIntersectionTests(unittest.TestCase):
    def test_forward_geometry_and_rigid_transform(self):
        rng=np.random.default_rng(7)
        angles=rng.uniform(-1,1,100);elevations=rng.uniform(-.15,.15,100)
        ranges=rng.uniform(2,8,100)
        rays=np.column_stack((np.cos(angles)*np.cos(elevations),
            np.sin(angles)*np.cos(elevations),np.sin(elevations)))
        theta=.7
        rotation=np.array([[math.cos(theta),0,math.sin(theta)],
                           [0,1,0],[-math.sin(theta),0,math.cos(theta)]])
        origin=np.array([1.,-2.,.4]);normal=rotation@np.array([0.,0.,1.])
        for radius,azimuth,ray in zip(ranges,angles,rays):
            expected=origin+radius*(rotation@ray)
            result=solve(ranges=[radius],azimuths=[azimuth],origin=origin,
                sonar_to_map=rotation,plane_normal=normal,plane_distance=float(normal@expected))
            self.assertEqual(result['count'][0],1)
            np.testing.assert_allclose(result['points'][0,0],expected,atol=1e-12)

    def test_two_branches_are_preserved_and_plane_sign_invariant(self):
        # A vertical wall and a range slightly longer than its perpendicular
        # distance permit either a positive or negative elevation.
        result=solve(plane_normal=[1.,0.,0.],plane_distance=3*math.cos(.1))
        self.assertEqual(result['count'][0],2)
        np.testing.assert_allclose(result['elevations'][0],[-.1,.1],atol=1e-12)
        inverse=solve(plane_normal=[-1.,0.,0.],plane_distance=-3*math.cos(.1))
        np.testing.assert_allclose(result['points'],inverse['points'],atol=1e-12)

    def test_no_intersection_outside_fan_and_unbounded_ambiguity(self):
        for changes in (dict(plane_distance=-4.),dict(plane_distance=-1.)):
            result=solve(**changes)
            self.assertEqual(result['count'][0],0)
            self.assertTrue(np.isnan(result['points']).all())
        result=solve(plane_normal=[0.,1.,0.],plane_distance=0.)
        self.assertEqual(result['count'][0],-1)
        self.assertTrue(np.isnan(result['points']).all())

    def test_tangent_has_one_branch_but_zero_elevation_information(self):
        result=solve(plane_normal=[1.,0.,0.],plane_distance=3.)
        self.assertEqual(result['count'][0],1)
        self.assertAlmostEqual(result['plane_elevation_derivative'][0,0],0.)

    def test_invalid_geometry_is_rejected(self):
        for changes in (dict(ranges=[0.]),dict(ranges=[float('nan')]),
                dict(azimuths=[math.pi/2]),dict(azimuths=[0.,.1]),
                dict(vertical_fov_rad=math.pi),dict(plane_normal=[0.,0.,2.]),
                dict(sonar_to_map=np.ones((3,3))),dict(plane_distance=float('inf'))):
            with self.assertRaises(ValueError):solve(**changes)


if __name__=='__main__':unittest.main()

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from fit_sonar_surface_fixture import fit_fixture, surface_from_parameters
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface


class SurfaceFixtureOwnershipTests(unittest.TestCase):
    def packet(self):
        packet = dict(positions=np.array([[0., 0., z] for z in (-.1, .1, -.2, .2)]),
                      rotations=np.tile(np.eye(3), (4, 1, 1)),
                      azimuth=np.linspace(-.2, .2, 7), elevation=np.linspace(-.1, .1, 5),
                      native_ranges=np.arange(80) * .1, train=np.array([0, 1]), test=np.array([2, 3]),
                      known_yz_support=np.array([[-.7, -.5], [.7, -.5], [.7, .5], [-.7, .5]]),
                      known_background=np.array([[[5., -2., -2.], [5., 2., -2.], [5., 2., 2.]],
                                                 [[5., -2., -2.], [5., 2., 2.], [5., -2., 2.]]]),
                      yz_origin=np.zeros(2), initial_parameters=np.array([3.4, 0, 0, -7., -7.]),
                      parameter_bounds=np.array([[2.5, 4.], [-.75, .75], [-.75, .75], [-12., -3.], [-12., -3.]]),
                      maximum_range_m=np.array(7.9))
        model = SurfacePowerModel(packet['azimuth'], packet['elevation'], packet['native_ranges'],
                                  maximum_range_m=7.9, range_group=4, beam_group=3)
        surface = TriangleSurface(surface_from_parameters(packet, [3.15, -.2, -.1, -7., -7.]))
        packet['observed'] = np.array([model.observation(surface, p, r).forward(np.full(4, .001))
                                       for p, r in zip(packet['positions'], packet['rotations'])])
        return packet

    def test_heldout_power_cannot_change_the_fitted_geometry(self):
        packet = self.packet()
        first, _, _ = fit_fixture(packet, max_iterations=1)
        packet['observed'][packet['test']] *= 13
        second, _, _ = fit_fixture(packet, max_iterations=1)
        np.testing.assert_array_equal(first['estimated_parameters'], second['estimated_parameters'])
        self.assertNotEqual(first['heldout_relative_power_l2'], second['heldout_relative_power_l2'])

    def test_overlapping_split_and_undeclared_reference_geometry_are_rejected(self):
        packet = self.packet(); packet['test'] = np.array([0, 1])
        with self.assertRaises(ValueError): fit_fixture(packet)
        packet = self.packet(); packet['reference_plane'] = np.array([3.15, -.2, -.1])
        with self.assertRaises(ValueError): fit_fixture(packet)


if __name__ == '__main__':
    unittest.main()

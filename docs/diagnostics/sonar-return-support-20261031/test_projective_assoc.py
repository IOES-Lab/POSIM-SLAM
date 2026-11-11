import unittest
from pathlib import Path
import json
import tempfile
import numpy as np
from projective_assoc import arc_pixel_distance, unique_matches, project_pixels, ProjectiveTracks
from replay_prepared import replay


class AssociationTests(unittest.TestCase):
    def test_competing_landmarks_and_duplicate_detections_are_all_unresolved(self):
        pairs, new = unique_matches(np.array([[.1,.2],[8.,8.],[.3,.4]]))
        self.assertEqual(pairs, [])
        self.assertEqual(new, [1])
        self.assertEqual(unique_matches(np.array([[.1],[.2]])), ([], []))

    def test_assignment_is_invariant_to_candidate_order_and_has_an_uncertain_gap(self):
        a = np.array([[.3,8.],[7.,.2],[3.,4.],[9.,9.]])
        self.assertEqual(unique_matches(a), ([(0,0),(1,1)], [3]))
        pairs, novel = unique_matches(a[[3,1,2,0]])
        self.assertEqual(pairs, [(1,1),(3,0)])
        self.assertEqual(novel, [0])

    def test_curve_support_gaps_do_not_create_false_continuous_matches(self):
        locus = np.array([[0.,0.],[1.,0.],[9.,0.],[10.,0.]])
        np.testing.assert_allclose(arc_pixel_distance([[5.,0.]],locus,[True,False,False,True]),[5.])
        np.testing.assert_allclose(arc_pixel_distance([[5.,1.]],locus,[True]*4),[1.])
        self.assertTrue(np.isinf(arc_pixel_distance([[5.,0.]],locus,[False]*4)).all())

    def test_pure_yaw_keeps_unknown_elevation_without_a_zero_height_prior(self):
        rz = np.array([[np.cos(.1),-np.sin(.1),0],[np.sin(.1),np.cos(.1),0],[0,0,1.]])
        ranges = np.linspace(0,10,401)
        angles = np.linspace(-1.,1.,201)
        geometry = dict(imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),range_min=.2,
                        range_max=10.,horizontal_fov=2.,vertical_fov=.35)
        before = dict(position=np.zeros(3),attitude=np.eye(3))
        after = dict(position=np.zeros(3),attitude=rz)
        pixels, supported = project_pixels(dict(range_m=5.,azimuth_rad=.2),before,after,geometry,ranges,angles,257)
        self.assertTrue(supported.all())
        np.testing.assert_allclose(pixels,np.tile([110.,200.],(257,1)),atol=1e-10)

    def test_projection_outside_image_is_unavailable_instead_of_clamped(self):
        ranges = np.linspace(0,4,161)
        angles = np.linspace(-.4,.4,81)
        geometry = dict(imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),range_min=.2,
                        range_max=10.,horizontal_fov=2.,vertical_fov=.35)
        pose = dict(position=np.zeros(3),attitude=np.eye(3))
        pixels, supported = project_pixels(dict(range_m=5.,azimuth_rad=.2),pose,pose,geometry,ranges,angles,65)
        self.assertFalse(supported.any())
        self.assertTrue(np.isnan(pixels[:,1]).all())

    def test_feature_gap_retains_identity_but_expired_features_are_new(self):
        geometry = dict(imu_to_sonar=np.zeros(3),sonar_to_imu=np.eye(3),range_min=.2,
                        range_max=10.,horizontal_fov=2.,vertical_fov=.35)
        tracks = ProjectiveTracks(geometry)
        pose = dict(position=np.zeros(3),attitude=np.eye(3))
        ranges, angles = np.linspace(0,10,401), np.linspace(-1,1,201)
        obs = [dict(range_m=5.,azimuth_rad=.2)]
        self.assertEqual(tracks.push(0,pose,[[120.,200.]],obs,ranges,angles)['new'],1)
        tracks.push(500_000_000,pose,[],[],ranges,angles)
        returned = tracks.push(1_000_000_000,pose,[[120.,200.]],obs,ranges,angles)
        self.assertEqual(returned['observations'][0]['track_id'],0)
        expired = tracks.push(4_000_000_000,pose,[[120.,200.]],obs,ranges,angles)
        self.assertEqual(expired['observations'][0]['track_id'],1)
        with self.assertRaises(ValueError):tracks.push(4_000_000_000,pose,[],[],ranges,angles)

    def test_invalid_search_is_rejected(self):
        for value in (np.array([[np.nan]]),np.array([[-1.]]),np.zeros((301,1))):
            with self.assertRaises(ValueError):unique_matches(value)
        with self.assertRaises(ValueError):unique_matches(np.zeros((1,1)),5,2)

    def test_original_metric_values_survive_a_pixel_roundtrip_difference(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent=Path(temporary)
            reference=parent/'sonar-inspection-scene-20261031'
            reference.mkdir()
            geometry=dict(imu_to_sonar=[0,0,0],sonar_to_imu=np.eye(3).tolist(),range_min=.2,
                          range_max=10.,horizontal_fov=2.,vertical_fov=.35)
            (reference/'calibration.json').write_text(json.dumps(dict(sonar=geometry)))
            root=parent/'prepared';root.mkdir()
            original=np.nextafter(3.,np.inf)
            arrays=dict(stamps_ns=[1],offsets=[0,1],pixels=[[2.,3.]],
                descriptors=np.zeros((1,61),dtype=np.uint8),ranges=np.arange(5.),
                angles=np.linspace(-1,1,5),range_azimuth=[[original,0.]],
                nav_stamps_ns=[1],nav_positions=np.zeros((1,3)),nav_attitudes=np.eye(3)[None,:,:])
            np.savez(root/'detected-features.npz',**arrays)
            for mode in ('descriptor','projective'):
                _,tracks=replay(root,mode)
                self.assertEqual(tracks[0]['observations'][0]['range_m'],original)
                self.assertNotEqual(tracks[0]['observations'][0]['range_m'],3.)
            arrays['range_azimuth']=[[3.01,0.]]
            np.savez(root/'detected-features.npz',**arrays)
            with self.assertRaisesRegex(ValueError,'prepared_metric_observations_changed'):
                replay(root)


if __name__ == '__main__':
    unittest.main()

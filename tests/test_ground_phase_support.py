import unittest
import numpy as np
from test_sonar_ground import scene
from posim_slam.sonar_ground import ground_entries, ground_entry_curve, fit_ground_entry
from posim_slam.sonar_phase_bounds import BeamPhaseBounds


class GroundPhaseSupportTests(unittest.TestCase):
    def test_identity_response_preserves_original_samples_and_plane(self):
        frame, up, _ = scene(occlusion=True)
        model = BeamPhaseBounds(np.eye(256))
        original = fit_ground_entry(frame, vertical_fov_rad=.35, up_sonar=up)
        screened = fit_ground_entry(frame, vertical_fov_rad=.35, up_sonar=up,
                                    source_bounds=model, amplitude_error=(.1, 0.))
        self.assertTrue(screened['accepted'])
        np.testing.assert_array_equal(screened['inlier_beams'], original['inlier_beams'])
        np.testing.assert_array_equal(screened['points_sonar'], original['points_sonar'])
        np.testing.assert_array_equal(screened['normal_sonar'], original['normal_sonar'])
        self.assertEqual(screened['entries']['source_bound_excluded'], 0)

    def test_unresolved_source_censors_without_selecting_later_return(self):
        frame, up, _ = scene()
        original = ground_entries(frame)
        screened = ground_entry_curve(frame, vertical_fov_rad=.35, up_sonar=up,
            source_bounds=BeamPhaseBounds(np.eye(256)), amplitude_error=(0., 1e8))
        self.assertFalse(screened['accepted'])
        e = screened['entries']; ids = e['selected_beams']
        np.testing.assert_array_equal(ids, original['selected_beams'])
        self.assertEqual(e['source_bound_tested'], int(np.isfinite(original['range_m'][ids]).sum()))
        self.assertEqual(e['source_bound_excluded'], e['source_bound_tested'])
        self.assertFalse(np.isfinite(e['range_m'][ids]).any())

    def test_response_error_and_dimensions_are_explicit_even_without_echo(self):
        frame, up, _ = scene(visible=False)
        for kwargs in (dict(amplitude_error=(0., 0.)),
                       dict(source_bounds=BeamPhaseBounds(np.eye(256))),
                       dict(source_bounds=BeamPhaseBounds(np.eye(2)), amplitude_error=(0., 0.)),
                       dict(source_bounds=BeamPhaseBounds(np.eye(256)), amplitude_error=(1., 0.))):
            with self.subTest(kwargs=tuple(kwargs)), self.assertRaises(ValueError):
                fit_ground_entry(frame, vertical_fov_rad=.35, up_sonar=up, **kwargs)

    def test_same_gain_and_scaled_absolute_error_preserve_selection(self):
        frame, _, _ = scene()
        shifted, _, _ = scene(gain=20.)
        response = np.eye(256) + .2*np.roll(np.eye(256), 1, axis=1)
        model = BeamPhaseBounds(response)
        a = ground_entries(frame, source_bounds=model, amplitude_error=(.1, .2))
        b = ground_entries(shifted, source_bounds=model, amplitude_error=(.1, 2.))
        np.testing.assert_array_equal(a['range_m'], b['range_m'])


if __name__ == '__main__':
    unittest.main()

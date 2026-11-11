"""Zero power is valid input; it must not become a measured landmark or noise."""
import unittest
import numpy as np
from posim_slam.sonar_input import validate_frame,log_image_values,log_quantile
from posim_slam.acoustic_flow import FILTERS,image,corner_pixels,correspondences,return_support
from posim_slam.acoustic_backend import cv,NativeOpenCV
from posim_slam.acoustic_tracks import AcousticTracks
from posim_slam.sonar_edges import echo_boundaries,echo_tracking_image
from posim_slam.sonar_ground import ground_entries,fit_ground_entry
from posim_slam.sonar_surface import segment_surface
from test_sonar_edges import frame


class NoReturnTests(unittest.TestCase):
    variants=('raw','echo_boundary')

    def test_absence_is_retained_but_malformed_values_are_rejected(self):
        value=frame();value['intensity_db'][100:110,10:20]=-np.inf
        original=value['intensity_db'].copy();validate_frame(value)
        for variant in self.variants:
            with self.subTest(variant=variant),np.errstate(all='raise'):
                result=image(value,variant)
                self.assertEqual(result.dtype,np.uint8)
                self.assertFalse(result[100:110,10:20].any())
                pixels=corner_pixels(value,variant)
                self.assertTrue(return_support(value,pixels).all())
        np.testing.assert_array_equal(value['intensity_db'],original)
        for invalid in (np.nan,np.inf):
            value['intensity_db'][0,0]=invalid
            with self.assertRaises(ValueError):validate_frame(value)
            with self.assertRaises(ValueError):log_image_values(value['intensity_db'])

    def test_empty_power_has_no_image_features_ground_or_shadow_claim(self):
        value=frame();value['intensity_db'][:]=-np.inf
        validate_frame(value)
        with np.errstate(all='raise'):
            for variant in self.variants:
                self.assertFalse(image(value,variant).any())
                self.assertEqual(len(corner_pixels(value,variant)),0)
            edge=echo_boundaries(value)
            ground=ground_entries(value)
            surface=segment_surface(value,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
            fit=fit_ground_entry(value,vertical_fov_rad=.35,up_sonar=[0,0,1])
        self.assertIsNone(edge['noise_db']);self.assertFalse(edge['noise_available'])
        self.assertFalse(edge['edge_mask'].any());self.assertFalse(edge['tracking_mask'].any())
        self.assertIsNone(ground['noise_db']);self.assertFalse(ground['noise_available'])
        self.assertFalse(np.isfinite(ground['range_m']).any())
        self.assertIsNone(surface['noise_db']);self.assertFalse(surface['noise_available'])
        self.assertEqual(surface['echo_regions'],[]);self.assertEqual(surface['shadow_candidates'],[])
        self.assertFalse(fit['accepted'])

    def test_partial_absence_is_censored_without_nonfinite_arithmetic(self):
        value=frame();value['intensity_db'][:,:8]=-np.inf
        value['intensity_db'][145:165,25:35]=-np.inf
        with np.errstate(all='raise'):
            edge=echo_boundaries(value)
            ground=ground_entries(value)
            surface=segment_surface(value,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
        self.assertTrue(edge['noise_available']);self.assertIsNotNone(surface['noise_db'])
        self.assertFalse(edge['tracking_mask'][:,:8].any())
        self.assertFalse(np.isfinite(ground['range_m'][:8]).any())
        shifted={**value,'intensity_db':value['intensity_db']+70}
        np.testing.assert_array_equal(echo_tracking_image(value)[0],echo_tracking_image(shifted)[0])

    def test_actual_flow_drops_absent_echoes_then_reseeds_after_recovery(self):
        stream=AcousticTracks();first=frame()
        a=stream.push(first);self.assertGreater(len(a['observations']),0)
        blank=frame();blank['intensity_db'][:]=-np.inf;blank['stamp']=np.array([1,500_000_000])
        lost=stream.push(blank)
        self.assertEqual(lost['observations'],[])
        self.assertNotEqual(lost.get('reason'),'invalid_acoustic_frame')
        recovered=frame();recovered['stamp']=np.array([2,0])
        result=stream.push(recovered)
        self.assertGreater(len(result['observations']),0)
        self.assertTrue(all(row['length']==1 for row in result['observations']))
        self.assertTrue(np.isneginf(blank['intensity_db']).all())

    def test_correspondence_cannot_assign_a_no_return_bin(self):
        first=frame();second=frame();second['stamp']=np.array([1,500_000_000])
        second['intensity_db'][:,:32]=-np.inf
        result=correspondences(first,second)
        self.assertTrue(return_support(first,result['previous_pixels']).all())
        self.assertTrue(return_support(second,result['current_pixels']).all())

    def test_log_quantiles_preserve_extended_real_limits_and_finite_results(self):
        values=np.arange(40,dtype=float).reshape(5,8)
        for q in (0.,.5,.7,.95,1.):
            for axis in (None,0,1):
                np.testing.assert_array_equal(log_quantile(values,q,axis=axis),np.quantile(values,q,axis=axis))
        with np.errstate(all='raise'):
            for q in (0.,.5,.7,1.):
                self.assertTrue(np.isneginf(log_quantile(np.full((5,8),-np.inf),q,axis=1)).all())
            self.assertTrue(np.isneginf(log_quantile([-np.inf,1.,2.],.25)))
            self.assertEqual(log_quantile([-np.inf,1.,2.],.5),1.)
            self.assertEqual(log_quantile([-np.inf,1.,2.],1.),2.)


@unittest.skipIf(isinstance(cv,NativeOpenCV),'Research filters require the Python OpenCV backend; native API supplies raw and echo-boundary images')
class ResearchFilterNoReturnTests(NoReturnTests):
    variants=FILTERS


if __name__=='__main__':unittest.main()

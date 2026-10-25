import unittest
import numpy as np
from posim_slam.sonar_edges import echo_boundaries, near_edge
from posim_slam.acoustic_tracks import AcousticTracks


def frame(*, gain=0., shift=0):
    ranges=np.linspace(.025,10,400);az=np.linspace(-1,1,64)
    power=np.full((400,64),-10.)
    power[80:]=30.  # broad ground onset at two metres
    power[120+shift:134+shift,20:40]=60.  # finite bright object
    power[134+shift:210+shift,20:40]=0.  # shadow, no measured elevation
    power[250,50]=100.  # isolated spike
    return dict(intensity_db=power+gain,ranges=ranges,
        beam_directions=np.column_stack((np.cos(az),np.sin(az),az*0)),stamp=np.array([1,0]))


class SonarEdgeTests(unittest.TestCase):
    def test_measured_bright_onset_without_ground_or_isolated_spike(self):
        f=frame();result=echo_boundaries(f,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
        rows,beams=np.nonzero(result['edge_mask'])
        self.assertTrue(np.any((rows>=118)&(rows<=123)&(beams>=21)&(beams<=38)))
        self.assertFalse(result['edge_mask'][70:90].any())
        self.assertFalse(result['edge_mask'][245:255].any())
        self.assertFalse(result['edge_mask'][132:136].any())  # falling shadow boundary
        self.assertFalse(result['edge_mask'][205:215].any())  # recovery to ordinary seabed
        self.assertNotIn('points',result)

    def test_gain_invariance_and_mask_does_not_create_edges(self):
        band=np.tile([1.8,2.2],(64,1))
        a=echo_boundaries(frame(),ground_entry_band_m=band)
        b=echo_boundaries(frame(gain=70),ground_entry_band_m=band)
        np.testing.assert_array_equal(a['image'],b['image'])
        np.testing.assert_array_equal(a['edge_mask'],b['edge_mask'])
        plain=frame();plain['intensity_db'][:]=30
        self.assertFalse(echo_boundaries(plain,ground_entry_band_m=band)['edge_mask'].any())
        # Masking an actual bright onset cannot create a shifted substitute.
        excluded=echo_boundaries(frame(),ground_entry_band_m=np.tile([2.8,3.3],(64,1)))
        self.assertGreater(excluded['ground_excluded_pixels'],0)
        self.assertFalse(excluded['tracking_mask'][112:132].any())

    def test_missing_geometry_rejects_without_fallback(self):
        result=AcousticTracks(frontend='echo_boundary').push(frame())
        self.assertEqual(result['state'],'lost')
        self.assertEqual(result['reason'],'invalid_ground_entry_band')
        self.assertEqual(result['observations'],[])
        with self.assertRaises(ValueError):AcousticTracks(frontend='unknown')

    def test_measured_edge_gate_rejects_outside_and_nonfinite_tracks(self):
        mask=np.zeros((20,20),dtype=bool);mask[10,10]=True
        np.testing.assert_array_equal(near_edge([[10,10],[12,12],[13,13],[-1,10],[np.nan,10]],mask),[True,True,False,False,False])

    def test_actual_flow_preserves_known_small_translation_under_gain_change(self):
        stream=AcousticTracks(frontend='echo_boundary');band=np.tile([1.8,2.2],(64,1))
        first=stream.push(frame(),ground_entry_band_m=band)
        second=frame(gain=20,shift=1);second['stamp']=np.array([1,500_000_000])
        result=stream.push(second,ground_entry_band_m=band)
        self.assertGreater(result.get('retained',0),0)
        old={o['track_id']:o for o in first['observations']}
        for observation in result['observations']:
            if observation['length']>1:
                np.testing.assert_allclose(np.array(observation['pixel'])-old[observation['track_id']]['pixel'],[0,1],atol=.15)


if __name__=='__main__':unittest.main()

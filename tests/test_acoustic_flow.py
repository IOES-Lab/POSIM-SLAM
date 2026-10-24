import json
import unittest
from unittest.mock import patch
import numpy as np
from posim_slam.acoustic_flow import correspondences, estimate, polar_observations


def frame(ns):
    angles=np.linspace(-.8,.8,32)
    return dict(intensity_db=np.full((40,32),50.,dtype=np.float32),ranges=np.linspace(.1,10.,40),
        beam_directions=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(32))),stamp=np.array([0,ns]))


class CorrespondenceTests(unittest.TestCase):
    def track(self,previous,current):
        pixels=np.array([[[5.,5.]],[[10.,10.]],[[15.,15.]],[[20.,20.]]],dtype=np.float32)
        moved=pixels+np.array([[[1.,2.]],[[2.,-1.]],[[1.,1.]],[[0.,0.]]],dtype=np.float32)
        back=pixels.copy();back[2,0,0]+=1.
        forward=np.array([[1],[1],[1],[0]],dtype=np.uint8);reverse=np.ones((4,1),dtype=np.uint8)
        with patch('posim_slam.acoustic_flow.cv2.goodFeaturesToTrack',return_value=pixels),patch(
                'posim_slam.acoustic_flow.cv2.calcOpticalFlowPyrLK',side_effect=[(moved,forward,None),(back,reverse,None)]):
            return correspondences(previous,current)

    def test_pairs_keep_timestamps_polar_values_and_pixel_identity(self):
        a,b=frame(100),frame(200);r=self.track(a,b)
        self.assertEqual((r['previous_stamp_ns'],r['current_stamp_ns']),(100,200))
        self.assertEqual((r['corners'],r['matched']),(4,2))
        self.assertEqual(r['previous_pixels'],[[5.,5.],[10.,10.]])
        self.assertEqual(r['current_pixels'],[[6.,7.],[12.,9.]])
        np.testing.assert_allclose(r['previous_range_azimuth'],polar_observations(np.array([[5.,5.],[10.,10.]]),a))
        np.testing.assert_allclose(r['current_range_azimuth'],polar_observations(np.array([[6.,7.],[12.,9.]]),b))
        self.assertEqual(r['forward_backward_error_pixels'],[0.,0.])
        json.dumps(r,allow_nan=False)

    def test_insufficient_planar_tracks_still_expose_observations(self):
        a,b=frame(100),frame(200);tracks=self.track(a,b)
        with patch('posim_slam.acoustic_flow.correspondences',return_value=tracks):
            r=estimate(a,b,'raw',include_correspondences=True)
        self.assertFalse(r['fitted']);self.assertEqual(r['correspondences'],tracks)
        self.assertNotIn('translation',r)
        with patch('posim_slam.acoustic_flow.correspondences',return_value=tracks):
            self.assertNotIn('correspondences',estimate(a,b,'raw'))

    def test_empty_frames_have_no_invented_tracks(self):
        r=correspondences(frame(100),frame(200))
        self.assertEqual(r['matched'],0);self.assertEqual(r['previous_range_azimuth'],[])
        self.assertEqual(r['reason'],'no_acoustic_corners')

    def test_pair_rejects_time_and_calibration_changes(self):
        a=frame(100)
        for b in (frame(100),frame(99),dict(frame(200),ranges=frame(200)['ranges']*1.01)):
            with self.assertRaises(ValueError):correspondences(a,b)


if __name__=='__main__':unittest.main()

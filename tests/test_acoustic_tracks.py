import unittest
from unittest.mock import patch
import numpy as np
from posim_slam.acoustic_tracks import AcousticTracks
from test_acoustic_flow import frame


class PersistentTrackTests(unittest.TestCase):
    def test_survivors_use_previous_coordinates_and_ids_are_not_recycled(self):
        stream=AcousticTracks(max_tracks=2)
        with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.],[15.,15.]],dtype=np.float32)):
            first=stream.push(frame(100))
        result=dict(matched=1,previous_indices=[0],current_pixels=[[6.,5.]],forward_backward_error_pixels=[.1])
        with patch('posim_slam.acoustic_tracks.correspondences',return_value=result) as flow,patch(
                'posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[6.,5.],[24.,24.]],dtype=np.float32)):
            second=stream.push(frame(200))
            np.testing.assert_array_equal(flow.call_args.kwargs['seed_pixels'],[[5.,5.],[15.,15.]])
        self.assertEqual([o['track_id'] for o in first['observations']],[0,1])
        self.assertEqual([o['track_id'] for o in second['observations']],[0,2])
        self.assertEqual([o['length'] for o in second['observations']],[2,1])
        self.assertEqual(second['observations'][0]['forward_backward_error_pixels'],.1)
        self.assertIsNone(second['observations'][1]['forward_backward_error_pixels'])
        np.testing.assert_array_equal(stream.pixels,[[6.,5.],[24.,24.]])

    def test_losses_clear_observations_and_break_id_namespace(self):
        for reason in ('gap','time','calibration','flow','invalid'):
            with self.subTest(reason=reason):
                stream=AcousticTracks();a=frame(100)
                with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.]],dtype=np.float32)):
                    stream.push(a)
                a['intensity_db'][:]=123
                self.assertFalse(np.all(stream.previous['intensity_db']==123))
                b=frame(200)
                if reason=='gap':b['stamp']=np.array([3,0])
                if reason=='time':b['stamp']=np.array([0,99])
                if reason=='calibration':b['ranges']=b['ranges']*2
                if reason=='invalid':b['intensity_db'][0,0]=np.nan
                if reason=='flow':
                    with patch('posim_slam.acoustic_tracks.correspondences',return_value={'matched':0}):r=stream.push(b)
                else:r=stream.push(b)
                self.assertEqual(r['state'],'lost');self.assertEqual(r['observations'],[])
                self.assertEqual(stream.segment,1);self.assertIsNone(stream.previous)
                with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.]],dtype=np.float32)):
                    new=stream.push(frame(300))
                self.assertEqual(new['segment'],1);self.assertEqual(new['observations'][0]['length'],1)

    def test_capacity_and_featureless_input(self):
        stream=AcousticTracks(max_tracks=1)
        self.assertEqual(stream.push(frame(100))['observations'],[])
        with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.],[15.,15.]],dtype=np.float32)):
            result=stream.push(frame(200))
        self.assertEqual(len(result['observations']),1)
        for capacity in (0,301,True):
            with self.assertRaises(ValueError):AcousticTracks(max_tracks=capacity)

    def test_colliding_tracks_do_not_duplicate_landmark_identity(self):
        stream=AcousticTracks()
        with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.],[15.,15.]],dtype=np.float32)):
            stream.push(frame(100))
        value=dict(matched=2,previous_indices=[0,1],current_pixels=[[10.,10.],[10.2,10.]],forward_backward_error_pixels=[.1,.1])
        with patch('posim_slam.acoustic_tracks.correspondences',return_value=value):result=stream.push(frame(200))
        self.assertEqual(result['reason'],'ambiguous_acoustic_tracks')
        self.assertEqual(result['observations'],[])


class ReferenceTrackTests(unittest.TestCase):
    def test_pairwise_survivor_rejected_when_reference_disagrees(self):
        stream=AcousticTracks(max_tracks=1,reference_check=True)
        with patch('posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.]],dtype=np.float32)):
            stream.push(frame(100))
        pair=dict(matched=1,previous_indices=[0],current_pixels=[[6.,5.]],forward_backward_error_pixels=[.1])
        with patch('posim_slam.acoustic_tracks.correspondences',return_value=pair),patch(
                'posim_slam.acoustic_tracks.corner_pixels',return_value=[]):
            stream.push(frame(200))
        direct=dict(matched=1,previous_indices=[0],current_pixels=[[8.,5.]],forward_backward_error_pixels=[.1])
        with patch('posim_slam.acoustic_tracks.correspondences',side_effect=[pair,direct]),patch(
                'posim_slam.acoustic_tracks.corner_pixels',return_value=[]):
            result=stream.push(frame(300))
        self.assertEqual(result['reference_rejected'],1)
        self.assertEqual(result['observations'],[])
        self.assertEqual(stream.references,{})

    def test_reference_lifetime_is_bounded_and_id_changes(self):
        stream=AcousticTracks(max_tracks=1,reference_check=True)
        pair=dict(matched=1,previous_indices=[0],current_pixels=[[5.,5.]],forward_backward_error_pixels=[.1])
        with patch('posim_slam.acoustic_tracks.correspondences',return_value=pair),patch(
                'posim_slam.acoustic_tracks.corner_pixels',return_value=np.array([[5.,5.]],dtype=np.float32)):
            results=[stream.push(frame(i*100)) for i in range(1,10)]
        self.assertEqual(results[7]['observations'][0]['length'],8)
        self.assertEqual(results[8]['reference_expired'],1)
        self.assertEqual(results[8]['observations'][0]['track_id'],1)
        self.assertEqual(results[8]['observations'][0]['length'],1)
        self.assertEqual(len(stream.references),1)
        stream.reset();self.assertEqual(stream.references,{})


if __name__=='__main__':unittest.main()

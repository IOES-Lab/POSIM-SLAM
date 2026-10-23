import sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from posim_slam.acoustic_stream import AcousticStream


def sample(index):
    # Keep stream tests usable on SDK Python without the research cv2 module.
    angles=np.linspace(-.8,.8,129)
    return {'intensity_db':np.full((399,129),20.,dtype=np.float32),
            'ranges':np.linspace(.1,10.,399),
            'beam_directions':np.column_stack((np.cos(angles),np.sin(angles),np.zeros(129))),
            'stamp':np.array([index//2,(index%2)*500000000])}

class StreamTests(unittest.TestCase):
    def test_keyframes_match_fixed_step_and_own_their_input(self):
        stream=AcousticStream();a=sample(0)
        self.assertEqual(stream.push(a)['state'],'initializing')
        a['intensity_db'][:]=999
        self.assertFalse((stream.anchor['intensity_db']==999).any())
        with patch('posim_slam.acoustic_stream.estimate',return_value={'fitted':True,'translation':[.2,0.],'yaw_rad':np.pi/2}):
            results=[stream.push(sample(i)) for i in range(1,9)]
        self.assertEqual([r['state'] for r in results],['waiting']*3+['tracking']+['waiting']*3+['tracking'])
        np.testing.assert_allclose(results[-1]['position_fan_m'],[.2,.2],atol=1e-12)
        self.assertEqual(results[3]['anchor_stamp'],0.)
        self.assertEqual(results[-1]['anchor_stamp'],2.)

    def test_loss_never_bridges_unknown_motion(self):
        for case in ('gap','backward','calibration','invalid','negative_time','fractional_stamp'):
            stream=AcousticStream();stream.push(sample(0));f=sample(1)
            if case=='gap':f['stamp']=np.array([5,0])
            if case=='backward':f['stamp']=np.array([0,0])
            if case=='calibration':f['ranges']=f['ranges']*1.01
            if case=='invalid':f['intensity_db'][0,0]=np.nan
            if case=='negative_time':f['stamp']=np.array([-1,0])
            if case=='fractional_stamp':f['stamp']=np.array([0,0.5])
            result=stream.push(f)
            self.assertEqual(result['state'],'lost');self.assertNotIn('position_fan_m',result)
            self.assertEqual(stream.push(sample(12))['state'],'initializing')
            self.assertEqual(stream.segment,1)

    def test_no_echo_does_not_emit_pose(self):
        stream=AcousticStream()
        results=[stream.push(sample(i)) for i in range(5)]
        self.assertEqual(results[-1]['state'],'lost')
        self.assertNotIn('position_fan_m',results[-1])

if __name__=='__main__':unittest.main()

import sys,unittest,copy
from pathlib import Path
import numpy as np
from posim_slam.acoustic_dvl_session import AcousticDvlSession


def measurement(t,**changes):
    result={'session_nonce':'test','source':'/vehicle/sensors/dvl','frame_id':'vehicle/dvl',
            'version':'0.1.0','valid':True,'reason':'bottom_locked','locked_beams':4,
            'stamp':{'sec':int(t),'nanosec':int(round((t-int(t))*1e9))},
            'velocity_m_s':[.025,0,0],'covariance':(np.eye(3)*.000004).tolist()}
    result.update(changes);return result


def acoustic(start,end,**changes):
    result={'state':'tracking','segment':0,'anchor_stamp':start,'stamp':end,
            'registration':{'translation':[.07,0],'yaw_rad':0.,
                            'inlier_points_current_fan_m':[[1.,0],[2.,0],[3.,0]]}}
    result.update(changes);return result


class SessionTests(unittest.TestCase):
    def session(self):
        return AcousticDvlSession('test',{'velocity_to_sonar':np.eye(3),'sonar_to_dvl':[0,0,0]})
    def tracking(self):
        s=self.session();s.set_enabled(True)
        for t in (0,1,2):s.dvl(measurement(t),t*.1)
        result=s.acoustic(acoustic(0,2),.3)
        self.assertEqual(result[-1]['state'],'tracking')
        return s

    def test_disabled_and_actual_dvl_contribution_to_map(self):
        s=self.session()
        self.assertFalse(s.dvl(measurement(0),0));self.assertFalse(s.acoustic(acoustic(0,2),0))
        s=self.tracking()
        np.testing.assert_allclose(s.position,[.05,0])
        np.testing.assert_allclose(s.feature_map.snapshot()['points_fan_m'],[[1.05,0],[2.05,0],[3.05,0]])
        self.assertEqual(s.set_enabled(False)[-1]['state'],'disabled')
        self.assertFalse(s.feature_map.cells);self.assertFalse(s.intervals.samples)

    def test_delayed_bracketing_sample_flushes_once(self):
        s=self.session();s.set_enabled(True);s.dvl(measurement(0),0)
        frame=acoustic(.25,2.25)
        self.assertFalse(s.acoustic(frame,.1));self.assertEqual(s.state,'waiting_dvl')
        frame['registration']['inlier_points_current_fan_m'][0][0]=999
        for t in (1,2):self.assertFalse(s.dvl(measurement(t),t*.1))
        updates=s.dvl(measurement(3),.4)
        self.assertEqual(len(updates),1);self.assertEqual(len(s.feature_map.cells),3)
        self.assertLess(max(p[0] for p in s.feature_map.snapshot()['points_fan_m']),4)
        self.assertFalse(s.tick(.5));self.assertFalse(s.pending)

    def test_wrong_nonce_ignored_wrong_source_invalidates(self):
        s=self.tracking();segment=s.segment
        s.dvl(measurement(3,session_nonce='old'),.4)
        self.assertEqual(s.segment,segment);self.assertTrue(s.feature_map.cells)
        s.dvl(measurement(3,source='/wrong'),.4)
        self.assertEqual(s.state,'lost');self.assertFalse(s.feature_map.cells)
        self.assertFalse(s.intervals.samples)

    def test_lock_loss_and_recovery_never_bridge_motion(self):
        s=self.tracking();old=s.segment
        s.dvl(measurement(3,valid=False),.4)
        self.assertFalse(s.feature_map.cells)
        for t in (3,4,5,6):s.dvl(measurement(t),.5+t*.01)
        self.assertEqual(s.acoustic(acoustic(2,4),.7)[-1]['state'],'lost')
        self.assertFalse(s.feature_map.cells)
        result=s.acoustic(acoustic(4,6),.8)[-1]
        self.assertGreater(result['segment'],old)
        np.testing.assert_allclose(result['position_fan_m'],[.05,0])

    def test_stale_timeout_and_gap_clear_maps(self):
        for which in ('dvl','acoustic','gap'):
            s=self.tracking()
            if which=='dvl':
                s.acoustic({'state':'waiting','segment':0,'stamp':2.5},3.1)
                updates=s.tick(3.3)
            elif which=='acoustic':updates=s.tick(3.4)
            else:updates=s.dvl(measurement(5),.4)
            self.assertEqual(updates[-1]['state'],'lost',which)
            self.assertFalse(s.feature_map.cells,which)

    def test_normal_excursion_clears_existing_map_and_starts_new_segment(self):
        s=self.tracking();old=s.segment
        for t in (3,4):s.dvl(measurement(t,velocity_m_s=[.025,0,.1]),.4+t*.01)
        update=s.acoustic(acoustic(2,4),.8)[-1]
        self.assertEqual(update['reason'],'dvl_motion_outside_sonar_plane')
        self.assertEqual(update['state'],'lost');self.assertGreater(s.segment,old)
        self.assertFalse(s.feature_map.cells)

    def test_pending_timeout_and_capacity(self):
        s=self.session();s.set_enabled(True);s.dvl(measurement(0),0)
        s.acoustic(acoustic(0,2),.1)
        self.assertEqual(s.tick(3.2)[-1]['state'],'lost')
        self.assertFalse(s.pending)
        s=self.session();s.set_enabled(True)
        for i in range(3):self.assertFalse(s.acoustic(acoustic(i*2,i*2+2),i*.1))
        self.assertEqual(s.acoustic(acoustic(6,8),.4)[-1]['reason'],'dvl_pairing_overflow')
        self.assertFalse(s.pending)

    def test_acoustic_loss_or_segment_change_clears_fused_map(self):
        for value in ({'state':'lost','segment':1},{'state':'initializing','segment':1,'stamp':3.}):
            s=self.tracking();s.acoustic(value,.4)
            self.assertFalse(s.feature_map.cells)
            self.assertIsNone(s.last_end)

    def test_disagreement_has_no_raw_fallback(self):
        s=self.tracking()
        for t in (3,4):s.dvl(measurement(t),t*.1)
        v=acoustic(2,4);v['registration']['translation']=[1,0]
        self.assertEqual(s.acoustic(v,.5)[-1]['reason'],'acoustic_dvl_disagreement')
        self.assertFalse(s.feature_map.cells)

    def test_invalid_features_do_not_advance_fused_pose(self):
        s=self.tracking()
        for t in (3,4):s.dvl(measurement(t),t*.1)
        v=acoustic(2,4);v['registration']['inlier_points_current_fan_m']=[[float('nan'),0]]
        self.assertEqual(s.acoustic(v,.5)[-1]['state'],'lost')
        np.testing.assert_array_equal(s.position,[0,0]);self.assertFalse(s.feature_map.cells)


if __name__=='__main__':unittest.main()

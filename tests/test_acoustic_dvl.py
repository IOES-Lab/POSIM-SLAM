import sys,unittest,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
from posim_slam.acoustic_dvl import AcousticDvlIntervals

COV=np.eye(3)*.000004


class AcousticDvlTests(unittest.TestCase):
    def window(self,baseline=(0,0,0)):
        return AcousticDvlIntervals(np.eye(3),baseline)

    def test_noncoincident_clocks_interpolate_measured_velocity(self):
        w=self.window()
        for t in (0.,1.,2.,3.):w.add(t,[.02*t,0,0],COV)
        # Integral of .02t from .25 to 2.25 = .05 m.
        result=w.fuse(.25,2.25,[.07,0],0.)
        np.testing.assert_allclose(result['dvl_translation_m'],[.05,0],atol=1e-12)
        np.testing.assert_allclose(result['translation'],[.05,0],atol=1e-12)
        self.assertFalse(result['elevation_resolved'])

    def test_no_extrapolation_gap_or_time_reversal(self):
        w=self.window();w.add(0.,[0,0,0],COV);w.add(1.,[0,0,0],COV)
        self.assertEqual(w.fuse(0.,2.,[0,0],0.)['state'],'waiting_dvl')
        w.add(3.,[0,0,0],COV)
        with self.assertRaisesRegex(ValueError,'not_covered'):w.fuse(0.,3.,[0,0],0.)
        with self.assertRaisesRegex(ValueError,'time_reversal'):w.add(3.,[0,0,0],COV)
        self.assertEqual(len(w.samples),0)

    def test_rotation_compensates_sensor_lever_arm(self):
        baseline=np.array([.2,.05,0]);w=self.window(baseline);omega=.1
        velocity=[-omega*baseline[1],omega*baseline[0],0]
        for t in (0.,1.,2.):w.add(t,velocity,COV)
        result=w.fuse(0.,2.,[0,0],.2)
        # Numerical trapezoid integration error is below 0.04 mm.
        self.assertLess(np.linalg.norm(result['dvl_translation_m']),.00004)

    def test_rejects_disagreement_bad_quality_and_unbounded_rotation(self):
        w=self.window()
        for t in (0.,1.,2.):w.add(t,[0,0,0],COV)
        with self.assertRaisesRegex(ValueError,'disagreement'):w.fuse(0.,2.,[1,0],0.)
        with self.assertRaisesRegex(ValueError,'rotation_too_fast'):w.fuse(0.,2.,[0,0],.3)
        for value in (np.eye(3)*.1,np.eye(3)*-1,np.full((3,3),np.nan)):
            with self.assertRaisesRegex(ValueError,'covariance'):w.add(3.,[0,0,0],value)
            self.assertFalse(w.samples)
        with self.assertRaisesRegex(ValueError,'tracking_lost'):w.add(4.,[0,0,0],COV,valid=False)

    def test_bounded_history_and_owned_samples(self):
        w=self.window();v=np.array([.1,0,0])
        for t in range(100):w.add(t,v,COV)
        v[:]=999
        self.assertEqual(len(w.samples),32)
        np.testing.assert_allclose(w.fuse(97.,99.,[.2,0],0.)['translation'],[.2,0])
        with self.assertRaisesRegex(ValueError,'not_covered'):w.fuse(0.,2.,[0,0],0.)



if __name__=='__main__':unittest.main()

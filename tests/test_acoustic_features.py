"""Acoustic detections preserve range/bearing and reject unsupported metadata."""
import sys,unittest
from pathlib import Path
import numpy as np
from posim_slam.acoustic_features import detect

class AcousticFeaturesTests(unittest.TestCase):
    def fixture(self):
        angles=np.array([-.5,0,.5]);directions=np.column_stack((np.cos(angles),np.sin(angles),np.zeros(3)))
        return np.zeros((50,3)),np.arange(50)*.1,directions
    def test_polar_peaks_and_separation(self):
        p,r,d=self.fixture();p[10,:]=30;p[12,:]=25;p[30,:]=20
        result=detect(p,r,d)
        self.assertEqual(len(result),6)
        for beam in range(3):
            values=[v for v in result if v['beam']==beam]
            self.assertEqual([v['range_m'] for v in values],[1.,3.])
            self.assertAlmostEqual(values[0]['bearing_rad'],[-.5,0,.5][beam])
    def test_blank_nonfinite_and_threshold(self):
        p,r,d=self.fixture()
        self.assertEqual(detect(p,r,d),[])
        p[:]=float('-inf');self.assertEqual(detect(p,r,d),[])
        p[:]=0;p[20,:]=11;self.assertEqual(detect(p,r,d),[])
        p[20,:]=15;p[1,0]=float('nan')
        self.assertEqual(len(detect(p,r,d)),3)
    def test_geometry_and_bounds(self):
        p,r,d=self.fixture()
        for ranges,dirs in ((r[::-1],d),(r,d*2),(r,np.tile([0,0,1],(3,1)))):
            with self.assertRaises(ValueError):detect(p,ranges,dirs)
        with self.assertRaises(ValueError):detect(p[:2],r[:2],d)
        with self.assertRaises(ValueError):detect(p,r,d,peaks_per_beam=100)
    def test_bound_and_offset_invariance(self):
        p,r,d=self.fixture();p[::3,:]=30
        a=detect(p,r,d,peaks_per_beam=2);b=detect(p+80,r,d,peaks_per_beam=2)
        self.assertLessEqual(len(a),6)
        self.assertEqual([(v['beam'],v['bin'],v['contrast_db']) for v in a],[(v['beam'],v['bin'],v['contrast_db']) for v in b])
if __name__=='__main__':unittest.main()

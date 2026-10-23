import sys,unittest
from pathlib import Path
import numpy as np
from posim_slam.acoustic_registration import register
class RegistrationTests(unittest.TestCase):
    def test_known_small_motion(self):
        rng=np.random.default_rng(29);points=rng.uniform([-3,1],[3,8],(220,2))
        angle=.015;r=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
        t=np.array([.04,-.03]);current=(points-t)@r
        value=register(points,current)
        np.testing.assert_allclose(value['translation'],t,atol=.001)
        self.assertAlmostEqual(value['yaw_rad'],angle,places=3)
    def test_static_and_no_overlap(self):
        points=np.random.default_rng(3).uniform(-3,3,(200,2))
        np.testing.assert_allclose(register(points,points)['translation'],[0,0],atol=1e-8)
        with self.assertRaises(ValueError):register(points,points+100)
    def test_degenerate_and_nonfinite(self):
        points=np.column_stack((np.arange(40),np.zeros(40)))
        with self.assertRaisesRegex(ValueError,'degenerate'):register(points,points)
        points[0,0]=np.nan
        with self.assertRaises(ValueError):register(points,points)
if __name__=='__main__':unittest.main()

"""Pose interpolation and DVL stream reset checks without ROS dependencies."""
from pathlib import Path
import json
import sys
import time
import unittest
from types import SimpleNamespace
import numpy as np
from posim_slam.dvl_sync import PoseBuffer
from posim_slam.dvl_scale import DvlScaleWindow

class Tests(unittest.TestCase):
    def test_interpolated_translation_and_rotation(self):
        b=PoseBuffer();a=.2
        r=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
        b.add(0,[0,0,0],np.eye(3));b.add(.2,[.2,0,0],r)
        p,q=b.at(.1);np.testing.assert_allclose(p,[.1,0,0]);self.assertAlmostEqual(q[0,0],np.cos(.1))
        self.assertIsNone(b.at(.3))
    def test_physics_step_alignment_without_bridging_missing_frames(self):
        b=PoseBuffer();b.add(4.,[1,2,3],np.eye(3))
        np.testing.assert_array_equal(b.at(4.001)[0],[1,2,3])
        self.assertIsNone(b.at(4.003))
        b.add(4.6,[2,2,3],np.eye(3))
        np.testing.assert_array_equal(b.at(4.001)[0],[1,2,3])
        np.testing.assert_array_equal(b.at(4.599)[0],[2,2,3])
        with self.assertRaisesRegex(ValueError,'interpolation_gap'):b.at(4.3)
    def test_gaps_reversal_and_bounded_history(self):
        b=PoseBuffer();b.add(0,[0,0,0],np.eye(3));b.add(1,[1,0,0],np.eye(3))
        with self.assertRaisesRegex(ValueError,'interpolation_gap'):b.at(.5)
        with self.assertRaisesRegex(ValueError,'time_reversal'):b.add(.5,[0,0,0],np.eye(3))
        self.assertEqual(len(b.values),0)
        for i in range(100):b.add(i,[i,0,0],np.eye(3))
        self.assertEqual(len(b.values),30)

if __name__=='__main__':unittest.main()

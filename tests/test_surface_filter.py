"""Water-plane clipping in metric map coordinates and complete PLY export."""
from pathlib import Path
import sys
import unittest
import numpy as np
from posim_slam.surface_filter import SurfaceFilter

class SurfaceTests(unittest.TestCase):
    def test_underwater_surface_and_air(self):
        points=np.array([[0,0,-3],[1,0,-.31],[2,0,-.3],[3,0,0],[4,0,2]],dtype='<f4')
        # Use exactly representable boundary for this classification assertion.
        f=SurfaceFilter([0,0,1,0],.25)
        points[2,2]=-.25
        np.testing.assert_array_equal(f.apply(points),points[:2])
        np.testing.assert_array_equal(points[3],[3,0,0]) # raw data untouched

    def test_rotated_translated_plane(self):
        # Water surface x=5, air lies toward +x. Map z has no world-height meaning.
        points=np.array([[4,0,100],[4.8,0,-100],[6,0,-100]],dtype='<f4')
        np.testing.assert_array_equal(SurfaceFilter([2,0,0,-10]).apply(points),points[:1])

    def test_unknown_plane_and_reset(self):
        points=np.array([[0,0,-1]],dtype='<f4')
        f=SurfaceFilter();np.testing.assert_array_equal(f.apply(points),points)
        self.assertEqual(f.status()['state'],'unconfigured')
        f=SurfaceFilter([0,0,1,0]);f.invalidate() # startup waiting does not invalidate
        f.apply(points);f.invalidate()
        self.assertEqual(f.status()['state'],'map_reset')
        with self.assertRaisesRegex(ValueError,'recalibration'):f.apply(points)

    def test_reject_bad_calibration_and_points(self):
        for plane in ([0,0,0,1],[0,0,float('nan'),0],[0,1,0]):
            with self.assertRaises(ValueError):SurfaceFilter(plane)
        for clearance in (-1,float('inf'),6):
            with self.assertRaises(ValueError):SurfaceFilter(None,clearance)
        with self.assertRaises(ValueError):SurfaceFilter().apply([[0,0,float('nan')]])


if __name__=='__main__':unittest.main()

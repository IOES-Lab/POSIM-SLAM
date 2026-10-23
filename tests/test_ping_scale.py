"""Metric scale recovery with sensor baseline, tilt, invalid and stale data."""
from pathlib import Path
import sys
import unittest
import numpy as np
from posim_slam.ping_scale import candidate_scale, ScaleWindow, PING_ORIGIN, PING_DIRECTIONS, PING_BIAS_M

class ScaleTests(unittest.TestCase):
    def plane(self, scale=2.5, normal=None):
        normal=np.array([.15,-.1,1.]) if normal is None else normal
        normal=normal/np.linalg.norm(normal)
        xy=np.array([(x,y) for x in np.linspace(-.55,.55,20) for y in np.linspace(-.55,.55,20)])
        z=(2.-xy@normal[:2])/normal[2]
        points=np.column_stack([xy,z])/scale
        distance=(2.-normal@PING_ORIGIN)/np.max(PING_DIRECTIONS@normal)+PING_BIAS_M
        return points,distance
    def test_scale_and_baseline_on_tilted_plane(self):
        for scale in [.2,1.,2.5,20.]:
            points,distance=self.plane(scale)
            self.assertAlmostEqual(candidate_scale(points,distance)['scale'],scale,places=8)
    def test_requires_temporal_evidence_and_rejects_stale_ping(self):
        points,distance=self.plane();window=ScaleWindow()
        for i in range(8):
            result=window.add(i*.2,points,distance,i*.2)
            self.assertEqual(result['ready'],i==7)
        with self.assertRaises(ValueError):window.add(1.6,points,distance,1.2)
        self.assertFalse(window.add(1.8,points,distance,1.8)['ready'])
        self.assertFalse(window.add(0.,points,distance,0.)['ready'])
    def test_inconsistent_scale_is_not_ready(self):
        points,distance=self.plane();window=ScaleWindow()
        for i in range(15):
            result=window.add(i*.2,points,distance+(i%2)*.5,i*.2)
            self.assertFalse(result["ready"])
        with self.assertRaises(ValueError):window.add(window.last_stamp,points,distance,window.last_stamp)

    def test_missing_surface_invalid_ranges(self):
        points,distance=self.plane()
        for value in [float('nan'),float('inf'),-.1,100.]:
            with self.assertRaises(ValueError):candidate_scale(points,value)
        with self.assertRaises(ValueError):candidate_scale(points[:10],distance)
        with self.assertRaises(ValueError):candidate_scale(np.zeros((30,3)),distance)

    def test_triangular_patch_cannot_extrapolate_into_missing_corner(self):
        points,distance=self.plane(scale=1.,normal=np.array([0.,0.,1.]))
        triangle=points[points[:,0]+points[:,1]<.2]
        # Its x/y bounding rectangle encloses the Ping footprint but its hull
        # does not; the positive-x/positive-y corner has no observed surface.
        with self.assertRaisesRegex(ValueError,'does_not_cover'):
            candidate_scale(triangle,distance)
    def test_outliers_and_rough_surface(self):
        points,distance=self.plane()
        noisy=points.copy();noisy[:20,2]+=.3
        self.assertAlmostEqual(candidate_scale(noisy,distance)['scale'],2.5,places=8)
        rng=np.random.default_rng(1);noisy=points.copy();noisy[:,2]+=rng.uniform(-.2,.2,len(noisy))
        with self.assertRaises(ValueError):candidate_scale(noisy,distance)

if __name__=='__main__':unittest.main()

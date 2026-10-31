# SPDX-License-Identifier: Apache-2.0
"""Connectivity, aperture extrema, gain and zero-power controls."""
from pathlib import Path
import json
import unittest
import numpy as np
from shadow_extract import components,plane_aperture,extract,plane_background
from posim_slam.sonar_surface_power import SurfacePowerModel

ROOT=Path(__file__).resolve().parent


class ShadowExtraction(unittest.TestCase):
    def test_connected_components_against_pixel_flood_fill(self):
        rng=np.random.default_rng(415)
        for density in (.02,.3,.8):
            mask=rng.random((17,31))<density;remaining=set(map(tuple,np.argwhere(mask)))
            expected=[]
            while remaining:
                stack=[remaining.pop()];group=set(stack)
                while stack:
                    y,x=stack.pop()
                    for dy in (-1,0,1):
                        for dx in (-1,0,1):
                            point=y+dy,x+dx
                            if point in remaining:remaining.remove(point);group.add(point);stack.append(point)
                expected.append(frozenset(group))
            actual=[frozenset((y,beam) for beam,start,stop in row['runs'] for y in range(start,stop))
                    for row in components(mask)]
            self.assertEqual(set(actual),set(expected))

    def test_analytic_aperture_extrema(self):
        a=np.linspace(-1.,1.,19);e=np.linspace(-.2,.2,100001)
        for n in ([.8,0.,-.6],[0.,0.,-1.],[1.,0.,0.],[.6,.8,0.]):
            near,far=plane_aperture(a,n,2.,.4)
            dot=(n[0]*np.cos(a)+n[1]*np.sin(a))[:,None]*np.cos(e)+n[2]*np.sin(e)
            supported=dot.min(axis=1)>0
            np.testing.assert_allclose(near[dot.max(axis=1)>0],2/dot.max(axis=1)[dot.max(axis=1)>0],atol=2e-9)
            np.testing.assert_allclose(far[supported],2/dot.min(axis=1)[supported],atol=2e-9)
            self.assertTrue(np.isinf(far[~supported]).all())

    def test_original_input_gain_and_zero_power(self):
        data=json.loads((ROOT/'inputs.json').read_text());row=data['frames'][53]
        reference=json.loads((ROOT/'references.json').read_text())['representative_raw'][1]
        with np.load(ROOT/reference['path'],allow_pickle=False) as z:frame=dict(z)
        a=np.arctan2(frame['beam_directions'][:,1],frame['beam_directions'][:,0])
        model=SurfacePowerModel(a,np.linspace(-data['calibration']['sonar']['vertical_fov']/2,
                  data['calibration']['sonar']['vertical_fov']/2,256),frame['ranges'],maximum_range_m=15.)
        expected=plane_background(model,row['normal_sonar'],row['plane_distance_m'])
        kwargs=dict(normal=row['normal_sonar'],distance=row['plane_distance_m'],
                    vertical_fov=data['calibration']['sonar']['vertical_fov'],range_max=15.,expected_power=expected)
        original=extract(frame,**kwargs)
        gained=dict(frame,intensity_db=frame['intensity_db'].astype(float)+20.)
        shifted=extract(gained,**kwargs)
        self.assertEqual(original['components'],shifted['components'])
        np.testing.assert_allclose(original['contrast'],shifted['contrast'],atol=2e-13)
        self.assertAlmostEqual(shifted['gain_db']-original['gain_db'],20.,places=11)
        scaled=extract(frame,**dict(kwargs,expected_power=expected*1e6))
        self.assertEqual(original['components'],scaled['components'])
        empty=extract(dict(frame,intensity_db=np.full_like(frame['intensity_db'],-np.inf)),**kwargs)
        self.assertFalse(empty['available']);self.assertEqual(empty['components'],[])


if __name__=='__main__':unittest.main()

import math
import unittest
from dataclasses import replace
import numpy as np
from posim_slam.sonar_ground import GroundEntryConfig, ground_entries, fit_ground_entry, ground_entry_curve


def scene(*,gain=0.,occlusion=False,visible=True,normal=None):
    angles=np.linspace(-math.pi/3,math.pi/3,256);ranges=np.linspace(.05,10,400)
    up=np.array([-math.sin(.35),0.,math.cos(.35)])
    normal=np.array([-.45,.08,math.sqrt(1-.45**2-.08**2)]) if normal is None else normal
    # Independent dense fan intersections, rather than the fitter's model.
    elevation=np.linspace(-.175,.175,4097)
    entry=[]
    for az in angles:
        rays=np.column_stack([np.cos(elevation)*np.cos(az),np.cos(elevation)*np.sin(az),np.sin(elevation)])
        dots=rays@normal
        entry.append(np.min(-2.2/dots[dots<0]))
    entry=np.array(entry)
    if occlusion:entry[95:155]=1.5  # a nearer object occupies less than half the fan
    rng=np.random.default_rng(28);db=rng.normal(-15,.4,(len(ranges),len(angles)))
    if visible:db+=45*(ranges[:,None]>=entry)
    return dict(intensity_db=db+gain,ranges=ranges,stamp=np.array([1,0]),
                beam_directions=np.column_stack([np.cos(angles),np.sin(angles),angles*0])),up,normal


class GroundEntryTests(unittest.TestCase):
    def test_curve_preserves_measured_onsets_without_plane_or_elevation_claim(self):
        frame,up,_=scene(occlusion=True)
        curve=ground_entry_curve(frame,vertical_fov_rad=.35,up_sonar=up)
        plane=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)
        self.assertTrue(curve['accepted'])
        self.assertGreater(len(curve['beam_ids']),len(plane['inlier_beams']))
        np.testing.assert_allclose(np.linalg.norm(curve['points_sonar'],axis=1),
                                   curve['entries']['range_m'][curve['beam_ids']])
        self.assertFalse(curve['map_qualified'])
        self.assertIn('elevation not measured',curve['scope'])

    def test_recovers_sloped_plane_from_independent_fan_intersections(self):
        frame,up,normal=scene()
        result=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)
        self.assertTrue(result['accepted'])
        self.assertLess(math.acos(np.clip(result['normal_sonar']@normal,-1,1)),.015)
        self.assertAlmostEqual(result['distance_m'],-2.2,delta=.04)
        self.assertLess(result['range_rmse_m'],.02)
        # The points are actual observed entries; the fit does not fill a mesh.
        lengths=np.linalg.norm(result['points_sonar'],axis=1)
        np.testing.assert_allclose(lengths,result['entries']['range_m'][result['inlier_beams']])
        self.assertFalse(result['map_qualified']);self.assertFalse(result['navigation_ready'])

    def test_occlusion_rejected_and_gain_shift_does_not_move_plane(self):
        frame,up,_=scene(occlusion=True)
        shifted,_,_=scene(occlusion=True,gain=37.)
        a=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)
        b=fit_ground_entry(shifted,vertical_fov_rad=.35,up_sonar=up)
        self.assertTrue(a['accepted'])
        self.assertFalse(any(95<=i<155 for i in a['inlier_beams']))
        np.testing.assert_array_equal(a['inlier_beams'],b['inlier_beams'])
        np.testing.assert_allclose(a['points_sonar'],b['points_sonar'])

    def test_no_echo_and_censored_start_cannot_supply_ground(self):
        frame,up,_=scene(visible=False)
        self.assertFalse(fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)['accepted'])
        frame['intensity_db'][frame['ranges']>.6]+=45
        self.assertFalse(np.isfinite(ground_entries(frame)['range_m']).any())

    def test_weak_broad_leakage_is_not_a_local_first_return(self):
        frame,up,_=scene()
        expected=ground_entries(frame)['range_m']
        # A bright object in another beam produces a weak same-range band.
        # It exceeds the near noise threshold but is weaker than local ground.
        band=(frame['ranges']>1.5)&(frame['ranges']<1.9)
        frame['intensity_db'][band]+=20
        old=ground_entries(frame,config=replace(GroundEntryConfig(),beam_relative_margin_db=None))
        current=ground_entries(frame)
        self.assertLess(np.nanmedian(old['range_m']),2.)
        np.testing.assert_allclose(current['range_m'],expected,atol=.026)
        self.assertTrue(fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)['accepted'])

    def test_unknown_aperture_branch_and_narrow_support_rejected(self):
        frame,up,_=scene()
        result=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=[1.,0.,0.])
        self.assertEqual(result['reason'],'ground_aperture_branch_unresolved')
        frame['intensity_db'][:,:100]=-15;frame['intensity_db'][:,150:]=-15
        self.assertEqual(fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)['reason'],
                         'insufficient_ground_entry_support')

    def test_rotation_about_sonar_x_changes_aperture_branch_consistently(self):
        frame,up,normal=scene()
        expected=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=up)
        # 180-degree roll reverses azimuth ordering and the vertical side.
        frame['intensity_db']=frame['intensity_db'][:,::-1].copy()
        transform=np.diag([1.,-1.,-1.])
        result=fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=transform@up)
        self.assertTrue(result['accepted'])
        np.testing.assert_allclose(result['normal_sonar'],transform@expected['normal_sonar'],atol=.003)
        self.assertGreater(result['boundary_elevation_rad'],0)

    def test_invalid_geometry_and_config(self):
        frame,up,_=scene()
        for bad in ([0,0,2],[0,0,float('nan')],[0,0]):
            with self.assertRaises(ValueError):fit_ground_entry(frame,vertical_fov_rad=.35,up_sonar=bad)
        for changes in ({'range_sigma_m':0},{'minimum_consensus':2},{'ransac_trials':0},
                        {'noise_range_max_m':.1},{'maximum_slope_rad':math.pi}):
            with self.assertRaises(ValueError):ground_entries(frame,config=replace(GroundEntryConfig(),**changes))


if __name__=='__main__':unittest.main()

import unittest
import numpy as np
from posim_slam.sonar_surface import ground_entry_band, segment_surface, ping_plane_in_sonar
from posim_slam.inertial import exp_so3


def frame(*, shadow=True, clipped=False, gain=0.):
    ranges=np.linspace(.05,10,400);angles=np.linspace(-1,1,64)
    rng=np.random.default_rng(42)
    a=rng.normal(-10,1,(400,64))
    a[ranges>=2]=rng.normal(40,1,(np.count_nonzero(ranges>=2),64))
    if shadow:
        a[np.ix_((ranges>=3)&(ranges<3.3),np.arange(25,35))]+=20
        a[np.ix_((ranges>=3.3)&(ranges<(11 if clipped else 6)),np.arange(25,35))]-=30
    return dict(intensity_db=a+gain,ranges=ranges,
                beam_directions=np.column_stack([np.cos(angles),np.sin(angles),angles*0]),
                stamp=np.array([1,0]))


class SonarSurfaceTests(unittest.TestCase):
    def test_ping_plane_levers_bias_and_tilt(self):
        r=exp_so3([0,.2,0]);rs=exp_so3([0,.35,0]);lp=np.array([0,0,-.1]);ls=np.array([.2,0,0])
        rays=np.array([[0,0,-1.],[np.sin(.1),0,-np.cos(.1)]])
        # Independently intersect rays with world z=-2, then choose first return.
        distance=min((-2-(r@lp)[2])/(r@ray)[2] for ray in rays)
        normal,altitude=ping_plane_in_sonar(r,np.eye(3),lp,rs,ls,rays,distance+.03,bias_m=.03)
        np.testing.assert_allclose(normal,(r@rs).T@[0,0,1],atol=1e-12)
        self.assertAlmostEqual(altitude,2+(r@ls)[2])

    def test_ping_plane_invalid_geometry_and_no_downward_return(self):
        args=[np.eye(3),np.eye(3),np.zeros(3),np.eye(3),np.zeros(3),[[0,0,-1]],2.]
        for index,value in ((0,np.diag([1,1,-1])),(3,np.eye(3)*2),(5,[[0,0,-2]]),(6,-1.)):
            changed=list(args);changed[index]=value
            with self.assertRaisesRegex(ValueError,'invalid_ping_plane_inputs'):ping_plane_in_sonar(*changed)
        args[5]=[[0,0,1]]
        with self.assertRaisesRegex(ValueError,'ping_has_no_downward_ray'):ping_plane_in_sonar(*args)

    def test_central_flat_ground_matches_geometry(self):
        # 2 m altitude; downward 20-degree central axis and 20-degree aperture.
        normal=np.array([-np.sin(np.deg2rad(20)),0,np.cos(np.deg2rad(20))])
        result=ground_entry_band([0],np.deg2rad(20),normal,-2)
        np.testing.assert_allclose(result,[[4,4]],atol=1e-12)
        uncertain=ground_entry_band([0],np.deg2rad(20),normal,-2,
                                    distance_bound_m=.1,normal_bound_rad=np.deg2rad(5))
        np.testing.assert_allclose(uncertain,[[1.9/np.sin(np.deg2rad(35)),2.1/np.sin(np.deg2rad(25))]])

    def test_tilted_plane_extrema_match_dense_independent_ray_intersections(self):
        angles=np.linspace(-1,1,27);e=np.linspace(-.45,.45,20001)
        for normal in ([0,0,1],[-.8,.1,.5916079783],[.2,-.3,.9327379053]):
            n=np.asarray(normal);n=n/np.linalg.norm(n)
            result=ground_entry_band(angles,.9,n,-2)
            for i,a in enumerate(angles):
                rays=np.column_stack([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
                projection=rays@n;valid=projection < -1e-9
                expected=np.min(-2/projection[valid]) if valid.any() else np.inf
                np.testing.assert_allclose(result[i],[expected,expected],rtol=1e-8)
        # Minimum lies inside the aperture, not at either elevation edge.
        np.testing.assert_allclose(ground_entry_band([0],.9,[-1,0,0],-2),[[2,2]])

    def test_bounds_contain_perturbed_plane_and_normal(self):
        n=np.array([-.3,.1,np.sqrt(.9)]);az=np.linspace(-1,1,30)
        band=ground_entry_band(az,.35,n,-2,distance_bound_m=.2,normal_bound_rad=.08)
        rng=np.random.default_rng(9)
        for _ in range(30):
            axis=rng.normal(size=3);axis*=rng.uniform(0,.08)/np.linalg.norm(axis)
            changed=ground_entry_band(az,.35,exp_so3(axis)@n,-rng.uniform(1.8,2.2))[:,0]
            self.assertTrue(np.all(changed>=band[:,0]-1e-10))
            self.assertTrue(np.all(changed<=band[:,1]+1e-10))

    def test_closed_shadow_is_one_region_with_observed_end(self):
        f=frame();d=segment_surface(f,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
        self.assertEqual(len(d['shadow_candidates']),1)
        group=d['shadow_candidates'][0]
        self.assertEqual((group['beam_first'],group['beam_last']),(25,34))
        for row in group['intervals']:
            self.assertTrue(row['start_observed'])
            self.assertTrue(row['end_observed']);self.assertAlmostEqual(row['range_end_m'],6,delta=.04)
        self.assertFalse(d['map_qualified']);self.assertNotIn('points',d)

    def test_range_clipping_and_lost_background_do_not_invent_shadow_end(self):
        for lost_background in (False,True):
            f=frame(clipped=True)
            if lost_background:f['intensity_db'][f['ranges']>8]=-10
            d=segment_surface(f,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
            self.assertEqual(len(d['shadow_candidates']),1)
            for row in d['shadow_candidates'][0]['intervals']:
                self.assertFalse(row['end_observed']);self.assertIsNone(row['range_end_m'])

    def test_gain_shift_invariance_and_empty_seabed(self):
        band=np.tile([1.8,2.2],(64,1))
        a=segment_surface(frame(),ground_entry_band_m=band)
        b=segment_surface(frame(gain=70),ground_entry_band_m=band)
        self.assertEqual(a['echo_regions'],b['echo_regions'])
        self.assertEqual(a['shadow_candidates'],b['shadow_candidates'])
        self.assertEqual(segment_surface(frame(shadow=False),ground_entry_band_m=band)['shadow_candidates'],[])
        # A smooth range-dependent decay is background, not an object shadow.
        f=frame(shadow=False);f['intensity_db']-=2*f['ranges'][:,None]
        self.assertEqual(segment_surface(f,ground_entry_band_m=band)['shadow_candidates'],[])

    def test_unknown_ground_and_isolated_dark_beam_are_not_regions(self):
        f=frame();band=np.full((64,2),np.inf)
        self.assertEqual(segment_surface(f,ground_entry_band_m=band)['shadow_candidates'],[])
        f=frame(shadow=False);f['intensity_db'][100:300,30]-=30
        self.assertEqual(segment_surface(f,ground_entry_band_m=np.tile([1.8,2.2],(64,1)))['shadow_candidates'],[])

    def test_ground_mask_boundary_is_censored_not_a_physical_shadow_start(self):
        result=segment_surface(frame(),ground_entry_band_m=np.tile([4,5],(64,1)))
        self.assertEqual(len(result['shadow_candidates']),1)
        self.assertTrue(all(not row['start_observed'] for row in result['shadow_candidates'][0]['intervals']))
        self.assertTrue(np.isnan(result['ground_onset_m']).all())
        actual=segment_surface(frame(shadow=False),ground_entry_band_m=np.tile([1.8,2.2],(64,1)))
        np.testing.assert_allclose(actual['ground_onset_m'],2.,atol=.03)

    def test_invalid_geometry_and_frame(self):
        for n,d in (([0,0,2],-2),([0,0,1],0),([0,0,1],np.nan)):
            with self.assertRaises(ValueError):ground_entry_band([0],.35,n,d)
        with self.assertRaises(ValueError):ground_entry_band([0],.35,[0,0,1],-2,distance_bound_m=2)
        f=frame();f['intensity_db'][0,0]=np.nan
        with self.assertRaises(ValueError):segment_surface(f,ground_entry_band_m=np.tile([2,1],(64,1)))


if __name__=='__main__':unittest.main()

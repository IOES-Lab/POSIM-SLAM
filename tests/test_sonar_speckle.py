import unittest
import numpy as np
from posim_slam.sonar_speckle import power_moments, periodic_hamming_amplitude, surface_pixel_moments
from posim_slam.sonar_surface_power import SurfacePowerModel, TriangleSurface


class SpeckleTests(unittest.TestCase):
    def test_exact_window_matches_independent_direct_frequency_sum(self):
        for n in (8, 79, 80, 399):
            rng=np.random.default_rng(n);d=np.r_[0,1,-1,n,-n,n/2,rng.uniform(-2*n,2*n,20)]
            f=np.arange(n);w=.54-.46*np.cos(2*np.pi*(f+1)/n);w/=np.linalg.norm(w)
            expected=np.exp(2j*np.pi*d[:,None]*f/n)@w/np.sqrt(n)
            np.testing.assert_allclose(periodic_hamming_amplitude(d,n),expected,atol=2e-13,rtol=2e-11)

    def test_single_scatterer_is_one_look_even_after_averaging_twelve_pixels(self):
        h=np.exp(1j*np.arange(12))[:,None]
        moments=power_moments(h,[3.],np.ones((1,12))/12)
        np.testing.assert_allclose(moments['mean'],3.)
        np.testing.assert_allclose(moments['covariance'],9.)
        np.testing.assert_allclose(moments['equivalent_looks'],1.)

    def test_independent_equal_power_pixels_gain_twelve_looks(self):
        result=power_moments(np.eye(12),np.ones(12)*3,np.ones((1,12))/12)
        np.testing.assert_allclose(result['mean'],3.)
        np.testing.assert_allclose(result['covariance'],9/12)
        np.testing.assert_allclose(result['equivalent_looks'],12.)

    def test_covariance_matches_independent_complex_gaussian_monte_carlo(self):
        rng=np.random.default_rng(48);h=rng.normal(size=(4,3))+1j*rng.normal(size=(4,3));q=np.array([.4,1.3,.8])
        w=np.array([[.5,.5,0,0],[0,0,.5,.5]])
        result=power_moments(h,q,w)
        xi=(rng.normal(size=(100000,3))+1j*rng.normal(size=(100000,3)))*np.sqrt(q/2)
        intensity=abs(xi@h.T)**2@w.T
        np.testing.assert_allclose(intensity.mean(0),result['mean'],rtol=.015)
        np.testing.assert_allclose(np.cov(intensity.T),result['covariance'],rtol=.04)

    def test_surface_marginal_mean_matches_direct_frequency_plane_sum(self):
        az=np.linspace(-.2,.2,7);el=np.linspace(-.1,.1,5);rr=np.arange(80)*.1
        m=SurfacePowerModel(az,el,rr,maximum_range_m=7.9)
        a,b,c,d=np.array([[3.15,-2,-2],[3.15,2,-2],[3.15,2,2],[3.15,-2,2]])
        o=m.observation(TriangleSurface(np.array([[a,b,c],[a,c,d]])),np.zeros(3),np.eye(3))
        pixels=np.array([[30,2],[31,3],[32,4]])
        result=surface_pixel_moments(o,[.001,.001],pixels)
        beam=abs(np.sinc(.884/(az[1]-az[0])*np.sin(az[:,None]-az)));beam/=np.linalg.norm(beam)
        f=np.arange(80);window=.54-.46*np.cos(2*np.pi*(f+1)/80);window/=np.linalg.norm(window)
        expected=[]
        for rb,bb in pixels:
            amplitude=np.exp(2j*np.pi*(o.selected_ranges[:,None]/.1-rb)*f/80)@window/np.sqrt(80)
            expected.append(np.sum(abs(amplitude*beam[bb,o.beam_ids])**2*o.unit_power*.001))
        np.testing.assert_allclose(result['mean'],expected,rtol=1e-12)
        self.assertTrue(np.all(np.linalg.eigvalsh(result['covariance'])>=-1e-8))

    def test_invalid_response_weights_and_variances_are_rejected(self):
        for q in ([-1.],[float('nan')],[True]):
            with self.assertRaises(ValueError):power_moments([[1]],q)
        for w in ([[2.]], [[-1.]], [[float('inf')]]):
            with self.assertRaises(ValueError):power_moments([[1]],[1.],w)
        zero=power_moments([[1.]],[0.]);np.testing.assert_array_equal(zero['equivalent_looks'],0)

    def test_float32_power_is_accumulated_without_fourth_moment_overflow(self):
        result=power_moments(np.ones((1,2),dtype=np.float32),np.full(2,1e20,dtype=np.float32))
        self.assertTrue(np.isfinite(result['covariance']).all())
        np.testing.assert_allclose(result['equivalent_looks'],1.)
        with self.assertRaisesRegex(ValueError,'finite_conditional'):
            power_moments([[1e300]],[1e300])


if __name__=='__main__':unittest.main()

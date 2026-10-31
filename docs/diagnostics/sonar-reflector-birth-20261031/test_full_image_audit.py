import unittest
import numpy as np
from full_image_audit import positive_image
from posim_slam.sonar_birth import _mean_image
from posim_slam.sonar_surface_power import SurfacePowerModel,TriangleSurface
from posim_slam.sonar_source_surface import SourceSurfaceObservation


class FullImageTests(unittest.TestCase):
    def test_positive_sum_image_matches_all_exact_pixels_including_weak_tails(self):
        model=SurfacePowerModel(np.linspace(-.3,.3,7),np.linspace(-.15,.15,9),
                                np.arange(83)*.1,maximum_range_m=8.2)
        corners=np.array([[3.17,-8,-8],[3.17,8,-8],[3.17,8,8],[3.17,-8,8]])
        obs=SourceSurfaceObservation(model,TriangleSurface(corners[[[0,1,2],[0,2,3]]]))
        for mu in [[.001,.003],[1e-15,3e-15],[0.,0.]]:
            mean=positive_image(obs,mu)
            pixels=np.array([(i,j) for i in range(83) for j in range(7)])
            exact=np.concatenate([obs.moments(mu,pixels[i:i+32])['mean'] for i in range(0,len(pixels),32)])
            np.testing.assert_allclose(mean.ravel(),exact,rtol=2e-12,atol=0.)
            self.assertTrue(np.all(mean>=0))
            proposal=_mean_image(obs,mu)
            self.assertTrue(np.all(abs(mean-proposal['mean'])<=proposal['arithmetic_envelope']))


if __name__=='__main__':unittest.main()

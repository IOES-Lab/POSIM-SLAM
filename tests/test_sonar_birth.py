import unittest
import numpy as np
from posim_slam.sonar_birth import _mean_image, _average, unexplained_regions, support_cells, shared_support, choose_surface_birth
from posim_slam.sonar_source_surface import SourceSurfaceObservation
from posim_slam.sonar_surface_power import TriangleSurface, SurfacePowerModel


class BirthTests(unittest.TestCase):
    def test_full_native_mean_matches_exact_selected_fourier_for_odd_and_even_lengths(self):
        for bins in [31,32,80,83]:
            model = SurfacePowerModel(np.linspace(-.3,.3,9),np.linspace(-.15,.15,7),
                                      np.arange(bins)*.2,maximum_range_m=(bins-.1)*.2)
            vertices = np.array([[2.31,-5,-5],[2.31,5,-5],[2.31,5,5],[2.31,-5,5]])
            obs = SourceSurfaceObservation(model,TriangleSurface(vertices[[[0,1,2],[0,2,3]]]))
            image = _mean_image(obs,[.001,.003])
            pixels = np.array([(i,j) for i in range(bins) for j in [1,4,7]])
            exact = np.concatenate([obs.moments([.001,.003],pixels[i:i+32])['mean']
                                    for i in range(0,len(pixels),32)])
            np.testing.assert_array_less(abs(image['mean'][tuple(pixels.T)]-exact),
                                          image['arithmetic_envelope'][tuple(pixels.T)])
            np.testing.assert_allclose(image['mean'][tuple(pixels.T)],exact,
                                       rtol=1e-8,atol=exact.max()*1e-12)
            self.assertFalse(image['confidence_calibrated'])

    def test_empty_and_scaled_weak_returns_preserve_power_and_arithmetic_scale(self):
        model = SurfacePowerModel(np.linspace(-.2,.2,5),np.linspace(-.1,.1,5),
                                  np.arange(40)*.1,maximum_range_m=3.9)
        vertices = np.array([[2.,-5,-5],[2.,5,-5],[2.,5,5],[2.,-5,5]])
        obs = SourceSurfaceObservation(model,TriangleSurface(vertices[[[0,1,2],[0,2,3]]]))
        zero = _mean_image(obs,[0.,0.]);np.testing.assert_array_equal(zero['mean'],0.)
        normal, weak = _mean_image(obs,[1.,1.]),_mean_image(obs,[1e-12,1e-12])
        np.testing.assert_allclose(weak['mean'],normal['mean']*1e-12,
                                   atol=normal['arithmetic_envelope'].max()*1e-12,rtol=1e-10)
        np.testing.assert_allclose(weak['arithmetic_envelope'],normal['arithmetic_envelope']*1e-12)

    def test_regions_keep_original_zeros_and_do_not_count_gain_as_a_new_object(self):
        mean = np.ones((31,21));data = 3*mean
        prediction = dict(mean=mean,arithmetic_envelope=np.full_like(mean,1e-10))
        self.assertEqual(unexplained_regions(data,prediction)['regions'],[])
        data[10:15,8:12] = 150.;data[0,0]=0.
        found = unexplained_regions(data,prediction)
        self.assertTrue(any(r['selected'] for r in found['regions']))
        np.testing.assert_array_equal(found['observed'],data)
        self.assertFalse(found['map_accepted'])

    def test_local_power_average_preserves_weak_regions_after_strong_echo(self):
        image = np.full((20,20),1e-20);image[:3,:3] = 1e20
        np.testing.assert_allclose(_average(image,2)[8:,8:],1e-20,rtol=1e-15,atol=0.)

    def test_full_elevation_and_distinct_views_are_retained(self):
        cells = support_cells(np.array([[2,2]]),ranges=np.arange(5,dtype=float),
            azimuth=np.linspace(-.3,.3,5),origin=np.zeros(3),rotation=np.eye(3),vertical_fov=.4)
        self.assertGreater(np.ptp(cells[:,2]),1)
        one = shared_support([[cells,cells]])
        self.assertFalse(one['selected'].any())
        two = shared_support([[cells,cells],[cells]])
        self.assertTrue(two['selected'].all())
        np.testing.assert_array_equal(two['distinct_views'],2)
        self.assertFalse(two['physical_identity_verified'])

    def test_inverse_selects_from_same_raw_queries_without_a_native_update(self):
        from test_sonar_scene_fit import fixture
        vertices,faces,view = fixture()
        missing = faces[:2]
        full = dict(name='second_reflector',vertices=vertices,faces=faces,reflectivity=np.full(4,.001))
        result = choose_surface_birth(vertices,missing,np.full(2,.001),[view],[full])
        self.assertEqual(result['proposed_index'],1)
        self.assertLess(result['alternatives'][1]['score'],result['alternatives'][0]['score'])
        self.assertFalse(result['native_update'])


if __name__=='__main__':
    unittest.main()
